/**
 * ep0.c
 */

#include <common.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
/* #include <linux/usb/composite.h> */

#include "core.h"
#include "gadget.h"
#include "io.h"

#define IS_ALIGNED(x, a)		(((x) & ((typeof(x))(a) - 1)) == 0)

#undef dev_vdbg
#define dev_vdbg	dev_dbg

#undef dev_WARN
#define dev_WARN	dev_dbg

static void __sunxi_ep0_do_control_status(struct sunxi *sunxi, struct sunxi_ep *dep);

static void sunxi_ep0_do_control_status(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event);

static void __sunxi_ep0_do_control_data(struct sunxi *sunxi,
		struct sunxi_ep *dep, struct sunxi_request *req);

int sunxi_gadget_resize_tx_fifos(struct sunxi *sunxi);


static const char *sunxi_ep0_state_string(enum sunxi_ep0_state state)
{
	switch (state) {
	case EP0_UNCONNECTED:
		return "Unconnected";
	case EP0_SETUP_PHASE:
		return "Setup Phase";
	case EP0_DATA_PHASE:
		return "Data Phase";
	case EP0_STATUS_PHASE:
		return "Status Phase";
	default:
		return "UNKNOWN";
	}
}
u32 *trb_address;

static int sunxi_ep0_start_trans(struct sunxi *sunxi, u8 epnum, dma_addr_t buf_dma,
		u32 len, u32 type)
{
	struct sunxi_gadget_ep_cmd_params params;
	struct sunxi_trb_hw		*trb_hw;
	struct sunxi_trb			trb;
	struct sunxi_ep			*dep;

	int				ret;

	dep = sunxi->eps[epnum];
	if (dep->flags & SUNXI_EP_BUSY) {
		dev_vdbg(sunxi->dev, "%s: still busy\n", dep->name);
		return 0;
	}

	trb_hw = sunxi->ep0_trb;
	memset(&trb, 0, sizeof(trb));

	trb.trbctl = type;
	trb.bplh = buf_dma;
	trb.length = len;

	trb.hwo	= 1;
	trb.lst	= 1;
	trb.ioc	= 1;
	trb.isp_imi = 1;
	
	//Before writing to the registers 
	sunxi_trb_to_hw(&trb, trb_hw);
	
	trb_address = (u32*)sunxi->ep0_trb_addr;
	memset(&params, 0, sizeof(params));
	params.param0 = upper_32_bits(sunxi->ep0_trb_addr);
	params.param1 = lower_32_bits(sunxi->ep0_trb_addr);

	ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number,
			SUNXI_DEPCMD_STARTTRANSFER, &params);
	if (ret < 0) {
		dev_dbg(sunxi->dev, "failed to send STARTTRANSFER command\n");
		return ret;
	}

	dep->flags |= SUNXI_EP_BUSY;
	dep->resource_index = sunxi_gadget_ep_get_transfer_index(sunxi,
			dep->number);

	sunxi->ep0_next_event = SUNXI_EP0_COMPLETE;

	return 0;
}

static int __sunxi_gadget_ep0_queue(struct sunxi_ep *dep,
		struct sunxi_request *req)
{
	struct sunxi		*sunxi = dep->sunxi;

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->epnum		= dep->number;

	list_add_tail(&req->list, &dep->request_list);

	/*
	 * Gadget driver might not be quick enough to queue a request
	 * before we get a Transfer Not Ready event on this endpoint.
	 *
	 * In that case, we will set SUNXI_EP_PENDING_REQUEST. When that
	 * flag is set, it's telling us that as soon as Gadget queues the
	 * required request, we should kick the transfer here because the
	 * IRQ we were waiting for is long gone.
	 */
	if (dep->flags & SUNXI_EP_PENDING_REQUEST) {
		unsigned	direction;

		direction = !!(dep->flags & SUNXI_EP0_DIR_IN);

		if (sunxi->ep0state != EP0_DATA_PHASE) {
			//dev_WARN(sunxi->dev, "Unexpected pending request\n");
			return 0;
		}

		__sunxi_ep0_do_control_data(sunxi, sunxi->eps[direction], req);

		dep->flags &= ~(SUNXI_EP_PENDING_REQUEST |
				SUNXI_EP0_DIR_IN);

		return 0;
	}

	/*
	 * In case gadget driver asked us to delay the STATUS phase,
	 * handle it here.
	 */
	if (sunxi->delayed_status) {
		unsigned	direction;

		direction = !sunxi->ep0_expect_in;
		sunxi->delayed_status = false;

		if (sunxi->ep0state == EP0_STATUS_PHASE)
			__sunxi_ep0_do_control_status(sunxi, sunxi->eps[direction]);
		else
			dev_dbg(sunxi->dev, "too early for delayed status\n");

		return 0;
	}

	/*
	 * Unfortunately we have uncovered a limitation wrt the Data Phase.
	 *
	 * Section 9.4 says we can wait for the XferNotReady(DATA) event to
	 * come before issueing Start Transfer command, but if we do, we will
	 * miss situations where the host starts another SETUP phase instead of
	 * the DATA phase.  Such cases happen at least on TD.7.6 of the Link
	 * Layer Compliance Suite.
	 *
	 * The problem surfaces due to the fact that in case of back-to-back
	 * SETUP packets there will be no XferNotReady(DATA) generated and we
	 * will be stuck waiting for XferNotReady(DATA) forever.
	 *
	 * By looking at tables 9-13 and 9-14 of the Databook, we can see that
	 * it tells us to start Data Phase right away. It also mentions that if
	 * we receive a SETUP phase instead of the DATA phase, core will issue
	 * XferComplete for the DATA phase, before actually initiating it in
	 * the wire, with the TRB's status set to "SETUP_PENDING". Such status
	 * can only be used to print some debugging logs, as the core expects
	 * us to go through to the STATUS phase and start a CONTROL_STATUS TRB,
	 * just so it completes right away, without transferring anything and,
	 * only then, we can go back to the SETUP phase.
	 *
	 * Because of this scenario, SNPS decided to change the programming
	 * model of control transfers and support on-demand transfers only for
	 * the STATUS phase. To fix the issue we have now, we will always wait
	 * for gadget driver to queue the DATA phase's struct usb_request, then
	 * start it right away.
	 *
	 * If we're actually in a 2-stage transfer, we will wait for
	 * XferNotReady(STATUS).
	 */
	if (sunxi->three_stage_setup) {
		unsigned        direction;

		direction = sunxi->ep0_expect_in;
		sunxi->ep0state = EP0_DATA_PHASE;

		__sunxi_ep0_do_control_data(sunxi, sunxi->eps[direction], req);

		dep->flags &= ~SUNXI_EP0_DIR_IN;
	}

	return 0;
}

int sunxi_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
		gfp_t gfp_flags)
{
	struct sunxi_request		*req = to_sunxi_request(request);
	struct sunxi_ep			*dep = to_sunxi_ep(ep);
	struct sunxi			*sunxi = dep->sunxi;


	int				ret;

	if (!dep->desc) {
		dev_dbg(sunxi->dev, "trying to queue request %p to disabled %s\n",
				request, dep->name);
		ret = -ESHUTDOWN;
		goto out;
	}

	/* we share one TRB for ep0/1 */
	if (!list_empty(&dep->request_list)) {
		ret = -EBUSY;
		goto out;
	}

	dev_vdbg(sunxi->dev, "queueing request %p to %s length %d, state '%s'\n",
			request, dep->name, request->length,
			sunxi_ep0_state_string(sunxi->ep0state));

	ret = __sunxi_gadget_ep0_queue(dep, req);

out:

	return ret;
}

static void sunxi_ep0_stall_and_restart(struct sunxi *sunxi)
{
	struct sunxi_ep		*dep;

	/* reinitialize physical ep1 */
	dep = sunxi->eps[1];
	dep->flags = SUNXI_EP_ENABLED;

	/* stall is always issued on EP0 */
	dep = sunxi->eps[0];
	__sunxi_gadget_ep_set_halt(dep, 1);

	dep->flags = SUNXI_EP_ENABLED;
	sunxi->delayed_status = false;

	if (!list_empty(&dep->request_list)) {
		struct sunxi_request	*req;

		req = next_request(&dep->request_list);
		sunxi_gadget_giveback(dep, req, -ECONNRESET);
	}

	sunxi->ep0state = EP0_SETUP_PHASE;
	sunxi_ep0_out_start(sunxi);
}

int sunxi_gadget_ep0_set_halt(struct usb_ep *ep, int value)
{
	struct sunxi_ep			*dep = to_sunxi_ep(ep);
	struct sunxi			*sunxi = dep->sunxi;

	sunxi_ep0_stall_and_restart(sunxi);

	return 0;
}

void sunxi_ep0_out_start(struct sunxi *sunxi)
{
	int				ret;

	ret = sunxi_ep0_start_trans(sunxi, 0, sunxi->ctrl_req_addr, 8,
			SUNXI_TRBCTL_CONTROL_SETUP);
	WARN_ON(ret < 0);
}

static struct sunxi_ep *sunxi_wIndex_to_dep(struct sunxi *sunxi, __le16 wIndex_le)
{
	struct sunxi_ep		*dep;
	u32			windex = le16_to_cpu(wIndex_le);
	u32			epnum;

	epnum = (windex & USB_ENDPOINT_NUMBER_MASK) << 1;
	if ((windex & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
		epnum |= 1;

	dep = sunxi->eps[epnum];
	if (dep->flags & SUNXI_EP_ENABLED)
		return dep;

	return NULL;
}

static void sunxi_ep0_status_cmpl(struct usb_ep *ep, struct usb_request *req)
{
}
/*
 * ch 9.4.5
 */
static int sunxi_ep0_handle_status(struct sunxi *sunxi,
		struct usb_ctrlrequest *ctrl)
{
	struct sunxi_ep		*dep;
	u32			recip;
	u32			reg;
	u16			usb_status = 0;
	__le16			*response_pkt;

	recip = ctrl->bRequestType & USB_RECIP_MASK;
	switch (recip) {
	case USB_RECIP_DEVICE:
		/*
		 * LTM will be set once we know how to set this in HW.
		 */
		usb_status |= sunxi->is_selfpowered << USB_DEVICE_SELF_POWERED;

		if (sunxi->speed == SUNXI_DSTS_SUPERSPEED) {
			reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
			if (reg & SUNXI_DCTL_INITU1ENA)
				usb_status |= 1 << USB_DEV_STAT_U1_ENABLED;
			if (reg & SUNXI_DCTL_INITU2ENA)
				usb_status |= 1 << USB_DEV_STAT_U2_ENABLED;
		}

		break;

	case USB_RECIP_INTERFACE:
		/*
		 * Function Remote Wake Capable	D0
		 * Function Remote Wakeup	D1
		 */
		break;

	case USB_RECIP_ENDPOINT:
		dep = sunxi_wIndex_to_dep(sunxi, ctrl->wIndex);
		if (!dep)
			return -EINVAL;

		if (dep->flags & SUNXI_EP_STALL)
			usb_status = 1 << USB_ENDPOINT_HALT;
		break;
	default:
		return -EINVAL;
	};

	response_pkt = (__le16 *) sunxi->setup_buf;
	*response_pkt = cpu_to_le16(usb_status);

	dep = sunxi->eps[0];
	sunxi->ep0_usb_req.dep = dep;
	sunxi->ep0_usb_req.request.length = sizeof(*response_pkt);
	sunxi->ep0_usb_req.request.dma = sunxi->setup_buf_addr;
	sunxi->ep0_usb_req.request.complete = sunxi_ep0_status_cmpl;

	return __sunxi_gadget_ep0_queue(dep, &sunxi->ep0_usb_req);
}

static int sunxi_ep0_handle_feature(struct sunxi *sunxi,
		struct usb_ctrlrequest *ctrl, int set)
{
	struct sunxi_ep		*dep;
	u32			recip;
	u32			wValue;
	u32			wIndex;
	u32			reg;
	int			ret;

	wValue = le16_to_cpu(ctrl->wValue);
	wIndex = le16_to_cpu(ctrl->wIndex);
	recip = ctrl->bRequestType & USB_RECIP_MASK;
	switch (recip) {
	case USB_RECIP_DEVICE:

		switch (wValue) {
		case USB_DEVICE_REMOTE_WAKEUP:
			break;
 
		/*
		 * 9.4.1 says only only for SS, in AddressState only for
		 * default control pipe
		 */
		case USB_DEVICE_U1_ENABLE:
			if (sunxi->dev_state != SUNXI_CONFIGURED_STATE)
				return -EINVAL;
			if (sunxi->speed != SUNXI_DSTS_SUPERSPEED)
				return -EINVAL;
			reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
			if (set)
				reg |= SUNXI_DCTL_INITU1ENA;
			else
				reg &= ~SUNXI_DCTL_INITU1ENA;
			sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);
			break;

		case USB_DEVICE_U2_ENABLE:
			if (sunxi->dev_state != SUNXI_CONFIGURED_STATE)
				return -EINVAL;
			if (sunxi->speed != SUNXI_DSTS_SUPERSPEED)
				return -EINVAL;

			reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
			if (set)
				reg |= SUNXI_DCTL_INITU2ENA;
			else
				reg &= ~SUNXI_DCTL_INITU2ENA;
			sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);
			break;

		case USB_DEVICE_LTM_ENABLE:
			return -EINVAL;
			break;

		case USB_DEVICE_TEST_MODE:
			if ((wIndex & 0xff) != 0)
				return -EINVAL;
			if (!set)
				return -EINVAL;

			sunxi->test_mode_nr = wIndex >> 8;
			sunxi->test_mode = true;
			break;

		default:
			return -EINVAL;
		}
		break;

	case USB_RECIP_INTERFACE:
		switch (wValue) {
		case USB_INTRF_FUNC_SUSPEND:
			if (wIndex & USB_INTRF_FUNC_SUSPEND_LP)
				/* XXX enable Low power suspend */
				;
			if (wIndex & USB_INTRF_FUNC_SUSPEND_RW)
				/* XXX enable remote wakeup */
				;
			break;
		default:
			return -EINVAL;
		}
		break;

	case USB_RECIP_ENDPOINT:
		switch (wValue) {
		case USB_ENDPOINT_HALT:
			dep = sunxi_wIndex_to_dep(sunxi, wIndex);
			if (!dep)
				return -EINVAL;
			ret = __sunxi_gadget_ep_set_halt(dep, set);
			if (ret)
				return -EINVAL;
			break;
		default:
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	};

	return 0;
}

static int sunxi_ep0_set_address(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	u32 addr;
	u32 reg;

	addr = le16_to_cpu(ctrl->wValue);
	if (addr > 127) {
		dev_dbg(sunxi->dev, "invalid device address %d\n", addr);
		return -EINVAL;
	}

	if (sunxi->dev_state == SUNXI_CONFIGURED_STATE) {
		dev_dbg(sunxi->dev, "trying to set address when configured\n");
		return -EINVAL;
	}

	reg = sunxi_readl(sunxi->regs, SUNXI_DCFG);
	reg &= ~(SUNXI_DCFG_DEVADDR_MASK);
	reg |= SUNXI_DCFG_DEVADDR(addr);
	sunxi_writel(sunxi->regs, SUNXI_DCFG, reg);

	if (addr)
		sunxi->dev_state = SUNXI_ADDRESS_STATE;
	else
		sunxi->dev_state = SUNXI_DEFAULT_STATE;

	return 0;
}

static int sunxi_ep0_delegate_req(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	int ret;

	spin_unlock(&sunxi->lock);
	ret = sunxi->gadget_driver->setup(&sunxi->gadget, ctrl);
	spin_lock(&sunxi->lock);
	return ret;
}

static int sunxi_ep0_set_config(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	u32 cfg;
	int ret;
	u32 reg;

	sunxi->start_config_issued = false;
	cfg = le16_to_cpu(ctrl->wValue);

	switch (sunxi->dev_state) {
	case SUNXI_DEFAULT_STATE:
		return -EINVAL;
		break;

	case SUNXI_ADDRESS_STATE:
		ret = sunxi_ep0_delegate_req(sunxi, ctrl);
		/* if the cfg matches and the cfg is non zero */
		if (cfg && (!ret || (ret == USB_GADGET_DELAYED_STATUS))) {
			sunxi->dev_state = SUNXI_CONFIGURED_STATE;

			/*
			 * Enable transition to U1/U2 state when
			 * nothing is pending from application.
			 */
			reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
			reg |= (SUNXI_DCTL_ACCEPTU1ENA | SUNXI_DCTL_ACCEPTU2ENA);
			sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

			sunxi->resize_fifos = true;
			dev_dbg(sunxi->dev, "resize fifos flag SET\n");
		}
		break;

	case SUNXI_CONFIGURED_STATE:
		ret = sunxi_ep0_delegate_req(sunxi, ctrl);
		if (!cfg)
			sunxi->dev_state = SUNXI_ADDRESS_STATE;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static void sunxi_ep0_set_sel_cmpl(struct usb_ep *ep, struct usb_request *req)
{
	struct sunxi_ep	*dep = to_sunxi_ep(ep);
	struct sunxi	*sunxi = dep->sunxi;

	u32		param = 0;
	u32		reg;

	struct timing {
		u8	u1sel;
		u8	u1pel;
		u16	u2sel;
		u16	u2pel;
	} __packed timing;

	int		ret;

	memcpy(&timing, req->buf, sizeof(timing));

	sunxi->u1sel = timing.u1sel;
	sunxi->u1pel = timing.u1pel;
	sunxi->u2sel = le16_to_cpu(timing.u2sel);
	sunxi->u2pel = le16_to_cpu(timing.u2pel);

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	if (reg & SUNXI_DCTL_INITU2ENA)
		param = sunxi->u2pel;
	if (reg & SUNXI_DCTL_INITU1ENA)
		param = sunxi->u1pel;

	/*
	 * According to Synopsys Databook, if parameter is
	 * greater than 125, a value of zero should be
	 * programmed in the register.
	 */
	if (param > 125)
		param = 0;

	/* now that we have the time, issue DGCMD Set Sel */
	ret = sunxi_send_gadget_generic_command(sunxi,
			SUNXI_DGCMD_SET_PERIODIC_PAR, param);
	WARN_ON(ret < 0);
}

static int sunxi_ep0_set_sel(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	struct sunxi_ep	*dep;
	u16		wLength;

	if (sunxi->dev_state == SUNXI_DEFAULT_STATE)
		return -EINVAL;

	wLength = le16_to_cpu(ctrl->wLength);

	if (wLength != 6) {
		dev_err(sunxi->dev, "Set SEL should be 6 bytes, got %d\n",
				wLength);
		return -EINVAL;
	}

	/*
	 * To handle Set SEL we need to receive 6 bytes from Host. So let's
	 * queue a usb_request for 6 bytes.
	 *
	 * Remember, though, this controller can't handle non-wMaxPacketSize
	 * aligned transfers on the OUT direction, so we queue a request for
	 * wMaxPacketSize instead.
	 */
	dep = sunxi->eps[0];
	sunxi->ep0_usb_req.dep = dep;
	sunxi->ep0_usb_req.request.length = dep->endpoint.maxpacket;
	sunxi->ep0_usb_req.request.buf = sunxi->setup_buf;
	sunxi->ep0_usb_req.request.complete = sunxi_ep0_set_sel_cmpl;

	return __sunxi_gadget_ep0_queue(dep, &sunxi->ep0_usb_req);
}

static int sunxi_ep0_set_isoch_delay(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	u16		wLength;
	u16		wValue;
	u16		wIndex;

	wValue = le16_to_cpu(ctrl->wValue);
	wLength = le16_to_cpu(ctrl->wLength);
	wIndex = le16_to_cpu(ctrl->wIndex);

	if (wIndex || wLength)
		return -EINVAL;

	/*
	 * REVISIT It's unclear from Databook what to do with this
	 * value. For now, just cache it.
	 */
	sunxi->isoch_delay = wValue;

	return 0;
}

static int sunxi_ep0_std_request(struct sunxi *sunxi, struct usb_ctrlrequest *ctrl)
{
	int ret;

	switch (ctrl->bRequest) {
	case USB_REQ_GET_STATUS:
		dev_vdbg(sunxi->dev, "USB_REQ_GET_STATUS\n");
		ret = sunxi_ep0_handle_status(sunxi, ctrl);
		break;
	case USB_REQ_CLEAR_FEATURE:
		dev_vdbg(sunxi->dev, "USB_REQ_CLEAR_FEATURE\n");
		ret = sunxi_ep0_handle_feature(sunxi, ctrl, 0);
		break;
	case USB_REQ_SET_FEATURE:
		dev_vdbg(sunxi->dev, "USB_REQ_SET_FEATURE\n");
		ret = sunxi_ep0_handle_feature(sunxi, ctrl, 1);
		break;
	case USB_REQ_SET_ADDRESS:
		dev_vdbg(sunxi->dev, "USB_REQ_SET_ADDRESS\n");
		ret = sunxi_ep0_set_address(sunxi, ctrl);
		break;
	case USB_REQ_SET_CONFIGURATION:
		dev_vdbg(sunxi->dev, "USB_REQ_SET_CONFIGURATION\n");
		ret = sunxi_ep0_set_config(sunxi, ctrl);
		break;
	case USB_REQ_SET_SEL:
		dev_vdbg(sunxi->dev, "USB_REQ_SET_SEL\n");
		ret = sunxi_ep0_set_sel(sunxi, ctrl);
		break;
	case USB_REQ_SET_ISOCH_DELAY:
		dev_vdbg(sunxi->dev, "USB_REQ_SET_ISOCH_DELAY\n");
		ret = sunxi_ep0_set_isoch_delay(sunxi, ctrl);
		break;
	default:
		dev_vdbg(sunxi->dev, "Forwarding to gadget driver\n");
		ret = sunxi_ep0_delegate_req(sunxi, ctrl);
		break;
	};

	return ret;
}

static void sunxi_ep0_inspect_setup(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	struct usb_ctrlrequest *ctrl = sunxi->ctrl_req;
	int ret = -EINVAL;
	u32 len;

	if (!sunxi->gadget_driver)
		goto out;

	len = le16_to_cpu(ctrl->wLength);
	if (!len) {
		sunxi->three_stage_setup = false;
		sunxi->ep0_expect_in = false;
		sunxi->ep0_next_event = SUNXI_EP0_NRDY_STATUS;
	} else {
		sunxi->three_stage_setup = true;
		sunxi->ep0_expect_in = !!(ctrl->bRequestType & USB_DIR_IN);
		sunxi->ep0_next_event = SUNXI_EP0_NRDY_DATA;
	}

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
		ret = sunxi_ep0_std_request(sunxi, ctrl);
	else
		ret = sunxi_ep0_delegate_req(sunxi, ctrl);

	if (ret == USB_GADGET_DELAYED_STATUS)
		sunxi->delayed_status = true;

out:
	if (ret < 0)
		sunxi_ep0_stall_and_restart(sunxi);
}

static void sunxi_ep0_complete_data(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	struct sunxi_request	*r = NULL;
	struct usb_request	*ur;
	struct sunxi_trb		*trb;
	struct sunxi_ep		*ep0;
	u32			transferred;
	u32			status;
	u32			length;
	u8			epnum;

	epnum = event->endpoint_number;
	ep0 = sunxi->eps[0];

	sunxi->ep0_next_event = SUNXI_EP0_NRDY_STATUS;

	r = next_request(&ep0->request_list);
	ur = &r->request;

	trb = 	(struct sunxi_trb*)sunxi->ep0_trb;

	status = trb->trbsts;
	if (status == SUNXI_TRBSTS_SETUP_PENDING) {
		dev_dbg(sunxi->dev, "Setup Pending received\n");

		if (r)
			sunxi_gadget_giveback(ep0, r, -ECONNRESET);

		return;
	}

	length = trb->length;

	if (sunxi->ep0_bounced) {
		unsigned transfer_size = ur->length;
		unsigned maxp = ep0->endpoint.maxpacket;

		transfer_size += (maxp - (transfer_size % maxp));
		transferred = min_t(u32, ur->length,
				transfer_size - length);
		memcpy(ur->buf, sunxi->ep0_bounce, transferred);
	} else {
		transferred = ur->length - length;
	}

	ur->actual += transferred;

	if ((epnum & 1) && ur->actual < ur->length) {
		/* for some reason we did not get everything out */
		sunxi_ep0_stall_and_restart(sunxi);
	} else {
		/*
		 * handle the case where we have to send a zero packet. This
		 * seems to be case when req.length > maxpacket. Could it be?
		 */
		if (r)
			sunxi_gadget_giveback(ep0, r, 0);
	}
}


static void sunxi_ep0_complete_status(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	struct sunxi_request	*r;
	struct sunxi_ep		*dep;
	struct sunxi_trb		*trb;
	u32			status;

	dep = sunxi->eps[0];
	trb = (struct sunxi_trb *)sunxi->ep0_trb;

	if (!list_empty(&dep->request_list)) {
		r = next_request(&dep->request_list);

		sunxi_gadget_giveback(dep, r, 0);
	}

	if (sunxi->test_mode) {
		int ret;

		ret = sunxi_gadget_set_test_mode(sunxi, sunxi->test_mode_nr);
		if (ret < 0) {
			dev_dbg(sunxi->dev, "Invalid Test #%d\n",
					sunxi->test_mode_nr);
			sunxi_ep0_stall_and_restart(sunxi);
			return;
		}
	}

	status = trb->trbsts;
	if (status == SUNXI_TRBSTS_SETUP_PENDING)
		dev_dbg(sunxi->dev, "Setup Pending received\n");

	sunxi->ep0state = EP0_SETUP_PHASE;
	sunxi_ep0_out_start(sunxi);
}

static void sunxi_ep0_xfer_complete(struct sunxi *sunxi,
			const struct sunxi_event_depevt *event)
{
	struct sunxi_ep		*dep = sunxi->eps[event->endpoint_number];

	dep->flags &= ~SUNXI_EP_BUSY;
	dep->resource_index = 0;
	sunxi->setup_packet_pending = false;

	switch (sunxi->ep0state) {
	case EP0_SETUP_PHASE:
		dev_vdbg(sunxi->dev, "Inspecting Setup Bytes\n");
		sunxi_ep0_inspect_setup(sunxi, event);
		break;

	case EP0_DATA_PHASE:
		dev_vdbg(sunxi->dev, "Data Phase\n");
		sunxi_ep0_complete_data(sunxi, event);
		break;

	case EP0_STATUS_PHASE:
		dev_vdbg(sunxi->dev, "Status Phase\n");
		sunxi_ep0_complete_status(sunxi, event);
		break;
	default:
		WARN(true, "UNKNOWN ep0state %d\n", sunxi->ep0state);
	}
}

static void __sunxi_ep0_do_control_data(struct sunxi *sunxi,
		struct sunxi_ep *dep, struct sunxi_request *req)
{
	int			ret = 0;

	req->direction = !!dep->number;

	if (req->request.length == 0) {
		ret = sunxi_ep0_start_trans(sunxi, dep->number,
				sunxi->ctrl_req_addr, 0,
				SUNXI_TRBCTL_CONTROL_DATA);
	} else if (!IS_ALIGNED(req->request.length, dep->endpoint.maxpacket)
			&& (dep->number == 0)) {
		u32	transfer_size;
		u32	maxpacket;

		sunxi_map_buffer_to_dma(req);

		WARN_ON(req->request.length > SUNXI_EP0_BOUNCE_SIZE);

		maxpacket = dep->endpoint.maxpacket;
		transfer_size = roundup(req->request.length, maxpacket);
 
		sunxi->ep0_bounced = true;

		/*
		 * REVISIT in case request length is bigger than
		 * SUNXI_EP0_BOUNCE_SIZE we will need two chained
		 * TRBs to handle the transfer.
		 */
		ret = sunxi_ep0_start_trans(sunxi, dep->number,
				sunxi->ep0_bounce_addr, transfer_size,
				SUNXI_TRBCTL_CONTROL_DATA);
	} else {
		//ret = usb_gadget_map_request(&sunxi->gadget, &req->request,
		//		dep->number);

		sunxi_map_buffer_to_dma(req);

		if (ret) {
			dev_dbg(sunxi->dev, "failed to map request\n");
			return;
		}

		ret = sunxi_ep0_start_trans(sunxi, dep->number, req->request.dma,
				req->request.length, SUNXI_TRBCTL_CONTROL_DATA);
	}

	WARN_ON(ret < 0);
}
static int sunxi_ep0_start_control_status(struct sunxi_ep *dep)
{
	struct sunxi		*sunxi = dep->sunxi;
	u32			type;

	type = sunxi->three_stage_setup ? SUNXI_TRBCTL_CONTROL_STATUS3
		: SUNXI_TRBCTL_CONTROL_STATUS2;

	return sunxi_ep0_start_trans(sunxi, dep->number,
			sunxi->ctrl_req_addr, 0, type);
}

static void __sunxi_ep0_do_control_status(struct sunxi *sunxi, struct sunxi_ep *dep)
{

	if (sunxi->resize_fifos) {
		dev_dbg(sunxi->dev, "starting to resize fifos\n");
		sunxi_gadget_resize_tx_fifos(sunxi);
		sunxi->resize_fifos = 0;
	}

	WARN_ON(sunxi_ep0_start_control_status(dep));
}

static void sunxi_ep0_do_control_status(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	struct sunxi_ep		*dep = sunxi->eps[event->endpoint_number];

	__sunxi_ep0_do_control_status(sunxi, dep);
}

static void sunxi_ep0_end_control_data(struct sunxi *sunxi, struct sunxi_ep *dep)
{
	struct sunxi_gadget_ep_cmd_params params;
	u32			cmd;
	int			ret;

	if (!dep->resource_index)
		return;
	cmd = SUNXI_DEPCMD_ENDTRANSFER;
	cmd |= SUNXI_DEPCMD_CMDIOC;
	cmd |= SUNXI_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number, cmd, &params);
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;
}

static void sunxi_ep0_xfernotready(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	sunxi->setup_packet_pending = true;

	switch (event->status) {
	case DEPEVT_STATUS_CONTROL_DATA:
		dev_vdbg(sunxi->dev, "Control Data\n");

		/*
		 * We already have a DATA transfer in the controller's cache,
		 * if we receive a XferNotReady(DATA) we will ignore it, unless
		 * it's for the wrong direction.
		 *
		 * In that case, we must issue END_TRANSFER command to the Data
		 * Phase we already have started and issue SetStall on the
		 * control endpoint.
		 */
		if (sunxi->ep0_expect_in != event->endpoint_number) {
			struct sunxi_ep	*dep = sunxi->eps[sunxi->ep0_expect_in];

			dev_vdbg(sunxi->dev, "Wrong direction for Data phase\n");
			sunxi_ep0_end_control_data(sunxi, dep);
			sunxi_ep0_stall_and_restart(sunxi);
			return;
		}

		break;

	case DEPEVT_STATUS_CONTROL_STATUS:
		if (sunxi->ep0_next_event != SUNXI_EP0_NRDY_STATUS)
			return;

		dev_vdbg(sunxi->dev, "Control Status\n");

		sunxi->ep0state = EP0_STATUS_PHASE;

		if (sunxi->delayed_status) {
			WARN_ON_ONCE(event->endpoint_number != 1);
			dev_vdbg(sunxi->dev, "Mass Storage delayed status\n");
			return;
		}

		sunxi_ep0_do_control_status(sunxi, event);
	}
}

void sunxi_ep0_interrupt(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	u8			epnum = event->endpoint_number;

	dev_dbg(sunxi->dev, "%s while ep%d%s in state '%s'\n",
			sunxi_ep_event_string(event->endpoint_event),
			epnum >> 1, (epnum & 1) ? "in" : "out",
			sunxi_ep0_state_string(sunxi->ep0state));


	switch (event->endpoint_event) {
	case SUNXI_DEPEVT_XFERCOMPLETE:
		sunxi_ep0_xfer_complete(sunxi, event);
		break;

	case SUNXI_DEPEVT_XFERNOTREADY:
		sunxi_ep0_xfernotready(sunxi, event);
		break;

	case SUNXI_DEPEVT_XFERINPROGRESS:
	case SUNXI_DEPEVT_RXTXFIFOEVT:
	case SUNXI_DEPEVT_STREAMEVT:
	case SUNXI_DEPEVT_EPCMDCMPLT:
		break;
	}
}
