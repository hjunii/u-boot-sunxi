/**
 * gadget.c
 */

#include <common.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

void sunxi_map_buffer_to_dma(struct sunxi_request *req)
{
	struct sunxi			*sunxi = req->dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	if (req->request.length == 0) {
		/* req->request.dma = sunxi->setup_buf_addr; */
		return;
	}

	if (req->request.dma == DMA_ADDR_INVALID) {
		req->request.dma = dma_map_single(sunxi->dev, req->request.buf,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = true;
	}
}

void sunxi_unmap_buffer_from_dma(struct sunxi_request *req)
{
	struct sunxi			*sunxi = req->dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	if (req->request.length == 0) {
		req->request.dma = DMA_ADDR_INVALID;
		return;
	}

	if (req->mapped) {
		dma_unmap_single(sunxi->dev, req->request.dma,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = 0;
		req->request.dma = DMA_ADDR_INVALID;
	}
}

/**
 * sunxi_gadget_set_test_mode - Enables USB2 Test Modes
 * @sunxi: pointer to our context structure
 * @mode: the mode to set (J, K SE0 NAK, Force Enable)
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -EINVAL if wrong Test Selector
 * is passed
 */
int sunxi_gadget_set_test_mode(struct sunxi *sunxi, int mode)
{
	u32		reg;

	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	reg &= ~SUNXI_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		reg |= mode << 1;
		break;
	default:
		return -EINVAL;
	}

	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	return 0;
}

/**
 * sunxi_gadget_set_link_state - Sets USB Link to a particular State
 * @sunxi: pointer to our context structure
 * @state: the state to put link into
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -ETIMEDOUT.
 */
int sunxi_gadget_set_link_state(struct sunxi *sunxi, enum sunxi_link_state state)
{
	int		retries = 10000;
	u32		reg;

	printf ("%s\n", __FUNCTION__);

	/*
	 * Wait until device controller is ready. Only applies to 1.94a and
	 * later RTL.
	 */
	if (sunxi->revision >= SUNXI_REVISION_194A) {
		while (--retries) {
			reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);
			if (reg & SUNXI_DSTS_DCNRD)
				udelay(5);
			else
				break;
		}

		if (retries <= 0)
			return -ETIMEDOUT;
	}

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	reg &= ~SUNXI_DCTL_ULSTCHNGREQ_MASK;

	/* set requested state */
	reg |= SUNXI_DCTL_ULSTCHNGREQ(state);
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	/*
	 * The following code is racy when called from sunxi_gadget_wakeup,
	 * and is not needed, at least on newer versions
	 */
	if (sunxi->revision >= SUNXI_REVISION_194A)
		return 0;

	/* wait for a change in DSTS */
	retries = 10000;
	while (--retries) {
		reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);

		if (SUNXI_DSTS_USBLNKST(reg) == state)
			return 0;

		udelay(5);
	}

	dev_vdbg(sunxi->dev, "link state change request timed out\n");

	return -ETIMEDOUT;
}

/**
 * sunxi_gadget_resize_tx_fifos - reallocate fifo spaces for current use-case
 * @sunxi: pointer to our context structure
 *
 * This function will a best effort FIFO allocation in order
 * to improve FIFO usage and throughput, while still allowing
 * us to enable as many endpoints as possible.
 *
 * Keep in mind that this operation will be highly dependent
 * on the configured size for RAM1 - which contains TxFifo -,
 * the amount of endpoints enabled on coreConsultant tool, and
 * the width of the Master Bus.
 *
 * In the ideal world, we would always be able to satisfy the
 * following equation:
 *
 * ((512 + 2 * MDWIDTH-Bytes) + (Number of IN Endpoints - 1) * \
 * (3 * (1024 + MDWIDTH-Bytes) + MDWIDTH-Bytes)) / MDWIDTH-Bytes
 *
 * Unfortunately, due to many variables that's not always the case.
 */

#define SUNXI_MDWIDTH(n)		(((n) & 0xff00) >> 8)
#define SUNXI_RAM1_DEPTH(n)	((n) & 0xffff)
	
int sunxi_gadget_resize_tx_fifos(struct sunxi *sunxi)
{
	int		last_fifo_depth = 0;
	int		fifo_size;
	int		mdwidth;
	int		num;

	printf ("%s\n", __FUNCTION__);

	//if (!sunxi->needs_fifo_resize)
	//	return 0;

	mdwidth = SUNXI_MDWIDTH(sunxi->hwparams.hwparams0);

	/* MDWIDTH is represented in bits, we need it in bytes */
	mdwidth >>= 3;

	/*
	 * FIXME For now we will only allocate 1 wMaxPacketSize space
	 * for each enabled endpoint, later patches will come to
	 * improve this algorithm so that we better use the internal
	 * FIFO space
	 */
	for (num = 0; num < SUNXI_ENDPOINTS_NUM; num++) {
		struct sunxi_ep	*dep = sunxi->eps[num];
		int		fifo_number = dep->number >> 1;
		int		mult = 1;
		int		tmp;

		if (!(dep->number & 1))
			continue;

		if (!(dep->flags & SUNXI_EP_ENABLED))
			continue;

		if (usb_endpoint_xfer_bulk(dep->desc)
				|| usb_endpoint_xfer_isoc(dep->desc))
			mult = 3;

		/*
		 * REVISIT: the following assumes we will always have enough
		 * space available on the FIFO RAM for all possible use cases.
		 * Make sure that's true somehow and change FIFO allocation
		 * accordingly.
		 *
		 * If we have Bulk or Isochronous endpoints, we want
		 * them to be able to be very, very fast. So we're giving
		 * those endpoints a fifo_size which is enough for 3 full
		 * packets
		 */
		tmp = mult * (dep->endpoint.maxpacket + mdwidth);
		tmp += mdwidth;

		fifo_size = DIV_ROUND_UP(tmp, mdwidth);

		fifo_size |= (last_fifo_depth << 16);

		dev_vdbg(sunxi->dev, "%s: Fifo Addr %04x Size %d\n",
				dep->name, last_fifo_depth, fifo_size & 0xffff);

		sunxi_writel(sunxi->regs, SUNXI_GTXFIFOSIZ(fifo_number),
				fifo_size);

		last_fifo_depth += (fifo_size & 0xffff);
	}

	return 0;
}

void sunxi_gadget_giveback(struct sunxi_ep *dep, struct sunxi_request *req,
		int status)
{
	struct sunxi			*sunxi = dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	if (req->queued) {
		dep->busy_slot++;
		/*
		 * Skip LINK TRB. We can't use req->trb and check for
		 * SUNXI_TRBCTL_LINK_TRB because it points the TRB we just
		 * completed (not the LINK TRB).
		 */
		if (((dep->busy_slot & SUNXI_TRB_MASK) == SUNXI_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->desc))
			dep->busy_slot++;
	}
	list_del(&req->list);

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	sunxi_unmap_buffer_from_dma(req);

	dev_dbg(sunxi->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);

	spin_unlock(&sunxi->lock);
	req->request.complete(&req->dep->endpoint, &req->request);
	spin_lock(&sunxi->lock);
}


int sunxi_send_gadget_generic_command(struct sunxi *sunxi, int cmd, u32 param)
{
	u32		timeout = 500;
	u32		reg;

	printf ("%s\n", __FUNCTION__);

	sunxi_writel(sunxi->regs, SUNXI_DGCMDPAR, param);
	sunxi_writel(sunxi->regs, SUNXI_DGCMD, cmd | SUNXI_DGCMD_CMDACT);

	do {
		reg = sunxi_readl(sunxi->regs, SUNXI_DGCMD);
		if (!(reg & SUNXI_DGCMD_CMDACT)) {
			dev_vdbg(sunxi->dev, "Command Complete --> %d\n",
					SUNXI_DGCMD_STATUS(reg));
			return 0;
		}

		/*
		 * We can't sleep here, because it's also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;
		udelay(1);
	} while (1);
}

int sunxi_send_gadget_ep_cmd(struct sunxi *sunxi, unsigned ep,
		unsigned cmd, struct sunxi_gadget_ep_cmd_params *params)
{
	u32			timeout = 500;
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	sunxi_writel(sunxi->regs, SUNXI_DEPCMDPAR0(ep), params->param0);
	sunxi_writel(sunxi->regs, SUNXI_DEPCMDPAR1(ep), params->param1);
	sunxi_writel(sunxi->regs, SUNXI_DEPCMDPAR2(ep), params->param2);

	sunxi_writel(sunxi->regs, SUNXI_DEPCMD(ep), cmd | SUNXI_DEPCMD_CMDACT);

	do {
		reg = sunxi_readl(sunxi->regs, SUNXI_DEPCMD(ep));
		if (!(reg & SUNXI_DEPCMD_CMDACT)) {
			dev_dbg(sunxi->dev, "Command Complete --> %d\n",
					SUNXI_DEPCMD_STATUS(reg));
			return 0;
		}

		/*
		 * We can't sleep here, because it is also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout) {
			return -ETIMEDOUT;
		}

		udelay(1);
	} while (1);
}

static dma_addr_t sunxi_trb_dma_offset(struct sunxi_ep *dep,
		struct sunxi_trb_hw *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	printf ("%s\n", __FUNCTION__);

	return dep->trb_pool_dma + offset;
}

static int sunxi_alloc_trb_pool(struct sunxi_ep *dep)
{
	struct sunxi		*sunxi = dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	if (dep->trb_pool)
		return 0;

	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = dma_alloc_coherent(sunxi->dev,
			sizeof(struct sunxi_trb) * SUNXI_TRB_NUM + 0x10,
			&dep->trb_pool_dma, GFP_KERNEL);

	dep->trb_pool = (struct sunxi_trb_hw *)((0x10 - ((u32)dep->trb_pool & 0xf)) + (long)dep->trb_pool);
	dep->trb_pool_dma = (dma_addr_t)dep->trb_pool;

	if (!dep->trb_pool) {
		dev_err(dep->sunxi->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	return 0;
}

static void sunxi_free_trb_pool(struct sunxi_ep *dep)
{
	struct sunxi		*sunxi = dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	dma_free_coherent(sunxi->dev, sizeof(struct sunxi_trb) * SUNXI_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int sunxi_gadget_start_config(struct sunxi *sunxi, struct sunxi_ep *dep)
{
	struct sunxi_gadget_ep_cmd_params params;
	u32			cmd;

	printf ("%s\n", __FUNCTION__);

	memset(&params, 0x00, sizeof(params));
	if (dep->number != 1) {
		cmd = SUNXI_DEPCMD_DEPSTARTCFG;
		/* XferRscIdx == 0 for ep0 and 2 for the remaining */
		if (dep->number > 1) {
			if (sunxi->start_config_issued)
				return 0;
			sunxi->start_config_issued = true;
			cmd |= SUNXI_DEPCMD_PARAM(2);
		}

		return sunxi_send_gadget_ep_cmd(sunxi, 0, cmd, &params);
	}

	return 0;
}

static int sunxi_gadget_set_ep_config(struct sunxi *sunxi, struct sunxi_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct sunxi_gadget_ep_cmd_params params;

	printf ("%s\n", __FUNCTION__);

	memset(&params, 0x00, sizeof(params));

	params.param0 = SUNXI_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| SUNXI_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	if (sunxi->gadget.speed == USB_SPEED_SUPER) {
		params.param0 |= SUNXI_DEPCFG_BURST_SIZE(dep->endpoint.maxburst-1);
	}

	if (ignore)
		params.param0 |= SUNXI_DEPCFG_IGN_SEQ_NUM;

	params.param1 = SUNXI_DEPCFG_XFER_COMPLETE_EN
		| SUNXI_DEPCFG_XFER_NOT_READY_EN;

#if 0
	if (comp_desc && USB_SS_MAX_STREAMS(comp_desc->bmAttributes)
			&& usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= SUNXI_DEPCFG_STREAM_CAPABLE
			| SUNXI_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}
#endif

	if (usb_endpoint_xfer_isoc(desc))
		params.param1 |= SUNXI_DEPCFG_XFER_IN_PROGRESS_EN;

	/*
	 * We are doing 1:1 mapping for endpoints, meaning
	 * Physical Endpoints 2 maps to Logical Endpoint 2 and
	 * so on. We consider the direction bit as part of the physical
	 * endpoint number. So USB endpoint 0x81 is 0x03.
	 */
	params.param1 |= SUNXI_DEPCFG_EP_NUMBER(dep->number);

	/*
	 * We must use the lower 16 TX FIFOs even though
	 * HW might have more
	 */
	if (dep->direction)
		params.param0 |= SUNXI_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		params.param1 |= SUNXI_DEPCFG_BINTERVAL_M1(desc->bInterval - 1);
		dep->interval = 1 << (desc->bInterval - 1);
	}
	return sunxi_send_gadget_ep_cmd(sunxi, dep->number,
			SUNXI_DEPCMD_SETEPCONFIG, &params);
}

static int sunxi_gadget_set_xfer_resource(struct sunxi *sunxi, struct sunxi_ep *dep)
{
	struct sunxi_gadget_ep_cmd_params params;

	printf ("%s\n", __FUNCTION__);

	memset(&params, 0x00, sizeof(params));

	params.param0 = SUNXI_DEPXFERCFG_NUM_XFER_RES(1);

	return sunxi_send_gadget_ep_cmd(sunxi, dep->number,
			SUNXI_DEPCMD_SETTRANSFRESOURCE, &params);
}

/**
 * __sunxi_gadget_ep_enable - Initializes a HW endpoint
 * @dep: endpoint to be initialized
 * @desc: USB Endpoint Descriptor
 *
 * Caller should take care of locking
 */
static int __sunxi_gadget_ep_enable(struct sunxi_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct sunxi		*sunxi = dep->sunxi;
	u32			reg;
	int			ret = -ENOMEM;

	printf ("%s\n", __FUNCTION__);

	if (!(dep->flags & SUNXI_EP_ENABLED)) {
		ret = sunxi_gadget_start_config(sunxi, dep);
		if (ret)
			return ret;
	}

	ret = sunxi_gadget_set_ep_config(sunxi, dep, desc, comp_desc, ignore);
	if (ret)
		return ret;

	if (!(dep->flags & SUNXI_EP_ENABLED)) {
		struct sunxi_trb_hw *trb_st_hw;
		struct sunxi_trb_hw *trb_link_hw;
		struct sunxi_trb	trb_link;

		ret = sunxi_gadget_set_xfer_resource(sunxi, dep);
		if (ret)
			return ret;

		dep->desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= SUNXI_EP_ENABLED;

		reg = sunxi_readl(sunxi->regs, SUNXI_DALEPENA);
		reg |= SUNXI_DALEPENA_EP(dep->number);
		sunxi_writel(sunxi->regs, SUNXI_DALEPENA, reg);

		if (!usb_endpoint_xfer_isoc(desc))
			return 0;

		memset(&trb_link, 0, sizeof(trb_link));
		/* Link TRB for ISOC. The HWO bit is never reset */
		trb_st_hw = &dep->trb_pool[0];
		trb_link.bplh = sunxi_trb_dma_offset(dep, trb_st_hw);
                trb_link.trbctl = SUNXI_TRBCTL_LINK_TRB;
                trb_link.hwo = true;

                trb_link_hw = &dep->trb_pool[SUNXI_TRB_NUM - 1];
                sunxi_trb_to_hw(&trb_link, trb_link_hw);
	}

	return 0;
}

static void sunxi_stop_active_transfer(struct sunxi *sunxi, u32 epnum);
static void sunxi_remove_requests(struct sunxi *sunxi, struct sunxi_ep *dep)
{
	struct sunxi_request		*req;

	printf ("%s\n", __FUNCTION__);

	if (!list_empty(&dep->req_queued))
		sunxi_stop_active_transfer(sunxi, dep->number);

	while (!list_empty(&dep->request_list)) {
		req = next_request(&dep->request_list);

		sunxi_gadget_giveback(dep, req, -ESHUTDOWN);
	}
}

/**
 * __sunxi_gadget_ep_disable - Disables a HW endpoint
 * @dep: the endpoint to disable
 *
 * This function also removes requests which are currently processed ny the
 * hardware and those which are not yet scheduled.
 * Caller should take care of locking.
 */
static int __sunxi_gadget_ep_disable(struct sunxi_ep *dep)
{
	struct sunxi		*sunxi = dep->sunxi;
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	sunxi_remove_requests(sunxi, dep);

	reg = sunxi_readl(sunxi->regs, SUNXI_DALEPENA);
	reg &= ~SUNXI_DALEPENA_EP(dep->number);
	sunxi_writel(sunxi->regs, SUNXI_DALEPENA, reg);

	dep->stream_capable = false;
	dep->desc = NULL;
	dep->comp_desc = NULL;
	dep->type = 0;
	dep->flags = 0;

	return 0;
}

/* -------------------------------------------------------------------------- */

static int sunxi_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	printf ("%s\n", __FUNCTION__);
	return -EINVAL;
}

static int sunxi_gadget_ep0_disable(struct usb_ep *ep)
{
	printf ("%s\n", __FUNCTION__);
	return -EINVAL;
}

/* -------------------------------------------------------------------------- */

static int sunxi_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct sunxi_ep			*dep;
	struct sunxi			*sunxi;
	int				ret;

	printf ("%s\n", __FUNCTION__);

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("sunxi: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("sunxi: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_sunxi_ep(ep);
	sunxi = dep->sunxi;

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		strncat(dep->name, "-control", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_ISOC:
		strncat(dep->name, "-isoc", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_BULK:
		strncat(dep->name, "-bulk", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_INT:
		strncat(dep->name, "-int", sizeof(dep->name));
		break;
	default:
		dev_err(sunxi->dev, "invalid endpoint transfer type\n");
	}

	if (dep->flags & SUNXI_EP_ENABLED) {
		dev_WARN_ONCE(true, "%s is already enabled\n",
				dep->name);
		return 0;
	}

	dev_vdbg(sunxi->dev, "Enabling %s\n", dep->name);

	ret = __sunxi_gadget_ep_enable(dep, desc, NULL /*ep->comp_desc*/,false);

	return ret;
}

static int sunxi_gadget_ep_disable(struct usb_ep *ep)
{
	struct sunxi_ep			*dep;
	int				ret;

	printf ("%s\n", __FUNCTION__);

	if (!ep) {
		pr_debug("sunxi: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_sunxi_ep(ep);

	if (!(dep->flags & SUNXI_EP_ENABLED)) {
		dev_WARN_ONCE(true, "%s is already disabled\n",
				dep->name);
		return 0;
	}

	snprintf(dep->name, sizeof(dep->name), "ep%d%s",
			dep->number >> 1,
			(dep->number & 1) ? "in" : "out");

	ret = __sunxi_gadget_ep_disable(dep);

	return ret;
}

static struct usb_request *sunxi_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct sunxi_request		*req;
	struct sunxi_ep			*dep = to_sunxi_ep(ep);

	printf ("%s\n", __FUNCTION__);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(sunxi->dev, "not enough memory\n");
		return NULL;
	}

	req->epnum	= dep->number;
	req->dep	= dep;
	req->request.dma = DMA_ADDR_INVALID;

	return &req->request;
}

static void sunxi_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct sunxi_request		*req = to_sunxi_request(request);

	printf ("%s\n", __FUNCTION__);

	kfree(req);
}

/*
 * sunxi_prepare_trbs - setup TRBs from requests
 * @dep: endpoint for which requests are being prepared
 * @starting: true if the endpoint is idle and no requests are queued.
 *
 * The functions goes through the requests list and setups TRBs for the
 * transfers. The functions returns once there are not more TRBs available or
 * it run out of requests.
 */
static struct sunxi_request *sunxi_prepare_trbs(struct sunxi_ep *dep,
		bool starting)
{
	struct sunxi_request	*req, *n, *ret = NULL;
	struct sunxi_trb_hw		*trb_hw;
	struct sunxi_trb		trb;
	u32			trbs_left;

	printf ("%s\n", __FUNCTION__);

	BUILD_BUG_ON_NOT_POWER_OF_2(SUNXI_TRB_NUM);

	/* the first request must not be queued */
	trbs_left = (dep->busy_slot - dep->free_slot) & SUNXI_TRB_MASK;
	/*
	 * if busy & slot are equal than it is either full or empty. If we are
	 * starting to proceed requests then we are empty. Otherwise we ar
	 * full and don't do anything
	 */
	if (!trbs_left) {
		if (!starting)
			return NULL;
		trbs_left = SUNXI_TRB_NUM;
		/*
		 * In case we start from scratch, we queue the ISOC requests
		 * starting from slot 1. This is done because we use ring
		 * buffer and have no LST bit to stop us. Instead, we place
		 * IOC bit TRB_NUM/4. We try to avoid to having an interrupt
		 * after the first request so we start at slot 1 and have
		 * 7 requests proceed before we hit the first IOC.
		 * Other transfer types don't use the ring buffer and are
		 * processed from the first TRB until the last one. Since we
		 * don't wrap around we have to start at the beginning.
		 */
		if (usb_endpoint_xfer_isoc(dep->desc)) {
			dep->busy_slot = 1;
			dep->free_slot = 1;
		} else {
			dep->busy_slot = 0;
			dep->free_slot = 0;
		}
	}

	/* The last TRB is a link TRB, not used for xfer */
	if ((trbs_left <= 1) && usb_endpoint_xfer_isoc(dep->desc))
		return NULL;

	list_for_each_entry_safe(req, n, &dep->request_list, list) {
		unsigned int last_one = 0;
		unsigned int cur_slot;

		trb_hw = &dep->trb_pool[dep->free_slot & SUNXI_TRB_MASK];
		cur_slot = dep->free_slot;
		dep->free_slot++;

		/* Skip the LINK-TRB on ISOC */
		if (((cur_slot & SUNXI_TRB_MASK) == SUNXI_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->desc))
			continue;

		sunxi_gadget_move_request_queued(req);
		memset(&trb, 0, sizeof(trb));
		trbs_left--;

		/* Is our TRB pool empty? */
		if (!trbs_left)
			last_one = 1;
		/* Is this the last request? */
		if (list_empty(&dep->request_list))
			last_one = 1;

		/*
		 * FIXME we shouldn't need to set LST bit always but we are
		 * facing some weird problem with the Hardware where it doesn't
		 * complete even though it has been previously started.
		 *
		 * While we're debugging the problem, as a workaround to
		 * multiple TRBs handling, use only one TRB at a time.
		 */
		last_one = 1;

		req->trb = trb_hw;
		if (!ret)
			ret = req;

		trb.bplh = req->request.dma;

		if (usb_endpoint_xfer_isoc(dep->desc)) {
			trb.isp_imi = true;
			trb.csp = true;
		} else {
			trb.lst = last_one;
		}

		if (usb_endpoint_xfer_bulk(dep->desc) && dep->stream_capable)
			trb.sid_sofn = req->request.stream_id;

		switch (usb_endpoint_type(dep->desc)) {
		case USB_ENDPOINT_XFER_CONTROL:
			trb.trbctl = SUNXI_TRBCTL_CONTROL_SETUP;
			break;

		case USB_ENDPOINT_XFER_ISOC:
			trb.trbctl = SUNXI_TRBCTL_ISOCHRONOUS_FIRST;

			/* IOC every SUNXI_TRB_NUM / 4 so we can refill */
			if (!(cur_slot % (SUNXI_TRB_NUM / 4)))
				trb.ioc = last_one;
			break;

		case USB_ENDPOINT_XFER_BULK:
		case USB_ENDPOINT_XFER_INT:
			trb.trbctl = SUNXI_TRBCTL_NORMAL;
			break;
		default:
			/*
			 * This is only possible with faulty memory because we
			 * checked it already :)
			 */
			BUG();
		}

		trb.length	= req->request.length;
		trb.hwo = true;

		sunxi_trb_to_hw(&trb, trb_hw);
		req->trb_dma = sunxi_trb_dma_offset(dep, trb_hw);

		if (last_one)
			break;
	}

	return ret;
}

static int __sunxi_gadget_kick_transfer(struct sunxi_ep *dep, u16 cmd_param,
		int start_new)
{
	struct sunxi_gadget_ep_cmd_params params;
	struct sunxi_request		*req;
	struct sunxi			*sunxi = dep->sunxi;
	int				ret;
	u32				cmd;

	printf ("%s\n", __FUNCTION__);

	if (start_new && (dep->flags & SUNXI_EP_BUSY)) {
		dev_vdbg(sunxi->dev, "%s: endpoint busy\n", dep->name);
		return -EBUSY;
	}
	dep->flags &= ~SUNXI_EP_PENDING_REQUEST;

	/*
	 * If we are getting here after a short-out-packet we don't enqueue any
	 * new requests as we try to set the IOC bit only on the last request.
	 */
	if (start_new) {
		if (list_empty(&dep->req_queued))
			sunxi_prepare_trbs(dep, start_new);

		/* req points to the first request which will be sent */
		req = next_request(&dep->req_queued);
	} else {
		/*
		 * req points to the first request where HWO changed
		 * from 0 to 1
		 */
		req = sunxi_prepare_trbs(dep, start_new);
	}
	if (!req) {
		dep->flags |= SUNXI_EP_PENDING_REQUEST;
		return 0;
	}

	memset(&params, 0, sizeof(params));
	params.param0 = upper_32_bits(req->trb_dma);
	params.param1 = lower_32_bits(req->trb_dma);

	if (start_new)
		cmd = SUNXI_DEPCMD_STARTTRANSFER;
	else
		cmd = SUNXI_DEPCMD_UPDATETRANSFER;

	cmd |= SUNXI_DEPCMD_PARAM(cmd_param);
	ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number, cmd, &params);
	if (ret < 0) {
		dev_dbg(sunxi->dev, "failed to send STARTTRANSFER command\n");

		/*
		 * FIXME we need to iterate over the list of requests
		 * here and stop, unmap, free and del each of the linked
		 * requests instead of we do now.
		 */
		sunxi_unmap_buffer_from_dma(req);
		list_del(&req->list);
		return ret;
	}

	dep->flags |= SUNXI_EP_BUSY;
	dep->resource_index = sunxi_gadget_ep_get_transfer_index(sunxi,
			dep->number);
	return 0;
}

static int __sunxi_gadget_ep_queue(struct sunxi_ep *dep, struct sunxi_request *req)
{
	printf ("%s\n", __FUNCTION__);

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	/*
	 * We only add to our list of requests now and
	 * start consuming the list once we get XferNotReady
	 * IRQ.
	 *
	 * That way, we avoid doing anything that we don't need
	 * to do now and defer it until the point we receive a
	 * particular token from the Host side.
	 *
	 * This will also avoid Host cancelling URBs due to too
	 * many NACKs.
	 */
	sunxi_map_buffer_to_dma(req);
	list_add_tail(&req->list, &dep->request_list);

	/*
	 * There is one special case: XferNotReady with
	 * empty list of requests. We need to kick the
	 * transfer here in that situation, otherwise
	 * we will be NAKing forever.
	 *
	 * If we get XferNotReady before gadget driver
	 * has a chance to queue a request, we will ACK
	 * the IRQ but won't be able to receive the data
	 * until the next request is queued. The following
	 * code is handling exactly that.
	 */
	if (dep->flags & SUNXI_EP_PENDING_REQUEST) {
		int ret;
		int start_trans;

		start_trans = 1;
		if (usb_endpoint_xfer_isoc(dep->desc) &&
				dep->flags & SUNXI_EP_BUSY)
			start_trans = 0;

		ret =  __sunxi_gadget_kick_transfer(dep, 0, start_trans);
		if (ret && ret != -EBUSY) {
			struct sunxi	*sunxi = dep->sunxi;

			dev_dbg(sunxi->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
	};

	return 0;
}

static int sunxi_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct sunxi_request		*req = to_sunxi_request(request);
	struct sunxi_ep			*dep = to_sunxi_ep(ep);
	struct sunxi			*sunxi = dep->sunxi;

	printf ("%s\n", __FUNCTION__);

	int				ret;

	if (!dep->desc) {
		dev_dbg(sunxi->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(sunxi->dev, "queing request %p to %s length %d\n",
			request, ep->name, request->length);

	ret = __sunxi_gadget_ep_queue(dep, req);

	return ret;
}

static int sunxi_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct sunxi_request		*req = to_sunxi_request(request);
	struct sunxi_request		*r = NULL;

	struct sunxi_ep			*dep = to_sunxi_ep(ep);
	struct sunxi			*sunxi = dep->sunxi;

	int				ret = 0;

	printf ("%s\n", __FUNCTION__);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		list_for_each_entry(r, &dep->req_queued, list) {
			if (r == req)
				break;
		}
		if (r == req) {
			/* wait until it is processed */
			sunxi_stop_active_transfer(sunxi, dep->number);
			goto out0;
		}
		dev_err(sunxi->dev, "request %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}

	/* giveback the request */
	sunxi_gadget_giveback(dep, req, -ECONNRESET);

out0:

	return ret;
}

int __sunxi_gadget_ep_set_halt(struct sunxi_ep *dep, int value)
{
	struct sunxi_gadget_ep_cmd_params	params;
	struct sunxi				*sunxi = dep->sunxi;
	int					ret;

	printf ("%s\n", __FUNCTION__);

	memset(&params, 0x00, sizeof(params));

	if (value) {
		if (dep->number == 0 || dep->number == 1) {
			/*
			 * Whenever EP0 is stalled, we will restart
			 * the state machine, thus moving back to
			 * Setup Phase
			 */
			sunxi->ep0state = EP0_SETUP_PHASE;
		}

		ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number,
			SUNXI_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(sunxi->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= SUNXI_EP_STALL;
	} else {
		if (dep->flags & SUNXI_EP_WEDGE)
			return 0;

		ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number,
			SUNXI_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(sunxi->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~SUNXI_EP_STALL;
	}

	return ret;
}

static int sunxi_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct sunxi_ep			*dep = to_sunxi_ep(ep);


	int				ret;

	printf ("%s\n", __FUNCTION__);


	if (usb_endpoint_xfer_isoc(dep->desc)) {
		dev_err(sunxi->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	ret = __sunxi_gadget_ep_set_halt(dep, value);
out:

	return ret;
}

/* -------------------------------------------------------------------------- */

static struct usb_endpoint_descriptor sunxi_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops sunxi_gadget_ep0_ops = {
	.enable		= sunxi_gadget_ep0_enable,
	.disable	= sunxi_gadget_ep0_disable,
	.alloc_request	= sunxi_gadget_ep_alloc_request,
	.free_request	= sunxi_gadget_ep_free_request,
	.queue		= sunxi_gadget_ep0_queue,
	.dequeue	= sunxi_gadget_ep_dequeue,
	.set_halt	= sunxi_gadget_ep_set_halt,
};

static const struct usb_ep_ops sunxi_gadget_ep_ops = {
	.enable		= sunxi_gadget_ep_enable,
	.disable	= sunxi_gadget_ep_disable,
	.alloc_request	= sunxi_gadget_ep_alloc_request,
	.free_request	= sunxi_gadget_ep_free_request,
	.queue		= sunxi_gadget_ep_queue,
	.dequeue	= sunxi_gadget_ep_dequeue,
	.set_halt	= sunxi_gadget_ep_set_halt,
};

/* -------------------------------------------------------------------------- */

static int sunxi_gadget_get_frame(struct usb_gadget *g)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);
	return SUNXI_DSTS_SOFFN(reg);
}

static int sunxi_gadget_wakeup(struct usb_gadget *g)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);

	unsigned long		timeout;

	u32			reg;

	int			ret = 0;

	u8			link_state;
	u8			speed;

	printf ("%s\n", __FUNCTION__);

	/*
	 * According to the Databook Remote wakeup request should
	 * be issued only when the device is in early suspend state.
	 *
	 * We can check that via USB Link State bits in DSTS register.
	 */
	reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);

	speed = reg & SUNXI_DSTS_CONNECTSPD;
	if (speed == SUNXI_DSTS_SUPERSPEED) {
		dev_dbg(sunxi->dev, "no wakeup on SuperSpeed\n");
		ret = -EINVAL;
		goto out;
	}

	link_state = SUNXI_DSTS_USBLNKST(reg);

	switch (link_state) {
	case SUNXI_LINK_STATE_RX_DET:	/* in HS, means Early Suspend */
	case SUNXI_LINK_STATE_U3:	/* in HS, means SUSPEND */
		break;
	default:
		dev_dbg(sunxi->dev, "can't wakeup from link state %d\n",
				link_state);
		ret = -EINVAL;
		goto out;
	}

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);

	/*
	 * Switch link state to Recovery. In HS/FS/LS this means
	 * RemoteWakeup Request
	 */
	reg |= SUNXI_DCTL_ULSTCHNG_RECOVERY;
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	/* wait for at least 2000us */
	usleep_range(2000, 2500);

	/* write zeroes to Link Change Request */
	reg &= ~SUNXI_DCTL_ULSTCHNGREQ_MASK;
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	/* pool until Link State change to ON */
	timeout = 100;

	while (timeout) {
		reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);

		/* in HS, means ON */
		if (SUNXI_DSTS_USBLNKST(reg) == SUNXI_LINK_STATE_U0)
			break;
		timeout--;
		mdelay(1);
	}

	if (SUNXI_DSTS_USBLNKST(reg) != SUNXI_LINK_STATE_U0) {
		dev_err(sunxi->dev, "failed to send remote wakeup\n");
		ret = -EINVAL;
	}

out:

	return ret;
}

static int sunxi_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);

	printf ("%s\n", __FUNCTION__);

	sunxi->is_selfpowered = !!is_selfpowered;

	return 0;
}

void usb3_phy_power(bool on);

static void sunxi_gadget_run_stop(struct sunxi *sunxi, int is_on)
{
	u32			reg;
	u32			timeout = 500;

	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	if (is_on) {
		reg &= ~SUNXI_DCTL_KEEP_CONNECT;
		reg |= SUNXI_DCTL_RUN_STOP;
	}
	else
		reg &= ~SUNXI_DCTL_RUN_STOP;

	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);
	usb3_phy_power(1);
	do {
		reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);
		if (is_on) {
			if (!(reg & SUNXI_DSTS_DEVCTRLHLT)) {
				break;
			}
		} else {
			if (reg & SUNXI_DSTS_DEVCTRLHLT)
				break;
		}
		timeout--;
		if (!timeout)
			break;
		udelay(1);
	} while (1);
}

#if defined(CONFIG_DRA7XX)

#define USBOTGSS_UTMI_OTG_STATUS 0x48880084
#define USBOTGSS_IRQSTATUS_0	 0x48880028
#define USBOTGSS_IRQSTATUS_1	 0x48880038

#else

#define USBOTGSS_UTMI_OTG_STATUS 0x4A020084
#define USBOTGSS_IRQSTATUS_0	 0x4A020028
#define USBOTGSS_IRQSTATUS_1	 0x4A020038

#endif

/* UTMI_OTG_STATUS REGISTER */
#define USBOTGSS_UTMI_OTG_STATUS_SW_MODE	(1 << 31)
#define USBOTGSS_UTMI_OTG_STATUS_POWERPRESENT	(1 << 9)
#define USBOTGSS_UTMI_OTG_STATUS_TXBITSTUFFENABLE (1 << 8)
#define USBOTGSS_UTMI_OTG_STATUS_IDDIG		(1 << 4)
#define USBOTGSS_UTMI_OTG_STATUS_SESSEND	(1 << 3)
#define USBOTGSS_UTMI_OTG_STATUS_SESSVALID	(1 << 2)
#define USBOTGSS_UTMI_OTG_STATUS_VBUSVALID	(1 << 1)

static int sunxi_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);
	u32			val;
	is_on = !!is_on;
	u32 retry = 0;	

	printf ("%s\n", __FUNCTION__);

	sunxi_gadget_run_stop(sunxi, is_on);
	//Set UTMI Status to 0x8000216
	val = readl(USBOTGSS_UTMI_OTG_STATUS);
	writel(0x80000216, USBOTGSS_UTMI_OTG_STATUS);
	
	mdelay(25);
	retry = 0;	
	do {
		val = readl(USBOTGSS_IRQSTATUS_1);
		retry++;
	} while ((val !=0)&&(retry < 25));



	return 0;
}
extern int sunxi_core_late_init(struct sunxi *sunxi);

static int sunxi_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);
	struct sunxi_ep		*dep;
	int			ret = 0;
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	if (sunxi->gadget_driver) {
		dev_err(sunxi->dev, "%s is already bound\n",
				sunxi->gadget.name);
		ret = -EBUSY;
		goto err0;
	}

	sunxi->gadget_driver	= driver;

	reg = sunxi_readl(sunxi->regs, SUNXI_DCFG);
	reg &= ~(SUNXI_DCFG_SPEED_MASK);
	reg |= SUNXI_DCFG_SUPERSPEED ;
	sunxi_writel(sunxi->regs, SUNXI_DCFG, reg);

	sunxi->start_config_issued = false;

	/* Start with SuperSpeed Default */
	sunxi_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = sunxi->eps[0];
	ret = __sunxi_gadget_ep_enable(dep, &sunxi_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(sunxi->dev, "failed to enable %s\n", dep->name);
		goto err0;
	}

	dep = sunxi->eps[1];
	ret = __sunxi_gadget_ep_enable(dep, &sunxi_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(sunxi->dev, "failed to enable %s\n", dep->name);
		goto err1;
	}

	/* begin to receive SETUP packets */
	sunxi->ep0state = EP0_SETUP_PHASE;
	sunxi_ep0_out_start(sunxi);


	return 0;

err1:
	__sunxi_gadget_ep_disable(sunxi->eps[0]);

err0:

	return ret;
}

#if 0
static int sunxi_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct sunxi		*sunxi = gadget_to_sunxi(g);
	unsigned long		flags;

	spin_lock_irqsave(&sunxi->lock, flags);

	__sunxi_gadget_ep_disable(sunxi->eps[0]);
	__sunxi_gadget_ep_disable(sunxi->eps[1]);

	sunxi->gadget_driver	= NULL;

	spin_unlock_irqrestore(&sunxi->lock, flags);

	return 0;
}
#endif
static const struct usb_gadget_ops sunxi_gadget_ops = {
	.get_frame		= sunxi_gadget_get_frame,
	.wakeup			= sunxi_gadget_wakeup,
	.set_selfpowered	= sunxi_gadget_set_selfpowered,
	.pullup			= sunxi_gadget_pullup,


};

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	int ret;

	printf ("%s\n", __FUNCTION__);
	ret = bind(&global_sunxi->gadget);
	if (ret)
		return ret;
	ret = sunxi_gadget_start(&global_sunxi->gadget, driver);
	if (ret)
		return ret;
	ret = usb_gadget_connect(&global_sunxi->gadget);
	if (ret)
		return ret;

	return 0;
}

/* -------------------------------------------------------------------------- */

static int __devinit sunxi_gadget_init_endpoints(struct sunxi *sunxi)
{
	struct sunxi_ep			*dep;
	u8				epnum;

	printf ("%s\n", __FUNCTION__);

	INIT_LIST_HEAD(&sunxi->gadget.ep_list);

	for (epnum = 0; epnum < SUNXI_ENDPOINTS_NUM; epnum++) {
		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(sunxi->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->sunxi = sunxi;
		dep->number = epnum;
		sunxi->eps[epnum] = dep;

		snprintf(dep->name, sizeof(dep->name), "ep%d%s", epnum >> 1,
				(epnum & 1) ? "in" : "out");
		dep->endpoint.name = dep->name;
		dep->direction = (epnum & 1);

		if (epnum == 0 || epnum == 1) {
			dep->endpoint.maxpacket = 512;
			dep->endpoint.ops = &sunxi_gadget_ep0_ops;
			if (!epnum)
				sunxi->gadget.ep0 = &dep->endpoint;
		} else {
			int		ret;

			dep->endpoint.maxpacket = 1024;
			dep->endpoint.ops = &sunxi_gadget_ep_ops;
			list_add_tail(&dep->endpoint.ep_list,
					&sunxi->gadget.ep_list);

			ret = sunxi_alloc_trb_pool(dep);
			if (ret)
				return ret;
		}

		INIT_LIST_HEAD(&dep->request_list);
		INIT_LIST_HEAD(&dep->req_queued);
	}

	return 0;
}

static void sunxi_gadget_free_endpoints(struct sunxi *sunxi)
{
	struct sunxi_ep			*dep;
	u8				epnum;

	printf ("%s\n", __FUNCTION__);

	for (epnum = 0; epnum < SUNXI_ENDPOINTS_NUM; epnum++) {
		dep = sunxi->eps[epnum];
		sunxi_free_trb_pool(dep);

		if (epnum != 0 && epnum != 1)
			list_del(&dep->endpoint.ep_list);

		kfree(dep);
	}
}

static struct sunxi	*the_sunxi;

/* -------------------------------------------------------------------------- */
static int sunxi_cleanup_done_reqs(struct sunxi *sunxi, struct sunxi_ep *dep,
		const struct sunxi_event_depevt *event, int status)
{
	struct sunxi_request	*req;
	struct sunxi_trb         trb;
	unsigned int		count;
	unsigned int		s_pkt = 0;

	printf ("%s\n", __FUNCTION__);

	do {
		req = next_request(&dep->req_queued);
		if (!req) {
			WARN_ON_ONCE(1);
			return 1;
		}

		sunxi_trb_to_nat(req->trb, &trb);

		if (trb.hwo && status != -ESHUTDOWN)
			/*
			 * We continue despite the error. There is not much we
			 * can do. If we don't clean in up we loop for ever. If
			 * we skip the TRB than it gets overwritten reused after
			 * a while since we use them in a ring buffer. a BUG()
			 * would help. Lets hope that if this occures, someone
			 * fixes the root cause instead of looking away :)
			 */
			dev_err(sunxi->dev, "%s's TRB (%p) still owned by HW\n",
					dep->name, req->trb);
		count = trb.length;

		if (dep->direction) {
			if (count) {
				dev_err(sunxi->dev, "incomplete IN transfer %s\n",
						dep->name);
				status = -ECONNRESET;
			}
		} else {
			if (count && (event->status & DEPEVT_STATUS_SHORT))
				s_pkt = 1;
		}

		/*
		 * We assume here we will always receive the entire data block
		 * which we should receive. Meaning, if we program RX to
		 * receive 4K but we receive only 2K, we assume that's all we
		 * should receive and we simply bounce the request back to the
		 * gadget driver for further processing.
		 */
		req->request.actual += req->request.length - count;
		sunxi_gadget_giveback(dep, req, status);
		if (s_pkt)
			break;
		if ((event->status & DEPEVT_STATUS_LST) && trb.lst)
			break;
		if ((event->status & DEPEVT_STATUS_IOC) && trb.ioc)
			break;
	} while (1);

	if ((event->status & DEPEVT_STATUS_IOC) && trb.ioc)
		return 0;
	return 1;
}

static void sunxi_endpoint_transfer_complete(struct sunxi *sunxi,
		struct sunxi_ep *dep, const struct sunxi_event_depevt *event,
		int start_new)
{
	unsigned		status = 0;
	int			clean_busy;

	printf ("%s\n", __FUNCTION__);

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	clean_busy =  sunxi_cleanup_done_reqs(sunxi, dep, event, status);
	if (clean_busy) {
		dep->flags &= ~SUNXI_EP_BUSY;
		dep->resource_index = 0;
	}

	/*
	 * WORKAROUND: This is the 2nd half of U1/U2 -> U0 workaround.
	 * See sunxi_gadget_linksts_change_interrupt() for 1st half.
	 */
	if (sunxi->revision < SUNXI_REVISION_183A) {
		u32		reg;
		int		i;

		for (i = 0; i < SUNXI_ENDPOINTS_NUM; i++) {
			struct sunxi_ep	*dep = sunxi->eps[i];

			if (!(dep->flags & SUNXI_EP_ENABLED))
				continue;

			if (!list_empty(&dep->req_queued))
				return;
		}

		reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
		reg |= sunxi->u1u2;
		sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

		sunxi->u1u2 = 0;
	}
}

static void sunxi_gadget_start_isoc(struct sunxi *sunxi,
		struct sunxi_ep *dep, const struct sunxi_event_depevt *event)
{
	u32 uf;

	printf ("%s\n", __FUNCTION__);

	if (list_empty(&dep->request_list)) {
		dev_vdbg(sunxi->dev, "ISOC ep %s run out for requests.\n",
			dep->name);
		return;
	}

	if (event->parameters) {
		u32 mask;

		mask = ~(dep->interval - 1);
		uf = event->parameters & mask;
		/* 4 micro frames in the future */
		uf += dep->interval * 4;
	} else {
		uf = 0;
	}

	__sunxi_gadget_kick_transfer(dep, uf, 1);
}

static void sunxi_process_ep_cmd_complete(struct sunxi_ep *dep,
		const struct sunxi_event_depevt *event)
{
	struct sunxi *sunxi = dep->sunxi;
	struct sunxi_event_depevt mod_ev = *event;

	printf ("%s\n", __FUNCTION__);

	/*
	 * We were asked to remove one requests. It is possible that this
	 * request and a few other were started together and have the same
	 * transfer index. Since we stopped the complete endpoint we don't
	 * know how many requests were already completed (and not yet)
	 * reported and how could be done (later). We purge them all until
	 * the end of the list.
	 */
	mod_ev.status = DEPEVT_STATUS_LST;
	sunxi_cleanup_done_reqs(sunxi, dep, &mod_ev, -ESHUTDOWN);
	dep->flags &= ~SUNXI_EP_BUSY;
	/* pending requets are ignored and are queued on XferNotReady */
}

static void sunxi_ep_cmd_compl(struct sunxi_ep *dep,
		const struct sunxi_event_depevt *event)
{
	u32 param = event->parameters;
	u32 cmd_type = (param >> 8) & ((1 << 5) - 1);

	printf ("%s\n", __FUNCTION__);

	switch (cmd_type) {
	case SUNXI_DEPCMD_ENDTRANSFER:
		sunxi_process_ep_cmd_complete(dep, event);
		break;
	case SUNXI_DEPCMD_STARTTRANSFER:
		dep->resource_index = param & 0x7f;
		break;
	default:
		break;
	};
}

static void sunxi_endpoint_interrupt(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event)
{
	struct sunxi_ep		*dep;
	u8			epnum = event->endpoint_number;

	printf ("%s\n", __FUNCTION__);

	dep = sunxi->eps[epnum];

	dev_vdbg(sunxi->dev, "%s: %s\n", dep->name,
			sunxi_ep_event_string(event->endpoint_event));

	if (epnum == 0 || epnum == 1) {
		
		sunxi_ep0_interrupt(sunxi, event);
		return;
	}

	switch (event->endpoint_event) {
	case SUNXI_DEPEVT_XFERCOMPLETE:
		if (usb_endpoint_xfer_isoc(dep->desc)) {
			dev_dbg(sunxi->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return;
		}

		sunxi_endpoint_transfer_complete(sunxi, dep, event, 1);
		break;
	case SUNXI_DEPEVT_XFERINPROGRESS:
		if (!usb_endpoint_xfer_isoc(dep->desc)) {
			dev_dbg(sunxi->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			return;
		}

		sunxi_endpoint_transfer_complete(sunxi, dep, event, 0);
		break;
	case SUNXI_DEPEVT_XFERNOTREADY:
		if (usb_endpoint_xfer_isoc(dep->desc)) {
			sunxi_gadget_start_isoc(sunxi, dep, event);
		} else {
			int ret;

			dev_vdbg(sunxi->dev, "%s: reason %s\n",
					dep->name, event->status
					? "Transfer Active"
					: "Transfer Not Active");

			ret = __sunxi_gadget_kick_transfer(dep, 0, 1);
			if (!ret || ret == -EBUSY)
				return;
			dev_dbg(sunxi->dev, "%s: failed to kick transfers %d.\n",
					dep->name, ret);
		}

		break;
	case SUNXI_DEPEVT_STREAMEVT:
		if (!usb_endpoint_xfer_bulk(dep->desc)) {
			dev_err(sunxi->dev, "Stream event for non-Bulk %s\n",
					dep->name);
			return;
		}

		switch (event->status) {
		case DEPEVT_STREAMEVT_FOUND:
			dev_vdbg(sunxi->dev, "Stream %d found and started\n",
					event->parameters);

			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
			/* FALLTHROUGH */
		default:
			dev_dbg(sunxi->dev, "Couldn't find suitable stream\n");
		}
		break;
	case SUNXI_DEPEVT_RXTXFIFOEVT:
		dev_dbg(sunxi->dev, "%s FIFO Overrun\n", dep->name);
		break;
	case SUNXI_DEPEVT_EPCMDCMPLT:
		sunxi_ep_cmd_compl(dep, event);
		break;
	}
}

static void sunxi_disconnect_gadget(struct sunxi *sunxi)
{
	printf ("%s\n", __FUNCTION__);
	if (sunxi->gadget_driver && sunxi->gadget_driver->disconnect) {
		spin_unlock(&sunxi->lock);
		sunxi->gadget_driver->disconnect(&sunxi->gadget);
		spin_lock(&sunxi->lock);
	}
}

static void sunxi_stop_active_transfer(struct sunxi *sunxi, u32 epnum)
{
	struct sunxi_ep *dep;
	struct sunxi_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;

	printf ("%s\n", __FUNCTION__);

	dep = sunxi->eps[epnum];

	WARN_ON(!dep->resource_index);
	if (dep->resource_index) {
		cmd = SUNXI_DEPCMD_ENDTRANSFER;
		cmd |= SUNXI_DEPCMD_HIPRI_FORCERM | SUNXI_DEPCMD_CMDIOC;
		cmd |= SUNXI_DEPCMD_PARAM(dep->resource_index);
		memset(&params, 0, sizeof(params));
		ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number, cmd, &params);
		WARN_ON_ONCE(ret);
		dep->resource_index = 0;
	}
}

static void sunxi_stop_active_transfers(struct sunxi *sunxi)
{
	u32 epnum;

	printf ("%s\n", __FUNCTION__);

	for (epnum = 2; epnum < SUNXI_ENDPOINTS_NUM; epnum++) {
		struct sunxi_ep *dep;

		dep = sunxi->eps[epnum];
		if (!(dep->flags & SUNXI_EP_ENABLED))
			continue;

		sunxi_remove_requests(sunxi, dep);
	}
}

static void sunxi_clear_stall_all_ep(struct sunxi *sunxi)
{
	u32 epnum;

	printf ("%s\n", __FUNCTION__);

	for (epnum = 1; epnum < SUNXI_ENDPOINTS_NUM; epnum++) {
		struct sunxi_ep *dep;
		struct sunxi_gadget_ep_cmd_params params;
		int ret;

		dep = sunxi->eps[epnum];

		if (!(dep->flags & SUNXI_EP_STALL))
			continue;

		dep->flags &= ~SUNXI_EP_STALL;

		memset(&params, 0, sizeof(params));
		ret = sunxi_send_gadget_ep_cmd(sunxi, dep->number,
				SUNXI_DEPCMD_CLEARSTALL, &params);
		WARN_ON_ONCE(ret);
	}
}

static void sunxi_gadget_disconnect_interrupt(struct sunxi *sunxi)
{
	printf ("%s\n", __FUNCTION__);
	dev_vdbg(sunxi->dev, "%s\n", __func__);
#if 0
	XXX
	U1/U2 is powersave optimization. Skip it for now. Anyway we need to
	enable it before we can disable it.

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	reg &= ~SUNXI_DCTL_INITU1ENA;
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	reg &= ~SUNXI_DCTL_INITU2ENA;
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);
#endif

	sunxi_stop_active_transfers(sunxi);
	sunxi_disconnect_gadget(sunxi);
	sunxi->start_config_issued = false;

	sunxi->gadget.speed = USB_SPEED_UNKNOWN;
	sunxi->setup_packet_pending = false;
}

static void sunxi_gadget_usb3_phy_power(struct sunxi *sunxi, int on)
{
	u32			reg;

	printf ("%s\n", __FUNCTION__);
	reg = sunxi_readl(sunxi->regs, SUNXI_GUSB3PIPECTL(0));

	if (on)
		reg &= ~SUNXI_GUSB3PIPECTL_SUSPHY;
	else
		reg |= SUNXI_GUSB3PIPECTL_SUSPHY;

	sunxi_writel(sunxi->regs, SUNXI_GUSB3PIPECTL(0), reg);
}

static void sunxi_gadget_usb2_phy_power(struct sunxi *sunxi, int on)
{
	u32			reg;
	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_GUSB2PHYCFG(0));

	if (on)
		reg &= ~SUNXI_GUSB2PHYCFG_SUSPHY;
	else
		reg |= SUNXI_GUSB2PHYCFG_SUSPHY;

	sunxi_writel(sunxi->regs, SUNXI_GUSB2PHYCFG(0), reg);
}

static void sunxi_gadget_reset_interrupt(struct sunxi *sunxi)
{
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	dev_vdbg(sunxi->dev, "%s\n", __func__);
	/*
	 * WORKAROUND: SUNXI revisions <1.88a have an issue which
	 * would cause a missing Disconnect Event if there's a
	 * pending Setup Packet in the FIFO.
	 *
	 * There's no suggested workaround on the official Bug
	 * report, which states that "unless the driver/application
	 * is doing any special handling of a disconnect event,
	 * there is no functional issue".
	 *
	 * Unfortunately, it turns out that we _do_ some special
	 * handling of a disconnect event, namely complete all
	 * pending transfers, notify gadget driver of the
	 * disconnection, and so on.
	 *
	 * Our suggested workaround is to follow the Disconnect
	 * Event steps here, instead, based on a setup_packet_pending
	 * flag. Such flag gets set whenever we have a XferNotReady
	 * event on EP0 and gets cleared on XferComplete for the
	 * same endpoint.
	 *
	 * Refers to:
	 *
	 * STAR#9000466709: RTL: Device : Disconnect event not
	 * generated if setup packet pending in FIFO
	 */
	if (sunxi->revision < SUNXI_REVISION_188A) {
		if (sunxi->setup_packet_pending)
			sunxi_gadget_disconnect_interrupt(sunxi);
	}

	sunxi->dev_state = SUNXI_DEFAULT_STATE;

	/* Enable PHYs */
	if (sunxi->revision < SUNXI_REVISION_194A) {
		sunxi_gadget_usb2_phy_power(sunxi, true);
		sunxi_gadget_usb3_phy_power(sunxi, true);
	}

	if (sunxi->gadget.speed != USB_SPEED_UNKNOWN)
		sunxi_disconnect_gadget(sunxi);

	reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
	reg &= ~SUNXI_DCTL_TSTCTRL_MASK;
	sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);

	sunxi_stop_active_transfers(sunxi);
	sunxi_clear_stall_all_ep(sunxi);
	sunxi->start_config_issued = false;

	/* Reset device address to zero */
	reg = sunxi_readl(sunxi->regs, SUNXI_DCFG);
	reg &= ~(SUNXI_DCFG_DEVADDR_MASK);
	sunxi_writel(sunxi->regs, SUNXI_DCFG, reg);
}

static void sunxi_update_ram_clk_sel(struct sunxi *sunxi, u32 speed)
{
	u32 reg;
	u32 usb30_clock = SUNXI_GCTL_CLK_BUS;

	printf ("%s\n", __FUNCTION__);

	/*
	 * We change the clock only at SS but I dunno why I would want to do
	 * this. Maybe it becomes part of the power saving plan.
	 */

	if (speed != SUNXI_DSTS_SUPERSPEED)
		return;

	/*
	 * RAMClkSel is reset to 0 after USB reset, so it must be reprogrammed
	 * each time on Connect Done.
	 */
	if (!usb30_clock)
		return;

	reg = sunxi_readl(sunxi->regs, SUNXI_GCTL);
	reg |= SUNXI_GCTL_RAMCLKSEL(usb30_clock);
	sunxi_writel(sunxi->regs, SUNXI_GCTL, reg);
}

static void sunxi_gadget_disable_phy(struct sunxi *sunxi, u8 speed)
{
	printf ("%s\n", __FUNCTION__);
	switch (speed) {
	case USB_SPEED_SUPER:
		sunxi_gadget_usb2_phy_power(sunxi, false);
		break;
	case USB_SPEED_HIGH:
	case USB_SPEED_FULL:
	case USB_SPEED_LOW:
		sunxi_gadget_usb3_phy_power(sunxi, false);
		break;
	}
}

static void sunxi_gadget_conndone_interrupt(struct sunxi *sunxi)
{
	struct sunxi_gadget_ep_cmd_params params;
	struct sunxi_ep		*dep;
	int			ret;
	u32			reg;
	u8			speed;
	printf ("%s\n", __FUNCTION__);

	dev_vdbg(sunxi->dev, "%s\n", __func__);

	memset(&params, 0x00, sizeof(params));

	reg = sunxi_readl(sunxi->regs, SUNXI_DSTS);
	speed = reg & SUNXI_DSTS_CONNECTSPD;
	sunxi->speed = speed;

	sunxi_update_ram_clk_sel(sunxi, speed);

	switch (speed) {
	case SUNXI_DCFG_SUPERSPEED:
		/*
		 * WORKAROUND: SUNXI revisions <1.90a have an issue which
		 * would cause a missing USB3 Reset event.
		 *
		 * In such situations, we should force a USB3 Reset
		 * event by calling our sunxi_gadget_reset_interrupt()
		 * routine.
		 *
		 * Refers to:
		 *
		 * STAR#9000483510: RTL: SS : USB3 reset event may
		 * not be generated always when the link enters poll
		 */
		if (sunxi->revision < SUNXI_REVISION_190A)
			sunxi_gadget_reset_interrupt(sunxi);

		sunxi_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		sunxi->gadget.ep0->maxpacket = 512;
		sunxi->gadget.speed = USB_SPEED_SUPER;
		break;
	case SUNXI_DCFG_HIGHSPEED:
		sunxi_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		sunxi->gadget.ep0->maxpacket = 64;
		sunxi->gadget.speed = USB_SPEED_HIGH;
		break;
	case SUNXI_DCFG_FULLSPEED2:
	case SUNXI_DCFG_FULLSPEED1:
		sunxi_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		sunxi->gadget.ep0->maxpacket = 64;
		sunxi->gadget.speed = USB_SPEED_FULL;
		break;
	case SUNXI_DCFG_LOWSPEED:
		sunxi_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		sunxi->gadget.ep0->maxpacket = 8;
		sunxi->gadget.speed = USB_SPEED_LOW;
		break;
	}


	/* Recent versions support automatic phy suspend and don't need this */
	if (sunxi->revision < SUNXI_REVISION_194A) {
		/* Suspend unneeded PHY */
		sunxi_gadget_disable_phy(sunxi, sunxi->gadget.speed);
	}

	dep = sunxi->eps[0];
	ret = __sunxi_gadget_ep_enable(dep, &sunxi_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(sunxi->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = sunxi->eps[1];
	ret = __sunxi_gadget_ep_enable(dep, &sunxi_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(sunxi->dev, "failed to enable %s\n", dep->name);
		return;
	}

	/*
	 * Configure PHY via GUSB3PIPECTLn if required.
	 *
	 * Update GTXFIFOSIZn
	 *
	 * In both cases reset values should be sufficient.
	 */
}

static void sunxi_gadget_wakeup_interrupt(struct sunxi *sunxi)
{

	printf ("%s\n", __FUNCTION__);

	dev_vdbg(sunxi->dev, "%s\n", __func__);

	/*
	 * TODO take core out of low power mode when that's
	 * implemented.
	 */

	sunxi->gadget_driver->resume(&sunxi->gadget);
}

static void sunxi_gadget_linksts_change_interrupt(struct sunxi *sunxi,
		unsigned int evtinfo)
{
	enum sunxi_link_state	next = evtinfo & SUNXI_LINK_STATE_MASK;

	printf ("%s\n", __FUNCTION__);

	/*
	 * WORKAROUND: SUNXI Revisions <1.83a have an issue which, depending
	 * on the link partner, the USB session might do multiple entry/exit
	 * of low power states before a transfer takes place.
	 *
	 * Due to this problem, we might experience lower throughput. The
	 * suggested workaround is to disable DCTL[12:9] bits if we're
	 * transitioning from U1/U2 to U0 and enable those bits again
	 * after a transfer completes and there are no pending transfers
	 * on any of the enabled endpoints.
	 *
	 * This is the first half of that workaround.
	 *
	 * Refers to:
	 *
	 * STAR#9000446952: RTL: Device SS : if U1/U2 ->U0 takes >128us
	 * core send LGO_Ux entering U0
	 */
	if (sunxi->revision < SUNXI_REVISION_183A) {
		if (next == SUNXI_LINK_STATE_U0) {
			u32	u1u2;
			u32	reg;

			switch (sunxi->link_state) {
			case SUNXI_LINK_STATE_U1:
			case SUNXI_LINK_STATE_U2:
				reg = sunxi_readl(sunxi->regs, SUNXI_DCTL);
				u1u2 = reg & (SUNXI_DCTL_INITU2ENA
						| SUNXI_DCTL_ACCEPTU2ENA
						| SUNXI_DCTL_INITU1ENA
						| SUNXI_DCTL_ACCEPTU1ENA);

				if (!sunxi->u1u2)
					sunxi->u1u2 = reg & u1u2;

				reg &= ~u1u2;

				sunxi_writel(sunxi->regs, SUNXI_DCTL, reg);
				break;
			default:
				/* do nothing */
				break;
			}
		}
	}

	sunxi->link_state = next;

	dev_vdbg(sunxi->dev, "%s link %d\n", __func__, sunxi->link_state);
}

static void sunxi_gadget_interrupt(struct sunxi *sunxi,
		const struct sunxi_event_devt *event)
{
	printf ("%s\n", __FUNCTION__);
	dev_vdbg(sunxi->dev, "%s: event_type \n", __func__, event->type);
	switch (event->type) {
	case SUNXI_DEVICE_EVENT_DISCONNECT:
		dev_vdbg(sunxi->dev, "Disconnect\n");
		sunxi_gadget_disconnect_interrupt(sunxi);
		break;
	case SUNXI_DEVICE_EVENT_RESET:
		sunxi_gadget_reset_interrupt(sunxi);
		break;
	case SUNXI_DEVICE_EVENT_CONNECT_DONE:
		sunxi_gadget_conndone_interrupt(sunxi);
		break;
	case SUNXI_DEVICE_EVENT_WAKEUP:
		sunxi_gadget_wakeup_interrupt(sunxi);
		break;
	case SUNXI_DEVICE_EVENT_LINK_STATUS_CHANGE:
		sunxi_gadget_linksts_change_interrupt(sunxi, event->event_info);
		break;
	case SUNXI_DEVICE_EVENT_EOPF:
		dev_vdbg(sunxi->dev, "End of Periodic Frame\n");
		break;
	case SUNXI_DEVICE_EVENT_SOF:
		dev_vdbg(sunxi->dev, "Start of Periodic Frame\n");
		break;
	case SUNXI_DEVICE_EVENT_ERRATIC_ERROR:
		dev_vdbg(sunxi->dev, "Erratic Error\n");
		break;
	case SUNXI_DEVICE_EVENT_CMD_CMPL:
		dev_vdbg(sunxi->dev, "Command Complete\n");
		break;
	case SUNXI_DEVICE_EVENT_OVERFLOW:
		dev_vdbg(sunxi->dev, "Overflow\n");
		break;
	default:
		dev_dbg(sunxi->dev, "UNKNOWN IRQ %d\n", event->type);
	}
}

static void sunxi_process_event_entry(struct sunxi *sunxi,
		const union sunxi_event *event)
{
	printf ("%s\n", __FUNCTION__);
	/* Endpoint IRQ, handle it and return early */
	if (event->type.is_devspec == 0) {
		/* depevt */
		return sunxi_endpoint_interrupt(sunxi, &event->depevt);
	}

	switch (event->type.type) {
	case SUNXI_EVENT_TYPE_DEV:
		sunxi_gadget_interrupt(sunxi, &event->devt);
		break;
	/* REVISIT what to do with Carkit and I2C events ? */
	default:
		dev_err(sunxi->dev, "UNKNOWN IRQ type %d\n", event->raw);
	}
}

static irqreturn_t sunxi_process_event_buf(struct sunxi *sunxi, u32 buf)
{
	struct sunxi_event_buffer *evt;
	int left;
	u32 count;

	printf ("%s\n", __FUNCTION__);

	count = sunxi_readl(sunxi->regs, SUNXI_GEVNTCOUNT(buf));
	count &= SUNXI_GEVNTCOUNT_MASK;
	if (!count) {
		return IRQ_NONE;
	}
	evt = sunxi->ev_buffs[buf];
	left = count;

	while (left > 0) {
		union sunxi_event event;

		memcpy(&event.raw, (evt->buf + evt->lpos), sizeof(event.raw));
		sunxi_process_event_entry(sunxi, &event);
		/*
		 * XXX we wrap around correctly to the next entry as almost all
		 * entries are 4 bytes in size. There is one entry which has 12
		 * bytes which is a regular entry followed by 8 bytes data. ATM
		 * I don't know how things are organized if were get next to the
		 * a boundary so I worry about that once we try to handle that.
		 */
		evt->lpos = (evt->lpos + 4) % SUNXI_EVENT_BUFFERS_SIZE;
		left -= 4;

		sunxi_writel(sunxi->regs, SUNXI_GEVNTCOUNT(buf), 4);
	}

	return IRQ_HANDLED;
}
extern u32 *trb_address;
irqreturn_t sunxi_interrupt(int irq, void *_sunxi)
{
	struct sunxi			*sunxi = _sunxi;
	int				i;
	irqreturn_t			ret = IRQ_NONE;
	u32				irq_any;

	printf ("%s\n", __FUNCTION__);

	spin_lock(&sunxi->lock);
	irq_any = readl(USBOTGSS_IRQSTATUS_0);

	if (!irq_any) { 
		goto exit;
	}	
	for (i = 0; i < sunxi->num_event_buffers; i++) {
		irqreturn_t status;

		status = sunxi_process_event_buf(sunxi, i);
		if (status == IRQ_HANDLED)
			ret = status;
	}
exit:
	spin_unlock(&sunxi->lock);
	return ret;
}
static void sunxi_gadget_usb3_phy_suspend(struct sunxi *sunxi, int suspend)
{
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_GUSB3PIPECTL(0));

	if (suspend)
		reg |= SUNXI_GUSB3PIPECTL_SUSPHY;
	else
		reg &= ~SUNXI_GUSB3PIPECTL_SUSPHY;

	sunxi_writel(sunxi->regs, SUNXI_GUSB3PIPECTL(0), reg);
}

static void sunxi_gadget_usb2_phy_suspend(struct sunxi *sunxi, int suspend)
{
	u32			reg;

	printf ("%s\n", __FUNCTION__);

	reg = sunxi_readl(sunxi->regs, SUNXI_GUSB2PHYCFG(0));

	if (suspend)
		reg |= SUNXI_GUSB2PHYCFG_SUSPHY;
	else
		reg &= ~SUNXI_GUSB2PHYCFG_SUSPHY;

	sunxi_writel(sunxi->regs, SUNXI_GUSB2PHYCFG(0), reg);
}

/**
 * sunxi_gadget_init - Initializes gadget related registers
 * @sunxi: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int __devinit sunxi_gadget_init(struct sunxi *sunxi)
{
	u32					reg;
	int					ret;

	printf ("%s\n", __FUNCTION__);

	sunxi->ctrl_req = dma_alloc_coherent(sunxi->dev, sizeof(*sunxi->ctrl_req),
			&sunxi->ctrl_req_addr, GFP_KERNEL);
	if (!sunxi->ctrl_req) {
		dev_err(sunxi->dev, "failed to allocate ctrl request\n");
		ret = -ENOMEM;
		goto err0;
	}

	sunxi->ep0_trb = dma_alloc_coherent(sunxi->dev, sizeof(*sunxi->ep0_trb) + 0x10,
			&sunxi->ep0_trb_addr, GFP_KERNEL);

	sunxi->ep0_trb = (struct sunxi_trb_hw *)((0x10 - ((u32)sunxi->ep0_trb & 0xf)) + 
		            (long)sunxi->ep0_trb);
	sunxi->ep0_trb_addr = (dma_addr_t)sunxi->ep0_trb;

	if (!sunxi->ep0_trb) {
		dev_err(sunxi->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err1;
	}

	sunxi->setup_buf = dma_alloc_coherent(sunxi->dev,
			sizeof(*sunxi->setup_buf) * 2,
			&sunxi->setup_buf_addr, GFP_KERNEL);
	if (!sunxi->setup_buf) {
		dev_err(sunxi->dev, "failed to allocate setup buffer\n");
		ret = -ENOMEM;
		goto err2;
	}

	sunxi->ep0_bounce = dma_alloc_coherent(sunxi->dev,
			512, &sunxi->ep0_bounce_addr, GFP_KERNEL);
	if (!sunxi->ep0_bounce) {
		dev_err(sunxi->dev, "failed to allocate ep0 bounce buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	dev_set_name(&sunxi->gadget.dev, "gadget");

	sunxi->gadget.ops			= &sunxi_gadget_ops;
	sunxi->gadget.is_dualspeed	= true;
	sunxi->gadget.speed		= USB_SPEED_UNKNOWN;

	sunxi->gadget.name		= "sunxi-gadget";

	the_sunxi	= sunxi;

	/*
	 * REVISIT: Here we should clear all pending IRQs to be
	 * sure we're starting from a well known location.
	 */

	ret = sunxi_gadget_init_endpoints(sunxi);
	if (ret)
		goto err4;

	printf("%s: success\n", __FUNCTION__);

	return 0;

	device_unregister(&sunxi->gadget.dev);
	sunxi_gadget_free_endpoints(sunxi);

err4:
	dma_free_coherent(sunxi->dev, 512, sunxi->ep0_bounce,
			sunxi->ep0_bounce_addr);

err3:
	dma_free_coherent(sunxi->dev, sizeof(*sunxi->setup_buf) * 2,
			sunxi->setup_buf, sunxi->setup_buf_addr);

err2:
	dma_free_coherent(sunxi->dev, sizeof(*sunxi->ep0_trb),
			sunxi->ep0_trb, sunxi->ep0_trb_addr);

err1:
	dma_free_coherent(sunxi->dev, sizeof(*sunxi->ctrl_req),
			sunxi->ctrl_req, sunxi->ctrl_req_addr);

err0:
	return ret;
}

void sunxi_gadget_exit(struct sunxi *sunxi)
{
	printf ("%s\n", __FUNCTION__);
	sunxi_writel(sunxi->regs, SUNXI_DEVTEN, 0x00);

	sunxi_gadget_free_endpoints(sunxi);

	dma_free_coherent(sunxi->dev, 512, sunxi->ep0_bounce,
			sunxi->ep0_bounce_addr);

	dma_free_coherent(sunxi->dev, sizeof(*sunxi->setup_buf) * 2,
			sunxi->setup_buf, sunxi->setup_buf_addr);

	dma_free_coherent(sunxi->dev, sizeof(*sunxi->ep0_trb),
			sunxi->ep0_trb, sunxi->ep0_trb_addr);

	dma_free_coherent(sunxi->dev, sizeof(*sunxi->ctrl_req),
			sunxi->ctrl_req, sunxi->ctrl_req_addr);

	device_unregister(&sunxi->gadget.dev);

	the_sunxi = NULL;
}

/* -------------------------------------------------------------------------- */

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	printf ("%s\n", __FUNCTION__);
	return usb_gadget_probe_driver(driver, driver->bind);
}

/**
 * usb_gadget_unregister_driver - unregisters a gadget driver.
 * @driver: the gadget driver to unregister
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sunxi		*sunxi = the_sunxi;
	printf ("%s\n", __FUNCTION__);

	if (!driver || !driver->unbind)
		return -EINVAL;

	if (!sunxi)
		return -ENODEV;

	if (sunxi->gadget_driver != driver)
		return -EINVAL;

	driver->disconnect(&sunxi->gadget);
	driver->unbind(&sunxi->gadget);


	sunxi->gadget_driver	= NULL;


	return 0;
}
