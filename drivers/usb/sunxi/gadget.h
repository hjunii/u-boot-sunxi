/**
 * gadget.h
 */

#ifndef __DRIVERS_USB_SUNXI_GADGET_H
#define __DRIVERS_USB_SUNXI_GADGET_H

#include <linux/list.h>
#include <linux/usb/gadget.h>
#include "io.h"

struct sunxi;
#define to_sunxi_ep(ep)		(container_of(ep, struct sunxi_ep, endpoint))
#define gadget_to_sunxi(g)	(container_of(g, struct sunxi, gadget))

/* DEPCFG parameter 1 */
#define SUNXI_DEPCFG_INT_NUM(n)		((n) << 0)
#define SUNXI_DEPCFG_XFER_COMPLETE_EN	(1 << 8)
#define SUNXI_DEPCFG_XFER_IN_PROGRESS_EN	(1 << 9)
#define SUNXI_DEPCFG_XFER_NOT_READY_EN	(1 << 10)
#define SUNXI_DEPCFG_FIFO_ERROR_EN	(1 << 11)
#define SUNXI_DEPCFG_STREAM_EVENT_EN	(1 << 13)
#define SUNXI_DEPCFG_BINTERVAL_M1(n)	((n) << 16)
#define SUNXI_DEPCFG_STREAM_CAPABLE	(1 << 24)
#define SUNXI_DEPCFG_EP_NUMBER(n)	((n) << 25)
#define SUNXI_DEPCFG_BULK_BASED		(1 << 30)
#define SUNXI_DEPCFG_FIFO_BASED		(1 << 31)

/* DEPCFG parameter 0 */
#define SUNXI_DEPCFG_EP_TYPE(n)		((n) << 1)
#define SUNXI_DEPCFG_MAX_PACKET_SIZE(n)	((n) << 3)
#define SUNXI_DEPCFG_FIFO_NUMBER(n)	((n) << 17)
#define SUNXI_DEPCFG_BURST_SIZE(n)	((n) << 22)
#define SUNXI_DEPCFG_DATA_SEQ_NUM(n)	((n) << 26)
#define SUNXI_DEPCFG_IGN_SEQ_NUM		(1 << 31)

/* DEPXFERCFG parameter 0 */
#define SUNXI_DEPXFERCFG_NUM_XFER_RES(n)	((n) & 0xffff)

struct sunxi_gadget_ep_cmd_params {
	u32	param2;
	u32	param1;
	u32	param0;
};

/* -------------------------------------------------------------------------- */

#define to_sunxi_request(r)	(container_of(r, struct sunxi_request, request))

static inline struct sunxi_request *next_request(struct list_head *list)
{
	if (list_empty(list))
		return NULL;

	return list_first_entry(list, struct sunxi_request, list);
}

static inline void sunxi_gadget_move_request_queued(struct sunxi_request *req)
{
	struct sunxi_ep		*dep = req->dep;

	req->queued = true;
	list_move_tail(&req->list, &dep->req_queued);
}

void sunxi_gadget_giveback(struct sunxi_ep *dep, struct sunxi_request *req,
		int status);

int sunxi_gadget_set_test_mode(struct sunxi *sunxi, int mode);
int sunxi_gadget_set_link_state(struct sunxi *sunxi, enum sunxi_link_state state);

void sunxi_ep0_interrupt(struct sunxi *sunxi,
		const struct sunxi_event_depevt *event);
void sunxi_ep0_out_start(struct sunxi *sunxi);
int sunxi_gadget_ep0_set_halt(struct usb_ep *ep, int value);
int sunxi_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
		gfp_t gfp_flags);
int __sunxi_gadget_ep_set_halt(struct sunxi_ep *dep, int value);
int sunxi_send_gadget_ep_cmd(struct sunxi *sunxi, unsigned ep,
		unsigned cmd, struct sunxi_gadget_ep_cmd_params *params);
void sunxi_map_buffer_to_dma(struct sunxi_request *req);
void sunxi_unmap_buffer_from_dma(struct sunxi_request *req);
int sunxi_send_gadget_generic_command(struct sunxi *sunxi, int cmd, u32 param);

/**
 * sunxi_gadget_ep_get_transfer_index - Gets transfer index from HW
 * @sunxi: DesignWare USB3 Pointer
 * @number: DWC endpoint number
 *
 * Caller should take care of locking
 */
static inline u32 sunxi_gadget_ep_get_transfer_index(struct sunxi *sunxi, u8 number)
{
	u32			res_id;

	res_id = sunxi_readl(sunxi->regs, SUNXI_DEPCMD(number));

	return SUNXI_DEPCMD_GET_RSC_IDX(res_id);
}

/**
 * sunxi_gadget_event_string - returns event name
 * @event: the event code
 */
static inline const char *sunxi_gadget_event_string(u8 event)
{
	switch (event) {
	case SUNXI_DEVICE_EVENT_DISCONNECT:
		return "Disconnect";
	case SUNXI_DEVICE_EVENT_RESET:
		return "Reset";
	case SUNXI_DEVICE_EVENT_CONNECT_DONE:
		return "Connection Done";
	case SUNXI_DEVICE_EVENT_LINK_STATUS_CHANGE:
		return "Link Status Change";
	case SUNXI_DEVICE_EVENT_WAKEUP:
		return "WakeUp";
	case SUNXI_DEVICE_EVENT_EOPF:
		return "End-Of-Frame";
	case SUNXI_DEVICE_EVENT_SOF:
		return "Start-Of-Frame";
	case SUNXI_DEVICE_EVENT_ERRATIC_ERROR:
		return "Erratic Error";
	case SUNXI_DEVICE_EVENT_CMD_CMPL:
		return "Command Complete";
	case SUNXI_DEVICE_EVENT_OVERFLOW:
		return "Overflow";
	}

	return "UNKNOWN";
}

/**
 * sunxi_ep_event_string - returns event name
 * @event: then event code
 */
static inline const char *sunxi_ep_event_string(u8 event)
{
	switch (event) {
	case SUNXI_DEPEVT_XFERCOMPLETE:
		return "Transfer Complete";
	case SUNXI_DEPEVT_XFERINPROGRESS:
		return "Transfer In-Progress";
	case SUNXI_DEPEVT_XFERNOTREADY:
		return "Transfer Not Ready";
	case SUNXI_DEPEVT_RXTXFIFOEVT:
		return "FIFO";
	case SUNXI_DEPEVT_STREAMEVT:
		return "Stream";
	case SUNXI_DEPEVT_EPCMDCMPLT:
		return "Endpoint Command Complete";
	}

	return "UNKNOWN";
}

#endif /* __DRIVERS_USB_SUNXI_GADGET_H */
