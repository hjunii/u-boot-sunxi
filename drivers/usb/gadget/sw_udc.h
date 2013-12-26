#ifndef  __SW_UDC_H__
#define  __SW_UDC_H__

#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/list.h>
#include <usb/lin_gadget_compat.h>

typedef struct sw_udc_ep {
	struct list_head			queue;
	unsigned long				last_io;
	struct usb_gadget			*gadget;
	struct sw_udc				*dev;
	const struct usb_endpoint_descriptor	*desc;
	struct usb_ep				ep;
	u8					num;

	unsigned short				fifo_size;
	u8					bEndpointAddress;
	u8					bmAttributes;

	unsigned				halted : 1;
	unsigned				already_seen : 1;
	unsigned				setup_stage : 1;
} sw_udc_ep_t;

#define  EP0_FIFO_SIZE				64

#define  SW_UDC_EP_FIFO_SIZE			512

#define  SW_UDC_EP_CTRL_INDEX			0x00
#define  SW_UDC_EP_BULK_IN_INDEX		0x01
#define  SW_UDC_EP_BULK_OUT_INDEX		0x02

#define  SW_UDC_FIFO_NUM			1

static const char ep0name [] = "ep0";

static const char *const ep_name[] = {
	ep0name,        /* everyone has ep0 */

	/* sw_udc four bidirectional bulk endpoints */
	"ep1-bulk",
	"ep2-bulk",
	"ep3-bulk",
	"ep4-bulk",
	"ep5-int",
};

#define SW_UDC_ENDPOINTS			6

struct sw_udc_request {
	struct list_head			queue;          /* ep's requests */
	struct usb_request			req;

	__u32 is_queue;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

typedef struct sw_udc_io {
	struct resource *usb_base_res;          /* USB  resources               */
	struct resource *usb_base_req;          /* USB  resources               */
	void __iomem    *usb_vbase;             /* USB  base address    	*/

	struct resource *sram_base_res;         /* SRAM resources               */
	struct resource *sram_base_req;         /* SRAM resources               */
	void __iomem    *sram_vbase;            /* SRAM base address    	*/

	struct resource *clock_base_res;        /* clock resources              */
	struct resource *clock_base_req;        /* clock resources              */
	void __iomem    *clock_vbase;           /* clock base address   	*/

#if 0
	bsp_usbc_t usbc;                        /* usb bsp config               */
	__hdle usb_bsp_hdle;                    /* usb bsp handle               */
#endif

	__u32 clk_is_open;                      /* is usb clock open?   */
	struct clk      *sie_clk;               /* SIE clock handle     */
	struct clk      *phy_clk;               /* PHY clock handle     */
	struct clk      *phy0_clk;              /* PHY0 clock handle    */

	long Drv_vbus_Handle;
} sw_udc_io_t;

typedef struct sw_udc {
	struct sw_udc_ep                    	ep[SW_UDC_ENDPOINTS];
	int                                     address;
	struct usb_gadget                   	gadget;
	struct usb_gadget_driver        	*driver;
	struct sw_udc_request           	fifo_req;
	u8                                      fifo_buf[SW_UDC_EP_FIFO_SIZE];
	u16                                     devstatus;

	u32                                     port_status;
	int                                     ep0state;

	unsigned                                got_irq : 1;

	unsigned                                req_std : 1;
	unsigned                                req_config : 1;
	unsigned                                req_pending : 1;
	u8                                      vbus;
	struct dentry                       	*regs_info;

	sw_udc_io_t                             *sw_udc_io;
	char                                    driver_name[32];
	__u32                                   usbc_no;

	u32                                     stoped;
	u32                                     irq_no;
} sw_udc_t;

enum sw_udc_cmd_e {
	SW_UDC_P_ENABLE = 1,    /* Pull-up enable        */
	SW_UDC_P_DISABLE = 2,   /* Pull-up disable       */
	SW_UDC_P_RESET  = 3,    /* UDC reset, in case of */
};

typedef struct sw_udc_mach_info {
	struct usb_port_info *port_info;
	unsigned int usbc_base;
} sw_udc_mach_info_t;

#endif   //__SW_UDC_H__ 
