/*
 * drivers/usb/gadget/sunxi_udc_otg.c
 */
//#undef DEBUG
#define DEBUG
#include <common.h>
#include <asm/errno.h>
#include <linux/list.h>
#include <malloc.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/io.h>

#include <asm/mach-types.h>
#include <asm/arch/gpio.h>

#include <asm/arch/clock.h>
#include <usb/lin_gadget_compat.h>

#include "regs-otg-sunxi.h"

/***********************************************************/

#define OTG_DMA_MODE		1

#define DEBUG_SETUP 1
#define DEBUG_EP0 1
#define DEBUG_ISR 1
#define DEBUG_OUT_EP 1
#define DEBUG_IN_EP 1

#include <usb/sunxi_udc.h>

#define EP0_CON		0
#define EP_MASK		0xF

#define CTRL_EP_INDEX		0
#define BULK_IN_EP_INDEX        1       /* tx */
#define BULK_OUT_EP_INDEX       2       /* rx */

static char *state_names[] = {
	"WAIT_FOR_SETUP",
	"DATA_STATE_XMIT",
	"DATA_STATE_NEED_ZLP",
	"WAIT_FOR_OUT_STATUS",
	"DATA_STATE_RECV",
	"WAIT_FOR_COMPLETE",
	"WAIT_FOR_OUT_COMPLETE",
	"WAIT_FOR_IN_COMPLETE",
	"WAIT_FOR_NULL_COMPLETE",
};

#define DRIVER_DESC "SUNXI HS USB OTG Device Driver"
#define DRIVER_VERSION "23 Dec 2013"

struct sunxi_udc	*the_controller;

static const char driver_name[] = "sunxi-udc";
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

/* Max packet size*/
static unsigned int ep0_fifo_size = 64;
static unsigned int ep_fifo_size =  512;
static unsigned int ep_fifo_size2 = 1024;
static int reset_available = 1;

static struct usb_ctrlrequest *usb_ctrl;
static dma_addr_t usb_ctrl_dma_addr;

/*
  Local declarations.
*/
static int sunxi_ep_enable(struct usb_ep *ep,
			 const struct usb_endpoint_descriptor *);
static int sunxi_ep_disable(struct usb_ep *ep);
static struct usb_request *sunxi_alloc_request(struct usb_ep *ep,
					     gfp_t gfp_flags);
static void sunxi_free_request(struct usb_ep *ep, struct usb_request *);

static int sunxi_queue(struct usb_ep *ep, struct usb_request *, gfp_t gfp_flags);
static int sunxi_dequeue(struct usb_ep *ep, struct usb_request *);
static int sunxi_fifo_status(struct usb_ep *ep);
static void sunxi_fifo_flush(struct usb_ep *ep);
static void sunxi_ep0_read(struct sunxi_udc *dev);
static void sunxi_ep0_kick(struct sunxi_udc *dev, struct sunxi_ep *ep);
static void sunxi_handle_ep0(struct sunxi_udc *dev);
static int sunxi_ep0_write(struct sunxi_udc *dev);
static int write_fifo_ep0(struct sunxi_ep *ep, struct sunxi_request *req);
static void done(struct sunxi_ep *ep, struct sunxi_request *req, int status);
static void stop_activity(struct sunxi_udc *dev,
			  struct usb_gadget_driver *driver);
static int udc_enable(struct sunxi_udc *dev);
static void udc_set_address(struct sunxi_udc *dev, unsigned char address);
static void reconfig_usbd(void);
static void set_max_pktsize(struct sunxi_udc *dev, enum usb_device_speed speed);
static void nuke(struct sunxi_ep *ep, int status);
static int sunxi_udc_set_halt(struct usb_ep *_ep, int value);
static void sunxi_udc_set_nak(struct sunxi_ep *ep);

static void sunxi_udc_reset(void);

void set_udc_gadget_private_data(void *p)
{
#if 0
	debug_cond(DEBUG_SETUP != 0,
		   "%s: the_controller: 0x%p, p: 0x%p\n", __func__,
		   the_controller, p);
	the_controller->gadget.dev.device_data = p;
#endif
}

void *get_udc_gadget_private_data(struct usb_gadget *gadget)
{
#if 0
	return gadget->dev.device_data;
#else
	return NULL;
#endif
}

static struct usb_ep_ops sunxi_ep_ops = {
	.enable = sunxi_ep_enable,
	.disable = sunxi_ep_disable,

	.alloc_request = sunxi_alloc_request,
	.free_request = sunxi_free_request,

	.queue = sunxi_queue,
	.dequeue = sunxi_dequeue,

	.set_halt = sunxi_udc_set_halt,
	.fifo_status = sunxi_fifo_status,
	.fifo_flush = sunxi_fifo_flush,
};

#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

/***********************************************************/

void __iomem		*regs_otg;
struct sunxi_usbotg_reg *reg;
struct sunxi_usbotg_phy *phy;
static unsigned int usb_phy_ctrl;

void otg_phy_init(struct sunxi_udc *dev)
{
	/*USB PHY0 Enable */
	printf("USB PHY0 Enable\n");

	/* Enable PHY */
	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *) dev->ccmu_base;

	// close_usb_clock(udc.ccmu_base);
	clrbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	clrbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);

	// open_usb_clock(udc.ccmu_base);
	setbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	setbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);
}

void otg_phy_off(struct sunxi_udc *dev)
{
#if 0
	/* reset controller just in case */
	writel(PHY_SW_RST0, &phy->rstcon);
	udelay(20);
	writel(readl(&phy->phypwr) &~PHY_SW_RST0, &phy->rstcon);
	udelay(20);

	writel(readl(&phy->phypwr) | OTG_DISABLE_0 | ANALOG_PWRDOWN
	       | FORCE_SUSPEND_0, &phy->phypwr);

	writel(readl(usb_phy_ctrl) &~USB_PHY_CTRL_EN0, usb_phy_ctrl);

	writel((readl(&phy->phyclk) & ~(ID_PULLUP0 | COMMON_ON_N0)),
	      &phy->phyclk);

	udelay(10000);

	dev->pdata->phy_control(0);
#endif
}

/***********************************************************/

#include "sunxi_udc_otg_xfer_dma.c"

/*
 *	udc_disable - disable USB device controller
 */
static void udc_disable(struct sunxi_udc *dev)
{
	debug_cond(DEBUG_SETUP != 0, "%s: %p\n", __func__, dev);

	udc_set_address(dev, 0);

	dev->ep0state = WAIT_FOR_SETUP;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->usb_address = 0;

	otg_phy_off(dev);
}

/*
 *	udc_reinit - initialize software state
 */
static void udc_reinit(struct sunxi_udc *dev)
{
	unsigned int i;

	debug_cond(DEBUG_SETUP != 0, "%s: %p\n", __func__, dev);

	/* device/ep0 records init */
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);
	dev->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < SUNXI_MAX_ENDPOINTS; i++) {
		struct sunxi_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */
}

#define BYTES2MAXP(x)	(x / 8)
#define MAXP2BYTES(x)	(x * 8)

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static int udc_enable(struct sunxi_udc *dev)
{
	debug_cond(DEBUG_SETUP != 0, "%s: %p\n", __func__, dev);

	otg_phy_init(dev);
	reconfig_usbd();
	sunxi_udc_reset();

	debug_cond(DEBUG_SETUP != 0,
		   "SUNXI USB 2.0 OTG Controller Core Initialized\n");

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	return 0;
}

/*
  Register entry point for the peripheral controller driver.
*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct sunxi_udc *dev = the_controller;
	int retval = 0;
	unsigned long flags;

	debug_cond(DEBUG_SETUP != 0, "%s: %s\n", __func__, "no name");

	if (!driver
	    || !driver->bind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	if (!dev)
		return -ENODEV;
	if (dev->driver)
		return -EBUSY;

	spin_lock_irqsave(&dev->lock, flags);
	/* first hook up the driver ... */
	dev->driver = driver;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (retval) { /* TODO */
		printf("target device_add failed, error %d\n", retval);
		return retval;
	}

	retval = driver->bind(&dev->gadget);
	if (retval) {
		debug_cond(DEBUG_SETUP != 0,
			   "%s: bind to driver --> error %d\n",
			    dev->gadget.name, retval);
		dev->driver = 0;
		return retval;
	}

	enable_irq(IRQ_OTG);

	debug_cond(DEBUG_SETUP != 0,
		   "Registered gadget driver %s\n", dev->gadget.name);
	udc_enable(dev);

	return 0;
}

/*
 * Unregister entry point for the peripheral controller driver.
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sunxi_udc *dev = the_controller;
	unsigned long flags;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);
	dev->driver = 0;
	stop_activity(dev, driver);
	spin_unlock_irqrestore(&dev->lock, flags);

	driver->unbind(&dev->gadget);

	disable_irq(IRQ_OTG);

	udc_disable(dev);

	return 0;
}

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct sunxi_ep *ep, struct sunxi_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	debug("%s: %s %p, req = %p, stopped = %d\n",
	      __func__, ep->ep.name, ep, &req->req, stopped);

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN) {
		debug("complete %s req %p stat %d len %u/%u\n",
		      ep->ep.name, &req->req, status,
		      req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

#ifdef DEBUG
	printf("calling complete callback\n");
	{
		int i, len = req->req.length;

		printf("pkt[%d] = ", req->req.length);
		if (len > 64)
			len = 64;
		for (i = 0; i < len; i++) {
			printf("%02x", ((u8 *)req->req.buf)[i]);
			if ((i & 7) == 7)
				printf(" ");
		}
		printf("\n");
	}
#endif
	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);

	debug("callback completed\n");

	ep->stopped = stopped;
}

/*
 *	nuke - dequeue ALL requests
 */
static void nuke(struct sunxi_ep *ep, int status)
{
	struct sunxi_request *req;

	debug("%s: %s %p\n", __func__, ep->ep.name, ep);

	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct sunxi_request, queue);
		done(ep, req, status);
	}
}

static void stop_activity(struct sunxi_udc *dev,
			  struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < SUNXI_MAX_ENDPOINTS; i++) {
		struct sunxi_ep *ep = &dev->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&dev->lock);
		driver->disconnect(&dev->gadget);
		spin_lock(&dev->lock);
	}

	/* re-init driver-visible data structures */
	udc_reinit(dev);
}

static void reconfig_usbd(void)
{
	/* 2. Soft-reset OTG Core and then unreset again. */
	debug("Reseting OTG controller\n");

	the_controller->usb_address = 0;
	the_controller->gadget.speed = USB_SPEED_UNKNOWN;
}

static void sunxi_udc_reset(void)
{
	u32 reg;

	// USBC_ForceId(udc.bsp, USBC_ID_TYPE_DEVICE)
	// __USBC_ForceIdToHigh(usbc_otg->base_addr)
	reg = readl(the_controller->usb_base + SUNXI_ISCR);
        reg |=  (0x03 << SUNXI_BP_ISCR_FORCE_ID);
	
	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	
	writel(reg, the_controller->usb_base + SUNXI_ISCR);
	
	// USBC_ForceVbusValid(udc.bsp, USBC_VBUS_TYPE_HIGH)
	// __USBC_ForceVbusValidToHigh(usbc_otg->base_addr)
	reg = readl(the_controller->usb_base + SUNXI_ISCR);
	reg |= (0x03 << SUNXI_BP_ISCR_FORCE_VBUS_VALID);

	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);
	
	writel(reg, the_controller->usb_base + SUNXI_ISCR);

	// USBC_Dev_ConectSwitch(udc.bsp, USBC_DEVICE_SWITCH_OFF);
	clrbits_8(the_controller->usb_base + SUNXI_PCTL, 0x1 << SUNXI_BP_POWER_D_SOFT_CONNECT);

	// USBC_EnableDpDmPullUp(udc.bsp);
	reg = readl(the_controller->usb_base + SUNXI_ISCR);
	reg |= (1 << SUNXI_BP_ISCR_DPDM_PULLUP_EN);

	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);

	writel(reg, the_controller->usb_base + SUNXI_ISCR);

	// USBC_EnableIdPullUp(udc.bsp);
	reg = readl(the_controller->usb_base + SUNXI_ISCR);
	reg |= (1 << SUNXI_BP_ISCR_ID_PULLUP_EN);

	// reg_val = __USBC_WakeUp_ClearChangeDetect(reg_val)
	reg &= ~(1 << SUNXI_BP_ISCR_VBUS_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_ID_CHANGE_DETECT);
	reg &= ~(1 << SUNXI_BP_ISCR_DPDM_CHANGE_DETECT);

	writel(reg, the_controller->usb_base + SUNXI_ISCR);

	// USBC_SelectBus(udc.bsp, USBC_IO_TYPE_PIO, 0, 0);
	reg = readb(the_controller->usb_base + SUNXI_VEND0);
	reg &= 0x00;
	writeb(reg, the_controller->usb_base + SUNXI_VEND0);

	// USBC_ConfigFIFO_Base(udc.bsp, udc.sram_base, USBC_FIFO_MODE_8K);
	reg = readl(the_controller->sram_base + 0x04);
	reg |= (1 << 0);
	writel(reg, the_controller->sram_base + 0x04);

	// USBC_EnhanceSignal(udc.bsp);
	// NONE
	
	// USBC_Dev_ConfigTransferMode(udc.bsp, USBC_TS_TYPE_BULK, USBC_TS_MODE_HS);
	// __USBC_Dev_TsType_Bulk(usbc_otg->base_addr)
	clrbits_8(the_controller->usb_base + SUNXI_PCTL, 0x1 < SUNXI_BP_POWER_D_ISO_UPDATE_EN);
	// __USBC_Dev_TsMode_Hs(usbc_otg->base_addr)
	setbits_8(the_controller->usb_base + SUNXI_PCTL, 0x1 < SUNXI_BP_POWER_D_HIGH_SPEED_EN);

	/* disable all interrupt */
	// USBC_INT_DisableUsbMiscAll(udc.bsp);
	writeb(0, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_DisableEpAll(udc.bsp, USBC_EP_TYPE_RX);
	// __USBC_INT_DisableTxAll(usbc_otg->base_addr);
	writew(0, the_controller->usb_base + SUNXI_INTRxE);
	// USBC_INT_DisableEpAll(udc.bsp, USBC_EP_TYPE_TX);
	// __USBC_INT_DisableTxAll(usbc_otg->base_addr);
	writew(0, the_controller->usb_base + SUNXI_INTTxE);

	// USBC_INT_EnableUsbMiscUint(udc.bsp, USBC_BP_INTUSB_SOF);
	reg = readb(the_controller->usb_base + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_SOF;
	writeb(reg, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_EnableUsbMiscUint(udc.bsp, USBC_BP_INTUSB_SUSPEND);
	reg = readb(the_controller->usb_base + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_SUSPEND;
	writeb(reg, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_EnableUsbMiscUint(udc.bsp, USBC_BP_INTUSB_RESUME);
	reg = readb(the_controller->usb_base + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_RESUME;
	writeb(reg, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_EnableUsbMiscUint(udc.bsp, USBC_BP_INTUSB_RESET);
	reg = readb(the_controller->usb_base + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_RESET;
	writeb(reg, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_EnableUsbMiscUint(udc.bsp, USBC_BP_INTUSB_DISCONNECT);
	reg = readb(the_controller->usb_base + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_DISCONNECT;
	writeb(reg, the_controller->usb_base + SUNXI_INTUSBE);
	// USBC_INT_EnableEp(udc.bsp, USBC_EP_TYPE_TX, 0);
	// __USBC_INT_EnableTxEp(usbc_otg->base_addr, ep_index);
	setbits_le16(the_controller->usb_base + SUNXI_INTTxE, 1 << 0);

	// fastboot_bulk_endpoint_reset();
	u8 old_ep_index = 0;

	//old_ep_index = USBC_GetActiveEp(udc.bsp);
	old_ep_index = readb(the_controller->usb_base + SUNXI_EPIND);

	/* tx */
	// USBC_SelectActiveEp(udc.bsp, BULK_IN_EP_INDEX);
	writeb(BULK_IN_EP_INDEX, the_controller->usb_base + SUNXI_EPIND);
	// USBC_Dev_ConfigEp(udc.bsp, USBC_TS_TYPE_BULK, USBC_EP_TYPE_TX, 1, udc.bulk_ep_size & 0x7ff);
	// __USBC_Dev_Tx_ConfigEp(usbc_otg->base_addr, ts_type, is_double_fifo,  ep_MaxPkt)
	// --<1>--config tx csr
	reg = (1 << SUNXI_BP_TXCSR_D_MODE);
	reg |= (1 << SUNXI_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
	reg |= (1 << SUNXI_BP_TXCSR_D_FLUSH_FIFO);
	writew(reg, the_controller->usb_base + SUNXI_TXCSR);
	writew(reg, the_controller->usb_base + SUNXI_TXCSR);
	// --<2>--config tx ep max packet
	reg = readw(the_controller->usb_base + SUNXI_TXMAXP);
	reg |= (512 & 0x7ff) & ((1 << SUNXI_BP_TXMAXP_PACKET_COUNT) - 1);
	writew(reg, the_controller->usb_base + SUNXI_TXMAXP);
	// --<3>--config ep transfer type
	// __USBC_Dev_Tx_EnableBulkEp(usbc_base_addr);
	clrbits_le16(the_controller->usb_base + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_ISO);
	// USBC_ConfigFifo(udc.bsp, USBC_EP_TYPE_TX, 1, udc.fifo_size, 1024);
	// __USBC_ConfigFifo_TxEp(usbc_otg->base_addr, is_double_fifo, fifo_size, fifo_addr);
	u32 temp = 0;
	u32 size = 0;
	temp = 512 + 511;
	temp &= ~511;
	temp >>= 3;
	temp >>= 1;
	while(temp){
		size++;
		temp >>= 1;
	}
	writew(1024 >> 3, the_controller->usb_base + SUNXI_TXFIFOAD);
	writeb(size & 0x0f, the_controller->usb_base + SUNXI_TXFIFOSZ);
	setbits_8(the_controller->usb_base + SUNXI_TXFIFOSZ, 1 << SUNXI_BP_TXFIFOSZ_DPB);
	// USBC_INT_EnableEp(udc.bsp, USBC_EP_TYPE_TX, BULK_IN_EP_INDEX);
	// __USBC_INT_EnableTxEp(usbc_otg->base_addr, ep_index);
	setbits_le16(the_controller->usb_base + SUNXI_INTTxE, 1 << BULK_IN_EP_INDEX);

	/* rx */
	// USBC_SelectActiveEp(udc.bsp, BULK_OUT_EP_INDEX);
	writeb(BULK_OUT_EP_INDEX, the_controller->usb_base + SUNXI_EPIND);
	// USBC_Dev_ConfigEp(udc.bsp, USBC_TS_TYPE_BULK, USBC_EP_TYPE_RX, 1, udc.bulk_ep_size & 0x7ff);
	// __USBC_Dev_Rx_ConfigEp(usbc_otg->base_addr, ts_type, is_double_fifo, ep_MaxPkt);
	// --<1>--config tx csr
	reg = (1 << SUNXI_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << SUNXI_BP_RXCSR_D_FLUSH_FIFO);
	writew(reg, the_controller->usb_base + SUNXI_RXCSR);
	writew(reg, the_controller->usb_base + SUNXI_RXCSR);
	// --<2>--config tx ep max packet
	reg = readw(the_controller->usb_base + SUNXI_RXMAXP);
	reg |= (512 & 0x7ff) & ((1 << SUNXI_BP_RXMAXP_PACKET_COUNT) - 1);
	writew(reg, the_controller->usb_base + SUNXI_RXMAXP);
	// __USBC_Dev_Rx_EnableBulkEp(usbc_base_addr);
	clrbits_le16(the_controller->usb_base + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_ISO);
	// USBC_ConfigFifo(udc.bsp, USBC_EP_TYPE_RX, 1, udc.fifo_size, 2048);
	//  __USBC_ConfigFifo_RxEp(usbc_otg->base_addr, is_double_fifo, fifo_size, fifo_addr);
	temp = 0;
	size = 0;
	temp = 512 + 511;
	temp &= ~511;
	temp >>= 3;
	temp >>= 1;
	while(temp){
		size++;
		temp >>= 1;
	}
	writew(2048 >> 3, the_controller->usb_base + SUNXI_RXFIFOAD);
	writeb(size & 0x0f, the_controller->usb_base + SUNXI_RXFIFOSZ);
	setbits_8(the_controller->usb_base + SUNXI_RXFIFOSZ, 1 << SUNXI_BP_RXFIFOSZ_DPB);
	// USBC_INT_EnableEp(udc.bsp, USBC_EP_TYPE_RX, BULK_OUT_EP_INDEX);
	// __USBC_INT_EnableRxEp(usbc_otg->base_addr, ep_index);
	setbits_le16(the_controller->usb_base + SUNXI_INTRxE, 1 << BULK_OUT_EP_INDEX);

	// USBC_SelectActiveEp(udc.bsp, old_ep_index);
	writeb(old_ep_index, the_controller->usb_base + SUNXI_EPIND);

	// USBC_Dev_ConectSwitch(udc.bsp, USBC_DEVICE_SWITCH_ON);
	setbits_8(the_controller->usb_base + SUNXI_PCTL, 0x1 << SUNXI_BP_POWER_D_SOFT_CONNECT);
}

static void set_max_pktsize(struct sunxi_udc *dev, enum usb_device_speed speed)
{
	unsigned int ep_ctrl;
	int i;

	if (speed == USB_SPEED_HIGH) {
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		dev->gadget.speed = USB_SPEED_HIGH;
	} else {
		ep0_fifo_size = 64;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;
		dev->gadget.speed = USB_SPEED_FULL;
	}

	dev->ep[0].ep.maxpacket = ep0_fifo_size;
	for (i = 1; i < SUNXI_MAX_ENDPOINTS; i++)
		dev->ep[i].ep.maxpacket = ep_fifo_size;

#if 0
	/* EP0 - Control IN (64 bytes)*/
	ep_ctrl = readl(&reg->in_endp[EP0_CON].diepctl);
	writel(ep_ctrl|(0<<0), &reg->in_endp[EP0_CON].diepctl);

	/* EP0 - Control OUT (64 bytes)*/
	ep_ctrl = readl(&reg->out_endp[EP0_CON].doepctl);
	writel(ep_ctrl|(0<<0), &reg->out_endp[EP0_CON].doepctl);
#endif
}

static int sunxi_ep_enable(struct usb_ep *_ep,
			 const struct usb_endpoint_descriptor *desc)
{
	struct sunxi_ep *ep;
	struct sunxi_udc *dev;
	unsigned long flags;

	debug("%s: %p\n", __func__, _ep);

	ep = container_of(_ep, struct sunxi_ep, ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->bEndpointAddress != desc->bEndpointAddress
	    || ep_maxpacket(ep) <
	    le16_to_cpu(get_unaligned(&desc->wMaxPacketSize))) {

		debug("%s: bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {

		debug("%s: %s type mismatch\n", __func__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(get_unaligned(&desc->wMaxPacketSize)) !=
	     ep_maxpacket(ep)) || !get_unaligned(&desc->wMaxPacketSize)) {

		debug("%s: bad %s maxpacket\n", __func__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {

		debug("%s: bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(get_unaligned(&desc->wMaxPacketSize));

	/* Reset halt state */
	sunxi_udc_set_nak(ep);
	sunxi_udc_set_halt(_ep, 0);

	spin_lock_irqsave(&ep->dev->lock, flags);
	sunxi_udc_ep_activate(ep);
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	debug("%s: enabled %s, stopped = %d, maxpacket = %d\n",
	      __func__, _ep->name, ep->stopped, ep->ep.maxpacket);
	return 0;
}

/*
 * Disable EP
 */
static int sunxi_ep_disable(struct usb_ep *_ep)
{
	struct sunxi_ep *ep;
	unsigned long flags;

	debug("%s: %p\n", __func__, _ep);

	ep = container_of(_ep, struct sunxi_ep, ep);
	if (!_ep || !ep->desc) {
		debug("%s: %s not enabled\n", __func__,
		      _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	debug("%s: disabled %s\n", __func__, _ep->name);

	return 0;
}

static struct usb_request *sunxi_alloc_request(struct usb_ep *ep,
					     gfp_t gfp_flags)
{
	struct sunxi_request *req;

	debug("%s: %s %p\n", __func__, ep->name, ep);

	req = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*req));
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void sunxi_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct sunxi_request *req;

	debug("%s: %p\n", __func__, ep);

	req = container_of(_req, struct sunxi_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/* dequeue JUST ONE request */
static int sunxi_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sunxi_ep *ep;
	struct sunxi_request *req;
	unsigned long flags;

	debug("%s: %p\n", __func__, _ep);

	ep = container_of(_ep, struct sunxi_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&ep->dev->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	return 0;
}

/*
 * Return bytes in EP FIFO
 */
static int sunxi_fifo_status(struct usb_ep *_ep)
{
	int count = 0;
	struct sunxi_ep *ep;

	ep = container_of(_ep, struct sunxi_ep, ep);
	if (!_ep) {
		debug("%s: bad ep\n", __func__);
		return -ENODEV;
	}

	debug("%s: %d\n", __func__, ep_index(ep));

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;
	return count;
}

/*
 * Flush EP FIFO
 */
static void sunxi_fifo_flush(struct usb_ep *_ep)
{
	struct sunxi_ep *ep;

	ep = container_of(_ep, struct sunxi_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		debug("%s: bad ep\n", __func__);
		return;
	}

	debug("%s: %d\n", __func__, ep_index(ep));
}

static const struct usb_gadget_ops sunxi_udc_ops = {
	/* current versions must always be self-powered */
};

static struct sunxi_udc memory = {
	.usb_base = SUNXI_USB0_BASE,
	.sram_base = SUNXI_SRAMC_BASE,
	.ccmu_base = SUNXI_CCM_BASE,

	.usb_address = 0,
	.gadget = {
		.ops = &sunxi_udc_ops,
		.ep0 = &memory.ep[0].ep,
		.name = driver_name,
	},

	/* control endpoint */
	.ep[0] = {
		.ep = {
			.name = ep0name,
			.ops = &sunxi_ep_ops,
			.maxpacket = EP0_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 0,
		.bmAttributes = 0,

		.ep_type = ep_control,
	},

	/* first group of endpoints */
	.ep[1] = {
		.ep = {
			.name = "ep1-bulk",
			.ops = &sunxi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 1,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_out,
		.fifo_num = 1,
	},

	.ep[2] = {
		.ep = {
			.name = "ep2-bulk",
			.ops = &sunxi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 2,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_in,
		.fifo_num = 2,
	},

	.ep[3] = {
		.ep = {
			.name = "ep3-bulk",
			.ops = &sunxi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 3,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_in,
		.fifo_num = 3,
	},

	.ep[4] = {
		.ep = {
			.name = "ep4-bulk",
			.ops = &sunxi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 4,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,

		.ep_type = ep_bulk_in,
		.fifo_num = 4,
	},

	.ep[5] = {
		.ep = {
			.name = "ep5-int",
			.ops = &sunxi_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,

		.bEndpointAddress = 5,
		.bmAttributes = USB_ENDPOINT_XFER_INT,

		.ep_type = ep_interrupt,
		.fifo_num = 5,
	},
};

/*
 *	probe - binds to the platform device
 */

int sunxi_udc_probe()
{
	struct sunxi_udc *dev = &memory;
	int retval = 0, i;

	debug("%s: %p\n", __func__, dev);

	dev->gadget.is_dualspeed = 1;	/* Hack only*/
	dev->gadget.is_otg = 0;
	dev->gadget.is_a_peripheral = 0;
	dev->gadget.b_hnp_enable = 0;
	dev->gadget.a_hnp_support = 0;
	dev->gadget.a_alt_hnp_support = 0;

	the_controller = dev;

	for (i = 0; i < SUNXI_MAX_ENDPOINTS+1; i++) {
		dev->dma_buf[i] = memalign(CONFIG_SYS_CACHELINE_SIZE,
					   DMA_BUFFER_SIZE);
		dev->dma_addr[i] = (dma_addr_t) dev->dma_buf[i];
		invalidate_dcache_range((unsigned long) dev->dma_buf[i],
					(unsigned long) (dev->dma_buf[i]
							 + DMA_BUFFER_SIZE));
	}
	usb_ctrl = dev->dma_buf[0];
	usb_ctrl_dma_addr = dev->dma_addr[0];

	udc_reinit(dev);

	return retval;
}

int usb_gadget_init_udc(void)
{
        return sunxi_udc_probe();
}

void usb_gadget_exit_udc(void)
{
}

int usb_gadget_handle_interrupts()
{
	return sunxi_udc_irq(1, (void *)the_controller);
}
