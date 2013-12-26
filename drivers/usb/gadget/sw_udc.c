#include <common.h>

#include "sw_udc.h"
#include "regs-otg-sunxi.h"

#include <asm/io.h>

#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>

#define DRIVER_DESC     "SoftWinner USB Device Controller"
#define DRIVER_VERSION  "20080411"
#define DRIVER_AUTHOR   "SoftWinner USB Developer"

static const char               gadget_name[] = "sw_usb_udc";
static const char               driver_desc[] = DRIVER_DESC;

static struct sw_udc *the_controller = NULL;
static u32 usbd_port_no = 0;
static sw_udc_io_t g_sw_udc_io;
static u32 usb_connect = 0;
static u32 is_controller_alive = 0;
static u8 is_udc_enable = 0; /* is udc enable by gadget? */

static void cfg_udc_command(enum sw_udc_cmd_e cmd);

static __u32 is_peripheral_active(void)
{
	return is_controller_alive;
}

static void clear_all_irq(void)
{
	//USBC_INT_ClearEpPendingAll(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX);
	//__USBC_INT_ClearTxPendingAll(usbc_otg->base_addr);
	writew(0xffff, g_sw_udc_io.usb_vbase + SUNXI_INTTx);
	//USBC_INT_ClearEpPendingAll(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX);
	//__USBC_INT_ClearRxPendingAll(usbc_otg->base_addr);
	writew(0xffff, g_sw_udc_io.usb_vbase + SUNXI_INTRx);
	//USBC_INT_ClearMiscPendingAll(g_sw_udc_io.usb_bsp_hdle);
	writeb(0xff, g_sw_udc_io.usb_vbase + SUNXI_INTUSB);
}

static int sw_udc_set_halt(struct usb_ep *_ep, int value);

static inline struct sw_udc_ep *to_sw_udc_ep(struct usb_ep *ep)
{
	return container_of(ep, struct sw_udc_ep, ep);
}

static inline struct sw_udc *to_sw_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct sw_udc, gadget);
}

static inline struct sw_udc_request *to_sw_udc_req(struct usb_request *req)
{
	return container_of(req, struct sw_udc_request, req);
}

static int sw_udc_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct sw_udc *dev = NULL;
	struct sw_udc_ep *ep = NULL;
	u32 max	= 0;
	unsigned long flags = 0;
	u32 old_ep_index = 0;
	__u32 fifo_addr = 0;
	u32 reg;

	if (_ep == NULL || desc == NULL) {
		printf("ERR: invalid argment\n");
		return -EINVAL;
	}

	if (_ep->name == ep0name || desc->bDescriptorType != USB_DT_ENDPOINT) {
		printf("PANIC : _ep->name(%s) == ep0name || desc->bDescriptorType(%d) != USB_DT_ENDPOINT\n", _ep->name , desc->bDescriptorType);
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if (ep == NULL) {
		printf("ERR: usbd_ep_enable, ep = NULL\n");
		return -EINVAL;
	}

	if (ep->desc) {
		printf("ERR: usbd_ep_enable, ep->desc is not NULL, ep%d(%s)\n", ep->num, _ep->name);
		return -EINVAL;
	}

	printf("ep enable: ep%d(0x%p, %s, %d, %d)\n",
			ep->num, _ep, _ep->name,
			(desc->bEndpointAddress & USB_DIR_IN), _ep->maxpacket);

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		printf("PANIC : dev->driver = 0x%p ?= NULL  dev->gadget->speed =%d ?= USB_SPEED_UNKNOWN\n", dev->driver ,dev->gadget.speed);
		return -ESHUTDOWN;
	}

	max = le16_to_cpu(desc->wMaxPacketSize) & 0x1fff;

	spin_lock_irqsave(&ep->dev->lock, flags);

	_ep->maxpacket          = max & 0x7ff;
	ep->desc                = desc;
	ep->halted              = 0;
	ep->bEndpointAddress    = desc->bEndpointAddress;

	fifo_addr = ep->num * 1024;

	if(!is_peripheral_active()){
		printf("ERR: usb device is not active\n");
		goto end;
	}

	//old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_bsp_hdle);
	old_ep_index = readb(g_sw_udc_io.usb_vbase + SUNXI_EPIND);
	//USBC_SelectActiveEp(g_sw_udc_io.usb_bsp_hdle, ep->num);
	writeb(ep->num, g_sw_udc_io.usb_vbase + SUNXI_EPIND);

	if ((ep->bEndpointAddress) & USB_DIR_IN)
	{ /* tx */
		//USBC_Dev_ConfigEp(g_sw_udc_io.usb_bsp_hdle, USBC_TS_TYPE_BULK, USBC_EP_TYPE_TX, SW_UDC_FIFO_NUM, _ep->maxpacket & 0x7ff);
		// __USBC_Dev_Tx_ConfigEp(usbc_otg->base_addr, ts_type, is_double_fifo,  ep_MaxPkt)
		// --<1>--config tx csr
		reg = (1 << SUNXI_BP_TXCSR_D_MODE);
		reg |= (1 << SUNXI_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
		reg |= (1 << SUNXI_BP_TXCSR_D_FLUSH_FIFO);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_TXCSR);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_TXCSR);
		// --<2>--config tx ep max packet
		reg = readw(g_sw_udc_io.usb_vbase + SUNXI_TXMAXP);
		reg |= (_ep->maxpacket & 0x7ff) & ((1 << SUNXI_BP_TXMAXP_PACKET_COUNT) - 1);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_TXMAXP);
		// --<3>--config ep transfer type
		// __USBC_Dev_Tx_EnableBulkEp(usbc_base_addr);
		clrbits_le16(g_sw_udc_io.usb_vbase + SUNXI_TXCSR, 1 << SUNXI_BP_TXCSR_D_ISO);
		//USBC_ConfigFifo(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX, SW_UDC_FIFO_NUM, _ep->maxpacket, fifo_addr);
		// __USBC_ConfigFifo_TxEp(usbc_otg->base_addr, is_double_fifo, fifo_size, fifo_addr);
		u32 temp = 0;
		u32 size = 0;
		temp = _ep->maxpacket + 511;
		temp &= ~511;
		temp >>= 3;
		temp >>= 1;
		while (temp) {
			size++;
			temp >>= 1;
		}
		writew(fifo_addr >> 3, g_sw_udc_io.usb_vbase + SUNXI_TXFIFOAD);
		writeb(size & 0x0f, g_sw_udc_io.usb_vbase + SUNXI_TXFIFOSZ);
		setbits_8(g_sw_udc_io.usb_vbase + SUNXI_TXFIFOSZ, 1 << SUNXI_BP_TXFIFOSZ_DPB);
		//USBC_INT_EnableEp(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX, ep->num);
		// __USBC_INT_EnableTxEp(usbc_otg->base_addr, ep_index);
		setbits_le16(g_sw_udc_io.usb_vbase + SUNXI_INTTxE, 1 << ep->num);
	} 
	else
	{ /* rx */
		//USBC_Dev_ConfigEp(g_sw_udc_io.usb_bsp_hdle, USBC_TS_TYPE_BULK, USBC_EP_TYPE_RX, SW_UDC_FIFO_NUM, _ep->maxpacket & 0x7ff);
		// __USBC_Dev_Rx_ConfigEp(usbc_otg->base_addr, ts_type, is_double_fifo, ep_MaxPkt);
		// --<1>--config tx csr
		reg = (1 << SUNXI_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << SUNXI_BP_RXCSR_D_FLUSH_FIFO);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_RXCSR);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_RXCSR);
		// --<2>--config tx ep max packet
		reg = readw(g_sw_udc_io.usb_vbase + SUNXI_RXMAXP);
		reg |= (_ep->maxpacket & 0x7ff) & ((1 << SUNXI_BP_RXMAXP_PACKET_COUNT) - 1);
		writew(reg, g_sw_udc_io.usb_vbase + SUNXI_RXMAXP);
		// __USBC_Dev_Rx_EnableBulkEp(usbc_base_addr);
		clrbits_le16(g_sw_udc_io.usb_vbase + SUNXI_RXCSR, 1 << SUNXI_BP_RXCSR_D_ISO);
		//USBC_ConfigFifo(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX, SW_UDC_FIFO_NUM, 512, fifo_addr);
		//__USBC_ConfigFifo_RxEp(usbc_otg->base_addr, is_double_fifo, fifo_size, fifo_addr);
		u32 temp = 0;
		u32 size = 0;
		temp = 512 + 511;
		temp &= ~511;
		temp >>= 3;
		temp >>= 1;
		while (temp) {
			size++;
			temp >>= 1;
		}
		writew(fifo_addr >> 3, g_sw_udc_io.usb_vbase + SUNXI_RXFIFOAD);
		writeb(size & 0x0f, g_sw_udc_io.usb_vbase + SUNXI_RXFIFOSZ);
		setbits_8(g_sw_udc_io.usb_vbase + SUNXI_RXFIFOSZ, 1 << SUNXI_BP_RXFIFOSZ_DPB);
		//USBC_INT_EnableEp(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX, ep->num);
		// __USBC_INT_EnableRxEp(usbc_otg->base_addr, ep_index);
		setbits_le16(g_sw_udc_io.usb_vbase + SUNXI_INTRxE, 1 << ep->num);
	}

	//USBC_SelectActiveEp(g_sw_udc_io.usb_bsp_hdle, old_ep_index);
	writeb(old_ep_index, g_sw_udc_io.usb_vbase + SUNXI_EPIND);

end:
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	sw_udc_set_halt(_ep, 0);

	return 0;
}

static int sw_udc_ep_disable(struct usb_ep *_ep)
{
#if 0
	struct sw_udc_ep   	*ep     = NULL;
    u32 old_ep_index            = 0;
	unsigned long flags = 0;

	if (!_ep) {
		DMSG_PANIC("ERR: invalid argment\n");
		return -EINVAL;
	}

	ep = to_sw_udc_ep(_ep);
	if(ep == NULL){
		DMSG_PANIC("ERR: usbd_ep_disable: ep = NULL\n");
		return -EINVAL;
	}

	if (!ep->desc) {
		DMSG_PANIC("ERR: %s not enabled\n", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	DMSG_INFO_UDC("ep disable: ep%d(0x%p, %s, %d, %x)\n",
		          ep->num, _ep, _ep->name,
		          (ep->bEndpointAddress & USB_DIR_IN), _ep->maxpacket);

	spin_lock_irqsave(&ep->dev->lock, flags);

	DMSG_DBG_UDC("ep_disable: %s\n", _ep->name);

	ep->desc = NULL;
	ep->halted = 1;

	sw_udc_nuke (ep->dev, ep, -ESHUTDOWN);

	if(!is_peripheral_active()){
		DMSG_PANIC("ERR: usb device is not active\n");
		goto end;
	}

	old_ep_index = USBC_GetActiveEp(g_sw_udc_io.usb_bsp_hdle);
    USBC_SelectActiveEp(g_sw_udc_io.usb_bsp_hdle, ep->num);

	if ((ep->bEndpointAddress) & USB_DIR_IN){ /* tx */
	    USBC_Dev_ConfigEp_Default(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX);
		USBC_INT_DisableEp(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX, ep->num);
	}else{ /* rx */
	    USBC_Dev_ConfigEp_Default(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX);
		USBC_INT_DisableEp(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX, ep->num);
	}

	USBC_SelectActiveEp(g_sw_udc_io.usb_bsp_hdle, old_ep_index);

end:
	spin_unlock_irqrestore(&ep->dev->lock, flags);

	DMSG_DBG_UDC("%s disabled\n", _ep->name);
#endif

	return 0;
}

static struct usb_request * sw_udc_alloc_request(struct usb_ep *_ep, gfp_t mem_flags)
{
	struct sw_udc_request *req = NULL;

	if(!_ep){
		printf("ERR: invalid argment\n");
		return NULL;
	}

	req = kzalloc (sizeof(struct sw_udc_request), mem_flags);
	if(!req){
		printf("ERR: kzalloc failed\n");
		return NULL;
	}

	memset(req, 0, sizeof(struct sw_udc_request));

	INIT_LIST_HEAD (&req->queue);

	printf("alloc request: ep(0x%p, %s, %d), req(0x%p)\n",
			_ep, _ep->name, _ep->maxpacket, req);

	return &req->req;
}

static void sw_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sw_udc_request   *req = NULL;

	if(_ep == NULL || _req == NULL){
		printf("ERR: invalid argment\n");
		return;
	}

	req = to_sw_udc_req(_req);
	if(req == NULL){
		printf("ERR: invalid argment\n");
		return;
	}

	printf("free request: ep(0x%p, %s, %d), req(0x%p)\n",
			_ep, _ep->name, _ep->maxpacket, req);

	kfree(req);

	return;
}

static int sw_udc_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	return 0;
}

static int sw_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	return 0;
}

static int sw_udc_set_halt(struct usb_ep *_ep, int value)
{
	return 0;
}

static const struct usb_ep_ops sw_udc_ep_ops = {
	.enable                 = sw_udc_ep_enable,
	.disable                = sw_udc_ep_disable,

	.alloc_request  	= sw_udc_alloc_request,
	.free_request   	= sw_udc_free_request,

	.queue                  = sw_udc_queue,
	.dequeue                = sw_udc_dequeue,

	.set_halt               = sw_udc_set_halt,
};

static int sw_udc_get_frame(struct usb_gadget *_gadget)
{
	return 0;
}

static int sw_udc_wakeup(struct usb_gadget *_gadget)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	return 0;
}

static int sw_udc_set_selfpowered(struct usb_gadget *gadget, int value)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	return 0;
}

static void sw_udc_disable(struct sw_udc *dev);
static void sw_udc_enable(struct sw_udc *dev);

static int sw_udc_set_pullup(struct sw_udc *udc, int is_on)
{
	printf("sw_udc_set_pullup\n");

	is_udc_enable = is_on;
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}


	if (is_on) {
		sw_udc_enable(udc);
	} else {
		if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
			if (udc->driver && udc->driver->disconnect)
				udc->driver->disconnect(&udc->gadget);
		}

		sw_udc_disable(udc);
	}

	return 0;
}

static int sw_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct sw_udc *udc = to_sw_udc(gadget);

	printf("sw_udc_vbus_session\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	udc->vbus = (is_active != 0);
	sw_udc_set_pullup(udc, is_active);

	return 0;
}

static int sw_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct sw_udc *udc = to_sw_udc(gadget);

	printf("sw_udc_pullup, is_on = %d\n", is_on);

	sw_udc_set_pullup(udc, is_on);

	return 0;
}

static int sw_udc_vbus_draw(struct usb_gadget *_gadget, unsigned ma)
{
	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	printf("sw_udc_vbus_draw\n");

	return 0;
}

static void sw_udc_reinit(struct sw_udc *dev)
{
	u32 i = 0;

	/* device/ep0 records init */
	INIT_LIST_HEAD (&dev->gadget.ep_list);
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	for (i = 0; i < SW_UDC_ENDPOINTS; i++) {
		struct sw_udc_ep *ep = &dev->ep[i];

		if (i != 0) {
			list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);
		}

		ep->dev     = dev;
		ep->desc    = NULL;
		ep->halted  = 0;
		INIT_LIST_HEAD (&ep->queue);
	}
}

static void sw_udc_enable(struct sw_udc *dev)
{
	u32 reg;

	printf("sw_udc_enable called\n");

	/* dev->gadget.speed = USB_SPEED_UNKNOWN; */
	dev->gadget.speed = USB_SPEED_UNKNOWN;

	printf("CONFIG_USB_GADGET_DUALSPEED\n");

	// USBC_Dev_ConfigTransferMode(g_sw_udc_io.usb_bsp_hdle, USBC_TS_TYPE_BULK, USBC_TS_MODE_HS);
	// __USBC_Dev_TsType_Bulk(usbc_otg->base_addr)
	clrbits_8(g_sw_udc_io.usb_vbase + SUNXI_PCTL, 0x1 < SUNXI_BP_POWER_D_ISO_UPDATE_EN);
	// __USBC_Dev_TsMode_Hs(usbc_otg->base_addr)
	setbits_8(g_sw_udc_io.usb_vbase + SUNXI_PCTL, 0x1 < SUNXI_BP_POWER_D_HIGH_SPEED_EN);

	/* Enable reset and suspend interrupt interrupts */
	//USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_bsp_hdle, USBC_BP_INTUSB_SUSPEND);
	reg = readb(g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_SUSPEND;
	writeb(reg, g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	//USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_bsp_hdle, USBC_BP_INTUSB_RESUME);
	reg = readb(g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
        reg |= SUNXI_BP_INTUSB_RESUME;
        writeb(reg, g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);	
	//USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_bsp_hdle, USBC_BP_INTUSB_RESET);
	reg = readb(g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_RESET;
	writeb(reg, g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	//USBC_INT_EnableUsbMiscUint(g_sw_udc_io.usb_bsp_hdle, USBC_BP_INTUSB_DISCONNECT);
	reg = readb(g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	reg |= SUNXI_BP_INTUSB_DISCONNECT;
	writeb(reg, g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);

	/* Enable ep0 interrupt */
	//USBC_INT_EnableEp(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX, 0);
	// __USBC_INT_EnableTxEp(usbc_otg->base_addr, ep_index);
	setbits_le16(g_sw_udc_io.usb_vbase + SUNXI_INTTxE, 1 << 0);

	cfg_udc_command(SW_UDC_P_ENABLE);
}

static void sw_udc_disable(struct sw_udc *dev)
{
	printf("sw_udc_disable\n");

	/* Disable all interrupts */
	//USBC_INT_DisableUsbMiscAll(g_sw_udc_io.usb_bsp_hdle);
	writeb(0, g_sw_udc_io.usb_vbase + SUNXI_INTUSBE);
	//USBC_INT_DisableEpAll(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_RX);
	// __USBC_INT_DisableTxAll(usbc_otg->base_addr);
	writew(0, g_sw_udc_io.usb_vbase + SUNXI_INTRxE);
	//USBC_INT_DisableEpAll(g_sw_udc_io.usb_bsp_hdle, USBC_EP_TYPE_TX);
	// __USBC_INT_DisableTxAll(usbc_otg->base_addr);
	writew(0, g_sw_udc_io.usb_vbase + SUNXI_INTTxE);

	/* Clear the interrupt registers */
	clear_all_irq();
	cfg_udc_command(SW_UDC_P_DISABLE);

	/* Set speed to unknown */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
}

s32 usbd_start_work(void)
{
	printf("usbd_start_work\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	//USBC_Dev_ConectSwitch(g_sw_udc_io.usb_bsp_hdle, USBC_DEVICE_SWITCH_ON);
	setbits_8(g_sw_udc_io.usb_vbase + SUNXI_PCTL, 0x1 << SUNXI_BP_POWER_D_SOFT_CONNECT);

	return 0;
}

s32 usbd_stop_work(void)
{
	printf("usbd_stop_work\n");

	if (!is_peripheral_active()) {
		printf("ERR: usb device is not active\n");
		return 0;
	}

	//USBC_Dev_ConectSwitch(g_sw_udc_io.usb_bsp_hdle, USBC_DEVICE_SWITCH_OFF);
	clrbits_8(g_sw_udc_io.usb_vbase + SUNXI_PCTL, 0x1 << SUNXI_BP_POWER_D_SOFT_CONNECT);

	return 0;
}

static const struct usb_gadget_ops sw_udc_ops = {
	.get_frame              = sw_udc_get_frame,
	.wakeup                 = sw_udc_wakeup,
	.set_selfpowered	= sw_udc_set_selfpowered,
	.pullup                 = sw_udc_pullup,
	.vbus_session           = sw_udc_vbus_session,
	.vbus_draw              = sw_udc_vbus_draw,
};

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct sw_udc *dev = the_controller;
	int retval = 0;
	unsigned long flags;

	printf("%s: %s\n", __func__, "no name");

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
		printf("%s: bind to driver --> error %d\n",
				dev->gadget.name, retval);
		dev->driver = 0;
		return retval;
	}


	printf("Registered gadget driver %s\n", dev->gadget.name);
	sw_udc_enable(dev);

	return 0;
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sw_udc *dev = the_controller;
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

	sw_udc_disable(dev);

	return 0;
}

static struct sw_udc sw_udc = {
	.gadget = {
		.ops            = &sw_udc_ops,
		.ep0            = &sw_udc.ep[0].ep,
		.name           = gadget_name,
	},

	/* control endpoint */
	.ep[0] = {
		.num                    = 0,
		.ep = {
			.name           = ep0name,
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = EP0_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
	},

	/* first group of endpoints */
	.ep[1] = {
		.num                    = 1,
		.ep = {
			.name           = "ep1-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 1,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},
	.ep[2] = {
		.num                    = 2,
		.ep = {
			.name           = "ep2-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 2,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[3] = {
		.num                    = 3,
		.ep = {
			.name           = "ep3-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 3,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[4] = {
		.num                    = 4,
		.ep = {
			.name           = "ep4-bulk",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 4,
		.bmAttributes       = USB_ENDPOINT_XFER_BULK,
	},

	.ep[5] = {
		.num                    = 5,
		.ep = {
			.name           = "ep5-int",
			.ops            = &sw_udc_ep_ops,
			.maxpacket      = SW_UDC_EP_FIFO_SIZE,
		},
		.dev                    = &sw_udc,
		.fifo_size              = (SW_UDC_EP_FIFO_SIZE * (SW_UDC_FIFO_NUM + 1)),
		.bEndpointAddress   = 5,
		.bmAttributes       = USB_ENDPOINT_XFER_INT,
	},
};

static __s32 sw_udc_io_init(__u32 usbc_no, sw_udc_io_t *sw_udc_io)
{
	sw_udc_io->usb_vbase  = (void __iomem *) SUNXI_USB0_BASE;
	sw_udc_io->sram_vbase = (void __iomem *) SUNXI_SRAMC_BASE;

	struct sunxi_ccm_reg *ccm = (struct sunxi_ccm_reg *) SUNXI_CCM_BASE;

	// open_usb_clock(udc.ccmu_base);
	setbits_le32(&ccm->ahb_gate0, 0x1 << AHB_GATE_OFFSET_USB);
	sdelay(10000);
	setbits_le32(&ccm->usb_clk_cfg, 0x1 << 8 | 0x1);
	sdelay(10000);

	sw_udc_io->clk_is_open = 1;

	return 0;
}	

int sw_udc_probe(void)
{
	struct sw_udc *udc = &sw_udc;
	int ret = 0;

	memset(&g_sw_udc_io, 0, sizeof(sw_udc_io_t));

	ret = sw_udc_io_init(usbd_port_no, &g_sw_udc_io);
	if (ret != 0)
	{
		printf("ERR: sw_udc_io_init failed\n");
		return -1;
	}

	spin_lock_init (&udc->lock);

	is_controller_alive = 1;
	the_controller = udc;

	sw_udc_disable(udc);
	sw_udc_reinit(udc);

	udc->sw_udc_io = &g_sw_udc_io;
	udc->usbc_no = usbd_port_no;
	strcpy((char *)udc->driver_name, gadget_name);

	return 0;
}

static void cfg_udc_command(enum sw_udc_cmd_e cmd)
{
	struct sw_udc *udc = the_controller;

	switch (cmd)
	{
		case SW_UDC_P_ENABLE:
		{
			if (udc->driver) {
				usbd_start_work();
			} else {
				printf("udc->driver is null, udc is need not start\n");
			}
		}
		break;

		case SW_UDC_P_DISABLE:
		{
			if (udc->driver) {
				usbd_stop_work();
			} else {
				printf("udc->driver is null, udc is need not stop\n");
			}
		}
		break;

		case SW_UDC_P_RESET :
			printf("ERR: reset is not support\n");
			break;

		default:
			printf("ERR: unkown cmd(%d)\n", cmd);
			break;
	}
}

int usb_gadget_init_udc(void)
{
	return sw_udc_probe();
}

void usb_gadget_exit_udc(void)
{
}

int usb_gadget_handle_interrupts()
{
	return 0;
}
