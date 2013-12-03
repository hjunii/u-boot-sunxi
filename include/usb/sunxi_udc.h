/*
 * drivers/usb/gadget/sunxi_udc.h
 * Samsung S3C on-chip full/high speed USB device controllers
 * Copyright (C) 2005 for Samsung Electronics
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SUNXI_USB_GADGET
#define __SUNXI_USB_GADGET

#include <asm/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/list.h>
#include <usb/lin_gadget_compat.h>

#define PHY0_SLEEP              (1 << 5)

/*-------------------------------------------------------------------------*/
/* DMA bounce buffer size, 16K is enough even for mass storage */
#define DMA_BUFFER_SIZE	(4096*4)

#define EP0_FIFO_SIZE		64
#define EP_FIFO_SIZE		512
#define EP_FIFO_SIZE2		1024
/* ep0-control, ep1in-bulk, ep2out-bulk, ep3in-int */
#define SUNXI_MAX_ENDPOINTS	6
#define SUNXI_MAX_HW_ENDPOINTS	16

#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4
#define WAIT_FOR_COMPLETE	5
#define WAIT_FOR_OUT_COMPLETE	6
#define WAIT_FOR_IN_COMPLETE	7
#define WAIT_FOR_NULL_COMPLETE	8

#define TEST_J_SEL		0x1
#define TEST_K_SEL		0x2
#define TEST_SE0_NAK_SEL	0x3
#define TEST_PACKET_SEL		0x4
#define TEST_FORCE_ENABLE_SEL	0x5

/* ************************************************************************* */
/* IO
 */

enum ep_type {
	ep_control, ep_bulk_in, ep_bulk_out, ep_interrupt
};

struct sunxi_ep {
	struct usb_ep ep;
	struct sunxi_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;
	int len;
	void *dma_buf;

	u8 stopped;
	u8 bEndpointAddress;
	u8 bmAttributes;

	enum ep_type ep_type;
	int fifo_num;
};

struct sunxi_request {
	struct usb_request req;
	struct list_head queue;
};

struct sunxi_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;

	struct sunxi_plat_otg_data *pdata;

	void *dma_buf[SUNXI_MAX_ENDPOINTS+1];
	dma_addr_t dma_addr[SUNXI_MAX_ENDPOINTS+1];

	int ep0state;
	struct sunxi_ep ep[SUNXI_MAX_ENDPOINTS];

	unsigned char usb_address;

	unsigned req_pending:1, req_std:1;
};

extern struct sunxi_udc *the_controller;

#define ep_is_in(EP) (((EP)->bEndpointAddress&USB_DIR_IN) == USB_DIR_IN)
#define ep_index(EP) ((EP)->bEndpointAddress&0xF)
#define ep_maxpacket(EP) ((EP)->ep.maxpacket)

extern void otg_phy_init(struct sunxi_udc *dev);
extern void otg_phy_off(struct sunxi_udc *dev);

extern void sunxi_udc_ep_set_stall(struct sunxi_ep *ep);
extern int sunxi_udc_probe(struct sunxi_plat_otg_data *pdata);

struct sunxi_plat_otg_data {
	int		(*phy_control)(int on);
	unsigned int	regs_phy;
	unsigned int	regs_otg;
	unsigned int    usb_phy_ctrl;
	unsigned int    usb_flags;
};
#endif
