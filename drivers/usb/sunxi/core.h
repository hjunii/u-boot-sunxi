#ifndef __DRIVERS_USB_SUNXI_CORE_H
#define __DRIVERS_USB_SUNXI_CORE_H

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

struct sunxi {

	struct usb_gadget       gadget;
	struct usb_gadget_driver *gadget_driver;
};

#endif /* __DRIVERS_USB_SUNXI_CORE_H */
