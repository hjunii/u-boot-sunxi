#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "misc.h"

static int sunxi_gadget_get_frame(struct usb_gadget *g)
{
}

static int sunxi_gadget_wakeup(struct usb_gadget *g)
{
}

static int sunxi_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
}

static int sunxi_gadget_pullup(struct usb_gadget *g, int is_on)
{
}

static const struct usb_gadget_ops sunxi_gadget_ops = {
	.get_frame              = sunxi_gadget_get_frame,
	.wakeup                 = sunxi_gadget_wakeup,
	.set_selfpowered        = sunxi_gadget_set_selfpowered,
	.pullup                 = sunxi_gadget_pullup,


};

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		                int (*bind)(struct usb_gadget *))
{
	int ret;
	ret = bind(&global_sunxi->gadget);
	if (ret)
		return ret;

	ret = usb_gadget_connect(&global_sunxi->gadget);
	if (ret)
		return ret;

	return 0;
}

static int __devinit sunxi_gadget_init_endpoints(struct sunxi *sunxi)
{
}

static void sunxi_gadget_free_endpoints(struct sunxi *sunxi)
{
}

static struct sunxi      *the_sunxi;

int __devinit sunxi_gadget_init(struct sunxi *sunxi)
{
	int                                     ret;

	dev_set_name(&sunxi->gadget.dev, "gadget");

	sunxi->gadget.ops                 = &sunxi_gadget_ops;
	sunxi->gadget.is_dualspeed        = true;
	sunxi->gadget.speed               = USB_SPEED_UNKNOWN;

	sunxi->gadget.name                = "sunxi-gadget";

	the_sunxi = sunxi;

	ret = sunxi_gadget_init_endpoints(sunxi);

	return ret;
}

void sunxi_gadget_exit(struct sunxi *sunxi)
{
	sunxi_gadget_free_endpoints(sunxi);

	device_unregister(&sunxi->gadget.dev);

	the_sunxi = NULL;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	return usb_gadget_probe_driver(driver, driver->bind);
}

/**
 * usb_gadget_unregister_driver - unregisters a gadget driver.
 * @driver: the gadget driver to unregister
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct sunxi             *sunxi = the_sunxi;

	if (!driver || !driver->unbind)
		return -EINVAL;

	if (!sunxi)
		return -ENODEV;

	if (sunxi->gadget_driver != driver)
		return -EINVAL;

	driver->disconnect(&sunxi->gadget);
	driver->unbind(&sunxi->gadget);


	sunxi->gadget_driver      = NULL;


	return 0;
}
