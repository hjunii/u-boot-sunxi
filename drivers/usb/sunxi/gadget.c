#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		                int (*bind)(struct usb_gadget *))
{
	return 0;
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
	//struct dwc3             *dwc = the_dwc;

	if (!driver || !driver->unbind)
		return -EINVAL;

	//if (!dwc)
	//	return -ENODEV;

	//if (dwc->gadget_driver != driver)
	//	return -EINVAL;

	//driver->disconnect(&dwc->gadget);
	//driver->unbind(&dwc->gadget);


	//dwc->gadget_driver      = NULL;


	return 0;
}
