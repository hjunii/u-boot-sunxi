/**
 * host.c - DesignWare USB3 DRD Controller Host Glue
 */

#include <linux/platform_device.h>

#include "core.h"

static struct resource generic_resources[] = {
	{
		.flags = IORESOURCE_IRQ,
	},
	{
		.flags = IORESOURCE_MEM,
	},
};

int sunxi_host_init(struct sunxi *sunxi)
{
	struct platform_device	*xhci;
	int			ret;

	xhci = platform_device_alloc("xhci", -1);
	if (!xhci) {
		dev_err(sunxi->dev, "couldn't allocate xHCI device\n");
		ret = -ENOMEM;
		goto err0;
	}

	dma_set_coherent_mask(&xhci->dev, sunxi->dev->coherent_dma_mask);

	xhci->dev.parent	= sunxi->dev;
	xhci->dev.dma_mask	= sunxi->dev->dma_mask;
	xhci->dev.dma_parms	= sunxi->dev->dma_parms;

	sunxi->xhci = xhci;

	/* setup resources */
	generic_resources[0].start = sunxi->irq;

	generic_resources[1].start = sunxi->res->start;
	generic_resources[1].end = sunxi->res->start + 0x7fff;

	ret = platform_device_add_resources(xhci, generic_resources,
			ARRAY_SIZE(generic_resources));
	if (ret) {
		dev_err(sunxi->dev, "couldn't add resources to xHCI device\n");
		goto err1;
	}

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(sunxi->dev, "failed to register xHCI device\n");
		goto err1;
	}

	return 0;

err1:
	platform_device_put(xhci);

err0:
	return ret;
}

void sunxi_host_exit(struct sunxi *sunxi)
{
	platform_device_unregister(sunxi->xhci);
}
