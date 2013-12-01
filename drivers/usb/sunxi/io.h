/**
 * io.h
 */

#ifndef __DRIVERS_USB_SUNXI_IO_H
#define __DRIVERS_USB_SUNXI_IO_H

#include <asm/io.h>

static inline u32 sunxi_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void sunxi_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

#endif /* __DRIVERS_USB_SUNXI_IO_H */
