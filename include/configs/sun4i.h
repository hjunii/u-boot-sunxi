/*
 * (C) Copyright 2012-2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * Configuration settings for the Allwinner A10 (sun4i) CPU
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * A10 specific configuration
 */
#define CONFIG_SUN4I		/* sun4i SoC generation */

#define CONFIG_SYS_PROMPT		"sun4i# "
#define CONFIG_MACH_TYPE		4104

#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_SUNXI_UDC_OTG
/*#define CONFIG_USB_SUNXI*/
#define CONFIG_CMD_FASTBOOT
#define CONFIG_SYS_CACHELINE_SIZE		64
#define MEMORY_BASE                             0x80000000
#define CONFIG_ADDR_DOWNLOAD			(MEMORY_BASE + 0x02000000)
#define KERNEL_ENTRY                            0x40000000
#define DEVICE_TREE                             0x43000000

#define CONFIG_BOARD_LATE_INIT

/*
 * Include common sunxi configuration where most the settings are
 */
#include <configs/sunxi-common.h>

#endif /* __CONFIG_H */
