#include <common.h>
#include <usb/fastboot.h>
#include <asm/io.h>

#define DEVICE_SERIAL_NUMBER		"20080411"

char serialno[100];

/* To support the Android-style naming of flash */
#define MAX_PTN 16

static fastboot_ptentry ptable[MAX_PTN];
static unsigned int pcount = 0;

static struct usb_string def_usb_fb_strings[] = {
	{ FB_STR_SERIAL_IDX,            &serialno },
	{  }
};

static struct usb_gadget_strings def_fb_strings = {
	.language       = 0x0409, /* en-us */
	.strings        = def_usb_fb_strings,
};

/*
 * Hardcoded memory region to stash data which comes over USB before it is
 * stored on media
 */
DECLARE_GLOBAL_DATA_PTR;
#define SZ_16M                          0x01000000
#define SZ_128M                         0x10000000
#define CFG_FASTBOOT_TRANSFER_BUFFER (void *)(gd->bd->bi_dram[0].start + SZ_16M)
#define CFG_FASTBOOT_TRANSFER_BUFFER_SIZE (SZ_128M - SZ_16M)

int fastboot_board_init(struct fastboot_config *interface,
		struct usb_gadget_strings **str) {

	interface->transfer_buffer = CFG_FASTBOOT_TRANSFER_BUFFER;
	interface->transfer_buffer_size = CFG_FASTBOOT_TRANSFER_BUFFER_SIZE;

	/* Determine the serial number */
	printf("Device Serial Number: %08X%08X\n", DEVICE_SERIAL_NUMBER);
	sprintf(serialno, "%s", DEVICE_SERIAL_NUMBER);

	*str = &def_fb_strings;
    
	board_mmc_ftbtptn_init();
	return 0;
}

fastboot_ptentry *fastboot_flash_find_ptn(const char *name)
{
	unsigned int n;

	for (n = 0; n < MAX_PTN; n++) {
		/* Make sure a substring is not accepted */
		if (strlen(name) == strlen(ptable[n].name)) {
			if (0 == strcmp(ptable[n].name, name))
				return ptable + n;
		}
	}

	return NULL;
}
void fastboot_flash_reset_ptn(void)
{
	pcount = 0;
}

void fastboot_flash_add_ptn(fastboot_ptentry *ptn, int count)
{
	if(pcount < MAX_PTN) {
	    memcpy(ptable + pcount, ptn, sizeof(*ptn));
	    pcount++;
	}
}


