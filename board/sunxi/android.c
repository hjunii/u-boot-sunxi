#include <usb/fastboot.h>
#include <mmc.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

struct bootloader_message {
	char command[32];
	char status[32];
	char recovery[1024];
};

int android_boot_check(void)
{
	int status = 0;
	struct fastboot_ptentry *pte;
	char buffer[2048];
	static struct bootloader_message misc_message;
	struct mmc* mmc = NULL;

	pte = fastboot_flash_find_ptn("misc");
	if (!pte)
	{
		printf("no misc partition is found\n");
		return -1;
	}

	mmc = find_mmc_device(0);
	if (mmc == NULL) {
		printf("no mmc0\n");
		return status;
	}

	status = mmc_init(mmc);
	if (status != 0) {
		printf("mmc init failed\n");
		return status;
	}

	memset(buffer, 0, 2048);

	status = mmc->block_dev.block_read(0, pte->start, 2048 / 512, (void *)buffer);
	if (status < 0) {
		printf("mmc read failed\n");
		return status;
	}

	memcpy(&misc_message, buffer, sizeof(misc_message));

	DBG("misc.command  : %s\n", misc_message.command);
	DBG("misc.status   : %s\n", misc_message.status);
	DBG("misc.recovery : %s\n", misc_message.recovery);

	if (!strcmp(misc_message.command, "recovery"))
	{
		/* there is a recovery command */
		printf("find boot recovery\n");
		setenv("bootcmd", "booti mmc0 recovery");
		printf("Recovery detected, will boot recovery\n");
		/* android recovery will clean the misc */
	}
	else 
	{
		setenv("bootcmd", "booti mmc0");
	}

	if (!strcmp(misc_message.command, "bootloader"))
	{
		/* there is a fastboot command */
		setenv("bootcmd", "fastboot");
		printf("Fastboot detected, will enter fastboot\n");
		/* clean the misc partition ourself */
		memset(buffer, 0, 2048);

		status = mmc->block_dev.block_write(0, pte->start, 2048 / 512, (void *) buffer);
		if (status < 0) {
				printf("mmc write failed\n");
				return status;
		}
	}
}
