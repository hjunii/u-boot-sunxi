/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <common.h>
#include <mmc.h>
#include <malloc.h>
#include <usb/fastboot.h>

#define MMCSD_SECTOR_SIZE 512

#define SUN4I_MBR_MAGIC "softw311"
#define SUN4I_MBR_VERSION 0x100

#define SUN4I_ENTRIES 15

#define SUN4I_MBR_COPY_NUM 4
#define SUN4I_MBR_SIZE 1024
#define SUN4I_MBR_RESERVED (SUN4I_MBR_SIZE - 20 - (SUN4I_ENTRIES * 64))

#define MMC_DEVICE 0

#define DEBUG
#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

static int load_ptbl(void);
int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]);

struct sun4i_entry {
	u32 addrhi;
	u32 addrlo;
	u32 lenhi;
	u32 lenlo;
	u8 classname[12];
	u8 name[12];
	unsigned int user_type;
	unsigned int ro;
	u8 res[16];
};

struct sun4i_mbr {
	u32 crc32;
	u32 version;
	u8 magic[8];
	u8 copy;
	u8 index;
	u16 entries_count;
	struct sun4i_entry entry[SUN4I_ENTRIES];
	u8 res[SUN4I_MBR_RESERVED];
};

static void start_ptbl(struct sun4i_mbr *mbr)
{
	DBG("start_ptbl\n");

	memset(mbr, 0, sizeof(*mbr));

	memcpy(mbr->magic, SUN4I_MBR_MAGIC, 8);
	mbr->version = SUN4I_MBR_VERSION;

	DBG("magic		= %s \n",  mbr->magic);
	DBG("version		= %u \n",  mbr->version);
}

static void end_ptbl(struct sun4i_mbr *mbr, u32 count)
{
	mbr->entries_count = count - 1;
}

static int add_ptn(struct sun4i_mbr *mbr, u64 first, u64 len, const char *name)
{
	struct sun4i_entry *entry = mbr->entry;

	u32 n; int i = 0;

	if (first < 34) {
		printf("partition '%s' overlaps partition table\n", name);
		return -1;
	}

	for (n = 0; n < SUN4I_ENTRIES; n++, entry++) {
		if (entry->lenlo)
			continue;
		memcpy(entry->name, name, 12);
		strcat(entry->classname, "DISK");
		memset(entry->res, 0, sizeof(mbr->res));
		entry->user_type = 0;
		entry->ro = 0;
		entry->addrhi = 0;
		entry->lenhi = 0;
		entry->addrlo = first;
		entry->lenlo = len;

		return 0;
	}

	return -1;
}

static void import_sun4i_partition(struct sun4i_entry *entry, int count)
{
	struct fastboot_ptentry e;
	int n;

	if (entry->lenlo == 0)
		return;

	for (n = 0; n < (sizeof(entry->name)-1); n++) {
		e.name[n] = entry->name[n];
	}
	e.name[n] = 0;

	e.start = entry->addrlo;
	e.length = ((unsigned long long) entry->lenlo) * MMCSD_SECTOR_SIZE;
	e.flags = 0;

	if (!strcmp(e.name, "environment"))
		e.flags |= FASTBOOT_PTENTRY_FLAGS_WRITE_ENV;
	
	fastboot_flash_add_ptn(&e, count);

#ifdef DEBUG
	if (e.length > 0x100000)
		DBG("%8d %7lluM %s\n", e.start,
				(u64)(e.length/0x100000), e.name);
	else
		DBG("%8d %7lluK %s\n", e.start,
				(u64)(e.length/0x400), e.name);
#endif

}

static int load_ptbl(void)
{
	u64 sun4i_mbr_sectors = 0;
	int i = 0, r = 0;

	struct sun4i_mbr *mbr;
	struct mmc* mmc = NULL;

	mmc = find_mmc_device(MMC_DEVICE);
	if (mmc == NULL) {
		printf("No MMC in slot 1\n");
		return -1;
	}
	mmc->has_init = 0;
	mmc_init(mmc);
	if (r!= 0) {
		printf("mmc init failed\n");
		return r;
	}

	int sun4i_mbr_size = sizeof(struct sun4i_mbr);

	mbr = (struct ptable *) malloc(sun4i_mbr_size);
	if (!mbr) {
		r = -1;
		goto fail;
	}

	sun4i_mbr_sectors = (u64)(sun4i_mbr_size / MMCSD_SECTOR_SIZE);

	r = mmc->block_dev.block_read(MMC_DEVICE, 0, sun4i_mbr_sectors,( void*)mbr);
	if (r == -1) {
		printf("error reading GPT\n");
		goto fail;
	}

	if (memcmp(mbr->magic, SUN4I_MBR_MAGIC, 8)) {
		printf("sun4i partition table not found\n");
		r = -1;
		goto fail;
	}

	for (i = 0; i < SUN4I_ENTRIES; i++)
	{
		if (i == mbr->entries_count - 1)
			mbr->entry[i].lenlo = mmc->block_dev.lba - mbr->entry[i].addrlo;
		import_sun4i_partition(&mbr->entry[i], i);
	}

fail:
	free((void *)mbr);
	return r;
}

static u64 get_entry_size_kb(struct sun4i_entry *entry, const char *ptn)
{
	int ret = 0;
	u64 sz = 0;

	if (!strcmp(entry->name, ptn))
		sz = entry->lenlo/2;

	return sz;
}

char *get_ptn_size(char *buf, const char *ptn)
{
	u64 sun4i_mbr_sectors = 0;
	int i = 0, r = 0;
	u32 sz_mb;
	u64 sz = 0;

	struct sun4i_mbr *mbr;
	struct mmc* mmc = NULL;

	mmc = find_mmc_device(MMC_DEVICE);
	if (mmc == NULL) {
		printf("No MMC in slot 1\n");
		return NULL;
	}
	mmc->has_init = 0;
	mmc_init(mmc);
	if (r!= 0) {
		printf("mmc init failed\n");
		return NULL;
	}

	int sun4i_mbr_size = sizeof(struct sun4i_mbr);

	mbr =  (struct sun4i_mbr *) malloc(sun4i_mbr_size);
	if (!mbr) {
		r = -1;
		goto fail;
	}

	sun4i_mbr_sectors = (u64)(sun4i_mbr_size / MMCSD_SECTOR_SIZE);

	r = mmc->block_dev.block_read(MMC_DEVICE,0,sun4i_mbr_sectors,(void*)mbr);
	if (r == -1) {
		printf("error reading MBR\n");
		goto fail;
	}

	if (memcmp(mbr->magic, SUN4I_MBR_MAGIC, 8)) {
		printf("sun4i partition table not found\n");
		r = -1;
		goto fail;
	}

	for (i = 0; i < SUN4I_ENTRIES; i++) {
		sz = get_entry_size_kb(&mbr->entry[i], ptn);
		if (sz)
			break;
	}

	if (sz >= 0xFFFFFFFF) {
		sz_mb = (u32)(sz >> 20);
		DBG("sz is > 0xFFFFFFFF\n");
		sprintf(buf, "0x%d MB", sz_mb);
	} else {
		DBG("Size of the partition = %d KB\n", (u32)sz);
		sprintf(buf, "%d KB", (u32)sz);
	}

fail:
	free((void *)mbr);
	return buf;
}

struct _partition {
	const char *name;
	unsigned size_kb;
};



static struct _partition partitions[] = {
	{ "-", 300 },
	{ "environment", 44 },
	{ "boot", 36 * 1024 },
	{ "system", 500 * 1024 },
	{ "data", 300 * 1024 },
	{ "misc", 16 * 1024 },
	{ "recovery", 36 * 1024 },
	{ "cache", 125 * 1024 },
	{ "private", 16 * 1024 },
	{ "userdata", 0},
	{ NULL, 0 },
};


static int do_format(void)
{
	struct sun4i_mbr *mbr;
	unsigned blocks;
	unsigned next;
	int n;
	struct mmc* mmc = NULL;
	int status = 0;
	char *mmc_write[5]	= {"mmc", "write", NULL, NULL, NULL};

	char *dev[3] = { "mmc", "dev", MMC_DEVICE };
	char source[32], dest[32], length[32];

	mmc = find_mmc_device(MMC_DEVICE);
	if (mmc == NULL) {
		return -1;
	}
	mmc->has_init = 0;
	status = mmc_init(mmc);
	if (status != 0) {
		printf("mmc init failed\n");
		return status;
	}

	status = do_mmcops(NULL, 0, 3, dev);
	if (status) {
		printf("Unable to set MMC device\n");
		return status;
	}

	blocks = mmc->block_dev.lba;

	mbr = (struct sun4i_mbr *) malloc(sizeof(struct sun4i_mbr));

	start_ptbl(mbr);
	n = 0;
	next = 0;
	for (n = 0, next = 0; partitions[n].name; n++) {
		unsigned sz = partitions[n].size_kb * 2;
		if (!strcmp(partitions[n].name,"-")) {
			next += sz;
			continue;
		}
		if (sz == 0)
			sz = blocks - next;

	    if (add_ptn(mbr, next, sz, partitions[n].name)) {
	        printf("Add partition failed\n");
			status = -1;
	        goto fail;
		}
		next += sz;
	}
	end_ptbl(mbr, n);
	fastboot_flash_reset_ptn();
	mmc_write[2] = source;
	mmc_write[3] = dest;
	mmc_write[4] = length;

	sprintf(source, "0x%x", (unsigned int)mbr);
	sprintf(length, "0x%x", (unsigned int)
			(sizeof(struct sun4i_mbr)/mmc->block_dev.blksz));

	for (n = 0; n < SUN4I_MBR_COPY_NUM; ++n)
	{
		mbr->index = n;
		mbr->crc32 = crc32(0, (u32 *)mbr + 1,SUN4I_MBR_SIZE - 4);
		sprintf(dest, "0x%x", SUN4I_MBR_SIZE / mmc->block_dev.blksz * n);

		if (do_mmcops(NULL, 0, 5, mmc_write)) {
			printf("Writing mbr is FAILED!\n");
			goto fail;
		}
	}
	printf("Writing mbr is DONE!\n");

	load_ptbl();

fail:
	free(mbr);
	return status;
}


int fastboot_oem(const char *cmd)
{
	if (!strcmp(cmd,"format")) {
		return do_format();
	}
	return -1;
}

int board_mmc_ftbtptn_init(void)
{
	return load_ptbl();
}

int board_late_init(void)
{
	return load_ptbl();
}

