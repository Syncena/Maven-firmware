/*
 * Copyright (c) 2022, Steve C. Woodford.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#include "flash_nxp_lpc5x.h"
#include "flash_applet.h"
#include "timer.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Driver for the Flash Controller found LPC5X.
 */

/* FMC Registers; offset from ls_regs */
#define	FMC_REG_CMD			0x00u
#define	 FMC_CMD_INIT			0u
#define	 FMC_CMD_POWERDOWN		1u
#define	 FMC_CMD_SET_READ_MODE		2u
#define	 FMC_CMD_READ_SINGLE_WORD	3u
#define	 FMC_CMD_ERASE_RANGE		4u
#define	 FMC_CMD_BLANK_CHECK		5u
#define	 FMC_CMD_MARGIN_CHECK		6u
#define	 FMC_CMD_CHECKSUM		7u
#define	 FMC_CMD_WRITE			8u
#define	 FMC_CMD_WRITE_PROG		10u
#define	 FMC_CMD_PROGRAM		12u
#define	 FMC_CMD_REPORT_ECC		13u
#define	 FMC_CMD_DUMMY			255u	/* A no-op. */
#define	FMC_REG_EVENT			0x04u
#define	 FMC_EVENT_RST			(1u << 0)
#define	 FMC_EVENT_WAKEUP		(1u << 1)
#define	 FMC_EVENT_ABORT		(1u << 2)
#define	FMC_REG_STARTA			0x10u
#define	FMC_REG_STOPA			0x14u
#define	 FMC_ADDRESS(a)			(((a) >> 4) & 0x3ffffu)
#define	FMC_REG_DATAW(n)		(0x80u + ((n) * 4u))
#define	FMC_REG_INT_STATUS		0xfe0u
#define	FMC_REG_INT_CLR_STATUS		0xfe8u
#define	 FMC_INT_FAIL			(1u << 0)
#define	 FMC_INT_ERR			(1u << 1)
#define	 FMC_INT_DONE			(1u << 2)
#define	 FMC_INT_ECC_ERR		(1u << 3)
#define	 FMC_INT_ANY			(FMC_INT_FAIL | FMC_INT_ERR | \
					 FMC_INT_DONE | FMC_INT_ECC_ERR)
#define	 FMC_INT_ERRORS			(FMC_INT_FAIL | FMC_INT_ERR | \
					 FMC_INT_ECC_ERR)
#define	FMC_REG_MODULE_ID		0xffcu
#define	 FMC_MODULE_ID_APERTURE(r)	((r) & 0xffu)
#define	 FMC_MODULE_ID_MINOR_REV(r)	(((r) >> 8) & 0xfu)
#define	 FMC_MODULE_ID_MAJOR_REV(r)	(((r) >> 12) & 0xfu)
#define	 FMC_MODULE_ID_ID(r)		(((r) >> 16) & 0xffffu)
/* Note: Datasheet says ID is C40F. The datasheet is lying. */
#define	 FMC_MODULE_ID			0xc40au

#define	FMC_FLASH_BYTES_PER_WORD	0x10u
#define	FMC_FLASH_BYTES_PER_PAGE	0x200u

struct fmc_state {
	struct target_flash fs_tf;
	target_t fs_target;
	target_addr_t fs_regs;		/* Address of FMC registers */
	bool fs_init_done;
	void *fs_flash_applet;
};

static const uint8_t fmc_flash_applet[] = {
#include "flash-applets/applet-nxp-lpc55.applet"
};

/*
 * Convenience functions for register read/write
 */
static int
fmc_read_reg(struct fmc_state *fs, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(fs->fs_target, fs->fs_regs + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(fs->fs_target);

	return rv;
}

static int
fmc_write_reg(struct fmc_state *fs, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(fs->fs_target, fs->fs_regs + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(fs->fs_target);

	return rv;
}

static int
fmc_issue_command(struct fmc_state *fs, uint32_t cmd)
{
	timer_timeout_t to;
	uint32_t status;
	int rv;

	if (cmd != FMC_CMD_DUMMY) {
		if (fmc_write_reg(fs, FMC_REG_CMD, cmd) < 0)
			return -1;
	}

	timer_timeout_start(&to, 2000);

	do {
		if (fmc_read_reg(fs, FMC_REG_INT_STATUS, &status) < 0) {
			status = 0;
			break;
		}
	}  while ((status & FMC_INT_ANY) == 0 && !timer_timeout_expired(&to));

	if ((status & FMC_INT_ANY) == 0) {
		/* Timeout, or error reading status. */
		(void) fmc_write_reg(fs, FMC_REG_EVENT, FMC_EVENT_ABORT);
		rv = -1;
	} else
	if ((status & FMC_INT_ERRORS) != 0)
		rv = -1;
	else
		rv = ((status & FMC_INT_DONE) != 0) ? 0 : -1;

	if (fmc_write_reg(fs, FMC_REG_INT_CLR_STATUS, FMC_INT_ANY) < 0)
		rv = -1;

	return rv;
}

static int
fmc_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{

	(void) tf;

	/* Page and sector (erase) sizes are identical. */
	fs->fs_size = FMC_FLASH_BYTES_PER_PAGE;
	fs->fs_pages = 1;
	fs->fs_number = *offset / FMC_FLASH_BYTES_PER_PAGE;
	fs->fs_cookie = fs->fs_number * FMC_FLASH_BYTES_PER_PAGE;
	*offset %= FMC_FLASH_BYTES_PER_PAGE;

	return 0;
}

static int
fmc_erase_sector(target_flash_t tf, const struct target_flash_sector *tfs)
{
	struct fmc_state *fs = (struct fmc_state *)tf;
	uint32_t pgoff = tfs->fs_cookie;

	if (fs->fs_init_done == false) {
		if (fmc_write_reg(fs, FMC_REG_EVENT, FMC_EVENT_RST) < 0) {
			DBFPRINTF("failed to write EVENT.RST\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		if (fmc_issue_command(fs, FMC_CMD_DUMMY) < 0) {
			DBFPRINTF("Timeout waiting for EVENT.RST\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		fs->fs_init_done = true;
	}

	if (fmc_write_reg(fs, FMC_REG_STARTA, FMC_ADDRESS(pgoff)) < 0 ||
	    fmc_write_reg(fs, FMC_REG_STOPA, FMC_ADDRESS(pgoff)) < 0) {
		DBFPRINTF("failed to write starta/stopa %08" PRIx32 "\n",
		    (uint32_t)tfs->fs_cookie);
		return TARGET_FLASH_RESULT_ERROR;
	}

	if (fmc_issue_command(fs, FMC_CMD_ERASE_RANGE) < 0) {
		DBFPRINTF("Erase failed @ %08" PRIx32 "\n",
		    (uint32_t)tfs->fs_cookie);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
fmc_write_page_std(struct fmc_state *fs, uint32_t start_addr)
{
	uint32_t *data, addr;

	for (uint32_t off = 0;
	    off < FMC_FLASH_BYTES_PER_PAGE;
	    off += FMC_FLASH_BYTES_PER_WORD) {
		data = (uint32_t *)(uintptr_t)&fs->fs_tf.tf_buffer[off];
		addr = start_addr + off;

		if (fmc_write_reg(fs, FMC_REG_STARTA, FMC_ADDRESS(addr)) < 0) {
			DBFPRINTF("failed to write starta %08" PRIx32 "\n",
			    addr);
			return TARGET_FLASH_RESULT_ERROR;
		}

		for (unsigned int i = 0;
		    i < (FMC_FLASH_BYTES_PER_WORD / sizeof(uint32_t)); i++) {
			if (fmc_write_reg(fs, FMC_REG_DATAW(i), data[i]) < 0) {
				DBFPRINTF("failed to write DATAW(%u)\n", i);
				return TARGET_FLASH_RESULT_ERROR;
			}
		}

		if (fmc_issue_command(fs, FMC_CMD_WRITE_PROG) < 0) {
			DBFPRINTF("Write/prog failed @ %08" PRIx32 "\n", addr);
			return TARGET_FLASH_RESULT_ERROR;
		}
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
fmc_write_page(target_flash_t tf, const struct target_flash_sector *tfs,
    uint32_t page)
{
	struct fmc_state *fs = (struct fmc_state *)tf;
	uint32_t addr;
	int rv;

	/* Because page size == sector size, 'page' must always be zero. */
	assert(page == 0);
	(void) page;

	addr = (uint32_t)tfs->fs_cookie;

	if (fs->fs_flash_applet == NULL) {
		rv = fmc_write_page_std(fs, addr);
	} else {
		rv = flash_applet_write_page(tf, addr, tf->tf_max_page_size,
		    fs->fs_regs, 0, fs->fs_flash_applet);
	}

	return rv;
}

static int
fmc_finish(target_flash_t tf)
{
	struct fmc_state *fs = (struct fmc_state *)tf;
	int rv = TARGET_FLASH_RESULT_OK;

	if (fs->fs_flash_applet != NULL)
		rv = flash_applet_finish(fs->fs_flash_applet);

	fs->fs_init_done = false;

	return rv;
}

static void
fmc_free(target_flash_t tf)
{
	struct fmc_state *fs = (struct fmc_state *)tf;

	if (fs->fs_flash_applet != NULL)
		flash_applet_detach(fs->fs_flash_applet);

	zone_free(fs);
}

int
flash_nxp_lpc5x_attach(target_t t, target_addr_t flash_regs,
    target_addr_t flash_base, uint32_t flash_size, unsigned int flags,
    const char *desc)
{
	struct flash_applet_params fp;
	struct fmc_state *fs;
	uint32_t reg;

	assert(flash_size != 0);

	if ((fs = zone_calloc(1, sizeof(*fs))) == NULL)
		return -1;

	fs->fs_target = t;
	fs->fs_regs = flash_regs;

	if (fmc_read_reg(fs, FMC_REG_MODULE_ID, &reg) < 0) {
		DBFPRINTF("failed to read module ID register\n");
		zone_free(fs);
		return -1;
	}

	if (FMC_MODULE_ID_ID(reg) != FMC_MODULE_ID) {
		DBFPRINTF("Bad Module ID reg: %08" PRIx32 "\n", reg);
		zone_free(fs);
		return -1;
	}

	/* We have a Flash applet for this device. */
	fp.fp_max_page_size = FMC_FLASH_BYTES_PER_PAGE;
	fp.fp_applet = fmc_flash_applet;
	fp.fp_applet_size = sizeof(fmc_flash_applet);
	fp.fp_applet_stack_size = 0;
	fs->fs_flash_applet = flash_applet_attach(t, &fp);

	fs->fs_tf.tf_sector_desc = fmc_sector_desc;
	fs->fs_tf.tf_erase_sector = fmc_erase_sector;
	fs->fs_tf.tf_write_page = fmc_write_page;
	fs->fs_tf.tf_finish = fmc_finish;
	fs->fs_tf.tf_free = fmc_free;
	fs->fs_tf.tf_erase_byte = 0x00u;	/* Yes, it's an oddball. */
	fs->fs_tf.tf_size = flash_size;
	fs->fs_tf.tf_max_page_size = FMC_FLASH_BYTES_PER_PAGE;

	/* Add the Flash to the target */
	if (target_add_flash(t, &fs->fs_tf, flash_base, flash_size, desc,
	    flags) < 0) {
		DBFPRINTF("failed to add flash @ %08" PRIxTADDR "\n",
		    flash_base);
		return -1;
	}

	return 0;
}
