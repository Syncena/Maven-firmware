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

#include "flash_sam_calw.h"
#include "timer.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Driver for the Flash "FLASHCALW" Controller found on some SAM parts.
 */

struct sam_calw_state {
	struct target_flash cs_tf;
	target_t cs_target;
	target_addr_t cs_calw_regs;	/* Address of CALW registers */
	target_addr_t cs_base;		/* Flash base in memory map */
	uint32_t cs_pages_per_sector;
	uint32_t cs_fcr_saved;		/* Preserved SAM_CALW_FCR register */
	uint32_t cs_psr_saved;		/* Preserved PicoCache SR register */
	int cs_setup_done;
};

/* CALW Registers; offset from cs_calw_regs */
#define	SAM_CALW_FCR			0x00u
#define	 SAM_CALW_FSR_CLEAR_BITS	0x000000ddu
#define	 SAM_CALW_FSR_FWS		(1u << 6)
#define	SAM_CALW_FCMD			0x04u
#define	 SAM_CALW_FCMD_CMD(c,a)		(((c) | ((a) << 8)) | 0xa5000000u)
#define	 SAM_CALW_CMD_WP		0x01u		/* Write page */
#define	 SAM_CALW_CMD_EP		0x02u		/* Erase page */
#define	 SAM_CALW_CMD_CPB		0x03u		/* Clear page buffer */
#define	 SAM_CALW_CMD_UP		0x05u		/* Unlock page(s) */
#define	 SAM_CALW_CMD_QPR		0x0cu		/* Quick page read */
#define	SAM_CALW_FSR			0x08u
#define	 SAM_CALW_FSR_BUSY(fsr)		(((fsr) & 1u) == 0)
#define	 SAM_CALW_FSR_PAGE_ERASED(fsr)	(((fsr) & (1u << 5)) != 0)
#define	 SAM_CALW_FSR_LOCKED(fsr,l)	(((fsr) & (1u << (16 + (l)))) != 0)
#define	 SAM_CALW_LOCK_BITS		16
#define	SAM_CALW_FPR			0x0cu
#define	 SAM_CALW_FPR_PSZ(p)		(0x20u << (((p) >> 8) & 0x07))
#define	 SAM_CALW_FPR_FSZ(p)		((p) & 0x0fu)

#define	SAM_CALW_PICO_CTRL		0x408
#define	 SAM_CALW_PICO_CTRL_DISABLE	0u
#define	 SAM_CALW_PICO_CTRL_ENABLE	1u
#define	SAM_CALW_PICO_SR		0x40c
#define	 SAM_CALW_PICO_SR_ENABLED(sr)	(((sr) & 1u) != 0)
#define	SAM_CALW_PICO_MAINT0		0x420
#define	 SAM_CALW_PICO_MAINT0_INVALL	(1u << 0)

/*
 * Convenience functions for read/write of CALW registers
 */
static int
sam_calw_read_reg(struct sam_calw_state *cs, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(cs->cs_target, cs->cs_calw_regs + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(cs->cs_target);

	return rv;
}

static int
sam_calw_write_reg(struct sam_calw_state *cs, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(cs->cs_target, cs->cs_calw_regs + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(cs->cs_target);

	return rv;
}

static int
sam_calw_send_command(struct sam_calw_state *cs, uint32_t cmd)
{
	timer_timeout_t to;
	uint32_t fsr;

	if (sam_calw_write_reg(cs, SAM_CALW_FCMD, cmd) < 0)
		return -1;

	timer_timeout_start(&to, 2000);

	do {
		if (sam_calw_read_reg(cs, SAM_CALW_FSR, &fsr) < 0)
			return -1;
	}  while (SAM_CALW_FSR_BUSY(fsr) && !timer_timeout_expired(&to));

	/* XXX: Check FSR error bits */

	return SAM_CALW_FSR_BUSY(fsr) ? -1 : 0;
}

static int
sam_calw_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{
	struct sam_calw_state *cs = (struct sam_calw_state *)tf;

	/*
	 * Our sector size is simply the number of bytes per lock region.
	 * The number of lock regions is fixed.
	 */
	fs->fs_size = tf->tf_size / SAM_CALW_LOCK_BITS;
	fs->fs_pages = cs->cs_pages_per_sector;
	fs->fs_number = *offset / fs->fs_size;
	*offset %= fs->fs_size;

	return 0;
}

static int
sam_calw_erase(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_calw_state *cs = (struct sam_calw_state *)tf;
	uint32_t page, cmd, reg, lockbits, i;

	if (cs->cs_setup_done == 0) {
		/*
		 * Configure a sensible value for Flash wait states, just in
		 * case... Keep the old value for later; we'll write it back
		 * when done.
		 */
		if (sam_calw_read_reg(cs, SAM_CALW_FCR, &cs->cs_fcr_saved) < 0){
			DBFPRINTF("failed to read CALW.FCR\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		/* We want 1 wait state and no interrupts */
		reg = cs->cs_fcr_saved & ~SAM_CALW_FSR_CLEAR_BITS;
		reg |= SAM_CALW_FSR_FWS;

		if (sam_calw_write_reg(cs, SAM_CALW_FCR, reg) < 0) {
			DBFPRINTF("failed to write CALW.FCR\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		/* We may also need to disable the PicoCache */
		if (sam_calw_read_reg(cs, SAM_CALW_PICO_SR,
		    &cs->cs_psr_saved) < 0) {
			DBFPRINTF("failed to read PICO_SR\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		if (SAM_CALW_PICO_SR_ENABLED(cs->cs_psr_saved)) {
			/*
			 * PicoCache is enabled. Before disabling it
			 * make sure the cache is invalidated.
			 */
			if (sam_calw_write_reg(cs, SAM_CALW_PICO_MAINT0,
			    SAM_CALW_PICO_MAINT0_INVALL) < 0) {
				DBFPRINTF("failed to clean PicoCache\n");
				return TARGET_FLASH_RESULT_ERROR;
			}

			/* Now safe to disable the cache */
			if (sam_calw_write_reg(cs, SAM_CALW_PICO_CTRL,
			    SAM_CALW_PICO_CTRL_DISABLE) < 0) {
				DBFPRINTF("failed to disable PicoCache\n");
				return TARGET_FLASH_RESULT_ERROR;
			}
		}

		cs->cs_setup_done = 1;
	}

	/* Fetch the FSR so we can check status of lock bits */
	if (sam_calw_read_reg(cs, SAM_CALW_FSR, &lockbits) < 0) {
		DBFPRINTF("failed to read CALW.FSR\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Calculate the starting page for this sector */
	page = fs->fs_number * cs->cs_pages_per_sector;

	/* Unlock sector if required */
	if (SAM_CALW_FSR_LOCKED(lockbits, page) == 0 &&
	    sam_calw_send_command(cs,
	    SAM_CALW_FCMD_CMD(SAM_CALW_CMD_UP, page)) < 0) {
		DBFPRINTF("unlock failed @ page %08" PRIx32 "\n", page);
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Erase all pages in this sector */
	for (i = 0; i < cs->cs_pages_per_sector; i++) {
		/*
		 * Perform a "Quick Page Read" to determine if page
		 * is already erased
		 */ 
		cmd = SAM_CALW_FCMD_CMD(SAM_CALW_CMD_QPR, page + i);
		if (sam_calw_send_command(cs, cmd) < 0) {
			DBFPRINTF("QPR failed @ page %08" PRIx32 "\n",
			    page + i);
			return TARGET_FLASH_RESULT_ERROR;
		}

		/* Result is in the FSR */
		if (sam_calw_read_reg(cs, SAM_CALW_FSR, &reg) < 0) {
			DBFPRINTF("failed to read QPR result\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		if (SAM_CALW_FSR_PAGE_ERASED(reg))
			continue;

		/* Erase the page */
		cmd = SAM_CALW_FCMD_CMD(SAM_CALW_CMD_EP, page + i);
		if (sam_calw_send_command(cs, cmd) < 0) {
			DBFPRINTF("erase failed @ page %08" PRIx32 "\n",
			    page + i);
			return TARGET_FLASH_RESULT_ERROR;
		}
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_calw_write(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct sam_calw_state *cs = (struct sam_calw_state *)tf;
	target_addr_t faddr;

	/*
	 * The supplied page number is relative to the start of the
	 * sector. Adjust so that it is relative to the start of the
	 * Flash region.
	 */
	page += cs->cs_pages_per_sector * fs->fs_number;

	/* Calculate the physical address within the Flash region */
	faddr = cs->cs_base + (page * tf->tf_max_page_size);

	/* Clear the page buffer (probably uncessary) */
	if (sam_calw_send_command(cs,
	    SAM_CALW_FCMD_CMD(SAM_CALW_CMD_CPB, 0)) < 0) {
		DBFPRINTF("clear PB failed @ page %08" PRIx32 "\n", page);
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Fill the page buffer */
	if (TARGET_MEM_WRITE(cs->cs_target, faddr,
	    tf->tf_max_page_size, tf->tf_buffer) < 0) {
		(void) TARGET_LINK_ERROR(cs->cs_target);
		DBFPRINTF("failed to fill page buffer\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Write the page */
	if (sam_calw_send_command(cs,
	    SAM_CALW_FCMD_CMD(SAM_CALW_CMD_WP, page)) < 0) {
		DBFPRINTF("write failed @ page %08" PRIx32 "\n", page);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_calw_finish(target_flash_t tf)
{
	struct sam_calw_state *cs = (struct sam_calw_state *)tf;
	int rv = TARGET_FLASH_RESULT_OK;

	if (cs->cs_setup_done) {
		/*
		 * Restore the original CALW.FCR
		 */
		(void) sam_calw_write_reg(cs, SAM_CALW_FCR, cs->cs_fcr_saved);

		/*
		 * Restore PicoCache enable state
		 */
		if (SAM_CALW_PICO_SR_ENABLED(cs->cs_psr_saved)) {
			(void) sam_calw_write_reg(cs, SAM_CALW_PICO_CTRL,
			    SAM_CALW_PICO_CTRL_ENABLE);
		}

		cs->cs_setup_done = 0;
	}

	return rv;
}

static void
sam_calw_free(target_flash_t tf)
{

	zone_free(tf);
}

int
flash_sam_calw_attach(target_t t, target_addr_t flash_base,
    target_addr_t calw_regs, const uint16_t *flash_size_table)
{
	struct sam_calw_state *cs;
	uint32_t fpr, flash_size;

	if ((cs = zone_calloc(1, sizeof(*cs))) == NULL)
		return -1;

	cs->cs_target = t;
	cs->cs_base = flash_base;
	cs->cs_calw_regs = calw_regs;
	cs->cs_tf.tf_sector_desc = sam_calw_sector_desc;
	cs->cs_tf.tf_erase_sector = sam_calw_erase;
	cs->cs_tf.tf_write_page = sam_calw_write;
	cs->cs_tf.tf_finish = sam_calw_finish;
	cs->cs_tf.tf_free = sam_calw_free;
	cs->cs_tf.tf_erase_byte = 0xffu;

	/*
	 * The Flash is self-describing via the Parameter Register
	 */
	if (sam_calw_read_reg(cs, SAM_CALW_FPR, &fpr) < 0) {
		zone_free(cs);
		DBFPRINTF("failed to read CALW.FPR\n");
		return -1;
	}

	flash_size = (uint32_t)flash_size_table[SAM_CALW_FPR_FSZ(fpr)] * 1024u;
	if (flash_size == 0) {
		zone_free(cs);
		DBFPRINTF("Unsupported Flash size!\n");
		return -1;
	}

	cs->cs_tf.tf_size = flash_size;
	cs->cs_tf.tf_max_page_size = SAM_CALW_FPR_PSZ(fpr);

	/* Work out how many pages per 'erase' sector */
	cs->cs_pages_per_sector = (cs->cs_tf.tf_size / SAM_CALW_LOCK_BITS) /
	    cs->cs_tf.tf_max_page_size;

	cs->cs_setup_done = 0;

	/* Add the Flash to the target */
	if (target_add_flash(t, &cs->cs_tf, cs->cs_base, flash_size,
	    NULL, 0) < 0) {
		DBFPRINTF("failed to add flash @ %08" PRIxTADDR "\n",
		    cs->cs_base);
		return -1;
	}

	return 0;
}
