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
#include <string.h>

#include "target.h"
#include "flash_sam_efc.h"
#include "flash_applet.h"
#include "flash-applets/applet-sam-efc.h"
#include "timer.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Driver for the [Enhanced] Embedded Flash Controller found on most SAM3/4 SoCs
 */

struct sam_efc_plane {
	struct target_flash ep_tf;
	target_t ep_target;
	void *ep_applet;
	int ep_applet_instance;
	target_addr_t ep_base;		/* Flash base in memory map */
	target_addr_t ep_efc_regs;	/* Address of plane's EFC registers */
	uint32_t ep_sector_size;
	uint32_t ep_pages_per_sector;
	uint32_t ep_page_shift;
	uint32_t ep_wait_states;	/* # of wait states for writing flash */
	uint32_t ep_fmr_saved;		/* Preserved # of wait states */
	int ep_flags;			/* True if using EWP command */
#define	SAM_EFC_FLAG_USE_EWP		(1u << 0)
#define	SAM_EFC_FLAG_2_PLANES		(1u << 1)
#define	SAM_EFC_FLAG_SAVED_STATE	(1u << 2)
};

/* EFC Registers; offset from ep_efc_regs */
#define	SAM_EFC_FMR			0x00u
#define	 SAM_EFC_FMR_FRDY		(1u << 0)
#define	 SAM_EFC_FMR_FWS_SHIFT		8
#define	 SAM_EFC_FMR_FWS_MASK		(0x0fu << 8)
#define	SAM_EFC_FCR			0x04u
#define	 SAM_EFC_FCR_CMD(c,a)		(((c) | ((a) << 8)) | 0x5a000000u)
#define	 SAM_EFC_CMD_GETD		0x00u		/* Get descriptor */
#define	 SAM_EFC_CMD_WP			0x01u		/* Write page */
#define	 SAM_EFC_CMD_EWP		0x03u		/* Erase/Write page */
#define	 SAM_EFC_CMD_EPA_16		0x207u		/* Erase 16 pages */
#define	 SAM_EFC_CMD_CLB		0x09u		/* Unlock page(s) */
#define	 SAM_EFC_CMD_SGPB		0x0bu		/* Set GPNVM bit */
#define	 SAM_EFC_CMD_CGPB		0x0cu		/* Clear GPNVM bit */
#define	 SAM_EFC_CMD_GGPB		0x0du		/* Get GPNVM bit */
#define	SAM_EFC_FSR			0x08u
#define	 SAM_EFC_FSR_BUSY(fsr)		(((fsr) & 1u) == 0)
#define	SAM_EFC_FRR			0x0cu

/*
 * EFC provides a descriptor with which we can automagically determine the
 * parameters of the on-chip Flash.
 */
struct sam_efc_descriptor {
	uint32_t fl_id;
	uint32_t fl_size;
	uint32_t fl_page_size;
	uint32_t fl_nb_plane;
	uint32_t fl_plane[SAM_EFC_MAX_PLANES];
	uint32_t fl_nb_lock;
	uint32_t fl_lock0;	/* Only the first one is of interest */
};

#define	IS_POWER_OF_TWO(x)		(((x) & ((x) - 1)) == 0)

static const uint8_t sam_efc_flash_applet[] = {
#include "flash-applets/applet-sam-efc.applet"
};

/*
 * Convenience functions for read/write of EFC registers
 */
static int
sam_efc_read_reg(struct sam_efc_plane *ep, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(ep->ep_target, ep->ep_efc_regs + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ep->ep_target);

	return rv;
}

static int
sam_efc_write_reg(struct sam_efc_plane *ep, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(ep->ep_target, ep->ep_efc_regs + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ep->ep_target);

	return rv;
}

static int
sam_efc_send_command(struct sam_efc_plane *ep, uint32_t cmd)
{
	timer_timeout_t to;
	uint32_t fsr;

	if (sam_efc_write_reg(ep, SAM_EFC_FCR, cmd) < 0)
		return -1;

	timer_timeout_start(&to, 2000);

	do {
		if (sam_efc_read_reg(ep, SAM_EFC_FSR, &fsr) < 0)
			return -1;
	}  while (SAM_EFC_FSR_BUSY(fsr) && !timer_timeout_expired(&to));

	/* XXX: Check FSR error bits */

	return SAM_EFC_FSR_BUSY(fsr) ? -1 : 0;
}

static int
sam_efc_read_frr(struct sam_efc_plane *ep, uint32_t *dest, uint32_t count)
{

	while (count--) {
		if (sam_efc_read_reg(ep, SAM_EFC_FRR, dest++) < 0)
			return -1;
	}

	return 0;
}

static int
sam_efc_read_descriptor(struct sam_efc_plane *ep,
    struct sam_efc_descriptor *fl)
{
	uint32_t plane;

	/*
	 * Send the Get Descriptor command
	 */
	if (sam_efc_send_command(ep,
	    SAM_EFC_FCR_CMD(SAM_EFC_CMD_GETD, 0)) < 0) {
		DBFPRINTF("GETD failed\n");
		return -1;
	}

	/* Read the fixed-size portion */
	if (sam_efc_read_frr(ep, &fl->fl_id, 4) < 0) {
		DBFPRINTF("header read failed\n");
		return -1;
	}

	/* Do some simple validation checks on the descriptor so far */
	if (!IS_POWER_OF_TWO(fl->fl_size)) {
		DBFPRINTF("fl_size bogus %08" PRIx32 "\n", fl->fl_size);
		return -1;
	}

	/*
	 * Validate the page size value. There are only two options
	 * for SAM34x
	 */
	if (fl->fl_page_size != 0x100u && fl->fl_page_size != 0x200u) {
		DBFPRINTF("bogus page size: %" PRIu32 "\n", fl->fl_page_size);
		return -1;
	}

	/* Validate nb_plane */
	if (fl->fl_nb_plane == 0 || fl->fl_nb_plane > SAM_EFC_MAX_PLANES) {
		DBFPRINTF("bogus nb_plane: %" PRIu32 "\n", fl->fl_nb_plane);
		return -1;
	}

	/* Read in fl_plane[fl_nb_plane] */
	if (sam_efc_read_frr(ep, fl->fl_plane, fl->fl_nb_plane) < 0) {
		DBFPRINTF("nb_plane read failed\n");
		return -1;
	}

	/* Validate fl_plane[] */
	for (plane = 0; plane < fl->fl_nb_plane; plane++) {
		if (fl->fl_plane[plane] == 0 ||
		    !IS_POWER_OF_TWO(fl->fl_plane[plane])) {
			DBFPRINTF("nb_plane[%" PRIu32 "] bogus (%08" PRIx32
			    ")\n", plane, fl->fl_plane[plane]);
			return -1;
		}
	}

	/* Read fl_nb_lock and nb_lock[0] */
	if (sam_efc_read_frr(ep, &fl->fl_nb_lock, 2) < 0) {
		DBFPRINTF("nb_lock read failed\n");
		return -1;
	}

	/* Verify that fl_nb_lock is non-zero */
	if (fl->fl_nb_lock == 0) {
		DBFPRINTF("fl_nb_lock is zero!\n");
		return -1;
	}

	/* Likewise fl_lock0 is a non-zero power-of-two */
	if (fl->fl_lock0 == 0 || !IS_POWER_OF_TWO(fl->fl_lock0)) {
		DBFPRINTF("bogus fl_lock0: %08" PRIx32 "\n", fl->fl_lock0);
		return -1;
	}

	DBPRINTF("EFC Descriptor: ID %08" PRIx32 "\n", fl->fl_id);
	DBPRINTF("\tfl_size:      %" PRIu32 "\n", fl->fl_size);
	DBPRINTF("\tfl_page_size: %" PRIu32 "\n", fl->fl_page_size);
	DBPRINTF("\tfl_nb_lock:   %" PRIu32 "\n", fl->fl_nb_lock);
	DBPRINTF("\tfl_lock0      %" PRIu32 "\n", fl->fl_lock0);
	DBPRINTF("\tfl_nb_plane:  %" PRIu32 "\n", fl->fl_nb_plane);

	for (plane = 0; plane < fl->fl_nb_plane; plane++) {
		DBPRINTF("\tfl_plane[%" PRIu32 "]:  %" PRIu32 "\n", plane,
		    fl->fl_plane[plane]);
	}

	/*
	 * Validate the assumption that all lock regions are the same size
	 */
	for (plane = 1; plane < fl->fl_nb_lock; plane++) {
		uint32_t lock;

		if (sam_efc_read_frr(ep, &lock, 1) < 0) {
			DBFPRINTF("bogons\n");
			break;
		}

		if (lock != fl->fl_lock0) {
			/*
			 * Don't hide this behind DBPRINTF. If the check fails,
			 * I want to know about it.
			 */
#if (RELEASE_BUILD == 0)
			printf("\tfl_lock%-5" PRIu32 ":%" PRIu32 " <--- "
			    "DANGER!\n\n", plane, lock);
#endif
			break;
		}
	}

	/* Looks kosher */
	return 0;
}

static int
sam_efc_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{
	struct sam_efc_plane *ep = (struct sam_efc_plane *)tf;

	fs->fs_size = ep->ep_sector_size;
	fs->fs_pages = ep->ep_sector_size / tf->tf_max_page_size;
	fs->fs_number = *offset / ep->ep_sector_size;
	*offset %= ep->ep_sector_size;

	return 0;
}

static int
sam_efc_erase_sector(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_efc_plane *ep = (struct sam_efc_plane *)tf;
	uint32_t start_page, erase_pages;

	if ((ep->ep_flags & SAM_EFC_FLAG_SAVED_STATE) == 0) {
		uint32_t fmr;

		/*
		 * Configure a sensible value for Flash wait states, just in
		 * case... Keep the old value for later; we'll write it back
		 * when done.
		 */
		if (sam_efc_read_reg(ep, SAM_EFC_FMR, &ep->ep_fmr_saved) < 0) {
			DBFPRINTF("failed to read EFC.FMR\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		/* Clear FRDY and FWS */
		fmr = ep->ep_fmr_saved &
		    ~(SAM_EFC_FMR_FRDY | SAM_EFC_FMR_FWS_MASK);

		/* Insert the desired number of wait states */
		fmr |= ep->ep_wait_states << SAM_EFC_FMR_FWS_SHIFT;

		if (sam_efc_write_reg(ep, SAM_EFC_FMR, fmr) < 0) {
			DBFPRINTF("failed to write EFC.FMR\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		ep->ep_flags |= SAM_EFC_FLAG_SAVED_STATE;
	}

	/* Calculate the starting page number */
	start_page = fs->fs_number * ep->ep_pages_per_sector;

	/* Ensure the lock bits are clear. */
	if (sam_efc_send_command(ep,
	    SAM_EFC_FCR_CMD(SAM_EFC_CMD_CLB, start_page)) < 0) {
		DBFPRINTF("unlock failed @ start page %08" PRIx32 "\n",
		    start_page);
		return TARGET_FLASH_RESULT_ERROR;
	}

	/*
	 * SAM3x does not support page erase. We erase each page as
	 * they are written. But we do need to ensure lock bits are
	 * clear beforehand.
	 */
	if ((ep->ep_flags & SAM_EFC_FLAG_USE_EWP) != 0)
		return TARGET_FLASH_RESULT_OK;

	/*
	 * We erase in chunks of 16 pages since that seems to be a
	 * number which works across all devices and sector sizes.
	 * It does mean that we may need to issue multiple erase
	 * commands to cover a full sector.
	 */
	for (erase_pages = 0; erase_pages < ep->ep_pages_per_sector;
	    erase_pages += 16) {
		DBFPRINTF("16 pages @ %" PRIx32 " (%08" PRIxTADDR ")\n",
		    start_page,
		    ep->ep_base + (start_page * ep->ep_tf.tf_max_page_size));

		if (sam_efc_send_command(ep,
		    SAM_EFC_FCR_CMD(SAM_EFC_CMD_EPA_16, start_page)) < 0) {
			DBFPRINTF("erase failed @ page %" PRIx32 "\n",
			    start_page);
			return TARGET_FLASH_RESULT_ERROR;
		}

    		start_page += 16;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_efc_write_page(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct sam_efc_plane *ep = (struct sam_efc_plane *)tf;
	uint32_t faddr, cmd;
	int rv;

	/*
	 * The supplied page number is relative to the start of the
	 * sector. Adjust so that it is relative to the start of the
	 * Flash region.
	 */
	page += ep->ep_pages_per_sector * fs->fs_number;

	/* Calculate the physical address within the Flash region */
	faddr = ep->ep_base + (page * tf->tf_max_page_size);

	if (ep->ep_flags & SAM_EFC_FLAG_USE_EWP)
		cmd = SAM_EFC_CMD_EWP;
	else
		cmd = SAM_EFC_CMD_WP;

	if (ep->ep_applet == NULL) {
		/* Prime the latch buffer */
		if (TARGET_MEM_WRITE(ep->ep_target, (target_addr_t)faddr,
		    tf->tf_max_page_size, tf->tf_buffer) < 0) {
			(void) TARGET_LINK_ERROR(ep->ep_target);
			DBFPRINTF("failed to prime latch buffer\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		if (sam_efc_send_command(ep, SAM_EFC_FCR_CMD(cmd, page)) < 0) {
			DBFPRINTF("write failed @ page %08" PRIx32 "\n", faddr);
			return TARGET_FLASH_RESULT_ERROR;
		}
		rv = TARGET_FLASH_RESULT_OK;
	} else {
		rv = flash_applet_write_page(tf, faddr, tf->tf_max_page_size,
		    ep->ep_efc_regs,
		    SAM_EFC_APPLET_MAKE_MD_FLAGS(cmd, ep->ep_page_shift),
		    ep->ep_applet);
		if (rv == TARGET_FLASH_RESULT_ERROR) {
			DBFPRINTF("applet failed @ page %08" PRIx32 "\n",
			    faddr);
		}
	}

	return rv;
}

static int
sam_efc_finish(target_flash_t tf)
{
	struct sam_efc_plane *ep = (struct sam_efc_plane *)tf;
	int rv = TARGET_FLASH_RESULT_OK;

	if (ep->ep_applet != NULL)
		rv = flash_applet_finish(ep->ep_applet);

	if ((ep->ep_flags & SAM_EFC_FLAG_SAVED_STATE) != 0) {
		/*
		 * Restore the original EFC.FMR
		 */
		(void) sam_efc_write_reg(ep, SAM_EFC_FMR, ep->ep_fmr_saved);
		ep->ep_flags &= ~SAM_EFC_FLAG_SAVED_STATE;
	}

	return rv;
}

static void
sam_efc_free(target_flash_t tf)
{

	flash_sam_efc_detach(tf);
}

void
flash_sam_efc_detach(void *cookie)
{
	struct sam_efc_plane *ep = cookie;

	if (ep->ep_applet_instance == 0 && ep->ep_applet != NULL)
		flash_applet_detach(ep->ep_applet);

	zone_free(ep);
}

void *
flash_sam_efc_attach(target_t t, const target_addr_t *flash_base,
    target_addr_t efc_regs, uint32_t efc_offset,
    uint32_t wait_states, int use_ewp, int add_flash)
{
	struct sam_efc_descriptor fl;
	struct sam_efc_plane *ep, *first_ep;
	struct flash_applet_params fp;
	uint32_t plane = 0;
	void *flash_applet;

	if (add_flash) {
		fp.fp_applet = sam_efc_flash_applet;
		fp.fp_applet_size = sizeof(sam_efc_flash_applet);
		fp.fp_applet_stack_size = 0;
		fp.fp_max_page_size = 0x200;	/* Default, for now. */
		flash_applet = flash_applet_attach(t, &fp);
	} else {
		flash_applet = NULL;
	}

	first_ep = NULL;

	do {
		/*
		 * Each Flash plane needs its own state structure.
		 */
		if ((ep = zone_calloc(1, sizeof(*ep))) == NULL)
			return first_ep;

		/*
		 * Fill out the stuff needed for sam_efc_read_descriptor()
		 */
		ep->ep_target = t;
		ep->ep_applet = flash_applet;
		ep->ep_applet_instance = first_ep != NULL;
		ep->ep_efc_regs = efc_regs + (plane * efc_offset);

		/*
		 * The EFC has a Flash Descriptor which describes
		 * all the interesting Flash parameters. Fetch it
		 * when dealing with the first plane.
		 *
		 * Example Flash descriptor for SAM4S4A:
		 *
		 *	fl_id:        0x000f0730
		 *	fl_size:      0x00040000
		 *	fl_page_size: 0x00000200
		 *	fl_nb_plane:  1
		 *	fl_plane[0]:  0x00040000
		 *	fl_nb_lock:   32
		 *	fl_lock0:     0x00002000
		 */
		if (plane == 0 && sam_efc_read_descriptor(ep, &fl) < 0) {
			zone_free(ep);
			return NULL;
		}

		if (plane == 0 ||
		    flash_base[plane] != SAM_EFC_FLASH_BASE_CONTIG) {
			/*
			 * Flash plane may not be contiguous with previous
			 */
			ep->ep_base = flash_base[plane];
		} else {
			/*
			 * Flash plane is contiguous with previous
			 */
			ep->ep_base = flash_base[plane - 1] +
			    fl.fl_plane[plane - 1];
		}

		ep->ep_wait_states = wait_states;
		ep->ep_flags = (use_ewp ? SAM_EFC_FLAG_USE_EWP : 0) |
		    ((fl.fl_nb_plane == 2) ? SAM_EFC_FLAG_2_PLANES : 0);
		ep->ep_tf.tf_sector_desc = sam_efc_sector_desc;
		ep->ep_tf.tf_erase_sector = sam_efc_erase_sector;
		ep->ep_tf.tf_write_page = sam_efc_write_page;
		ep->ep_tf.tf_finish = sam_efc_finish;
		ep->ep_tf.tf_free = sam_efc_free;
		ep->ep_tf.tf_erase_byte = 0xffu;

		/* Record the size of this plane */
		ep->ep_tf.tf_size = fl.fl_plane[plane];

		/* Record write-unit size */
		ep->ep_tf.tf_max_page_size = fl.fl_page_size;

		/*
		 * Our erase size is simply the number of bytes per lock region.
		 *
		 * Note that the descriptor can support different sizes of lock
		 * regions in a single Flash. However all SAM3 and SAM4 devices
		 * use a fixed value so we can use fl_lock0 on its own.
		 */
		ep->ep_sector_size = fl.fl_lock0;

		/*
		 * A logical 'sector' is basically a lock region. So work out
		 * the number of pages therein.
		 */
		ep->ep_pages_per_sector = fl.fl_lock0 / fl.fl_page_size;
		ep->ep_page_shift = ffs(fl.fl_page_size) - 1;

		if (add_flash) {
			DBPRINTF("Flash plane %" PRIu32 ": %" PRIu32 " sectors "
			    "@ 0x%" PRIxTADDR ", Write size %" PRIu32 ", Erase "
			    "size %" PRIu32 "\n", plane,
			    ep->ep_tf.tf_size / ep->ep_sector_size, ep->ep_base,
			    ep->ep_tf.tf_max_page_size, ep->ep_sector_size);
			DBPRINTF("\tPages per sector: %" PRIu32 "\n",
			    ep->ep_sector_size / ep->ep_tf.tf_max_page_size);

			/*
			 * Add this Flash plane to the target
			 */
			if (add_flash && target_add_flash(t, &ep->ep_tf,
			    ep->ep_base, fl.fl_plane[plane], NULL, 0) < 0) {
				DBFPRINTF("failed to add plane %" PRIu32 "\n",
				    plane);
				zone_free(ep);
				/* Ealier zones freed during cleanup */
				first_ep = NULL;
				break;
			}
		}

		if (first_ep == NULL)
			first_ep = ep;
	} while (add_flash && ++plane < fl.fl_nb_plane &&
	    flash_base[plane] != SAM_EFC_FLASH_BASE_INVALID);

	return first_ep;
}

int
flash_sam_efc_gpnvm(void *arg, uint32_t op)
{
	struct sam_efc_plane *ep = arg;
	uint32_t cmd, res;

	if (ep == NULL)
		return -1;

	switch (op & 3) {
	case FLASH_SAM_EFC_GPNVM_QUERY:
		if (sam_efc_send_command(ep,
		    SAM_EFC_FCR_CMD(SAM_EFC_CMD_GGPB, 0)) < 0) {
			return -1;
		}

		if (sam_efc_read_reg(ep, SAM_EFC_FRR, &res) < 0) {
			return -1;
		}

		return (int) res;

	case FLASH_SAM_EFC_GPNVM_SET(0):
		cmd = SAM_EFC_CMD_SGPB;
		break;

	case FLASH_SAM_EFC_GPNVM_CLEAR(0):
		cmd = SAM_EFC_CMD_CGPB;
		break;

	default:
		return -1;
	}

	cmd = SAM_EFC_FCR_CMD(cmd, (op >> 8) & 0x1f);

	return sam_efc_send_command(ep, cmd);
}

int
flash_sam_efc_nplanes(void *arg)
{
	struct sam_efc_plane *ep = arg;

	if (ep == NULL)
		return 0;

	return (ep->ep_flags & SAM_EFC_FLAG_2_PLANES) ? 2 : 1;
}
