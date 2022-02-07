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

#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#include "target.h"
#include "tmon.h"
#include "timer.h"
#include "rtos.h"
#include "zone_alloc.h"
#include "flash_applet.h"
#include "flash-applets/applet-shared.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Driver for Flash "Applets" - responsible for bulk writes to Flash.
 */

struct flash_applet_state {
	target_t fa_target;
	const void *fa_applet;
	uint32_t fa_applet_size;
	target_addr_t fa_applet_address;
	target_addr_t fa_applet_stack;
	struct flash_applet_descriptor *fa_desc;
	target_addr_t fa_desc_address;
	target_addr_t fa_next_address;
	int fa_active;
};

#define	DESC_ADDR(fa, field)	((fa)->fa_desc_address + \
				offsetof(struct flash_applet_descriptor, field))

void *
flash_applet_attach(target_t t, const struct flash_applet_params *fp)
{
	struct flash_applet_state *fa;
	target_addr_t ram_base;
	unsigned int ram_size, ram_min;
	uint32_t applet_size;

	/* 3 * page_size is a reasonable minimum. */
	ram_min = fp->fp_max_page_size * 3;

	/* Likewise, 64KB seems to be enough for excellent performance. */
	ram_size = 0x10000u;

	/* Grab some of the target's RAM */
	while (target_alloc_ram(t, ram_size, &ram_base) < 0 &&
	    ram_size > ram_min) {
		ram_size >>= 1;
	}

	if (ram_size <= ram_min) {
		DBFPRINTF("Failed to alloc RAM (0x%x, 0x%x)\n", ram_size,
		    ram_min);
		return NULL;
	}

	if ((fa = zone_calloc(1, sizeof(*fa))) == NULL) {
		DBFPRINTF("Failed to alloc state\n");
		return NULL;
	}

	if ((fa->fa_desc = zone_calloc(1, sizeof(*fa->fa_desc))) == NULL) {
		zone_free(fa);
		DBFPRINTF("Failed to alloc fa_desc\n");
		return NULL;
	}

	applet_size = (uint32_t)fp->fp_applet_stack_size;
	if (applet_size < FLASH_APPLET_DEFAULT_STACK)
		applet_size = FLASH_APPLET_DEFAULT_STACK;
	applet_size += (uint32_t)fp->fp_applet_size;
	applet_size = (applet_size + 7u) & ~7u;

	fa->fa_desc->fad_page_size = fp->fp_max_page_size;

	fa->fa_desc->fad_buffer_size = (uint32_t)ram_size;
	assert(fa->fa_desc->fad_buffer_size > applet_size);

	fa->fa_desc->fad_buffer_size -= applet_size;
	assert(fa->fa_desc->fad_buffer_size > sizeof(*fa->fa_desc));

	fa->fa_desc->fad_buffer_size -= sizeof(*fa->fa_desc);
	fa->fa_desc->fad_buffer_size &= ~(fp->fp_max_page_size - 1u);
	assert(fa->fa_desc->fad_buffer_size >= fp->fp_max_page_size);

	fa->fa_target = t;
	fa->fa_applet = fp->fp_applet;
	fa->fa_applet_size = (uint32_t)fp->fp_applet_size;
	fa->fa_applet_address = ram_base;
	fa->fa_applet_stack = ram_base + ram_size;
	fa->fa_desc_address = ram_base +
	    (((uint32_t)fp->fp_applet_size + 3u) & ~(sizeof(uint32_t) - 1u));
	fa->fa_active = 0;

	DBFPRINTF("Applet @ 0x%08" PRIxTADDR ", size 0x%" PRIx32 ", desc @ "
	    "0x%08" PRIxTADDR ", buffer size 0x%" PRIx32 "\n",
	    fa->fa_applet_address, fa->fa_applet_size, fa->fa_desc_address,
	    fa->fa_desc->fad_buffer_size);

	return fa;
}

void
flash_applet_detach(void *cookie)
{
	struct flash_applet_state *fa = cookie;

	flash_applet_finish(fa);

	zone_free(fa->fa_desc);
	zone_free(fa);
}

static int
flash_applet_activate(struct flash_applet_state *fa)
{
	struct target_run_applet_arg aa;
	struct target_run_applet ra;
	int s;

	if (fa->fa_active != 0)
		return 0;

	DBFPRINTF("starting applet\n");

	s = target_ctl(fa->fa_target, TARGET_CTL_HALT, NULL);
	if (s < 0) {
		DBFPRINTF("HALT failed.\n");
		return -1;
	}

	DBFPRINTF("halt state %d\n", s);

	s = target_ctl(fa->fa_target, TARGET_CTL_HALT_POLL, NULL);
	if (s < 0) {
		DBFPRINTF("HALT_POLL failed.\n");
		return -1;
	}

	DBFPRINTF("halt poll state %d\n", s);

	if (TARGET_MEM_WRITE(fa->fa_target, fa->fa_applet_address,
	    fa->fa_applet_size, fa->fa_applet) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to d/l applet\n");
		return -1;
	}

	fa->fa_desc->fad_write_idx = 0;
	fa->fa_desc->fad_read_idx = 0;
	fa->fa_desc->fad_target_idle = 1;
	fa->fa_desc->fad_dest = ~0u;
	fa->fa_desc->fad_controller = 0;
	fa->fa_desc->fad_flags =
	    tmon_flash_verify_enabled() ? FLASH_APPLET_FLAGS_VERIFY : 0;

	if (TARGET_MEM_WRITE(fa->fa_target, fa->fa_desc_address,
	    sizeof(*fa->fa_desc), fa->fa_desc) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to d/l descriptor\n");
		return -1;
	}

	aa.aa_reg = 0;	/* XXX: Too ARM-specific; make it generic. */
	aa.aa_val = fa->fa_desc_address;

	ra.ra_applet_address = fa->fa_applet_address;
	ra.ra_applet_sp = fa->fa_applet_stack;
	ra.ra_args = &aa;
	ra.ra_nargs = 1;

	if (target_ctl(fa->fa_target, TARGET_CTL_APPLET_RUN, &ra) < 0) {
		DBFPRINTF("Failed to start applet\n");
		return -1;
	}

	fa->fa_active = 1;

	DBFPRINTF("applet running\n");

	return 0;
}

static int
flash_applet_deactivate(struct flash_applet_state *fa)
{

	if (fa->fa_active == 0)
		return 0;

	DBFPRINTF("deactivating applet\n");

	fa->fa_active = 0;

	if (target_ctl(fa->fa_target, TARGET_CTL_APPLET_STOP, NULL) < 0) {
		DBFPRINTF("failed to stop applet\n");
		return -1;
	}

	/* XXX: Should we efface the applet from SRAM here? */

	return 0;
}

static void
flash_applet_dump_fad(struct flash_applet_state *fa)
{
#if !defined(RELEASE_BUILD) || (RELEASE_BUILD == 0)
	struct flash_applet_descriptor *fad = fa->fa_desc;

	if (TARGET_MEM_READ(fa->fa_target, DESC_ADDR(fa, fad_write_idx), sizeof(*fad),
	    fad) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		printf("flash_applet_buffer_space: Failed to read FAD\n");
		return;
	}

	printf("flash_applet_dump_fad: Applet State...\n");
	printf("\tfad_write_idx %" PRIx32 "\n\tfad_read_idx %" PRIx32 "\n",
	    fad->fad_write_idx, fad->fad_read_idx);
	printf("\tfad_target_idle %" PRIx32 "\n\tfad_status %" PRIx32 "\n",
	    fad->fad_target_idle, fad->fad_status);
	printf("\tfad_dest %" PRIx32 "\n\tfad_controller %" PRIx32 "\n",
	    fad->fad_dest, fad->fad_controller);
	printf("\tfad_page_size %" PRIx32 "\n\tfad_md_flags %" PRIx32 "\n",
	    fad->fad_page_size, fad->fad_md_flags);
	printf("\tfad_buffer_size %" PRIx32 "\n", fad->fad_buffer_size);
#else
	(void) fa;
#endif
}

static int
flash_applet_wait_idle(struct flash_applet_state *fa)
{
	timer_timeout_t to;

//	DBFPRINTF("will wait for idle\n");

	/* Update the write index on the target. */
	if (TARGET_REG_WRITE32(fa->fa_target,
	    DESC_ADDR(fa, fad_write_idx), fa->fa_desc->fad_write_idx) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to update widx\n");
		return -1;
	}

	/* Let the target catch up, if necessary. */
	rtos_task_sleep(1);

	/* Grab read_idx . */
	if (TARGET_REG_READ32(fa->fa_target,
	    DESC_ADDR(fa, fad_read_idx), &fa->fa_desc->fad_read_idx) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to read ridx\n");
		return -1;
	}

	/* 10 seconds seems a reasonable starting point... */
	timer_timeout_start(&to, 10000);

	do {
		if (TARGET_REG_READ32(fa->fa_target,
		    DESC_ADDR(fa, fad_target_idle),
		    &fa->fa_desc->fad_target_idle) < 0) {
			(void) TARGET_LINK_ERROR(fa->fa_target);
			return -1;
		}

		if (fa->fa_desc->fad_target_idle) {
			DBFPRINTF("applet is idle (%" PRIu32 ")\n",
			    fa->fa_desc->fad_target_idle);
			break;
		}

		rtos_task_sleep(1);
	} while (!timer_timeout_expired(&to));

	if (fa->fa_desc->fad_target_idle == 0) {
		DBFPRINTF("timeout waiting for idle\n");
		return -1;
	}

	if (fa->fa_desc->fad_target_idle == 1)
		return 0;

	/* Target has halted due to an error. */
	DBFPRINTF("Target error.\n");
	flash_applet_dump_fad(fa);

	return -1;
}

static int
flash_applet_wait_space(struct flash_applet_state *fa, uint32_t page_size)
{
	timer_timeout_t to;
	uint32_t sync_space;

#if 0
	DBFPRINTF("will wait for space (0x%" PRIx32 ")\n", page_size);
#endif

	/*
	 * Short-circuit if we think there's enough space, but don't
	 * get ahead of the target by more than half the buffer size
	 * without a resync.
	 */
	sync_space = fa->fa_desc->fad_buffer_size / 2u;

	if (page_size && flash_applet_buffer_count(fa->fa_desc) <= sync_space &&
	    flash_applet_buffer_space(fa->fa_desc, page_size)) {
		return 0;
	}

	/* Need to resync target's read index */

	/* Update the write index on the target. */
	if (TARGET_REG_WRITE32(fa->fa_target,
	    DESC_ADDR(fa, fad_write_idx), fa->fa_desc->fad_write_idx) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to update widx\n");
		return -1;
	}

	/* 10 seconds seems a reasonable starting point... */
	timer_timeout_start(&to, 10000);

	do {
		rtos_task_sleep(5);

		/* Grab read_idx and target_idle in one operation. */
		if (TARGET_MEM_READ(fa->fa_target,
		    DESC_ADDR(fa, fad_read_idx), sizeof(uint32_t) * 2u,
		    &fa->fa_desc->fad_read_idx) < 0) {
			(void) TARGET_LINK_ERROR(fa->fa_target);
			DBFPRINTF("Failed to read ridx\n");
			return -1;
		}

		if (fa->fa_desc->fad_target_idle < 2 && page_size &&
		    flash_applet_buffer_count(fa->fa_desc) <= sync_space &&
		    flash_applet_buffer_space(fa->fa_desc, page_size)) {
			DBPRINTF("Buffer space after resync 0x%08" PRIx32 ", "
			    "idle 0x%" PRIx32 ", took %" PRIx32 " mS\n",
			    fa->fa_desc->fad_buffer_size -
			    flash_applet_buffer_count(fa->fa_desc),
			    fa->fa_desc->fad_target_idle,
			    10000 - timer_timeout_remaining(&to));
//			flash_applet_dump_fad(fa);
			return 0;
		}

		/*
		 * If the target is not busy then either we're done
		 * or the target has stopped.
		 */
		if (fa->fa_desc->fad_target_idle) {
			DBFPRINTF("target finished (%" PRIu32 ")\n",
			    fa->fa_desc->fad_target_idle);
			break;
		}
	} while (!timer_timeout_expired(&to));

	if (page_size == 0 && fa->fa_desc->fad_target_idle == 1 &&
	    fa->fa_desc->fad_read_idx == fa->fa_desc->fad_write_idx) {
		DBFPRINTF("finished\n");
		return 0;
	}

	if (fa->fa_desc->fad_target_idle == 0)
		DBFPRINTF("timeout\n");
	else
	if (fa->fa_desc->fad_target_idle == 2) {
		DBFPRINTF("target error\n");
		flash_applet_dump_fad(fa);
	}

	return -1;
}

int
flash_applet_write_page(target_flash_t tf, target_addr_t flash_address,
    uint32_t page_size, target_addr_t ctrl_address, uint32_t md_flags,
    void *cookie)
{
	struct flash_applet_state *fa = cookie;

	/* First time through, activate the applet */
	if (fa->fa_active == 0 && flash_applet_activate(fa) < 0)
		return TARGET_FLASH_RESULT_ERROR;

//	DBFPRINTF("addr 0x%08" PRIxTADDR ", page "0x%" PRIx32 ", ctrl "
//	    "0x%08" PRIxTADDR "\n", flash_address, page_size, ctrl_address);

	/*
	 * If the target Flash address is not contiguous with the previous
	 * page, or the page size has changed, or the Flash controller
	 * address has changed, then we need to wait until the target is
	 * idle, then update the relevant fields.
	 *
	 * Note that this will always be the case the first time through.
	 */
	if (flash_address != fa->fa_desc->fad_dest ||
	    ctrl_address != fa->fa_desc->fad_controller ||
	    page_size != fa->fa_desc->fad_page_size ||
	    md_flags != fa->fa_desc->fad_md_flags) {
		DBFPRINTF("resync params\n");

		if (flash_applet_wait_idle(fa) < 0)
			return TARGET_FLASH_RESULT_ERROR;

		fa->fa_desc->fad_dest = flash_address;
		fa->fa_desc->fad_controller = ctrl_address;
		fa->fa_desc->fad_page_size = page_size;
		fa->fa_desc->fad_md_flags = md_flags;

		if (TARGET_MEM_WRITE(fa->fa_target, DESC_ADDR(fa, fad_dest),
		     sizeof(uint32_t) * 4, &fa->fa_desc->fad_dest) < 0) {
			(void) TARGET_LINK_ERROR(fa->fa_target);
			DBFPRINTF("resync failed\n");
			return TARGET_FLASH_RESULT_ERROR;
		}
	}

	/* Wait for contiguous space for this page. */
	if (flash_applet_wait_space(fa, page_size) < 0)
		return TARGET_FLASH_RESULT_ERROR;

//	DBPRINTF("wp: w 0x%" PRIx32 ", r 0x%" PRIx32 "\n",
//	    fa->fa_desc->fad_write_idx, fa->fa_desc->fad_read_idx);

	/*
	 * Write the page to the ring buffer. Note that the ring-buffer
	 * is always a multiple of the maximum page size, so we can
	 * ignore wrap-around here.
	 */
	if (TARGET_MEM_WRITE(fa->fa_target,
	    DESC_ADDR(fa, fad_buffer) + fa->fa_desc->fad_write_idx,
	    page_size, tf->tf_buffer) < 0) {
		(void) TARGET_LINK_ERROR(fa->fa_target);
		DBFPRINTF("Failed to write page\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Update the write index, watching for wrap-around. */
	fa->fa_desc->fad_write_idx += page_size;
	if (fa->fa_desc->fad_write_idx >= fa->fa_desc->fad_buffer_size)
		fa->fa_desc->fad_write_idx = 0;

	/* Update our notion of the next Flash address. */
	fa->fa_desc->fad_dest += page_size;

	/* Applet will handle verification. */
	return TARGET_FLASH_RESULT_VERIFIED;
}

int
flash_applet_finish(void *cookie)
{
	struct flash_applet_state *fa = cookie;
	int rv = TARGET_FLASH_RESULT_OK;

	if (fa->fa_active) {
		/* Sync with the target and wait until it completes. */
		if (flash_applet_wait_space(fa, 0) < 0)
			rv = TARGET_FLASH_RESULT_ERROR;

		if (flash_applet_deactivate(fa) < 0)
			rv = TARGET_FLASH_RESULT_ERROR;
	}

	return rv;
}
