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
#include <string.h>

#include "rtos.h"
#include "cortexm.h"
#include "target.h"
#include "soc_rp2040.h"
#include "timer.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT     1
#include "debug.h"

#define	RP2040_TARGETSEL_VALID(ts)	(((ts) & 0x0fffffffu) == 0x01002927)
#define	RP2040_CORE_INSTANCE(ts)	(((ts) >> 28) & 0xfu)

#define	RP2040_PART_VENDOR		"Raspberry PI"
#define	RP2040_PART_DEVICE		"RP2040"

#define	RP2040_SRAM_STRIPED_BASE	0x20000000u
#define	RP2040_SRAM_STRIPED_SIZE	0x40000u
#define	RP2040_SRAM_8K_BASE		0x20040000u
#define	RP2040_SRAM_8K_SIZE		0x2000u
#define	RP2040_SRAM_NON_STRIPED_BASE	0x21000000u
#define	RP2040_SRAM_NON_STRIPED_SIZE	0x40000u
#define	RP2040_SRAM_XIP_CACHE_BASE	0x15000000u
#define	RP2040_SRAM_XIP_CACHE_SIZE	0x4000u
#define	RP2040_SRAM_USB_BASE		0x50100000
#define	RP2040_SRAM_USB_SIZE		0x1000u

#define	RP2040_FLASH_BASE		0x10000000u
#define	RP2040_FLASH_SIZE		(2u*1024u*1024u)
#define	RP2040_FLASH_PAGE_SIZE		0x100u
#define	RP2040_FLASH_ERASE_SIZE		0x1000u

#define	RP2040_ROM_BASE			0x00000000u
#define	RP2040_ROM_SIZE			0x4000u
#define	RP2040_ROM_OFF_SP		(RP2040_ROM_BASE + 0x00u)
#define	RP2040_ROM_OFF_MAGIC_VERSION	(RP2040_ROM_BASE + 0x10u)
#define	 RP2040_ROM_MAGIC_VALID(m)	(((m) & 0x00ffffffu) == 0x01754du)
#define	 RP2040_ROM_VERSION(m)		(((m) >> 24) & 0xffu)
#define	RP2040_ROM_OFF_FUNC_LOOKUP	(RP2040_ROM_BASE + 0x14u)

struct rp2040_state {
	struct target_flash rs_tf;	/* Must be the first item. */
	target_t rs_target;
	target_addr_t rs_ram_buff;
	bool rs_flash_init_done;
	uint32_t rs_rom_sp;
	uint32_t rs_rom_functions;
	uint32_t rs_rom_debug_tramp;
	uint32_t rs_rom_debug_tramp_end;
	uint32_t rs_rom_flash_connect;
	uint32_t rs_rom_flash_exit_xip;
	uint32_t rs_rom_flash_flush;
	uint32_t rs_rom_flash_enter_xip;
	uint32_t rs_rom_flash_erase;
	uint32_t rs_rom_flash_program;
};

struct rp2040_rom_call {
	uint32_t rc_args[4];
	uint32_t rc_function;
};

static int
rp2040_rom_lookup(struct rp2040_state *rs, char c1, char c2, uint32_t *addr)
{
	struct {
		uint8_t c1, c2;
		uint16_t offset;
	} rf;
	unsigned int a;

	for (a = 0; a < (RP2040_ROM_SIZE - sizeof(rf)); a += sizeof(rf)) {
		if (TARGET_MEM_READ(rs->rs_target,
		    (target_addr_t)(a + rs->rs_rom_functions),
		    sizeof(rf), &rf) < 0) {
			DBFPRINTF("Read failed @ 0x%08" PRIx32 "\n",
			    rs->rs_rom_functions + a);
			return -1;
		}

		if ((uint8_t)c1 == 0u && (uint8_t)c2 == 0u)
			break;

		if ((uint8_t)c1 == rf.c1 && (uint8_t)c2 == rf.c2) {
			*addr = (uint32_t)rf.offset + RP2040_ROM_BASE;
			return 1;
		}
	}

	return 0;
}

/*
 * Invoke RP2040 ROM routine.
 * XXX: Much of this is generic enough to be in cortexm.c
 */
static int
rp2040_call_rom(struct rp2040_state *rs, const struct rp2040_rom_call *rc,
    uint32_t *rvp)
{
	struct target_run_applet_arg aa[5];
	struct target_run_applet ra;
	union target_halt_state ths;
	timer_timeout_t to;
	uint32_t pc;
	int rv;

	/* Error if there is no debug trampoline. */
	if (rs->rs_rom_debug_tramp == 0)
		return -1;

	/* Build an applet structure to execute the ROM trampoline. */
	ra.ra_applet_address = (target_addr_t)rs->rs_rom_debug_tramp;
	ra.ra_applet_sp = (target_addr_t)rs->rs_rom_sp;
	ra.ra_nargs = 5;
	ra.ra_args = aa;

	/* Initialise parameters. */
	for (int i = 0; i < 4; i++) {
		aa[i].aa_reg = (uint32_t)i;
		aa[i].aa_val = rc->rc_args[i];
	}

	/* Trampoline expects function's address in r7. */
	aa[4].aa_reg = 7;
	aa[4].aa_val = rc->rc_function;

	if (target_ctl(rs->rs_target, TARGET_CTL_APPLET_RUN, &ra) < 0)
		return -1;

	/* Half a second seems like a resonable time to wait. */
	timer_timeout_start(&to, 500);

	do {
		/*
		 * All being well, the target will halt at the BKPT #0
		 * instruction at address rs->rs_rom_debug_tramp_end.
		 */
		rv = target_ctl(rs->rs_target, TARGET_CTL_HALT_POLL, &ths);
		if (rv != TARGET_HALT_STATE_RUNNING)
			break;
		rtos_task_sleep(1);
	} while (!timer_timeout_expired(&to));

	switch (rv) {
	case TARGET_HALT_STATE_FAULT:
	case TARGET_HALT_STATE_BREAKPOINT:
		pc = ths.ths_halt.th_addr | 1u;
		rv = (pc == rs->rs_rom_debug_tramp_end) ? 0 : -1;
		break;

	default:
		rv = -1;
		break;
	}

	if (rv < 0)
		(void) target_ctl(rs->rs_target, TARGET_CTL_APPLET_STOP, rvp);
	else
	if (rvp != NULL)
		*rvp = cortexm_core_reg_read(rs->rs_target->t_core, 0);

	return rv;
}

static int
rp2040_flash_init(struct rp2040_state *rs)
{
	struct rp2040_rom_call rc;
	int rv;

	if (rs->rs_flash_init_done)
		return 0;

	DBFPRINTF("Calling connect_internal_flash()\n");

	rc.rc_args[0] = 0;
	rc.rc_args[1] = 0;
	rc.rc_args[2] = 0;
	rc.rc_args[3] = 0;
	rc.rc_function = rs->rs_rom_flash_connect;

	if ((rv = rp2040_call_rom(rs, &rc, NULL)) < 0)
		return rv;

	DBFPRINTF("Calling flash_exit_xip()\n");
	rc.rc_function = rs->rs_rom_flash_exit_xip;

	if ((rv = rp2040_call_rom(rs, &rc, NULL)) >= 0)
		rs->rs_flash_init_done = true;

	DBFPRINTF("Done. rv %d\n", rv);
	return rv;
}

static int
rp2040_flash_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{

	(void) tf;

	fs->fs_size = RP2040_FLASH_ERASE_SIZE;
	fs->fs_pages = RP2040_FLASH_ERASE_SIZE / RP2040_FLASH_PAGE_SIZE;
	fs->fs_number = *offset / fs->fs_size;
	fs->fs_cookie = fs->fs_number * fs->fs_size;
	*offset %= fs->fs_size;

	return 0;
}

static int
rp2040_flash_erase_sector(target_flash_t tf,
    const struct target_flash_sector *fs)
{
	struct rp2040_state *rs = (struct rp2040_state *)tf;
	struct rp2040_rom_call rc;

	if (rp2040_flash_init(rs) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	rc.rc_args[0] = fs->fs_cookie;
	rc.rc_args[1] = fs->fs_size;
	rc.rc_args[2] = fs->fs_size;
	rc.rc_args[3] = 0x20u;	/* XXX: Should not be used. */
	rc.rc_function = rs->rs_rom_flash_erase;

	DBFPRINTF("Calling flash_range_erase(0x%" PRIx32 ", 0x%" PRIx32
	    ", 0x%" PRIx32 ", 0x%" PRIx32 ")\n", rc.rc_args[0], rc.rc_args[1],
	    rc.rc_args[2], rc.rc_args[3]);

	if (rp2040_call_rom(rs, &rc, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	return TARGET_FLASH_RESULT_OK;
}

static int
rp2040_flash_write_page(target_flash_t tf,
    const struct target_flash_sector *fs, uint32_t page)
{
	struct rp2040_state *rs = (struct rp2040_state *)tf;
	struct rp2040_rom_call rc;

	if (rp2040_flash_init(rs) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	if (TARGET_MEM_WRITE(rs->rs_target, rs->rs_ram_buff,
	    tf->tf_max_page_size, tf->tf_buffer) < 0) {
		DBFPRINTF("Failed to copy page to SRAM buffer.\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	rc.rc_args[0] = (page * tf->tf_max_page_size) + fs->fs_cookie;
	rc.rc_args[1] = (uint32_t)rs->rs_ram_buff;
	rc.rc_args[2] = tf->tf_max_page_size;
	rc.rc_args[3] = 0;
	rc.rc_function = rs->rs_rom_flash_program;

	DBFPRINTF("Calling flash_range_program(0x%" PRIx32 ", 0x%" PRIx32
	    ", 0x%" PRIx32 ", 0x%" PRIx32 ")\n", rc.rc_args[0], rc.rc_args[1],
	    rc.rc_args[2], rc.rc_args[3]);

	if (rp2040_call_rom(rs, &rc, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	/*
	 * Return TARGET_FLASH_RESULT_VERIFIED to prevent upper-layer code's
	 * attempt to read back Flash page. The read will fail because the
	 * Flash is not in XIP mode at this time.
	 */
	return TARGET_FLASH_RESULT_VERIFIED;
}

static int
rp2040_flash_finish(target_flash_t tf)
{
	struct rp2040_state *rs = (struct rp2040_state *)tf;
	struct rp2040_rom_call rc;

	if (rs->rs_flash_init_done == false)
		return TARGET_FLASH_RESULT_OK;

	rs->rs_flash_init_done = false;

	rc.rc_args[0] = 0;
	rc.rc_args[1] = 0;
	rc.rc_args[2] = 0;
	rc.rc_args[3] = 0;
	rc.rc_function = rs->rs_rom_flash_flush;

	DBFPRINTF("Calling flash_flush_cache(0x%" PRIx32 ", 0x%" PRIx32
	    ", 0x%" PRIx32 ", 0x%" PRIx32 ")\n", rc.rc_args[0], rc.rc_args[1],
	    rc.rc_args[2], rc.rc_args[3]);

	if (rp2040_call_rom(rs, &rc, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	rc.rc_args[0] = 0;
	rc.rc_args[1] = 0;
	rc.rc_args[2] = 0;
	rc.rc_args[3] = 0;
	rc.rc_function = rs->rs_rom_flash_enter_xip;

	DBFPRINTF("Calling rs_rom_flash_enter_xip(0x%" PRIx32 ", 0x%" PRIx32
	    ", 0x%" PRIx32 ", 0x%" PRIx32 ")\n", rc.rc_args[0], rc.rc_args[1],
	    rc.rc_args[2], rc.rc_args[3]);

	if (rp2040_call_rom(rs, &rc, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	if (target_ctl(rs->rs_target, TARGET_CTL_APPLET_STOP, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	return TARGET_FLASH_RESULT_OK;
}

static void
rp2040_flash_free(target_flash_t tf)
{

	/* No-op */
	(void) tf;
}

static void
rp2040_free(cortexm_t cm)
{
	struct rp2040_state *rs = cm->cm_soc;

	cm->cm_soc = NULL;
	zone_free(rs);
}

int
soc_rp2040_attach(target_t t)
{
	cortexm_t cm = t->t_core;
	struct rp2040_state *rs;
	struct adiv5_memap_link_details ld;
	uint32_t v;
	uint16_t f;

	/*
	 * These devices have a multi-drop DPv2 with valid TARGETSEL values
	 * for each core. Therefore, we can match on targetsel.
	 */
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_LINK_DETAILS, &ld) < 0 ||
	    !RP2040_TARGETSEL_VALID(ld.ld_targetsel)) {
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * Fetch and validate the bootrom magic number.
	 */
	if (TARGET_REG_READ32(t, RP2040_ROM_OFF_MAGIC_VERSION, &v) < 0 ||
	    !RP2040_ROM_MAGIC_VALID(v)) {
		DBFPRINTF("Bad magic 0x%08" PRIx32 "\n", v);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/* Fetch the base of the bootrom's function lookup table. */
	if (TARGET_REG_READ16(t, RP2040_ROM_OFF_FUNC_LOOKUP, &f) < 0) {
		DBFPRINTF("ROM lookup read failure.\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (f >= RP2040_ROM_SIZE) {
		DBFPRINTF("Bad ROM lookup table offset: 0x%04" PRIx16 ".\n", f);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * We're pretty sure this is an RP2040 at this point. The remainder
	 * of the work is only required when attaching Core#0.
	 */
	if (RP2040_CORE_INSTANCE(ld.ld_targetsel) != 0) {
		t->t_flags |= TARGET_FLAG_FLASH_RDONLY;
		cm->cm_flags |= CORTEXM_FLAG_NO_RESET;
		return CORTEXM_SOC_ATTACH_OK;
	}

	if ((rs = zone_calloc(1, sizeof(*rs))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	rs->rs_rom_functions = (target_addr_t)f + RP2040_ROM_BASE;
	rs->rs_target = t;

	/*
	 * Fetch start/end address of debug trampoline function.
	 * It's not fatal if we can't find them - we just won't support
	 * Flash memory.
	 */
	if (TARGET_REG_READ32(t, RP2040_ROM_OFF_SP, &rs->rs_rom_sp) < 0 ||
	    rp2040_rom_lookup(rs, 'D', 'T', &rs->rs_rom_debug_tramp) < 0 ||
	    rp2040_rom_lookup(rs, 'D', 'E', &rs->rs_rom_debug_tramp_end) < 0 ||
	    rp2040_rom_lookup(rs, 'I', 'F', &rs->rs_rom_flash_connect) < 0 ||
	    rp2040_rom_lookup(rs, 'E', 'X', &rs->rs_rom_flash_exit_xip) < 0 ||
	    rp2040_rom_lookup(rs, 'F', 'C', &rs->rs_rom_flash_flush) < 0 ||
	    rp2040_rom_lookup(rs, 'C', 'X', &rs->rs_rom_flash_enter_xip) < 0 ||
	    rp2040_rom_lookup(rs, 'R', 'E', &rs->rs_rom_flash_erase) < 0 ||
	    rp2040_rom_lookup(rs, 'R', 'P', &rs->rs_rom_flash_program) < 0) {
		zone_free(rs);
		DBFPRINTF("debug helper read failure.\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	target_add_memory(t, TARGET_MEM_RAM, RP2040_SRAM_STRIPED_BASE,
	    RP2040_SRAM_STRIPED_SIZE, "Striped");
	target_add_memory(t, TARGET_MEM_RAM, RP2040_SRAM_8K_BASE,
	    RP2040_SRAM_8K_SIZE, NULL);
	target_add_memory(t, TARGET_MEM_RAM, RP2040_SRAM_NON_STRIPED_BASE,
	    RP2040_SRAM_NON_STRIPED_SIZE, "Non-Striped");
	target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
	    RP2040_SRAM_XIP_CACHE_BASE, RP2040_SRAM_XIP_CACHE_SIZE,
	    "XIP Cache");
	target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
	    RP2040_SRAM_USB_BASE, RP2040_SRAM_USB_SIZE, "USB DPRAM");
	target_add_memory(t, TARGET_MEM_ROM, RP2040_ROM_BASE,
	    RP2040_ROM_SIZE, "Boot ROM");

	if (rs->rs_rom_debug_tramp != 0 && rs->rs_rom_debug_tramp_end != 0 &&
	    rs->rs_rom_flash_connect != 0 && rs->rs_rom_flash_exit_xip != 0 &&
	    rs->rs_rom_flash_flush != 0 && rs->rs_rom_flash_erase != 0 &&
	    rs->rs_rom_flash_program != 0 && rs->rs_rom_flash_enter_xip != 0 &&
	    target_alloc_ram(t, RP2040_FLASH_PAGE_SIZE, &rs->rs_ram_buff) >=0) {

		DBFPRINTF("RAM buff at 0x%08" PRIxTADDR "\n", rs->rs_ram_buff);

		rs->rs_tf.tf_size = RP2040_FLASH_SIZE;
		rs->rs_tf.tf_max_page_size = RP2040_FLASH_PAGE_SIZE;
		rs->rs_tf.tf_erase_byte = 0xffu;
		rs->rs_tf.tf_sector_desc = rp2040_flash_sector_desc;
		rs->rs_tf.tf_erase_sector = rp2040_flash_erase_sector;
		rs->rs_tf.tf_write_page = rp2040_flash_write_page;
		rs->rs_tf.tf_finish = rp2040_flash_finish;
		rs->rs_tf.tf_free = rp2040_flash_free;

		target_add_flash(t, &rs->rs_tf, RP2040_FLASH_BASE,
		    RP2040_FLASH_SIZE, NULL, 0);
	} else {
		target_add_memory(t, TARGET_MEM_ROM, RP2040_FLASH_BASE,
		    RP2040_FLASH_SIZE, "Read-Only Flash");
	}

	cm->cm_soc_ctl = NULL;
	cm->cm_soc_free = rp2040_free;
	cm->cm_soc = rs;

	target_register_name(RP2040_PART_VENDOR, RP2040_PART_DEVICE,
	    NULL, NULL);

	return CORTEXM_SOC_ATTACH_OK;
}
