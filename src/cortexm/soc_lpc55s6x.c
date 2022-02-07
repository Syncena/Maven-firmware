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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cortexm.h"
#include "cortexm_regs.h"
#include "cortexm_txml.h"
#include "soc_lpc55s6x.h"
#include "flash_nxp_lpc5x.h"
#include "nxp_debug_mbox.h"
#include "tmon.h"
#include "hardware.h"
#include "timer.h"
#include "zone_alloc.h"

/*
 * Support for NXP LPC55S6x devices.
 */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Set this to '1' to force core#0 to stop in the boot ROM at reset.
 * Otherwise, set to '0' to attempt to stop in application code.
 *
 * Note that '0' doesn't work as expected - the boot ROM will invoke
 * a buzz-loop in SRAM, where we eventually halt. This is not useful!
 */
#define	LPC55S6X_HALT_IN_ROM	1

#define LPC55S6X_ROM_PIDR	0x000095000ull
#define LPC55S6X_ROM_MASK	0xf000fffffull

/* Undocumented Flash memory size register */
#define	LPC55S6X_SYSCON_MEMCFG	0x40000fe0u

/* Device/ROM revision registers. */
#define	LPC55S6X_SYSCON_DEV_ID	0x40000ff8u
#define	 LPC55S6X_DEV_ID_ROM(v)	(((v) >> 20) & 0xfu)
#define	LPC55S6X_SYSCON_DIEID	0x40000ffcu
#define	 LPC55S6X_DIEID_REV(v)	((v) & 0xfu)

#define	LPC55S6X_PART_VENDOR	"NXP Semiconductors"
#define	LPC55S6X_PART_PREFIX	"LPC55S6X"

#define	LPC55S6X_FLASH_REGS	0x40034000u
#define	LPC55S6X_FLASH_BASE_NS	0x00000000u
#define	LPC55S6X_FLASH_BASE_S	0x10000000u

#define	LPC55S6X_ROM_BASE	0x13000000u
#define	LPC55S6X_ROM_SIZE	0x00020000u

#define	LPC55S6X_SRAMX_BASE	0x04000000u
#define	LPC55S6X_SRAMX_SIZE	0x00008000u

#define	LPC55S6X_SRAM0_BASE	0x20000000u
#define	LPC55S6X_SRAM0_SIZE	0x00010000u

#define	LPC55S6X_SRAM1_BASE	0x20010000u
#define	LPC55S6X_SRAM2_BASE	0x20020000u
#define	LPC55S6X_SRAM3_BASE	0x20030000u
#define	LPC55S6X_SRAM4_BASE	0x20040000u

#define	LPC55S6X_USB_SRAM_BASE	0x40100000u
#define	LPC55S6X_USB_SRAM_SIZE	0x00004000u

#define	LPC55S6X_CFPA_BASE	0x0009de00u
#define	LPC55S6X_CFPA_SIZE	0x00000600u

#define	LPC55S6X_CMPA_KSA_BASE	0x0009e400u
#define	LPC55S6X_CMPA_KSA_SIZE	0x00000800u

#define	LPC55S6X_NMPA_BASE	0x0009ec00u
#define	LPC55S6X_NMPA_SIZE	0x00001200u

struct lpc55s6x_state {
	void *ls_mbox;
	uint32_t ls_flash_size;
};

struct lpc_mem_config {
	uint8_t mc_cfg;
	uint16_t mc_flash;
	uint8_t mc_ram1;
	uint8_t mc_ram2;
	uint8_t mc_ram3;
	uint8_t mc_ram4;
};
static const struct lpc_mem_config lpc_mem_config_55s6x[] = {
	{0u, 0x200u, 32u, 0u, 0u, 0u},
	{1u, 0x400u, 32u, 0u, 0u, 0u},
	{2u, 0x800u, 64u, 64u, 0u, 16u},
	{5u, 0x0u, 64u, 64u, 64u, 16u},
	{255u, 0x9d8u, 64u, 64u, 64u, 16u},
};
#define	LPC_MEM_CONFIG_55S6X_COUNT	(sizeof(lpc_mem_config_55s6x) / \
					 sizeof(lpc_mem_config_55s6x[0]))
#define	LPC_RAM_SIZE(m)		((uint32_t)(m) * 0x400u)
#define	LPC_FLASH_SIZE(m)	((mc)->mc_flash * 0x100u)

static bool
soc_lpc55s6x_rom_lookup(target_t t)
{
	struct adiv5_memap_rom_table_lookup rt;

	rt.rt_pidr = LPC55S6X_ROM_PIDR;
	rt.rt_pidr_mask = LPC55S6X_ROM_MASK;

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 1) {
		DBFPRINTF("LPC55S6x\n");
		return true;
	}

	return false;
}

static const struct lpc_mem_config *
soc_lpc55s66x_mem_config(target_t t)
{
	const struct lpc_mem_config *mc;
	cortexm_t cm = t->t_core;
	target_addr_t memcfg;
	uint32_t reg;

	/*
	 * Point to the undocumented Flash memory config register
	 */
	memcfg = LPC55S6X_SYSCON_MEMCFG;
	if (cortexm_v8m_secure_debug_available(cm))
		memcfg |= 0x10000000u;

	if (TARGET_REG_READ32(t, memcfg, &reg) < 0)
		reg = 5;

	for (mc = lpc_mem_config_55s6x;
	    mc < &lpc_mem_config_55s6x[LPC_MEM_CONFIG_55S6X_COUNT];
	    mc++) {
		if (mc->mc_cfg == (uint8_t)reg)
			return mc;
		if (mc->mc_cfg == 255u)
			return mc;
	}

	return NULL;
}

static int
lpc_scs_read(cortexm_t cm, uint32_t reg, uint32_t *v)
{
	target_addr_t a = (target_addr_t)(cm->cm_scs_base + reg);

	return TARGET_REG_READ32(cm->cm_target, a, v);
}

static int
lpc_scs_write(cortexm_t cm, uint32_t reg, uint32_t v)
{
	target_addr_t a = (target_addr_t)(cm->cm_scs_base + reg);

	return TARGET_REG_WRITE32(cm->cm_target, a, v);
}

static int
lpc_dwt_write(cortexm_t cm, uint32_t reg, uint32_t v)
{
	target_addr_t a = (target_addr_t)(cm->cm_dwt_base + reg);

	return TARGET_REG_WRITE32(cm->cm_target, a, v);
}

#if 0
static int
lpc_dwt_read(cortexm_t cm, uint32_t reg, uint32_t *v)
{
	target_addr_t a = (target_addr_t)(cm->cm_dwt_base + reg);

	return TARGET_REG_READ32(cm->cm_target, a, v);
}
#endif

static bool
lpc_dwt_set_stop(cortexm_t cm, u_int n, uint32_t start, uint32_t end)
{
	
	return lpc_dwt_write(cm, CMDWT_REG_FUNCTION(n), 0x412u) == 0 &&
	    lpc_dwt_write(cm, CMDWT_REG_COMP(n), start) == 0 &&
	    lpc_dwt_write(cm, CMDWT_REG_COMP(n + 1), end) == 0 &&
	    lpc_dwt_write(cm, CMDWT_REG_FUNCTION(n + 1), 0x403u) == 0;
}

static bool
lpc_dwt_clear_stop(cortexm_t cm, u_int n)
{

	return lpc_dwt_write(cm, CMDWT_REG_FUNCTION(n), 0u) == 0 &&
	    lpc_dwt_write(cm, CMDWT_REG_FUNCTION(n + 1), 0u) == 0;
}

#if (LPC55S6X_HALT_IN_ROM == 0)
static bool
lpc_bad_sp(uint32_t sp, uint32_t sec)
{

	/* Bits 0-2 must be clear (also catches erased Flash). */
	if ((sp & 0x7u) != 0u)
		return true;

	/* If we're limited to insecure debug, SP must also be insecure. */
	if (sec == 0u && (sp & 0x10000000u) != 0)
		return true;

	/* Finally, SP must be in SRAM. */
	return sp < LPC55S6X_SRAM0_BASE || sp >= 0x40000000u;
}

static bool
lpc_bad_pc(struct lpc55s6x_state *ls, uint32_t pc, uint32_t sec)
{

	/* Bit 0 must be set. */
	if ((pc & 0x1u) == 0)
		return true;

	/* If we're limited to insecure debug, PC must also be insecure. */
	if (sec == 0u && (pc & 0x10000000u) != 0)
		return true;

	pc &= ~0x10000000u;

	/* Good if PC is in Flash. */
	if (pc < ls->ls_flash_size)
		return false;

	/* Also good if PC is in SRAM X. */
	if (pc >= LPC55S6X_SRAMX_BASE &&
	    pc < (LPC55S6X_SRAMX_BASE + LPC55S6X_SRAMX_SIZE))
		return false;

	return true;
}
#endif /* (LPC55S6X_HALT_IN_ROM == 0) */

static int
soc_lpc55s6x_core_reset_impl(cortexm_t cm, int and_halt)
{
	struct lpc55s6x_state *ls = cm->cm_soc;

	/*
	 * We must override the default 'reset/halt' procedure because we
	 * need the boot ROM to execute enough code to enable the DAPs after
	 * a reset. This means the usual exercise of catching the reset
	 * vector will not work; the core will halt, but the DAP is disabled
	 * so we lose contact with it!
	 *
	 * So, to regain control after reset we use the DWT to trap the
	 * boot ROM reading the "imageType" field from the application's
	 * vector table.
	 */

	/* If halting, CMSCS_DEMCR_DWTENA must be set. */
	if (and_halt && (cm->cm_demcr & CMSCS_DEMCR_DWTENA) == 0) {
		DBFPRINTF("DWT unavailable!\n");
		return -1;
	}

	DBFPRINTF("DEMCR %08" PRIx32 " -> %08" PRIx32 "\n",
	    cm->cm_demcr, cm->cm_demcr & ~CMSCS_DEMCR_VC_CORERESET);

	/* Ensure CMSCS_DEMCR_VC_CORERESET is clear. */
	if (lpc_scs_write(cm, CMSCS_REG_DEMCR,
	    cm->cm_demcr & ~CMSCS_DEMCR_VC_CORERESET) < 0) {
		DBFPRINTF("Failed to clear VC_CORERESET\n");
		return -1;
	}

	if (and_halt) {
#if (LPC55S6X_HALT_IN_ROM == 0)
		uint32_t flash_start, flash_end, sramx_start, sramx_end, sec;
		uint32_t sp, pc;
#endif

		/*
		 * Halt the core.
		 */
		if (lpc_scs_write(cm, CMSCS_REG_DHCSR, CMSCS_DHCSR_DBGKEY |
		    CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_DEBUGEN) < 0) {
			DBFPRINTF("Failed to set C_DEBUGEN & C_HALT\n");
			return -1;
		}

#if (LPC55S6X_HALT_IN_ROM == 0)
		sec = cortexm_v8m_secure_debug_available(cm) ? 0x10000000u : 0u;

		/*
		 * Our goal is to stop the core on the first instruction
		 * in user Flash, or in the boot ROM if the Flash vector
		 * table entry for 'reset' appears to be bad.
		 *
		 * Fetch the reset MSP and PC from Flash...
		 */
		if (TARGET_REG_READ32(cm->cm_target, sec | 0x0u, &sp) < 0 ||
		    TARGET_REG_READ32(cm->cm_target, sec | 0x4u, &pc) < 0) {
			DBFPRINTF("Failed to read vector table\n");
			return -1;
		}

		/*
		 * Validate SP and PC.
		 */
		if (lpc_bad_sp(sp, sec) || lpc_bad_pc(ls, pc, sec)) {
			DBFPRINTF("SP (%08" PRIx32 ") or PC (%08" PRIx32
			    "bad\n", sp, pc);

			if (sec) {
				/*
				 * SP or PC are invalid, but we have secure
				 * debug. Arrange to halt in the boot ROM.
				 */
				uint32_t roms, rome;

				DBFPRINTF("Will halt in boot ROM\n");
				roms = LPC55S6X_ROM_BASE;
				rome = LPC55S6X_ROM_BASE + LPC55S6X_ROM_SIZE;
				if (!lpc_dwt_set_stop(cm, 0, roms, rome - 1u)) {
					DBFPRINTF("DWT write failed (ROM)\n");
					return -1;
				}
			}
		} else {
			/*
			 * SP and PC are valid.
			 * Arrange to halt in Flash or SRAM X.
			 */
			flash_start = LPC55S6X_FLASH_BASE_NS |
			    (pc & 0x10000000u);
			flash_end = flash_start + (ls->ls_flash_size - 1u);
			sramx_start = LPC55S6X_SRAMX_BASE | (pc & 0x10000000u);
			sramx_end = sramx_start + 0xffffu;

			DBFPRINTF("DWT halt on %08" PRIx32 "-%08" PRIx32 " and "
			    "%08" PRIx32 "-%08" PRIx32 "\n",
			    flash_start, flash_end, sramx_start, sramx_end);

			if (!lpc_dwt_set_stop(cm, 0, flash_start, flash_end) ||
			    !lpc_dwt_set_stop(cm, 2, sramx_start, sramx_end)) {
				DBFPRINTF("Failed to write DWT\n");
				return -1;
			}
		}
#else
		uint32_t roms, rome;

		roms = LPC55S6X_ROM_BASE;
		rome = LPC55S6X_ROM_BASE + LPC55S6X_ROM_SIZE;
		if (!lpc_dwt_set_stop(cm, 0, roms, rome - 1u))
			DBFPRINTF("Failed to write DWT\n");
#endif /* (LPC55S6X_HALT_IN_ROM == 0) */
	}

	/* Perform the reset. */
	if (!nxp_debug_mailbox_reset(ls->ls_mbox))
		DBFPRINTF("Failed to reset\n");

	rtos_task_sleep(5);

	if (and_halt) {
		uint32_t reg;

		lpc_dwt_clear_stop(cm, 0);
		lpc_dwt_clear_stop(cm, 2);

		/*
		 * Did the core stop?
		 */
		if (lpc_scs_read(cm, CMSCS_REG_DHCSR, &reg) < 0) {
			DBFPRINTF("Failed to read DHCSR\n");
			return -1;
		}

		if ((reg & CMSCS_DHCSR_S_HALT) == 0) {
			DBFPRINTF("Core did not halt!\n");

			if (lpc_scs_write(cm, CMSCS_REG_DHCSR,
			    CMSCS_DHCSR_DBGKEY | CMSCS_DHCSR_C_HALT |
			    CMSCS_DHCSR_C_DEBUGEN) < 0) {
				DBFPRINTF("Failed to force core to halt\n");
				return -1;
			}
		}
	}

	return 0;
}

static int
soc_lpc55s6x_core_reset(cortexm_t cm, int and_halt)
{
	int rv;

	assert(cm->cm_soc != NULL);

	rv = soc_lpc55s6x_core_reset_impl(cm, and_halt);
	if (rv < 0) {
		DBFPRINTF("LPC55S6x core reset%s failed\n",
		    and_halt ? " and halt" : "");
		(void) TARGET_LINK_ERROR(cm->cm_target);
	} else {
		DBFPRINTF("LPC55S6x core reset%s succeeded\n",
		    and_halt ? " and halt" : "");
	}

	return rv;
}

static void
soc_lpc55s6x_free(cortexm_t cm)
{
	struct lpc55s6x_state *ls;

	if ((ls = cm->cm_soc) != NULL) {
		if (ls->ls_mbox != NULL)
			nxp_debug_mailbox_detach(ls->ls_mbox);
		zone_free(ls);
	}
}

static void
lpc_add_memory(cortexm_t cm, unsigned int type, target_addr_t base,
    uint32_t len, const char *name_ns, const char *name_s)
{

	if (name_ns != NULL)
		target_add_memory(cm->cm_target, type, base, len, name_ns);

	if (name_s != NULL && cortexm_v8m_secure_debug_available(cm)) {
		type |= TARGET_MEM_EXCLUSIVE;

		/* Don't place Flash applets in SRAM secure aliases. */
		if ((type & TARGET_MEM_TYPE_MASK) == TARGET_MEM_RAM)
			type |= TARGET_MEM_NO_APPLET;

		target_add_memory(cm->cm_target, type, base | 0x10000000u, len,
		name_s);
	}
}

int
soc_lpc55s6x_attach(target_t t)
{
	char soc_name[TARGET_NAME_LEN_MAX];
	char revid_string[TARGET_NAME_LEN_MAX], rom_ver[TARGET_NAME_LEN_MAX];
	const struct lpc_mem_config *mc;
	struct lpc55s6x_state *ls;
	uint32_t device_id, dieid;
	target_addr_t a;
	cortexm_t cm;

	/*
	 * These devices have an NXP ROM table so quickly check if it exists.
	 */
	if (soc_lpc55s6x_rom_lookup(t) == false) {
		DBFPRINTF("ROM lookup failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	cm = (cortexm_t)t->t_core;
	a = LPC55S6X_SYSCON_DEV_ID;
	if (cortexm_v8m_secure_debug_available(cm))
		a |= 0x10000000u;
	if (TARGET_REG_READ32(t, a, &device_id) < 0) {
		DBFPRINTF("Failed to read DEVICE_ID\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	a = LPC55S6X_SYSCON_DIEID;
	if (cortexm_v8m_secure_debug_available(cm))
		a |= 0x10000000u;
	if (TARGET_REG_READ32(t, a, &dieid) < 0) {
		DBFPRINTF("Failed to read DIEID\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	cm->cm_soc = NULL;
	cm->cm_soc_ctl = NULL;
	cm->cm_soc_free = NULL;
	cm->cm_core_reset = soc_lpc55s6x_core_reset;

	/*
	 * Stop here if probing the second core; it's normally held in
	 * reset on startup so we can't probe too far...
	 */
	if (t->t_index != 0) {
		t->t_flags |= TARGET_FLAG_FLASH_RDONLY;
		cm->cm_flags |= CORTEXM_FLAG_NO_RESET;
		return CORTEXM_SOC_ATTACH_OK;
	}

	if ((ls = zone_malloc(sizeof(*ls))) == NULL) {
		DBFPRINTF("Out of memory!\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if ((ls->ls_mbox = nxp_debug_mailbox_attach()) == NULL) {
		DBFPRINTF("Out of memory!\n");
		zone_free(ls);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	cm->cm_soc = ls;
	cm->cm_soc_free = soc_lpc55s6x_free;

	/*
	 * There is no documented method to determine Flash memory size
	 * programmatically. However, there is an undocumented SYSCON
	 * register at (volatile uint8_t *)0x50000fe0 which is used by
	 * the ROM to detect Flash size. We use this method here.
	 */
	mc = soc_lpc55s66x_mem_config(t);
	assert(mc != NULL);

	lpc_add_memory(cm, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
	    LPC55S6X_SRAMX_BASE, LPC55S6X_SRAMX_SIZE, "SRAM-X",
	    "SRAM-X secure alias");

	lpc_add_memory(cm, TARGET_MEM_RAM, LPC55S6X_SRAM0_BASE,
	    LPC55S6X_SRAM0_SIZE, "SRAM0", "SRAM0 secure alias");

	if (LPC_RAM_SIZE(mc->mc_ram1) != 0) {
		lpc_add_memory(cm, TARGET_MEM_RAM, LPC55S6X_SRAM1_BASE,
		    LPC_RAM_SIZE(mc->mc_ram1), "SRAM1", "SRAM1 secure alias");
	}
	if (LPC_RAM_SIZE(mc->mc_ram2) != 0) {
		lpc_add_memory(cm, TARGET_MEM_RAM, LPC55S6X_SRAM2_BASE,
		    LPC_RAM_SIZE(mc->mc_ram2), "SRAM2", "SRAM2 secure alias");
	}
	if (LPC_RAM_SIZE(mc->mc_ram3) != 0) {
		lpc_add_memory(cm, TARGET_MEM_RAM, LPC55S6X_SRAM3_BASE,
		LPC_RAM_SIZE(mc->mc_ram3), "SRAM3", "SRAM3 secure alias");
	}
	if (LPC_RAM_SIZE(mc->mc_ram4) != 0) {
		lpc_add_memory(cm, TARGET_MEM_RAM, LPC55S6X_SRAM4_BASE,
		LPC_RAM_SIZE(mc->mc_ram4), "SRAM4", "SRAM4 secure alias");
	}

	lpc_add_memory(cm, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
	    LPC55S6X_USB_SRAM_BASE, LPC55S6X_USB_SRAM_SIZE, "USB SRAM",
	    "USB SRAM secure alias");

	if ((ls->ls_flash_size = LPC_FLASH_SIZE(mc)) != 0) {
		flash_nxp_lpc5x_attach(t, LPC55S6X_FLASH_REGS,
		    LPC55S6X_FLASH_BASE_NS, ls->ls_flash_size, 0, NULL);

		if (cortexm_v8m_secure_debug_available(cm)) {
			flash_nxp_lpc5x_attach(t, LPC55S6X_FLASH_REGS,
			    LPC55S6X_FLASH_BASE_S, ls->ls_flash_size,
			    TARGET_MEM_EXCLUSIVE, "Flash secure alias");
		}
	}

	lpc_add_memory(cm, TARGET_MEM_ROM, LPC55S6X_ROM_BASE, LPC55S6X_ROM_SIZE,
	    NULL, "Boot ROM");

	lpc_add_memory(cm, TARGET_MEM_ROM, LPC55S6X_CFPA_BASE,
	    LPC55S6X_CFPA_SIZE, "CFPA", NULL);

	lpc_add_memory(cm, TARGET_MEM_ROM, LPC55S6X_CMPA_KSA_BASE,
	    LPC55S6X_CMPA_KSA_SIZE, "CMPA and KSA", NULL);

	lpc_add_memory(cm, TARGET_MEM_ROM, LPC55S6X_NMPA_BASE,
	    LPC55S6X_NMPA_SIZE, "NMPA", NULL);

	strncpy(soc_name, LPC55S6X_PART_PREFIX, sizeof(soc_name));

	snprintf(revid_string, sizeof(revid_string), "REV_ID %" PRIx32,
	    LPC55S6X_DIEID_REV(dieid));
	switch (LPC55S6X_DEV_ID_ROM(device_id)) {
	case 0:
		strcpy(rom_ver, "ROM version 0A");
		break;
	case 1:
		strcpy(rom_ver, "ROM version 1B");
		break;
	default:
		snprintf(rom_ver, sizeof(rom_ver), "Unknown ROM (%" PRIx32 ")",
		    LPC55S6X_DEV_ID_ROM(device_id));
		break;
	}

	target_register_name(LPC55S6X_PART_VENDOR, soc_name, rom_ver,
	    revid_string);

	return CORTEXM_SOC_ATTACH_OK;
}
