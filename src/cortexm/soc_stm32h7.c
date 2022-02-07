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
#include "soc_stm32h7.h"
#include "flash_stm_efmi_h7.h"
#include "tmon.h"
#include "hardware.h"
#include "zone_alloc.h"

/*
 * Support for STM32H7xx devices.
 *
 * ToDo:
 *  -  Some of these are dual-core, and have boot options to disable the clock
 *     of either core at startup. We may need special hacks to enable the core
 *     clock if a debug session is started...
 *  -  Cortex code assumes AHBv3 bus, we may need to change this, particularly
 *     for M7s on an AXI4 bus.
 *  -  Option bytes.
 *  -  These devices support SFI. Perhaps we should too...
 */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#define STM32H7_ROM_PIDR_7x5_7	0x0000a0450ull
#define STM32H7_ROM_MASK_7x5_7	0xf000fffffull
#define	STM32H7_DBGMCU_IDCODE	0x5C001000
#define	STM32H7_IDCODE_DEV(x)	((unsigned int)((x) & 0xfffu))
#define	STM32H7_IDCODE_REV(x)	((unsigned int)(((x) >> 16) & 0xffffu))
#define	STM32H7_MEM_SIZE	0x1ff1e880u

/*
 * To read SYSCFG_PKGR, first enable SYSCFG clock in RCC_APB4ENR.
 */
#define	STM32H7_PACKAGE_REG	0x1fff7bf0u
#define	STM32H7_PACKAGE(x)	(((x) >> 8) & 0x7u)

#define	STM32H7_EFMI_REGBASE	0x52002000u
#define	STM32H7_RCC_APB4ENR	0x580244f4u
#define	STM32H7_RCC_APB4ENR_SYSCFGEN	(1u << 1)
#define	STM32H7_SYSCFG_REGBASE	0x58000400u
#define	STM32H7_SYSCFG_PKGR	(STM32H7_SYSCFG_REGBASE + 0x124u)
#define	STM32H7_SYSCFG_UR(x)	(STM32H7_SYSCFG_REGBASE + 0x300u + ((x)*4u))
#define	STM32H7_SYSCFG_RDP(ur)	((((ur) & 0xff0000u) == 0xaa0000) ? 0 : \
				 ((((ur) & 0xff0000u) == 0xcc0000) ? 2 : 1))
#define	STM32H7_SYSCFG_BKS(ur)	((ur) & 1u)

#define	STM32H7_PART_VENDOR	"STMicroelectronics"
#define	STM32H7_PART_PREFIX	"STM32H7"

struct stm32h7_state {
	void *ss_flash_ctx;
	void *ss_otp_ctx;
};

static int
soc_stm32h7_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct stm32h7_state *ss = cm->cm_soc;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_h7_rdp_query(ss->ss_flash_ctx, arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_h7_rdp_set(ss->ss_flash_ctx, arg);
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_stm32h7_free(cortexm_t cm)
{
	struct stm32h7_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

static bool
soc_stm32h7_rom_lookup(target_t t)
{
	struct adiv5_memap_rom_table_lookup rt;

	rt.rt_pidr = STM32H7_ROM_PIDR_7x5_7;
	rt.rt_pidr_mask = STM32H7_ROM_MASK_7x5_7;

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 1) {
		DBFPRINTF("STM32H7[45][57]\n");
		return true;
	}

	return false;
}

int
soc_stm32h7_attach(target_t t)
{
	struct stm32h7_state *ss;
	struct flash_stm_efmi_h7_desc ed;
	uint16_t flash_size, dtcmram_size, itcmram_size;
	uint16_t sram_axi_size, sram_size[2], sram_bkup_size;
	uint32_t dtcmram_base, itcmram_base;
	uint32_t sram_axi_base, sram_base[2], sram_bkup_base;
	const char *sram_desc[2];
	uint32_t idcode, tcmflag, pkgr, ur0, rcc_apb4enr;
	const char *part;
	char soc_name[TARGET_NAME_LEN_MAX];
	char revid_string[TARGET_NAME_LEN_MAX];
	cortexm_t cm;

	/*
	 * These devices have an STMicro ROM table so quickly check if
	 * it exists.
	 */
	if (soc_stm32h7_rom_lookup(t) == false) {
		DBFPRINTF("ROM lookup failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * Mostly use the officially-sanctioned method of probing for a
	 * supported part. STMicro have made it extremely difficult to
	 * distinguish between members of this family, particularly when
	 * it comes to figuring out RAM size.
	 */
	if (TARGET_REG_READ32(t, STM32H7_DBGMCU_IDCODE, &idcode) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("IDCODE read failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * We're fairly sure this is an STM32H7 device now. The remainder
	 * of the work is only required when attaching CPU0.
	 */
	if (t->t_index != 0) {
		t->t_flags |= TARGET_FLAG_FLASH_RDONLY;
		return CORTEXM_SOC_ATTACH_OK;
	}

	/* Enable SYSCFG temporarily so we can read its registers. */
	if (TARGET_REG_READ32(t, STM32H7_RCC_APB4ENR, &rcc_apb4enr) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("RCC_APB4ENR read failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if ((rcc_apb4enr & STM32H7_RCC_APB4ENR_SYSCFGEN) == 0 &&
	    TARGET_REG_WRITE32(t, STM32H7_RCC_APB4ENR,
	    rcc_apb4enr | STM32H7_RCC_APB4ENR_SYSCFGEN) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("RCC_APB4ENR write1 failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (TARGET_REG_READ32(t, STM32H7_SYSCFG_UR(0), &ur0) < 0 ||
	    TARGET_REG_READ32(t, STM32H7_SYSCFG_PKGR, &pkgr) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("UR0/pkgr read failed\n");
		(void)TARGET_REG_WRITE32(t, STM32H7_RCC_APB4ENR, rcc_apb4enr);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/* Restore original RCC_APB4ENR value. */
	(void)TARGET_REG_WRITE32(t, STM32H7_RCC_APB4ENR, rcc_apb4enr);
	(void) TARGET_LINK_ERROR(t);

	flash_size = 0;
	if (STM32H7_SYSCFG_RDP(ur0) == 0 &&
	    TARGET_REG_READ16(t, STM32H7_MEM_SIZE, &flash_size) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("Memsize read failed\n");
		/* Assume the target is at RDP level > 0 */
	}

	DBFPRINTF("IDCODE 0x%08" PRIx32 ", Flash %" PRIu16 " KB\n", idcode,
	    flash_size);

	/*
	 * XXX: Read SYSCFG_UR1 to determine core clock gating. This will
	 * need some additional target code changes to support cores in
	 * a 'stopped' state.
	 */

	part = NULL;
	tcmflag = 0;

	dtcmram_size = itcmram_size = 0;
	dtcmram_base = 0x20000000u;
	itcmram_base = 0x0u;

	sram_axi_size = sram_bkup_size = 0;
	sram_axi_base = sram_bkup_base = 0;

	sram_size[0] = sram_size[1] = 0;
	sram_base[0] = sram_base[1] = 0;
	sram_desc[0] = sram_desc[1] = NULL;

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* XXX: Default values, for now. */
	ed.ed_target = t;
	ed.ed_regbase = STM32H7_EFMI_REGBASE;
	ed.ed_flash_base = 0x08000000u;
	ed.ed_flash_size = flash_size * 1024u;
	ed.ed_sector_size = 128u * 1024u;
	ed.ed_max_page_size = 32u;
	ed.ed_mass_erase_time = 32000u;
	ed.ed_sector_erase_time = 4200u;
	ed.ed_swapped = STM32H7_SYSCFG_BKS(ur0) != 0;

	switch (STM32H7_IDCODE_DEV(idcode)) {
	case 0x450:
		/* STM32H7[45][57] */
		switch (pkgr & 0xfu) {
		case 0x2u:
		case 0x9u:
			part = "[45]7";
			break;
		case 0x3u:
		case 0x6u:
		case 0x7u:
		case 0xau:
			part = "[45]5";
			break;
		default:
			part = "[45][57]";
			break;
		}
		sram_axi_size = 512u;
		sram_axi_base = 0x24000000u;
		sram_bkup_size = 4u;
		sram_bkup_base = 0x38800000u;
		sram_size[0] = 288u;		/* SRAM1, 2, and 3 */
		sram_base[0] = 0x30000000u;
		sram_desc[0] = "SRAM1/2/3";
		sram_size[1] = 64u;		/* SRAM4 */
		sram_base[1] = 0x38000000u;
		sram_desc[1] = "SRAM4";
		if (t->t_index == 0) {
			/* Cortex-M7 core has tightly-coupled memories */
			dtcmram_size = 128u;
			dtcmram_base = 0x20000000u;
			itcmram_size = 64u;
			itcmram_base = 0x00000000u;
			tcmflag = TARGET_MEM_EXCLUSIVE;
		}
		break;

	default:
		/* Unsupported flavour */
		break;
	}

	if (part == NULL) {
		zone_free(ss);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (sram_axi_size != 0) {
		target_add_memory(t, TARGET_MEM_RAM, sram_axi_base,
		    (uint32_t)sram_axi_size * 1024u, "AXI SRAM");
	}

	for (u_int i = 0; i < (sizeof(sram_size) / sizeof(sram_size[0])); i++) {
		if (sram_size[0] != 0) {
			target_add_memory(t, TARGET_MEM_RAM, sram_base[i],
			    (uint32_t)sram_size[i] * 1024u, sram_desc[i]);
		}
	}

	if (dtcmram_size != 0) {
		target_add_memory(t,
		    tcmflag | TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    dtcmram_base, (uint32_t)dtcmram_size * 1024u, "DTCM");
	}

	if (itcmram_size != 0) {
		target_add_memory(t,
		    tcmflag | TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    itcmram_base, (uint32_t)itcmram_size * 1024u, "ITCM");
	}

	if (sram_bkup_size != 0) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    sram_bkup_base, (uint32_t)sram_bkup_size * 1024u,
		    "Backup SRAM");
	}

	if (flash_size)
		ss->ss_flash_ctx = flash_stm_efmi_h7_attach(&ed);

	snprintf(soc_name, sizeof(soc_name), STM32H7_PART_PREFIX "%s", part);
	snprintf(revid_string, sizeof(revid_string), "REV_ID 0x%04X",
	    STM32H7_IDCODE_REV(idcode));
	target_register_name(STM32H7_PART_VENDOR, soc_name, revid_string, NULL);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_stm32h7_ctl;
	cm->cm_soc_free = soc_stm32h7_free;

	return CORTEXM_SOC_ATTACH_OK;
}
