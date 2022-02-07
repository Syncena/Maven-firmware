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
#include "soc_stm32f7.h"
#include "flash_stm_efmi.h"
#include "tmon.h"
#include "hardware.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#define STM32F7_ROM_PIDR_7x7	0x0000a0451ull
#define STM32F7_ROM_MASK_7x7	0xf000fffffull
#define STM32F7_ROM_PIDR_7x6	0x0000a0449ull
#define STM32F7_ROM_MASK_7x6	0xf000fffffull
#define	STM32F7_DBGMCU_IDCODE	0xe0042000u
#define	STM32F7_IDCODE_DEV(x)	((unsigned int)((x) & 0xfffu))
#define	STM32F7_IDCODE_REV(x)	((unsigned int)(((x) >> 16) & 0xffffu))
#define	STM32F7_MEM_SIZE	0x1ff0f442u
#define	STM32F7_PACKAGE_REG	0x1fff7bf0u
#define	STM32F7_PACKAGE(x)	(((x) >> 8) & 0x7u)

#define	STM32F7_EFMI_REGBASE	0x40023c00u

#define	STM32F7_PART_VENDOR	"STMicroelectronics"
#define	STM32F7_PART_PREFIX	"STM32F7"

struct stm32f7_state {
	void *ss_flash_ctx;
	void *ss_otp_ctx;
	void *ss_option_ctx;
};

static int
soc_stm32f7_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct stm32f7_state *ss = cm->cm_soc;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_rdp_query(ss->ss_option_ctx, arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_rdp_set(ss->ss_option_ctx, arg);
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_stm32f7_free(cortexm_t cm)
{
	struct stm32f7_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

static bool
soc_stm32f7_rom_lookup(target_t t)
{
	struct adiv5_memap_rom_table_lookup rt;

	rt.rt_pidr = STM32F7_ROM_PIDR_7x7;
	rt.rt_pidr_mask = STM32F7_ROM_MASK_7x7;

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 1) {
		DBFPRINTF("STM32F7x7\n");
		return true;
	}

	rt.rt_pidr = STM32F7_ROM_PIDR_7x6;
	rt.rt_pidr_mask = STM32F7_ROM_MASK_7x6;

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 1) {
		DBFPRINTF("STM32F7x6\n");
		return true;
	}

	return false;
}

int
soc_stm32f7_attach(target_t t)
{
	struct stm32f7_state *ss;
	struct flash_stm_efmi_desc ed;
	uint16_t flash_size, ram_size, dtcmram_size, itcmram_size;
	uint32_t ram_base, dtcmram_base, itcmram_base;
	uint32_t idcode;
	const char *part;
	char soc_name[TARGET_NAME_LEN_MAX];
	char revid_string[TARGET_NAME_LEN_MAX];
	cortexm_t cm;

	/*
	 * These devices have an STMicro ROM table so quickly check if
	 * it exists.
	 */
	if (soc_stm32f7_rom_lookup(t) == false) {
		DBFPRINTF("ROM lookup failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * Mostly use the officially-sanctioned method of probing for a
	 * supported part. STMicro have made it extremely difficult to
	 * distinguish between members of this family, particularly when
	 * it comes to figuring out RAM size.
	 */
	if (TARGET_REG_READ32(t, STM32F7_DBGMCU_IDCODE, &idcode) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("IDCODE read failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (TARGET_REG_READ16(t, STM32F7_MEM_SIZE, &flash_size) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("Memsize read failed\n");
		flash_size = 0;
		/* Assume the target is at RDP level 1 */
	}

	DBFPRINTF("IDCODE 0x%08" PRIx32 ", Flash %" PRIu16 " KB\n", idcode,
	    flash_size);

	part = NULL;
	ram_size = 0;
	ram_base = 0x20000000u;
	dtcmram_size = 0;
	dtcmram_base = 0;
	itcmram_size = 0;
	itcmram_base = 0;

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	memset(&ed, 0, sizeof(ed));
	ed.ed_target = t;
	ed.ed_regbase = STM32F7_EFMI_REGBASE;
	ed.ed_size_shift = 0;
	ed.ed_instance = 0;
	ed.ed_mass_erase_time = 32000u;
	ed.ed_sector_erase_time = 4200u;

	switch (STM32F7_IDCODE_DEV(idcode)) {
	case 0x449:
		/* STM32F7[45]xxx */
		part = "[45]xxx";
		ram_size = 256;
		ram_base = 0x20010000u;
		dtcmram_size = 64;
		dtcmram_base = 0x20000000u;
		itcmram_size = 16;
		itcmram_base = 0x00000000u;
		ed.ed_nopts = 2;
		ed.ed_optcr_mask[0] = 0xc0fffffcu;
		ed.ed_optcr_mask[1] = 0xffffffffu;
		break;

	case 0x451:
		/* STM32F7[67]xxx */
		part = "[67]xxx";
		ram_size = 384;
		ram_base = 0x20020000u;
		dtcmram_size = 128;
		dtcmram_base = 0x20000000u;
		itcmram_size = 16;
		itcmram_base = 0x00000000u;
		ed.ed_nopts = 2;
		ed.ed_optcr_mask[0] = 0xfffffffcu;
		ed.ed_optcr_mask[1] = 0xffffffffu;
		break;

	case 0x452:
		/* STM32F7[23]xxx */
		part = "[23]xxx";
		ram_size = 192;
		ram_base = 0x20010000u;
		dtcmram_size = 64;
		dtcmram_base = 0x20000000u;
		itcmram_size = 16;
		itcmram_base = 0x00000000u;
		ed.ed_nopts = 3;
		ed.ed_optcr_mask[0] = 0xc0fffffcu;
		ed.ed_optcr_mask[1] = 0xffffffffu;
		ed.ed_optcr_mask[2] = 0x800000ffu;
		break;

	default:
		/* Unsupported flavour */
		break;
	}

	if (part == NULL) {
		zone_free(ss);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	target_add_memory(t, TARGET_MEM_RAM, ram_base,
	    (uint32_t)ram_size * 1024u, NULL);

	if (dtcmram_size != 0) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    dtcmram_base, (uint32_t)dtcmram_size * 1024u, "DTCM");
	}

	if (itcmram_size != 0) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    itcmram_base, (uint32_t)itcmram_size * 1024u, "ITCM");
	}

	if (flash_size) {
		ed.ed_type = FLASH_STM_EFMI_FLASH;
		ed.ed_flash_base = 0x08000000u;
		ed.ed_flash_size = flash_size * 1024u;
		ed.ed_max_page_size = 32u;
		ed.ed_sector_small = 32u * 1024u;
		ed.ed_sector_med = 128u * 1024u;
		ed.ed_sector_large = 256u * 1024u;
		ss->ss_flash_ctx = flash_stm_efmi_attach(&ed);

		ed.ed_type = FLASH_STM_EFMI_OTP;
		ed.ed_flash_base = 0x1ff0f000u;
		ed.ed_flash_size = 0x400;	/* EFMI driver will add 0x20 */
		ed.ed_otp_blocks = 32u;
		ss->ss_otp_ctx = flash_stm_efmi_attach(&ed);
	}

	ed.ed_type = FLASH_STM_EFMI_OBYTES;
	ed.ed_flash_base = 0x1fff0000u;
	ed.ed_flash_size = ed.ed_nopts * 16;
	ed.ed_max_page_size = 32u;
	ss->ss_option_ctx = flash_stm_efmi_attach(&ed);

	snprintf(soc_name, sizeof(soc_name), STM32F7_PART_PREFIX "%s", part);
	snprintf(revid_string, sizeof(revid_string), "REV_ID 0x%04X",
	    STM32F7_IDCODE_REV(idcode));
	target_register_name(STM32F7_PART_VENDOR, soc_name, revid_string, NULL);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_stm32f7_ctl;
	cm->cm_soc_free = soc_stm32f7_free;

	return CORTEXM_SOC_ATTACH_OK;
}
