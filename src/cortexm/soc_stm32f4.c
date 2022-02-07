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

#include "cortexm.h"
#include "soc_stm32f4.h"
#include "flash_stm_efmi.h"
#include "tmon.h"
#include "hardware.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#define STM32F4_ROM_PIDR	0x0000a0411ull
#define STM32F4_ROM_MASK	0xf000fffffull
#define	STM32F4_DBGMCU_IDCODE	0xe0042000u
#define	STM32F4_IDCODE_DEV(x)	((unsigned int)((x) & 0xfffu))
#define	STM32F4_IDCODE_REV(x)	((unsigned int)(((x) >> 16) & 0xffffu))
#define	STM32F4_MEM_SIZE	0x1fff7a22u

#define	STM32F4_EFMI_REGBASE	0x40023c00u

#define	STM32F4_PART_VENDOR	"STMicroelectronics"
#define	STM32F4_PART_PREFIX	"STM32F4"

struct stm32f4_state {
	void *ss_flash_ctx;
	void *ss_otp_ctx;
	void *ss_option_ctx[2];
};

static int
soc_stm32f4_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct stm32f4_state *ss = cm->cm_soc;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_rdp_query(ss->ss_option_ctx[0], arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_rdp_set(ss->ss_option_ctx[0], arg);
		break;

	case TARGET_CTL_UNLOCK:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_efmi_rdp_clear(ss->ss_option_ctx[0], arg);
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_stm32f4_free(cortexm_t cm)
{
	struct stm32f4_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

static bool
soc_stm32f4_dual_bank_dbm1(uint32_t optcr)
{

	return (optcr & (1u << 30)) != 0;
}

static bool
soc_stm32f4_dual_bank_true(uint32_t optcr)
{

	(void) optcr;

	return true;
}

int
soc_stm32f4_attach(target_t t)
{
	struct stm32f4_state *ss;
	struct flash_stm_efmi_desc ed;
	uint16_t flash_size, i, ram_size, ccmram_size;
	struct adiv5_memap_rom_table_lookup rt;
	uint32_t idcode;
	const char *part;
	char soc_name[TARGET_NAME_LEN_MAX];
	char revid_string[TARGET_NAME_LEN_MAX];
	cortexm_t cm;

	/*
	 * These devices have an STMicro ROM table so quickly check if
	 * it exists.
	 */
	rt.rt_pidr = STM32F4_ROM_PIDR;
	rt.rt_pidr_mask = STM32F4_ROM_MASK;
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 0) {
		DBFPRINTF("ROM lookup failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	/*
	 * Mostly use the officially-sanctioned method of probing for a
	 * supported part. STMicro have made it extremely difficult to
	 * distinguish between members of this family, particularly when
	 * it comes to figuring out RAM size.
	 */
	if (TARGET_REG_READ32(t, STM32F4_DBGMCU_IDCODE, &idcode) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("IDCODE read failed\n");
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (TARGET_REG_READ16(t, STM32F4_MEM_SIZE, &flash_size) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBFPRINTF("Memsize read failed\n");
		flash_size = 0;
		/* Assume the target is at RDP level 1 */
	}

	DBFPRINTF("IDCODE 0x%08" PRIx32 ", Flash %" PRIu16 " KB\n", idcode,
	    flash_size);

	part = NULL;
	ram_size = 0;
	ccmram_size = 0;

	memset(&ed, 0, sizeof(ed));
	ed.ed_target = t;
	ed.ed_regbase = STM32F4_EFMI_REGBASE;
	ed.ed_size_shift = 0;
	ed.ed_instance = 0;
	ed.ed_mass_erase_time = 32000u;
	ed.ed_sector_erase_time = 4200u;
	ed.ed_nopts = 1;

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	switch (STM32F4_IDCODE_DEV(idcode)) {
	case 0x411:
		/*
		 * IDCODE Erratum for Rev-A device.
		 * Note that this clashes with the IDCODE for STM32F20xx but
		 * since that device is Cortex-M3 and the STM32F4xx is M4 then
		 * there's no confusion. The Cortex driver probes SoCs
		 * according to their CPU core, so we will never be called
		 * if it finds an M3.
		 */
		/*FALLTHROUGH*/

	case 0x413:
		/* STM32F40[57]xx/STM32F41[57]xx */
		ram_size = 128;
		ccmram_size = 64;
		part = "[01][57]xx";
		ed.ed_optcr_mask[0] = 0x0fffffecu;
		break;

	case 0x419:
		/* STM32F42xxx and STM32F43xxx */
		ram_size = 192;
		ccmram_size = 64;
		part = "[23]xxx";
		if (flash_size <= 1024u) {
			ed.ed_dual_bank = soc_stm32f4_dual_bank_dbm1;
			ed.ed_optcr_mask[0] = 0xcffffffcu;
		} else {
			ed.ed_dual_bank = soc_stm32f4_dual_bank_true;
			ed.ed_optcr_mask[0] = 0x8ffffffcu;
		}
		ed.ed_optcr_mask[1] = 0x0fff0000u;
		ed.ed_nopts = 2;
		break;

	case 0x421:
		/* STM32F446xx */
		ram_size = 128;
		part = "446xx";
		ed.ed_optcr_mask[0] = 0x80ffffecu;
		break;

	case 0x423:
		/* STM32F401xB/C */
		ram_size = 64;
		part = "01xB/C";
		ed.ed_optcr_mask[0] = 0x803fffecu;
		break;

	case 0x431:
		/* STM32F411xC/E */
		ram_size = 128;
		part = "11xC/E";
		ed.ed_optcr_mask[0] = 0x80ffffecu;
		break;

	case 0x433:
		/* STM32F401xD/E */
		ram_size = 96;
		part = "01xD/E";
		ed.ed_optcr_mask[0] = 0x80ffffecu;
		break;

	case 0x434:
		/* STM32F4[67]9xx */
		ram_size = 320;
		part = "[67]9xx";
		if (flash_size <= 1024u) {
			ed.ed_dual_bank = soc_stm32f4_dual_bank_dbm1;
			ed.ed_optcr_mask[0] = 0xcffffffcu;
		} else {
			ed.ed_dual_bank = soc_stm32f4_dual_bank_true;
			ed.ed_optcr_mask[0] = 0x8ffffffcu;
		}
		ed.ed_optcr_mask[1] = 0x0fff0000u;
		ed.ed_nopts = 2;
		break;

	case 0x441:
		/* STM32F412 */
		ram_size = 256;
		part = "12";
		ed.ed_optcr_mask[0] = 0x8fffffecu;
		break;

	case 0x458:
		/* STM32F410 */
		ram_size = 32;
		part = "10";
		ed.ed_optcr_mask[0] = 0x80ffffecu;
		break;

	case 0x463:
		/* STM32F4[12]3 */
		ram_size = 320;
		part = "4[12]3";
		ed.ed_optcr_mask[0] = 0xffffffecu;
		break;

	default:
		/* Unsupported flavour */
		break;
	}

	if (part == NULL) {
		zone_free(ss);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	target_add_memory(t, TARGET_MEM_RAM, 0x20000000u,
	    (uint32_t)ram_size * 1024u, NULL);

	if (ccmram_size != 0) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    0x10000000u, (uint32_t)ccmram_size * 1024u, "CCM");
	}

	if (flash_size) {
		ed.ed_type = FLASH_STM_EFMI_FLASH;
		ed.ed_flash_base = 0x08000000u;
		ed.ed_flash_size = flash_size * 1024u;
		ed.ed_max_page_size = 32u;
		ed.ed_sector_small = 0x4000u;
		ed.ed_sector_med = 0x10000u;
		ed.ed_sector_large = 0x20000u;
		ss->ss_flash_ctx = flash_stm_efmi_attach(&ed);

		ed.ed_type = FLASH_STM_EFMI_OTP;
		ed.ed_flash_base = 0x1fff7800u;
		ed.ed_flash_size = 0x200u;	/* EFMI driver will add 0x10 */
		ed.ed_otp_blocks = 16u;
		ss->ss_otp_ctx = flash_stm_efmi_attach(&ed);
	}

	ed.ed_type = FLASH_STM_EFMI_OBYTES;
	ed.ed_flash_base = 0x1fffc000u;
	ed.ed_flash_size = 0x10u;
	ed.ed_max_page_size = 16u;
	for (i = 0; i < ed.ed_nopts; i++) {
		ed.ed_flash_base = 0x1fffc000u - (i * 0x00010000u);
		ed.ed_instance = i;
		ss->ss_option_ctx[i] = flash_stm_efmi_attach(&ed);
	}

	snprintf(soc_name, sizeof(soc_name), STM32F4_PART_PREFIX "%s",
	    part);
	snprintf(revid_string, sizeof(revid_string), "REV_ID 0x%04X",
	    STM32F4_IDCODE_REV(idcode));
	target_register_name(STM32F4_PART_VENDOR, soc_name, revid_string, NULL);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_stm32f4_ctl;
	cm->cm_soc_free = soc_stm32f4_free;

	return CORTEXM_SOC_ATTACH_OK;
}
