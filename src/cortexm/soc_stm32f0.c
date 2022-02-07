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

#include "cortexm.h"
#include "soc_stm32f0.h"
#include "flash_stm_fpec.h"
#include "hardware.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#define STM32F0_ROM_PIDR	0x0000a0440ull
#define STM32F0_ROM_MASK	0xf000fffffull
#define	STM32F0_DBGMCU_IDCODE	0x40015800u
#define	STM32F0_IDCODE_DEV(x)	((unsigned int)((x) & 0xfffu))
#define	STM32F0_IDCODE_REV(x)	((unsigned int)(((x) >> 16) & 0xffffu))
#define	STM32F0_FLASH_SIZE	0x1ffff7ccu

static const char stm_vendor_string[] = "STMicroelectronics";
static const char stm_f0_part_prefix[] = "STM32F0";

struct stm32f0_state {
	void *ss_flash_ctx;
	void *ss_option_ctx;
};

static int
soc_stm32f0_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct stm32f0_state *ss = cm->cm_soc;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_fpec_rdp_query(ss->ss_option_ctx, arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_fpec_rdp_set(ss->ss_option_ctx, arg);
		break;

	case TARGET_CTL_UNLOCK:
		*cmd = TARGET_CTL_NOP;
		rv = flash_stm_fpec_rdp_clear(ss->ss_option_ctx, arg);
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_stm32f0_free(cortexm_t cm)
{
	struct stm32f0_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

int
soc_stm32f0_attach(target_t t)
{
	struct stm32f0_state *ss;
	uint32_t idcode;
	uint16_t flash_size, ram_size;
	char suff, *soc_name, revid[TARGET_NAME_LEN_MAX];
	struct adiv5_memap_rom_table_lookup rt;
	const char *part;
	cortexm_t cm;

	/*
	 * These devices have an STMicro ROM table so quickly check if
	 * it exists.
	 */
	rt.rt_pidr = STM32F0_ROM_PIDR;
	rt.rt_pidr_mask = STM32F0_ROM_MASK;
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 0)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/*
	 * Mostly use the officially-sanctioned method of probing for a
	 * supported part. STMicro have made it extremely difficult to
	 * distinguish between members of this family, particularly when
	 * it comes to figuring out RAM size.
	 */
	if (TARGET_REG_READ32(t, STM32F0_DBGMCU_IDCODE, &idcode) < 0) {
		(void) TARGET_LINK_ERROR(t);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (TARGET_REG_READ16(t, STM32F0_FLASH_SIZE, &flash_size) < 0) {
		(void) TARGET_LINK_ERROR(t);
		/*
		 * This could be because the device is protected, so
		 * don't sweat it too much.
		 */
		flash_size = 0;
	}

	if ((soc_name = zone_malloc(TARGET_NAME_LEN_MAX)) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	DBFPRINTF("IDCODE 0x%08" PRIx32 ", flash_size %04" PRIx16 "\n", idcode,
	    flash_size);

	ram_size = 0;
	part = NULL;

	/* Figure out the part suffix */
	switch (flash_size) {
	case 16:	suff = '4'; break;
	case 32:	suff = '6'; break;
	case 64:	suff = '8'; break;
	case 128:	suff = 'B'; break;
	case 256:	suff = 'C'; break;
	case 384:	suff = 'D'; break;
	case 512:	suff = 'E'; break;
	case 768:	suff = 'F'; break;
	case 1024:	suff = 'G'; break;
	default:	suff = 'x'; break;
	}

	switch (STM32F0_IDCODE_DEV(idcode)) {
	case 0x0440:
		/*
		 * STM32F051x4	16/8
		 * STM32F051x6	32/8
		 * STM32F030x8	64/8
		 * STM32F051x8	64/8
		 * STM32F058x8	64/8
		 */
		ram_size = 8;
		switch (flash_size) {
		case 16:
		case 32:
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s51x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
			break;
		case 64:
			snprintf(soc_name, TARGET_NAME_LEN_MAX,
			    "%s30x%c/51x%c/58x%c",
			    stm_f0_part_prefix, suff, suff, suff);
			part = soc_name;
			break;

		default:
			break;
		}
		break;

	case 0x0442:
		/*
		 * STM32F091xB	128/32
		 * STM32F030xC	256/32
		 * STM32F091xC	256/32
		 * STM32F098xC	256/32
		 */
		ram_size = 32;
		switch (flash_size) {
		case 128:
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s91x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
			break;
		case 256:
			snprintf(soc_name, TARGET_NAME_LEN_MAX,
			    "%s30x%c/91x%c/98x%c",
			    stm_f0_part_prefix, suff, suff, suff);
			part = soc_name;
			break;
		default:
			break;
		}
		break;

	case 0x0444:
		/*
		 * STM32F030F4	16/4
		 * STM32F031x4	16/4
		 * STM32F030x6	32/4
		 * STM32F031x6	32/4
		 * STM32F038x6	32/4
		 */
		ram_size = 4;
		switch (flash_size) {
		case 16:
			snprintf(soc_name, TARGET_NAME_LEN_MAX,
			    "%s30F%c/31x%c", stm_f0_part_prefix, suff, suff);
			part = soc_name;
			break;
		case 32:
			snprintf(soc_name, TARGET_NAME_LEN_MAX,
			    "%s30x%c/31x%c/38x%c",
			    stm_f0_part_prefix, suff, suff, suff);
			part = soc_name;
			break;
		default:
			break;
		}
		break;

	case 0x0445:
		/*
		 * STM32F042x4	16/6
		 * STM32F070x6	32/6
		 * STM32F042x6	32/6
		 * STM32F048x6	32/6
		 */
		ram_size = 6;
		switch (flash_size) {
		case 16:
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s42x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
			break;
		case 32:
			snprintf(soc_name, TARGET_NAME_LEN_MAX,
			    "%s70x%c/42x%c/48x%c",
			    stm_f0_part_prefix, suff, suff, suff);
			part = soc_name;
			break;
		default:
			break;
		}
		break;

	case 0x0448:
		/*
		 * STM32F070x6	32/6
		 * STM32F071x8	64/16
		 * STM32F072x8	64/16
		 * STM32F070xB	128/16
		 * STM32F071xB	128/16
		 * STM32F072xB	128/16
		 * STM32F078xB	128/16
		 */
		ram_size = 16;
		switch (flash_size) {
		case 32:
			ram_size = 6;
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s70x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
			break;
		case 64:
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s7[12]x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
			break;
		case 128:
			snprintf(soc_name, TARGET_NAME_LEN_MAX, "%s7[0128]x%c",
			    stm_f0_part_prefix, suff);
			part = soc_name;
		default:
			break;
		}
		break;

	default:
		/* Unsupported flavour */
		break;
	}

	if (part == NULL) {
		zone_free(soc_name);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL) {
		zone_free(soc_name);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	target_add_memory(t, TARGET_MEM_RAM, 0x20000000u,
	    (uint32_t)ram_size * 1024u, NULL);

	if (flash_size != 0) {
		u_int16_t page_size = (flash_size < 128) ? 1u : 2u;
		ss->ss_flash_ctx = flash_stm_fpec_attach(t,
		    FLASH_STM_FPEC_FLASH,
		    0x08000000u, 0x00000000u,
		    page_size * 1024u,
		    flash_size / page_size);
	}

	snprintf(revid, sizeof(revid), "REV_ID 0x%04x",
	    STM32F0_IDCODE_REV(idcode));
	target_register_name(stm_vendor_string, soc_name, revid, NULL);
	zone_free(soc_name);

	ss->ss_option_ctx = flash_stm_fpec_attach(t,
	    FLASH_STM_FPEC_OBYTES_TYPE2, 0x1ffff800u, 0x00000000u, 16, 1);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_stm32f0_ctl;
	cm->cm_soc_free = soc_stm32f0_free;

	return CORTEXM_SOC_ATTACH_OK;
}
