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
#include "soc_stm32f1.h"
#include "flash_stm_fpec.h"
#include "hardware.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#define STM32F1_ROM_PIDR	0x0000a0410ull
#define STM32F1_ROM_MASK	0xf000fffffull
#define	STM32F1_DBGMCU_IDCODE	0xe0042000u
#define	STM32F1_IDCODE_DEV(x)	((unsigned int)((x) & 0xfffu))
#define	STM32F1_IDCODE_REV(x)	((unsigned int)(((x) >> 16) & 0xffffu))
#define	STM32F1_MEM_SIZE	0x1ffff7e0u

struct srm32f1_mem_size {
	uint16_t ms_flash;
	uint16_t ms_resvd;
	uint8_t ms_x1;
	uint8_t ms_x2;
	uint8_t ms_ram;
	uint8_t ms_x3;
};

#define	STM32F1_PART_VENDOR	"STMicroelectronics"
#define	STM32F1_PART_PREFIX	"STM32F1"

struct stm32f1_state {
	void *ss_flash_ctx;
	void *ss_option_ctx;
};

static __always_inline const char *
stm_f100x_probe_low(uint16_t flash_size, uint16_t *pram_size)
{
	uint16_t ram_size;

	/* F100x Value Line, Low/Medium density */
	switch (flash_size) {
	case 16:
		ram_size = 4;
		break;
	case 32:
		ram_size = 4;
		break;
	case 64:
		ram_size = 8;
		break;
	case 128:
		ram_size = 8;
		break;
	default:
		ram_size = 4;
		break;
	}

	*pram_size = ram_size;

	return "00x";
}

static __always_inline const char *
stm_f100x_probe_high(uint16_t flash_size, uint16_t *pram_size)
{
	uint16_t ram_size;

	/* F100x Value Line, High density */
	switch (flash_size) {
	case 256:
		ram_size = 24;
		break;
	case 384:
		ram_size = 32;
		break;
	case 512:
		ram_size = 32;
		break;
	default:
		ram_size = 24;
		break;
	}

	*pram_size = ram_size;

	return "00x";
}

static __always_inline const char *
stm_f10x_probe_low(uint16_t flash_size, uint16_t *pram_size)
{
	const char *part = "0[123]x";
	uint16_t ram_size;

	/* F10x Low density */
	switch (flash_size) {
	case 16:
		/* 101/102/103 */
		ram_size = 6;	/* XXX: 101/102 have 4K */
		break;
	case 32:
		/* 101/102/103 */
		ram_size = 10;	/* XXX: 101/102 have 6K */
		break;
	default:
		ram_size = 4;
		break;
	}

	*pram_size = ram_size;

	return part;
}

static __always_inline const char *
stm_f10x_probe_med(uint16_t flash_size, uint16_t *pram_size)
{
	const char *part = "0[123]x";
	uint16_t ram_size;

	/* F10x Medium density */
	switch (flash_size) {
	case 64:
		/* 101/102/103 */
		ram_size = 20;	/* XXX: 101/102 have 10K */
		break;
	case 128:
		/* 101/102/103 */
		ram_size = 20;	/* XXX: 101/102 have 16K */
		break;
	default:
		ram_size = 16;
		break;
	}

	*pram_size = ram_size;

	return part;
}

static __always_inline const char *
stm_f10x_probe_high(uint16_t flash_size, uint16_t *pram_size)
{
	const char *part = "0[13]x";
	uint16_t ram_size;

	/* F10x High density */
	switch (flash_size) {
	case 256:
		/* 101/103 */
		ram_size = 64;	/* XXX: 101 has 32K */
		break;
	case 384:
		/* 101/103 */
		ram_size = 64;	/* XXX: 101 has 48K */
		break;
	case 512:
		/* 101/103 */
		ram_size = 64;	/* XXX: 101 has 48K */
		break;
	default:
		ram_size = 32;
		break;
	}

	*pram_size = ram_size;

	return part;
}

static __always_inline const char *
stm_f10x_probe_xl(uint16_t flash_size, uint16_t *pram_size)
{
	const char *part = "0[13]x";
	uint16_t ram_size;

	/* F10x High density */

	switch (flash_size) {
	case 768:
		/* 101/103 */
		ram_size = 96;	/* XXX: 101 has 80K */
		break;
	case 1024:
		/* 101/103 */
		ram_size = 96;	/* XXX: 103 has 80K */
		break;
	default:
		ram_size = 80;
		break;
	}

	*pram_size = ram_size;

	return part;
}

static int
soc_stm32f1_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct stm32f1_state *ss = cm->cm_soc;
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
soc_stm32f1_free(cortexm_t cm)
{
	struct stm32f1_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

int
soc_stm32f1_cm3_attach(target_t t)
{
	struct stm32f1_state *ss;
	struct srm32f1_mem_size ms;
	uint32_t idcode;
	uint16_t flash_size, page_size, ram_size;
	struct adiv5_memap_rom_table_lookup rt;
	const char *part;
	cortexm_t cm;
	char soc_name[TARGET_NAME_LEN_MAX];
	char revid_string[TARGET_NAME_LEN_MAX];
	char suff;

	/*
	 * These devices have an STMicro ROM table so quickly check if
	 * it exists.
	 */
	rt.rt_pidr = STM32F1_ROM_PIDR;
	rt.rt_pidr_mask = STM32F1_ROM_MASK;
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 0)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/*
	 * Mostly use the officially-sanctioned method of probing for a
	 * supported part. STMicro have made it extremely difficult to
	 * distinguish between members of this family, particularly when
	 * it comes to figuring out RAM size.
	 */
	if (TARGET_REG_READ32(t, STM32F1_DBGMCU_IDCODE, &idcode) < 0) {
		(void) TARGET_LINK_ERROR(t);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if (TARGET_MEM_READ(t, STM32F1_MEM_SIZE, sizeof(ms), &ms) < 0) {
		(void) TARGET_LINK_ERROR(t);
		/*
		 * This could be because the device is protected, so
		 * don't sweat it too much.
		 */
		memset(&ms, 0, sizeof(ms));
	}

	DBFPRINTF("IDCODE 0x%08" PRIx32 "\n", idcode);
	DBPRINTF("\tms_flash %" PRIu16 ", ms_ram %" PRIu16 "\n", ms.ms_flash,
	    ms.ms_ram);
	DBPRINTF("\tms_x1 0x%x, ms_x2 0x%x, ms_x3 0x%x\n",
	    (unsigned int)ms.ms_x1, (unsigned int)ms.ms_x2,
	    (unsigned int)ms.ms_x3);

	part = NULL;
	ram_size = 0;
	flash_size = ms.ms_flash;
	page_size = 1u;

	switch (STM32F1_IDCODE_DEV(idcode)) {
	case 0x0420:
		/* Low/Medium/High density STM32F100x */
		part = stm_f100x_probe_low(flash_size, &ram_size);
		break;

	case 0x0428:
		/* High density STM32F100x */
		part = stm_f100x_probe_high(flash_size, &ram_size);
		page_size = 2u;
		break;

	case 0x0412:
		/* Low density STM32F10[1-3]xy */
		part = stm_f10x_probe_low(flash_size, &ram_size);
		break;

	case 0x0410:
		/* Medium density STM32F10[1-3]xy */
		part = stm_f10x_probe_med(flash_size, &ram_size);
		break;

	case 0x0414:
		/* High density STM32F10[1-3]xy */
		part = stm_f10x_probe_high(flash_size, &ram_size);
		page_size = 2u;
		break;

	case 0x0430:
		/* XL density STM32F10[1-3]xy */
		part = stm_f10x_probe_xl(flash_size, &ram_size);
		page_size = 2u;
		break;

	case 0x0418:
		/* Connectivity device STM32F10[57]xy */
		/* Fortunately RAM size is identical for all these */
		part = "0[57]x";
		ram_size = 64;

		switch (flash_size) {
		case 64:
		case 128:
		case 256:
			/*FALLTHROUGH*/
			page_size = 2u;
			break;
		default:
			break;
		}
		break;

	default:
		/* Unsupported flavour */
		break;
	}

	if (part == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

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

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_BEST_GUESS,
	    0x20000000u, (uint32_t)ram_size * 1024u, NULL);

	if (flash_size != 0) {
		ss->ss_flash_ctx = flash_stm_fpec_attach(t,
		    FLASH_STM_FPEC_FLASH,
		    0x08000000u, 0x00000000u,
		    page_size * 1024u,
		    flash_size / page_size);
	}

	ss->ss_option_ctx = flash_stm_fpec_attach(t,
	    FLASH_STM_FPEC_OBYTES_TYPE1, 0x1ffff800u, 0x00000000u, 16, 1);

	snprintf(soc_name, sizeof(soc_name), STM32F1_PART_PREFIX "%s%c", part,
	    suff);
	snprintf(revid_string, sizeof(revid_string), "REV_ID 0x%04X",
	    STM32F1_IDCODE_REV(idcode));
	target_register_name(STM32F1_PART_VENDOR, soc_name, revid_string, NULL);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_stm32f1_ctl;
	cm->cm_soc_free = soc_stm32f1_free;

	return CORTEXM_SOC_ATTACH_OK;
}
