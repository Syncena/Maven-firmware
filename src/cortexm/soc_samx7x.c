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
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cortexm.h"
#include "soc_samx7x.h"
#include "hardware.h"
#include "flash_sam_efc.h"
#include "shell.h"
#include "zone_alloc.h"

SHELL_CMD_DECL_CTX(gpnvm, samx7x_cmd_gpnvm, "Clear/Set/Show GPNVM bits");

#define	SAMX7X_PART_VENDOR	"Microchip"
#define	SAMX7X_PART_PREFIX	"ATSAM"
#define	SAMX7X_PART_ID_LEN	8

#define	SAMX7X_ATMEL_ROM_PIDR	0x00009f000ull
#define	SAMX7X_ATMEL_ROM_MASK	0xf000fffffull

struct sam_id {
	const char si_part[SAMX7X_PART_ID_LEN];
	uint32_t si_chipid;
	uint32_t si_chipid_ex;
};

#define	SAMX7x_REG_CHIPID		0x400e0940u
#define	SAMX7x_EEFC_BASE		0x400e0c00u
#define	SAMX7x_EEFC_OFFSET		0x0u
#define	SAMX7x_ROM_BASE			0x00800000u
#define	SAMX7x_ROM_SIZE			0x8000u
#define	SAMX7x_ITCM_BASE		0x00000000u
#define	SAMX7x_DTCM_BASE		0x20000000u
#define	SAMX7x_SRAM_BASE		0x20400000u
#define	SAMX7x_FLASH_BASE		0x00400000u

static const uint16_t samx7x_sram_size[16] = {
	48, 192, 384, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};

#define	SAMX7X_CHIPID_SRAMSIZ(c)	samx7x_sram_size[(((c) >> 16) & 0x0fu)]
static const struct sam_id samx7x_ids[] = {
	{"E70Q21", 0xa1020e00u, 0x00000002},
	{"E70Q20", 0xa1020c00u, 0x00000002},
	{"E70Q19", 0xa10d0a00u, 0x00000002},
	{"E70N21", 0xa1020e00u, 0x00000001},
	{"E70N20", 0xa1020c00u, 0x00000001},
	{"E70N19", 0xa10d0a00u, 0x00000001},
	{"E70J21", 0xa1020e00u, 0x00000000},
	{"E70J20", 0xa1020c00u, 0x00000000},
	{"E70J19", 0xa10d0a00u, 0x00000000},

	{"S70Q21", 0xa1120e00u, 0x00000002},
	{"S70Q20", 0xa1120c00u, 0x00000002},
	{"S70Q19", 0xa11d0a00u, 0x00000002},
	{"S70N21", 0xa1120e00u, 0x00000001},
	{"S70N20", 0xa1120c00u, 0x00000001},
	{"S70N19", 0xa11d0a00u, 0x00000001},
	{"S70J21", 0xa1120e00u, 0x00000000},
	{"S70J20", 0xa1120c00u, 0x00000000},
	{"S70J19", 0xa11d0a00u, 0x00000000},

	{"V71Q21", 0xa1220e00u, 0x00000002},
	{"V71Q20", 0xa1220c00u, 0x00000002},
	{"V71Q19", 0xa12d0a00u, 0x00000002},
	{"V71N21", 0xa1220e00u, 0x00000001},
	{"V71N20", 0xa1220c00u, 0x00000001},
	{"V71N19", 0xa12d0a00u, 0x00000001},
	{"V71J21", 0xa1220e00u, 0x00000000},
	{"V71J20", 0xa1220c00u, 0x00000000},
	{"V71J19", 0xa12d0a00u, 0x00000000},

	{"V70Q20", 0xa1320c00u, 0x00000002},
	{"V70Q19", 0xa13d0a00u, 0x00000002},
	{"V70N20", 0xa1320c00u, 0x00000001},
	{"V70N19", 0xa13d0a00u, 0x00000001},
	{"V70J20", 0xa1320c00u, 0x00000000},
	{"V70J19", 0xa13d0a00u, 0x00000000},
};
#define	SAMX7X_NIDS		(sizeof(samx7x_ids) / sizeof(samx7x_ids[0]))
#define	SAMX7X_CHIPID_MASK	0xffffffe0u
#define	SAMX7X_WAIT_STATES	5

struct samx7x_state {
	void *ss_flash_ctx;
	bool ss_has_tcm;
};

static int
sam_read_reg32(target_t t, uint32_t addr, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(t, (target_addr_t)addr, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(t);

	return rv;
}

static uint32_t
sam_lookup_part(target_t t, const struct sam_id *si, int nids,
    uint32_t chip_id_reg, uint32_t chip_id_mask)
{
	char part[SAMX7X_PART_ID_LEN + 1];
	uint32_t chipid, chipid_ex;
	int i;
	char soc_name[TARGET_NAME_LEN_MAX];
	char id_string[TARGET_NAME_LEN_MAX];

	if (sam_read_reg32(t, chip_id_reg, &chipid) < 0 ||
	    sam_read_reg32(t, chip_id_reg + 4, &chipid_ex) < 0) {
		return 0;
	}

	while (nids--) {
		if (si->si_chipid == (chipid & chip_id_mask) &&
		    si->si_chipid_ex == chipid_ex) {
			for (i = 0; i < SAMX7X_PART_ID_LEN; i++)
				part[i] = si->si_part[i];
			part[i] = '\0';
			snprintf(soc_name, sizeof(soc_name),
			    SAMX7X_PART_PREFIX "%s", part);
			snprintf(id_string, sizeof(id_string),
			    "ID %08" PRIX32 " %08" PRIX32, chipid, chipid_ex);
			target_register_name(SAMX7X_PART_VENDOR, soc_name,
			    id_string, NULL);
			return chipid;
		}

		si++;
	}

	return 0;
}

static void
soc_samx7x_free(cortexm_t cm)
{
	struct samx7x_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		cm->cm_soc = NULL;
		SHELL_CMD_DEL(gpnvm);
		zone_free(ss);
	}
}

static const char gpnvm_usage_string[] = {
	"usage:\n"
	"gpnvm\n"
	"\tShow the current status of GPNVM bits.\n"
	"gpnvm [set | clear] <bit-number>\n"
	"\tSet or clear the specified GPNVM bit number. Valid bit numbers\n"
	"\trange from 0 to 8 (note that bit 6 is reserved). See the device\n"
	"\tdata sheet for the precise meanings of each bit.\n"
};

static void
samx7x_cmd_gpnvm(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct samx7x_state *ss = ctx;
	int set, bit, maxbit, i;
	uint32_t op;

	maxbit = ss->ss_has_tcm ? 8 : 5;

	if (argc) {
		if (argc != 2 || strcasecmp(argv[0], "help") == 0) {
 usage:
			fprintf(os, "%s\n", gpnvm_usage_string);
			return;
		}

		if (strcasecmp(argv[0], "set") == 0)
			set = 1;
		else
		if (strcasecmp(argv[0], "clr") == 0 ||
		    strcasecmp(argv[0], "clear") == 0) {
			set = 0;
		} else {
			goto usage;
		}

		if (sscanf(argv[1], "%d", &bit) != 1) {
			fprintf(os, "Malformed bit number '%s'\n", argv[1]);
			goto usage;
		}

		if (bit < 0 || bit == 6 || bit > maxbit) {
			fprintf(os, "Invalid bit number. Please use a value"
			    " between 0 and %d%s\n", maxbit,
			    ss->ss_has_tcm ? " (not including 6)" : "");
			return;
		}

		if (set)
			op = FLASH_SAM_EFC_GPNVM_SET(bit);
		else
			op = FLASH_SAM_EFC_GPNVM_CLEAR(bit);

		if (flash_sam_efc_gpnvm(ss->ss_flash_ctx, op) < 0) {
			fprintf(os, "Failed to %s GPNVM bit %d\n",
			    set ? "set" : "clear", bit);
			return;
		}
	}

	bit = flash_sam_efc_gpnvm(ss->ss_flash_ctx, FLASH_SAM_EFC_GPNVM_QUERY);

	fprintf(os, "GPNVM0: %d (Security Bit: %sabled)\n", bit & 1,
	    (bit & 1) ? "En" : "Dis");

	fprintf(os, "GPNVM1: %d (Boot Mode: %s)\n", (bit >> 1) & 1,
	    (bit & 2) ? "Flash" : "SAM-BA ROM");

	for (i = 2; i <= 5; i++)
		fprintf(os, "GPNVM%d: %d\n", i, (bit & (1u << i)) != 0);

	if (ss->ss_has_tcm) {
		int tcm = ((bit >> 7) & 3) * 32;
		fprintf(os,
		    "GPNVM8-7: %d%d (%d Kbytes DTCM + %d Kbytes ITCM)\n",
		    (bit & (1<<8)) != 0, (bit & (1<<7)) != 0, tcm, tcm);
	}
}

int
soc_samx7x_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	struct adiv5_memap_rom_table_lookup rt;
	struct samx7x_state *ss;
	size_t tcm_size, sram_size;
	uint32_t chipid;
	cortexm_t cm;

	/*
	 * These devices have an Atmel ROM table so quickly check if
	 * it exists.
	 */
	rt.rt_pidr = SAMX7X_ATMEL_ROM_PIDR;
	rt.rt_pidr_mask = SAMX7X_ATMEL_ROM_MASK;
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 0)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* Looks promising. Go check CHIPID */
	chipid = sam_lookup_part(t, samx7x_ids, SAMX7X_NIDS, SAMX7x_REG_CHIPID,
	    SAMX7X_CHIPID_MASK);
	if (chipid == 0)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	sram_size = SAMX7X_CHIPID_SRAMSIZ(chipid) * 1024u;
	tcm_size = 0;

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/*
	 * 256KB and 384KB variants have ITCM and DTCM available,
	 * depending on GPNVM bits 7 & 8.
	 */
	ss->ss_has_tcm = sram_size >= (256u * 1024u);

	/* A single Flash plane */
	flash_planes[0] = SAMX7x_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAMX7x_EEFC_BASE, SAMX7x_EEFC_OFFSET, SAMX7X_WAIT_STATES, 0, 1);

	if (ss->ss_flash_ctx != NULL) {
		if (ss->ss_has_tcm) {
			int bit = flash_sam_efc_gpnvm(ss->ss_flash_ctx,
			    FLASH_SAM_EFC_GPNVM_QUERY);

			if (bit >= 0) {
				tcm_size = 0x8000u * ((bit >> 7) & 3);
				sram_size -= tcm_size * 2u;
			}
		}

		SHELL_CMD_ADD_CTX(gpnvm, ss);
	} else {
		/*
		 * This can happen if the ERASE pin is tied to Vcc,
		 * or the security bit is set.
		 * Not much we can do.
		 */
	}

	/* All devices have a Boot ROM */
	target_add_memory(t, TARGET_MEM_ROM, SAMX7x_ROM_BASE,
	    SAMX7x_ROM_SIZE, "SAM-BA ROM");

	/*
	 * Add SRAM, if there's any remaining after accounting for TCM.
	 * XXX: There is a bug lurking *somewhere* which prevents us using the
	 * Flash applet. Disable applet support for now.
	 */
	if (sram_size) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    SAMX7x_SRAM_BASE, sram_size, NULL);
	}

	/*
	 * Optional TCM
	 */
	if (tcm_size) {
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    SAMX7x_ITCM_BASE, tcm_size, "ITCM");
		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    SAMX7x_DTCM_BASE, tcm_size, "DTCM");
	}

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = NULL;
	cm->cm_soc_free = soc_samx7x_free;

	return CORTEXM_SOC_ATTACH_OK;
}
