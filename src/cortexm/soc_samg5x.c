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
#include "soc_samg5x.h"
#include "hardware.h"
#include "flash_sam_efc.h"
#include "shell.h"
#include "zone_alloc.h"

SHELL_CMD_DECL_CTX(gpnvm, samg5x_cmd_gpnvm, "Clear/Set/Show GPNVM bits");

#define	SAMG5X_PART_VENDOR	"Microchip"
#define	SAMG5X_PART_PREFIX	"ATSAMG5"
#define	SAMG5X_PART_ID_LEN	4

struct sam_id {
	const char si_part[SAMG5X_PART_ID_LEN];
	uint32_t si_chipid;
};

static const uint16_t samg5x_sram_size[16] = {
	48, 192, 384, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};

#define	SAMG5x_REG_CHIPID		0x400e0740u
#define	SAMG5X_CHIPID_SRAMSIZ(c)	samg5x_sram_size[(((c) >> 16) & 0x0fu)]
#define	SAMG5x_EEFC_BASE		0x400e0a00u
#define	SAMG5x_EEFC_OFFSET		0x0u
#define	SAMG5x_ROM_BASE			0x00800000u
#define	SAMG5x_ROM_SIZE			0x8000u
#define	SAMG5x_SRAM_BASE		0x20000000u
#define	SAMG5x_FLASH_BASE		0x00400000u

static const struct sam_id samg51_ids[] = {
	{"1G18",  0x243b09e0},
	{"1N18",  0x243b09e8},
};
#define	SAMG51_NIDS		(sizeof(samg51_ids) / sizeof(samg51_ids[0]))
#define	SAMG51_CHIPID_MASK	0x7ffffff8u
#define	SAMG51_WAIT_STATES	3

static const struct sam_id samg53_ids[] = {
	{"3G19",  0x247e0ae0},
	{"3N19",  0x247e0ae8},
};
#define	SAMG53_NIDS		(sizeof(samg53_ids) / sizeof(samg53_ids[0]))
#define	SAMG53_CHIPID_MASK	0x7ffffff8u
#define	SAMG53_WAIT_STATES	3

static const struct sam_id samg54_ids[] = {
	{"4G19",  0x247e0ae2},
	{"4J19",  0x247e0ae6},
	{"4N19",  0x247e0aea},
};
#define	SAMG54_NIDS		(sizeof(samg54_ids) / sizeof(samg54_ids[0]))
#define	SAMG54_CHIPID_MASK	0x7ffffffeu
#define	SAMG54_WAIT_STATES	7

static const struct sam_id samg55_ids[] = {
	{"5G19",  0x24470ae0},
	{"5J19",  0x24570ae0},
};
#define	SAMG55_NIDS		(sizeof(samg55_ids) / sizeof(samg55_ids[0]))
#define	SAMG55_CHIPID_MASK	0x7fffffe0u
#define	SAMG55_WAIT_STATES	5

struct samg5x_state {
	void *ss_flash_ctx;
	int ss_flags;
};
#define	SOC_SAMG5X_FLASH_BOOT	(1u << 0)

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
	char part[SAMG5X_PART_ID_LEN + 1];
	uint32_t chipid;
	int i;
	char soc_name[TARGET_NAME_LEN_MAX];
	char id_string[TARGET_NAME_LEN_MAX];

	if (sam_read_reg32(t, chip_id_reg, &chipid) < 0)
		return 0;

	while (nids--) {
		if (si->si_chipid == (chipid & chip_id_mask)) {
			for (i = 0; i < SAMG5X_PART_ID_LEN; i++)
				part[i] = si->si_part[i];
			part[i] = '\0';
			snprintf(soc_name, sizeof(soc_name),
			    SAMG5X_PART_PREFIX "%s", part);
			snprintf(id_string, sizeof(id_string), "ID %08" PRIX32,
			    chipid);
			target_register_name(SAMG5X_PART_VENDOR, soc_name,
			    id_string, NULL);
			return chipid;
		}

		si++;
	}

	return 0;
}

static void
soc_samg5x_free(cortexm_t cm)
{
	struct samg5x_state *ss;

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
	"\tare 0 or 1, depending on the target device. The current status\n"
	"\twill indicate how may bits are implemented."
};

static void
samg5x_cmd_gpnvm(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct samg5x_state *ss = ctx;
	int set, bit, maxbit;
	uint32_t op;

	maxbit = (ss->ss_flags & SOC_SAMG5X_FLASH_BOOT) ? 1 : 0;

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

		if (bit < 0 || bit > maxbit) {
			fprintf(os, "Invalid bit number. This SoC has a single"
			    " GPNVM bit, #0\n");
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

	if (maxbit) {
		fprintf(os, "GPNVM1: %d (Boot Mode: %s)\n", (bit >> 1) & 1,
		    (bit & 2) ? "Flash" : "Boot loader");
	}
}

int
soc_samg5x_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	uint32_t chipid, wait_states;
	struct samg5x_state *ss;
	cortexm_t cm;
	int flags;

	/*
	 * Try SAMG51
	 */
	chipid = sam_lookup_part(t, samg51_ids, SAMG51_NIDS, SAMG5x_REG_CHIPID,
	    SAMG51_CHIPID_MASK);
	if (chipid) {
		wait_states = SAMG51_WAIT_STATES;
		flags = 0;
		goto found;
	}

	/*
	 * Try SAMG53
	 */
	chipid = sam_lookup_part(t, samg53_ids, SAMG53_NIDS, SAMG5x_REG_CHIPID,
	    SAMG53_CHIPID_MASK);
	if (chipid) {
		wait_states = SAMG53_WAIT_STATES;
		flags = SOC_SAMG5X_FLASH_BOOT;
		goto found;
	}

	/*
	 * Try SAMG54
	 */
	chipid = sam_lookup_part(t, samg54_ids, SAMG54_NIDS, SAMG5x_REG_CHIPID,
	    SAMG54_CHIPID_MASK);
	if (chipid) {
		wait_states = SAMG54_WAIT_STATES;
		flags = 0;
		goto found;
	}

	/*
	 * Try SAMG55
	 */
	chipid = sam_lookup_part(t, samg55_ids, SAMG55_NIDS, SAMG5x_REG_CHIPID,
	    SAMG55_CHIPID_MASK);
	if (chipid) {
		wait_states = SAMG55_WAIT_STATES;
		flags = SOC_SAMG5X_FLASH_BOOT;
		goto found;
	}

	return CORTEXM_SOC_ATTACH_NO_MATCH;

 found:
	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	ss->ss_flags = flags;

	/* All devices have a Boot ROM */
	target_add_memory(t, TARGET_MEM_ROM, SAMG5x_ROM_BASE,
	    SAMG5x_ROM_SIZE, "Bootloader ROM");

	/* Stir in some SRAM. */
	target_add_memory(t, TARGET_MEM_RAM, SAMG5x_SRAM_BASE,
	    (uint32_t) SAMG5X_CHIPID_SRAMSIZ(chipid) * 1024u, NULL);

	/* A single Flash plane */
	flash_planes[0] = SAMG5x_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAMG5x_EEFC_BASE, SAMG5x_EEFC_OFFSET, wait_states, 0, 1);

	cm = (cortexm_t)t->t_core;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = NULL;
	cm->cm_soc_free = soc_samg5x_free;

	SHELL_CMD_ADD_CTX(gpnvm, ss);

	return CORTEXM_SOC_ATTACH_OK;
}
