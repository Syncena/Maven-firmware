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
#include "tmon.h"
#include "soc_sam34x.h"
#include "flash_sam_efc.h"
#include "flash_sam_calw.h"
#include "hardware.h"
#include "shell.h"
#include "zone_alloc.h"

SHELL_CMD_DECL_CTX(gpnvm, sam34x_cmd_gpnvm, "Clear/Set/Show GPNVM bits");

#define	SAM34X_PART_VENDOR	"Microchip"
#define	SAM34X_PART_PREFIX	"ATSAM"
#define	SAM34X_PART_ID_LEN	4

struct sam_id {
	const char si_part[SAM34X_PART_ID_LEN];
	uint32_t si_chipid;
};

struct sam_exid {
	const char si_part[SAM34X_PART_ID_LEN];
	uint32_t si_chipid;
	uint32_t si_chipid_exid;
};

#define	SAM34X_CHIPID_ARCH_MASK		0x7fffffe0u
#define	SAM34X_CHIPID_REVISION(c)	(((c) >> 0) & 0x1fu)
#define	SAM34X_CHIPID_SRAMSIZ(c)	(((c) >> 16) & 0x0fu)
#define	SAM34X_CHIPID_EXID_VALID(c)	(((c) & (1u << 31)) != 0)

/* These also have 32KB SRAM at 0x20080000 */
static const struct sam_id sam3xa_ids[] = {
	{"X8H",  0x286e0A60},
	{"X8E",  0x285e0A60},
	{"X4E",  0x285b0960},
	{"X8C",  0x284e0A60},
	{"X4C",  0x284b0960},
	{"A8C",  0x283e0A60},
	{"A4C",  0x283b0960}
};
static const uint16_t sam3xa_sram_size[16] = {
	48, 1, 2, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};
#define	SAM3XA_NIDS		(sizeof(sam3xa_ids) / sizeof(sam3xa_ids[0]))
#define	SAM3XA_REG_CHIPID	0x400e0940u
#define	SAM3XA_EEFC_BASE	0x400e0a00u
#define	SAM3XA_EEFC_OFFSET	0x400u
#define	SAM3XA_WAIT_STATES	4
#define	SAM3XA_SRAM0_BASE	0x20000000u
#define	SAM3XA_SRAM1_BASE	0x20080000u
#define	SAM3XA_SRAM1_SIZE	0x8000u
#define	SAM3XA_FLASH_BASE	0x00080000u
#define	SAM3XA_SAMBA_BASE	0x00100000u
#define	SAM3XA_SAMBA_SIZE	0x8000u

static const struct sam_id sam3n_ids[] = {
	{"N4C",  0x29540960},
	{"N2C",  0x29590760},
	{"N1C",  0x29580560},
	{"N4B",  0x29440960},
	{"N2B",  0x29490760},
	{"N1B",  0x29480560},
	{"N4A",  0x29340960},
	{"N2A",  0x29390760},
	{"N1A",  0x29380560},
	{"N0C",  0x29580360},
	{"N0B",  0x29480360},
	{"N0A",  0x29380360},
	{"N00B", 0x29450260},
	{"N00A", 0x29350260}
};
static const uint16_t sam3n_sram_size[16] = {
	48, 1, 2, 6, 112, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};
#define	SAM3N_NIDS		(sizeof(sam3n_ids) / sizeof(sam3n_ids[0]))
#define	SAM3N_REG_CHIPID	0x400e0740u
#define	SAM3N_EEFC_BASE		0x400e0a00u
#define	SAM3N_EEFC_OFFSET	0u		/* Only a single instance */
#define	SAM3N_WAIT_STATES	2
#define	SAM3N_SRAM_BASE		0x20000000u
#define	SAM3N_FLASH_BASE	0x00400000u
#define	SAM3N_SAMBA_BASE	0x00800000u
#define	SAM3N_SAMBA_SIZE	0x8000u

static const struct sam_id sam3s_ids[] = {
	{"S4A",  0x28800960},
	{"S2A",  0x288a0760},
	{"S1A",  0x28890560},
	{"S4B",  0x28900960},
	{"S2B",  0x289a0760},
	{"S1B",  0x28990560},
	{"S4C",  0x28a00960},
	{"S2C",  0x28aa0760},
	{"S1C",  0x28a90560},
	{"S8A",  0x288b0a60},
	{"S8B",  0x289b0a60},
	{"S8C",  0x28ab0a60},
	{"SD8A", 0x298b0a60},
	{"SD8B", 0x299b0a60},
	{"SD8C", 0x29ab0a60}
};
#define	sam3s_sram_size		sam3n_sram_size
#define	SAM3S_NIDS		(sizeof(sam3s_ids) / sizeof(sam3s_ids[0]))
#define	SAM3S_REG_CHIPID	0x400e0740u
#define	SAM3S_EEFC_BASE		0x400e0a00u
#define	SAM3S_EEFC_OFFSET	0u		/* Only a single instance */
#define	SAM3S_WAIT_STATES	3
#define	SAM3S_SRAM_BASE		0x20000000u
#define	SAM3S_FLASH_BASE	0x00400000u
#define	SAM3S_SAMBA_BASE	0x00800000u
#define	SAM3S_SAMBA_SIZE	0x8000u

static const struct sam_id sam3u_ids[] = {
	{"U4C", 0x28000960},	/* Also 16KB SRAM at 0x20080000 */
	{"U2C", 0x280a0760},	/* Also 16KB SRAM at 0x20080000 */
	{"U1C", 0x28090560},	/* Also  8KB SRAM at 0x20080000 */
	{"U4E", 0x28100960},	/* Also 16KB SRAM at 0x20080000 */
	{"U2E", 0x281a0760},	/* Also 16KB SRAM at 0x20080000 */
	{"U1E", 0x28190560},	/* Also  8KB SRAM at 0x20080000 */
};
#define	sam3u_sram_size		sam3xa_sram_size
#define	SAM3U_NIDS		(sizeof(sam3u_ids) / sizeof(sam3u_ids[0]))
#define	SAM3U_REG_CHIPID	0x400e0740u
#define	SAM3U_EEFC_BASE		0x400e0800u
#define	SAM3U_EEFC_OFFSET	0x200u
#define	SAM3U_WAIT_STATES	3
#define	SAM3U_SRAM0_BASE	0x20000000u
#define	SAM3U_SRAM1_BASE	0x20080000u
#define	SAM3U_FLASH0_BASE	0x00080000u
#define	SAM3U_FLASH1_BASE	0x00100000u
#define	SAM3U_SAMBA_BASE	0x00180000u
#define	SAM3U_SAMBA_SIZE	0x8000u

static const struct sam_id sam4e_ids[] = {
	{"E16E", 0x00120200},	/* Matched on CHIPID_EXID */
	{"E8E",  0x00120208},	/* Matched on CHIPID_EXID */
	{"E16C", 0x00120201},	/* Matched on CHIPID_EXID */
	{"E8C",  0x00120209},	/* Matched on CHIPID_EXID */
};
static const uint16_t sam4e_sram_size[16] = {
	48, 192, 384, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};
#define	SAM4E_NIDS		(sizeof(sam4e_ids) / sizeof(sam4e_ids[0]))
#define	SAM4E_REG_CHIPID	0x400e0740u
#define	SAM4E_CHIPID_4E		0xa3cc0ce0u
#define	SAM4E_REG_CHIPID_EXID	0x400e0744u
#define	SAM4E_EEFC_BASE		0x400e0a00u
#define	SAM4E_EEFC_OFFSET	0x0u
#define	SAM4E_WAIT_STATES	6
#define	SAM4E_SRAM_BASE		0x20000000u
#define	SAM4E_FLASH_BASE	0x00400000u
#define	SAM4E_SAMBA_BASE	0x00800000u
#define	SAM4E_SAMBA_SIZE	0x8000u

static const struct sam_exid sam4l_ids[] = {
	{"LC8C", 0x2b0b0ae0, 0x1400000f},
	{"LC4C", 0x2b0a09e0, 0x0400000f},
	{"LC2C", 0x2b0a07e0, 0x0400000f},
	{"LC8B", 0x2b0b0ae0, 0x1300000f},
	{"LC4B", 0x2b0a09e0, 0x0300000f},
	{"LC2B", 0x2b0a07e0, 0x0300000f},
	{"LC8A", 0x2b0b0ae0, 0x1100000f},
	{"LC4A", 0x2b0a09e0, 0x0100000f},
	{"LC2A", 0x2b0a07e0, 0x0100000f},
	{"LS8C", 0x2b0b0ae0, 0x14000002},
	{"LS4C", 0x2b0a09e0, 0x04000002},
	{"LS2C", 0x2b0a07e0, 0x04000002},
	{"LS8B", 0x2b0b0ae0, 0x13000002},
	{"LS4B", 0x2b0a09e0, 0x03000002},
	{"LS2B", 0x2b0a07e0, 0x03000002},
	{"LS8A", 0x2b0b0ae0, 0x12000002},
	{"LS4A", 0x2b0a09e0, 0x02000002},
	{"LS2A", 0x2b0a07e0, 0x02000002},
};
#define	sam4l_sram_size		sam3xa_sram_size
static const uint16_t sam4l_flash_size_table[16] = {
	4, 8, 16, 32, 48, 64, 96, 128, 192, 256, 384, 512, 768, 1024, 2048, 0
};
#define	SAM4L_NIDS		(sizeof(sam4l_ids) / sizeof(sam4l_ids[0]))
#define	SAM4L_REG_CHIPID	0x400e0740u
#define	SAM4L_CALW_BASE		0x400a0000u
#define	SAM4L_SRAM_BASE		0x20000000u
#define	SAM4L_FLASH_BASE	0x00000000u

static const struct sam_id sam4n_ids[] = {
	{"N16B", 0x29460ce0},
	{"N16C", 0x29560ce0},
	{"N8A",  0x293b0ae0},
	{"N8B",  0x294b0ae0},
	{"N8C",  0x295b0ae0},
};
static const uint16_t sam4n_sram_size[16] = {
	48, 192, 2, 6, 24, 4, 80, 160, 8, 16, 32, 64, 128, 256, 96, 512
};
#define	SAM4N_NIDS		(sizeof(sam4n_ids) / sizeof(sam4n_ids[0]))
#define	SAM4N_REG_CHIPID	0x400e0740u
#define	SAM4N_EEFC_BASE		0x400e0a00u
#define	SAM4N_EEFC_OFFSET	0x0u
#define	SAM4N_WAIT_STATES	4
#define	SAM4N_SRAM_BASE		0x20000000u
#define	SAM4N_FLASH_BASE	0x00400000u
#define	SAM4N_SAMBA_BASE	0x00800000u
#define	SAM4N_SAMBA_SIZE	0x8000u

static const struct sam_id sam4s_ids[] = {
	{"D32C", 0x29a70ee0},
	{"D32B", 0x29970ee0},
	{"D16C", 0x29a70ce0},
	{"D16B", 0x29970ce0},
	{"A16C", 0x28a70ce0},
	{"A16B", 0x28970ce0},
	{"16B",  0x289c0ce0},
	{"16C",  0x28ac0ce0},
	{"8B",   0x289c0ae0},
	{"8C",   0x28ac0ae0},
	{"4C",   0x28ab09e0},
	{"4B",   0x289b09e0},
	{"4A",   0x288b09e0},
	{"2C",   0x28ab07e0},
	{"2B",   0x289b07e0},
	{"2A",   0x288b07e0}
};
#define	sam4s_sram_size		sam4e_sram_size
#define	SAM4S_NIDS		(sizeof(sam4s_ids) / sizeof(sam4s_ids[0]))
#define	SAM4S_REG_CHIPID	0x400e0740u
#define	SAM4S_EEFC_BASE		0x400e0a00u
#define	SAM4S_EEFC_OFFSET	0x200u
#define	SAM4S_WAIT_STATES	5
#define	SAM4S_SRAM_BASE		0x20000000u
#define	SAM4S_FLASH_BASE	0x00400000u
#define	SAM4S_SAMBA_BASE	0x00800000u
#define	SAM4S_SAMBA_SIZE	0x8000u

static const char sam_ba_rom[] = "SAM-BA ROM";

struct sam34x_state {
	void *ss_flash_ctx;
	int ss_flags;
};
#define	SOC_SAM34X_HAS_SMAP	(1u << 0)

static int
sam_read_reg32(target_t t, uint32_t addr, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(t, (target_addr_t)addr, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(t);

	return rv;
}

static void
soc_sam34x_free(cortexm_t cm)
{
	struct sam34x_state *ss;

	if ((ss = cm->cm_soc) != NULL) {
		if ((ss->ss_flags & SOC_SAM34X_HAS_SMAP) == 0)
			SHELL_CMD_DEL(gpnvm);

		cm->cm_soc = NULL;
		zone_free(ss);
	}
}

static int
soc_sam34x_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	int rv = 0;

	switch (*cmd) {
	case CORTEXM_CTL_CACHE_PARAMS:
		if (cm->cm_flags & CORTEXM_FLAG_HAS_SOC_CACHE) {
			struct cortexm_cache_params *cp = arg;

			cp->cp_type = "Unified, WT/RA";
			cp->cp_ways = 4;
			cp->cp_linesize = 16;
			cp->cp_sets = 2048u / (cp->cp_linesize * cp->cp_ways);
		}
		break;

	default:
		break;
	}

	return rv;
}

static const char gpnvm_usage_string[] = {
	"usage:\n"
	"gpnvm\n"
	"\tShow the current status of GPNVM bits.\n"
	"gpnvm [set | clear] <bit-number>\n"
	"\tSet or clear the specified GPNVM bit number. The bit number is\n"
	"\tusually 0 or 1 but can be 2 in devices with larger Flash memory.\n"
	"\tThe current status will indicate how may bits are implemented."
};

struct sam34x_gpnvm_ctx {
	void *gc_ctx;
	uint32_t gc_op;
};

static uintptr_t
sam34x_gpnvm_locked(void *arg)
{
	struct sam34x_gpnvm_ctx *gc = arg;

	return (uintptr_t)flash_sam_efc_gpnvm(gc->gc_ctx, gc->gc_op);
}

static void
sam34x_cmd_gpnvm(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct sam34x_state *ss = ctx;
	struct sam34x_gpnvm_ctx gc;
	int set, bit, planes, rv;

	if ((planes = flash_sam_efc_nplanes(ss->ss_flash_ctx)) < 0) {
		fprintf(os, "Failed to retrieve current GPNVM bits\n");
		return;
	}

	gc.gc_ctx = ss->ss_flash_ctx;

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

		if (bit < 0 || bit > planes) {
			fprintf(os, "Invalid bit number. Must be between 0"
			    " and %d\n", planes);
			return;
		}

		if (set)
			gc.gc_op = FLASH_SAM_EFC_GPNVM_SET(bit);
		else
			gc.gc_op = FLASH_SAM_EFC_GPNVM_CLEAR(bit);

		rv = (int)tmon_locked_callback(sam34x_gpnvm_locked, &gc);
		if (rv < 0) {
			fprintf(os, "Failed to %s GPNVM bit %d\n",
			    set ? "set" : "clear", bit);
			return;
		}
	}

	gc.gc_op = FLASH_SAM_EFC_GPNVM_QUERY;
	bit = (int)tmon_locked_callback(sam34x_gpnvm_locked, &gc);

	fprintf(os, "GPNVM0: %d (Security Bit: %sabled)\n", bit & 1,
	    (bit & 1) ? "En" : "Dis");
	fprintf(os, "GPNVM1: %d (Boot Mode: %s)\n", (bit >> 1) & 1,
	    (bit & 2) ? "Flash" : sam_ba_rom);

	if (planes == 2) {
		fprintf(os, "GPNVM2: %d (Flash Selection: Flash %d)\n",
		    (bit >> 2) & 1, (bit >> 2) & 1);
	}
}

static uint32_t
sam_lookup_part(target_t t, const struct sam_id *si, int nids,
    uint32_t chip_id_reg, const char *soc_prefix)
{
	char part[SAM34X_PART_ID_LEN + 1];
	char soc_name[TARGET_NAME_LEN_MAX];
	char id_string[TARGET_NAME_LEN_MAX];
	uint32_t chipid;
	int i;

	if (sam_read_reg32(t, chip_id_reg, &chipid) < 0)
		return 0;

	while (nids--) {
		if (si->si_chipid == (chipid & SAM34X_CHIPID_ARCH_MASK)) {
			for (i = 0; i < SAM34X_PART_ID_LEN; i++)
				part[i] = si->si_part[i];
			part[i] = '\0';
			snprintf(soc_name, sizeof(soc_name),
			    SAM34X_PART_PREFIX "%s%s", soc_prefix, part);
			snprintf(id_string, sizeof(id_string),
			    "CHIP ID %08" PRIX32, chipid);
			target_register_name(SAM34X_PART_VENDOR, soc_name,
			    id_string, NULL);
			return chipid;
		}

		si++;
	}

	return 0;
}

static uint32_t
sam_lookup_part_exid(target_t t, const struct sam_exid *si, int nids,
    uint32_t chip_id_reg, const char *soc_prefix)
{
	char part[SAM34X_PART_ID_LEN + 1];
	char soc_name[TARGET_NAME_LEN_MAX];
	char id_string[TARGET_NAME_LEN_MAX];
	uint32_t chipid, chipid_exid;
	int i;

	if (sam_read_reg32(t, chip_id_reg, &chipid) < 0 ||
	    sam_read_reg32(t, chip_id_reg + 0x4, &chipid_exid) < 0) {
		return 0;
	}

	while (nids--) {
		if (si->si_chipid == (chipid & SAM34X_CHIPID_ARCH_MASK) &&
		    (SAM34X_CHIPID_EXID_VALID(chipid) == 0 ||
		    si->si_chipid_exid == chipid_exid)) {
			for (i = 0; i < SAM34X_PART_ID_LEN; i++)
				part[i] = si->si_part[i];
			part[i] = '\0';
			snprintf(soc_name, sizeof(soc_name),
			    SAM34X_PART_PREFIX "%s%s", soc_prefix, part);
			snprintf(id_string, sizeof(id_string),
			    "ID %08" PRIX32 " %08" PRIX32 "", chipid,
			    chipid_exid);
			target_register_name(SAM34X_PART_VENDOR, soc_name,
			    id_string, NULL);
			return chipid;
		}

		si++;
	}

	return 0;
}

static struct sam34x_state *
soc_sam34x_attach_common(cortexm_t cm, int flags)
{
	struct sam34x_state *ss;

	if ((ss = zone_calloc(1, sizeof(*ss))) == NULL)
		return NULL;

	ss->ss_flags = flags;
	cm->cm_soc = ss;
	cm->cm_soc_free = soc_sam34x_free;
	cm->cm_soc_ctl = soc_sam34x_ctl;

	if ((flags & SOC_SAM34X_HAS_SMAP) == 0)
		SHELL_CMD_ADD_CTX(gpnvm, ss);

	return ss;
}

int
soc_sam3xa_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	chipid = sam_lookup_part(t, sam3xa_ids, SAM3XA_NIDS, SAM3XA_REG_CHIPID,
	    "3");
	if (chipid == 0)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM3XA_SAMBA_BASE,
	    SAM3XA_SAMBA_SIZE, sam_ba_rom);

	/* Stir in some SRAM. One variable bank and a 2nd fixed 32KB bank */
	target_add_memory(t, TARGET_MEM_RAM, SAM3XA_SRAM0_BASE,
	    (uint32_t)sam3xa_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    "SRAM0");
	target_add_memory(t, TARGET_MEM_RAM, SAM3XA_SRAM1_BASE,
	    SAM3XA_SRAM1_SIZE, "SRAM1");

	/* Multiple Flash planes are always contiguous */
	flash_planes[0] = SAM3XA_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_CONTIG;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM3XA_EEFC_BASE, SAM3XA_EEFC_OFFSET, SAM3XA_WAIT_STATES, 1, 1);

	return CORTEXM_SOC_ATTACH_OK;
}

int
soc_sam3n_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	chipid = sam_lookup_part(t, sam3n_ids, SAM3N_NIDS, SAM3N_REG_CHIPID,
	   "3");
	if (chipid == 0)
		return 0;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM3N_SAMBA_BASE,
	    SAM3N_SAMBA_SIZE, sam_ba_rom);

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM3N_SRAM_BASE,
	    (uint32_t)sam3n_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* Only a single plane */
	flash_planes[0] = SAM3N_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM3N_EEFC_BASE, SAM3N_EEFC_OFFSET, SAM3N_WAIT_STATES, 1, 1);

	return 1;
}

int
soc_sam3s_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	chipid = sam_lookup_part(t, sam3s_ids, SAM3S_NIDS, SAM3S_REG_CHIPID,
	   "3");
	if (chipid == 0)
		return 0;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM3S_SAMBA_BASE,
	    SAM3S_SAMBA_SIZE, sam_ba_rom);

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM3S_SRAM_BASE,
	    (uint32_t)sam3s_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* Only a single plane */
	flash_planes[0] = SAM3S_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM3S_EEFC_BASE, SAM3S_EEFC_OFFSET, SAM3S_WAIT_STATES, 1, 1);

	return 1;
}

int
soc_sam3u_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid, sram_size;

	chipid = sam_lookup_part(t, sam3u_ids, SAM3U_NIDS, SAM3U_REG_CHIPID,
	   "3");
	if (chipid == 0)
		return 0;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM3U_SAMBA_BASE,
	    SAM3U_SAMBA_SIZE, sam_ba_rom);

	/* Stir in some SRAM */
	sram_size = (uint32_t)sam3u_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)];
	target_add_memory(t, TARGET_MEM_RAM, SAM3U_SRAM0_BASE,
	    sram_size * 1024u, "SRAM0");
	target_add_memory(t, TARGET_MEM_RAM, SAM3U_SRAM1_BASE,
	    (sram_size == 8) ?  0x2000u : 0x4000u, "SRAM1");

	/* Up to two non-contiguous planes */
	flash_planes[0] = SAM3U_FLASH0_BASE;
	flash_planes[1] = SAM3U_FLASH1_BASE;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM3U_EEFC_BASE, SAM3U_EEFC_OFFSET, SAM3U_WAIT_STATES, 1, 1);

	return 1;
}

int
soc_sam4e_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	if (sam_read_reg32(t, SAM4E_REG_CHIPID, &chipid) < 0 ||
	    chipid != SAM4E_CHIPID_4E) {
		return 0;
	}

	if (sam_lookup_part(t, sam4e_ids, SAM4E_NIDS, SAM4E_REG_CHIPID_EXID,
	    "4") == 0) {
		return 0;
	}

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM4E_SAMBA_BASE, SAM4E_SAMBA_SIZE,
	    sam_ba_rom);

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM4E_SRAM_BASE,
	    (uint32_t)sam4e_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* One Flash plane */
	flash_planes[0] = SAM4E_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM4E_EEFC_BASE, SAM4E_EEFC_OFFSET, SAM4E_WAIT_STATES, 0, 1);

	cm->cm_flags |= CORTEXM_FLAG_HAS_SOC_CACHE;

	return 1;
}

int
soc_sam4l_attach(target_t t)
{
	cortexm_t cm = (cortexm_t)t->t_core;
	uint32_t chipid;

	chipid = sam_lookup_part_exid(t, sam4l_ids, SAM4L_NIDS,
	    SAM4L_REG_CHIPID, "4");
	if (chipid == 0)
		return 0;

	if (soc_sam34x_attach_common(cm, SOC_SAM34X_HAS_SMAP) == NULL)
		return 0;

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM4L_SRAM_BASE,
	    (uint32_t)sam4l_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* Probe for Flash */
	flash_sam_calw_attach(t, SAM4L_FLASH_BASE, SAM4L_CALW_BASE,
	    sam4l_flash_size_table);

	return 1;
}

int
soc_sam4n_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	chipid = sam_lookup_part(t, sam4n_ids, SAM4N_NIDS, SAM4N_REG_CHIPID,
	    "4");
	if (chipid == 0)
		return 0;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM4N_SAMBA_BASE, SAM4N_SAMBA_SIZE,
	    sam_ba_rom);

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM4N_SRAM_BASE,
	    (uint32_t)sam4n_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* One Flash plane */
	flash_planes[0] = SAM4N_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_INVALID;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM4N_EEFC_BASE, SAM4N_EEFC_OFFSET, SAM4N_WAIT_STATES, 0, 1);

	return 1;
}

int
soc_sam4s_attach(target_t t)
{
	target_addr_t flash_planes[SAM_EFC_MAX_PLANES];
	cortexm_t cm = (cortexm_t)t->t_core;
	struct sam34x_state *ss;
	uint32_t chipid;

	chipid = sam_lookup_part(t, sam4s_ids, SAM4S_NIDS, SAM4S_REG_CHIPID,
	    "4S");
	if (chipid == 0)
		return 0;

	if ((ss = soc_sam34x_attach_common(cm, 0)) == NULL)
		return 0;

	/* All devices have 8KB of ROM containing SAM-BA */
	target_add_memory(t, TARGET_MEM_ROM, SAM4S_SAMBA_BASE, SAM4S_SAMBA_SIZE,
	    sam_ba_rom);

	/* Stir in some SRAM */
	target_add_memory(t, TARGET_MEM_RAM, SAM4S_SRAM_BASE,
	    (uint32_t)sam4s_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] * 1024u,
	    NULL);

	/* Flash planes are contiguous */
	flash_planes[0] = SAM4S_FLASH_BASE;
	flash_planes[1] = SAM_EFC_FLASH_BASE_CONTIG;

	/* Attach the Flash */
	ss->ss_flash_ctx = flash_sam_efc_attach(t, flash_planes,
	    SAM4S_EEFC_BASE, SAM4S_EEFC_OFFSET, SAM4S_WAIT_STATES, 0, 1);

	if (sam4s_sram_size[SAM34X_CHIPID_SRAMSIZ(chipid)] >= 160u) {
		/* SoC has HCACHE */
		cm->cm_flags |= CORTEXM_FLAG_HAS_SOC_CACHE;
	}

	return 1;
}
