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
#include "soc_samd5x.h"
#include "microchip_dsu.h"
#include "hardware.h"
#include "flash_sam_nvmctrl.h"
#include "zone_alloc.h"

#define	SAMD5X_ATMEL_DSUROM_PIDR	0x00009fcd0u
#define	SAMD5X_ATMEL_DSUROM_MASK	0xf000fffffu

#define	SAMD5X_DSUROM_TO_DSUREG(a)	((a) - 0x1000u)

#define	SAMD5X_DSUREG_DID		0x018u
#define	SAMD5X_DID_MASK			0xffff00ffu
#define	SAMD5X_DID_REVISION(d)		(((d) >> 8) & 0x0fu)

#define	SAMD5X_PART_ID_LEN		8
#define	SAMD5X_PART_VENDOR		"Microchip"
#define	SAMD5X_PART_PREFIX		"ATSAM"

#define	SAMD5X_NVM0_BOD_DISABLE(x)	(((x) & 0x01ul) << 0)
#define	SAMD5X_NVM0_BOD_LEVEL(x)	(((x) & 0xfful) << 1)
#define	SAMD5X_NVM0_BOD_ACTION(x)	(((x) & 0x03ul) << 9)
#define	SAMD5X_NVM0_BOD_HYSTERESIS(x)	(((x) & 0x0ful) << 11)
#define	SAMD5X_NVM0_RESVD15(x)		(((x) & 0x7fful) << 15)
#define	SAMD5X_NVM0_NVM_BOOT(x)		(((x) & 0x0ful) << 26)
#define	SAMD5X_NVM0_RESVD30(x)		(((x) & 0x03ul) << 30)
#define	SAMD5X_NVM1_SEESBLK(x)		(((x) & 0x0ful) << 0)
#define	SAMD5X_NVM1_SEEPSZ(x)		(((x) & 0x07ul) << 4)
#define	SAMD5X_NVM1_RAM_ECCDIS(x)	(((x) & 0x01ul) << 7)
#define	SAMD5X_NVM1_RESVD8(x)		(((x) & 0xfful) << 8)
#define	SAMD5X_NVM1_WDT_ENABLE(x)	(((x) & 0x01ul) << 16)
#define	SAMD5X_NVM1_WDT_ALWAYS_ON(x)	(((x) & 0x01ul) << 17)
#define	SAMD5X_NVM1_WDT_PERIOD(x)	(((x) & 0x0ful) << 18)
#define	SAMD5X_NVM1_WDT_WINDOW(x)	(((x) & 0x0ful) << 22)
#define	SAMD5X_NVM1_WDT_EWOFFSET(x)	(((x) & 0x0ful) << 26)
#define	SAMD5X_NVM1_WDT_WEN(x)		(((x) & 0x01ul) << 30)
#define	SAMD5X_NVM1_RESVD31(x)		(((x) & 0x01ul) << 31)
#define	SAMD5X_NVM2_NVM_LOCKS		0xfffffffful

#define	SAMD5X_NVM0_DEFAULTS		(SAMD5X_NVM0_BOD_DISABLE(0x01) | \
					 SAMD5X_NVM0_BOD_LEVEL(0x1c) | \
					 SAMD5X_NVM0_BOD_ACTION(0x01) | \
					 SAMD5X_NVM0_BOD_HYSTERESIS(0x02) | \
					 SAMD5X_NVM0_NVM_BOOT(0x0f))
#define	SAMD5X_NVM1_DEFAULTS		(SAMD5X_NVM1_SEESBLK(0x0) | \
					 SAMD5X_NVM1_SEEPSZ(0x0) | \
					 SAMD5X_NVM1_RAM_ECCDIS(0x1) | \
					 SAMD5X_NVM1_WDT_ENABLE(0x0) | \
					 SAMD5X_NVM1_WDT_ALWAYS_ON(0x0) | \
					 SAMD5X_NVM1_WDT_PERIOD(0x0b) | \
					 SAMD5X_NVM1_WDT_WINDOW(0x0b) | \
					 SAMD5X_NVM1_WDT_EWOFFSET(0x0b) | \
					 SAMD5X_NVM1_WDT_WEN(0x0))

#define	SAMD5X_RESVD0_MASK		(SAMD5X_NVM0_RESVD15(0x7ff) | \
					 SAMD5X_NVM0_RESVD30(0x3))
#define	SAMD5X_RESVD1_MASK		(SAMD5X_NVM1_RESVD8(0xff) | \
					 SAMD5X_NVM1_RESVD31(0x1))

struct samd5_id {
	const char si_part[SAMD5X_PART_ID_LEN];
	uint32_t si_did;
	uint16_t si_sram_size;
};
static const struct samd5_id samd5_ids[] = {
	{"D51P20A", 0x60060000u, 256},
	{"D51P19A", 0x60060001u, 192},
	{"D51N20A", 0x60060002u, 256},
	{"D51N19A", 0x60060003u, 192},
	{"D51J20A", 0x60060004u, 256},
	{"D51J19A", 0x60060005u, 192},
	{"D51J18A", 0x60060006u, 128},
	{"D51G19A", 0x60060007u, 192},
	{"D51G18A", 0x60060008u, 128},

	{"E51N20A", 0x61810000u, 256},
	{"E51N19A", 0x61810001u, 192},
	{"E51J19A", 0x61810002u, 192},
	{"E51J18A", 0x61810003u, 128},
	{"E51J20A", 0x61810004u, 256},

	{"E53N20A", 0x61830002u, 256},
	{"E53N19A", 0x61830003u, 192},
	{"E53J20A", 0x61830004u, 256},
	{"E53J19A", 0x61830005u, 192},
	{"E53J18A", 0x61830006u, 128},

	{"E54P20A", 0x61840000u, 256},
	{"E54P19A", 0x61840001u, 192},
	{"E54N20A", 0x61840002u, 256},
	{"E54N19A", 0x61840003u, 192},
};
#define	SAMD5_NIDS		(sizeof(samd5_ids) / sizeof(samd5_ids[0]))

#define	SAMD5_SRAM_BASE			0x20000000u
#define	SAMD5_FLASH_BASE		0x00000000u
#define	SAMD5_USER_BASE			0x00804000u
#define	SAMD5_USER_SIZE			512u
#define	SAMD5_NVMCTRL_REGS		0x41004000u
#define	SAMD5_SEEPROM_BASE		0x44000000u
#define	SAMD5_BKUPSRAM_BASE		0x47000000u
#define	SAMD5_BKUPSRAM_SIZE		0x2000u
#define	SAMD5X_CMCC_BASE		0x41006000u
#define	SAMD5X_CMCC_CTRL		(SAMD5X_CMCC_BASE + 0x08u)
#define	SAMD5X_CMCC_SR			(SAMD5X_CMCC_BASE + 0x0cu)
#define	 SAMD5X_CMCC_SR_CSTS		(1u << 0)
#define	SAMD5X_CMCC_MAINT0		(SAMD5X_CMCC_BASE + 0x20u)
#define	 SAMD5X_CMCC_MAINT0_INVALL	(1u << 0)

struct samd5x_state {
	void *ss_nvmctrl;
	void *ss_dsu;
	uint32_t ss_cmcc_ctrl;
	microchip_dsu_protection_t ss_prot;
};

static int
soc_samd5x_cmcc_disable(cortexm_t cm)
{
	uint32_t reg;

	/* Clear CTRL.CEN. */
	if (TARGET_REG_WRITE32(cm->cm_target, SAMD5X_CMCC_CTRL, 0) < 0)
		return -1;

	/* Wait for confirmation that the cache is disabled. */
	do {
		if (TARGET_REG_READ32(cm->cm_target, SAMD5X_CMCC_SR, &reg) < 0)
			return -1;
	} while ((reg & SAMD5X_CMCC_SR_CSTS) != 0);

	/* Invalidate the cache. */
	if (TARGET_REG_WRITE32(cm->cm_target, SAMD5X_CMCC_MAINT0,
	    SAMD5X_CMCC_MAINT0_INVALL) < 0) {
		return -1;
	}

	return 0;
}

static int
soc_samd5x_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct samd5x_state *ss = cm->cm_soc;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
	case TARGET_CTL_UNLOCK:
		/*
		 * DSU handles TARGET_CTL_PROTECTION_QUERY and TARGET_CTL_UNLOCK
		 */
		rv = microchip_dsu_ctl(ss->ss_dsu, cmd, arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		rv = flash_sam_nvmctrl0_prot_set(ss->ss_nvmctrl, arg,
		    ss->ss_prot);
		break;

	case TARGET_CTL_APPLET_RUN:
		/* Ensure CMCC is disabled. */
		if (TARGET_REG_READ32(cm->cm_target, SAMD5X_CMCC_CTRL,
		    &ss->ss_cmcc_ctrl) < 0) {
			rv = -1;
		} else {
			rv = soc_samd5x_cmcc_disable(cm);
		}
		break;

	case TARGET_CTL_APPLET_STOP:
		/* Restore CMCC enable setting. */
		if (TARGET_REG_WRITE32(cm->cm_target, SAMD5X_CMCC_CTRL,
		    ss->ss_cmcc_ctrl) < 0) {
			rv = -1;
		}
		break;

	case CORTEXM_CTL_CACHE_PARAMS:
		if (cm->cm_flags & CORTEXM_FLAG_HAS_SOC_CACHE) {
			struct cortexm_cache_params *cp = arg;

			cp->cp_type = "Unified, WT/RA";
			cp->cp_ways = 4;
			cp->cp_linesize = 16;
			cp->cp_sets = 4096u / (cp->cp_linesize * cp->cp_ways);
		}
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_samd5x_free(cortexm_t cm)
{
	struct samd5x_state *ss = cm->cm_soc;

	microchip_dsu_done(ss->ss_dsu);
	cm->cm_soc = NULL;
	zone_free(ss);
}

static const struct samd5_id *
soc_samd5x_match(uint32_t did)
{
	const struct samd5_id *si;

	for (si = samd5_ids; si < &samd5_ids[SAMD5_NIDS]; si++) {
		if (si->si_did == (did & SAMD5X_DID_MASK))
			break;
	}

	if (si == &samd5_ids[SAMD5_NIDS])
		return NULL;

	return si;
}

static void
soc_samd5x_register(uint32_t did, const struct samd5_id *si)
{
	char soc_name[TARGET_NAME_LEN_MAX];
	char did_string[TARGET_NAME_LEN_MAX];

	snprintf(soc_name, sizeof(soc_name), SAMD5X_PART_PREFIX "%s",
	    si->si_part);
	snprintf(did_string, sizeof(did_string), "DID 0x%08" PRIX32, did);
	target_register_name(SAMD5X_PART_VENDOR, soc_name, did_string, NULL);
}

static int
samd5x_user_erase(void *arg, uint8_t *nvm, uint32_t len)
{
	uint32_t word;

	(void) arg;

	/*
	 * 'nvm' points to an in-memory copy of the entire user page.
	 * 'len' must be SAMD5_USER_SIZE.
	 * The first 4 words must be set to default, preserving the
	 * 'reserved' bitfields. The remainder of the user page must
	 * be set to 0xff.
	 */
	assert(len == SAMD5_USER_SIZE);
#if 1
	/* Handle word#0 */
	memcpy(&word, &nvm[sizeof(uint32_t) * 0], sizeof(word));
	word &= SAMD5X_RESVD0_MASK;
	word |= SAMD5X_NVM0_DEFAULTS;
#else
	word = 0xfe9a9239;	/* XXX: Failsafe for SAME54 */
#endif
	memcpy(&nvm[sizeof(uint32_t) * 0], &word, sizeof(word));

	/* Handle word#1 */
#if 1
	memcpy(&word, &nvm[sizeof(uint32_t) * 1], sizeof(word));
	word &= SAMD5X_RESVD1_MASK;
	word |= SAMD5X_NVM1_DEFAULTS;
#else
	word = 0xaeecff80;	/* XXX: Failsafe for SAME54 */
#endif
	memcpy(&nvm[sizeof(uint32_t) * 1], &word, sizeof(word));

	/* Words#2-3 can be erased. */
	memset(&nvm[sizeof(uint32_t) * 2], 0xff, sizeof(word) * 2);

	/* Word#4 must be preserved. */
#if 0
	word = 0x00804010;	/* XXX: Failsafe for SAME53 */
	memcpy(&nvm[sizeof(uint32_t) * 4], &word, sizeof(word));
#endif

	/* Fill the remaining buffer with 0xff. */
	memset(&nvm[sizeof(uint32_t) * 5], 0xff, len - (sizeof(word) * 5));

	return TARGET_FLASH_RESULT_OK;
}

static int
samd5x_user_check(void *arg, uint8_t *nvm, uint32_t pagenum, uint8_t *page)
{
	uint32_t word, *nvm32, *page32;

	(void) arg;

	/*
	 * 'nvm' points to the same page with the in-memory copy of the
	 * entire User Page. Chances are, we erased it earlier but this is
	 * not guaranteed.
	 * 'pagenum' is the page number within the user row. 'pagenum' * 16
	 * must not exceed SAMD5_USER_SIZE.
	 * 'page' points to the data supplied by the user.
	 *
	 * Only the first two pages are of interest.
	 */
	assert((pagenum * 16) < SAMD5_USER_SIZE);
	assert(((uintptr_t)nvm & 0x3u) == 0);
	assert(((uintptr_t)page & 0x3u) == 0);

	nvm32 = (uint32_t *)(void *)nvm;
	page32 = (uint32_t *)(void *)page;

	switch (pagenum) {
	case 0:
		/* Preserve the reserved bitfields in word#0 */
		word = nvm32[0] & SAMD5X_RESVD0_MASK;
		word |= page32[0] & ~SAMD5X_RESVD0_MASK;
		page32[0] = word;

		/* Preserve the reserved bitfields in word#1 */
		word = nvm32[1] & SAMD5X_RESVD1_MASK;
		word |= page32[1] & ~SAMD5X_RESVD1_MASK;
		page32[1] = word;

		/* Word#2 is NVMLOCKS */
		/* Word#3 is User bits */
		break;

	case 1:
		/* First word must be preserved. */
		page32[0] = nvm32[0];
		break;

	default:
		break;
	}

	return 0;
}

int
soc_samd5x_attach(target_t t)
{
	const struct samd5_id *si;
	struct samd5x_state *ss;
	cortexm_t cm;
	uint32_t did;
	void *dsu;
	microchip_dsu_protection_t prot;

	/*
	 * These devices have DSU. Got look for one.
	 */
	if ((dsu = microchip_dsu_core_probe_did(t, &did)) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* Do we support this device? */
	if ((si = soc_samd5x_match(did)) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* Device is supported. Grab protection status from DSU. */
	if (microchip_dsu_protection_status(dsu, MCP_DSU_FLAGS_TYPE_M4,
	    &prot) < 0) {
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if ((ss = zone_malloc(sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	cm = (cortexm_t)t->t_core;
	ss->ss_dsu = dsu;
	ss->ss_prot = prot;
	ss->ss_cmcc_ctrl = 0;

	if (prot == MCP_DSU_PROT_NONE) {
		struct flash_sam_nvmctrl_attach_args aa;

		target_add_memory(t, TARGET_MEM_RAM, SAMD5_SRAM_BASE,
		    (uint32_t)si->si_sram_size * 1024u, NULL);

		target_add_memory(t, TARGET_MEM_RAM | TARGET_MEM_NO_APPLET,
		    SAMD5_BKUPSRAM_BASE, SAMD5_BKUPSRAM_SIZE, "Backup SRAM");

		aa.aa_target = t;
		aa.aa_nvmctrl_regs = SAMD5_NVMCTRL_REGS;
		aa.aa_flash_base = SAMD5_FLASH_BASE;
		aa.aa_user_base = SAMD5_USER_BASE;
		aa.aa_user_size = SAMD5_USER_SIZE;
		aa.aa_user_erase = samd5x_user_erase;
		aa.aa_user_check = samd5x_user_check;
		aa.aa_user_cookie = ss;
		aa.aa_seep_base = SAMD5_SEEPROM_BASE;
		aa.aa_type_ws =
		    FLASH_SAM_NVMCTRL_TYPE(FLASH_SAM_NVMCTRL_TYPE_0, 1);
		aa.aa_flags = FLASH_SAM_NVMCTRL_FLAG_EEP;

		ss->ss_nvmctrl = flash_sam_nvmctrl_attach(&aa);
		if (ss->ss_nvmctrl == NULL) {
			microchip_dsu_done(ss->ss_dsu);
			zone_free(ss);
			return CORTEXM_SOC_ATTACH_NO_MATCH;
		}
	} else {
		t->t_flags |= TARGET_FLAG_IS_LOCKED;
	}

	t->t_flags |= TARGET_FLAG_SUPPORTS_PROTECTION |
	    TARGET_FLAG_SUPPORTS_PERM_PROT;

	cm->cm_flags |= CORTEXM_FLAG_HAS_SOC_CACHE;
	cm->cm_soc = ss;
	cm->cm_soc_ctl = soc_samd5x_ctl;
	cm->cm_soc_free = soc_samd5x_free;

	soc_samd5x_register(did, si);

	return CORTEXM_SOC_ATTACH_OK;
}

void *
soc_samd5x_dsu_probe(uint32_t did, uint32_t *pflags)
{
	const struct samd5_id *si;

	if ((si = soc_samd5x_match(did)) != NULL)
		*pflags = MCP_DSU_FLAGS_TYPE_M4;

	return (void *)(uintptr_t)si;
}

uint32_t
soc_samd5x_dsu_register(uint32_t did, void *arg)
{
	const struct samd5_id *si = arg;

	soc_samd5x_register(did, si);

	return 10000u;
}
