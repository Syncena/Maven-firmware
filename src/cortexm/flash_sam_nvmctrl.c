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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash_sam_nvmctrl.h"
#include "flash_applet.h"
#include "rtos.h"
#include "timer.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Driver for the Flash "NVMCTRL" Controller found on some SAM parts.
 */
struct sam_nvmctrl_state;

struct sam_nvmctrl_flash {
	struct target_flash nf_tf;
	struct sam_nvmctrl_state *nf_ns;
	void *nf_flash_applet;
	uint32_t nf_base;
	uint32_t nf_erase_size;
};

struct sam_nvmctrl_state {
	struct flash_sam_nvmctrl_attach_args ns_aa;
	uint32_t ns_lock_region_size;
	uint32_t ns_lockbits;
	uint32_t ns_save_ctrl;
	uint16_t ns_ref_count;
	char ns_extra[40];
	bool ns_user_dirty;
	bool ns_init_done;
	uint8_t *ns_user_page;

	struct sam_nvmctrl_flash ns_flash[4];
};
#define	SAM_TF_FLASH		0
#define	SAM_TF_USER		1
#define	SAM_TF_RWWEEP		2
#define	SAM_TF_SEEP		3
#define	ns_target		ns_aa.aa_target
#define	ns_nvmctrl_regs		ns_aa.aa_nvmctrl_regs
#define	ns_flash_base		ns_aa.aa_flash_base
#define	ns_rwweep_base		ns_aa.aa_rwweep_base
#define	ns_seep_base		ns_aa.aa_seep_base
#define	ns_user_base		ns_aa.aa_user_base
#define	ns_user_size		ns_aa.aa_user_size
#define	ns_user_erase		ns_aa.aa_user_erase
#define	ns_user_check		ns_aa.aa_user_check
#define	ns_user_cookie		ns_aa.aa_user_cookie
#define	ns_type_ws		ns_aa.aa_type_ws
#define	ns_flags		ns_aa.aa_flags

#define	SAM_NVMCTRL_TYPE(n)		((n)->ns_type_ws & 0xffu)
#define	SAM_NVMCTRL_WS(n)		(((n)->ns_type_ws >> 8) & 0x0fu)
#define	SAM_NVM_CMD(c)			(0xa500u | (c))
#define	SAM_OFFSET_TO_LOCKBIT(ns,o)	((o) / ((ns)->ns_lock_region_size))

/*
 * NVMCTRL Registers; offset from ns_nvmctrl_regs.
 * There are some egregious differences between versions.
 */

/*
 * Type-0 Register Definitions
 */
#define	SAM_NVMCTRL0_CTRL		0x00u	/* 16-bits */
#define	 SAM_CTRL0_CACHEDIS1		(1u << 15)
#define	 SAM_CTRL0_CACHEDIS0		(1u << 14)
#define	 SAM_CTRL0_RWS_SHIFT		8
#define	 SAM_CTRL0_WMODE_SHIFT		4
#define	 SAM_CTRL0_WMODE_MASK		4
#define	 SAM_CTRL0_WMODE_MAN		0
#define	 SAM_CTRL0_WMODE_AP		3
#define	 SAM_CTRL0_AUTOWS		(1u << 2)

#define	SAM_NVMCTRL0_CMD		0x04u	/* 16-bits */
#define	 SAM_CMD0_EP			SAM_NVM_CMD(0x00u)
#define	 SAM_CMD0_EB			SAM_NVM_CMD(0x01u)
#define	 SAM_CMD0_WP			SAM_NVM_CMD(0x03u)
#define	 SAM_CMD0_WQW			SAM_NVM_CMD(0x04u)
#define	 SAM_CMD0_UR			SAM_NVM_CMD(0x12u)
#define	 SAM_CMD0_PBC			SAM_NVM_CMD(0x15u)
#define	 SAM_CMD0_SSB			SAM_NVM_CMD(0x16u)
#define	 SAM_CMD0_CELCK			SAM_NVM_CMD(0x18u)
#define	 SAM_CMD0_SBPDIS		SAM_NVM_CMD(0x1au)
#define	 SAM_CMD0_SEEFLUSH		SAM_NVM_CMD(0x33u)
#define	SAM_NVMCTRL0_CMD_NEEDS_ADDR(c)	((c) <= SAM_CMD0_UR)

#define	SAM_NVMCTRL0_PARAM		0x08u	/* 32-bits */
#define	 SAM_PARAM0_SEE			(1u << 31)
#define	 SAM_PARAM_PSZ(p)		(0x08u << (((p) >> 16) & 0x7u))
#define	 SAM_PARAM_NVMP(p)		((p) & 0x0000ffffu)

#define	SAM_NVMCTRL0_INTENCLR		0x0cu	/* 16-bits */
#define	SAM_NVMCTRL0_INTENSET		0x0eu	/* 16-bits */
#define	SAM_NVMCTRL0_INTFLAG		0x10u	/* 16-bits */
#define	 SAM_INT0_DONE			(1u << 0)
#define	 SAM_INT0_ADDRE			(1u << 1)
#define	 SAM_INT0_PROGE			(1u << 2)
#define	 SAM_INT0_LOCKE			(1u << 3)
#define	 SAM_INT0_ECCSE			(1u << 4)
#define	 SAM_INT0_ECCDE			(1u << 5)
#define	 SAM_INT0_NVME			(1u << 6)
#define	 SAM_INT0_SUSP			(1u << 7)
#define	 SAM_INT0_SEESFULL		(1u << 8)
#define	 SAM_INT0_SEESOVF		(1u << 9)
#define	 SAM_INT0_SEEWRC		(1u << 10)
#define	 SAM_INT0_ALL_ERRORS		(SAM_INT0_ADDRE | \
					 SAM_INT0_PROGE | \
					 SAM_INT0_LOCKE | \
					 SAM_INT0_ECCSE | \
					 SAM_INT0_ECCDE | \
					 SAM_INT0_NVME | \
					 SAM_INT0_NVME)

#define	SAM_NVMCTRL0_STATUS		0x12u	/* 16-bits */
#define	 SAM_STATUS0_READY		(1u << 0)
#define	 SAM_STATUS0_PRM		(1u << 1)
#define	 SAM_STATUS0_LOAD		(1u << 2)
#define	 SAM_STATUS0_SUSP		(1u << 3)
#define	 SAM_STATUS0_AFIRST		(1u << 4)
#define	 SAM_STATUS0_BPDIS		(1u << 5)
#define	 SAM_STATUS0_BOOTPROT(x)	((15u - (((x) >> 8) & 0xfu)) * 8192u)

#define	SAM_NVMCTRL0_ADDR		0x14u	/* 32-bits */
#define	 SAM_ADDR0(a)			((a) & 0x00ffffffu)

#define	SAM_NVMCTRL0_RUNLOCK		0x18u	/* 32-bits */
#define	SAM_NVMCTRL0_PBLDATAn0		0x1cu	/* 32-bits */
#define	SAM_NVMCTRL0_PBLDATAn1		0x20u	/* 32-bits */
#define	SAM_NVMCTRL0_ECCERR		0x24u	/* 32-bits */
#define	SAM_NVMCTRL0_DBGCTRL		0x28u	/* 8-bits */
#define	SAM_NVMCTRL0_SEECFG		0x2au	/* 8-bits */
#define	 SAM_SEECFG0_WMODE		(1u < 0)
#define	 SAM_SEECFG0_APRDIS		(1u < 1)
#define	SAM_NVMCTRL0_SEESTAT		0x2cu	/* 32-bits */
#define	 SAM_SEESTAT0_ASEES		(1u << 0)
#define	 SAM_SEESTAT0_LOAD		(1u << 1)
#define	 SAM_SEESTAT0_BUSY		(1u << 2)
#define	 SAM_SEESTAT0_LOCK		(1u << 3)
#define	 SAM_SEESTAT0_RLOCK		(1u << 4)
#define	 SAM_SEESTAT0_SBLK(x)		(((x) >> 8) & 0xfu)
#define	 SAM_SEESTAT0_PSZ(x)		(4u << (((x) >> 16) & 0x3u))

#define	SAM_NVMCTRL0_LOCK_REGIONS	32u
#define	SAM_NVMCTRL0_LOCK_BIT(r)	(1u << (r))
#define	SAM_NVMCTRL0_USER_SIZE		512u
#define	SAM_NVMCTRL0_USER_PAGE_SIZE	16u

/*
 * Type-1 Register Definitions
 */
#define	SAM_NVMCTRL1_CTRL		0x04u	/* 16-bits */
#define	 SAM_CTRL1_CACHEDIS		(1u << 18)
#define	 SAM_CTRL1_MANW			(1u << 7)
#define	 SAM_CTRL1_RWS_SHIFT		1
#define	 SAM_CTRL1_RWS_MASK		(0x0fu << 1)

#define	SAM_NVMCTRL1_CMD		0x00u	/* 16-bits */
#define	 SAM_CMD1_ER			SAM_NVM_CMD(0x02u)
#define	 SAM_CMD1_WP			SAM_NVM_CMD(0x04u)
#define	 SAM_CMD1_EAR			SAM_NVM_CMD(0x05u)
#define	 SAM_CMD1_WAP			SAM_NVM_CMD(0x06u)
#define	 SAM_CMD1_RWWEEER		SAM_NVM_CMD(0x1au)
#define	 SAM_CMD1_RWWEEWP		SAM_NVM_CMD(0x1cu)
#define	 SAM_CMD1_UR			SAM_NVM_CMD(0x41u)
#define	 SAM_CMD1_PBC			SAM_NVM_CMD(0x44u)
#define	 SAM_CMD1_SSB			SAM_NVM_CMD(0x45u)

#define	SAM_NVMCTRL1_PARAM		0x08u	/* 32-bits */
#define	 SAM_PARAM1_RWWEEP(p)		(((p) >> 20) & 0x00000fffu)

#define	SAM_NVMCTRL1_INTENCLR		0x0cu	/* 8-bits */
#define	SAM_NVMCTRL1_INTENSET		0x10u	/* 8-bits */
#define	SAM_NVMCTRL1_INTFLAG		0x14u	/* 8-bits */
#define	 SAM_INT1_READY			(1u << 0)	/* XXX: Undocumented */
#define	 SAM_INT1_ERROR			(1u << 1)

#define	SAM_NVMCTRL1_STATUS		0x18u	/* 16-bits */
#define	 SAM_STATUS1_PRM		(1u << 0)
#define	 SAM_STATUS1_LOAD		(1u << 1)
#define	 SAM_STATUS1_PROGE		(1u << 2)
#define	 SAM_STATUS1_LOCKE		(1u << 3)
#define	 SAM_STATUS1_NVME		(1u << 4)

#define	SAM_NVMCTRL1_ADDR		0x1cu	/* 32-bits */
#define	 SAM_ADDR1(a)			(((a) >> 1) & 0x003fffffu)

#define	SAM_NVMCTRL1_LOCK		0x20u	/* 16-bits */

#define	SAM_NVMCTRL1_LOCK_REGIONS	16u
#define	SAM_NVMCTRL1_LOCK_BIT(r)	(1u << (r))
#define	SAM_NVMCTRL1_PAGES_PER_ROW	4u

/*
 * Type-2 Register Definitions
 */
#define	SAM_NVMCTRL2_CTRL		0x04u	/* 32-bits */
#define	 SAM_CTRL2_CACHEDIS		(1u << 18)
#define	 SAM_CTRL2_FWUP			(1u << 11)
#define	 SAM_CTRL2_RWS_SHIFT		1
#define	 SAM_CTRL2_RWS_MASK		(0x0fu << 1)

#define	SAM_NVMCTRL2_CMD		0x00u	/* 16-bits */
#define	 SAM_CMD2_ER			SAM_NVM_CMD(0x02u)
#define	 SAM_CMD2_WP			SAM_NVM_CMD(0x04u)
#define	 SAM_CMD2_PBC			SAM_NVM_CMD(0x44u)
#define	 SAM_CMD2_INVALL		SAM_NVM_CMD(0x46u)
#define	 SAM_CMD2_SDAL0			SAM_NVM_CMD(0x4bu)
#define	 SAM_CMD2_SDAL1			SAM_NVM_CMD(0x4cu) /* TZ devs only */

#define	SAM_NVMCTRL2_CTRLC		0x08u	/* 8-bits */
#define	 SAM_CTRLC2_MANW		(1u << 0)

#define	SAM_NVMCTRL2_EVCTRL		0x0au	/* 8-bits */
#define	SAM_NVMCTRL2_INTENCLR		0x0cu	/* 8-bits */
#define	SAM_NVMCTRL2_INTENSET		0x10u	/* 8-bits */
#define	SAM_NVMCTRL2_INTFLAG		0x14u	/* 8-bits */
#define	 SAM_INT2_DONE			(1u << 0)
#define	 SAM_INT2_PROGE			(1u << 1)
#define	 SAM_INT2_LOCKE			(1u << 2)
#define	 SAM_INT2_NVME			(1u << 3)
#define	 SAM_INT2_KEYE			(1u << 4)
#define	 SAM_INT2_NSCHK			(1u << 5)
#define	 SAM_INT2_ALL_ERRORS		(SAM_INT2_PROGE | \
					 SAM_INT2_LOCKE | \
					 SAM_INT2_NVME | \
					 SAM_INT2_KEYE | \
					 SAM_INT2_NSCHK)

#define	SAM_NVMCTRL2_STATUS		0x18u	/* 16-bits */
#define	 SAM_STATUS2_PRM		(1u << 0)
#define	 SAM_STATUS2_LOAD		(1u << 1)
#define	 SAM_STATUS2_READY		(1u << 2)
#define	 SAM_STATUS2_DALFUSE(x)		(((x) >> 3) & 0x3u)

#define	SAM_NVMCTRL2_ADDR		0x1cu	/* 32-bits */
#define	 SAM_ADDR2(a)			((a) & 0x00c0ffffu)

#define	SAM_NVMCTRL2_SULCK		0x20u	/* 16-bits */
#define	SAM_NVMCTRL2_NSULCK		0x22u	/* 16-bits */

#define	SAM_NVMCTRL2_PARAM		0x24u	/* 32-bits */
#define	 SAM_PARAM2_DFLASHP(p)		(((p) >> 20) & 0x00000fffu)

#define	SAM_NVMCTRL2_DSCC		0x30u	/* 32-bits */
#define	SAM_NVMCTRL2_SECCTRL		0x34u	/* 32-bits */
#define	SAM_NVMCTRL2_SCFGB		0x38u	/* 32-bits */
#define	SAM_NVMCTRL2_SCFGAD		0x3cu	/* 32-bits */
#define	SAM_NVMCTRL2_NONSEC		0x40u	/* 32-bits */
#define	SAM_NVMCTRL2_NSCHK		0x44u	/* 32-bits */

static const uint8_t sam_nvmctrl0_flash_applet[] = {
#include "flash-applets/applet-sam-nvmctrl-t0.applet"
};
static const uint8_t sam_nvmctrl1_flash_applet[] = {
#include "flash-applets/applet-sam-nvmctrl-t1.applet"
};

/*
 * Convenience functions for read/write of NVMCTRL registers
 */
static int
sam_nvmctrl_read_reg8(struct sam_nvmctrl_state *ns, uint32_t reg, uint8_t *pv)
{
	int rv;

	rv = TARGET_MEM_READ(ns->ns_target, ns->ns_nvmctrl_regs + reg,
	    sizeof(*pv), pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_read_reg16(struct sam_nvmctrl_state *ns, uint32_t reg, uint16_t *pv)
{
	int rv;

	rv = TARGET_MEM_READ(ns->ns_target, ns->ns_nvmctrl_regs + reg,
	    sizeof(*pv), pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_read_reg32(struct sam_nvmctrl_state *ns, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(ns->ns_target, ns->ns_nvmctrl_regs + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_write_reg8(struct sam_nvmctrl_state *ns, uint32_t reg, uint8_t v)
{
	int rv;

	rv = TARGET_MEM_WRITE(ns->ns_target, ns->ns_nvmctrl_regs + reg,
	    sizeof(v), &v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_write_reg16(struct sam_nvmctrl_state *ns, uint32_t reg, uint16_t v)
{
	int rv;

	rv = TARGET_MEM_WRITE(ns->ns_target, ns->ns_nvmctrl_regs + reg,
	    sizeof(v), &v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_write_reg32(struct sam_nvmctrl_state *ns, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(ns->ns_target, ns->ns_nvmctrl_regs + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(ns->ns_target);

	return rv;
}

static int
sam_nvmctrl_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;

	/* Size of this sector */
	fs->fs_size = nf->nf_erase_size;

	/* Number of pages in this sector */
	fs->fs_pages = nf->nf_erase_size / tf->tf_max_page_size;

	/* Erase sector number */
	fs->fs_number = *offset / fs->fs_size;

	/* Offset within sector */
	*offset %= fs->fs_size;

	fs->fs_cookie = (uintptr_t)nf->nf_base;

	return 0;
}

static int
sam_nvmctrl_erase_user_common(struct sam_nvmctrl_state *ns, target_flash_t tf)
{
	int rv;

	/*
	 * Erase the in-memory copy of the user page. This is performed
	 * by SoC-specific code to ensure factory reserved regions are
	 * preserved.
	 * We'll perform the real erase at the end, before copying the
	 * modified user page back in place.
	 */
	if (ns->ns_user_erase != NULL) {
		rv = ns->ns_user_erase(ns->ns_user_cookie, ns->ns_user_page,
		    ns->ns_user_size);
	} else {
		memset(ns->ns_user_page, tf->tf_erase_byte, ns->ns_user_size);
		rv = TARGET_FLASH_RESULT_OK;
	}

	if (rv >= TARGET_FLASH_RESULT_OK)
		ns->ns_user_dirty = true;

	return rv;
}

/*
 * =============
 * Type-0 Driver
 * =============
 *
 * Memory space is divided in two:
 *
 *  o The main address space where 2 physical NVM banks (BANKA and
 *    BANKB) are mapped.
 *  o The auxiliary space which contains:
 *     - The User page (USER) at 0x00804000
 *     - Read-only calibration page (CB) at 0x00800000
 *     - Read-only Factory and signature pages (FS) at 0x00806000
 *
 * BANKA and BANKB can be swapped in the address space.
 * Refer to the Physical Memory Map for memory sizes and addresses for
 * each device.
 *
 * BANKA, BANKB and AUX pages have different erase and write
 * granularities, see the table below.
 *
 *			Erase Granularity	Write Granularity
 *      =========================================================
 *	BANKA		Block			Quad-Word or Page
 *	BANKB		Block			Quad-Word or Page
 *	AUX		Page			Quad-Word
 *
 * The NVM is organized into two banks, each bank is organized into
 * blocks, where each block contains sixteen pages.
 * The lower blocks in the NVM main address space can be allocated as
 * a boot loader section by using the BOOTPROT fuses, and the upper
 * rows can be allocated to EEPROM
 */
static int
sam_nvmctrl0_wait_ready(struct sam_nvmctrl_state *ns)
{
	timer_timeout_t to;
	uint16_t status;

	timer_timeout_start(&to, 2000);
	do {
		if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL0_STATUS,
		    &status) < 0) {
			DBFPRINTF("Failed to read STATUS.\n");
			return -1;
		}
	} while ((status & SAM_STATUS0_READY) == 0 &&
	    !timer_timeout_expired(&to));

	if ((status & SAM_STATUS0_READY) == 0)
		DBFPRINTF("Timeout waiting for STATUS.READY: %04x\n", status);

	return ((status & SAM_STATUS0_READY) != 0) ? 0 : -1;
}

static int
sam_nvmctrl0_wait_done(struct sam_nvmctrl_state *ns)
{
	timer_timeout_t to;
	uint16_t intflag;

	timer_timeout_start(&to, 2000);
	do {
		if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL0_INTFLAG,
		    &intflag) < 0) {
			DBFPRINTF("Failed to read INTFLAG.\n");
			return -1;
		}
	} while ((intflag & SAM_INT0_DONE) == 0 && !timer_timeout_expired(&to));

	if ((intflag & SAM_INT0_DONE) == 0)
		DBFPRINTF("Timeout waiting for INTFLAG.DONE: %04x\n", intflag);

	if ((intflag & SAM_INT0_ALL_ERRORS) != 0) {
		(void) sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL0_INTFLAG,
		    SAM_INT0_ALL_ERRORS);
		DBFPRINTF("INTFLAG errors: %04x\n", intflag);
	}

	return ((intflag & SAM_INT0_ALL_ERRORS) == 0) ? 0 : -1;
}

static int
sam_nvmctrl0_send_command(struct sam_nvmctrl_state *ns, uint32_t addr,
    uint16_t cmd)
{
	int rv;

	if (sam_nvmctrl0_wait_ready(ns) < 0) {
		DBFPRINTF("Controller not ready.\n");
		return -1;
	}

	if (SAM_NVMCTRL0_CMD_NEEDS_ADDR(cmd) && sam_nvmctrl_write_reg32(ns,
	    SAM_NVMCTRL0_ADDR, SAM_ADDR0(addr)) < 0) {
		DBFPRINTF("Failed to write address.\n");
		return -1;
	}

	if (sam_nvmctrl0_wait_ready(ns) < 0) {
		DBFPRINTF("Controller not ready after address.\n");
		return -1;
	}

	if (sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL0_CMD, cmd) < 0) {
		DBFPRINTF("Failed to write command.\n");
		return -1;
	}

	if ((rv = sam_nvmctrl0_wait_done(ns)) < 0)
		DBFPRINTF("Command failed.\n");

	return rv;
}

static int
sam_nvmctrl0_init(struct sam_nvmctrl_state *ns)
{
	uint16_t ctrl, status;

	if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL0_CTRL, &ctrl) < 0) {
		DBFPRINTF("Failed to read CTRL\n");
		return -1;
	}

	ns->ns_save_ctrl = (uint32_t) ctrl;

	/*
	 * We want the cache disabled, auto-wait states, and
	 * Manual programming mode.
	 */
	ctrl = SAM_CTRL0_CACHEDIS1 | SAM_CTRL0_CACHEDIS0;
	ctrl |= SAM_CTRL0_WMODE_MAN << SAM_CTRL0_WMODE_SHIFT;
	ctrl |= SAM_CTRL0_AUTOWS;

	if (sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL0_CTRL, ctrl) < 0) {
		DBFPRINTF("Failed to init CTRL\n");
		return -1;
	}

	/* Fetch the lock bits */
	if (sam_nvmctrl_read_reg32(ns, SAM_NVMCTRL0_RUNLOCK,
	    &ns->ns_lockbits) < 0) {
		DBFPRINTF("Failed to read locks\n");
		return -1;
	}

	/* Clear the page buffer */
	if (sam_nvmctrl0_send_command(ns, 0, SAM_CMD0_PBC) < 0) {
		DBFPRINTF("clear PB failed\n");
		return -1;
	}

	/* Check if we need to issue BPDIS to write the bootloader. */
	if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL0_STATUS, &status) < 0) {
		DBFPRINTF("Failed to read STATUS\n");
		return -1;
	}
	if ((status & SAM_STATUS0_BPDIS) == 0 &&
	    SAM_STATUS0_BOOTPROT(status) != 0) {
		/* Issue BPDIS. */
		if (sam_nvmctrl0_send_command(ns, 0, SAM_CMD0_SBPDIS) < 0) {
			DBFPRINTF("SBPDIS failed.\n");
			return -1;
		}
	}

	/*
	 * Make a copy of the user page.
	 */
	if (ns->ns_user_page != NULL &&
	    TARGET_MEM_READ(ns->ns_target, ns->ns_flash[SAM_TF_USER].nf_base,
	    ns->ns_user_size, ns->ns_user_page) < 0) {
		DBFPRINTF("User page read failed\n");
		return -1;
	}

	ns->ns_user_dirty = false;
	ns->ns_init_done = true;
	return 0;
}

static int
sam_nvmctrl0_erase(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t offset, addr, cmd;

	assert(nf == &ns->ns_flash[SAM_TF_FLASH] ||
	    nf == &ns->ns_flash[SAM_TF_USER] ||
	    nf == &ns->ns_flash[SAM_TF_SEEP]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl0_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* SmartEEPROM requires no further action. */
	if (nf == &ns->ns_flash[SAM_TF_SEEP])
		return TARGET_FLASH_RESULT_OK;

	/* Calculate the address of this sector */
	offset = fs->fs_number * fs->fs_size;
	addr = (uint32_t)fs->fs_cookie + offset;

	if (nf == &ns->ns_flash[SAM_TF_FLASH]) {
		/*
		 * Check if the region containing this sector is locked.
		 */
		unsigned int bit = SAM_OFFSET_TO_LOCKBIT(ns, offset);

		if ((ns->ns_lockbits & SAM_NVMCTRL0_LOCK_BIT(bit)) == 0) {
			/* Unlock region */
			DBFPRINTF("Unlocking region %u\n", bit);
			if (sam_nvmctrl0_send_command(ns, addr,
			    SAM_CMD0_UR) < 0) {
				DBFPRINTF("unlock failed @ region %08" PRIx32
				    "\n", offset);
				return TARGET_FLASH_RESULT_ERROR;
			}

			ns->ns_lockbits |= SAM_NVMCTRL0_LOCK_BIT(bit);
		}

		cmd = SAM_CMD0_EB;

	} else {
		DBFPRINTF("Erasing User row page %" PRIu32 "\n", fs->fs_number);

		cmd = SAM_CMD0_EP;
	}

	/* Erase the sector/page */
	if (sam_nvmctrl0_send_command(ns, addr, cmd) < 0) {
		DBFPRINTF("erase failed @ %08" PRIx32 ", sector %08" PRIx32
		    "\n", addr, fs->fs_number);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_nvmctrl0_erase_user(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;

	(void) fs;

	assert(nf == &ns->ns_flash[SAM_TF_USER]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl0_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	return sam_nvmctrl_erase_user_common(ns, tf);
}

/*
 * =============
 * Type-1 Driver
 * =============
 */
static int
sam_nvmctrl1_wait_ready(struct sam_nvmctrl_state *ns)
{
	timer_timeout_t to;
	uint8_t stat8;

	timer_timeout_start(&to, 2000);

	do {
		if (sam_nvmctrl_read_reg8(ns, SAM_NVMCTRL1_INTFLAG,
		    &stat8) < 0) {
			return -1;
		}
	}  while ((stat8 & SAM_INT1_READY) == 0 && !timer_timeout_expired(&to));

	if (stat8 & SAM_INT1_ERROR) {
		uint16_t stat16;
		(void) sam_nvmctrl_write_reg8(ns, SAM_NVMCTRL1_INTFLAG,
		    SAM_INT1_ERROR);
		(void) sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL1_STATUS, &stat16);
		(void) sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL1_STATUS, stat16);
		return -1;
	}

	return ((stat8 & SAM_INT1_READY) == 0) ? -1 : 0;
}

static int
sam_nvmctrl1_send_command(struct sam_nvmctrl_state *ns, uint32_t addr,
    uint16_t cmd)
{

	if (sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL1_ADDR, SAM_ADDR1(addr)) < 0)
		return -1;

	if (sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL1_CMD, cmd) < 0)
		return -1;

	return sam_nvmctrl1_wait_ready(ns);
}

static int
sam_nvmctrl1_init(struct sam_nvmctrl_state *ns)
{
	uint32_t ctrl;
	uint16_t locks;

	if (sam_nvmctrl_read_reg32(ns, SAM_NVMCTRL1_CTRL,
	    &ns->ns_save_ctrl) < 0) {
		DBFPRINTF("Failed to read CTRL\n");
		return -1;
	}

	/*
	 * We want the cache disabled, manual programming
	 * and the required number of wait states.
	 */
	ctrl = SAM_CTRL1_CACHEDIS | SAM_CTRL1_MANW;
	ctrl |= SAM_NVMCTRL_WS(ns) << SAM_CTRL1_RWS_SHIFT;

	if (sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL1_CTRL, ctrl) < 0) {
		DBFPRINTF("Failed to init CTRL\n");
		return -1;
	}

	/* Fetch the lock bits */
	if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL1_LOCK, &locks) < 0) {
		DBFPRINTF("Failed to read locks\n");
		return -1;
	}

	ns->ns_lockbits = (uint32_t) locks;

	/* Clear the page buffer */
	if (sam_nvmctrl1_send_command(ns, 0, SAM_CMD1_PBC) < 0) {
		DBFPRINTF("clear PB failed\n");
		return -1;
	}

	/*
	 * Make a copy of the user page.
	 */
	if (ns->ns_user_page != NULL &&
	    TARGET_MEM_READ(ns->ns_target, ns->ns_flash[SAM_TF_USER].nf_base,
	    ns->ns_user_size, ns->ns_user_page) < 0) {
		DBFPRINTF("User page read failed\n");
		return -1;
	}

	ns->ns_user_dirty = false;
	ns->ns_init_done = true;

	return 0;
}

static int
sam_nvmctrl1_erase(target_flash_t tf,
    const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t offset, addr;
	unsigned int bit;
	uint16_t cmd;

	assert(nf == &ns->ns_flash[SAM_TF_FLASH] ||
	    nf == &ns->ns_flash[SAM_TF_USER] ||
	    nf == &ns->ns_flash[SAM_TF_RWWEEP]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl1_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Calculate the address of this sector */
	offset = fs->fs_number * fs->fs_size;
	addr = (uint32_t)fs->fs_cookie + offset;

	if (nf == &ns->ns_flash[SAM_TF_FLASH]) {
		/*
		 * Check if the region containing this sector is locked.
		 */
		bit = SAM_OFFSET_TO_LOCKBIT(ns, offset);

		if ((ns->ns_lockbits & SAM_NVMCTRL1_LOCK_BIT(bit)) == 0) {
			/* Unlock region */
			DBFPRINTF("Unlocking region %u\n", bit);
			if (sam_nvmctrl1_send_command(ns, addr,
			    SAM_CMD1_UR) < 0) {
				DBFPRINTF("unlock failed @ 0x%08" PRIx32 ", row"
				    " %" PRIu32 "\n", addr, fs->fs_number);
				return TARGET_FLASH_RESULT_ERROR;
			}

			ns->ns_lockbits |= SAM_NVMCTRL1_LOCK_BIT(bit);
		}

#if 0
		DBFPRINTF("Erasing row %" PRIu32 " @ 0x%08" PRIx32 "\n",
		    fs->fs_number, addr);
#endif
		cmd = SAM_CMD1_ER;
	} else
	if (nf == &ns->ns_flash[SAM_TF_RWWEEP]) {
		DBFPRINTF("Erasing RWWEEP page @ 0x%08" PRIx32 "\n", addr);

		cmd = SAM_CMD1_RWWEEER;
	} else {
		DBFPRINTF("Erasing user page @ 0x%08" PRIx32 "\n", addr);

		cmd = SAM_CMD1_EAR;
	}

	/* Erase the row */
	if (sam_nvmctrl1_send_command(ns, addr, cmd) < 0){
		DBFPRINTF("erase failed for row %" PRIu32 "@ 0x%08" PRIx32 "\n",
		    fs->fs_number, addr);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_nvmctrl1_erase_user(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;

	(void) fs;

	assert(nf == &ns->ns_flash[SAM_TF_USER]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl1_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	return sam_nvmctrl_erase_user_common(ns, tf);
}

/*
 * =============
 * Type-2 Driver
 * =============
 */
static int
sam_nvmctrl2_wait_ready(struct sam_nvmctrl_state *ns)
{
	timer_timeout_t to;

	timer_timeout_start(&to, 2000);
	do {
		uint16_t status;
		if (sam_nvmctrl_read_reg16(ns, SAM_NVMCTRL2_STATUS, &status)<0)
			return -1;
		if ((status & SAM_STATUS2_READY) != 0)
			break;
	} while (!timer_timeout_expired(&to));

	return timer_timeout_expired(&to) ? -1 : 0;
}

static int
sam_nvmctrl2_send_command(struct sam_nvmctrl_state *ns, uint32_t addr,
    uint16_t cmd)
{
	timer_timeout_t to;
	uint8_t stat8;

	if (sam_nvmctrl2_wait_ready(ns) < 0)
		return -1;

	if (sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL2_ADDR, SAM_ADDR2(addr)) < 0)
		return -1;

	if (sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL2_CMD, cmd) < 0)
		return -1;

	timer_timeout_start(&to, 2000);

	do {
		if (sam_nvmctrl_read_reg8(ns, SAM_NVMCTRL2_INTFLAG,
		    &stat8) < 0) {
			return -1;
		}
	}  while ((stat8 & SAM_INT2_DONE) == 0 && !timer_timeout_expired(&to));

	(void) sam_nvmctrl_write_reg8(ns, SAM_NVMCTRL2_INTFLAG, stat8);
	if ((stat8 & SAM_INT2_ALL_ERRORS) != 0)
		return -1;

	return ((stat8 & SAM_INT2_DONE) == 0) ? -1 : 0;
}

static int
sam_nvmctrl2_init(struct sam_nvmctrl_state *ns)
{
	uint32_t ctrl;
	uint8_t ctrlc;

	if (sam_nvmctrl_read_reg32(ns, SAM_NVMCTRL2_CTRL,
	    &ns->ns_save_ctrl) < 0) {
		DBFPRINTF("Failed to read CTRL\n");
		return -1;
	}

	if (sam_nvmctrl_read_reg8(ns, SAM_NVMCTRL2_CTRLC, &ctrlc) < 0) {
		DBFPRINTF("Failed to read CTRLC\n");
		return -1;
	}

	ns->ns_save_ctrl &= 0x00ffffffu;
	ns->ns_save_ctrl |= (uint32_t)ctrlc << 24;

	if (sam_nvmctrl_write_reg8(ns, SAM_NVMCTRL2_INTFLAG,
	    SAM_INT2_DONE | SAM_INT2_ALL_ERRORS) < 0) {
		DBFPRINTF("Failed to write INTFLAG\n");
		return -1;
	}

	/*
	 * We want the cache disabled, manual programming
	 * and the required number of wait states.
	 */
	ctrl = SAM_CTRL2_CACHEDIS;
	ctrl |= SAM_NVMCTRL_WS(ns) << SAM_CTRL2_RWS_SHIFT;
	ctrlc |= SAM_CTRLC2_MANW;

	if (sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL2_CTRL, ctrl) < 0) {
		DBFPRINTF("Failed to init CTRL\n");
		return -1;
	}

	if (sam_nvmctrl_write_reg8(ns, SAM_NVMCTRL2_CTRLC, ctrlc) < 0) {
		DBFPRINTF("Failed to init CTRLC\n");
		return -1;
	}

	/*
	 * Make a copy of the user page.
	 */
	if (ns->ns_user_page != NULL &&
	    TARGET_MEM_READ(ns->ns_target, ns->ns_flash[SAM_TF_USER].nf_base,
	    ns->ns_user_size, ns->ns_user_page) < 0) {
		DBFPRINTF("User page read failed\n");
		return -1;
	}

	/* Clear the page buffer */
	if (sam_nvmctrl2_send_command(ns, 0, SAM_CMD2_PBC) < 0) {
		DBFPRINTF("clear PB failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Invalidate all cache lines */
	if (sam_nvmctrl2_send_command(ns, 0, SAM_CMD2_INVALL) < 0) {
		DBFPRINTF("clear PB failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	ns->ns_user_dirty = false;
	ns->ns_init_done = true;

	return 0;
}

static int
sam_nvmctrl2_erase(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t offset, addr;

	assert(nf == &ns->ns_flash[SAM_TF_FLASH] ||
	    nf == &ns->ns_flash[SAM_TF_USER]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl2_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Calculate the address of this sector */
	offset = fs->fs_number * fs->fs_size;
	addr = (uint32_t)fs->fs_cookie + offset;

	if (nf == &ns->ns_flash[SAM_TF_FLASH]) {
		/* XXX: Lock bits. */

#if 0
		DBFPRINTF("Erasing row %" PRIu32 " @ 0x%08" PRIx32 "\n",
		    fs->fs_number, addr);
#endif
	} else {
		DBFPRINTF("Erasing user page @ 0x%08" PRIx32 "\n", addr);
	}

	/* Erase the row */
	if (sam_nvmctrl2_send_command(ns, addr, SAM_CMD2_ER) < 0){
		DBFPRINTF("erase failed for row %" PRIu32 "@ 0x%08" PRIx32 "\n",
		    fs->fs_number, addr);
		return TARGET_FLASH_RESULT_ERROR;
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
sam_nvmctrl2_erase_user(target_flash_t tf, const struct target_flash_sector *fs)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;

	(void) fs;

	assert(nf == &ns->ns_flash[SAM_TF_USER]);

	/*
	 * Stash the current value of CTRL. We'll restore it when done.
	 */
	if (!ns->ns_init_done && sam_nvmctrl2_init(ns) < 0) {
		DBFPRINTF("init failed\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	return sam_nvmctrl_erase_user_common(ns, tf);
}

static int
sam_nvmctrl_write(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t faddr;
	uint16_t cmd;
	int rv;

	/*
	 * The supplied page number is relative to the start of the erase
	 * sector. Adjust so that it is relative to the start of the Flash
	 * region.
	 */
	page += fs->fs_pages * fs->fs_number;

	/* Calculate the physical address within the Flash region */
	faddr = nf->nf_base + (page * tf->tf_max_page_size);

	DBFPRINTF("page %" PRIu32 ", faddr 0x%08" PRIx32 "\n", page, faddr);

	if (nf->nf_flash_applet == NULL) {
		/* Fill the page buffer */
		if (TARGET_MEM_WRITE(ns->ns_target, faddr,
		    tf->tf_max_page_size, tf->tf_buffer) < 0) {
			(void) TARGET_LINK_ERROR(ns->ns_target);
			DBFPRINTF("failed to fill page buffer\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		/* Write the page */
		switch (SAM_NVMCTRL_TYPE(ns)) {
		case FLASH_SAM_NVMCTRL_TYPE_0:
			if (nf == &ns->ns_flash[SAM_TF_FLASH])
				cmd = SAM_CMD0_WP;
			else
				cmd = SAM_CMD0_WQW;
			rv = sam_nvmctrl0_send_command(ns, faddr, cmd);
			break;
	
		case FLASH_SAM_NVMCTRL_TYPE_1:
			if (nf == &ns->ns_flash[SAM_TF_FLASH])
				cmd = SAM_CMD1_WP;
			else
			if (nf == &ns->ns_flash[SAM_TF_RWWEEP])
				cmd = SAM_CMD1_RWWEEWP;
			else
				cmd = SAM_CMD1_WAP;
			rv = sam_nvmctrl1_send_command(ns, faddr, cmd);
			break;

		case FLASH_SAM_NVMCTRL_TYPE_2:
			rv = sam_nvmctrl2_send_command(ns, faddr, SAM_CMD2_WP);
			break;

		default:
			rv = -1;
			break;
		}

		rv = (rv < 0) ? TARGET_FLASH_RESULT_ERROR :
		    TARGET_FLASH_RESULT_OK;
	} else {
		rv = flash_applet_write_page(tf, faddr, tf->tf_max_page_size,
		    ns->ns_nvmctrl_regs, 0, nf->nf_flash_applet);
	}

	if (rv == TARGET_FLASH_RESULT_ERROR)
		DBFPRINTF("write failed @ page %08" PRIx32 "\n", faddr);

	DBFPRINTF("rv %d\n", rv);
	return rv;
}

static int
sam_nvmctr_write_user(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t poff;

	(void) fs;

	/* Calculate the offset within the user page */
	poff = page * tf->tf_max_page_size;

	DBFPRINTF("page %" PRIu32 ", poff 0x%04" PRIx32 "\n", page, poff);

	if (ns->ns_user_check != NULL) {
		/*
		 * Invoke the MD validation function. It'll ensure reserved
		 * fields are not changed.
		 */
		if (ns->ns_user_check(ns->ns_user_cookie,
		    &ns->ns_user_page[poff], page, tf->tf_buffer) < 0) {
			DBFPRINTF("user check failed\n");
			return TARGET_FLASH_RESULT_ERROR;
		}
	}

	/* Copy the new page into the buffer, but only if changed. */
	if (memcmp(&ns->ns_user_page[poff], tf->tf_buffer,
	    tf->tf_max_page_size) != 0) {
		memcpy(&ns->ns_user_page[poff], tf->tf_buffer,
		    tf->tf_max_page_size);
		ns->ns_user_dirty = true;
	}

	return TARGET_FLASH_RESULT_VERIFIED;	/* XXX: Not strictly true... */
}

static int
sam_nvmctrl_seestat_idle(struct sam_nvmctrl_state *ns, uint32_t *sp)
{
	timer_timeout_t to;
	uint32_t reg;

	timer_timeout_start(&to, 2000);

	/* Wait for Flash controller idle. */
	do {
		rtos_yield();
		if (sam_nvmctrl_read_reg32(ns, SAM_NVMCTRL0_SEESTAT, &reg) < 0)
			return -1;
	} while ((reg & SAM_SEESTAT0_BUSY) != 0 && !timer_timeout_expired(&to));

	if (sp != NULL)
		*sp = reg;

	return ((reg & SAM_SEESTAT0_BUSY) == 0) ? 0 : -1;
}

static int
sam_nvmctrl_write_seep(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	uint32_t faddr, reg;
	int rv;

	/* Calculate the physical address within the Flash region */
	page += fs->fs_pages * fs->fs_number;
	faddr = nf->nf_base + (page * tf->tf_max_page_size);

	DBFPRINTF("page %" PRIu32 ", faddr 0x%08" PRIx32 "\n", page, faddr);

	if (sam_nvmctrl_seestat_idle(ns, NULL) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	/* Copy the data. */
	if (TARGET_MEM_WRITE(ns->ns_target, faddr,
	    tf->tf_max_page_size, tf->tf_buffer) < 0) {
		return TARGET_FLASH_RESULT_ERROR;
	}

	if (sam_nvmctrl_seestat_idle(ns, &reg) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	if ((reg & SAM_SEESTAT0_LOAD) != 0) {
		rv = sam_nvmctrl0_send_command(ns, 0, SAM_CMD0_SEEFLUSH);
		if (rv < 0)
			return TARGET_FLASH_RESULT_ERROR;
		rv = sam_nvmctrl_seestat_idle(ns, NULL);
		rv = (rv < 0) ? TARGET_FLASH_RESULT_ERROR :
		    TARGET_FLASH_RESULT_OK;
	} else {
		rv = TARGET_FLASH_RESULT_OK;
	}

	return rv;
}

static int
sam_nvmctrl_flush_user_row(struct sam_nvmctrl_state *ns)
{
	target_flash_t mtf = &ns->ns_flash[SAM_TF_FLASH].nf_tf;
	target_flash_t tf = &ns->ns_flash[SAM_TF_USER].nf_tf;
	struct target_flash_sector fs;
	target_addr_t off;
	int rv;

	DBFPRINTF("flushing user row\n");

	/* Erase the user row. */
	off = 0;
	(void) tf->tf_sector_desc(tf, &off, &fs);

	/*
	 * Use the erase sector handler for the main Flash region.
	 */
	rv = mtf->tf_erase_sector(tf, &fs);
	if (rv < 0) {
		DBFPRINTF("erase failed.\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	/* Flush each page of the user row. */
	for (off = 0; off < ns->ns_user_size; off += tf->tf_max_page_size) {
		/* Copy next page into the buffer. */
		memcpy(tf->tf_buffer, &ns->ns_user_page[off],
		    tf->tf_max_page_size);

		/* Fabricate a sector descriptor. */
		(void) tf->tf_sector_desc(tf, &off, &fs);

		if (mtf->tf_write_page(tf, &fs,
		    off / tf->tf_max_page_size) < 0) {
			DBFPRINTF("write page %u failed\n",
			    (unsigned int)(off / tf->tf_max_page_size));
			rv = TARGET_FLASH_RESULT_ERROR;
		}

		/* XXX: Verify. */
	}

	return rv;
}

static int
sam_nvmctrl_finish(target_flash_t tf)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;
	int rv = TARGET_FLASH_RESULT_OK;

	if (nf->nf_flash_applet != NULL)
		rv = flash_applet_finish(nf->nf_flash_applet);

	if (ns->ns_user_dirty == true) {
		/*
		 * The in-memory copy of the user row has been modified.
		 * Now we must erase the real user row and flush our
		 * copy out to it.
		 */
		rv = sam_nvmctrl_flush_user_row(ns);
		ns->ns_user_dirty = false;
	}

	if (ns->ns_init_done == false)
		return rv;
	ns->ns_init_done = false;

	/*
	 * Restore the original CTRL register values
	 */
	switch (SAM_NVMCTRL_TYPE(ns)) {
	case FLASH_SAM_NVMCTRL_TYPE_0:
		(void) sam_nvmctrl0_wait_ready(ns);
		sam_nvmctrl_write_reg16(ns, SAM_NVMCTRL0_CTRL,
		    (uint16_t) ns->ns_save_ctrl);
		(void) sam_nvmctrl0_wait_ready(ns);
		break;

	case FLASH_SAM_NVMCTRL_TYPE_1:
		(void) sam_nvmctrl1_wait_ready(ns);
		sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL1_CTRL,
		    ns->ns_save_ctrl);
		(void) sam_nvmctrl1_wait_ready(ns);
		break;

	case FLASH_SAM_NVMCTRL_TYPE_2:
		(void) sam_nvmctrl2_wait_ready(ns);
		sam_nvmctrl_write_reg32(ns, SAM_NVMCTRL2_CTRL,
		    ns->ns_save_ctrl & 0x00ffffffu);
		sam_nvmctrl_write_reg8(ns, SAM_NVMCTRL2_CTRLC,
		    (uint8_t)(ns->ns_save_ctrl >> 24));
		(void) sam_nvmctrl2_wait_ready(ns);
		break;

	default:
		break;
	}

	return rv;
}

static void
sam_nvmctrl_free(target_flash_t tf)
{
	struct sam_nvmctrl_flash *nf = (struct sam_nvmctrl_flash *)tf;
	struct sam_nvmctrl_state *ns = nf->nf_ns;

	if (nf->nf_flash_applet != NULL)
		flash_applet_detach(nf->nf_flash_applet);

	assert(ns->ns_ref_count > 0);

	if (--(ns->ns_ref_count) == 0) {
		if (ns->ns_user_page != NULL)
			zone_free(ns->ns_user_page);
		zone_free(ns);
	}
}

void *
flash_sam_nvmctrl_attach(const struct flash_sam_nvmctrl_attach_args *aa)
{
	struct sam_nvmctrl_state *ns;
	uint32_t param_reg, param, flash_size, user_page_size, user_erase_size,
	    lock_regions, pages_per_erase_sector,
	    eeprom_size, seeprom_size, rwweep_size;
	target_flash_erase_sector_t erase_fn, erase_up_fn;
	struct flash_applet_params fp;
	void *flash_applet;
	const char *extra;

	DBFPRINTF("regs %08" PRIxTADDR ", flash %08" PRIxTADDR
	    ", rwweep %08" PRIxTADDR "\n",
	    aa->aa_nvmctrl_regs, aa->aa_flash_base, aa->aa_rwweep_base);
	DBFPRINTF("seep %08" PRIxTADDR ", user %08" PRIxTADDR
	    ", usersize %08" PRIx32 "\n",
	    aa->aa_seep_base, aa->aa_user_base, aa->aa_user_size);
	DBFPRINTF("type %04" PRIx16 ", flags %04" PRIx16 "\n",
	    aa->aa_type_ws, aa->aa_flags);

	if ((ns = zone_calloc(1, sizeof(*ns))) == NULL)
		return NULL;

	ns->ns_aa = *aa;

	if (ns->ns_user_size != 0) {
		assert(ns->ns_user_erase != NULL);
		assert(ns->ns_user_check != NULL);

		ns->ns_user_page = zone_malloc(ns->ns_user_size);
		if (ns->ns_user_page == NULL) {
			zone_free(ns);
			return NULL;
		}
	} else {
		ns->ns_user_page = NULL;
	}

	ns->ns_init_done = false;
	ns->ns_user_dirty = false;
	user_erase_size = 0;
	eeprom_size = 0;
	seeprom_size = 0;
	rwweep_size = 0;

	switch (SAM_NVMCTRL_TYPE(ns)) {
	case FLASH_SAM_NVMCTRL_TYPE_0:
		erase_fn = sam_nvmctrl0_erase;
		erase_up_fn = sam_nvmctrl0_erase_user;
		fp.fp_applet = sam_nvmctrl0_flash_applet;
		fp.fp_applet_size = sizeof(sam_nvmctrl0_flash_applet);
		fp.fp_applet_stack_size = 0;
		lock_regions = 32;
		pages_per_erase_sector = 16;
		user_page_size = 16u;	/* Quad word: 128 bits */
		param_reg = SAM_NVMCTRL0_PARAM;
		if (ns->ns_flags & FLASH_SAM_NVMCTRL_FLAG_EEP) {
			uint32_t seemax;

			if (sam_nvmctrl_read_reg32(ns, SAM_NVMCTRL0_SEESTAT,
			    &param) < 0) {
				zone_free(ns);
				DBFPRINTF("failed to read SEESTAT\n");
				return NULL;
			}

			/*
			 * Figure out the SmartEEPROM 'virtual' size,
			 * and number of main Flash pages allocated to it.
			 */
			eeprom_size = SAM_SEESTAT0_SBLK(param);
			if (eeprom_size && eeprom_size <= 10u) {
				/* seeprom_size is the 'virtual' size. */
				seeprom_size = SAM_SEESTAT0_PSZ(param) * 128u;

				/* Its max value is constrained by SBLK. */
				seemax = 512u << eeprom_size;
				if (seeprom_size > seemax)
					seeprom_size = seemax;

				/* Figure out # main Flash pages stolen. */
				eeprom_size *= 2u;
			} else {
				eeprom_size = 0;
				seeprom_size = 0;
			}
		}
		break;

	case FLASH_SAM_NVMCTRL_TYPE_1:
		erase_fn = sam_nvmctrl1_erase;
		erase_up_fn = sam_nvmctrl1_erase_user;
		fp.fp_applet = sam_nvmctrl1_flash_applet;
		fp.fp_applet_size = sizeof(sam_nvmctrl1_flash_applet);
		fp.fp_applet_stack_size = 0;
		lock_regions = 16;
		pages_per_erase_sector = 4;
		user_page_size = 8u;
		param_reg = SAM_NVMCTRL1_PARAM;
		if (ns->ns_flags & FLASH_SAM_NVMCTRL_FLAG_EEP) {
			uint32_t ur;

			if (TARGET_REG_READ32(ns->ns_target, ns->ns_user_base,
			    &ur) < 0) {
				(void) TARGET_LINK_ERROR(ns->ns_target);
				DBFPRINTF("User Row read " "failed\n");
			} else {
				ur >>= 4;
				ur &= 7u;
				/* Compute EEPROM size according to User Row. */
				eeprom_size = (0x4000 >> ur) & ~0xffu;
			}
		}
		break;

	case FLASH_SAM_NVMCTRL_TYPE_2:
		erase_fn = sam_nvmctrl2_erase;
		erase_up_fn = sam_nvmctrl2_erase_user;
		fp.fp_applet = NULL;
		lock_regions = 16;
		pages_per_erase_sector = 4;
		user_page_size = 32u;	/* 256 bits */
		param_reg = SAM_NVMCTRL2_PARAM;
		break;

	default:
		zone_free(ns);
		return NULL;
	}

	/*
	 * The Flash is self-describing via the PARAM Register
	 */
	if (sam_nvmctrl_read_reg32(ns, param_reg, &param) < 0) {
		zone_free(ns);
		DBFPRINTF("failed to read PARAM\n");
		return NULL;
	}

	DBFPRINTF("PARAM %08" PRIx32 "\n", param);

	/* Work out total size of Flash */
	flash_size = SAM_PARAM_PSZ(param) * SAM_PARAM_NVMP(param);
	if (flash_size == 0) {
		zone_free(ns);
		DBFPRINTF("Flash size zero!\n");
		return NULL;
	}

	ns->ns_lock_region_size = flash_size / lock_regions;

	if (SAM_NVMCTRL_TYPE(ns) == FLASH_SAM_NVMCTRL_TYPE_0) {
		/* Override the default if SmartEEPROM is not supported. */
		if ((param & SAM_PARAM0_SEE) == 0) {
			seeprom_size = eeprom_size = 0;
		}
		/* Work out the number of bytes assigned to EEPROM */
		eeprom_size *= pages_per_erase_sector * SAM_PARAM_PSZ(param);
		if (eeprom_size == 0)
			seeprom_size = 0;
	}

	if (eeprom_size > flash_size) {
		DBFPRINTF("EEPROM size exceeds main Flash size!\n");
		eeprom_size = 0;
	}

	if (eeprom_size == 0) {
		extra = NULL;
	} else {
		flash_size -= eeprom_size;
		snprintf(ns->ns_extra, sizeof(ns->ns_extra), "%" PRIu32 " %s "
		    "used for %sEEPROM",
		    (eeprom_size < 1024) ? eeprom_size : (eeprom_size / 1024u),
		    (eeprom_size < 1024) ? "bytes" : "KB",
		    (SAM_NVMCTRL_TYPE(ns) == FLASH_SAM_NVMCTRL_TYPE_0) ?
		    "Smart" : "");
		extra = ns->ns_extra;

		DBPRINTF("NVMCTRL-%u: Bytes reserved for EEPROM: %" PRIu32 "\n",
		    (u_int)SAM_NVMCTRL_TYPE(ns), eeprom_size);
	}

	ns->ns_flash[SAM_TF_FLASH].nf_ns = ns;
	ns->ns_flash[SAM_TF_FLASH].nf_base = ns->ns_flash_base;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_sector_desc =
	    sam_nvmctrl_sector_desc;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_erase_sector = erase_fn;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_write_page = sam_nvmctrl_write;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_finish = sam_nvmctrl_finish;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_free = sam_nvmctrl_free;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_erase_byte = 0xffu;
	ns->ns_flash[SAM_TF_FLASH].nf_erase_size =
	    pages_per_erase_sector * SAM_PARAM_PSZ(param);
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_size = flash_size;
	ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_max_page_size =
	    SAM_PARAM_PSZ(param);

	DBPRINTF("NVMCTRL-%u: Flash Base %08" PRIx32 ", Size %" PRIu32 " KB, "
	    "Erase Size 0x%" PRIx32 ", Page Size 0x%" PRIx32 "\n",
	    (u_int)SAM_NVMCTRL_TYPE(ns),
	    ns->ns_flash[SAM_TF_FLASH].nf_base,
	    ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_size / 1024u,
	    ns->ns_flash[SAM_TF_FLASH].nf_erase_size,
	    ns->ns_flash[SAM_TF_FLASH].nf_tf.tf_max_page_size);

	if (ns->ns_user_size) {
		/* Clone current Flash settings into User page settings. */
		ns->ns_flash[SAM_TF_USER] = ns->ns_flash[SAM_TF_FLASH];

		if (user_erase_size == 0)
			user_erase_size = ns->ns_user_size;

		ns->ns_flash[SAM_TF_USER].nf_base = ns->ns_user_base;
		ns->ns_flash[SAM_TF_USER].nf_erase_size = user_erase_size;
		ns->ns_flash[SAM_TF_USER].nf_tf.tf_erase_sector = erase_up_fn;
		ns->ns_flash[SAM_TF_USER].nf_tf.tf_write_page =
		    sam_nvmctr_write_user;
		ns->ns_flash[SAM_TF_USER].nf_tf.tf_size = ns->ns_user_size;
		ns->ns_flash[SAM_TF_USER].nf_tf.tf_max_page_size =
		    user_page_size;

		DBPRINTF("NVMCTRL-%u: User Base %08" PRIx32 ", Size %" PRIx32
		    ", Erase Size 0x%" PRIx32 ", Page Size 0x%" PRIx32 "\n",
		    (u_int)SAM_NVMCTRL_TYPE(ns),
		    ns->ns_flash[SAM_TF_USER].nf_base,
		    ns->ns_flash[SAM_TF_USER].nf_tf.tf_size,
		    ns->ns_flash[SAM_TF_USER].nf_erase_size,
		    ns->ns_flash[SAM_TF_USER].nf_tf.tf_max_page_size);
	}

	if ((ns->ns_flags & FLASH_SAM_NVMCTRL_FLAG_RWWEEP) != 0) {
		assert(SAM_NVMCTRL_TYPE(ns) == FLASH_SAM_NVMCTRL_TYPE_1);

		rwweep_size = SAM_PARAM1_RWWEEP(param);
		rwweep_size *= SAM_PARAM_PSZ(param);

		/* Clone current Flash settings into RWWEEP page settings. */
		ns->ns_flash[SAM_TF_RWWEEP] = ns->ns_flash[SAM_TF_FLASH];
		ns->ns_flash[SAM_TF_RWWEEP].nf_base = ns->ns_rwweep_base;
		ns->ns_flash[SAM_TF_RWWEEP].nf_tf.tf_size = rwweep_size;

		DBPRINTF("NVMCTRL-%u: RWWEEP Base %08" PRIx32 ", Size %" PRIx32
		    ", Erase Size 0x%" PRIx32 ", Page Size 0x%" PRIx32 "\n",
		    (u_int)SAM_NVMCTRL_TYPE(ns),
		    ns->ns_flash[SAM_TF_RWWEEP].nf_base,
		    ns->ns_flash[SAM_TF_RWWEEP].nf_tf.tf_size,
		    ns->ns_flash[SAM_TF_RWWEEP].nf_erase_size,
		    ns->ns_flash[SAM_TF_RWWEEP].nf_tf.tf_max_page_size);
	}

	if (seeprom_size) {
		assert(SAM_NVMCTRL_TYPE(ns) == FLASH_SAM_NVMCTRL_TYPE_0);

		/* Clone current Flash settings into SEEP settings. */
		ns->ns_flash[SAM_TF_SEEP] = ns->ns_flash[SAM_TF_FLASH];
		ns->ns_flash[SAM_TF_SEEP].nf_tf.tf_write_page =
		   sam_nvmctrl_write_seep;
		ns->ns_flash[SAM_TF_SEEP].nf_base = ns->ns_seep_base;
		ns->ns_flash[SAM_TF_SEEP].nf_erase_size = 1;
		ns->ns_flash[SAM_TF_SEEP].nf_tf.tf_size = seeprom_size;
		ns->ns_flash[SAM_TF_SEEP].nf_tf.tf_max_page_size = 1;
	}

	/* Attach to the Flash applet driver */
	flash_applet = NULL;
	if (fp.fp_applet != NULL) {
		fp.fp_max_page_size = SAM_PARAM_PSZ(param);
		flash_applet = flash_applet_attach(ns->ns_target, &fp);
		if (flash_applet == NULL) {
			DBFPRINTF("failed to attach applet\n");
		}
	}
	ns->ns_flash[SAM_TF_FLASH].nf_flash_applet = flash_applet;
	ns->ns_flash[SAM_TF_USER].nf_flash_applet = NULL;
	ns->ns_flash[SAM_TF_RWWEEP].nf_flash_applet = NULL;
	ns->ns_flash[SAM_TF_SEEP].nf_flash_applet = NULL;
	ns->ns_ref_count = 1;

	/* Add the Flash to the target */
	if (target_add_flash(ns->ns_target, &ns->ns_flash[SAM_TF_FLASH].nf_tf,
	    ns->ns_flash[SAM_TF_FLASH].nf_base, flash_size, extra, 0) < 0){
		DBFPRINTF("failed to add flash @ %08" PRIx32 "\n",
		    ns->ns_flash[SAM_TF_FLASH].nf_base);
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		zone_free(ns);
		return NULL;
	}

	/* Add user page, if required. */
	if (ns->ns_user_size != 0) {
		if (target_add_flash(ns->ns_target,
		    &ns->ns_flash[SAM_TF_USER].nf_tf,
		    ns->ns_flash[SAM_TF_USER].nf_base,
		    ns->ns_user_size, "NVM User Row", 0) < 0){
			DBFPRINTF("failed to add User page @ %08" PRIx32 "\n",
			    ns->ns_flash[SAM_TF_USER].nf_base);
		} else {
			ns->ns_ref_count++;
		}
	}

	/* Add RWWEEP, if required. */
	if (rwweep_size != 0) {
		if (target_add_flash(ns->ns_target,
		    &ns->ns_flash[SAM_TF_RWWEEP].nf_tf,
		    ns->ns_flash[SAM_TF_RWWEEP].nf_base,
		    rwweep_size, "RWW EEPROM", TARGET_MEM_EEPROM) < 0){
			DBFPRINTF("failed to add RWWEEP EEPROM @ %08" PRIx32
			    "\n", ns->ns_flash[SAM_TF_RWWEEP].nf_base);
		} else {
			ns->ns_ref_count++;
		}
	}

	/* Add SmartEEPROM, if required. */
	if (seeprom_size) {
		if (target_add_flash(ns->ns_target,
		    &ns->ns_flash[SAM_TF_SEEP].nf_tf,
		    ns->ns_flash[SAM_TF_SEEP].nf_base,
		    seeprom_size, "SmartEEPROM", TARGET_MEM_EEPROM) < 0) {
			DBFPRINTF("failed to add SmartEEPROM @ %08" PRIx32 "\n",
			    ns->ns_flash[SAM_TF_SEEP].nf_base);
		} else {
			ns->ns_ref_count++;
		}
	}

	return ns;
}

int
flash_sam_nvmctrl0_prot_set(void *cookie, struct target_protection_info *pi,
    microchip_dsu_protection_t curprot)
{
	struct sam_nvmctrl_state *ns = cookie;
	u_int curlev;
	int rv;

	if (curprot == MCP_DSU_PROT_NONE)
		curlev = 0;
	else
	if (curprot <= MCP_DSU_PROT_SECURE)
		curlev = 1;
	else
		curlev = 2;

	if (curlev == pi->pi_level)
		return TARGET_PI_UNCHANGED;

	if (curlev > pi->pi_level)
		return TARGET_PI_USE_UNLOCK;

	if (pi->pi_level == 2) {
		if (curlev != 0 || ns == NULL) {
			fprintf(pi->pi_out_file, "You can only transition to "
			    "level 2 from level 0.\n");
			return TARGET_PI_UNCHANGED;
		}

		if (pi->pi_permanent == 0)
			return TARGET_PI_NEED_PERM_FLAG;

		rv = sam_nvmctrl0_send_command(ns, 0, SAM_CMD0_CELCK);
		if (rv < 0) {
			fprintf(pi->pi_out_file, "Failed to lock CE\n");
			return -1;
		}
	}

	rv = sam_nvmctrl0_send_command(ns, 0, SAM_CMD0_SSB);
	if (rv < 0) {
		fprintf(pi->pi_out_file, "Failed to set security bit\n");
		return -1;
	}

	return TARGET_PI_OK_RESET;
}

int
flash_sam_nvmctrl1_prot_set(void *cookie, struct target_protection_info *pi,
    microchip_dsu_protection_t curprot)
{
	struct sam_nvmctrl_state *ns = cookie;
	u_int curlev;
	int rv;

	if (curprot == MCP_DSU_PROT_NONE)
		curlev = 0;
	else
		curlev = 1;

	if (curlev == pi->pi_level)
		return TARGET_PI_UNCHANGED;

	if (curlev > pi->pi_level)
		return TARGET_PI_USE_UNLOCK;

	rv = sam_nvmctrl1_send_command(ns, 0, SAM_CMD1_SSB);
	if (rv < 0) {
		fprintf(pi->pi_out_file, "Failed to set security bit\n");
		return -1;
	}

	return TARGET_PI_OK_RESET;
}

int
flash_sam_nvmctrl2_prot_set(void *cookie, struct target_protection_info *pi,
    microchip_dsu_protection_t curprot)
{
	(void) cookie;
	(void) pi;
	(void) curprot;

	/*
	 * XXX
	 *  - SAML10 Send SDAL0 command. No key required; no perm lockdown.
	 *  - SAML11: 'locked' is SDAL0 after setting CEKEY[0-2]==0xffffffff.
	 *            'secure' is SDAL0 after setting CEKEY[0-2]==0x00000000.
	 */

	return -1;
}
