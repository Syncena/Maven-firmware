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
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "adiv5.h"
#include "adiv5regs.h"
#include "adiv5_ll.h"
#include "tmon.h"
#include "timer.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "linked-lists.h"
#include "shell.h"
#include "zone_alloc.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "network_httpd.h"
#include "stringio.h"
#include "json_utils.h"
#endif
#endif /* CONFIG_USE_NETWORK */

#if (RELEASE_BUILD == 0)
/* Change this to #define for extra DP sanity checking */
#undef ADIV5_VALIDATE_DP_REG
#endif

/*
 * Debug support is extensive. Choose your logging carefully.
 * The hardware and register options in particular will be *very* chatty.
 */
#define	ADIV5_DP_HW		(1u << 0)	/* DP low-level hardware i/f */
#define	ADIV5_DP_REG		(1u << 1)	/* DP register read/write */
#define	ADIV5_DP_ERROR		(1u << 2)	/* DP error handling */
#define	ADIV5_DP_CLEAR_ERROR	(1u << 3)	/* DP error handling */
#define	ADIV5_DP_REF		(1u << 4)	/* DP reference counting */
#define	ADIV5_DP_PROBE		(1u << 5)	/* DP probing */
#define	ADIV5_DP_EXPLORE	(1u << 6)	/* DP exploring */
#define	ADIV5_DP_TARGETSEL	(1u << 7)	/* DP exploring */
#define	ADIV5_MEM_AP_REG	(1u << 16)	/* MEM-AP memory read/write */
#define	ADIV5_MEM_AP_EXPLORE	(1u << 17)	/* MEM-AP exploration */
#define	ADIV5_MEM_AP_REF	(1u << 18)	/* MEM-AP reference counting */
#define	ADIV5_MEM_AP_ATTACH	(1u << 19)	/* MEM-AP attach */
#define	ADIV5_MEM_AP_COMP_LIST	(1u << 20)	/* MEM-AP list components */
#define	ADIV5_MEM_AP_API	(1u << 21)	/* MEM-AP API read/write */
#define	ADIV5_MEM_AP_API_ERRORS	(1u << 22)	/* MEM-AP API rd/wr errors */
#define	ADIV5_OTHER_AP_ATTACH	(1u << 23)	/* Other-AP API ref attach */
#define	ADIV5_OTHER_AP_REF	(1u << 24)	/* Other-AP API ref counting */

#if 0
#define	DEBUG_FLAG	adiv5_debug_flag
#define DEBUG_FLAG_INIT	(ADIV5_DP_ERROR | ADIV5_DP_PROBE | ADIV5_DP_EXPLORE | \
			 ADIV5_DP_CLEAR_ERROR | ADIV5_MEM_AP_COMP_LIST | \
			 ADIV5_MEM_AP_API_ERRORS | ADIV5_MEM_AP_ATTACH | \
			 ADIV5_MEM_AP_EXPLORE)
#endif
#include "debug.h"
#ifdef DEBUG_FLAG
static const char *class_strings[16] = {
	[ADIV5_CLASS_GENERIC_VERIFICATION] = "Generic Verification",
	[ADIV5_CLASS_ROM_TABLE] = "ROM Table",
	[ADIV5_CLASS_CORESIGHT] = "CoreSight",
	[ADIV5_CLASS_PERIPHERAL_TEST_BLK] = "Peripheral Test Block",
	[ADIV5_CLASS_GENERIC_IP] = "Generic IP",
	[ADIV5_CLASS_CORELINK_ETC] = "CoreLink, PrimeCel, or System",
};
#endif /* DEBUG_FLAG */

SHELL_CMD_DECL(adapter, adiv5_cmd_adapter, "View/Modify debug port parameters");

const uint8_t adiv5_speed_2cfg[] = {
	ADIV5_SPEED_2CFG
};
#define	ADIV5_SPEED_2CFG_COUNT	(sizeof(adiv5_speed_2cfg) / \
				 sizeof(adiv5_speed_2cfg[0]))
static const char *adiv5_speed_2str[] = {
	ADIV5_SPEED_2STR
};
#define	ADIV5_SPEED_2STR_COUNT	(sizeof(adiv5_speed_2str) / \
				 sizeof(adiv5_speed_2str[0]))
static_assert(ADIV5_SPEED_2CFG_COUNT == ADIV5_SPEED_2STR_COUNT,
    "ADIV5_SPEED_2CFG_COUNT != ADIV5_SPEED_2STR_COUNT");
static_assert(ADIV5_SPEED_2CFG_COUNT == 9, "9 speeds please");

#ifndef ADIV5_DEFAULT_IDLE_CYCLES
#define	ADIV5_DEFAULT_IDLE_CYCLES	8
#endif
#ifndef ADIV5_DEFAULT_TURNAROUND
#define	ADIV5_DEFAULT_TURNAROUND	0
#endif
#ifndef ADIV5_DEFAULT_SPEED
#define	ADIV5_DEFAULT_SPEED		8
#endif
#ifndef ADIV5_DEFAULT_INTERFACE
#define	ADIV5_DEFAULT_INTERFACE		((1u << ADIV5_TYPE_SWDP) | \
					 (1u << ADIV5_TYPE_SWJDP))
#endif

/*
 * Type definitions
 */
struct adiv5_mem_ap;
typedef struct adiv5_mem_ap *adiv5_mem_ap_t;
struct adiv5_other_ap;
typedef struct adiv5_other_ap *adiv5_other_ap_t;

/*
 * An ADIv5 target is described by list of ROM Tables and their IDs. This
 * list provides a means to identify the debug resources available and
 * their address.
 */
struct adiv5_rom_table {
	uint64_t rt_pidr;
	target_addr_t rt_address;
};

TAILQ_HEAD(adiv5_dp_qhead, adiv5_dp);
static struct adiv5_dp_qhead adiv5_dp_list =
    TAILQ_HEAD_INITIALIZER(adiv5_dp_list);

static bool adiv5_dp_active;
static uint32_t adiv5_dp_current_targetsel;

/*
 * Common AP state
 */
struct adiv5_ap_state {
	adiv5_dp_t as_dp;
	uint32_t as_ap_sel;
	uint32_t as_idr;
	uint32_t as_error_log;
	TAILQ_ENTRY(adiv5_ap_state) as_qent;
};

/*
 * Overall state for a Memory Access Port
 */
struct adiv5_mem_ap {
	struct adiv5_ap_state ma_as;
	uint32_t ma_cfg;
	uint32_t ma_csw;
	u_int ma_flags;
#define	ADIV5_MA_FLAGS_TAR_VALID	(1u << 0)
#define	ADIV5_MA_FLAGS_FIXED_SIZE	(1u << 1)
#define	ADIV5_MA_FLAGS_SUPPORTS_PACKED	(1u << 2)
#define	ADIV5_MA_FLAGS_SUPPORTS_BARRIER	(1u << 3)	/* XXX: AXI4 bus only */
#define	ADIV5_MA_FLAGS_NO_DEBUG		(1u << 4)
	target_addr_t ma_tar;
	target_addr_t ma_tar_mask;
	uint32_t ma_data_size;
	uint32_t ma_addrinc;
	uint16_t ma_rom_table_count;
	uint16_t ma_rom_table_idx;
	struct adiv5_rom_table *ma_rom_tables;
};
#define	ADIV5_MEM_AP_ADDR64(ma)	ADIV5_MEM_AP_CFG_LA((ma)->ma_cfg)
#define	ADIV5_MEM_AP_DATA64(ma)	ADIV5_MEM_AP_CFG_LD((ma)->ma_cfg)

/*
 * State for non MEM-AP interfaces.
 */
struct adiv5_other_ap {
	struct adiv5_ap_state oa_as;
};

/*
 * adiv5_probe() will result in a list of MEM-APs which contain valid
 * ROM tables and components.
 */
TAILQ_HEAD(adiv5_ap_qhead, adiv5_ap_state);
static struct adiv5_ap_qhead adiv5_mem_ap_list =
    TAILQ_HEAD_INITIALIZER(adiv5_mem_ap_list);

/*
 * adiv5_probe() will also record unrecognised Access Ports, in case
 * they are required by SoC-specific code.
 */
static struct adiv5_ap_qhead adiv5_other_ap_list =
    TAILQ_HEAD_INITIALIZER(adiv5_other_ap_list);

/*
 * Supported DP interfaces
 */
extern const struct adiv5_dp_interface adiv5_swdp_if;
extern const struct adiv5_dp_interface adiv5_swjdp_if;
extern const struct adiv5_dp_interface adiv5_jtagdp_if;
static const struct adiv5_dp_interface *adiv5_dp_interfaces[] = {
	[ADIV5_TYPE_SWDP]   = &adiv5_swdp_if,
	[ADIV5_TYPE_SWJDP]  = &adiv5_swjdp_if,
	[ADIV5_TYPE_JTAGDP] = &adiv5_jtagdp_if
};
#define	ADIV5_DP_NINTERFACES	(sizeof(adiv5_dp_interfaces) / \
				 sizeof(adiv5_dp_interfaces[0]))

static uint32_t adiv5_dp_read(adiv5_dp_t dp, uint32_t reg);
static int adiv5_dp_write(adiv5_dp_t dp, uint32_t reg, uint32_t data);
static uint32_t adiv5_dp_clear_error(adiv5_dp_t dp);

void
adiv5_dp_get_speed_idles(unsigned int *pspeed, unsigned int *pidles)
{
#ifndef CONFIG_USE_CONFIGDB
	*pspeed = (unsigned int)glob.glob_adiv5_speed;
	*pidles = (unsigned int)glob.glob_adiv5_idles;
#else
	uint8_t speed, idles;

	if (cf_get_uint8(CF_KEY_UINT8_ADIV5_SPEED, &speed) == 0)
		speed = ADIV5_DEFAULT_SPEED;

	if (cf_get_uint8(CF_KEY_UINT8_ADIV5_IDLES, &idles) == 0)
		idles = ADIV5_DEFAULT_IDLE_CYCLES;

	*pspeed = (unsigned int)speed;
	*pidles = (unsigned int)idles;
#endif

	assert(*pspeed < ADIV5_SPEED_2CFG_COUNT);
}

static void
adiv5_dp_abort(adiv5_dp_t dp, uint32_t v)
{

	if (DP_DPIDR_VERSION(dp->dp_idr) == 0)
		v &= DP_ABORT_DAPABORT;

	if (dp->dp_if->dpi_abort != NULL)
		dp->dp_if->dpi_abort(dp, v);
	else
		adiv5_dp_write(dp, DP_REG_ABORT, v);
}

static __inline bool
adiv5_dp_error(adiv5_dp_t dp)
{
	uint32_t cs;

	if (dp->dp_last_ack != DP_ACK_OK || ADIV5_DP_STICKY_ERROR(dp))
		return true;

	if (ADIV5_DP_TYPE(dp) != ADIV5_TYPE_JTAGDP)
		return false;

	/* Need to consult CTRL/STAT on JTAG-DP */
	cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
	if (dp->dp_last_ack != DP_ACK_OK)
		return true;

	if ((cs & (DP_CTRL_STAT_STICKYORUN | DP_CTRL_STAT_STICKYCMP |
	    DP_CTRL_STAT_STICKYERR | DP_CTRL_STAT_WDATAERR)) != 0) {
		ADIV5_DP_STICKY_ERROR_SET(dp);
		return true;
	}

	return false;
}

static void
adiv5_dp_line_reset(adiv5_dp_t dp)
{

	DBMFPRINTF(ADIV5_DP_TARGETSEL, "Performing Line Reset\n");
	if ((dp->dp_select & DP_SELECT_DPBANKSEL_MASK) != 0) {
		DBMFPRINTF(ADIV5_DP_TARGETSEL, "Warning: DPBANKSEL non-zero\n");
		ADIV5_DP_FORCE_SELECT_SET(dp);
	}
	if (dp->dp_if->dpi_line_reset != NULL)
		dp->dp_if->dpi_line_reset(dp);
}

static uint32_t
adiv5_dp_target_select(adiv5_dp_t dp, uint32_t targetsel, uint32_t *ctrl_stat)
{
	uint32_t dpidr, dlpidr, inst;
	unsigned int retries;

	DBMFPRINTF(ADIV5_DP_TARGETSEL, "TARGETSEL 0x%08" PRIx32 "-> 0x%08"
	    PRIx32 "\n", adiv5_dp_current_targetsel, targetsel);

	adiv5_dp_current_targetsel = targetsel;
	inst = DP_TARGETSEL_TINSTANCE(targetsel);

	for (retries = 0; retries < 3; retries++) {
		/* Line Reset must preceded a write to TARGETSEL */
		adiv5_dp_line_reset(dp);

		if (DP_DPIDR_VERSION(dp->dp_idr) >= 2) {
			DBMFPRINTF(ADIV5_DP_TARGETSEL, "Writing TARGETSEL\n");
			/* The target will not ACK the write. */
			dp->dp_cfg |= ADIV5_LL_SWD_IGNORE_ACK;
			(void) (dp->dp_if->dpi_xfer)(dp, DP_REG_TARGETSEL,
			    &targetsel);
			dp->dp_cfg &= ~ADIV5_LL_SWD_IGNORE_ACK;
		}

		/* Must read DPIDR first. */
		DBMFPRINTF(ADIV5_DP_TARGETSEL, "Reading DPIDR\n");
		dpidr = adiv5_dp_read(dp, DP_REG_DPIDR);
		if (adiv5_dp_error(dp)) {
			ADIV5_DP_STICKY_ERROR_CLR(dp);
			DBMFPRINTF(ADIV5_DP_TARGETSEL | ADIV5_DP_ERROR,
			    "sticky error reading DPIDR\n");
		} else {
			DBMFPRINTF(ADIV5_DP_TARGETSEL, "DPIDR 0x%08" PRIx32
			    "\n", dpidr);
		}

		if (ctrl_stat != NULL) {
			DBMFPRINTF(ADIV5_DP_TARGETSEL, "Reading CTRL/STAT\n");
			*ctrl_stat = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
			if (adiv5_dp_error(dp)) {
				ADIV5_DP_STICKY_ERROR_CLR(dp);
				dpidr = 0;
				DBMFPRINTF(ADIV5_DP_ERROR | ADIV5_DP_TARGETSEL,
				    "sticky error reading CTRL/STAT\n");
			} else {
				DBMFPRINTF(ADIV5_DP_TARGETSEL, "CTRL/STAT "
				    "0x%08" PRIx32 "\n", *ctrl_stat);
			}
		}

		if (DP_DPIDR_VERSION(dp->dp_idr) > 0) {
			adiv5_dp_write(dp, DP_REG_ABORT,
			    DP_ABORT_STKCMPCLR | DP_ABORT_STKERRCLR |
			    DP_ABORT_WDERRCLR | DP_ABORT_ORUNERRCLR);
			if (adiv5_dp_error(dp)) {
				ADIV5_DP_STICKY_ERROR_CLR(dp);
				DBMFPRINTF(ADIV5_DP_ERROR | ADIV5_DP_TARGETSEL,
				    "sticky error writing ABORT\n");
				continue;
			}
		}

		if (DP_DPIDR_VERSION(dpidr) < 2) {
			if (dpidr != 0)
				return dpidr;
		} else {
			dlpidr = adiv5_dp_read(dp, DP_REG_DLPIDR);
			DBMFPRINTF(ADIV5_DP_TARGETSEL, "DLPIDR 0x%08" PRIx32
			    "\n", dlpidr);
			if (DP_DLPIDR_PROTVSN(dlpidr) == 1 &&
			    DP_DLPIDR_TINSTANCE(dlpidr) == inst) {
				return dpidr;
			}
		}
	}

	DBMFPRINTF(ADIV5_DP_TARGETSEL, "No DP detected\n");
	return 0;
}

#ifdef ADIV5_VALIDATE_DP_REG
#define	ADIV5_DP_FLAG_INVALID(a)	(0u << ((a) >> 1))
#define	ADIV5_DP_FLAG_RO(a)		(1u << ((a) >> 1))
#define	ADIV5_DP_FLAG_WO(a)		(2u << ((a) >> 1))
#define	ADIV5_DP_FLAG_RW(a)		(3u << ((a) >> 1))
#define	ADIV5_DP_FLAG_GET(m,b,a)	(((m)[b] >> ((a) >> 1)) & 3u)
#define	ADIV5_DP_FLAG_READ_OK(f)	(((f) & 1u) != 0)
#define	ADIV5_DP_FLAG_WRITE_OK(f)	(((f) & 2u) != 0)
#define	ADIV5_DP_MAP_ENTRY(bank, flag)	[(bank)] = (flag)

static const uint8_t adiv5_dp_map_v0[16] = {
	ADIV5_DP_MAP_ENTRY(0x0, ADIV5_DP_FLAG_WO(0x0)		|
				ADIV5_DP_FLAG_RW(0x4)		|
				ADIV5_DP_FLAG_RW(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	/* DPv0 supports bank 0 only. */
};

static const uint8_t adiv5_dp_map_v1[16] = {
	ADIV5_DP_MAP_ENTRY(0x0, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RW(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	ADIV5_DP_MAP_ENTRY(0x1, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RW(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	/* DPv1 supports banks 0 and 1 only. */
};

static const uint8_t adiv5_dp_map_v2[16] = {
	ADIV5_DP_MAP_ENTRY(0x0, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RW(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	ADIV5_DP_MAP_ENTRY(0x1, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RW(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	ADIV5_DP_MAP_ENTRY(0x2, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RO(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	ADIV5_DP_MAP_ENTRY(0x3, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RO(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	ADIV5_DP_MAP_ENTRY(0x4, ADIV5_DP_FLAG_RW(0x0)		|
				ADIV5_DP_FLAG_RO(0x4)		|
				ADIV5_DP_FLAG_WO(0x8)		|
				ADIV5_DP_FLAG_RO(0xc)),
	/* DPv2 supports banks 0 to 4 only. */
};
#endif

static bool
adiv5_dp_select(adiv5_dp_t dp, uint32_t reg)
{
	uint32_t sel, sel_mask, ack;

#ifdef ADIV5_VALIDATE_DP_REG
	if ((reg & DP_REQ_APnDP) == 0) {
		uint32_t addr = reg & (DP_REQ_A2 | DP_REQ_A3);
		uint32_t bank = (reg >> 4) & DP_SELECT_DPBANKSEL_MASK;
		const uint8_t *map;
		uint8_t flag;
		bool is_wr = (reg & DP_REQ_RnW) == 0;

		switch (DP_DPIDR_VERSION(dp->dp_idr)) {
		case 0:
			map = adiv5_dp_map_v0;
			break;

		case 1:
			map = adiv5_dp_map_v1;
			break;

		case 2:
			map = adiv5_dp_map_v2;
			break;

		default:
			assert(0);
			map = NULL;
			break;
		}

		flag = ADIV5_DP_FLAG_GET(map, bank, addr);
		assert(flag != 0);
		if (is_wr)
			assert(ADIV5_DP_FLAG_WRITE_OK(flag));
		else
			assert(ADIV5_DP_FLAG_READ_OK(flag));
	}
#endif

	if (DP_DPIDR_VERSION(dp->dp_idr) >= 2 &&
	    dp->dp_targetsel != adiv5_dp_current_targetsel) {
		if (adiv5_dp_target_select(dp, dp->dp_targetsel, NULL) == 0)
			return false;
	}

	if ((reg & DP_REQ_APnDP) == 0) {
		/* Debug Port. We only care about accesses to register 0x4. */
		if (DP_DPIDR_VERSION(dp->dp_idr) == 0 ||
		    (reg & (DP_REQ_A2 | DP_REQ_A3)) != DP_REQ_A2) {
			return true;
		}

		reg >>= 4;	/* DP Bank into low 4 bits. */
		sel_mask = DP_SELECT_DPBANKSEL_MASK;
	} else {
		/* Access port */
		sel_mask = DP_SELECT_APBANKSEL_MASK | DP_SELECT_APSEL_MASK;
	}

	/* Keep only DPBANKSEL or APBANKSEL/APSEL bits. */
	sel = reg & sel_mask;

	/* Keep the unchanged bits. */
	sel |= dp->dp_select & ~sel_mask;

	/* Do we need to write SELECT? */
	if (sel == dp->dp_select && !ADIV5_DP_FORCE_SELECT(dp))
		return true;

	DBMFPRINTF(ADIV5_DP_REG, "SELECT %08" PRIx32 " -> %08" PRIx32
	    "\n", dp->dp_select, sel);

	/* Write the new value for SELECT. */
	dp->dp_select = sel;
	ack = (dp->dp_if->dpi_xfer)(dp, DP_REG_SELECT, &sel);
	ADIV5_DP_FORCE_SELECT_CLR(dp);

	/* Don't tolerate any errors here */
	if (ack != DP_ACK_OK) {
		DBMFPRINTF(ADIV5_DP_REG | ADIV5_DP_ERROR, "bad ACK %" PRIx32
		    "\n", ack);
	}

	return ack == DP_ACK_OK;
}

static uint32_t
adiv5_dp_read(adiv5_dp_t dp, uint32_t reg)
{
	uint32_t rv, ack;

	DBMFPRINTF(ADIV5_DP_REG, "reg %08" PRIx32 "\n", reg);

	reg |= DP_REQ_RnW;
	if (adiv5_dp_select(dp, reg) == false) {
		DBMFPRINTF(ADIV5_DP_ERROR, "adiv5_dp_select failed\n");
		return 0;
	}

	reg &= DP_REG_MASK;

	if (ADIV5_DP_TYPE(dp) == ADIV5_TYPE_JTAGDP ||
	    (reg & DP_REQ_APnDP) != 0) {
		/* Post the read. */
		DBMFPRINTF(ADIV5_DP_REG, "Posting read\n");
		ack = (dp->dp_if->dpi_xfer)(dp, reg, NULL);

		if (ack != DP_ACK_OK/* && ack != DP_ACK_FAULT*/) {
			DBMFPRINTF(ADIV5_DP_ERROR, "post: bad ACK %08" PRIx32
			    "\n", ack);
			return 0;
		}

		reg = DP_REG_RDBUFF | DP_REQ_RnW;
	}

	DBMFPRINTF(ADIV5_DP_REG, "Reading result\n");
	rv = 0;
	ack = (dp->dp_if->dpi_xfer)(dp, reg, &rv);

	if (ack != DP_ACK_OK && ack != DP_ACK_FAULT) {
		DBMFPRINTF(ADIV5_DP_ERROR, "read: bad ACK %08" PRIx32 "\n",
		    ack);
		return 0;
	}

	DBMFPRINTF(ADIV5_DP_REG, "rv %08" PRIx32 "\n", rv);

	return rv;
}

static int
adiv5_dp_read_posted_first(adiv5_dp_t dp, uint32_t reg)
{
	uint32_t ack;

	DBMFPRINTF(ADIV5_DP_REG, "reg %08" PRIx32 "\n", reg);

	reg |= DP_REQ_RnW;
	if (adiv5_dp_select(dp, reg) == false) {
		DBMFPRINTF(ADIV5_DP_ERROR, "adiv5_dp_select failed\n");
		return -1;
	}
	dp->dp_posted_reg = reg & DP_REG_MASK;

	/* Post the read, ignoring result. */
	DBMFPRINTF(ADIV5_DP_REG, "Posting read\n");
	ack = (dp->dp_if->dpi_xfer)(dp, dp->dp_posted_reg, NULL);

	if (ack != DP_ACK_OK/* && ack != DP_ACK_FAULT*/)
		DBMFPRINTF(ADIV5_DP_ERROR, "post: bad ACK %" PRIx32 "\n", ack);

	return (ack == DP_ACK_OK) ? 0 : -1;
}

static uint32_t
adiv5_dp_read_posted_next(adiv5_dp_t dp, bool last)
{
	uint32_t rv, reg, ack;

	reg = last ? (DP_REG_RDBUFF | DP_REQ_RnW) : dp->dp_posted_reg;

	DBMFPRINTF(ADIV5_DP_REG, "reg %08" PRIx32 "\n", reg);

	/* Post the read, and fetch previous result. */
	DBMFPRINTF(ADIV5_DP_REG, "Posting read\n");
	rv = 0;
	ack = (dp->dp_if->dpi_xfer)(dp, reg, &rv);

	if (ack != DP_ACK_OK/* && ack != DP_ACK_FAULT*/) {
		DBMFPRINTF(ADIV5_DP_ERROR, "bad ACK %08" PRIx32 "\n",
		    ack);
		return 0;
	}

	DBMFPRINTF(ADIV5_DP_REG, "rv %08" PRIx32 "\n", rv);

	return rv;
}

static int
adiv5_dp_write(adiv5_dp_t dp, uint32_t reg, uint32_t data)
{
	uint32_t ack;

	DBMFPRINTF(ADIV5_DP_REG, "reg %08" PRIx32 " -> %08" PRIx32 "\n", reg,
	    data);

	reg &= ~DP_REQ_RnW;
	if (adiv5_dp_select(dp, reg) == false) {
		DBMFPRINTF(ADIV5_DP_ERROR, "adiv5_dp_select failed\n");
		return -1;
	}

	reg &= DP_REG_MASK;
	ack = (dp->dp_if->dpi_xfer)(dp, reg, &data);

	if (ack != DP_ACK_OK && ack != DP_ACK_FAULT)
		DBMFPRINTF(ADIV5_DP_ERROR, "write: bad ACK %" PRIx32 "\n", ack);

	return (ack == DP_ACK_OK) ? 0 : -1;
}

static uint32_t
adiv5_dp_clear_error(adiv5_dp_t dp)
{
	uint32_t cs, ab;

	DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "Sticky error %u. last ack 0x%" PRIx32
	    "\n", ADIV5_DP_STICKY_ERROR(dp), dp->dp_last_ack);

	if (!ADIV5_DP_STICKY_ERROR(dp) && dp->dp_last_ack == DP_ACK_OK)
		return 0;

	/* Must clear the sticky flag here to enable register read/write */
	ADIV5_DP_STICKY_ERROR_CLR(dp);

	if (DP_DPIDR_VERSION(dp->dp_idr) >= 2) {
		/* Perform a line reset. */
		if (adiv5_dp_target_select(dp, dp->dp_targetsel, &cs) == 0) {
			DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "Failed to read "
			    "CTRL/STAT\n");
			cs = DP_CTRL_STAT_STICKYORUN | DP_CTRL_STAT_STICKYCMP |
			    DP_CTRL_STAT_STICKYERR | DP_CTRL_STAT_WDATAERR;
		}
	} else {
		cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
	}

	ab = (dp->dp_last_ack != DP_ACK_OK) ? DP_ABORT_DAPABORT : 0;
	if ((cs & DP_CTRL_STAT_STICKYORUN) != 0)
		ab |= DP_ABORT_ORUNERRCLR;
	if ((cs & DP_CTRL_STAT_STICKYCMP) != 0)
		ab |= DP_ABORT_STKCMPCLR;
	if ((cs & DP_CTRL_STAT_STICKYERR) != 0)
		ab |= DP_ABORT_STKERRCLR;
	if ((cs & DP_CTRL_STAT_WDATAERR) != 0)
		ab |= DP_ABORT_WDERRCLR;

	DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "CS %08" PRIx32 ", ABORT %08" PRIx32
	    "\n", cs, ab);

	if (ADIV5_DP_TYPE(dp) == ADIV5_TYPE_JTAGDP &&
	    DP_DPIDR_VERSION(dp->dp_idr) == 0) {
		/*
		 * JTAG-DPv0 does things a little differently...
		 */
		if ((ab & DP_ABORT_DAPABORT) != 0)
			adiv5_dp_abort(dp, ab);

		if (adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs) < 0) {
			DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "Failed to write "
			    "CTRL/STAT\n");
		}
	} else {
		adiv5_dp_abort(dp, ab);
	}

	DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "post CS %08" PRIx32 "\n",
	    adiv5_dp_read(dp, DP_REG_CTRL_STAT));

	if (adiv5_dp_error(dp)) {
		ADIV5_DP_STICKY_ERROR_CLR(dp);
		DBMFPRINTF(ADIV5_DP_CLEAR_ERROR, "sticky error still "
		    "flagged.\n");
	}

	return ab;
}

static void
adiv5_dp_reference(adiv5_dp_t dp)
{

	dp->dp_refcnt++;

	DBMFPRINTF(ADIV5_DP_REF, "%p ref_cnt %u\n", (void *)dp, dp->dp_refcnt);
}

static void
adiv5_dp_unreference(adiv5_dp_t dp)
{

	assert(dp->dp_refcnt != 0);

	dp->dp_refcnt--;

	DBMFPRINTF(ADIV5_DP_REF, "%p ref_cnt %u\n", (void *)dp, dp->dp_refcnt);

	if (dp->dp_refcnt == 0) {
		DBMFPRINTF(ADIV5_DP_REF, "DP is now free\n");

		TAILQ_REMOVE(&adiv5_dp_list, dp, dp_qent);
		if (adiv5_dp_active && TAILQ_EMPTY(&adiv5_dp_list)) {
			adiv5_dp_active = 0;
			(dp->dp_if->dpi_done)(dp);
		}

		zone_free(dp);
	}
}

static uint32_t
adiv5_ap_read(void *arg, uint32_t reg)
{
	const struct adiv5_ap_state *as = arg;

	return adiv5_dp_read(as->as_dp, as->as_ap_sel | reg);
}

static int
adiv5_ap_write(void *arg, uint32_t reg, uint32_t data)
{
	const struct adiv5_ap_state *as = arg;

	return adiv5_dp_write(as->as_dp, as->as_ap_sel | reg, data);
}

static int
adiv5_mem_ap_read_posted_first(adiv5_mem_ap_t ma, uint32_t reg)
{

	return adiv5_dp_read_posted_first(ma->ma_as.as_dp,
	    ma->ma_as.as_ap_sel | reg);
}

static uint32_t
adiv5_mem_ap_read_posted_next(adiv5_mem_ap_t ma, bool last)
{

	return adiv5_dp_read_posted_next(ma->ma_as.as_dp, last);
}

/*
 * Configure the access port for a memory read/write.
 *  addr	The system memory address
 *  size	The access size (unshifted ADIV5_MEM_AP_CSW_SIZE_BYTE etc)
 *  addrinc	The auto-incr type (unshifted ADIV5_MEM_AP_CSW_ADDRINC_OFF etc)
 */
static int
adiv5_mem_ap_setup_address(adiv5_mem_ap_t ma, target_addr_t addr, uint32_t size,
    uint32_t addrinc)
{
	adiv5_dp_t dp = ma->ma_as.as_dp;
	uint32_t err;

	assert(!((ma->ma_flags & ADIV5_MA_FLAGS_FIXED_SIZE) &&
	    size != ADIV5_MEM_AP_CSW_SIZE_WORD));
	assert(!((ma->ma_flags & ADIV5_MA_FLAGS_SUPPORTS_PACKED) == 0 &&
	    addrinc == ADIV5_MEM_AP_CSW_ADDRINC_PACKED));

	size <<= ADIV5_MEM_AP_CSW_SIZE_SHIFT;
	addrinc <<= ADIV5_MEM_AP_CSW_ADDRINC_SHIFT;

	if (size != ma->ma_data_size || addrinc != ma->ma_addrinc) {
		uint32_t csw;
		DBMFPRINTF(ADIV5_MEM_AP_REG, "CSW -> %08" PRIx32 "\n",
		    ma->ma_csw | size | addrinc);

		csw = ma->ma_csw | size | addrinc;
		if (adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_CSW, csw) < 0 ||
		    adiv5_dp_error(dp)) {
			ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
			ma->ma_data_size = ~0u;
			ma->ma_addrinc = ~0u;
			err = adiv5_dp_clear_error(ma->ma_as.as_dp);
			ma->ma_as.as_error_log |= err;
			DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
			    "failed to write 0x%08" PRIx32 " to CSW. err "
			    "%08" PRIx32 "\n", csw, err);
			return -1;
		}
		ma->ma_data_size = size;
		ma->ma_addrinc = addrinc;
	} else
	if ((ma->ma_flags & ADIV5_MA_FLAGS_TAR_VALID) && addr == ma->ma_tar) {
		DBMFPRINTF(ADIV5_MEM_AP_REG, "TAR already set to %08" PRIxTADDR
		    "\n", addr);
		return 0;
	}

	if (addrinc == ADIV5_MEM_AP_CSW_ADDRINC_OFF) {
		ma->ma_tar = addr;
		ma->ma_flags |= ADIV5_MA_FLAGS_TAR_VALID;
	} else {
		ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
	}

	DBMFPRINTF(ADIV5_MEM_AP_REG, "TAR -> %08" PRIxTADDR "\n", addr);

#ifdef TARGET_64BIT_SUPPORT
	if (ADIV5_MEM_AP_ADDR64(ma)) {
		if (adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_TAR_MSW,
		    (uint32_t) addr) < 0 || adiv5_dp_error(dp)) {
			ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
			ma->ma_data_size = ~0u;
			ma->ma_addrinc = ~0u;
			err = adiv5_dp_clear_error(ma->ma_as.as_dp);
			ma->ma_as.as_error_log |= err;
			DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
			    "failed to write TAR_MSW. err %" PRIx32 "\n", err);
			return -1;
		}
	}
#endif /* TARGET_64BIT_SUPPORT */

	if (adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_TAR,
	    (uint32_t) addr) < 0 || adiv5_dp_error(dp)) {
		ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
		ma->ma_data_size = ~0u;
		ma->ma_addrinc = ~0u;
		err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;
		DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
		    "failed to write TAR. err %" PRIx32 "\n", err);
		return -1;
	}

	return 0;
}

static uint16_t
adiv5_mem_ap_read16(adiv5_mem_ap_t ma, target_addr_t addr)
{
	uint32_t drw;
	uint16_t rv;

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR "\n", addr);

	assert((addr & 1) == 0);

	if (adiv5_mem_ap_setup_address(ma, addr,ADIV5_MEM_AP_CSW_SIZE_HALF_WORD,
	    ADIV5_MEM_AP_CSW_ADDRINC_OFF) < 0) {
		return 0;
	}

	drw = adiv5_ap_read(&ma->ma_as, ADIV5_MEM_AP_REG_DRW);
	rv = (uint16_t)(drw >> ((addr & 0x2u) * 8));

	DBMFPRINTF(ADIV5_MEM_AP_REG, "result %04" PRIx16 "\n", rv);

	return rv;
}

static int
adiv5_mem_ap_write16(adiv5_mem_ap_t ma, target_addr_t addr, uint16_t data)
{
	uint32_t drw;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR ", data %04" PRIx16
	    "\n", addr, data);

	assert((addr & 1) == 0);

	if (adiv5_mem_ap_setup_address(ma, addr,ADIV5_MEM_AP_CSW_SIZE_HALF_WORD,
	    ADIV5_MEM_AP_CSW_ADDRINC_OFF) < 0) {
		return -1;
	}

	drw = ((uint32_t)data) << ((addr & 0x2u) * 8);
	rv = adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_DRW, drw);

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR " DONE\n", addr);

	if (rv < 0 || adiv5_dp_error(ma->ma_as.as_dp))
		return -1;

	return rv;
}

static uint32_t
adiv5_mem_ap_read32(adiv5_mem_ap_t ma, target_addr_t addr)
{
	uint32_t rv;

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR "\n", addr);

	assert((addr & 3) == 0);

	if (adiv5_mem_ap_setup_address(ma, addr, ADIV5_MEM_AP_CSW_SIZE_WORD,
	    ADIV5_MEM_AP_CSW_ADDRINC_OFF) < 0) {
		DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
		    "Failed to setup address.\n");
		return 0;
	}

	rv = adiv5_ap_read(&ma->ma_as, ADIV5_MEM_AP_REG_DRW);

	DBMFPRINTF(ADIV5_MEM_AP_REG, "result %08" PRIx32 "\n", rv);

	return rv;
}

static int
adiv5_mem_ap_write32(adiv5_mem_ap_t ma, target_addr_t addr, uint32_t data)
{
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR ", data %08" PRIx32
	    "\n", addr, data);

	assert((addr & 3) == 0);

	if (adiv5_mem_ap_setup_address(ma, addr, ADIV5_MEM_AP_CSW_SIZE_WORD,
	    ADIV5_MEM_AP_CSW_ADDRINC_OFF) < 0) {
		return -1;
	}

	rv = adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_DRW, data);

	DBMFPRINTF(ADIV5_MEM_AP_REG, "addr %08" PRIxTADDR " DONE\n", addr);

	return rv;
}

static uint32_t
adiv5_mem_ap_read_idr(adiv5_mem_ap_t ma, target_addr_t addr, uint8_t *pv)
{
	uint32_t err;

	*pv = adiv5_mem_ap_read32(ma, addr) & 0xffu;

	if (!adiv5_dp_error(ma->ma_as.as_dp))
		return 0;

	err = adiv5_dp_clear_error(ma->ma_as.as_dp);

	DBMFPRINTF(ADIV5_MEM_AP_EXPLORE, "Error 0x%" PRIx32 " while reading "
	    "[PC]IDR at 0x%08" PRIxTADDR "\n", err, addr);

	return err;
}

static bool
adiv5_probe_class9_rom_table(adiv5_mem_ap_t ma, target_addr_t base)
{
	uint32_t devarch;

	devarch = adiv5_mem_ap_read32(ma, base + ADIV5_CLASS9_DEVARCH);
	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		(void) err;
		DBMFPRINTF(ADIV5_MEM_AP_EXPLORE, "Error 0x%" PRIx32 " while "
		    "reading DEVARCH\n", err);
		return false;
	}

	if (ma->ma_rom_tables == NULL) {
		DBMFPRINTF(ADIV5_MEM_AP_EXPLORE, "DEVARCH 0x%08" PRIx32 "\n",
		    devarch);
	}

	if (ADIV5_DEVARCH_ARCHITECT(devarch) == ADIV5_DEVARCH_ARCHITECT_ARM &&
	    ADIV5_DEVARCH_PRESENT(devarch) == 1 &&
	    ADIV5_DEVARCH_ARCHID(devarch) == ADIV5_DEVARCH_ARCHID_ROM_TABLE) {
		DBMFPRINTF(ADIV5_MEM_AP_EXPLORE, "Valid Class 0x9 ROM table\n");
		return true;
	}

	return false;
}

static void
adiv5_mem_ap_explore_rom_table(adiv5_mem_ap_t ma, target_addr_t base, int level)
{
	union {
		uint64_t pidr;
		uint8_t b[8];
	} pu;
	union {
		uint32_t cidr;
		uint8_t b[4];
	} cu;
	uint32_t err, i, max_entries;

	if (ma->ma_rom_tables == NULL) {
		DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "%s%s(%d): base 0x%08" PRIxTADDR
		    "\n", (level == 0) ? "\nRoot: " : "\t", __func__, level,
		    base);
	}

	/*
	 * Read Product and Component IDs.
	 * Maven is running on a little-endian CPU, so this hack works.
	 *
	 * Note that some non-ROM table components may return bogus
	 * values here because they've not yet been enabled in hardware.
	 * For example ARMv7m's DWT doesn't respond properly until the
	 * TRCENA bit it set in the device's SCS->DEMCR register. We
	 * can't do that here because, well, that would just be an
	 * egregious hack.
	 */
	if (adiv5_mem_ap_read_idr(ma, base + CPID_REG_CIDR0, &cu.b[0]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_CIDR1, &cu.b[1]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_CIDR2, &cu.b[2]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_CIDR3, &cu.b[3]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_PIDR0, &pu.b[0]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_PIDR1, &pu.b[1]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_PIDR2, &pu.b[2]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_PIDR3, &pu.b[3]) != 0 ||
	    adiv5_mem_ap_read_idr(ma, base + CPID_REG_PIDR4, &pu.b[4]) != 0) {
		DBMPRINTF(ADIV5_MEM_AP_EXPLORE | ADIV5_DP_ERROR,
		    "\tDP err after PIDR/CIDR read.\n");
		tmon_log("\tDP error after PIDR/CIDR read. Aborting "
		    "enumeration.\n");
		return;
	}
	pu.b[5] = pu.b[6] = pu.b[7] = 0;

	if (ma->ma_rom_tables == NULL) {
		DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\tPIDR 0x%" PRIx32 "%08" PRIx32
		    ", CIDR 0x%08" PRIx32 "\n", (uint32_t)(pu.pidr >> 32),
		    (uint32_t)(pu.pidr & 0xffffffffu), cu.cidr);
	}

	/*
	 * We don't support legacy components, so bail now if the
	 * JEDEC bit is clear.
	 */
	if (ADIV5_PIDR_JEDEC(pu.pidr) == 0) {
		if (ma->ma_rom_tables == NULL) {
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\tIgnoring legacy "
			    "component (0x%" PRIx32 "%08" PRIx32 ")\n",
			    (uint32_t)(pu.pidr >> 32), (uint32_t)pu.pidr);
		}
		return;
	}

	if (!ADIV5_CIDR_VALID(cu.cidr)) {
		if (ma->ma_rom_tables == NULL) {
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\tInvalid CIDR "
			    "preamble. Skipping.\n");
		}
		return;
	}

	if (ma->ma_rom_tables != NULL) {
		DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "Type %u: %s Component.\n",
		    ADIV5_CIDR_CLASS(cu.cidr),
		    (class_strings[ADIV5_CIDR_CLASS(cu.cidr)] == NULL) ?
		    "Reserved" : class_strings[ADIV5_CIDR_CLASS(cu.cidr)]);

		DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\tDesigner %03x, Part %03x, "
		    "Revision %u.%u, CMOD %u, Size %u\n",
		    ADIV5_PIDR_DES(pu.pidr), ADIV5_PIDR_PART(pu.pidr),
		    ADIV5_PIDR_REVISION(pu.pidr), ADIV5_PIDR_REVAND(pu.pidr),
		    ADIV5_PIDR_CMOD(pu.pidr), ADIV5_PIDR_SIZE(pu.pidr));
	}

	/*
	 * We're only interested in finding ROM Tables.
	 */
	switch (ADIV5_CIDR_CLASS(cu.cidr)) {
	case ADIV5_CLASS_ROM_TABLE:
		/* Regular Class 0x1 ROM table. */
		max_entries = ADIV5_ROMTBL1_MAX_ENTRIES;
		break;

	case ADIV5_CLASS_CORESIGHT:
		/* This might be a Class 0x9 ROM table... */
		if (adiv5_probe_class9_rom_table(ma, base)) {
			max_entries = ADIV5_ROMTBL9_MAX_ENTRIES;
			break;
		}

		/*FALLTHROUGH*/
	default:
		if (ma->ma_rom_tables == NULL) {
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\tNot a ROM table. "
			    "Skipping.\n");
		}
		return;
	}

	if (ma->ma_rom_tables != NULL) {
		if (ma->ma_rom_table_idx < ma->ma_rom_table_count) {
			ma->ma_rom_tables[ma->ma_rom_table_idx].rt_pidr =
			    pu.pidr;
			ma->ma_rom_tables[ma->ma_rom_table_idx].rt_address =
			    base;
			ma->ma_rom_table_idx += 1;
		} else {
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "explore: found more "
			    "ROM Tables than expected (was %" PRIu16 ")!\n",
			    ma->ma_rom_table_count);
		}
	} else {
		ma->ma_rom_table_count++;
	}

	/*
	 * Explore the ROM table
	 */
	for (i = 0; i < max_entries; i++) {
		uint32_t romentry;
		target_addr_t rte_addr;

		romentry = adiv5_mem_ap_read32(ma, base + (i * 4));
		if (adiv5_dp_error(ma->ma_as.as_dp)) {
			err = adiv5_dp_clear_error(ma->ma_as.as_dp);
			(void) err;
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE | ADIV5_DP_ERROR,
			    "Sticky error while reading ROMENTRY idx %" PRIu32
			    ": %08" PRIx32 "\n", i, err);
		}

		if (!ADIV5_ROMENTRY_VALID(romentry)) {
			/* No more entries */
			break;
		}

		if (!ADIV5_ROMENTRY_PRESENT(romentry)) {
			/* Valid, but not enabled */
			continue;
		}

		if (!ADIV5_ROMENTRY_FORMAT_32(romentry)) {
			/* Invalid format (not 32-bits) */
			continue;
		}

		rte_addr = ADIV5_RTE2ADDR(base, romentry);

		if (ma->ma_rom_tables == NULL) {
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\nROM Entry %d.%" PRIu32
			    ": ROMENTRY %08" PRIx32 " -> addr %08" PRIxTADDR,
			    level, i, romentry, rte_addr);

			if (ADIV5_ROMENTRY_POWERID_VALID(romentry)) {
				DBMPRINTF(ADIV5_MEM_AP_EXPLORE, ", PowerID %02x",
				    ADIV5_ROMENTRY_POWERID(romentry));
			}
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE, "\n");
		}

		/* Explore recursively */
		adiv5_mem_ap_explore_rom_table(ma, rte_addr, level + 1);

		if (adiv5_dp_error(ma->ma_as.as_dp)) {
			err = adiv5_dp_clear_error(ma->ma_as.as_dp);
			(void) err;
			DBMPRINTF(ADIV5_MEM_AP_EXPLORE | ADIV5_DP_ERROR, "Sticky"
			    " error after exploring component at ROMENTRY idx "
			    "%" PRIu32 ": %08" PRIx32 "\n", i, err);
		}
	}
}

static void
adiv5_mem_ap_detach(adiv5_mem_ap_t ma)
{

	DBMFPRINTF(ADIV5_MEM_AP_REF, "%p detaching\n", (void *)ma);

	if (ma->ma_rom_tables != NULL)
		zone_free(ma->ma_rom_tables);

	adiv5_dp_unreference(ma->ma_as.as_dp);
	zone_free(ma);
}

static void
adiv5_other_ap_detach(adiv5_other_ap_t oa)
{
	struct adiv5_ap_state *as = &oa->oa_as;

	DBMFPRINTF(ADIV5_OTHER_AP_REF, "%p detaching\n", (void *)as);

	adiv5_dp_unreference(as->as_dp);
	zone_free(as);
}

static void
adiv5_mem_ap_attach(adiv5_dp_t dp, uint32_t ap_sel, uint32_t apidr,
    int dpversion)
{
	uint32_t cfg, csw, base32;
	target_addr_t base;
	uint32_t barrier;
	int fixed_size, packed;
	adiv5_mem_ap_t ma;
	u_int flags;

	/*
	 * Fetch some registers so that we can determine what we're
	 * playing with...
	 */
	base32 = adiv5_dp_read(dp, ap_sel | ADIV5_MEM_AP_REG_BASE);

	DBMFPRINTF(ADIV5_MEM_AP_ATTACH, "ap_sel %08" PRIx32
	    ": ADIV5_MEM_AP_REG_BASE 0x%08" PRIx32 "\n", ap_sel, base32);

	/*
	 * We're done if the base address indicates there's nothing here.
	 */
	if (ADIV5_MEM_AP_BASE_LEGACY_PRESENT(base32) == 0) {
		DBMFPRINTF(ADIV5_MEM_AP_ATTACH, "Nothing to see here\n");
		tmon_log("adiv5_mem_ap_attach(ap_sel %08" PRIx32 "): empty "
		    "legacy MEM-AP. Abort.\n", ap_sel);
		return;
	}

	flags = 0;
	if (ADIV5_MEM_AP_BASE_FORMAT_ADIv5(base32)) {
		/*
		 * We have an ADIv5 style MEM-AP.
		 * Does it have debug capability?
		 */
		if (ADIV5_MEM_AP_BASE_DEBUG_PRESENT(base32) == 0) {
			DBMFPRINTF(ADIV5_MEM_AP_ATTACH, "no debug entries\n");
			flags = ADIV5_MA_FLAGS_NO_DEBUG;
		}
	}

	/*
	 * ADIv5 MEM-APs may have a 64-bit base address register
	 */
	base = (target_addr_t) base32;

	cfg = adiv5_dp_read(dp, ap_sel | ADIV5_MEM_AP_REG_CFG);
	csw = adiv5_dp_read(dp, ap_sel | ADIV5_MEM_AP_REG_CSW);

	if (ADIV5_MEM_AP_CFG_LA(cfg) && ADIV5_MEM_AP_BASE_FORMAT_ADIv5(base32)){
#ifndef TARGET_64BIT_SUPPORT
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tNo support for 64-bit "
		    "Target\n");
		tmon_log("\tNo support for 64-bit Target\n");
		return;
#else
		uint32_t base_msb;

		base_msb = adiv5_dp_read(dp,
		    ap_sel | ADIV5_MEM_AP_REG_BASE_MSW);

		base |= ((target_addr_t) base_msb) << 32;
#endif /* TARGET_64BIT_SUPPORT */
	}

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "MEM-AP(ap_sel %08" PRIx32 ") "
	    "base %08" PRIxTADDR ", cfg %08" PRIx32 ", csw %08" PRIx32 "\n",
	    ap_sel, base, cfg, csw);
	tmon_log("MEM-AP(ap_sel %08" PRIx32 ") base %08" PRIxTADDR ", "
	    "cfg %08" PRIx32 ", csw %08" PRIx32 "\n", ap_sel, base, cfg, csw);

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tDebug entry %spresent\n",
	    (flags & ADIV5_MA_FLAGS_NO_DEBUG) ? "not " : "");
	tmon_log("\tDebug entry %spresent\n",
	    (flags & ADIV5_MA_FLAGS_NO_DEBUG) ? "not " : "");

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tAddress bits: %u, Data bits: %u",
	    ADIV5_MEM_AP_CFG_LA(cfg) ? 64u : 32u,
	    ADIV5_MEM_AP_CFG_LD(cfg) ? 64u : 32u);
	tmon_log("\tAddress bits: %u, Data bits: %u",
	    ADIV5_MEM_AP_CFG_LA(cfg) ? 64u : 32u,
	    ADIV5_MEM_AP_CFG_LD(cfg) ? 64u : 32u);

	if (dpversion < 2) {
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, ", %s-endian",
		    ADIV5_MEM_AP_CFG_BE(cfg) ? "Big" : "Little");
		tmon_log(", %s-endian", ADIV5_MEM_AP_CFG_BE(cfg) ? "Big" :
		    "Little");
	}

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\n");
	tmon_log("\n");

	base &= ~ADIV5_MEM_AP_BASE_BASEADDR_CLR_BITS;

	/* Let's see if CSW.SIZE is read-only */
	csw &= ~(ADIV5_MEM_AP_CSW_SIZE_MASK | ADIV5_MEM_AP_CSW_ADDRINC_MASK);
	csw |= (ADIV5_MEM_AP_CSW_SIZE_BYTE << ADIV5_MEM_AP_CSW_SIZE_SHIFT);
	csw |= (ADIV5_MEM_AP_CSW_ADDRINC_PACKED <<
	    ADIV5_MEM_AP_CSW_ADDRINC_SHIFT);
	adiv5_dp_write(dp, ap_sel | ADIV5_MEM_AP_REG_CSW, csw);

	/* Read back what we wrote to see what bits stuck */
	csw = adiv5_dp_read(dp, ap_sel | ADIV5_MEM_AP_REG_CSW);
	fixed_size = ADIV5_MEM_AP_CSW_SIZE(csw) == ADIV5_MEM_AP_CSW_SIZE_WORD;
	packed = csw & (ADIV5_MEM_AP_CSW_ADDRINC_PACKED <<
	    ADIV5_MEM_AP_CSW_ADDRINC_SHIFT);

	/*
	 * Determine if the MEM-AP has Barrier support
	 */
	if (AP_IDR_TYPE(apidr) == AP_IDR_TYPE_AMBA_AXI3_AXI4 &&
	    (csw & ADIV5_MEM_AP_CSW_MODE_MASK) == ADIV5_MEM_AP_CSW_MODE_BASIC) {
		/* Try to enable enable Barrier mode. */
		csw |= (ADIV5_MEM_AP_CSW_MODE_BARRIER <<
		    ADIV5_MEM_AP_CSW_MODE_SHIFT);
		adiv5_dp_write(dp, ap_sel | ADIV5_MEM_AP_REG_CSW, csw);
		csw = adiv5_dp_read(dp, ap_sel | ADIV5_MEM_AP_REG_CSW);
	}
	barrier = (csw & ADIV5_MEM_AP_CSW_MODE_MASK) >>
	    ADIV5_MEM_AP_CSW_MODE_SHIFT;

	/* Set CSW back to a sensible starting value */
	csw &= ~(ADIV5_MEM_AP_CSW_SIZE_MASK | ADIV5_MEM_AP_CSW_ADDRINC_MASK);
	csw |= (ADIV5_MEM_AP_CSW_SIZE_WORD << ADIV5_MEM_AP_CSW_SIZE_SHIFT);
	csw |= (ADIV5_MEM_AP_CSW_ADDRINC_OFF << ADIV5_MEM_AP_CSW_ADDRINC_SHIFT);
	adiv5_dp_write(dp, ap_sel | ADIV5_MEM_AP_REG_CSW, csw);

	if (adiv5_dp_error(dp)) {
		uint32_t err = adiv5_dp_clear_error(dp);
		(void) err;
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, "Caught error while checking "
		    "CSW: err %" PRIx32 "\n", err);
		tmon_log("Caught error while checking CSW: err %" PRIx32 "\n", err);
		return;
	}

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tSupports %s access size\n",
	    fixed_size ?  "fixed" : "variable");
	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\t%s packed auto-increment\n", packed ?
	    "Supports" : "Does not support");
	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\t%s barrier operation extension\n",
	    barrier ? "Supports" : "Does not support");

	tmon_log("\tSupports %s access size\n", fixed_size ?  "fixed" :
	    "variable");
	tmon_log("\t%s packed auto-increment\n", packed ? "Supports" :
	    "Does not support");
	tmon_log("\t%s barrier operation extension\n", barrier ? "Supports" :
	    "Does not support");

	if (ADIV5_MEM_AP_CSW_TRINPROG(csw)) {
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tWarning: Transfer In Progress "
		    "is set!\n");
		tmon_log("\tWarning: Transfer In Progress is set!\n");
	}

	/*
	 * Finally, is the MEM-AP's DEVICEEN signal asserted?
	 * I suspect this will be de-asserted if memory protection is
	 * enabled.
	 */
	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tDEVICEEN: %s.\n",
	    ADIV5_MEM_AP_CSW_DEVICE_EN(csw) ? "Enabled" : "Disbled");
	tmon_log("\tDEVICEEN: %s.\n", ADIV5_MEM_AP_CSW_DEVICE_EN(csw) ?
	    "Enabled" : "Disabled; device may be locked");

	if (ADIV5_MEM_AP_CSW_DEVICE_EN(csw) == false) {
		/* Debug is disabled. */
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, "MEM-AP enumeration aborted; "
		    "device disabled.\n");
		tmon_log("MEM-AP enumeration aborted; device disabled.\n");
		return;
	}

	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "\tUsing CSW value 0x%08" PRIx32 "\n",
	    csw);
	tmon_log("\tUsing CSW value 0x%08" PRIx32 "\n", csw);

	/*
	 * Let's run with this.
	 */
	if ((ma = zone_calloc(1, sizeof(*ma))) == NULL) {
		DBMPRINTF(ADIV5_MEM_AP_ATTACH, "Out of memory\n");
		return;
	}

	adiv5_dp_reference(dp);
	ma->ma_as.as_dp = dp;
	ma->ma_as.as_ap_sel = ap_sel;
	ma->ma_as.as_idr = apidr;
	ma->ma_as.as_error_log = 0;
	ma->ma_cfg = cfg;
	ma->ma_tar = (target_addr_t)(-1);
	ma->ma_tar_mask = ~0x3ffu;	/* Default 10-bits */
	ma->ma_csw = csw & ADIV5_MEM_AP_CSW_MASK;
	ma->ma_data_size = ~0u;
	ma->ma_addrinc = ~0u;
	ma->ma_flags = flags;
	if (fixed_size)
		ma->ma_flags |= ADIV5_MA_FLAGS_FIXED_SIZE;
	if (packed)
		ma->ma_flags |= ADIV5_MA_FLAGS_SUPPORTS_PACKED;
	if (barrier)
		ma->ma_flags |= ADIV5_MA_FLAGS_SUPPORTS_BARRIER;
	ma->ma_rom_table_count = 0;
	ma->ma_rom_table_idx = 0;
	ma->ma_rom_tables = NULL;

	/*
	 * First time through, just enumerate the components
	 */
	adiv5_mem_ap_explore_rom_table(ma, base, 0);

	DBMPRINTF(ADIV5_MEM_AP_ATTACH | ADIV5_MEM_AP_EXPLORE,
	    "MEM-AP enumeration found %" PRIu16 " components\n",
	    ma->ma_rom_table_count);
	tmon_log("MEM-AP enumeration found %" PRIu16 " components\n",
	    ma->ma_rom_table_count);

	if (ma->ma_rom_table_count != 0) {
		ma->ma_rom_tables = zone_calloc(ma->ma_rom_table_count,
		    sizeof(*ma->ma_rom_tables));

		if (ma->ma_rom_tables == NULL) {
			DBMPRINTF(ADIV5_MEM_AP_ATTACH, "No memory for "
			    "%" PRIu16 " components!\n",
			    ma->ma_rom_table_count);
			adiv5_mem_ap_detach(ma);
			return;
		} else {
			/*
			 * Now actually fill out the details.
			 */
			adiv5_mem_ap_explore_rom_table(ma, base, 0);

			DBMPRINTF(ADIV5_MEM_AP_ATTACH, "MEM-AP log found"
			    " %" PRIu16 " components\n", ma->ma_rom_table_idx);
		}
	}

	if (ma->ma_rom_tables == NULL) {
		DBMFPRINTF(ADIV5_MEM_AP_ATTACH, "no components found\n");
		adiv5_mem_ap_detach(ma);
		return;
	}

	/*
	 * Record this MEM-AP
	 */
	DBMPRINTF(ADIV5_MEM_AP_ATTACH, "MEM-AP(ap_sel %08" PRIx32
	    ") registering\n", ap_sel);

	DBMPRINTF(ADIV5_MEM_AP_COMP_LIST, "MEM-AP ROM Tables:\n");
	tmon_log("MEM-AP ROM Tables on ap_sel %08" PRIx32 ":\n", ap_sel);

	for (struct adiv5_rom_table *rt = ma->ma_rom_tables;
	    rt < &ma->ma_rom_tables[ma->ma_rom_table_count]; rt++) {
		DBMPRINTF(ADIV5_MEM_AP_COMP_LIST, "\tPIDR: %" PRIx32
		    "%08" PRIx32 " at %08" PRIxTADDR "\n",
		    (uint32_t)(rt->rt_pidr >> 32), (uint32_t)rt->rt_pidr,
		    rt->rt_address);
		tmon_log("\tPIDR: %" PRIx32 "%08" PRIx32 " at "
		    "%08" PRIxTADDR "\n", (uint32_t)(rt->rt_pidr >> 32),
		    (uint32_t)rt->rt_pidr, rt->rt_address);
	}

	TAILQ_INSERT_TAIL(&adiv5_mem_ap_list, &ma->ma_as, as_qent);
}

static void
adiv5_other_ap_attach(adiv5_dp_t dp, uint32_t ap_sel, uint32_t apidr)
{
	adiv5_other_ap_t oa;

	if ((oa = zone_calloc(1, sizeof(*oa))) == NULL) {
		DBMPRINTF(ADIV5_OTHER_AP_ATTACH, "Out of memory\n");
		return;
	}

	adiv5_dp_reference(dp);
	oa->oa_as.as_dp = dp;
	oa->oa_as.as_ap_sel = ap_sel;
	oa->oa_as.as_idr = apidr;
	oa->oa_as.as_error_log = 0;

	/* Record this AP */
	DBMPRINTF(ADIV5_OTHER_AP_ATTACH, "Other-AP(ap_sel %08" PRIx32
	    ", idr %08" PRIx32 ") registering\n", ap_sel, apidr);
	tmon_log("Other-AP(ap_sel %08" PRIx32 ", idr %08" PRIx32
	    ") registering\n", ap_sel, apidr);

	TAILQ_INSERT_TAIL(&adiv5_other_ap_list, &oa->oa_as, as_qent);
}

static void
adiv5_other_ap_free(void *arg)
{

	adiv5_other_ap_detach((adiv5_other_ap_t) arg);
}

static uint32_t
adiv5_mem_ap_align(target_addr_t addr, uint32_t len)
{
_Static_assert(ADIV5_MEM_AP_CSW_SIZE_BYTE == 0,
    "ADIV5_MEM_AP_CSW_SIZE_BYTE must be zero");
_Static_assert(ADIV5_MEM_AP_CSW_SIZE_HALF_WORD == 1,
    "ADIV5_MEM_AP_CSW_SIZE_BYTE must be zero");
_Static_assert(ADIV5_MEM_AP_CSW_SIZE_WORD == 2,
    "ADIV5_MEM_AP_CSW_SIZE_BYTE must be zero");

	static const uint8_t align_bytes[4] = {
		ADIV5_MEM_AP_CSW_SIZE_WORD, ADIV5_MEM_AP_CSW_SIZE_BYTE,
		ADIV5_MEM_AP_CSW_SIZE_HALF_WORD, ADIV5_MEM_AP_CSW_SIZE_BYTE};
	uint32_t aa, la;

	aa = (uint32_t)align_bytes[addr & 3u];
	la = (uint32_t)align_bytes[len & 3u];

	/* Return the minimum alignment. */
	if (la < aa)
		aa = la;

	return aa;
}

static __always_inline void
adiv5_read8(void *buff, target_addr_t addr, uint32_t data)
{
	uint8_t *b = buff;

	*b = (uint8_t)(data >> ((addr & 0x3u) * 8));
}

static __always_inline uint32_t
adiv5_write8(target_addr_t addr, const void *buff)
{
	const uint8_t *b = buff;

	return ((uint32_t)*b) << ((addr & 0x3u) * 8);
}

static __always_inline void
adiv5_read16(void *buff, target_addr_t addr, uint32_t data)
{
	uint16_t *b = buff;

	*b = (uint16_t)(data >> ((addr & 0x2u) * 8));
}

static __always_inline void
adiv5_unpack_read(void *buff, target_addr_t addr, uint32_t data, uint32_t size)
{

	switch (size) {
	case ADIV5_MEM_AP_CSW_SIZE_BYTE:
		adiv5_read8(buff, addr, data);
		break;
	case ADIV5_MEM_AP_CSW_SIZE_HALF_WORD:
		adiv5_read16(buff, addr, data);
		break;
	default:
		*((uint32_t *)buff) = data;
		break;
	}
}

static __always_inline uint32_t
adiv5_write16(target_addr_t addr, const void *buff)
{
	const uint16_t *b = buff;

	return ((uint32_t)*b) << ((addr & 0x2u) * 8);
}

static __always_inline uint32_t
adiv5_pack_write(target_addr_t addr, const void *buff, uint32_t size)
{
	uint32_t data;

	switch (size) {
	case ADIV5_MEM_AP_CSW_SIZE_BYTE:
		data = adiv5_write8(addr, buff);
		break;
	case ADIV5_MEM_AP_CSW_SIZE_HALF_WORD:
		data = adiv5_write16(addr, buff);
		break;
	default:
		data = *((const uint32_t *)buff);
		break;
	}

	return data;
}

#define	ADDR_INC(a)						\
	do {							\
		buff = (void *)((uintptr_t)buff + (1u << (a)));	\
		taddr += 1u << (a);				\
	} while (0)

static int
adiv5_mem_ap_sysmem_read_int(adiv5_mem_ap_t ma, target_addr_t taddr,
    uint32_t tlen, void *buff, bool do_posted)
{
	target_addr_t taddr_save;
	uint32_t size, data;

	if (tlen == 0)
		return 0;

	size = adiv5_mem_ap_align(taddr, tlen);
	tlen >>= size;
	if (adiv5_mem_ap_setup_address(ma, taddr, size,
	    ADIV5_MEM_AP_CSW_ADDRINC_SINGLE) < 0) {
		return -1;
	}

	taddr_save = taddr;

	/*
	 * Pipeline reads rather than the "post-and-read-result" as we would
	 * for registers. This improves performance for large read requests.
	 */
	if (do_posted) {
		if (adiv5_mem_ap_read_posted_first(ma,
		    ADIV5_MEM_AP_REG_DRW) < 0) {
			return -1;
		}
		tlen--;
	}

	while (tlen) {
		tlen--;

		if (do_posted)
			data = adiv5_mem_ap_read_posted_next(ma, false);
		else
			data = adiv5_ap_read(&ma->ma_as, ADIV5_MEM_AP_REG_DRW);

		adiv5_unpack_read(buff, taddr, data, size);
		ADDR_INC(size);

		if (((taddr ^ taddr_save) & ma->ma_tar_mask) != 0) {
			/*
			 * Crossed an autoinc boundary. Maybe re-write TAR.
			 *
			 * Note: For posted reads, the pending result must
			 * be ignored - TAR is pointing to the start of
			 * the previous autoinc boundary.
			 */
			ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
			taddr_save = taddr;
			if (adiv5_mem_ap_setup_address(ma, taddr, size,
			    ADIV5_MEM_AP_CSW_ADDRINC_SINGLE) < 0) {
				return -1;
			}

			/* For posted reads, start afresh. */
			if (do_posted && adiv5_mem_ap_read_posted_first(ma,
			    ADIV5_MEM_AP_REG_DRW) < 0) {
				DBMFPRINTF(ADIV5_MEM_AP_API_ERRORS, "Post "
				    "failed near 0x%08" PRIxTADDR "\n", taddr);
				ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
				return -1;
			}
		}
	}

	if (do_posted) {
		data = adiv5_mem_ap_read_posted_next(ma, true);
		adiv5_unpack_read(buff, taddr, data, size);
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		DBMFPRINTF(ADIV5_MEM_AP_API_ERRORS, "Final read failed near "
		    "0x%08" PRIxTADDR "\n", taddr);
		ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
		return -1;
	}

	ma->ma_flags |= ADIV5_MA_FLAGS_TAR_VALID;
	ma->ma_tar = taddr;

	return 0;
}

static int
adiv5_mem_ap_sysmem_write_int(adiv5_mem_ap_t ma, target_addr_t taddr,
    uint32_t tlen, const void *buff)
{
	target_addr_t taddr_save;
	uint32_t size, data;
	int rv;

	size = adiv5_mem_ap_align(taddr, tlen);
	tlen >>= size;
	if (adiv5_mem_ap_setup_address(ma, taddr, size,
	    ADIV5_MEM_AP_CSW_ADDRINC_SINGLE) < 0) {
		return -1;
	}

	taddr_save = taddr;

	while (tlen--) {
		data = adiv5_pack_write(taddr, buff, size);
		ADDR_INC(size);

		rv = adiv5_ap_write(&ma->ma_as, ADIV5_MEM_AP_REG_DRW, data);
		if (rv < 0) {
			DBMFPRINTF(ADIV5_MEM_AP_API_ERRORS, "Write failed at "
			    "0x%08" PRIxTADDR "\n", taddr);
			ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
			return -1;
		}

		if (((taddr ^ taddr_save) & ma->ma_tar_mask) != 0) {
			/* Crossed an autoinc boundary. Maybe re-write TAR */
			ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
			taddr_save = taddr;
			if (adiv5_mem_ap_setup_address(ma, taddr, size,
			    ADIV5_MEM_AP_CSW_ADDRINC_SINGLE) < 0) {
				DBMFPRINTF(ADIV5_MEM_AP_API_ERRORS, "Write "
				    "failed at 0x%08" PRIxTADDR "\n", taddr);
				return -1;
			}
		}
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		DBMFPRINTF(ADIV5_MEM_AP_API_ERRORS, "Write failed near "
		    "0x%08" PRIxTADDR "\n", taddr);
		ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;
		return -1;
	}

	ma->ma_flags |= ADIV5_MA_FLAGS_TAR_VALID;
	ma->ma_tar = taddr;

	return 0;
}

#define	ALIGN_ADDR_COUNT(a)	((uint32_t)((4u - ((a) & 0x3u)) & 0x3u))

static int
adiv5_mem_ap_sysmem_read(void *arg, target_addr_t taddr, uint32_t tlen,
    void *buff)
{
	adiv5_mem_ap_t ma = arg;
	uint32_t this_len;
	uint8_t *p;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "taddr %08" PRIxTADDR ", tlen %08" PRIx32
	    ", buff %p\n", taddr, tlen, buff);

	if (tlen == 0)
		return 0;

	/*
	 * For optimal performance, attempt to align the address to a
	 * 32-bit boundary.
	 */
	p = buff;
	rv = 0;
	if ((this_len = ALIGN_ADDR_COUNT(taddr)) != 0 && tlen >= this_len) {
		rv = adiv5_mem_ap_sysmem_read_int(ma, taddr, this_len, p,
		    false);
		p += this_len;
		taddr += this_len;
		tlen -= this_len;
	}

	if (rv >= 0 && tlen >= sizeof(uint32_t)) {
		this_len = tlen & ~0x3u;
		rv = adiv5_mem_ap_sysmem_read_int(ma, taddr, this_len, p, true);
		p += this_len;
		taddr += this_len;
		tlen -= this_len;
	}

	if (rv >= 0 && tlen != 0)
		rv = adiv5_mem_ap_sysmem_read_int(ma, taddr, tlen, p, false);

	if (rv < 0) {
		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "failed at taddr %08" PRIxTADDR "\n", taddr);
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "taddr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", taddr, err);
		rv = -1;
	}

	return rv;
}

static int
adiv5_mem_ap_sysmem_write(void *arg, target_addr_t taddr, uint32_t tlen,
    const void *buff)
{
	adiv5_mem_ap_t ma = arg;
	uint32_t this_len;
	const uint8_t *p;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "taddr %08" PRIxTADDR ", tlen %08" PRIx32
	    ", buff %p\n", taddr, tlen, buff);

	if (tlen == 0)
		return 0;

	/*
	 * For optimal performance, attempt to align the address to a
	 * 32-bit boundary.
	 */
	p = buff;
	rv = 0;
	if ((this_len = ALIGN_ADDR_COUNT(taddr)) != 0 && tlen >= this_len) {
		rv = adiv5_mem_ap_sysmem_write_int(ma, taddr, this_len, p);
		p += this_len;
		taddr += this_len;
		tlen -= this_len;
	}

	if (rv >= 0 && tlen >= sizeof(uint32_t)) {
		this_len = tlen & ~0x3u;
		rv = adiv5_mem_ap_sysmem_write_int(ma, taddr, this_len, p);
		p += this_len;
		taddr += this_len;
		tlen -= this_len;
	}

	if (rv >= 0 && tlen != 0)
		rv = adiv5_mem_ap_sysmem_write_int(ma, taddr, tlen, p);

	if (rv < 0) {
		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "failed @ taddr %08" PRIxTADDR "\n", taddr);
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "taddr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", taddr, err);
		rv = -1;
	}

	return rv;
}

static int
adiv5_mem_ap_csw_prot(adiv5_mem_ap_t ma, const uint8_t *vp)
{
	uint32_t old_csw;

	if (vp == NULL)
		return -1;

	old_csw = ma->ma_csw;
	ma->ma_csw &= ~ADIV5_MEM_AP_CSW_PROT_MASK;
	ma->ma_csw |= ((uint32_t)*vp) << ADIV5_MEM_AP_CSW_PROT_SHIFT;

	if (old_csw != ma->ma_csw)
		ma->ma_data_size = ma->ma_addrinc = ~0u;

	DBMFPRINTF(ADIV5_MEM_AP_ATTACH, "csw: %08" PRIx32 " -> %08" PRIx32 "\n",
	    old_csw, ma->ma_csw);

	return 0;
}

static int
adiv5_mem_ap_rom_table_lookup(adiv5_mem_ap_t ma,
    struct adiv5_memap_rom_table_lookup *rtl)
{
	struct adiv5_rom_table *rt;

	for (rt = ma->ma_rom_tables;
	    rt < &ma->ma_rom_tables[ma->ma_rom_table_count]; rt++) {
		if ((rt->rt_pidr & rtl->rt_pidr_mask) == rtl->rt_pidr) {
			rtl->rt_paddr = rt->rt_address;
			return 1;
		}
	}

	return 0;
}

static int
adiv5_mem_ap_link_details(adiv5_mem_ap_t ma,
    struct adiv5_memap_link_details *ld)
{

	ld->ld_link_type = ADIV5_DP_TYPE(ma->ma_as.as_dp);
	ld->ld_link_name = ma->ma_as.as_dp->dp_if->dpi_name;
	ld->ld_apsel = ma->ma_as.as_ap_sel;
	ld->ld_targetsel = ma->ma_as.as_dp->dp_targetsel;

	return 0;
}

static int
adiv5_mem_ap_set_tar_bits(adiv5_mem_ap_t ma, const unsigned int *pbits)
{

	assert(*pbits >= 10u);

	ma->ma_tar_mask = ~((1u << *pbits) - 1);
	ma->ma_flags &= ~ADIV5_MA_FLAGS_TAR_VALID;

	return 0;
}

static int
adiv5_dp_power_up(adiv5_dp_t dp)
{
	timer_timeout_t to;
	uint32_t cs, v;

	/*
	 * Power things up.
	 *
	 * Note that this attempts to follow the sequence described
	 * in ADIv5, section B2.5.1.
	 */
	DBMFPRINTF(ADIV5_DP_PROBE, "Powering up\n");

	cs = DP_CTRL_STAT_CSYSWRUPREQ | DP_CTRL_STAT_CDBPWRUPREQ |
	    DP_CTRL_STAT_STICKYERR;
	dp->dp_select &= ~DP_SELECT_DPBANKSEL_MASK;
	if (adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs) < 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Failed to init CTRL/STAT\n");
		return -1;
	}

	/* Request debug power */
	DBMFPRINTF(ADIV5_DP_PROBE, "Set CDBPWRUPREQ\n");
	cs |= DP_CTRL_STAT_CDBPWRUPREQ;
	if (adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs) < 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Failed to write CTRL/STAT\n");
		return -1;
	}

	/* Wait until CDBPWRUPACK is set */
	DBMFPRINTF(ADIV5_DP_PROBE, "Waiting for CDBPWRUPACK\n");
	timer_timeout_start(&to, 2000);
	do {
		cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
		v = cs & DP_CTRL_STAT_CDBPWRUPACK;
		rtos_task_sleep(2);
	} while (v == 0 && !timer_timeout_expired(&to));
	if (v == 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "CDBPWRUPACK didn't set\n");
		return -1;
	}

	/* Power up the system */
	DBMFPRINTF(ADIV5_DP_PROBE, "Set CSYSWRUPREQ\n");
	cs |= DP_CTRL_STAT_CSYSWRUPREQ;
	adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs);

	DBMFPRINTF(ADIV5_DP_PROBE, "Waiting for CSYSWRUPACK\n");
	timer_timeout_start(&to, 2000);
	do {
		cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
		v = cs & DP_CTRL_STAT_CSYSWRUPACK;
	} while (v == 0 && !timer_timeout_expired(&to));
	if (v == 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "power-up timed out\n");
		return -1;
	}

	DBMFPRINTF(ADIV5_DP_PROBE, "Post power-up CTRL/STAT: %08" PRIx32 "\n",
	    cs);

	return 0;
}

static int
adiv5_mem_ap_ioctl(void *cookie, int cmd, void *arg)
{
	adiv5_mem_ap_t ma = cookie;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "cmd %d\n", cmd);

	switch (cmd) {
	case ADIV5_MEMAP_CTL_CSW_PROT:
		rv = adiv5_mem_ap_csw_prot(ma, arg);
		break;

	case ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP:
		rv = adiv5_mem_ap_rom_table_lookup(ma, arg);
		break;

	case ADIV5_MEMAP_CTL_LINK_DETAILS:
		rv = adiv5_mem_ap_link_details(ma, arg);
		break;

	case ADIV5_MEMAP_CTL_SET_TAR_BITS:
		rv = adiv5_mem_ap_set_tar_bits(ma, arg);
		break;

	case ADIV5_MEMAP_CTL_DP_POWER_UP:
		rv = adiv5_dp_power_up(ma->ma_as.as_dp);
		break;

	default:
		rv = -1;
		break;
	}

	return rv;
}

static int
adiv5_mem_ap_reg16_read(void *arg, target_addr_t addr, uint16_t *result)
{
	adiv5_mem_ap_t ma = arg;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "addr %08" PRIxTADDR "\n", addr);

	rv = 0;
	*result = adiv5_mem_ap_read16(ma, addr);

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err;

		DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
		    "Failed @ addr %08" PRIxTADDR "\n", addr);

		err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "addr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", addr, err);
		rv = -1;
	}

	DBMFPRINTF(ADIV5_MEM_AP_API, "Result %04" PRIx16 ", (rv %d)\n",
	    *result, rv);

	return rv;
}

static int
adiv5_mem_ap_reg16_write(void *arg, target_addr_t addr, uint16_t data)
{
	adiv5_mem_ap_t ma = arg;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "addr %08" PRIxTADDR ", data %04" PRIx16
	    "\n", addr, data);

	rv = adiv5_mem_ap_write16(ma, addr, data);

	if (rv < 0 || adiv5_dp_error(ma->ma_as.as_dp)) {
		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "Failed @ addr %08" PRIxTADDR "\n", addr);
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "addr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", addr, err);
		rv = -1;
	}

	return rv;
}

static int
adiv5_mem_ap_reg32_read(void *arg, target_addr_t addr, uint32_t *result)
{
	adiv5_mem_ap_t ma = arg;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "addr %08" PRIxTADDR "\n", addr);

	rv = 0;
	*result = adiv5_mem_ap_read32(ma, addr);

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err;

		DBMFPRINTF(ADIV5_MEM_AP_REG | ADIV5_MEM_AP_API_ERRORS,
		    "Failed @ addr %08" PRIxTADDR "\n", addr);

		err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "addr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", addr, err);
		rv = -1;
	}

	DBMFPRINTF(ADIV5_MEM_AP_API, "Result %08" PRIx32 ", (rv %d)\n", *result,
	    rv);

	return rv;
}

static int
adiv5_mem_ap_reg32_write(void *arg, target_addr_t addr, uint32_t data)
{
	adiv5_mem_ap_t ma = arg;
	int rv;

	DBMFPRINTF(ADIV5_MEM_AP_API, "addr %08" PRIxTADDR ", data %08" PRIx32
	    "\n", addr, data);

	rv = adiv5_mem_ap_write32(ma, addr, data);

	if (rv < 0) {
		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "Failed @ addr %08" PRIxTADDR "\n", addr);
	}

	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		uint32_t err = adiv5_dp_clear_error(ma->ma_as.as_dp);
		ma->ma_as.as_error_log |= err;

		DBMFPRINTF(ADIV5_MEM_AP_API | ADIV5_MEM_AP_API_ERRORS,
		    "addr %08" PRIxTADDR ": sticky error logged: %08" PRIx32
		    "\n", addr, err);
		rv = -1;
	}

	return rv;
}

static int
adiv5_dp_attach(adiv5_dp_t dp, uint32_t dpidr)
{
	timer_timeout_t to;
	uint32_t cs, v;
	int rv;

	(void) dpidr;

	DBMFPRINTF(ADIV5_DP_PROBE, "Clearing power REQs\n");
	cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
	DBMFPRINTF(ADIV5_DP_PROBE, "initial CTRL/STAT %08" PRIx32 "\n", cs);
	cs &= ~(DP_CTRL_STAT_CDBPWRUPREQ | DP_CTRL_STAT_CSYSWRUPREQ |
	    DP_CTRL_STAT_CDBGRSTREQ);
	if (adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs) < 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Failed to write CTRL/STAT\n");
		return -1;
	}

	/* Wait until CDBPWRUPACK is cleared */
	DBMFPRINTF(ADIV5_DP_PROBE, "waiting for CDBPWRUPACK\n");
	timer_timeout_start(&to, 2000);
	do {
		cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
		v = cs & (DP_CTRL_STAT_CDBPWRUPACK | DP_CTRL_STAT_CSYSWRUPACK);
		if (adiv5_dp_error(dp)) {
			adiv5_dp_clear_error(dp);
			v = 1;
		}
		rtos_task_sleep(2);
	} while (v != 0 && !timer_timeout_expired(&to));

	if (v != 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Warning: CDBPWRUPACK didn't "
		    "clear\n");
	}

	rv = adiv5_dp_power_up(dp);

	if (adiv5_dp_error(dp)) {
		uint32_t err = adiv5_dp_clear_error(dp);
		(void) err;
		DBMFPRINTF(ADIV5_DP_PROBE, "Sticky error post reset: "
		    "0x%" PRIx32, err);
	}

	if (rv < 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Powerup failed\n");
		return -1;
	}

	/* Assert system reset */
	DBMFPRINTF(ADIV5_DP_PROBE, "Asserting System Reset\n");
	cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT) | DP_CTRL_STAT_CDBGRSTREQ;
	adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs);
	cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);

	if ((cs & DP_CTRL_STAT_CDBGRSTREQ) != 0) {
		/*
		 * The write was not ignored so let's see if it works.
		 */
		/* Wait for ACK */
		timer_timeout_start(&to, 500);
		do {
			cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
			v = cs & DP_CTRL_STAT_CDBGRSTACK;
		} while (v == 0 && !timer_timeout_expired(&to));

		if (v == 0) {
			/*
			 * This happens on some platforms.
			 * Let's not sweat it.
			 */
			DBMFPRINTF(ADIV5_DP_PROBE, "DBG RST assert timed "
			    "out\n");
		}

		/* Deassert system reset */
		cs &= ~DP_CTRL_STAT_CDBGRSTREQ;
		adiv5_dp_write(dp, DP_REG_CTRL_STAT, cs);

		if (v != 0) {
			/* Wait for ACK to clear */
			timer_timeout_start(&to, 2000);
			do {
				cs = adiv5_dp_read(dp, DP_REG_CTRL_STAT);
				v = cs & DP_CTRL_STAT_CDBGRSTACK;
			} while (v != 0 && !timer_timeout_expired(&to));

			if (v != 0) {
				/*
				 * This happens on some platforms.
				 * Let's not sweat it.
				 */
				DBMFPRINTF(ADIV5_DP_PROBE, "DBG RST deassert "
				    "timed out\n");
			}
		}
	} else {
		DBMFPRINTF(ADIV5_DP_PROBE, "Reset request was ignored.\n");
	}

	DBMFPRINTF(ADIV5_DP_PROBE, "Post reset CTRL/STAT: %08" PRIx32 "\n",
	    adiv5_dp_read(dp, DP_REG_CTRL_STAT));

	if (adiv5_dp_error(dp)) {
		uint32_t err = adiv5_dp_clear_error(dp);
		(void) err;
		DBMFPRINTF(ADIV5_DP_PROBE, "Sticky error post reset: "
		    "0x%" PRIx32, err);
	}

	if (ADIV5_DP_TYPE(dp) != ADIV5_TYPE_JTAGDP) {
		uint8_t turnaround;
		/*
		 * See if we need to configure DLCR.TURNAROUND. Default is zero
		 * so only non-zero values need to be configured.
		 */
#ifndef CONFIG_USE_CONFIGDB
		turnaround = glob.glob_adiv5_turnaround;
#else
		if (cf_get_uint8(CF_KEY_UINT8_ADIV5_TURNAROUND,
		    &turnaround) == 0) {
			turnaround = ADIV5_DEFAULT_TURNAROUND;
		}
#endif

		if (turnaround) {
			uint32_t dlcr;

			DBMFPRINTF(ADIV5_DP_PROBE, "setting turnaround\n");

			/*
			 * Writing to the register should not cause an error.
			 */
			dlcr = adiv5_dp_read(dp, DP_REG_DLCR);
			dlcr &= ~DP_DLCR_TURNAROUND_MASK;
			dlcr |= DP_DLCR_TURNAROUND(turnaround);
			adiv5_dp_write(dp, DP_REG_DLCR, dlcr);

			/* Change our turnaround value */
			DBMFPRINTF(ADIV5_DP_PROBE, "turnaround set to %u\n",
			    (unsigned int)turnaround + 1);
			dp->dp_cfg &= ~ADIV5_LL_SWD_TURNAROUND_MASK;
			dp->dp_cfg |=
			    (turnaround + 1) << ADIV5_LL_SWD_TURNAROUND_SHIFT;
		}
	}

	DBMPRINTF(ADIV5_DP_PROBE, "Found DP with ID %08" PRIx32 "\n", dpidr);
	DBMPRINTF(ADIV5_DP_PROBE, "Designer %03x, DPv%u, MIN "
	    "%u, Partno %u, Revision %u\n\n", DP_DPIDR_DESIGNER(dpidr),
	    DP_DPIDR_VERSION(dpidr), DP_DPIDR_MIN(dpidr),
	    DP_DPIDR_PARTNO(dpidr), DP_DPIDR_REVISION(dpidr));

	return 1;
}

static void
adiv5_dp_explore(adiv5_dp_t dp, int dpversion)
{
	uint32_t ap_sel, apidr;
	int i;

	/* Scan for Access Ports */
	for (i = 0; i < 256; i++) {
		(void) adiv5_dp_read(dp, DP_REG_CTRL_STAT);

		ap_sel = DP_ENCODE_APSEL(DP_REQ_APnDP, i);

		apidr = adiv5_dp_read(dp, ap_sel | AP_REG_IDR);
		if (adiv5_dp_error(dp)) {
			uint32_t err = adiv5_dp_clear_error(dp);
			(void) err;

			DBMFPRINTF(ADIV5_DP_EXPLORE | ADIV5_DP_ERROR,
			    "Sticky error at AP idx %d: %08" PRIx32 "\n", i,
			    err);
			break;
		}

		if (apidr == 0) {
			adiv5_dp_abort(dp, DP_ABORT_DAPABORT);
			continue;
		}

		DBMPRINTF(ADIV5_DP_EXPLORE, "%s-AP ID %08" PRIx32 " at "
		    "APSEL %d\n",
		    (AP_IDR_CLASS(apidr) == AP_CLASS_MEM) ? "MEM" : "Unknown",
		    apidr, i);
		DBMPRINTF(ADIV5_DP_EXPLORE, "Designer %03x, Revision %u, "
		    "Class %u, Type %u, Variant %u\n\n",
		    AP_IDR_DESIGNER(apidr), AP_IDR_REVISION(apidr),
		    AP_IDR_CLASS(apidr), AP_IDR_TYPE(apidr),
		    AP_IDR_VARIANT(apidr));

		tmon_log("\n%s-AP ID %08" PRIx32 " at APSEL %d: ",
		    (AP_IDR_CLASS(apidr) == AP_CLASS_MEM) ? "MEM" : "Unknown",
		    apidr, i);
		tmon_log("Designer %03x, Rev %u, Class %u, Type %u, "
		    "Variant %u\n", AP_IDR_DESIGNER(apidr),
		    AP_IDR_REVISION(apidr), AP_IDR_CLASS(apidr),
		    AP_IDR_TYPE(apidr), AP_IDR_VARIANT(apidr));

		/* Split Memory Access Ports from everything else. */
		if (AP_IDR_DESIGNER(apidr) == AP_IDR_DESIGNER_ARM &&
		    AP_IDR_CLASS(apidr) == AP_CLASS_MEM) {
			adiv5_mem_ap_attach(dp, ap_sel, apidr, dpversion);
		} else {
			adiv5_other_ap_attach(dp, ap_sel, apidr);
		}
	}
}

static int
adiv5_dp_attach_and_explore(adiv5_dp_t dp, uint32_t dpidr)
{
	uint32_t err;
	int rv;

	/*
	 * Try to start up the DP.
	 */
	rv = adiv5_dp_attach(dp, dpidr);

	if (rv < 0 || adiv5_dp_error(dp)) {
		DBMFPRINTF(ADIV5_DP_PROBE, "rv %d, sticky %d\n", rv,
		    ADIV5_DP_STICKY_ERROR(dp));
		err = adiv5_dp_clear_error(dp);
		(void) err;
		DBMFPRINTF(ADIV5_DP_PROBE | ADIV5_DP_ERROR, "Cleared error "
		    "after adiv5_dp_attach(): %08" PRIx32 "\n", err);
		return -1;
	}

	/*
	 * We seem to have found one. Now explore it.
	 */
	const char *swtype = (ADIV5_DP_TYPE(dp) == ADIV5_TYPE_SWDP) ? "SW" :
	    ((ADIV5_DP_TYPE(dp) == ADIV5_TYPE_SWJDP) ? "SWJ" : "JTAG");

	tmon_log("\nExploring %s-DP, version %u", swtype,
	    DP_DPIDR_VERSION(dpidr));
	DBMFPRINTF(ADIV5_DP_PROBE | ADIV5_DP_EXPLORE, "Exploring %s-DP, "
	    "version %u", swtype, DP_DPIDR_VERSION(dpidr));

	if (DP_DPIDR_VERSION(dpidr) >= 2) {
		tmon_log(", TARGETSEL 0x%08" PRIx32 "\n", dp->dp_targetsel);
		DBMPRINTF(ADIV5_DP_PROBE | ADIV5_DP_EXPLORE,
		    ", TARGETSEL 0x%08" PRIx32 "\n", dp->dp_targetsel);
	} else {
		tmon_log(".\n");
		DBMPRINTF(ADIV5_DP_PROBE | ADIV5_DP_EXPLORE, ".\n");
	}

	adiv5_dp_explore(dp, (int)DP_DPIDR_VERSION(dpidr));

	if (adiv5_dp_error(dp)) {
		err = adiv5_dp_clear_error(dp);
		(void) err;

		DBMFPRINTF(ADIV5_DP_PROBE | ADIV5_DP_ERROR, "Cleared error "
		    "after adiv5_dp_explore(): %08" PRIx32 "\n", err);
		tmon_log("DP explore failed.\n");
	}

	if (TAILQ_EMPTY(&adiv5_mem_ap_list)) {
		tmon_log("\nNo usable MEM-AP ports found.\n");
		DBMFPRINTF(ADIV5_DP_PROBE | ADIV5_DP_ERROR, "No usable MEM-AP "
		    "ports found\n");
		if (TAILQ_EMPTY(&adiv5_other_ap_list))
			return 0;
		tmon_log("However, will probe the unknown AP(s)...\n");
		DBMFPRINTF(ADIV5_DP_PROBE | ADIV5_DP_ERROR, "However, will "
		    "probe the unknown AP(s)\n");
		return 1;
	}

	tmon_log("MEM-AP ports found.\n");
	return 1;
}

void
adiv5_cleanup(void)
{
	adiv5_other_ap_t oa;

	while ((oa = (void *)TAILQ_FIRST(&adiv5_other_ap_list))!=NULL) {
		TAILQ_REMOVE(&adiv5_other_ap_list, &oa->oa_as, as_qent);
		adiv5_other_ap_detach(oa);
	}

	assert(TAILQ_EMPTY(&adiv5_other_ap_list));
	assert(TAILQ_EMPTY(&adiv5_mem_ap_list));
	assert(TAILQ_EMPTY(&adiv5_dp_list));
}

static int
adiv5_dp_match_common(adiv5_dp_t proto_cdp, uint32_t dpidr, uint32_t targetsel)
{
	adiv5_dp_t dp;
	int rv;

	if ((dp = zone_malloc(sizeof(*dp))) == NULL) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Failed to alloc dp memory\n");
		rv = 0;
	} else {
		/* Clone the prototype DP structure. */
		*dp = *proto_cdp;
		dp->dp_targetsel = targetsel;
		TAILQ_INSERT_TAIL(&adiv5_dp_list, dp, dp_qent);

		assert(dp->dp_refcnt == 0);
		adiv5_dp_reference(dp);

		/* Attach and explore the DP port. */
		rv = adiv5_dp_attach_and_explore(dp, dpidr);

		adiv5_dp_unreference(dp);
	}

	return rv;
}

static int
adiv5_dp_match_rp2040(adiv5_dp_t proto_cdp, uint32_t dpidr,
    uint32_t targetid)
{
	adiv5_dp_t dp = proto_cdp;
	int rv;

	/*
	 * This part has a multidrop DP with the following instance numbers:
	 *
	 *  0: Cortex-M0+ core #0
	 *  1: Cortex-M0+ core #1
	 *  F: Rescue DP.
	 *
	 *  If we get no response on instance 0, then try the rescue procedure
	 *  on instance F before retrying instance 0.
	 */

	DBMFPRINTF(ADIV5_DP_PROBE, "TARGETID %08" PRIx32 ", DLPIDR %08" PRIx32
	    "\n", targetid, adiv5_dp_read(dp, DP_REG_DLPIDR));

	/*
	 * Make sure TARGETSEL is configured correctly for instance #0.
	 */
	DBMFPRINTF(ADIV5_DP_PROBE, "switch to instance #0\n");

	ADIV5_DP_SET_MULTIDROP(dp);
	dp->dp_targetsel = DP_TARGETSEL(targetid, 0);
	dpidr = adiv5_dp_target_select(dp, dp->dp_targetsel, NULL);

	/* Were we able to read DPIDR for instance #0? */
	if (dpidr == 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "instance #0 DPIDR is not valid.\n");

		/* Nope. Let's try the rescue DP. */
		dp->dp_targetsel = DP_TARGETSEL(targetid, 15);
		dpidr = adiv5_dp_target_select(dp, dp->dp_targetsel, NULL);
		if (dpidr == 0) {
			/* Something's gone wrong. */
			DBMFPRINTF(ADIV5_DP_PROBE, "DPIDR of rescue instance "
			    "not valid.\n");
			return 0;
		}

		/*
		 * Rescue DP is responding. Perform the 'rescue' procedure.
		 * Set CDBPWRUPREQ in CTRL/STAT. As per the datasheet, the
		 * ROM code will use this to halt before jumping into user
		 * code.
		 */
		DBMFPRINTF(ADIV5_DP_PROBE, "Performing rescue procedure...\n");

		if (adiv5_dp_write(dp, DP_REG_CTRL_STAT,
		    DP_CTRL_STAT_CDBPWRUPREQ) < 0) {
			/* Well, it was worth a try. */
			DBMFPRINTF(ADIV5_DP_PROBE, "Failed to set CDBPWRUPREQ "
			    "on rescue instance\n");
			return 0;
		}
		rtos_task_sleep(10);
		if (adiv5_dp_write(dp, DP_REG_CTRL_STAT, 0) < 0) {
			DBMFPRINTF(ADIV5_DP_PROBE, "Failed to clear CDBPWRUPREQ"
			    " on rescue instance\n");
			return 0;
		}
		rtos_task_sleep(100);

		DBMFPRINTF(ADIV5_DP_PROBE, "Rescue procedure completed.\n");

		/* Try again to read DPIDR on instance #0. */
		dp->dp_targetsel = DP_TARGETSEL(targetid, 0);
		dpidr = adiv5_dp_target_select(dp, dp->dp_targetsel, NULL);
		if (dpidr == 0) {
			/* Still no go. This device is toast. */
			DBMFPRINTF(ADIV5_DP_PROBE, "Instance #0 still not "
			    "responding.\n");
			return 0;
		}
	}

	/* DP#0 is responsive. Try to attach both cores. */

	DBMFPRINTF(ADIV5_DP_PROBE, "Attaching instance #0\n");
	rv = adiv5_dp_match_common(proto_cdp, dpidr, DP_TARGETSEL(targetid, 0));

	if (rv >= 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "Attaching instance #1\n");

		if (adiv5_dp_match_common(proto_cdp, dpidr,
		    DP_TARGETSEL(targetid, 1)) > 0) {
			rv++;
		}
	}

	DBMFPRINTF(ADIV5_DP_PROBE, "rv %d\n", rv);

	return rv;
}

#if 0
/* Useful should we ever need to probe all instances of a multi-drop DP... */
static int
adiv5_dp_match_multidrop(adiv5_dp_t proto_cdp, uint32_t dpidr,
    uint32_t targetid)
{
	adiv5_dp_t dp = proto_cdp;
	int rv, found;

	/*
	 * Generic multidrop DP.
	 */

	DBMFPRINTF(ADIV5_DP_PROBE, "TARGETID %08" PRIx32 ", DLPIDR %08" PRIx32
	    "\n", targetid, adiv5_dp_read(dp, DP_REG_DLPIDR));

	/*
	 * Make sure TARGETSEL is configured correctly for instance #0.
	 */
	DBMFPRINTF(ADIV5_DP_PROBE, "switch to instance #0\n");

	ADIV5_DP_SET_MULTIDROP(dp);
	found = 0;

	for (unsigned int inst = 0; inst < 16; inst++) {
		dp->dp_targetsel = DP_TARGETSEL(targetid, inst);
		dpidr = adiv5_dp_target_select(dp, dp->dp_targetsel, NULL);

		/* Were we able to read DPIDR for this instance? */
		if (dpidr != 0) {
			DBMFPRINTF(ADIV5_DP_PROBE, "Attaching instance #%u\n",
			    inst);
			rv = adiv5_dp_match_common(proto_cdp, dpidr,
			    dp->dp_targetsel);

			if (rv >= 0)
				found++;
		}
	}

	DBMFPRINTF(ADIV5_DP_PROBE, "rv %d\n", rv);

	return found;
}
#endif

struct adiv5_dp_match {
	uint32_t dm_targetid;
	int (*dm_handler)(adiv5_dp_t, uint32_t, uint32_t);
};

static const struct adiv5_dp_match adiv5_dp_match[] = {
	/* Raspberry PICO, RP2040 */
	{0x01002927u, adiv5_dp_match_rp2040},
};
#define	ADIV4_DP_MATCH_COUNT	(sizeof(adiv5_dp_match) / \
				 sizeof(adiv5_dp_match[0]))

/*
 * Return values:
 *  -2:	Could not bring up the ADIv5 interface.
 *  -1: No debug port found.
 *   0: Debug port found, but no devices attached to it.
 *   1: Access Ports found.
 */
static int
adiv5_dp_probe(const struct adiv5_dp_interface *dp_if)
{
	const struct adiv5_dp_match *dm;
	uint32_t dpidr, targetid, targetsel, cfg;
	struct adiv5_dp dp_store;
	bool did_abort;
	adiv5_dp_t dp;
	uint32_t err;
	int rv;

	assert(TAILQ_EMPTY(&adiv5_dp_list));
	assert(TAILQ_EMPTY(&adiv5_mem_ap_list));
	assert(TAILQ_EMPTY(&adiv5_other_ap_list));

	/* Fix up a dummy 'dp' for probe purposes. */
	memset(&dp_store, 0, sizeof(dp_store));
	dp = &dp_store;
	dp->dp_flags = dp_if->dpi_type;
	dp->dp_if = dp_if;
	adiv5_dp_current_targetsel = 0;

	cfg = 0;
	did_abort = false;

	/* Bring up the required interface. */
	if (dp_if->dpi_init(dp, cfg) == 0) {
		DBMFPRINTF(ADIV5_DP_PROBE, "i/f init failed\n");
		return -2;
	}

	for (;;) {
		if (ADIV5_DP_TYPE(dp) != ADIV5_TYPE_JTAGDP) {
			/*
			 * Since we're on SW-DP or SWJ-DP, we can assume
			 * DP version > 0. Therefore, DPIDR exists.
			 */
#ifdef ADIV5_VALIDATE_DP_REG
			/* Fake a DPv1 so adiv5_dp_select() won't complain. */
			dp->dp_idr = 0x00001000u;
#endif
			dpidr = adiv5_dp_read(dp, DP_REG_DPIDR);
			if (adiv5_dp_error(dp)) {
				err = adiv5_dp_clear_error(dp);
				(void) err;
				DBMFPRINTF(ADIV5_DP_PROBE, "Error reading "
				    "DPIDR: err = %" PRIx32 ".\n", err);
			} else {
				DBMFPRINTF(ADIV5_DP_PROBE, "DPIDR "
				    "read succeeded.\n");
				break;
			}
		} else {
			/*
			 * JTAG-DP. Assume IDCODE <=> DPIDR (docs suggest
			 * this isn't always the case...).
			 * Tentative read of CTRL/STAT. If it succeeds,
			 * we're in business.
			 */
			(void) adiv5_dp_read(dp, DP_REG_CTRL_STAT);
			if (!adiv5_dp_error(dp)) {
				dpidr = dp->dp_idr;
				break;
			}
		}

		if (did_abort == false) {
			/* Try again after sending an ABORT. */
			adiv5_dp_abort(dp, DP_ABORT_DAPABORT);
			did_abort = true;
			continue;
		}

		dp_if->dpi_done(dp);

		if (ADIV5_DP_TYPE(dp) == ADIV5_TYPE_JTAGDP) {
			/*
			 * No point trying again on JTAG - DORMANT is not
			 * supported.
			 */
			return -1;
		}

		/* We've done all we can; nobody's at home. */
		if ((cfg & ADIV5_LL_SWD_DORMANT) != 0)
			return -1;

		/* Try again; the SWDP might be dormant. */
		DBMFPRINTF(ADIV5_DP_PROBE, "DORMANT retry\n");
		cfg |= ADIV5_LL_SWD_DORMANT;
		did_abort = false;

		if (dp_if->dpi_init(dp, cfg) == 0) {
			DBMFPRINTF(ADIV5_DP_PROBE, "i/f init #2 failed\n");
			return -2;
		}
	}

	DBMFPRINTF(ADIV5_DP_PROBE, "DPIDR = %08" PRIx32 "\n", dpidr);
	tmon_log("Found DPv%" PRIu32 ", IDR %08" PRIx32 "\n",
	    DP_DPIDR_VERSION(dpidr), dpidr);

	targetid = targetsel = 0;
	dp->dp_idr = dpidr;
	if (DP_DPIDR_VERSION(dpidr) >= 2) {
		uint32_t dlpidr;

		dlpidr = adiv5_dp_read(dp, DP_REG_DLPIDR);
		targetid = adiv5_dp_read(dp, DP_REG_TARGETID);
		if (adiv5_dp_error(dp)) {
			/* Ok, we'll assume TARGETID == 0. */
			err = adiv5_dp_clear_error(dp);
			(void) err;
			DBMFPRINTF(ADIV5_DP_PROBE, "Error reading TARGETID. "
			    "err = %" PRIx32 "\n", err);
			targetid = 0;
			tmon_log("DP.TARGETID unreadable. Assume 0x0.\n");
		} else {
			DBMFPRINTF(ADIV5_DP_PROBE, "TARGETID 0x%" PRIx32 "\n",
			    targetid);
			tmon_log("DP.TARGETID %08" PRIx32 ".\n", targetid);

			if (targetid == 0xffffffffu)
				targetid = 0;
		}

		/*
		 * Before going further, ensure our notion of TARGETSEL is
		 * initialised.
		 */
		targetsel = adiv5_dp_current_targetsel = DP_TARGETSEL(targetid,
		    DP_DLPIDR_TINSTANCE(dlpidr));
	}

	rv = 0;
	for (dm = adiv5_dp_match;
	    dm < &adiv5_dp_match[ADIV4_DP_MATCH_COUNT];
	    dm++) {
		/*
		 * If we know about this DP, let its handler deal with
		 * probing and attaching it.
		 */
		if (dm->dm_targetid == (targetid & DP_TARGETID_MASK)) {
			rv = dm->dm_handler(dp, dpidr, targetid);
			break;
		}
	}

	if (dm == &adiv5_dp_match[ADIV4_DP_MATCH_COUNT]) {
		/*
		 * No direct match. Assume generic.
		 */
		rv = adiv5_dp_match_common(dp, dpidr, targetsel);
	}

	if (rv <= 0) {
		adiv5_cleanup();
		assert(TAILQ_EMPTY(&adiv5_dp_list));
		(dp_if->dpi_done)(dp);
		adiv5_dp_active = 0;
	} else {
		adiv5_dp_active = 1;
	}

	return rv;
}

int
adiv5_probe(void)
{
	uint8_t if_mask;
	int rv = -1;

#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_uint8(CF_KEY_UINT8_ADIV5_INTERFACE, &if_mask) == 0)
		if_mask = ADIV5_DEFAULT_INTERFACE;
#else
	if_mask = glob.glob_adiv5_interface;
#endif

	/* Ensure nRST pin is correctly configured. */
	adiv5_ll_configure_reset();

	for (unsigned int i = 0; i < ADIV5_DP_NINTERFACES; i++) {
		if (((1u << adiv5_dp_interfaces[i]->dpi_type) & if_mask) == 0)
			continue;

		DBMFPRINTF(ADIV5_DP_PROBE, "Probing %s...\n",
		    adiv5_dp_interfaces[i]->dpi_name);

		rv = adiv5_dp_probe(adiv5_dp_interfaces[i]);
		if (rv >= 0)
			break;
	}

	if (rv < 0)
		DBMFPRINTF(ADIV5_DP_PROBE, "Nothing found.\n");

	return rv;
}

static uint32_t
adiv5_dp_get_error_log(void *arg)
{
	struct adiv5_ap_state *as = arg;
	uint32_t rv;

	rv = as->as_error_log;
	as->as_error_log = 0;

	return rv;
}

static void
adiv5_mem_ap_free(void *arg)
{

	adiv5_mem_ap_detach((adiv5_mem_ap_t) arg);
}

static int
adiv5_mem_ap_ping(void *arg)
{
	adiv5_mem_ap_t ma = arg;
	int rv;

	(void) adiv5_ap_read(&ma->ma_as, ADIV5_MEM_AP_REG_BASE);

	rv = 0;
	if (adiv5_dp_error(ma->ma_as.as_dp)) {
		rv = -1;
		(void) adiv5_dp_clear_error(ma->ma_as.as_dp);

		if (!adiv5_dp_error(ma->ma_as.as_dp))
			rv = 0;
	}

	return rv;
}

static void
adiv5_ap_list(FILE *os, const char *prefix)
{
	struct adiv5_rom_table *rt;
	struct adiv5_ap_state *as;
	adiv5_mem_ap_t ma;
	u_int i;

	TAILQ_FOREACH(as, &adiv5_mem_ap_list, as_qent) {
		ma = (adiv5_mem_ap_t)(uintptr_t)as;
		fprintf(os, "%sMem-AP: APSEL %08" PRIx32 ", IDR 0x%08" PRIx32
		    ", errs %" PRIx32 "\n", prefix, as->as_ap_sel, as->as_idr,
		    as->as_error_log);
		prefix = "";
		fprintf(os, "\tCFG %08" PRIx32 ", CSW %08" PRIx32
		    ", flags %08u\n", ma->ma_cfg, ma->ma_csw, ma->ma_flags);
		fprintf(os, "\tROM tables %u:\n", ma->ma_rom_table_count);

		for (i = 0; i < ma->ma_rom_table_count; i++) {
			rt = &ma->ma_rom_tables[i];
			fprintf(os, "\t%3u: PIDR %08" PRIx32 ".%08" PRIx32
			    ", Address 0x%08" PRIx32 "\n", i,
			    (uint32_t)(rt->rt_pidr >> 32),
			    (uint32_t)rt->rt_pidr, (uint32_t)rt->rt_address);
		}
	}

	TAILQ_FOREACH(as, &adiv5_other_ap_list, as_qent) {
		fprintf(os, "%sOther-AP: APSEL %08" PRIx32 ", errs %" PRIx32
		    ", IDR %08" PRIx32 "\n", prefix, as->as_ap_sel,
		    as->as_error_log, as->as_idr);
		prefix = "";
	}
}

static const struct target_link_ops adiv5_mem_ap_interface = {
	.lo_reg_read16 = adiv5_mem_ap_reg16_read,
	.lo_reg_write16 = adiv5_mem_ap_reg16_write,
	.lo_reg_read32 = adiv5_mem_ap_reg32_read,
	.lo_reg_write32 = adiv5_mem_ap_reg32_write,
	.lo_mem_read = adiv5_mem_ap_sysmem_read,
	.lo_mem_write = adiv5_mem_ap_sysmem_write,
	.lo_ioctl = adiv5_mem_ap_ioctl,
	.lo_link_error = adiv5_dp_get_error_log,
	.lo_ping = adiv5_mem_ap_ping,
	.lo_free = adiv5_mem_ap_free
};

static const struct adiv5_ap_interface adiv5_other_ap_interface = {
	.ai_read = adiv5_ap_read,
	.ai_write = adiv5_ap_write,
	.ai_dp_error = adiv5_dp_get_error_log,
	.ai_free = adiv5_other_ap_free
};

int
adiv5_attach(adiv5_mem_ap_iterator_t mem_ap_fn,
    adiv5_other_ap_iterator_t other_ap_fn)
{
	struct adiv5_ap_state *as;
	int rv;

#if (RELEASE_BUILD == 0)
	adiv5_ap_list(stdout, "\n");
#endif

	if (!TAILQ_EMPTY(&adiv5_mem_ap_list) ||
	    !TAILQ_EMPTY(&adiv5_other_ap_list)) {
		FILE *fp;
		if ((fp = tmon_log_fp()) != NULL) {
			tmon_log("\nAP Summary...\n");
			adiv5_ap_list(fp, "");
		}
	}

	rv = TARGET_ATTACH_NO_MATCH;

	TAILQ_FOREACH(as, &adiv5_other_ap_list, as_qent) {
		rv = other_ap_fn(&adiv5_other_ap_interface, (void *)as,
		    as->as_idr);

		/*
		 * The return value must be TARGET_ATTACH_NO_MATCH or
		 * TARGET_ATTACH_RESCAN.
		 */
		assert(rv == TARGET_ATTACH_NO_MATCH ||
		    rv == TARGET_ATTACH_RESCAN);

		if (rv == TARGET_ATTACH_RESCAN)
			break;
	}

	while (rv != TARGET_ATTACH_RESCAN &&
	    (as = TAILQ_FIRST(&adiv5_mem_ap_list)) != NULL) {
		TAILQ_REMOVE(&adiv5_mem_ap_list, as, as_qent);

		ADIV5_DP_FORCE_SELECT_SET(as->as_dp);

		/* See if we support this device. */
		rv = mem_ap_fn(&adiv5_mem_ap_interface, (void *)as);

		if (rv != TARGET_ATTACH_OK) {
			/*
			 * Failed to attached. Since we've unlinked it from
			 * the list, we must free it here.
			 */
			adiv5_mem_ap_free((void *)as);
		}
	}

	if (rv == TARGET_ATTACH_RESCAN) {
		while ((as = TAILQ_FIRST(&adiv5_mem_ap_list)) != NULL) {
			TAILQ_REMOVE(&adiv5_mem_ap_list, as, as_qent);
			adiv5_mem_ap_free((void *)as);
		}
	}

	return rv;
}

void *
adiv5_other_ap_match(uint32_t idr, uint32_t idr_mask,
    const struct adiv5_ap_interface **adi)
{
	struct adiv5_ap_state *as;

	TAILQ_FOREACH(as, &adiv5_other_ap_list, as_qent) {
		adiv5_other_ap_t oa = (adiv5_other_ap_t)as;

		if ((as->as_idr & idr_mask) == idr) {
			TAILQ_REMOVE(&adiv5_other_ap_list, as, as_qent);
			*adi = &adiv5_other_ap_interface;
			return oa;
		}
	}

	return NULL;
}

bool
adiv5_ap_exists(unsigned int ap_number)
{
	struct adiv5_ap_state *as;
	uint32_t ap_sel;

	assert(ap_number <= 255u);

	ap_sel = DP_ENCODE_APSEL(DP_REQ_APnDP, ap_number);
	TAILQ_FOREACH(as, &adiv5_mem_ap_list, as_qent) {
		if (as->as_ap_sel == ap_sel)
			return true;
	}

	TAILQ_FOREACH(as, &adiv5_other_ap_list, as_qent) {
		if (as->as_ap_sel == ap_sel)
			return true;
	}

	return false;
}

static const char adapter_cmd_usage1[] =
"usage:\n"
"adaptor speed <value>\n"
"\tSets the SWCLK frequency for communicating with the target. The\n"
"\tfollowing values are recognised (frequency is approximate):\n";

static const char adapter_cmd_usage2[] =
"adaptor idlecycles <value>\n"
"\tSpecifies the number of idle cycles inserted after completing an SWD\n"
"\ttransaction. Legal values are 0 to 255. The default is 8.\n\n"
"adaptor turnaround <cycles>\n"
"\tSpecifies the number of clock cycles for the turnaround period when\n"
"\tchanging the SWDIO pin between input and output. Note that not all\n"
"\ttargets support changing this value. The status output of this command\n"
"\twill display a warning if the current target did not honour the\n"
"\trequested turnaround period. Legal values are 1 to 4. The default\n"
"\tis one.\n\n"
"adaptor interface <bits>\n"
"\tSelects which combination of SW-DP, SWJ-DP, or JTAG-DP interfaces to\n"
"\tuse when probing for debug ports. <bits> is a 3-bit bitmap, defined\n"
"\tas follows:\n"
"\t\tBit#\tInterface\n"
"\t\t0\tSW-DP\n"
"\t\t1\tSWJ-DP\n"
"\t\t2\tJTAG-DP\n"
"\tAny combination of bits, other than all zero, is possible. Maven\n"
"\twill try them in order of lowest bit to highest bit when probing\n"
"\tfor debug ports. Valid values for <mode> are 1 to 7 inclusive.\n"
"\tThe default is 3, to enable SW-DP and SWJ-DP interfaces.\n\n"
"Changes to the above settings are persistant across reboots but will not\n"
"affect the current connection with an existing target. Power-cycling,\n"
"disconnecting/reconnecting the target or initiating a 'rescan' will cause\n"
"the changes to take effect.\n";

static void
adiv5_cmd_show_usage(FILE *os)
{

	fputs(adapter_cmd_usage1, os);

	for (unsigned int i = 0; i < ADIV5_SPEED_2STR_COUNT; i++)
		fprintf(os, "\t  %u\t(%s)\n", i + 1, adiv5_speed_2str[i]);

	fprintf(os, "\tThe default is %u, meaning %s.\n\n",
	    ADIV5_DEFAULT_SPEED + 1, adiv5_speed_2str[ADIV5_DEFAULT_SPEED]);

	fputs(adapter_cmd_usage2, os);
}

static const struct {
	const char *ak_key;
	uint8_t ak_vmin, ak_vmax, ak_vbase, ak_vdef;
#ifndef CONFIG_USE_CONFIGDB
#define	ADIV5_KEY(k,vmin,vmax,vbase,vdef,id,gl)	\
	{.ak_key = k, .ak_vmin = vmin, .ak_vmax = vmax, .ak_vbase = vbase, \
	 .ak_vdef = vdef, .ak_gl = gl}
	uint8_t *ak_gl;
#else
#define	ADIV5_KEY(k,vmin,vmax,vbase,vdef,id,gl)	\
	{.ak_key = k, .ak_vmin = vmin, .ak_vmax = vmax, .ak_vbase = vbase, \
	 .ak_vdef = vdef, .ak_id = id}
	cf_id_t ak_id;
#endif
} adiv5_keys[] = {
	ADIV5_KEY("speed", 1, 9, 1, ADIV5_DEFAULT_SPEED,
	    CF_KEY_UINT8_ADIV5_SPEED, &glob.glob_adiv5_speed),

	ADIV5_KEY("idlecycles", 0, 255, 0, ADIV5_DEFAULT_IDLE_CYCLES,
	    CF_KEY_UINT8_ADIV5_IDLES, &glob.glob_adiv5_idles),

	ADIV5_KEY("turnaround", 1, 4, 1, ADIV5_DEFAULT_TURNAROUND,
	    CF_KEY_UINT8_ADIV5_TURNAROUND, &glob.glob_adiv5_turnaround),

	ADIV5_KEY("interface", 1, 7, 0, ADIV5_DEFAULT_INTERFACE,
	    CF_KEY_UINT8_ADIV5_INTERFACE, &glob.glob_adiv5_interface),
};
#define	ADIV5_NKEYS	(sizeof(adiv5_keys) / sizeof(adiv5_keys[0]))

static void
adiv5_cmd_adapter(FILE *os, uint8_t argc, const char * const *argv)
{
	unsigned int v;
	size_t arglen;
	uint32_t cfg;
	uint8_t val;
	unsigned int i;
#ifndef CONFIG_USE_CONFIGDB
	uint8_t *vp;
#else
	cf_id_t id;
#endif

	if (argc) {
		if (argc != 2) {
			adiv5_cmd_show_usage(os);
			return;
		}

		arglen = strlen(argv[0]);

		for (i = 0; i < ADIV5_NKEYS; i++) {
			if (strncasecmp(adiv5_keys[i].ak_key, argv[0],
			    arglen) == 0) {
				break;
			}
		}

		if (i == ADIV5_NKEYS) {
			adiv5_cmd_show_usage(os);
			return;
		}

		if (sscanf(argv[1], "%u", &v) != 1) {
			fprintf(os, "Malformed value '%s'\n", argv[1]);
			return;
		}

#ifndef CONFIG_USE_CONFIGDB
		vp = adiv5_keys[i].ak_gl;
#else
		id = adiv5_keys[i].ak_id;
#endif
		if (v < adiv5_keys[i].ak_vmin || v > adiv5_keys[i].ak_vmax) {
			fprintf(os, "Legal values are %u to %u\n",
			    (unsigned int)adiv5_keys[i].ak_vmin,
			    (unsigned int)adiv5_keys[i].ak_vmax);
			return;
		}

		v -= adiv5_keys[i].ak_vbase;

#ifndef CONFIG_USE_CONFIGDB
		if (*vp != (uint8_t)v) {
			*vp = (uint8_t)v;
			glob_save();
		}
#else
		if (cf_get_uint8(id, &val) == 0)
			val = adiv5_keys[i].ak_vdef;

		if (val != (uint8_t)v) {
			cf_set_uint8(id, (uint8_t)v);
			configdb_persist();
		}
#endif
	}

	rtos_scheduler_suspend();
	adiv5_dp_t dp = TAILQ_FIRST(&adiv5_dp_list);
	if (dp != NULL)
		cfg = dp->dp_cfg;
	else
		cfg = 0xfffffffful;
	rtos_scheduler_resume();

	uint8_t if_mask;
	bool comma;
#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_uint8(CF_KEY_UINT8_ADIV5_INTERFACE, &if_mask) == 0)
		if_mask = ADIV5_DEFAULT_INTERFACE;
#else
	if_mask = glob.glob_adiv5_interface;
#endif
	comma = false;
	fprintf(os, "DP Interfaces: ");
	for (i = 0; i < ADIV5_DP_NINTERFACES; i++) {
		if ((if_mask & (1u << adiv5_dp_interfaces[i]->dpi_type)) != 0) {
			fprintf(os, "%s%s", comma ? ", " : "",
			    adiv5_dp_interfaces[i]->dpi_name);
			comma = true;
		}
	}
	fputc('\n', os);

	unsigned int speed, idles;
	adiv5_dp_get_speed_idles(&speed, &idles);

	fprintf(os, "Current speed: %s\n", adiv5_speed_2str[speed]);
	fprintf(os, "Idle cycles: %u\n", idles);

#ifndef CONFIG_USE_CONFIGDB
	val = glob.glob_adiv5_turnaround;
#else
	cf_get_uint8(CF_KEY_UINT8_ADIV5_TURNAROUND, &val);
#endif
	fprintf(os, "Turnaround period: %u", (unsigned int)val + 1);

	if (v != 0xfffffffful) {
		v = (cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
		    ADIV5_LL_SWD_TURNAROUND_SHIFT;
		if (v != (uint32_t)(val + 1)) {
			fputs(" (Warning: Target not using this setting)",
			    os);
		}
	}

	fprintf(os, "\n");
}

void
adiv5_init(void)
{

	adiv5_ll_init_target();

	SHELL_CMD_ADD(adapter);
}

#ifdef NETWORK_OPT_HTTPD
int
adiv5_update_settings(int nkeys,
    const struct network_http_furl_key_value *kv)
{
	unsigned int v32;
	int changed;
	uint8_t v8;

	changed = 0;
	while (nkeys--) {
		for (unsigned int i = 0; i < ADIV5_NKEYS; i++) {
			if (strncmp(kv->kv_key, "swd-", 4) == 0 &&
			    strcmp(adiv5_keys[i].ak_key, &kv->kv_key[4]) == 0 &&
			    sscanf(kv->kv_value, "%u", &v32) == 1 &&
			    v32 >= (unsigned int)adiv5_keys[i].ak_vmin &&
			    v32 <= (unsigned int)adiv5_keys[i].ak_vmax) {
				v32 -= adiv5_keys[i].ak_vbase;
				if (cf_get_uint8(adiv5_keys[i].ak_id, &v8) == 0)
					v8 = adiv5_keys[i].ak_vdef;
				if (v8 != (uint8_t)v32) {
					v8 = (uint8_t)v32;
					cf_set_uint8(adiv5_keys[i].ak_id, v8);
					changed++;
				}
			}
		}
		kv++;
	}

	return changed;
}

void
adiv5_debug_update(stringio_t ss)
{
	unsigned int i;

	for (i = 0; i < ADIV5_NKEYS; i++) {
		unsigned int v32;
		uint8_t v8;

		if (cf_get_uint8(adiv5_keys[i].ak_id, &v8) == 0)
			continue;
		v32 = (unsigned int)v8;
		v32 += adiv5_keys[i].ak_vbase;

		stringio_printf(ss, ",\"swd_%s\":%u", adiv5_keys[i].ak_key, v32);
	}

	for (i = 0; i < ADIV5_SPEED_2STR_COUNT; i++) {
		stringio_printf(ss, ",\"swd_speed_%u\":\"%s\"", i + 1,
		    adiv5_speed_2str[i]);
	}
}
#endif /* NETWORK_OPT_HTTPD */

void
adiv5_load_defaults(void)
{

	for (unsigned int i = 0; i < ADIV5_NKEYS; i++) {
#ifndef CONFIG_USE_CONFIGDB
		*adiv5_keys[i].ak_gl = adiv5_keys[i].ak_vdef;
#else
		cf_set_uint8(adiv5_keys[i].ak_id, adiv5_keys[i].ak_vdef);
#endif
	}
}
