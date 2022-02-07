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

#include "rtos.h"
#include "microchip_dsu.h"
#include "adiv5_ll.h"
#include "cortexm.h"
#include "cortexm_regs.h"
#include "cortexm_txml.h"
#include "target.h"
#include "soc_sam0.h"
#include "soc_samd5x.h"
#include "hardware.h"
#include "timer.h"
#include "twiddle.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Peripheral ID for DSU
 */
#define MCP_DSUROM_PIDR		0x000001fcd0u
#define MCP_DSUROM_MASK		0xffff07ffffu

/*
 * According to the datasheet:
 * <quote>
 *  For security reasons, DSU features have limitations when used from a debug
 *  adapter. To differentiate external accesses from internal ones, the first
 *  0x100 bytes of the DSU register map have been replicated at offset 0x100:
 *     o The first 0x100 bytes form the internal address range
 *     o The next 0x1F00 bytes form the external address range
 *  When the device is protected, the DAP can only issue MEM-AP accesses in the
 *  DSU address range limited to the 0x100-0x2000 offset range.
 * </quote>
 */
#define	MCP_DSUROM_TO_DSUREG(a)			((a) - 0x0f00u)

#define	MCP_DSU_REG_CTRL			0x00u
#define	 MCP_DSU_CTRL_SWRST			(1u << 0)
#define	 MCP_DSU_CTRL_MBIST			(1u << 3)
#define	 MCP_DSU_CTRL_CE			(1u << 4)	/* !BootROM */
#define	MCP_DSU_REG_STATUSA			0x01u
#define	 MCP_DSU_STATUSA_DONE			(1u << 0)
#define	 MCP_DSU_STATUSA_CRSTEXT		(1u << 1)
#define	 MCP_DSU_STATUSA_BERR			(1u << 2)
#define	 MCP_DSU_STATUSA_FAIL			(1u << 3)
#define	 MCP_DSU_STATUSA_PERR			(1u << 4)
#define	 MCP_DSU_STATUSA_BREXT			(1u << 5)
#define	MCP_DSU_REG_STATUSB			0x02u
#define	 MCP_DSU_STATUSB_PROT			(1u << 0)
#define	 MCP_DSU_STATUSB_DBGPRES		(1u << 1)
#define	 MCP_DSU_STATUSB_DCD0			(1u << 2)
#define	 MCP_DSU_STATUSB_DCD1			(1u << 3)
#define	 MCP_DSU_STATUSB_HPE			(1u << 4)
#define	 MCP_DSU_STATUSB_M4_CELCK		(1u << 5)
#define	 MCP_DSU_STATUSB_BR_DAL(x)		((x) & 3u)
#define	 MCP_DSU_STATUSB_BR_DAL_2		2u
#define	 MCP_DSU_STATUSB_BR_DAL_1		1u
#define	 MCP_DSU_STATUSB_BR_DAL_0		0u
#define	 MCP_DSU_STATUSB_BR_DBGPRES		(1u << 2)
#define	 MCP_DSU_STATUSB_BR_HPE			(1u << 3)
#define	 MCP_DSU_STATUSB_BR_DCCD(x)		(1u << (4 + ((x) & 1u)))
#define	 MCP_DSU_STATUSB_BR_BCCD(x)		(1u << (6 + ((x) & 1u)))
#define	MCP_DSU_REG_ADDRESS			0x04u
#define	 MCP_DSU_ADDRESS_AMOD_MASK		0x00000003u
#define	 MCP_DSU_ADDRESS_MASK			0xfffffffcu
#define	MCP_DSU_REG_LENGTH			0x08u
#define	 MCP_DSU_LENGTH_MASK			0xfffffffcu
#define	MCP_DSU_REG_DATA			0x0cu
#define	MCP_DSU_REG_DCC0			0x10u
#define	MCP_DSU_REG_DCC1			0x14u
#define	MCP_DSU_REG_DID				0x18u
#define	MCP_DSU_REG_CONFIG			0x1cu
#define	 MCP_DSU_CONFIG_DCCDMALEVEL_MASK(x)	(1u << (2 + ((x) & 1u)))
#define	 MCP_DSU_CONFIG_DCCDMALEVEL_READ(x)	(0u << (2 + ((x) & 1u)))
#define	 MCP_DSU_CONFIG_DCCDMALEVEL_WRITE(x)	(1u << (2 + ((x) & 1u)))
#define	 MCP_DSU_CONFIG_LQOS_MASK		0x00000003u
#define	MCP_DSU_REG_BCC0			0x20u
#define	MCP_DSU_REG_BCC1			0x24u

/*
 * Boot Interactive Mode Commands.
 */
#define	MCP_BIM_CMD_INIT			0x55u
#define	MCP_BIM_CMD_EXIT			0xaau
#define	MCP_BIM_CMD_RESET			0x52u
#define	MCP_BIM_CMD_CE0				0xe0u
#define	MCP_BIM_CMD_CE1				0xe1u
#define	MCP_BIM_CMD_CE2				0xe2u
#define	MCP_BIM_CMD_CHIPERASE			0xe3u
#define	MCP_BIM_CMD_PREFIX			0x44424700u

/*
 * Boot Interactive Mode Status.
 */
#define	MCP_BIM_SIG_NO				0x00u
#define	MCP_BIM_SIG_SAN_FFF			0x10u
#define	MCP_BIM_SIG_SAN_UROW			0x11u
#define	MCP_BIM_SIG_SAN_SECEN			0x12u
#define	MCP_BIM_SIG_SAN_BOCOR			0x13u
#define	MCP_BIM_SIG_SAN_BOOTPROT		0x14u
#define	MCP_BIM_SIG_SAN_NOSECREG		0x15u
#define	MCP_BIM_SIG_COMM			0x20u
#define	MCP_BIM_SIG_CMD_SUCCESS			0x21u
#define	MCP_BIM_SIG_CMD_FAIL			0x22u
#define	MCP_BIM_SIG_CMD_BADKEY			0x23u
#define	MCP_BIM_SIG_CMD_VALID			0x24u
#define	MCP_BIM_SIG_CMD_INVALID			0x25u
#define	MCP_BIM_SIG_ARG_VALID			0x26u
#define	MCP_BIM_SIG_ARG_INVALID			0x27u
#define	MCP_BIM_SIG_CE_CVM			0x30u
#define	MCP_BIM_SIG_CE_ARRAY_ERASE_FAIL		0x31u
#define	MCP_BIM_SIG_CE_ARRAY_NVME		0x32u
#define	MCP_BIM_SIG_CE_DATA_ERASE_FAIL		0x33u
#define	MCP_BIM_SIG_CE_DATA_NVME		0x34u
#define	MCP_BIM_SIG_CE_BCUR			0x35u
#define	MCP_BIM_SIG_CE_BC			0x36u
#define	MCP_BIM_SIG_BOOTOK			0x39u
#define	MCP_BIM_SIG_BOOT_OPT			0x40u
#define	MCP_BIM_SIG_MASK			0xffffff00u
#define	MCP_BIM_SIG_PREFIX			0xec000000u

#define	CMSCS_BASE				0xe0001000u

/* SoCs which have DSUs */
struct mcdsu_supported_soc {
	void *(*ss_probe)(uint32_t did, uint32_t *pflags);
	uint32_t (*ss_register)(uint32_t did, void *);
};
static const struct mcdsu_supported_soc mcdsu_supported_socs[] = {
	{soc_sam0_dsu_probe, soc_sam0_dsu_register},
	{soc_samd5x_dsu_probe, soc_samd5x_dsu_register},
};
#define	MCDSU_NSUPPORTED_SOCS	(sizeof(mcdsu_supported_socs) / \
				 sizeof(mcdsu_supported_socs[0]))

struct mcdsu_state {
	target_t ms_t;
	target_addr_t ms_dsu;
	microchip_dsu_protection_t ms_prot;
	uint32_t ms_soc_flags;
	uint32_t ms_erase_time;
	bool ms_chip_erase_requested;
	int (*ms_chip_erase)(struct mcdsu_state *);
	int (*ms_chip_erase_status)(struct mcdsu_state *);
};
static struct mcdsu_state mcdsu_state;

#define	MC_DSU_HAS_BOOTROM(ms)	(MCP_DSU_FLAGS_TYPE(ms->ms_soc_flags) == \
				 MCP_DSU_FLAGS_TYPE_ROM)
#define	MC_DSU_TRUSTZONE(ms)	(MC_DSU_HAS_BOOTROM(ms) && \
				 (ms->ms_soc_flags & MCP_DSU_FLAGS_TRUSTZONE))

static int
mc_dsu_read8(struct mcdsu_state *ms, u_int reg, uint8_t *pv)
{

	return TARGET_MEM_READ(ms->ms_t, ms->ms_dsu + reg, 1, pv);
}

static int
mc_dsu_write8(struct mcdsu_state *ms, u_int reg, uint8_t v)
{

	return TARGET_MEM_WRITE(ms->ms_t, ms->ms_dsu + reg, 1, &v);
}

static int
mc_dsu_read32(struct mcdsu_state *ms, u_int reg, uint32_t *pv)
{

	return TARGET_REG_READ32(ms->ms_t, ms->ms_dsu + reg, pv);
}

static int
mc_dsu_write32(struct mcdsu_state *ms, u_int reg, uint32_t v)
{

	return TARGET_REG_WRITE32(ms->ms_t, ms->ms_dsu + reg, v);
}

static int
mc_dsu_check_rom_status(struct mcdsu_state *ms, uint8_t *pstatus)
{
	uint32_t bcc;
	uint8_t s;

	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSB, &s) < 0)
		return -1;

	if ((s & MCP_DSU_STATUSB_BR_BCCD(1)) == 0)
		return 0;

	if (mc_dsu_read32(ms, MCP_DSU_REG_BCC1, &bcc) < 0)
		return -1;

	if ((bcc & MCP_BIM_SIG_MASK) != MCP_BIM_SIG_PREFIX)
		return -1;

	*pstatus = (uint8_t)bcc;

	return 1;
}

static int
mc_dsu_send_command(struct mcdsu_state *ms, uint8_t cmd, uint8_t *pstatus)
{
	timer_timeout_t to;
	int srv;

	if (mc_dsu_write32(ms, MCP_DSU_REG_BCC0,
	    MCP_BIM_CMD_PREFIX | (uint32_t)cmd) < 0) {
		DBFPRINTF("Failed to write cmd 0x%x\n", (unsigned int)cmd);
		return -1;
	}

	timer_timeout_start(&to, 1000u);
	while ((srv = mc_dsu_check_rom_status(ms, pstatus)) == 0 &&
	    !timer_timeout_expired(&to)) {
		rtos_task_sleep(1);
	}

	if (srv == 0) {
		DBFPRINTF("Timeout for cmd 0x%x\n", (unsigned int)cmd);
		return -1;
	}

	if (srv < 0) {
		DBFPRINTF("Status error for cmd 0x%x\n", (unsigned int)cmd);
		return -1;
	}

	return 0;
}

static int
mc_start_bootrom(struct mcdsu_state *ms)
{
	uint8_t statusb;
	uint32_t v32;

	/* Give the boot ROM a bit more time to do its thing. */
	rtos_task_sleep(5);

	/* Check STATUSB.BCC1D for errors */
	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSB, &statusb) < 0) {
		DBFPRINTF("failed to read STATUSB\n");
			return -1;
	}

	if (statusb & MCP_DSU_STATUSB_BR_BCCD(1)) {
		/* Read the error code... */
		if (mc_dsu_read32(ms, MCP_DSU_REG_BCC1, &v32) < 0) {
			DBFPRINTF("failed to read BCC1\n");
			return -1;
		}

		DBFPRINTF("Error code %08" PRIx32 "\n", v32);
		return -1;
	}

	DBFPRINTF("running.\n");

	return 0;
}

static int
mc_dsu_clear_brext(struct mcdsu_state *ms)
{
	uint8_t v;

	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSA, &v) < 0) {
		DBFPRINTF("failed to read STATUSA\n");
		return -1;
	}

	DBFPRINTF("STATUSA %02x\n", (unsigned int)v);

	if ((v & MCP_DSU_STATUSA_BREXT) != 0) {
		if (mc_dsu_write8(ms, MCP_DSU_REG_STATUSA, v) < 0) {
			DBFPRINTF("failed to clear BREXT\n");
			return -1;
		}
	}

	rtos_task_sleep(5);

	return 0;
}

static int
mc_bootrom_interact(struct mcdsu_state *ms, uint8_t statusb, bool clear_brext)
{
	uint8_t sig;

	DBFPRINTF("Starting Boot ROM\n");

	/*
	 * Let the boot ROM start up and perform its integrity checks.
	 */
	if (mc_start_bootrom(ms) < 0) {
		/* Something went wrong. */
		return TARGET_ATTACH_NO_MATCH;
	}

	if (MCP_DSU_STATUSB_BR_DAL(statusb) <= MCP_DSU_STATUSB_BR_DAL_1 &&
	    clear_brext) {
		/*
		 * Neither a chip erase nor manual extended reset have been
		 * requested, and the debugger still has access to untrusted
		 * resources on the device.
		 *
		 * Clear BREXT and issue an EXIT command to the boot ROM, to
		 * allow the CPU core and SoC-specific code to attach as usual.
		 */
		if (mc_dsu_clear_brext(ms) != 0)
			return TARGET_ATTACH_NO_MATCH;

		if (mc_dsu_send_command(ms, MCP_BIM_CMD_EXIT, &sig) != 0)
			return TARGET_ATTACH_NO_MATCH;

		DBFPRINTF("CMD_EXIT sig %02x\n", (unsigned int)sig);

		return TARGET_ATTACH_NO_MATCH;
	}

	/* Initiate comms with the boot ROM. */
	if (mc_dsu_send_command(ms, MCP_BIM_CMD_INIT, &sig) != 0)
		return TARGET_ATTACH_NO_MATCH;

	DBFPRINTF("CMD_INIT sig %02x\n", (unsigned int)sig);

	if (sig != MCP_BIM_SIG_COMM)
		return TARGET_ATTACH_NO_MATCH;

	/* Comms established with the boot ROM. */
	return TARGET_ATTACH_OK;
}

static int
mc_bootrom_erase(struct mcdsu_state *ms)
{
	uint8_t sig;

	if (mc_dsu_send_command(ms, MCP_BIM_CMD_CHIPERASE, &sig) != 0)
		return -1;

	DBFPRINTF("CMD_CHIPERASE sig %02x\n", (unsigned int)sig);

	if (sig == MCP_BIM_SIG_CMD_VALID)
		return 0;

	if (sig == MCP_BIM_SIG_NO) {
		DBFPRINTF("Unexpected MCP_BIM_SIG_NO\n");
		return -1;
	}

	return (int)sig;
}

static int
mc_bootrom_erase_tz(struct mcdsu_state *ms)
{
	uint8_t sig;

	if (mc_dsu_send_command(ms, MCP_BIM_CMD_CHIPERASE, &sig) != 0)
		return -1;

	DBFPRINTF("CMD_CHIPERASE sig %02x\n", (unsigned int)sig);

	if (sig == MCP_BIM_SIG_CMD_VALID)
		return 0;

	if (sig == MCP_BIM_SIG_NO) {
		DBFPRINTF("Unexpected MCP_BIM_SIG_NO\n");
		return -1;
	}

	return (int)sig;
}

static int
mc_bootrom_erase_status(struct mcdsu_state *ms)
{
	uint8_t sig;
	int rv;

	rv = mc_dsu_check_rom_status(ms, &sig);
	if (rv == 0)
		return 0;

	if (rv < 0) {
		DBFPRINTF("Status read failed.\n");
		return -1;
	}

	if (sig == 0) {
		DBFPRINTF("Unexpected zero sig.\n");
		sig = MCP_BIM_SIG_CMD_FAIL;
	}

	return (int)sig;
}

static int
mc_dumb_erase(struct mcdsu_state *ms)
{
	uint8_t v8;

	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSA, &v8) < 0) {
		DBFPRINTF("STATUSA read failed.\n");
	} else {
		DBFPRINTF("STATUSA 0x%02x.\n", (unsigned int)v8);
	}

	v8 &= MCP_DSU_STATUSA_DONE | MCP_DSU_STATUSA_BERR |
	    MCP_DSU_STATUSA_FAIL | MCP_DSU_STATUSA_PERR;
	if (v8 != 0) {
		DBFPRINTF("clearing stale status bits.\n");
		if (mc_dsu_write8(ms, MCP_DSU_REG_STATUSA, v8) < 0) {
			DBFPRINTF("failed to clear stale status bits.\n");
			return -1;
		}
	}

#if (DEBUG_FLAG_INIT != 0)
	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSB, &v8) < 0) {
		DBFPRINTF("STATUSB read failed.\n");
	} else {
		DBFPRINTF("STATUSB 0x%02x.\n", (unsigned int)v8);
	}
#endif

	return mc_dsu_write8(ms, MCP_DSU_REG_CTRL, MCP_DSU_CTRL_CE);
}

static int
mc_dumb_erase_status(struct mcdsu_state *ms)
{
	uint8_t statusa;

	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSA, &statusa) < 0) {
		DBFPRINTF("STATUSA read failed.\n");
		return -1;
	}

	if ((statusa & MCP_DSU_STATUSA_DONE) == 0)
		return 0;

	/*
	 * Clear the bits, except for BREXT; that comes later.
	 */
	if (mc_dsu_write8(ms, MCP_DSU_REG_STATUSA,
	    statusa & ~MCP_DSU_STATUSA_BREXT) < 0) {
		DBFPRINTF("STATUSA write failed.\n");
	}

	/* Keep only the DONE and error bits. */
	statusa &= MCP_DSU_STATUSA_DONE | MCP_DSU_STATUSA_BERR |
	    MCP_DSU_STATUSA_FAIL | MCP_DSU_STATUSA_PERR;

	return statusa;
}

static int
mc_dsu_rom_exit_to_reset(struct mcdsu_state *ms, FILE *os)
{
	uint8_t sig;
	int rv;

	/*
	 * Note that we assume BREXT still set at this point.
	 * XXX: Should we double-check?
	 */
	rv = mc_dsu_send_command(ms, MCP_BIM_CMD_EXIT, &sig);
	if (rv < 0 || sig != MCP_BIM_SIG_BOOTOK) {
		fprintf(os, "BIM EXIT command failed (Status 0x%x)\n",
		    (unsigned int)sig);
		return -1;
	}

	/*
	 * CPU is now parked awaiting BREXT to be cleared.
	 */
	if ((rv = mc_dsu_clear_brext(ms)) != 0) {
		fprintf(os, "Failed to clear BREXT. Try power-cycle.\n");
		return TARGET_PI_OK_POWER_CYCLE;
	}

	return TARGET_PI_OK_RESET;
}

static int
mc_dsu_ctl(target_t t, uint32_t *cmd, void *arg)
{
	struct mcdsu_state *ms = t->t_core;
	timer_timeout_t to;
	int status;
	FILE *os;

	if (*cmd != TARGET_CTL_UNLOCK)
		return 0;

	*cmd = TARGET_CTL_NOP;
	os = arg;

	/* Issue a Chip Erase command. */
	status = ms->ms_chip_erase(ms);

	/*
	 * status <  0: Interface error.
	 * status == 0: Erase in progress.
	 * status >  0: Boot ROM error code.
	 */

	if (status != 0) {
		fprintf(os, "Failed to initiate chip erase.");
		if (MC_DSU_HAS_BOOTROM(ms))
			fprintf(os, " BIM Status 0x%0x\n", status);
		else
			fputc('\n', os);
		return -1;
	}

	DBFPRINTF("erase_time %" PRIu32 "\n", ms->ms_erase_time);

	fprintf(os, "Chip erase in progress... ");
	fflush(os);

	/* Wait for DONE to be set. */
	timer_timeout_start(&to, ms->ms_erase_time);
	do {
		twiddle(os);

		status = ms->ms_chip_erase_status(ms);

		if (status < 0) {
			fprintf(os, "\bDevice error during erase\n");
			return -1;
		}

		if (status != 0)
			break;

		rtos_task_sleep(1);
	} while (!timer_timeout_expired(&to));

	/*
	 * status == 0: Timed out.
	 * status == 1: Success.
	 * status >  1: Error code.
	 */

	DBFPRINTF("Erase status %d\n", status);

	if (status != 1) {
		fprintf(os, "\bChip erase failed");

		if (status == 0)
			fprintf(os, ": Timeout.\n");
		else
		if (MC_DSU_HAS_BOOTROM(ms))
			fprintf(os, ". BIM Status 0x%0x\n", status);
		else
			fprintf(os, ". DSU STATUSA 0x%0x\n", status);

		status = -1;
	} else {
		fprintf(os, "\bDone.\n");

		if (MC_DSU_HAS_BOOTROM(ms))
			status = mc_dsu_rom_exit_to_reset(ms, os);
		else
			status = TARGET_PI_OK_RESET;
	}

	return status;
}

void *
microchip_dsu_core_probe_did(target_t t, uint32_t *did)
{
	struct mcdsu_state *ms = &mcdsu_state;
	struct adiv5_memap_rom_table_lookup rt;

	rt.rt_pidr = MCP_DSUROM_PIDR;
	rt.rt_pidr_mask = MCP_DSUROM_MASK;

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP, &rt) == 0)
		return NULL;

	ms->ms_t = t;
	ms->ms_dsu = MCP_DSUROM_TO_DSUREG(rt.rt_paddr);

	if (mc_dsu_read32(ms, MCP_DSU_REG_DID, did) < 0)
		return NULL;

	return ms;
}

int
microchip_dsu_protection_status(void *arg, uint32_t soc_flags,
    microchip_dsu_protection_t *prot)
{
	struct mcdsu_state *ms = arg;
	uint8_t statusb;

	ms->ms_soc_flags = soc_flags;

	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSB, &statusb) < 0) {
		DBFPRINTF("Failed to read STATUSB\n");
		return -1;
	}

	/* XXX: Handle MCP_DSU_PROT_SECURE. */

	if (MC_DSU_HAS_BOOTROM(ms)) {
		switch (MCP_DSU_STATUSB_BR_DAL(statusb)) {
		case MCP_DSU_STATUSB_BR_DAL_0:
			*prot = MCP_DSU_PROT_LOCKED;
			break;
		case MCP_DSU_STATUSB_BR_DAL_1:
			*prot = MCP_DSU_PROT_PARTIAL;
			break;
		case MCP_DSU_STATUSB_BR_DAL_2:
		default:
			*prot = MCP_DSU_PROT_NONE;
			break;
		}
	} else {
		*prot = (statusb & MCP_DSU_STATUSB_PROT) ? MCP_DSU_PROT_LOCKED :
		    MCP_DSU_PROT_NONE;
	}

	DBFPRINTF("%u\n", (unsigned int)*prot);

	ms->ms_prot = *prot;

	return 0;
}

static const char mc_dsu_prot_query_text[] =
"Possible level values:\n"
"\t0  Device is unlocked.\n"
"\t1  Device is locked.\n"
"Target must be reset after changing level.\n";

static const char mc_dsu_prot_query_text_tz[] =
"Possible level values:\n"
"\t0  Device is unlocked.\n"
"\t1  Device is partially locked (non-secure access only).\n"
"\t2  Device is locked.\n"
"\t3  Device is irrevocably locked.\n"
"Target must be reset after changing level.\n";

static int
mc_dsu_prot_query(struct mcdsu_state *ms, struct target_protection_info *pi)
{

	switch (MCP_DSU_FLAGS_TYPE(ms->ms_soc_flags)) {
	case MCP_DSU_FLAGS_TYPE_M0:
	case MCP_DSU_FLAGS_TYPE_M4:
	onelevel:
		pi->pi_text = mc_dsu_prot_query_text;
		pi->pi_max_level = 1;
		pi->pi_level = (ms->ms_prot != MCP_DSU_PROT_NONE);
		break;

	case MCP_DSU_FLAGS_TYPE_ROM:
		if (!MC_DSU_TRUSTZONE(ms))
			goto onelevel;
		pi->pi_max_level = 3;
		pi->pi_text = mc_dsu_prot_query_text_tz;
		switch (ms->ms_prot) {
		default:
			pi->pi_level = 0;
			break;
		case MCP_DSU_PROT_PARTIAL:
			pi->pi_level = 1;
			break;
		case MCP_DSU_PROT_LOCKED:
			pi->pi_level = 2;
			break;
		case MCP_DSU_PROT_SECURE:
			pi->pi_level = 3;
			break;
		}
		break;

	default:
		return -1;
	}

	return 0;
}

static int
mc_dsu_unlock(struct mcdsu_state *ms, FILE *os)
{

	if (ms->ms_prot == MCP_DSU_PROT_NONE) {
		fprintf(os, "Device not locked.\n");
		return 0;
	}

	if (ms->ms_prot == MCP_DSU_PROT_SECURE) {
		fprintf(os, "Device is permanently locked.\n");
		return 0;
	}

	fprintf(os, "Unlock not yet supported.\n");

	/*
	 * XXX
	 *  - SAML10 is a straightforward ChipErase, no key.
	 *  - SAML11 requires a key (0xffffffff).
	 */

	return -1;
}

int
microchip_dsu_ctl(void *cookie, uint32_t *cmd, void *arg)
{
	struct mcdsu_state *ms = cookie;
	int rv = 0;

	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
		*cmd = TARGET_CTL_NOP;
		rv = mc_dsu_prot_query(ms, arg);
		break;

	case TARGET_CTL_UNLOCK:
		*cmd = TARGET_CTL_NOP;
		rv = mc_dsu_unlock(ms, arg);
		break;

	default:
		break;
	}

	return rv;
}

int
microchip_dsu_attach(target_t t, target_addr_t rom_table)
{
	const struct mcdsu_supported_soc *ss;
	bool dbgpres, protected, ce_request, manual_er;
	struct mcdsu_state *ms = &mcdsu_state;
	uint8_t statusa, statusb, v8;
	void *soc;
	uint32_t did;

	manual_er = adiv5_ll_extended_reset_configured();
	ce_request = ms->ms_chip_erase_requested;
	ms->ms_chip_erase_requested = false;

	ms->ms_t = t;
	ms->ms_dsu = MCP_DSUROM_TO_DSUREG(rom_table);
	DBFPRINTF("DSU registers at 0x%" PRIxTADDR "\n", ms->ms_dsu);

	/* Read DID. */
	if (mc_dsu_read32(ms, MCP_DSU_REG_DID, &did) < 0) {
		DBFPRINTF("Failed to read DID\n");
		return TARGET_ATTACH_NO_MATCH;
	}
	DBFPRINTF("DID is 0x%" PRIx32 "\n", did);

	/* Do we support this SoC? */
	soc = NULL;
	for (ss = mcdsu_supported_socs;
	    ss < &mcdsu_supported_socs[MCDSU_NSUPPORTED_SOCS]; ss++) {
		if ((soc = ss->ss_probe(did, &ms->ms_soc_flags)) != NULL)
			break;
	}

	if (soc == NULL) {
		DBFPRINTF("Unsupported SoC (DID 0x%08" PRIx32 ")\n", did);
		return TARGET_ATTACH_NO_MATCH;
	}

	/* Device is supported. */
	DBFPRINTF("DID matched with a supported device.\n");

	/* Read STATUSA and STATUSB. */
	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSA, &statusa) < 0) {
		DBFPRINTF("Failed to read STATUSA\n");
		return TARGET_ATTACH_NO_MATCH;
	}
	if (mc_dsu_read8(ms, MCP_DSU_REG_STATUSB, &statusb) < 0) {
		DBFPRINTF("Failed to read STATUSB\n");
		return TARGET_ATTACH_NO_MATCH;
	}

	DBFPRINTF("STATUSA %02x, STATUSB %02x\n", (unsigned int)statusa,
	    (unsigned int)statusb);

	if (MC_DSU_HAS_BOOTROM(ms)) {
		dbgpres = (statusb & MCP_DSU_STATUSB_BR_DBGPRES) != 0;
		protected = MCP_DSU_STATUSB_BR_DAL(statusb) ==
		    MCP_DSU_STATUSB_BR_DAL_0;
		v8 = statusa & ~MCP_DSU_STATUSA_BREXT;
	} else {
		dbgpres = (statusb & MCP_DSU_STATUSB_DBGPRES) != 0;
		protected = (statusb & MCP_DSU_STATUSB_PROT) != 0;
		v8 = statusa;

		/*
		 * If extended reset was requested manualy via
		 * 'rstcfg extended 1', then don't clear CRSTEXT;
		 * The user may be attempting to recover a "bricked"
		 * device.
		 */
		if (manual_er)
			v8 &= ~MCP_DSU_STATUSA_CRSTEXT;
	}

	/*
	 * We may have performed an extended reset...
	 */
	if ((statusa & MCP_DSU_STATUSA_CRSTEXT) == 0) {
		/*
		 * Extended reset not detected. If the device is
		 * unprotected, then don't match; the CPU core should
		 * attach in the normal way.
		 */
		if (protected == false) {
			DBFPRINTF("unprotected.\n");
			return TARGET_ATTACH_NO_MATCH;
		}

		/*
		 * Device is protected. We need to perform an extended
		 * reset sequence.
		 */
		if ((adiv5_ll_cfg_extra_set & ADIV5_LL_NRST_EXTENDED) != 0) {
			/*
			 * But we already tried one!
			 */
			DBFPRINTF("Extended reset failed!\n");
			return TARGET_ATTACH_NO_MATCH;
		}

		DBFPRINTF("Do extended reset\n");
		cortexm_hw_cfg_extra_set |= ADIV5_LL_NRST_EXTENDED;
		return TARGET_ATTACH_RESCAN;
	}

	/*
	 * Extended reset has been performed. Clear bits in STATUSA.
	 * However:
	 *  - if the device has a boot ROM, don't clear BREXT yet.
	 *  - if non-Boot ROM, don't clear CRSTEXT if manual extended reset.
	 * Status bits move around according to the device type, so
	 * figure them out here.
	 */
	DBFPRINTF("Extended reset performed. Write 0x%02x to STATUSA\n", v8);
	if (mc_dsu_write8(ms, MCP_DSU_REG_STATUSA, v8) < 0) {
		DBFPRINTF("Failed to clear STATUSA\n");
		return TARGET_ATTACH_NO_MATCH;
	}

	if (dbgpres == false) {
		/*
		 * There's something fishy about this DSU.
		 */
		DBFPRINTF("DBGPRES clear?!?!\n");
		return TARGET_ATTACH_NO_MATCH;
	}

	/*
	 * For unprotected devices without a boot ROM, clearing CRSTEXT has
	 * released the CPU so that it can run application code. We simply
	 * return TARGET_ATTACH_NO_MATCH so that the CPU core's ID-ROM code
	 * can be matched, as normal.
	 *
	 * For devices *with* a boot ROM, the CPU is now executing it.
	 * We must interact with the boot ROM to determine where to go
	 * from here.
	 */

	if (MC_DSU_HAS_BOOTROM(ms) == 0 && protected == false) {
		DBFPRINTF("Dumb and unprotected\n");
		if (!manual_er)
			return TARGET_ATTACH_NO_MATCH;

		DBFPRINTF("Manual ER requested. Will attach DSU only.\n");
	}

	/*
	 * The device has one or more of the following:
	 *  - a boot ROM,
	 *  - some level of protection enabled,
	 *  - a manual extended reset.
	 */

	if (MC_DSU_HAS_BOOTROM(ms)) {
		bool clear_brext;
		int rv;

		clear_brext = !ce_request && !manual_er;
		rv = mc_bootrom_interact(ms, statusb, clear_brext);
		if (rv != TARGET_ATTACH_OK)
			return rv;

		if (MC_DSU_TRUSTZONE(ms))
			ms->ms_chip_erase = mc_bootrom_erase_tz;
		else
			ms->ms_chip_erase = mc_bootrom_erase;
		ms->ms_chip_erase_status = mc_bootrom_erase_status;
	} else {
		DBFPRINTF("Dumb and protected, or manual ER\n");
		ms->ms_chip_erase = mc_dumb_erase;
		ms->ms_chip_erase_status = mc_dumb_erase_status;
	}

	/*
	 * We must match in order to facilitate erase, if required.
	 */

	/* Register the target info. */
	ms->ms_erase_time = ss->ss_register(did, soc);
	assert(ms->ms_erase_time > 0);

	target_register_name(NULL, NULL, NULL, protected ? "** LOCKED **" :
	    "** EXTRST **");

	t->t_core = ms;
	t->t_flags |= TARGET_FLAG_IS_LOCKED;
	t->t_ctl = mc_dsu_ctl;

	return TARGET_ATTACH_OK;
}

void
microchip_dsu_done(void *arg)
{
	struct mcdsu_state *ms = arg;

	ms->ms_t = NULL;
	ms->ms_chip_erase_requested = false;
}
