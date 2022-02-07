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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "cortexm.h"
#include "cortexm_regs.h"
#include "cortexm_txml.h"
#ifdef CONFIG_GDB_RTOS_SUPPORT
#include "cortexm_rtos.h"
#endif
#include "tmon.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "adiv5_ll.h"
#include "bitmap.h"
#include "timer.h"
#include "shell.h"
#include "stringio.h"
#include "zone_alloc.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "network_httpd.h"
#include "json_utils.h"
#endif
#endif /* CONFIG_USE_NETWORK */
#include "soc_lpc55s6x.h"
#include "soc_rp2040.h"
#include "soc_sam0.h"
#include "soc_sam34x.h"
#include "soc_samd5x.h"
#include "soc_samg5x.h"
#include "soc_samx7x.h"
#include "soc_stm32f0.h"
#include "soc_stm32f1.h"
#include "soc_stm32f4.h"
#include "soc_stm32f7.h"
#include "soc_stm32h7.h"

//#define	DEBUG_FLAG	cortexm_debug
//#define	DEBUG_FLAG_INIT	0
#include "debug.h"

SHELL_CMD_DECL(cortexm, cortexm_cmd, "Configure Cortex-M behaviour");
static unsigned int cortexm_cmd_count;

/*
 * ROM Table indices for the various components on an ARMv7m, as set in stone
 * in the ARMv7m documentation.
 */
#define	CORTEXM_RTE_SCS_IDX	0
#define	CORTEXM_RTE_DWT_IDX	1
#define	CORTEXM_RTE_FPB_IDX	2
#define	CORTEXM_RTE_ITM_IDX	3
#define	CORTEXM_RTE_TPIU_IDX	4	/* n/a for Cortex-M7 */
#define	CORTEXM_RTE_ETM_IDX	5	/* n/a for Cortex-M7 */

/*
 * This is the list of SoCs we support
 */
typedef int (*soc_attach_t)(target_t);
struct soc_attach {
	uint32_t sp_part;
	soc_attach_t sp_attach;
};
/*
 * List all supported SoCs here.
 *
 * The table is roughly sorted by how invasive the ID probe is likely to be.
 * SoCs near the start can identify themselves by checking for recognisable
 * ROM table before probing target memory.
 *
 * SoCs towards the end require more invasive probing.
 *
 * There's also a secondary "popularity" sort such that older devices appear
 * further down the list.
 */
static const struct soc_attach cortexm_v8m_socs[] = {
	/* Devices which use ROM Table DSU for identification */
	{CPUID_PARTNO_CM23,	soc_sam0_attach},

	{CPUID_PARTNO_CM33,	soc_lpc55s6x_attach},
};
#define	CORTEXM_V8M_NSOCS	\
	(sizeof(cortexm_v8m_socs) / sizeof(cortexm_v8m_socs[0]))

static const struct soc_attach cortexm_v7m_socs[] = {
	/* Devices which use ROM Table DSU for identification */
	{CPUID_PARTNO_CM4,	soc_samd5x_attach},
	{CPUID_PARTNO_CM7,	soc_samx7x_attach},

	/* Devices which use CHIP ID for identification */
	{CPUID_PARTNO_CM3,	soc_stm32f1_cm3_attach},
	{CPUID_PARTNO_CM4,	soc_stm32f4_attach},
	{CPUID_PARTNO_CM7,	soc_stm32f7_attach},

	{CPUID_PARTNO_CM7,	soc_stm32h7_attach},	/* Dual-core: CPU0 */
	{CPUID_PARTNO_CM4,	soc_stm32h7_attach},	/* Dual-core: CPU1 */

	{CPUID_PARTNO_CM4,	soc_sam4e_attach},
	{CPUID_PARTNO_CM4,	soc_sam4l_attach},
	{CPUID_PARTNO_CM4,	soc_sam4n_attach},
	{CPUID_PARTNO_CM4,	soc_sam4s_attach},
	{CPUID_PARTNO_CM4,	soc_samg5x_attach},
	{CPUID_PARTNO_CM3,	soc_sam3xa_attach},
	{CPUID_PARTNO_CM3,	soc_sam3n_attach},
	{CPUID_PARTNO_CM3,	soc_sam3s_attach},
	{CPUID_PARTNO_CM3,	soc_sam3u_attach},
};
#define	CORTEXM_V7M_NSOCS	\
	(sizeof(cortexm_v7m_socs) / sizeof(cortexm_v7m_socs[0]))

static const struct soc_attach cortexm_v6m_socs[] = {
	/* Devices which use ROM Table DSU for identification */
	{CPUID_PARTNO_CM0_PLUS,	soc_sam0_attach},

	{CPUID_PARTNO_CM0,	soc_stm32f0_attach},

	/* Other devices. */
	{CPUID_PARTNO_CM0_PLUS,	soc_rp2040_attach},
};
#define	CORTEXM_V6M_NSOCS	\
	(sizeof(cortexm_v6m_socs) / sizeof(cortexm_v6m_socs[0]))

#ifndef CONFIG_USE_CONFIGDB
#define	cortexm_vc	glob.glob_cortexm_vc
#define	cortexm_gflags	glob.glob_cortexm_flags
#else
static uint32_t cortexm_vc;
static uint32_t cortexm_gflags;
static bool cortexm_cf_done;
#endif
#define	CORTEXM_GFLAG_MASK_ISR_SHIFT		0
#define	CORTEXM_GFLAG_MASK_ISR_MASK		(3u << 0)
#define	CORTEXM_GFLAG_MASK_ISR_AUTO		(0u << 0)
#define	CORTEXM_GFLAG_MASK_ISR_ON		(1u << 0)
#define	CORTEXM_GFLAG_MASK_ISR_OFF		(2u << 0)
#define	CORTEXM_GFLAG_RESET_CFG_SHIFT		2
#define	CORTEXM_GFLAG_RESET_CFG_MASK		(3u << 2)
#define	CORTEXM_GFLAG_RESET_CFG_SRST		(0u << 2)
#define	CORTEXM_GFLAG_RESET_CFG_SYSRESETREQ	(1u << 2)
#define	CORTEXM_GFLAG_RESET_CFG_VECTRESET	(2u << 2)


static u_int cortexm_core_halt(cortexm_t);
static u_int cortexm_core_halt_state(cortexm_t);
static void cortexm_core_resume(cortexm_t, bool);
static void cortexm_core_detach(cortexm_t);
static void cortexm_sync_cache(cortexm_t, target_addr_t, uint32_t, bool);
static void cortexm_sw_bkpt_all_clear(cortexm_t, bool);
static void cortexm_sw_bkpt_all_set(cortexm_t);

static uint32_t
cortexm_scs_read(cortexm_t cm, uint32_t reg)
{
	target_addr_t a = (target_addr_t)(cm->cm_scs_base + reg);
	uint32_t rv;

	if (TARGET_REG_READ32(cm->cm_target, a, &rv) < 0)
		rv = 0;

	return rv;
}

static void
cortexm_scs_write(cortexm_t cm, uint32_t reg, uint32_t v)
{
	target_addr_t a = (target_addr_t)(cm->cm_scs_base + reg);

	TARGET_REG_WRITE32(cm->cm_target, a, v);
}

static uint32_t
cortexm_fpb_read(cortexm_t cm, uint32_t reg)
{
	target_addr_t a;
	uint32_t rv;

	if (cm->cm_fpb_base == 0) {
		DBFPRINTF("WARNING! No FPB\n");
		return 0;
	}

	a = (target_addr_t)(cm->cm_fpb_base + reg);
	if (TARGET_REG_READ32(cm->cm_target, a, &rv) < 0)
		rv = 0;

	return rv;
}

static void
cortexm_fpb_write(cortexm_t cm, uint32_t reg, uint32_t v)
{

	if (cm->cm_fpb_base) {
		target_addr_t a = (target_addr_t)(cm->cm_fpb_base + reg);
		TARGET_REG_WRITE32(cm->cm_target, a, v);
	} else {
		DBFPRINTF("WARNING! No FPB\n");
	}
}

static uint32_t
cortexm_dwt_read(cortexm_t cm, uint32_t reg)
{
	target_addr_t a;
	uint32_t rv;

	if ((cm->cm_demcr & CMSCS_DEMCR_TRCENA) == 0) {
		DBFPRINTF("WARNING! No DWT\n");
		return 0;
	}

	a = (target_addr_t)(cm->cm_dwt_base + reg);
	if (TARGET_REG_READ32(cm->cm_target, a, &rv) < 0)
		rv = 0;

	return rv;
}

static void
cortexm_dwt_write(cortexm_t cm, uint32_t reg, uint32_t v)
{
	target_addr_t a;

	if ((cm->cm_demcr & CMSCS_DEMCR_TRCENA) == 0) {
		DBFPRINTF("WARNING! No DWT\n");
		return;
	}

	a = (target_addr_t)(cm->cm_dwt_base + reg);
	TARGET_REG_WRITE32(cm->cm_target, a, v);
}

static __always_inline uint32_t
cortexm_dhcsr_read(cortexm_t cm)
{

	return cortexm_scs_read(cm, CMSCS_REG_DHCSR);
}

static __always_inline void
cortexm_dhcsr_write(cortexm_t cm, uint32_t v)
{

	v &= 0x0000ffffu;
	v |= CMSCS_DHCSR_DBGKEY;
	cortexm_scs_write(cm, CMSCS_REG_DHCSR, v);
}

uint32_t
cortexm_core_reg_read(cortexm_t cm, u_int reg)
{
	timer_timeout_t to;
	uint32_t value;

	/*
	 * Special case pseudo register on armv8m ...
	 */
	if (reg == CMSCW_DCRSR_PSEUDO_REG_SECURE) {
		value = cortexm_scs_read(cm, CMSCS_REG_DSCSR);
		return (value & CMSCS_DSCSR_CDS) ? 1 : 0;
	}

	cortexm_scs_write(cm, CMSCS_REG_DCRSR,
	    CMSCS_DCRSR_REGSEL_RD(reg));

	timer_timeout_start(&to, 100);
	while ((cortexm_dhcsr_read(cm) & CMSCS_DHCSR_S_REGRDY) == 0 &&
	    !timer_timeout_expired(&to)) {
	}

	if (timer_timeout_expired(&to)) {
		DBFPRINTF("failed to read reg %u", reg);
		return 0;
	}

	value = cortexm_scs_read(cm, CMSCS_REG_DCRDR);

	DBFPRINTF("reg %u == 0x%08" PRIx32 "\n", reg, value);

	return value;
}

void
cortexm_core_reg_write(cortexm_t cm, u_int reg, uint32_t v)
{
	timer_timeout_t to;

	DBFPRINTF("0x%08" PRIx32 " -> reg %u\n", v, reg);

	/*
	 * Special case secure state register
	 */
	if (reg == CMSCW_DCRSR_PSEUDO_REG_SECURE) {
		uint32_t dscsr = cortexm_scs_read(cm, CMSCS_REG_DSCSR);
		if (((dscsr & CMSCS_DSCSR_CDS) ? 1u : 0u) != v) {
			dscsr &= ~(CMSCS_DSCSR_CDS | CMSCS_DSCSR_CDSKEY);
			if (v)
				dscsr |= CMSCS_DSCSR_CDS;
			cortexm_scs_write(cm, CMSCS_REG_DSCSR, dscsr);
		}
		return;
	}

	if (reg == CMSCW_DCRSR_REG_PC) {
		if ((v & 1u) == 0) {
			DBFPRINTF("Fixing bit #0\n");
			v |= 1u;
		}
	}

	cortexm_scs_write(cm, CMSCS_REG_DCRDR, v);
	cortexm_scs_write(cm, CMSCS_REG_DCRSR,
	   CMSCS_DCRSR_REGSEL_WR(reg));

	timer_timeout_start(&to, 100);
	while ((cortexm_dhcsr_read(cm) & CMSCS_DHCSR_S_REGRDY) == 0 &&
	    !timer_timeout_expired(&to)) {
	}

	if (timer_timeout_expired(&to))
		DBFPRINTF("failed to write reg %u\n", reg);
}

static void
cortexm_free(target_t t)
{
	cortexm_t cm = t->t_core;

	DBFPRINTF("tidying up\n");

	if (--cortexm_cmd_count == 0)
		SHELL_CMD_DEL(cortexm);

	cortexm_core_detach(cm);

	if (cm->cm_soc_free)
		(cm->cm_soc_free)(cm);

	zone_free(cm);
}

static int
cortexm_dwt_func_match(cortexm_t cm, unsigned int i,
    struct target_watchbreak *tw)
{
	uint32_t f;

	if ((cm->cm_demcr & CMSCS_DEMCR_TRCENA) == 0 ||
	    bitmap_get_state(cm->cm_hw_watchpoints, i) == false) {
		DBFPRINTF("watch %u not set\n", i);
		return 0;
	}

	f = cortexm_dwt_read(cm, CMDWT_REG_FUNCTION(i));

	(void) TARGET_LINK_ERROR(cm->cm_target);

	if ((f & CMDWT_FUNC_MATCHED) != 0) {
		DBFPRINTF("watch %u found\n", i);
		if (tw) {
			uint32_t t;

			tw->tw_addr = cortexm_dwt_read(cm, CMDWT_REG_COMP(i));
			DBFPRINTF("watch addr %08" PRIxTADDR "\n", tw->tw_addr);

			if (CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V8M) {
				switch (f & CMDWT_FUNC_ACTION_MASK_V8) {
				case CMDWT_FUNC_ACTION_READ_V8:
					DBFPRINTF("READ\n");
					t = TARGET_BKWP_TYPE_READ_WATCH;
					break;
				case CMDWT_FUNC_ACTION_WRITE_V8:
					DBFPRINTF("WRITE\n");
					t = TARGET_BKWP_TYPE_WRITE_WATCH;
					break;
				default:
					DBFPRINTF("ACCESS\n");
					t = TARGET_BKWP_TYPE_ACCESS_WATCH;
					break;
				}

				/* XXX: Ditch the magic constant. */
				tw->tw_kind = 2;
			} else {
				switch (f & CMDWT_FUNC_ACTION_MASK) {
				case CMDWT_FUNC_ACTION_READ:
					DBFPRINTF("READ\n");
					t = TARGET_BKWP_TYPE_READ_WATCH;
					break;
				case CMDWT_FUNC_ACTION_WRITE:
					DBFPRINTF("WRITE\n");
					t = TARGET_BKWP_TYPE_WRITE_WATCH;
					break;
				default:
					DBFPRINTF("ACCESS\n");
					t = TARGET_BKWP_TYPE_ACCESS_WATCH;
					break;
				}

				tw->tw_kind = cortexm_dwt_read(cm,
				    CMDWT_REG_MASK(i)) & CMDWT_MASK_MASK;
				tw->tw_kind += 1;
			}

			tw->tw_type = t;
			tw->tw_set = 0;
		}
		return 1;
	}

	DBFPRINTF("watch %u not found\n", i);
	return 0;
}

static int
cortexm_core_reset_generic(cortexm_t cm, int and_halt)
{
	uint32_t aircr, demcr, dhcsr, err;
	timer_timeout_t to;

	demcr = cm->cm_demcr & ~CMSCS_DEMCR_VC_CORERESET;
	dhcsr = 0;

	if (and_halt) {
		dhcsr = CMSCS_DHCSR_C_DEBUGEN;
		demcr |= CMSCS_DEMCR_VC_CORERESET;
	}

	DBFPRINTF("Original DEMCR %08" PRIx32 ", DHCSR %08" PRIx32 "\n",
	    cortexm_scs_read(cm, CMSCS_REG_DEMCR), cortexm_dhcsr_read(cm));
	DBFPRINTF("Writing DEMCR %08" PRIx32 ", DHCSR %08" PRIx32 "\n",
	    demcr, dhcsr);

	cortexm_scs_write(cm, CMSCS_REG_DEMCR, demcr);
	(void) cortexm_dhcsr_read(cm);
	cortexm_dhcsr_write(cm, dhcsr);

	cortexm_scs_write(cm, CMSCS_REG_AIRCR, CMSCS_AIRCR_SYSRESETREQ);

	DBFPRINTF("Waiting for CMSCS_AIRCR_SYSRESETREQ to clear\n");
	timer_timeout_start(&to, 2000);
	do {
		aircr = cortexm_scs_read(cm, CMSCS_REG_AIRCR);
		if ((err = TARGET_LINK_ERROR(cm->cm_target)) != 0)
			aircr = CMSCS_AIRCR_SYSRESETREQ;
		rtos_task_sleep(10);
	} while ((aircr & CMSCS_AIRCR_SYSRESETREQ) != 0 &&
	    !timer_timeout_expired(&to));

	if (timer_timeout_expired(&to))
		DBFPRINTF("AIRCR_SYSRESETREQ timeout (%08" PRIx32 ")\n", err);

	/*
	 * Wait for the first read with S_RESET_ST set.
	 */
	DBFPRINTF("Waiting for CMSCS_DHCSR_S_RESET to set\n");
	rtos_task_sleep(50);
	timer_timeout_start(&to, 2000);
	do {
		dhcsr = cortexm_dhcsr_read(cm);
		if ((err = TARGET_LINK_ERROR(cm->cm_target)) != 0)
			dhcsr = 0;
		rtos_task_sleep(10);
	} while (!timer_timeout_expired(&to) &&
	    (dhcsr & CMSCS_DHCSR_S_RESET) == 0);

	if (timer_timeout_expired(&to)) {
		DBFPRINTF("S_RESET_ST timeout (halt %d, err %08" PRIx32
		    " DHCSR %08" PRIx32 ").\n", and_halt, err, dhcsr);
		return -1;
	}

	/*
	 * S_RESET_ST should be clear on subsequent reads.
	 */
	rtos_task_sleep(100);
	timer_timeout_start(&to, 2000);
	do {
		dhcsr = cortexm_dhcsr_read(cm);
		if ((err = TARGET_LINK_ERROR(cm->cm_target)) != 0)
			dhcsr = CMSCS_DHCSR_S_RESET;
		rtos_task_sleep(10);
	} while (!timer_timeout_expired(&to) &&
	    (dhcsr & CMSCS_DHCSR_S_RESET) != 0);

	if (timer_timeout_expired(&to)) {
		DBFPRINTF("Warning Second S_RESET_ST is set (halt %d)."
		    "\n", and_halt);
	}

	return 0;
}

static int
cortexm_core_reset(cortexm_t cm, int and_halt)
{
	int rv;

	DBFPRINTF("halt %d\n", and_halt);

	if ((cm->cm_flags & CORTEXM_FLAG_NO_RESET) != 0) {
		DBFPRINTF("Core reset not permitted.\n");
		return -1;
	}

	if ((rv = cm->cm_core_reset(cm, and_halt)) < 0)
		return rv;

	DBFPRINTF("Reset complete. Restoring DEMCR %08" PRIx32 "\n",
	    cm->cm_demcr);

	cortexm_scs_write(cm, CMSCS_REG_DEMCR, cm->cm_demcr);

	if (and_halt) {
		DBFPRINTF("DFSR 0x%08" PRIx32 ", DHCSR 0x%08" PRIx32 "\n",
		    cortexm_scs_read(cm, CMSCS_REG_DFSR),
		    cortexm_dhcsr_read(cm));
	}

	(void) TARGET_LINK_ERROR(cm->cm_target);

	cm->cm_mpu_ctrl_save = 0;
	cm->cm_last_halt_reason = TARGET_HALT_STATE_RUNNING;

	return 0;
}

static void
cortexm_core_fpb_init(cortexm_t cm)
{
	unsigned int i;
	uint32_t reg;

	if (cm->cm_fpb_base == 0)
		return;

	bitmap_init(cm->cm_hw_breakpoints, CORTEXM_MAX_HW_BREAKPOINTS);

	reg = cortexm_fpb_read(cm, CMFPB_REG_FP_CTRL);

	DBFPRINTF("FP_CTRL %08" PRIx32 "\n", reg);

	if (CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V7M ||
	    CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V8M) {
		DBFPRINTF("FP_REMAP %08" PRIx32 "\n",
		    cortexm_fpb_read(cm, CMFPB_REG_FP_REMAP));

		if (CMFPB_FP_CTRL_REV(reg) != 0) {
			DBFPRINTF("V2 FPB\n");
			cm->cm_flags |= CORTEXM_FLAG_COMP_V2;
		} else {
			DBFPRINTF("V1 FPB\n");
		}

		cm->cm_num_hw_breakpoints = CMFPB_FP_CTRL_NUM_CODE(reg);
	} else {
		cm->cm_num_hw_breakpoints = CMFPB_FP_CTRL_NUM_CODEv6(reg);
	}

	/*
	 * On a debug/development build, it's useful to know if a target
	 * exceeds our configured maximum number of breakpoints.
	 */
	assert(cm->cm_num_hw_breakpoints < CORTEXM_MAX_HW_BREAKPOINTS);

	/*
	 * Otherwise, clamp to our supported maximum.
	 */
	if (cm->cm_num_hw_breakpoints > CORTEXM_MAX_HW_BREAKPOINTS)
		cm->cm_num_hw_breakpoints = CORTEXM_MAX_HW_BREAKPOINTS;

	for (i = 0; i < cm->cm_num_hw_breakpoints; i++)
		cortexm_fpb_write(cm, CMFPB_REG_FP_COMP(i), 0);

	if (cm->cm_num_hw_breakpoints) {
		reg = cortexm_fpb_read(cm, CMFPB_REG_FP_CTRL);
		reg |= CMFPB_FP_CTRL_ENABLE | CMFPB_FP_CTRL_KEY;
		cortexm_fpb_write(cm, CMFPB_REG_FP_CTRL, reg);
	}

	if (TARGET_LINK_ERROR(cm->cm_target) != 0)
		DBFPRINTF("DP error flagged (3)\n");
}

static void
cortexm_core_dwt_init(cortexm_t cm)
{
	unsigned int i;
	uint32_t reg;

	if (cm->cm_dwt_base == 0)
		return;

	bitmap_init(cm->cm_hw_watchpoints, CORTEXM_MAX_WATCHPOINTS);

	reg = cortexm_dwt_read(cm, CMDWT_REG_CTRL);

	DBFPRINTF("CTRL %08" PRIx32 "\n", reg);

	cm->cm_num_hw_watchpoints = CMDWT_CTRL_NUMCOMP(reg);

	/*
	 * On a development build, I want to know if some future target
	 * supports more than our configured number of watchpoints.
	 */
	assert(cm->cm_num_hw_watchpoints < CORTEXM_MAX_WATCHPOINTS);

	/* Othersise, clamp to our configure maximum. */
	if (cm->cm_num_hw_watchpoints > CORTEXM_MAX_WATCHPOINTS)
		cm->cm_num_hw_watchpoints = CORTEXM_MAX_WATCHPOINTS;
	else
	if (cm->cm_num_hw_watchpoints == 0 &&
	    CMSCS_CPUID_PARTNO(cm->cm_cpuid) == CPUID_PARTNO_CM7) {
		/*
		 * XXX:
		 * CMDWT_REG_CTRL.NUMCOMP is zero on some Cortex-M7 devices
		 * following power-up. They need a core reset to function
		 * before the NUMCOMP field is valid. Since the vast majority
		 * of these have 4 watchpoints, we'll fake it here so at least
		 * something is reported.
		 */
		cm->cm_num_hw_watchpoints = 4;
	}

	for (i = 0; i < cm->cm_num_hw_watchpoints; i++)
		cortexm_dwt_write(cm, CMDWT_REG_FUNCTION(i), 0);

	if (TARGET_LINK_ERROR(cm->cm_target) != 0)
		DBFPRINTF("DP error flagged (4)\n");
}

static u_int
cortexm_core_attach(cortexm_t cm, int do_reset)
{
	timer_timeout_t to;
	uint32_t reg;
	u_int halt;

	DBFPRINTF("do_reset=%d: attaching\n", do_reset);

	if ((cm->cm_flags & CORTEXM_FLAG_ATTACHED) != 0) {
		DBFPRINTF("already attached\n");
		return 1;
	}

	if ((cm->cm_flags & CORTEXM_FLAG_NO_RESET) != 0) {
		DBFPRINTF("Reset not permitted.\n");
		do_reset = 0;
	}

	if (cm->cm_last_halt_reason == TARGET_HALT_STATE_RUNNING) {
		timer_timeout_start(&to, 5000);

		DBFPRINTF("core is currently running\n");

		if (do_reset) {
			if (cortexm_core_reset(cm, 1) < 0)
				return 0;

			cm->cm_last_halt_reason = TARGET_HALT_STATE_FORCED;
		}

		DBFPRINTF("waiting for core to halt\n");

		while ((halt = cortexm_core_halt(cm)) ==
		    TARGET_HALT_STATE_RUNNING && !timer_timeout_expired(&to) &&
		    adiv5_ll_nRST_state() == 0) {
			rtos_task_sleep(10);
		}

		if (timer_timeout_expired(&to)) {
			DBFPRINTF("attach timeout\n");
			return 0;
		}

		if (TARGET_LINK_ERROR(cm->cm_target) != 0) {
			DBFPRINTF("DP error flagged\n");
			return 0;
		}
	} else {
		DBFPRINTF("core is already halted\n");

		halt = cortexm_core_halt_state(cm);
	}

#if 0
	DBFPRINTF("target stopped @ %08" PRIx32 ". Reason %u\n",
	    cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_PC), halt);
#endif

	cortexm_scs_write(cm, CMSCS_REG_DEMCR, cm->cm_demcr);
	if (TARGET_LINK_ERROR(cm->cm_target) != 0) {
		DBFPRINTF("DP error flagged 2 (demcr %08" PRIx32 ")\n",
		    cm->cm_demcr);
		return 0;
	}

	reg = cortexm_scs_read(cm, CMSCS_REG_DFSR);
	cortexm_scs_write(cm, CMSCS_REG_DFSR, reg);

	if (TARGET_LINK_ERROR(cm->cm_target) != 0) {
		DBFPRINTF("DP error flagged 3 (reg %08" PRIx32 ", demcr "
		    "%08" PRIx32 ")\n", reg, cm->cm_demcr);
		return 0;
	}

	DBFPRINTF("DFSR %08" PRIx32 "\n", reg);

	/*
	 * Initialise hardware breakpoint state, if we have an FPB
	 */
	if (cm->cm_fpb_base != 0)
		cortexm_core_fpb_init(cm);

	/*
	 * Likewise for hardware watchpoints if we have a DWT.
	 */
	if (cm->cm_dwt_base != 0)
		cortexm_core_dwt_init(cm);

	/* Finally, software breakpoints */
	bitmap_init(cm->cm_sw_breakpoints, CORTEXM_MAX_SW_BREAKPOINTS);

	cm->cm_flags |= CORTEXM_FLAG_ATTACHED;

	DBFPRINTF("breakpoints %u, watchpoints %u\n", cm->cm_num_hw_breakpoints,
	    cm->cm_num_hw_watchpoints);

	return 1;
}

static void
cortexm_core_detach(cortexm_t cm)
{
	uint32_t dhcsr;
	unsigned int i;

	/*
	 * Do nothing if we were not attached in the first place.
	 */
	if ((cm->cm_flags & CORTEXM_FLAG_ATTACHED) == 0)
		return;

#ifdef CONFIG_GDB_RTOS_SUPPORT
	/* Ensure RTOS state is clear. */
	cortexm_rtos_set_frame(cm, NULL);
#endif

	/*
	 * Clear all h/w breakpoints.
	 */
	if (cm->cm_num_hw_breakpoints) {
		for (i = 0; i < cm->cm_num_hw_breakpoints; i++) {
			cortexm_fpb_write(cm, CMFPB_REG_FP_COMP(i), 0);
			bitmap_set_state(cm->cm_hw_breakpoints, i, false);
		}

		cortexm_fpb_write(cm, CMFPB_REG_FP_CTRL, CMFPB_FP_CTRL_KEY);
	}

	/*
	 * Clear all h/w watchpoints.
	 */
	for (i = 0; i < cm->cm_num_hw_watchpoints; i++) {
		cortexm_dwt_write(cm, CMDWT_REG_FUNCTION(i), 0);
		bitmap_set_state(cm->cm_hw_watchpoints, i, false);
	}

	/*
	 * Clear all s/w breakpoints.
	 */
	cortexm_sw_bkpt_all_clear(cm, true);

	/*
	 * According to the ARM Book of Words:
	 *
	 * The effect of a write to DHCSR is unpredictable if any of:
	 *  o The write changes DHCSR.C_MASKINTS and either
	 *     - Before the write, DHCSR.C_HALT is 0.
	 *     - The write changes DHCSR.C_HALT from 1 to 0.
	 *    Unless both:
	 *     - Before the write, DHCSR.C_DEBUGEN is 0.
	 *     - The write sets DHCSR.C_MASKINTS to 0.
	 *  o The write changes DHCHCSR.C_DEBUGEN from 0 to 1 and sets
	 *    DHCSR.C_MASKINTS to 1.
	 *
	 * Obviously that final condition is not the case - we're exiting
	 * debug mode, so C_DEBUGEN will be 1.
	 *
	 * The first condition does apply though. C_HALT is almost certainly
	 * non-zero and, if we've been single-stepping, C_MASKINTS will also
	 * be non-zero. Therefore we cannot simply clear DHCSR to zero without
	 * first clearing C_MASKINTS.
	 */
	dhcsr = cortexm_dhcsr_read(cm);

	DBFPRINTF("DHCSR %08" PRIx32 "\n", dhcsr);

	/* A quick double-check that debug is indeed enabled. */
	if (dhcsr & CMSCS_DHCSR_C_DEBUGEN) {
		if (dhcsr & CMSCS_DHCSR_C_MASKINTS) {
			/* Need to clear C_MASKINTS first. */
			dhcsr &= ~CMSCS_DHCSR_C_MASKINTS;
			cortexm_dhcsr_write(cm, dhcsr);
		}
		dhcsr &= ~(CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_STEP);
		cortexm_dhcsr_write(cm, dhcsr);
		cortexm_dhcsr_write(cm, 0);

		(void) TARGET_LINK_ERROR(cm->cm_target);
	}

	cm->cm_flags &= ~(CORTEXM_FLAG_ATTACHED | CORTEXM_FLAG_RESUME_CALLED);
	cm->cm_last_halt_reason = TARGET_HALT_STATE_RUNNING;
}

static bool
cortexm_fault_can_recover(uint32_t hfsr, uint32_t cfsr)
{

	/*
	 * This function returns true if valid context has been saved to the
	 * stack following a fault so that we can recover the true CPU state
	 * at the point of the fault.
	 */

	/* Hardfault on vector read error. */
	if (hfsr & CMSCS_HFSR_VECTTBL)
		return true;

	/* Hardfault on breakpoint escalation. We should never see this. */
	if (hfsr & CMSCS_HFSR_DEBUGEVT)
		return true;

	/* Hardfault on fault escalation, with no other faults logged. */
	if ((hfsr & CMSCS_HFSR_FORCED) != 0 &&
	    (cfsr & CMSCS_CFSR_FAULT_MASK) == 0) {
		return true;
	}

	/*
	 * For BFSR.STKERR and MMFSR.MSTKERR, there's no saved state.
	 */
	if ((cfsr & (CMSCS_BFSR_STKERR | CMSCS_MMFSR_MSTKERR)) != 0)
		return false;

	/* Bus fault on exception return is unclear, but assume true. */
	if ((cfsr & (CMSCS_BFSR_UNSTKERR | CMSCS_MMFSR_MUNSTKERR)) != 0)
		return true;

	/* All other faults will save state. */
	return (cfsr & CMSCS_CFSR_FAULT_MASK) != 0;
}

static bool
cortexm_fault_recover(cortexm_t cm, uint32_t *pc)
{
	uint32_t hfsr, cfsr;
	uint32_t lr, sp, v;
	uint32_t regs[3], cfbp;
	struct target_mem_readwrite mr;
	u_int ssp;

	/*
	 * We intercept a couple of faults (for things like breakpoints) but
	 * we want them to be invisible to the target software. Thus we need
	 * to extract the relevant fault information and restore the state
	 * saved by the CPU before it took the fault.
	 *
	 * Not all faults have recoverable state
	 *
	 * Note:
	 * This function is not responsible for checking that we must handle
	 * a fault; that decision has already been made since DEMCR reflects
	 * the vector catch configuration. So if we're here, then the fault
	 * is being handled.
	 */

	if (cm->cm_features == CORTEXM_FEATURE_MAINLINE) {
		hfsr = cortexm_scs_read(cm, CMSCS_REG_HFSR);
		cfsr = cortexm_scs_read(cm, CMSCS_REG_CFSR);

		if (hfsr != 0)
			cortexm_scs_write(cm, CMSCS_REG_HFSR, hfsr);
		if (cfsr != 0)
			cortexm_scs_write(cm, CMSCS_REG_CFSR, cfsr);

		DBFPRINTF("HFSR %08" PRIx32 ", CFSR %08" PRIx32 "\n", hfsr, cfsr);

		if (TARGET_LINK_ERROR(cm->cm_target) != 0 ||
		    !cortexm_fault_can_recover(hfsr, cfsr)) {
			/* Don't attempt recovery for this fault. */
			DBFPRINTF("No state saved.\n");
			return false;
		}
	} else {
		/*
		 * HFSR and CFSR are not implemented on baseline devices.
		 * We assume, rightly or wrongly, that all faults can be
		 * recovered.
		 */
	}

	/*
	 * The base of the stack looks like this:
	 *
	 *  SP[0]:	R0
	 *  SP[4]:	R1
	 *  SP[8]:	R2
	 *  SP[12]:	R3
	 *  SP[16]:	R12
	 *  SP[20]:	LR	Corresponds to regs[0]
	 *  SP[24]:	PC	Corresponds to regs[1]
	 *  SP[28]:	xPSR	Corresponds to regs[2]
	 *  SP[32...]	FP registers, if present.
	 *
	 * Since nothing has touched R0-R12 we don't need to restore them.
	 */

#define	CORTEXM_FS_STACK_PSP(lr)	(((lr) & 0x04u) != 0)
#define	CORTEXM_FS_STACK_FPU(lr)	(((lr) & 0x10u) != 0)

	/* Key to this is the "magic" exception status in LR */
	lr = cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_LR);
	DBFPRINTF("LR %08" PRIx32 "\n", lr);

	/* Fetch the appropriate stack pointer */
	ssp = CORTEXM_FS_STACK_PSP(lr) ? CMSCW_DCRSR_REG_PSP :
	    CMSCW_DCRSR_REG_MSP;
	sp = cortexm_core_reg_read(cm, ssp);
	DBFPRINTF("State on %cSP @ %08" PRIx32 "\n",
	    CORTEXM_FS_STACK_PSP(lr) ? 'P' : 'M', sp);

	/*
	 * Read in the part of the saved stack frame containing the
	 * registers we need to restore.
	 */
	mr.mr_write = false;
	mr.mr_target_addr = (target_addr_t)(sp + 20);
	mr.mr_length = sizeof(regs);
	mr.mr_dest = regs;
	if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE, &mr) < 0) {
		DBFPRINTF("failed to read frame\n");
		return false;
	}

	DBFPRINTF("LR %08" PRIx32 ", PC %08" PRIx32 ", xPSR %08" PRIx32
	    "(%08" PRIx32 ")\n", regs[0], regs[1], regs[2],
	    cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_xPSR));

	/* Check xPSR to determine if stack alignment was applied */
	v = (regs[2] & (1u << 9)) ? 4 : 0;

	/* Stack frame size varies according to whether FP state was saved */
	v += CORTEXM_FS_STACK_FPU(lr) ? 0x20u : 0x68u;
	sp += v;

	DBFPRINTF("SP adjusted to %08" PRIx32 "\n", sp);

	/* We may need to modify CONTROL to fully restore state */
	cfbp = cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_CFBP);

	/* Adjust PSP or MSP as required */
	cortexm_core_reg_write(cm, ssp, sp);

	/* Restore the saved LR, PC and xPSR */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_LR, regs[0]);
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_PC, regs[1]);
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_xPSR, regs[2]);

	if (CORTEXM_FS_STACK_PSP(lr))
		cfbp |= (1u << 25);	/* Set SPSEL */
	if (CORTEXM_FS_STACK_FPU(lr))
		cfbp |= (1u << 26);	/* Set FPCA */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_CFBP, cfbp);

	/* Clear the exception */
	cortexm_scs_write(cm, CMSCS_REG_AIRCR, CMSCS_AIRCR_VECTCLRACTIVE);

	if (pc)
		*pc = regs[1];

	return TARGET_LINK_ERROR(cm->cm_target) == 0;
}

static uint32_t
cortexm_hw_bkpt_get_addr(cortexm_t cm, unsigned int n)
{
	uint32_t comp;

	comp = cortexm_fpb_read(cm, CMFPB_REG_FP_COMP(n));

	if (TARGET_LINK_ERROR(cm->cm_target) != 0)
		return 0xfffffffful;

	if ((cm->cm_flags & CORTEXM_FLAG_COMP_V2) == 0) {
		uint32_t bit1;
		bit1 = (CMFPB_FP_COMP_REPLACE(comp) ==
		    CMFPB_FP_COMP_COMP1) ? 2 : 0;
		comp = CMFPB_FP_COMP_ADDR(comp) | bit1;
	}

	return comp & ~1u;
}

static bool
cortexm_hw_bkpt_compare(cortexm_t cm, unsigned int n, uint32_t addr)
{
	uint32_t comp;

	comp = cortexm_hw_bkpt_get_addr(cm, n);
	if (comp == 0xfffffffful)
		return false;

	DBPRINTF(" %08" PRIx32 " == %08" PRIx32 ", ", addr, comp);

	return addr == comp;
}

static int
cortexm_hw_bkpt_find(cortexm_t cm, uint32_t addr)
{
	unsigned int bit;

	DBFPRINTF("%08" PRIx32 ": ", addr);

	addr &= ~1u;
	bit = 0;
	while ((bit = bitmap_find_first(cm->cm_hw_breakpoints,
	    cm->cm_num_hw_breakpoints, bit, true)) != 0) {
		/* Check if breakpoint exists */
		if (cortexm_hw_bkpt_compare(cm, bit - 1, addr))
			break;
	}

	if (bit == 0)
		DBPRINTF("no match\n");
	else
		DBPRINTF("found at idx %u\n", bit - 1);

	return (int)bit - 1;
}

static bool
cortexm_hw_bkpt_set(cortexm_t cm, unsigned int n, uint32_t addr)
{
	uint32_t comp;

	addr &= ~1u;

	if ((cm->cm_flags & CORTEXM_FLAG_COMP_V2) == 0) {
		if ((addr & ~2u) & ~CMFPB_FP_COMP_ADDR_MASK)
			return false;

		if (addr & 2u) {
			comp = CMFPB_FP_COMP_COMP1 <<
			    CMFPB_FP_COMP_REPLACESHIFT;
		} else {
			comp = CMFPB_FP_COMP_COMP0 <<
			    CMFPB_FP_COMP_REPLACESHIFT;
		}

		addr &= CMFPB_FP_COMP_ADDR_MASK;
		addr |= comp;
	}

	comp = addr | CMFPB_FP_COMP_ENABLE;

	DBFPRINTF("HARD BKPT %u @ %08" PRIx32 "\n", n,
	    comp);

	cortexm_fpb_write(cm, CMFPB_REG_FP_COMP(n), comp);
	if (TARGET_LINK_ERROR(cm->cm_target) != 0) {
		DBFPRINTF("DP error writing FP_COMP(%u)\n", n);
		return false;
	}

	bitmap_set_state(cm->cm_hw_breakpoints, n, true);
	return true;
}

static int
cortexm_wapt_find(cortexm_t cm, const struct target_watchbreak *tw)
{
	unsigned int bit;
	uint32_t addr;

	addr = (uint32_t)tw->tw_addr;
	bit = 0;
	while ((bit = bitmap_find_first(cm->cm_hw_watchpoints,
	    cm->cm_num_hw_watchpoints, bit, true)) != 0) {
		if (cortexm_dwt_read(cm, CMDWT_REG_COMP(bit - 1)) == addr)
			break;
	}

	return (int)bit - 1;
}

static int
cortexm_wapt_set(cortexm_t cm, const struct target_watchbreak *tw,
    unsigned int n)
{
	uint32_t action;

	DBFPRINTF("WAPT %u @ %08" PRIxTADDR "\n", n, tw->tw_addr);

	if (CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V8M) {
		switch (tw->tw_type) {
		case TARGET_BKWP_TYPE_WRITE_WATCH:
			action = CMDWT_FUNC_ACTION_WRITE_V8;
			break;
		case TARGET_BKWP_TYPE_READ_WATCH:
			action = CMDWT_FUNC_ACTION_READ_V8;
			break;
		case TARGET_BKWP_TYPE_ACCESS_WATCH:
			action = CMDWT_FUNC_ACTION_ACCESS_V8;
			break;
		default:
			return -1;
		}
	} else {
		switch (tw->tw_type) {
		case TARGET_BKWP_TYPE_WRITE_WATCH:
			action = CMDWT_FUNC_ACTION_WRITE;
			break;
		case TARGET_BKWP_TYPE_READ_WATCH:
			action = CMDWT_FUNC_ACTION_READ;
			break;
		case TARGET_BKWP_TYPE_ACCESS_WATCH:
			action = CMDWT_FUNC_ACTION_ACCESS;
			break;
		default:
			return -1;
		}

		cortexm_dwt_write(cm, CMDWT_REG_MASK(n), tw->tw_kind - 1);
	}

	cortexm_dwt_write(cm, CMDWT_REG_COMP(n), tw->tw_addr);
	cortexm_dwt_write(cm, CMDWT_REG_FUNCTION(n), action);
	bitmap_set_state(cm->cm_hw_watchpoints, n, true);

	return TARGET_LINK_ERROR(cm->cm_target) ? -1 : 0;
}

static bool
cortexm_sw_bkpt_match(cortexm_t cm, uint32_t pc)
{
	unsigned int c, id;
	uint16_t insn;

	DBFPRINTF("PC %08" PRIx32, pc);

	if (target_get_memory_type(cm->cm_target, pc, &c) == 0 ||
	    c == TARGET_MEM_FLASH || c == TARGET_MEM_ROM) {
		DBPRINTF(": read-only memory\n");
		return false;
	}

	if (TARGET_MEM_READ(cm->cm_target, pc, sizeof(uint16_t), &insn) < 0) {
		(void) TARGET_LINK_ERROR(cm->cm_target);
		DBPRINTF(": failed to read insn\n");
		return false;
	}

	DBPRINTF(", insn %04" PRIx16 ": ", insn);

	if (!CORTEXM_IS_BKPT_INSN(insn)) {
		DBPRINTF("not a BKPT insn\n");
		return false;
	}

	id = CORTEXM_BKPT_ID(insn);

	if (id >= CORTEXM_MAX_SW_BREAKPOINTS) {
		DBPRINTF("bkpt ID out of range\n");
		return false;
	}

	if (bitmap_get_state(cm->cm_sw_breakpoints, id) == false) {
		DBPRINTF("no SW bkpt registered\n");
		return false;
	}

	if (cm->cm_sw_breakpoint_addr[id] != pc) {
		DBPRINTF("SW bkpt address mismatch (%08" PRIx32 " != "
		    "%08" PRIx32 ")\n", cm->cm_sw_breakpoint_addr[id], pc);
		return false;
	}

	DBPRINTF("Hit!\n");

	return true;
}

static bool
cortexm_semihosting(cortexm_t cm, uint32_t pc)
{
	uint16_t insn;

	DBFPRINTF("PC %08" PRIx32, pc);

	if ((cm->cm_flags & CORTEXM_FLAG_RESUME_CALLED) == 0) {
		/*
		 * We have not (yet) resumed the target, thus we should
		 * not be handling semihosting calls.
		 */
		DBPRINTF(": not yet resumed\n");
		return false;
	}

	if (TARGET_MEM_READ(cm->cm_target, pc & ~1u,
	    sizeof(uint16_t), &insn) < 0) {
		(void) TARGET_LINK_ERROR(cm->cm_target);
		DBPRINTF(": failed to read insn\n");
		return false;
	}

	DBPRINTF(", insn %04" PRIx16 ": ", insn);

	if (!CORTEXM_IS_BKPT_INSN(insn)) {
		DBPRINTF(" not a BKPT insn\n");
		return false;
	}

	if (CORTEXM_BKPT_ID(insn) != CORTEXM_BKPT_SEMIHOSTING) {
		DBPRINTF(" not a semi-hosting BKPT\n");
		return false;
	}

	DBPRINTF(" semi-hosting BKPT\n");

	cm->cm_syscall.ts_pc = pc;

	/* Fetch the operation number and parameter block address */
	cm->cm_syscall.ts_args[0] = cortexm_core_reg_read(cm,
	    CMSCW_DCRSR_REG_GPR(0));
	cm->cm_syscall.ts_args[1] = cortexm_core_reg_read(cm,
	   CMSCW_DCRSR_REG_GPR(1));

	/* Fetch the current stack pointer. */
	cm->cm_syscall.ts_sp = cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_SP);

	if (TARGET_LINK_ERROR(cm->cm_target))
		(void) TARGET_LINK_ERROR(cm->cm_target);

	DBPRINTF("Semihost args: op 0x%" PRIx32 ", params @ 0x%08" PRIx32 "\n",
	    cm->cm_syscall.ts_args[0], cm->cm_syscall.ts_args[1]);

	return true;
}

static u_int
cortexm_core_halt_state_impl(cortexm_t cm)
{
	struct target_watchbreak *tw = &cm->cm_last_watchbreak;
	uint32_t pc, dhcsr, dfsr;
	unsigned int i;

	dhcsr = cortexm_dhcsr_read(cm);

	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("failed to read DHCSR\n");
		return TARGET_HALT_STATE_ERROR;
	}

	if ((dhcsr & CMSCS_DHCSR_S_LOCKUP) != 0) {
		/* Try to recover from core lockup */
		DBFPRINTF("Recovering from lockup\n");
		dhcsr = CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_DEBUGEN;
		cortexm_dhcsr_write(cm, dhcsr);
		dhcsr = cortexm_dhcsr_read(cm);
		DBFPRINTF("post lockup DHCSR 0x%08" PRIx32 "\n", dhcsr);
	}

	if ((dhcsr & CMSCS_DHCSR_S_HALT) == 0)
		return TARGET_HALT_STATE_RUNNING;

	DBFPRINTF("DHCSR 0x%08" PRIx32 "\n", dhcsr);

	dfsr = cortexm_scs_read(cm, CMSCS_REG_DFSR);
	cortexm_scs_write(cm, CMSCS_REG_DFSR, dfsr);

	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("failed to rd/wr DFSR\n");
		return TARGET_HALT_STATE_RUNNING;
	}

	cm->cm_ccr = cortexm_scs_read(cm, CMSCS_REG_CCR);
	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("failed to read CCR\n");
	}

	if ((cm->cm_flags & CORTEXM_FLAG_HAS_FPU) != 0) {
		cm->cm_fpccr = cortexm_scs_read(cm, CMSCS_REG_FPCCR);
		cm->cm_fpcar = cortexm_scs_read(cm, CMSCS_REG_FPCAR);
		if (TARGET_LINK_ERROR(cm->cm_target)) {
			DBFPRINTF("failed to read FPCCR/FPCAR\n");
		}
	}

	DBFPRINTF("DFSR 0x%08" PRIx32 "\n", dfsr);

	cm->cm_last_halt_reason = TARGET_HALT_STATE_FORCED;
	if (dfsr == 0) {
		DBFPRINTF("returning previous halt reason: %d\n",
		    cm->cm_last_halt_reason);
		return cm->cm_last_halt_reason;
	}

	pc = cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_PC);
	(void) TARGET_LINK_ERROR(cm->cm_target);
	tw->tw_addr = pc & ~1u;

	if (pc == CMPC_LOCKUP_VALUE) {
		DBFPRINTF("Core in lock-up.\n");
		return TARGET_HALT_STATE_FORCED;
	}

	if (dfsr & CMSCS_DFSR_VCATCH) {
		DBFPRINTF("Vector Catch @ %08" PRIx32 ", LR %08" PRIx32 "\n",
		    pc, cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_LR));

		if (cortexm_fault_recover(cm, &pc)) {
			DBFPRINTF("VCATCH. State restored. pc %08" PRIx32 "\n",
			    pc);

			if (cortexm_hw_bkpt_find(cm, pc) >= 0) {
				DBFPRINTF("VCATCH. HW BKPT hit @ %08" PRIx32
				    "\n", pc);
				tw->tw_type = TARGET_BKWP_TYPE_HARD_BREAK;
				return TARGET_HALT_STATE_BREAKPOINT;
			} else
			if (cortexm_sw_bkpt_match(cm, pc & ~1u)) {
				cm->cm_flags |= CORTEXM_FLAG_BKPT;
				DBFPRINTF("VCATCH. SW BKPT hit @ %08" PRIx32
				    "\n", pc);
				tw->tw_type = TARGET_BKWP_TYPE_SOFT_BREAK;
				return TARGET_HALT_STATE_BREAKPOINT;
			} else
			if (cortexm_semihosting(cm, pc)) {
				DBFPRINTF("VCATCH. Semi-hosting BKPT hit @ "
				    "%08" PRIx32 "\n", pc);
				return TARGET_HALT_STATE_SYSCALL;
			} else {
				return TARGET_HALT_STATE_FAULT;
			}
		} else
		if ((dfsr & (CMSCS_DFSR_HALTED | CMSCS_DFSR_BKPT |
		    CMSCS_DFSR_DWTTRAP)) == 0) {
			DBFPRINTF("VCATCH @ %08" PRIx32 ". State not restored, "
			    "assuming FORCED\n", pc);
			return TARGET_HALT_STATE_FORCED;
		} else {
			DBFPRINTF("VCATCH @ %08" PRIx32 ". State not "
			    "restored\n", pc);
		}
	}

	if (dfsr & CMSCS_DFSR_DWTTRAP) {
		DBFPRINTF("DWTTRAP @ %08" PRIx32 "\n", pc);

		for (i = 0; i < cm->cm_num_hw_watchpoints; i++) {
			if (cortexm_dwt_func_match(cm, i, tw)) {
				DBFPRINTF("watch %u matched\n", i);
				break;
			}
		}

		if (i == cm->cm_num_hw_watchpoints) {
			DBFPRINTF("watchpoint not found\n");
			memset(tw, 0, sizeof(*tw));
			tw->tw_set = -1;
			/* Assume forced. */
			return TARGET_HALT_STATE_FORCED;
		} else {
			return TARGET_HALT_STATE_WATCHPOINT;
		}
	}

	if (dfsr & CMSCS_DFSR_BKPT) {
		if (cortexm_hw_bkpt_find(cm, pc) >= 0) {
			tw->tw_type = TARGET_BKWP_TYPE_HARD_BREAK;
			DBFPRINTF("HW BKPT hit @ %08" PRIx32 "\n", pc);
		} else
		if (cortexm_sw_bkpt_match(cm, pc & ~1u)) {
			cm->cm_flags |= CORTEXM_FLAG_BKPT;
			tw->tw_type = TARGET_BKWP_TYPE_SOFT_BREAK;
			DBFPRINTF("sw BKPT hit @ %08" PRIx32 "\n", pc);
		} else
		if (cortexm_semihosting(cm, pc)) {
			DBFPRINTF("BKPT: Semi-hosting BKPT hit @ %08" PRIx32
			    "\n", pc);
			return TARGET_HALT_STATE_SYSCALL;
		} else {
			DBFPRINTF("hw BKPT hit @ %08" PRIx32 "\n", pc);
		}

		tw->tw_type = TARGET_BKWP_TYPE_HARD_BREAK;
		return TARGET_HALT_STATE_BREAKPOINT;
	}

	if (dfsr & CMSCS_DFSR_HALTED) {
		if (cm->cm_flags & CORTEXM_FLAG_SINGLE_STEP) {
			DBFPRINTF("HALT-STEPPING @ %08" PRIx32 "\n", pc);
			return TARGET_HALT_STATE_SINGLE_STEP;
		} else {
			DBFPRINTF("HALT @ %08" PRIx32 "\n", pc);
			return TARGET_HALT_STATE_FORCED;
		}
	}

	DBFPRINTF("defaulting to BREAKPOINT @ %08" PRIx32 "\n",
	    cortexm_core_reg_read(cm, CMSCW_DCRSR_REG_PC));
	tw->tw_type = TARGET_BKWP_TYPE_HARD_BREAK;
	return TARGET_HALT_STATE_BREAKPOINT;
}

static void
cortexm_sync_demcr(cortexm_t cm)
{

	if ((cm->cm_demcr & cm->cm_demcr_cfg_bits) !=
	    (cortexm_vc & cm->cm_demcr_cfg_bits)) {
		cm->cm_demcr &= ~cm->cm_demcr_cfg_bits;
		cm->cm_demcr |= cortexm_vc & cm->cm_demcr_cfg_bits;
		cortexm_scs_write(cm, CMSCS_REG_DEMCR, cm->cm_demcr);
		(void) TARGET_LINK_ERROR(cm->cm_target);
	}
}

static u_int
cortexm_core_halt_state(cortexm_t cm)
{
	u_int rv, last_halt = cm->cm_last_halt_reason;

	/*
	 * While we're checking core state, check if the user has tweaked
	 * the bitmap of vectors to catch, and update DEMCR if yes.
	 */
	cortexm_sync_demcr(cm);

	rv = cortexm_core_halt_state_impl(cm);
	cm->cm_last_halt_reason = rv;

	if (rv < TARGET_HALT_STATE_RUNNING) {
		/*
		 * Target is stopped. Ensure software breakpoints are
		 * removed.
		 */
		cortexm_sw_bkpt_all_clear(cm, false);

		/* XXX: Verify secure debug state... */
		if (cortexm_v8m_secure_debug_available(cm)) {
			DBFPRINTF("Halted: DSCSR %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_DSCSR));
		}
	}

	if (rv != last_halt) {
		if (cm->cm_soc_state_changed != NULL)
			(cm->cm_soc_state_changed)(cm, rv);
	}

	return rv;
}

static u_int
cortexm_core_halt(cortexm_t cm)
{
	uint32_t dhcsr;

	/*
	 * Try to stop the CPU.
	 * This can timeout if the CPU is suspended using a WFI
	 * instruction and no interrupt occurs.
	 */
	dhcsr = cortexm_dhcsr_read(cm);
	DBFPRINTF("dhcsr %08" PRIx32 "\n", dhcsr);
	if ((dhcsr & (CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_DEBUGEN)) !=
	    (CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_DEBUGEN)) {
		DBFPRINTF("Core running. Requesting HALT\n");
		dhcsr |= CMSCS_DHCSR_C_HALT | CMSCS_DHCSR_C_DEBUGEN;
		cortexm_dhcsr_write(cm, dhcsr);
	}

	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("Core not responding.\n");
	} else {
		DBFPRINTF("core halted. Determine reason.\n");
	}

	return cortexm_core_halt_state(cm);
}

static void
cortexm_core_resume(cortexm_t cm, bool single_step)
{
	uint32_t old_dhcsr, new_dhcsr;
	int flag;

	DBFPRINTF("%s\n", single_step ? "Step" :
	    "Continue");

	old_dhcsr = cortexm_dhcsr_read(cm);
	DBFPRINTF("Old DHCSR %08" PRIx32 "\n", old_dhcsr);

	/*
	 * Before resuming, check if the user has tweaked the bitmap of
	 * vectors to catch, and update DEMCR if yes.
	 */
	cortexm_sync_demcr(cm);

	if ((old_dhcsr & CMSCS_DHCSR_S_HALT) != 0) {
		/*
		 * Processor is halted. This should always be the case.
		 */
		cortexm_sw_bkpt_all_set(cm);

		cm->cm_flags |= CORTEXM_FLAG_RESUME_CALLED;
		flag = cm->cm_flags & CORTEXM_FLAG_SINGLE_STEP;
		new_dhcsr = CMSCS_DHCSR_C_DEBUGEN;

		if (single_step) {
			cm->cm_flags |= CORTEXM_FLAG_SINGLE_STEP;
			new_dhcsr |= CMSCS_DHCSR_C_STEP;

			if ((cortexm_gflags & CORTEXM_GFLAG_MASK_ISR_MASK) ==
			    CORTEXM_GFLAG_MASK_ISR_ON) {
				new_dhcsr |= CMSCS_DHCSR_C_MASKINTS;
			}
		} else {
			cm->cm_flags &= ~CORTEXM_FLAG_SINGLE_STEP;
		}

		if (flag ^ (cm->cm_flags & CORTEXM_FLAG_SINGLE_STEP)) {
			DBFPRINTF("setting C_HALT\n");
			new_dhcsr |= CMSCS_DHCSR_C_HALT;
		}

		/*
		 * We now need to saisfy the constraints for DHCSR spelled out
		 * in the ARM Book of Words.
		 */
		if ((old_dhcsr & CMSCS_DHCSR_C_MASKINTS) !=
		    (new_dhcsr & CMSCS_DHCSR_C_MASKINTS)) {
			/*
			 * We're changing C_MASKINTS. This requires special
			 * handling.
			 *
			 * If we're also changing C_HALT from 0 -> 1, then
			 * we can write the new DHCSR in one operation.
			 *
			 * However, changing C_HALT from 1 -> 0 requires
			 * first writing the new C_MASKINTS value.
			 */
			if ((old_dhcsr & CMSCS_DHCSR_C_HALT) != 0 &&
			    (new_dhcsr & CMSCS_DHCSR_C_HALT) == 0) {
				DBFPRINTF("DHCSR C_MASKINTS special "
				    "handling.\n");

				cortexm_dhcsr_write(cm,
				    (old_dhcsr & ~CMSCS_DHCSR_C_MASKINTS) |
				    (new_dhcsr & CMSCS_DHCSR_C_MASKINTS));
			}
		}

		DBFPRINTF("DHCSR %08" PRIx32 " -> %08" PRIx32 "\n", old_dhcsr,
		    new_dhcsr);

		cortexm_dhcsr_write(cm, new_dhcsr);

		if (new_dhcsr & CMSCS_DHCSR_C_HALT) {
			DBFPRINTF("Clear C_HALT\n");
			rtos_task_sleep(1);
			cortexm_dhcsr_write(cm, new_dhcsr & ~CMSCS_DHCSR_C_HALT);
		}
	} else {
		DBFPRINTF("core not halted!\n");
	}

	(void) TARGET_LINK_ERROR(cm->cm_target);
}

static int
cortexm_applet_run(cortexm_t cm, const struct target_run_applet *ra)
{

	/* Set the start address. */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_PC,
	    ra->ra_applet_address | 0x1u);

	/* Give it a stack pointer. */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_MSP,
	    ra->ra_applet_sp);
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_PSP,
	    ra->ra_applet_sp);

	/* Parameters. */
	for (unsigned int i = 0; i < ra->ra_nargs; i++) {
		cortexm_core_reg_write(cm, ra->ra_args[i].aa_reg,
		    ra->ra_args[i].aa_val);
	}

	/* Clear all bar the Thumb bit in xPSR. */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_xPSR, 0x01000000u);

	/*
	 * Set CONTROL, FAULTMASK, BASEPRI, and PRIMASK to ensure faults
	 * and interrupts don't get in the way.
	 */
	cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_CFBP,
	    (1u << CMSCW_DCRSR_CFBP_PRIMASK_SHIFT) |
	    (0u << CMSCW_DCRSR_CFBP_BASEPRI_SHIFT) |
	    (1u << CMSCW_DCRSR_CFBP_FAULTMASK_SHIFT) |
	    (0u << CMSCW_DCRSR_CFBP_CONTROL_SHIFT));

	cm->cm_mpu_ctrl_save = 0;
	cm->cm_flags |= CORTEXM_FLAG_APPLET_RUNNING;
	cortexm_core_resume(cm, false);

	return 0;
}

static int
cortexm_applet_stop(cortexm_t cm, uint32_t *rvp)
{
	u_int state;

	assert((cm->cm_flags & CORTEXM_FLAG_APPLET_RUNNING) != 0);

	cm->cm_flags &= ~(CORTEXM_FLAG_APPLET_RUNNING |
	    CORTEXM_FLAG_RESUME_CALLED);

	if (rvp != NULL) {
		state = cortexm_core_halt_state(cm);
		if (state != TARGET_HALT_STATE_RUNNING) {
			*rvp = cortexm_core_reg_read(cm,
			    CMSCW_DCRSR_REG_GPR(0));
		} else {
			*rvp = 0;
		}
	}

	if (cortexm_core_reset(cm, 1) < 0)
		return -1;

	state = cortexm_core_halt_state_impl(cm);
	return (state == TARGET_HALT_STATE_FORCED) ? 0 : -1;
}

static int
cortexm_sw_bkpt_find(cortexm_t cm, target_addr_t addr)
{
	unsigned int bit;

	addr &= ~1u;
	bit = 0;
	while ((bit = bitmap_find_first(cm->cm_sw_breakpoints,
	    CORTEXM_MAX_SW_BREAKPOINTS, bit, true)) != 0) {
		if (cm->cm_sw_breakpoint_addr[bit - 1] == addr)
			break;
	}

	return (int)bit - 1;
}

static uintptr_t
cortexm_sw_bkpt_set(cortexm_t cm, const struct target_watchbreak *tw)
{
	unsigned int bit;

	/* XXX: Ditch the magic constant. */
	if (tw->tw_kind != 2) {
		DBFPRINTF("tw_kind is %" PRIu32 "\n", tw->tw_kind);
		return TARGET_BKWP_STATUS_UNSUPPORTED;
	}

	/*
	 * Check if there's an existing hardware breakpoint here. If so,
	 * just return.
	 */
	if (cortexm_hw_bkpt_find(cm, tw->tw_addr) >= 0) {
		DBFPRINTF("HW bkpt already there\n");
		return TARGET_BKWP_STATUS_OK;
	}

	/*
	 * Scan existing soft breakpoints in case this is a duplicate.
	 */
	if (cortexm_sw_bkpt_find(cm, tw->tw_addr) >= 0) {
		DBFPRINTF("SW bkpt already there\n");
		return TARGET_BKWP_STATUS_OK;
	}

	/* Locate the first unused s/w breakpoint. */
	bit = bitmap_find_first(cm->cm_sw_breakpoints,
	    CORTEXM_MAX_SW_BREAKPOINTS, 0, false);
	if (bit == 0) {
		DBFPRINTF("No free SW breakpoints\n");
		return TARGET_BKWP_STATUS_UNSUPPORTED;
	}

	bit -= 1;

	/* Save the breakpoint address and mark it as used */
	cm->cm_sw_breakpoint_addr[bit] = tw->tw_addr & ~1u;
	bitmap_set_state(cm->cm_sw_breakpoints, bit, true);

	DBFPRINTF("New SW breakpoint idx %u\n", bit);

	return TARGET_BKWP_STATUS_OK;
}

static uintptr_t
cortexm_sw_bkpt_clear(cortexm_t cm, const struct target_watchbreak *tw)
{
	target_addr_t addr;
	unsigned int bit;

	DBFPRINTF("check %08" PRIxTADDR "\n", tw->tw_addr & ~1u);

	addr = tw->tw_addr & ~1u;
	bit = 0;

	/* Scan existing breakpoints */
	while ((bit = bitmap_find_first(cm->cm_sw_breakpoints,
	    CORTEXM_MAX_SW_BREAKPOINTS, bit, true)) != 0) {
		if (cm->cm_sw_breakpoint_addr[bit - 1] == addr)
			break;
	}

	/* Not found? */
	if (bit == 0) {
		DBFPRINTF("not found\n");
		return TARGET_BKWP_STATUS_UNSUPPORTED;
	}

	/* Mark the breakpoint as unused */
	bitmap_set_state(cm->cm_sw_breakpoints, bit - 1, false);

	DBFPRINTF("Delete SW breakpoint idx %u\n", bit - 1);

	return TARGET_BKWP_STATUS_OK;
}

static void
cortexm_sw_bkpt_all_clear(cortexm_t cm, bool del)
{
	unsigned int bit;
	uint16_t insn;

	/*
	 * Clear all s/w breakpoint instructions in the target's memory.
	 */
	bit = 0;
	while ((bit = bitmap_find_first(cm->cm_sw_breakpoints,
	    CORTEXM_MAX_SW_BREAKPOINTS, bit, true)) != 0) {
		struct target_mem_readwrite mr;
		unsigned int i = bit - 1;

		DBFPRINTF("idx %u\n", i);

		/*
		 * Read the target memory to ensure the BKPT instruction
		 * has not been overwritten, whether deliberately or by
		 * accident.
		 */
		mr.mr_write = false;
		mr.mr_target_addr = (target_addr_t)cm->cm_sw_breakpoint_addr[i];
		mr.mr_length = sizeof(uint16_t);
		mr.mr_dest = &insn;

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE, &mr) <0)
			DBFPRINTF("error reading bkpt\n");

		if (!CORTEXM_IS_BKPT_INSN(insn) || CORTEXM_BKPT_ID(insn) != i) {
			DBFPRINTF("bkpt insn has been modified.\n");
			continue;
		}

		DBFPRINTF("Restoring insn %04" PRIx16 "\n",
		    cm->cm_sw_breakpoint_insn[i]);

		/* Restore the original instruction. */
		mr.mr_write = true;
		mr.mr_src = &cm->cm_sw_breakpoint_insn[i];

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE, &mr) <0)
			DBFPRINTF("error restoring bkpt\n");

		if (del) {
			bitmap_set_state(cm->cm_sw_breakpoints, i, false);
			DBFPRINTF("Deleted\n");
		}
	}
}

static void
cortexm_sw_bkpt_all_set(cortexm_t cm)
{
	unsigned int bit;

	/* Ignore s/w breakpoints if we're running our own applet. */
	if ((cm->cm_flags & CORTEXM_FLAG_APPLET_RUNNING) != 0) {
		DBFPRINTF("Applet running. Ignoring.\n");
		return;
	}

	/*
	 * Set all s/w breakpoint instructions in the target's memory.
	 */
	bit = 0;
	while ((bit = bitmap_find_first(cm->cm_sw_breakpoints,
	    CORTEXM_MAX_SW_BREAKPOINTS, bit, true)) != 0) {
		struct target_mem_readwrite mr;
		unsigned int i = bit - 1;
		uint16_t insn;

		DBFPRINTF("idx %u\n", i);

		/*
		 * Re-fetch existing instruction. User may have changed it
		 * from within Gdb while the target was halted.
		 */
		mr.mr_write = false;
		mr.mr_target_addr = (target_addr_t)cm->cm_sw_breakpoint_addr[i];
		mr.mr_length = sizeof(uint16_t);
		mr.mr_dest = &cm->cm_sw_breakpoint_insn[i];

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE,
		    &mr) < 0) {
			DBFPRINTF("error reading existing insn\n");
			/*
			 * XXX: What to do here? Do we ignore this breakpoint?
			 * For now, that's really the only course of action.
			 */
			continue;
		}

		DBFPRINTF("Original insn %04" PRIx16 "\n",
		    cm->cm_sw_breakpoint_insn[i]);

		/*
		 * Make a breakpoint instruction with its index encoded in the
		 * 8-bit immediate field.
		 */
		insn = CORTEXM_MAKE_BKPT_INSN(i);

		mr.mr_write = true;
		mr.mr_src = &insn;

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE,
		    &mr) < 0) {
			DBFPRINTF("error writing BKPT insn\n");
		}
	}
}

static uintptr_t
cortexm_core_watchbreak_set(cortexm_t cm, const struct target_watchbreak *tw)
{
	unsigned int mem_type, bit;

	switch (tw->tw_type) {
	case TARGET_BKWP_TYPE_SOFT_BREAK:
		/*
		 * Don't permit soft breakpoints in read-only memory.
		 * Note that this deliberately permits them to be set
		 * in 'filler' address space (see target_degap_memory())
		 * just in case the target has external RAM which we
		 * don't know about.
		 */
		if (target_get_memory_type(cm->cm_target, tw->tw_addr & ~1u,
		    &mem_type) == 0 || mem_type == TARGET_MEM_FLASH ||
		    mem_type == TARGET_MEM_ROM) {
			DBFPRINTF("SOFT BKPT in something that's not RAM\n");
			return TARGET_BKWP_STATUS_UNSUPPORTED;
		}

		DBFPRINTF("RAM BKPT @ %08" PRIxTADDR "\n", tw->tw_addr);
		return cortexm_sw_bkpt_set(cm, tw);

	case TARGET_BKWP_TYPE_HARD_BREAK:
		/*
		 * Hardware breakpoint. Don't override an extant software
		 * breakpoint.
		 */
		if (cortexm_sw_bkpt_find(cm, tw->tw_addr) >= 0)
			return TARGET_BKWP_STATUS_OK;

		/* Has a matching hardware breakpoint already been set? */
		if (cortexm_hw_bkpt_find(cm, tw->tw_addr) >= 0)
			return TARGET_BKWP_STATUS_OK;

		/* New hardware breakpoint. */
		bit = bitmap_find_first(cm->cm_hw_breakpoints,
		    cm->cm_num_hw_breakpoints, 0, false);
		if (bit == 0) {
			DBFPRINTF("No free HW breakpoints\n");
			return TARGET_BKWP_STATUS_UNSUPPORTED;
		}

		DBFPRINTF("Setting HARD BKPT %x\n", bit - 1);

		if (cortexm_hw_bkpt_set(cm, bit - 1, tw->tw_addr) == false)
			return TARGET_BKWP_STATUS_UNSUPPORTED;

		return TARGET_BKWP_STATUS_OK;

	case TARGET_BKWP_TYPE_WRITE_WATCH:
	case TARGET_BKWP_TYPE_READ_WATCH:
	case TARGET_BKWP_TYPE_ACCESS_WATCH:
		/* XXX: Ditch the magic constants. */
		switch (tw->tw_kind) {
		case 1:
		case 2:
		case 4:
			break;
		default:
			return TARGET_BKWP_STATUS_UNSUPPORTED;
		}
		if (cortexm_wapt_find(cm, tw) >= 0)
			return TARGET_BKWP_STATUS_OK;

		/* New watchpoint. */
		bit = bitmap_find_first(cm->cm_hw_watchpoints,
		    cm->cm_num_hw_watchpoints, 0, false);
		if (bit == 0) {
			DBFPRINTF("No free HW watchpoints\n");
			return TARGET_BKWP_STATUS_UNSUPPORTED;
		}

		bit -= 1;

		DBFPRINTF("WAPT x %x\n", bit);

		if (cortexm_wapt_set(cm, tw, bit) < 0)
			return TARGET_BKWP_STATUS_UNSUPPORTED;

		return TARGET_BKWP_STATUS_OK;

	default:
		break;
	}

	return TARGET_BKWP_STATUS_UNSUPPORTED;
}

static uintptr_t
cortexm_core_watchbreak_clear(cortexm_t cm, const struct target_watchbreak *tw)
{
	uintptr_t rv;
	int c;

	switch (tw->tw_type) {
	case TARGET_BKWP_TYPE_SOFT_BREAK:
		rv = cortexm_sw_bkpt_clear(cm, tw);
		break;

	case TARGET_BKWP_TYPE_HARD_BREAK:
		/* Check if the breakpoint exists.  */
		c = cortexm_hw_bkpt_find(cm, tw->tw_addr);
		DBFPRINTF("clear BKPT @ %08" PRIxTADDR "result %d\n",
		    tw->tw_addr & ~1u, c);
		if (c >= 0) {
			unsigned int bit = (unsigned int)c;
			cortexm_fpb_write(cm, CMFPB_REG_FP_COMP(bit), 0);
			bitmap_set_state(cm->cm_hw_breakpoints, bit, false);
			(void) TARGET_LINK_ERROR(cm->cm_target);
		}
		rv = TARGET_BKWP_STATUS_OK;
		break;

	case TARGET_BKWP_TYPE_WRITE_WATCH:
	case TARGET_BKWP_TYPE_READ_WATCH:
	case TARGET_BKWP_TYPE_ACCESS_WATCH:
		c = cortexm_wapt_find(cm, tw);
		DBFPRINTF("clear WAPT @ %08" PRIxTADDR " result %d\n",
		    tw->tw_addr, c);
		if (c >= 0) {
			unsigned int bit = (unsigned int)c;
			cortexm_dwt_write(cm, CMDWT_REG_FUNCTION(bit), 0);
			cortexm_dwt_write(cm, CMDWT_REG_COMP(bit), 0);
			cortexm_dwt_write(cm, CMDWT_REG_MASK(bit), 0);
			bitmap_set_state(cm->cm_hw_watchpoints, bit, false);
			(void) TARGET_LINK_ERROR(cm->cm_target);
		}
		rv = TARGET_BKWP_STATUS_OK;
		break;

	default:
		rv = TARGET_BKWP_STATUS_UNSUPPORTED;
		break;
	}

	return rv;
}

static const char *
cortexm_fpu_string(cortexm_t cm)
{

	if ((cm->cm_flags & CORTEXM_FLAG_HAS_FPU) == 0)
		return "Not present";

	if (cm->cm_flags & CORTEXM_FLAG_FPU_DP)
		return "FPv5-DP";

	if (CMSCS_CPUID_PARTNO(cm->cm_cpuid) == CPUID_PARTNO_CM4)
		return "FPv4-SP";

	return "FPv5-SP";
}

static stringio_t
cortexm_cache_string(cortexm_t cm, u_int mask)
{
	stringio_t ss = stringio_new();

	if ((cm->cm_flags & mask) == 0) {
		stringio_printf(ss, "not present");
		return ss;
	}

	if ((cm->cm_flags & mask) & CORTEXM_FLAG_HAS_ARM_CACHE) {
		const char *cache_type, *sep;
		unsigned int cache_size;
		uint32_t reg;

		switch (CMSCS_CLIDR_CTYPE(cm->cm_clidr, 0)) {
		case CMSCS_CLIDR_CTYPE_I:
			cache_type = "Insn only";
			break;
		case CMSCS_CLIDR_CTYPE_D:
			cache_type = "Data only";
			break;
		case CMSCS_CLIDR_CTYPE_ID:
			cache_type = "Separate data and insn";
			break;
		case CMSCS_CLIDR_CTYPE_UNIFIED:
			cache_type = "Unified";
			break;
		default:
			cache_type = "Unknown";
			break;
		}

		reg = cortexm_scs_read(cm, CMSCS_REG_CCSIDR);

		cache_size = (CMSCS_CCSIDR_LINESIZE_BYTES(reg) *
		    CMSCS_CCSIDR_ASSOCIATIVITY(reg) *
		    CMSCS_CCSIDR_NUM_SETS(reg)) / 1024u;

		stringio_printf(ss, "%u KB, Ways %u, Linesize %u, %s",
		    cache_size, CMSCS_CCSIDR_ASSOCIATIVITY(reg),
		    CMSCS_CCSIDR_LINESIZE_BYTES(reg), cache_type);

		sep = ", ";
		if (reg & CMSCS_CCSIDR_RA) {
			stringio_printf(ss, "%sRA", sep);
			sep = "/";
		}
		if (reg & CMSCS_CCSIDR_WA) {
			stringio_printf(ss, "%sWA", sep);
			sep = "/";
		}
		if (reg & CMSCS_CCSIDR_WB) {
			stringio_printf(ss, "%sWB", sep);
			sep = "/";
		}
		if (reg & CMSCS_CCSIDR_WT)
			stringio_printf(ss, "%sWT", sep);
	}

	if ((cm->cm_flags & mask) & CORTEXM_FLAG_HAS_SOC_CACHE) {
		struct cortexm_cache_params cp;
		uint32_t cmd = CORTEXM_CTL_CACHE_PARAMS;

		assert(cm->cm_soc_ctl != NULL);
		(void) (cm->cm_soc_ctl)(cm, &cmd, &cp);

		if ((cm->cm_flags & mask) & CORTEXM_FLAG_HAS_ARM_CACHE)
			stringio_printf(ss, ", ");

		stringio_printf(ss, "%u KB, Ways %u, Linesize %u, %s",
		    cp.cp_linesize * cp.cp_ways * cp.cp_sets,
		    cp.cp_ways, cp.cp_linesize, cp.cp_type);
	}

	return ss;
}

static const char cortexm7_erratum_702596[] =
"This CPU core suffers from Cortex-M7 erratum 702596. Single-stepping "
"may be difficult if interrupts are enabled. Maven implements the work-"
"around suggested in the errata document, but problems will persist as "
"there is no complete fix.";

static void
cortexm_target_info(cortexm_t cm, struct target_info *ti)
{
#ifdef NETWORK_OPT_HTTPD
	stringio_t ss = ti->ti_sio;
#endif
	FILE *os = ti->ti_os;

	if (os != NULL) {
		struct adiv5_memap_link_details ld;

		if (TARGET_LINK_IOCTL(cm->cm_target,
		    ADIV5_MEMAP_CTL_LINK_DETAILS, &ld) == 0 &&
		        cm->cm_target->t_index == 0) {
			fprintf(os, "Debug Interface: %s\n", ld.ld_link_name);
		}

		fprintf(os, "CPU Core %u: %s (r%up%u), ARMv%u-M %s "
		    "implementation\n", cm->cm_target->t_index,
		    cm->cm_core_name,
		    CMSCS_CPUID_VARIANT(cm->cm_cpuid),
		    CMSCS_CPUID_REVISION(cm->cm_cpuid),
		    CORTEXM_FLAG_ARCH(cm) + 6,
		    CORTEXM_HAS_MAIN(cm) ? "Mainline" : "Base");

		if (CMSCS_CPUID_PARTNO(cm->cm_cpuid) == CPUID_PARTNO_CM7 &&
		    CMSCS_CPUID_VARIANT(cm->cm_cpuid) == 0 &&
		    CMSCS_CPUID_REVISION(cm->cm_cpuid) <= 1) {
			fputs("WARNING:\n", os);
			shell_puts_wrap(os, 77, cortexm7_erratum_702596, "   ");
		}
	}

#ifdef NETWORK_OPT_HTTPD
	if (ss != NULL) {
		const char *notes;
		char buff[8];

		stringio_printf(ss, JSON_NUMBER("num", cm->cm_target->t_index));
		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_STRING("name", cm->cm_core_name));
		stringio_printf(ss, JSON_SEPARATOR);

		snprintf(buff, sizeof(buff), "r%up%u",
		    CMSCS_CPUID_VARIANT(cm->cm_cpuid),
		    CMSCS_CPUID_REVISION(cm->cm_cpuid));
		stringio_printf(ss, JSON_STRING("rev", buff));
		stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_NUMBER("vers",
		    CORTEXM_FLAG_ARCH(cm) + 6));
		stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_STRING("imp",
		    CORTEXM_HAS_MAIN(cm) ? "Mainline" : "Base"));
		stringio_printf(ss, JSON_SEPARATOR);

		if (CMSCS_CPUID_PARTNO(cm->cm_cpuid) == CPUID_PARTNO_CM7 &&
		    CMSCS_CPUID_VARIANT(cm->cm_cpuid) == 0 &&
		    CMSCS_CPUID_REVISION(cm->cm_cpuid) <= 1) {
			notes = cortexm7_erratum_702596;
		} else {
			notes = "";
		}
		stringio_printf(ss, JSON_STRING("core_notes", notes));
	}
#endif

#ifdef DEBUG_ENABLED
	if (DEBUG_FLAG && os != NULL) {
		fprintf(os, "CPUID: %08" PRIx32 "\n",
		    cortexm_scs_read(cm, CMSCS_REG_CPUID));
	}
#endif

	if (CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V8M) {
		bool has_tz = CORTEXM_V8M_HAS_EXT_SECURITY(cm);
		bool secure_dbg = (cortexm_dhcsr_read(cm) &
		    CMSCS_DHCSR_S_SDE) != 0;

		if (os != NULL) {
			fprintf(os, "Security Extension: %spresent\n",
			    has_tz ? "" : "not ");

			if (has_tz) {
				fprintf(os, "Secure Debug: %sabled\n",
				    secure_dbg ? "En" : "Dis ");
			}
		}

#ifdef NETWORK_OPT_HTTPD
		if (ss != NULL) {
			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_BOOL("tz", has_tz));
			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_BOOL("secdbg", secure_dbg));
		}
#endif
	}

	if (os != NULL) {
		fprintf(os, "MPU: %spresent\n",
		    (cm->cm_flags & CORTEXM_FLAG_HAS_MPU) ? "" : "not ");
	}

#ifdef NETWORK_OPT_HTTPD
	if (ss != NULL) {
		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_BOOL("mpu",
		    cm->cm_flags & CORTEXM_FLAG_HAS_MPU));
	}
#endif

	if (CORTEXM_FLAG_ARCH(cm) >= CORTEXM_FLAG_ARCH_V7M) {
		const char *fpu_str;
		stringio_t acs, scs;

		fpu_str = cortexm_fpu_string(cm);
		acs = cortexm_cache_string(cm, CORTEXM_FLAG_HAS_ARM_CACHE);
		scs = cortexm_cache_string(cm, CORTEXM_FLAG_HAS_SOC_CACHE);

		if (os != NULL) {
			fprintf(os, "FPU: %s\n", fpu_str);
			fprintf(os, "ARM Cache: %s\n", stringio_get_buff(acs));
			fprintf(os, "SoC Cache: %s\n", stringio_get_buff(scs));
		}

#ifdef NETWORK_OPT_HTTPD
		if (ss != NULL) {
			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_STRING("fpu", fpu_str));
			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_STRING("armcache",
			    stringio_get_buff(acs)));
			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_STRING("soccache",
			    stringio_get_buff(scs)));
		}
#endif

		stringio_done(acs);
		stringio_done(scs);

#ifdef DEBUG_ENABLED
		if (DEBUG_FLAG && os != NULL && CORTEXM_HAS_MAIN(cm)) {
			fprintf(os, "ID_PFR0-1: %08" PRIx32 " %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_ID_PFR0),
			    cortexm_scs_read(cm, CMSCS_REG_ID_PFR1));

			fprintf(os, "ID_DFR0: %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_ID_DFR0));

			fprintf(os, "ID_AFR0: %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_ID_AFR0));

			fprintf(os,
			    "ID_MMFR0-3: %08" PRIx32 " %08" PRIx32 " %08" PRIx32
			    " %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_ID_MMFR0),
			    cortexm_scs_read(cm, CMSCS_REG_ID_MMFR1),
			    cortexm_scs_read(cm, CMSCS_REG_ID_MMFR2),
			    cortexm_scs_read(cm, CMSCS_REG_ID_MMFR3));

			fprintf(os,
			    "ID_ISAR0-4: %08" PRIx32 " %08" PRIx32 " %08" PRIx32
			    " %08" PRIx32 " %08" PRIx32 "\n",
			    cortexm_scs_read(cm, CMSCS_REG_ID_ISAR0),
			    cortexm_scs_read(cm, CMSCS_REG_ID_ISAR1),
			    cortexm_scs_read(cm, CMSCS_REG_ID_ISAR2),
			    cortexm_scs_read(cm, CMSCS_REG_ID_ISAR3),
			    cortexm_scs_read(cm, CMSCS_REG_ID_ISAR4));

			if ((cm->cm_flags & CORTEXM_FLAG_HAS_ARM_CACHE) != 0) {
				fprintf(os, "CLIDR: %08" PRIx32 "\n",
				    cm->cm_clidr);
				fprintf(os, "CTR: %08" PRIx32 "\n", cm->cm_ctr);
				fprintf(os, "CCR: %08" PRIx32 "\n", cm->cm_ccr);
				fprintf(os, "CCSIDR: %08" PRIx32 "\n",
				    cortexm_scs_read(cm, CMSCS_REG_CCSIDR));
				fprintf(os, "CSSELR: %08" PRIx32 "\n",
				    cortexm_scs_read(cm, CMSCS_REG_CSSELR));
			}
		}
#endif /* DEBUG_ENABLED */
	}

	if (os != NULL && cm->cm_num_hw_breakpoints) {
		u_int x, n;

		fprintf(os, "Hardware breakpoints: %u\n",
		    cm->cm_num_hw_breakpoints);

		for (x = n = 0; x < cm->cm_num_hw_breakpoints; x++) {
			if (bitmap_get_state(cm->cm_hw_breakpoints, x) == false)
				continue;

			fprintf(os, "\t#%u: 0x%08" PRIx32 "\n", n++,
			    cortexm_hw_bkpt_get_addr(cm, x));
		}
	}

	if (os != NULL && cm->cm_num_hw_watchpoints) {
		uint32_t addr, func;
		const char *op;
		u_int x, n;

		fprintf(os, "Hardware watchpoints: %u\n",
		    cm->cm_num_hw_watchpoints);

		for (x = n = 0; x < cm->cm_num_hw_watchpoints; x++) {
			if (bitmap_get_state(cm->cm_hw_watchpoints, x) == false)
				continue;

			func = cortexm_dwt_read(cm, CMDWT_REG_FUNCTION(x));
			addr = cortexm_dwt_read(cm, CMDWT_REG_COMP(x));

			switch (func & CMDWT_FUNC_ACTION_MASK) {
			case CMDWT_FUNC_ACTION_READ:
				op = "Read";
				break;
			case CMDWT_FUNC_ACTION_WRITE:
				op = "Write";
				break;
			default:
				op = "Read/Write";
				break;
			}

			fprintf(os, "\t#%iu: 0x%08" PRIx32 " (%s)\n", n++, addr,
			    op);
		}
	}

	if (os != NULL && TARGET_LINK_ERROR(cm->cm_target)) {
		fprintf(os, "Debug port error while gathering "
			    "target info.\n");
	}

#ifdef NETWORK_OPT_HTTPD
	if (ss != NULL) {
		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_NUMBER("hwbreak",
		    cm->cm_num_hw_breakpoints));

		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_NUMBER("hwwatch",
		    cm->cm_num_hw_watchpoints));
	}
#endif
}

/*
 * Invalidate the instruction cache for the specified address range.
 */
static void
cortexm_icache_sync(cortexm_t cm, target_addr_t addr, uint32_t len)
{
	target_addr_t start_addr, end_addr;
	uint32_t imin, reg;

	if ((cm->cm_ccr & CMSCS_CCR_IC) == 0)
		return;

	imin = 4 << CMSCS_CTR_IMINLINE(cm->cm_ctr);
	start_addr = addr & ~((target_addr_t)imin - 1);
	end_addr = start_addr + len;
	end_addr = (end_addr + ((target_addr_t)imin - 1)) &
	    ~((target_addr_t)imin - 1);

	while (start_addr < end_addr) {
		reg = (uint32_t) start_addr;
		cortexm_scs_write(cm, CMSCS_REG_ICIMVAU, reg);
		start_addr += imin;
	}

	cortexm_scs_write(cm, CMSCS_REG_BPIALL, 0);
}

/*
 * Flush the data cache for the specified address range.
 * This ensures SRAM is coherent with resepect to the data cache.
 * The cache is invalidated if 'inv' is true.
 */
static void
cortexm_dcache_clean(cortexm_t cm, target_addr_t addr, uint32_t len, bool inv)
{
	target_addr_t start_addr, end_addr;
	uint32_t dmin, reg, op;

	if ((cm->cm_ccr & CMSCS_CCR_DC) == 0)
		return;

	dmin = 4 << CMSCS_CTR_DMINLINE(cm->cm_ctr);
	start_addr = addr & ~((target_addr_t)dmin - 1);
	end_addr = start_addr + len;
	end_addr = (end_addr + ((target_addr_t)dmin - 1)) &
	    ~((target_addr_t)dmin - 1);
	op = inv ? CMSCS_REG_DCCIMVAC : CMSCS_REG_DCCMVAC;

	while (start_addr < end_addr) {
		reg = (uint32_t) start_addr;
		cortexm_scs_write(cm, op, reg);
		start_addr += dmin;
	}
}

static void
cortexm_sync_cache(cortexm_t cm, target_addr_t addr, uint32_t len, bool iswr)
{

	if (len == 0)
		return;

	switch (CMSCS_CLIDR_CTYPE(cm->cm_clidr, 0)) {
	case CMSCS_CLIDR_CTYPE_I:
		/*
		 * Target has an ICache only.
		 * For reads, do nothing.
		 * For writes, invalidate the range in the ICache.
		 */
		if (iswr)
			cortexm_icache_sync(cm, addr, len);
		break;

	case CMSCS_CLIDR_CTYPE_D:
	case CMSCS_CLIDR_CTYPE_UNIFIED:
		/*
		 * Target has a DCache only, or a unified cache.
		 * For reads, just clean the cache. For writes, a clean
		 * and invalidate is required.
		 */
		cortexm_dcache_clean(cm, addr, len, iswr);
		break;

	case CMSCS_CLIDR_CTYPE_ID:
		/*
		 * Target has separate instruction and data caches.
		 * For reads, just clean the DCache.
		 * For writes, clean/invalidate the DCache and invalidate
		 * the ICache.
		 */
		cortexm_dcache_clean(cm, addr, len, iswr);
		if (iswr)
			cortexm_icache_sync(cm, addr, len);
		break;

	default:
		break;
	}
}

static int
cortexm_fetch_halt_state(cortexm_t cm, int rv,
    union target_halt_state *ths)
{

	switch (rv) {
	case TARGET_HALT_STATE_SYSCALL:
		ths->ths_syscall = cm->cm_syscall;
		break;

	case TARGET_HALT_STATE_BREAKPOINT:
		ths->ths_halt.th_type = cm->cm_last_watchbreak.tw_type;
		ths->ths_halt.th_addr = cm->cm_last_watchbreak.tw_addr;
		break;

	case TARGET_HALT_STATE_WATCHPOINT:
		if (cm->cm_last_watchbreak.tw_set < 0)
			rv = TARGET_HALT_STATE_FAULT;
		else
			ths->ths_halt.th_type = cm->cm_last_watchbreak.tw_type;
		ths->ths_halt.th_addr = cm->cm_last_watchbreak.tw_addr;
		break;

	default:
		ths->ths_halt.th_addr = cm->cm_last_watchbreak.tw_addr;
		break;
	}

	return rv;
}

static int
cortexm_core_ctl(target_t t, uint32_t *cmd, void *arg)
{
	cortexm_t cm = t->t_core;
	int rv = 0;

	if (cm->cm_soc_ctl != NULL) {
		rv = (cm->cm_soc_ctl)(cm, cmd, arg);
		if (rv < 0)
			return rv;
	}

	switch (*cmd) {
	case TARGET_CTL_NOP:
		/* Used by SoC ctl code to indicate that it did all the work */
		break;

	case TARGET_CTL_GET_TARGET_NAME:
		{
			struct target_ctl_get_target_name *tn = arg;

			tn->tn_core_name = cm->cm_core_name;
		}
		break;

	case TARGET_CTL_QUERY_ATTACH:
		*((int *)arg) = (cm->cm_flags & CORTEXM_FLAG_ATTACHED) != 0;
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_ATTACH:
		*((int *)arg) = cortexm_core_attach(cm, *((int *)arg));
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_DETACH:
		cortexm_core_detach(cm);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_HALT_QUERY:
		rv = cm->cm_last_halt_reason;
		if (rv == TARGET_HALT_STATE_SYSCALL)
			rv = TARGET_HALT_STATE_RUNNING;
		else if (arg)
			rv = cortexm_fetch_halt_state(cm, rv, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_HALT_POLL:
		rv = cortexm_core_halt_state(cm);
		if (arg)
			rv = cortexm_fetch_halt_state(cm, rv, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_HALT:
		rv = cortexm_core_halt(cm);
		if (arg) {
			memcpy(arg, &cm->cm_last_watchbreak,
			    sizeof(cm->cm_last_watchbreak));
		}
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_RESUME:
		if ((cm->cm_flags & CORTEXM_FLAG_ATTACHED) != 0) {
			struct target_resume *tr = arg;

			if (tr->tr_flags & TARGET_RESUME_FLAG_ADDR_VALID) {
				cortexm_core_reg_write(cm, CMSCW_DCRSR_REG_PC,
				    (uint32_t)tr->tr_addr);
			}

			if (tr->tr_flags & TARGET_RESUME_FLAG_RV_VALID) {
				/* Syscall return value -> R0 */
				cortexm_core_reg_write(cm,
				    CMSCW_DCRSR_REG_GPR(0), tr->tr_rv);
			}

			cortexm_core_resume(cm,
			  (tr->tr_flags & TARGET_RESUME_FLAG_SINGLE_STEP) != 0);
			rv = 1;
		}
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_RESET:
		if ((*(int *)arg) < TARGET_RESET_HARD) {
			rv = cortexm_core_reset(cm, *((int *)arg));
		} else {
			adiv5_ll_nRST_toggle();
			rv = 1;
		}
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_READ_REGS:
		rv = cortexm_txml_read_regs(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_WRITE_REGS:
		rv = cortexm_txml_write_regs(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_READ_ONE_REG:
		rv = cortexm_txml_one_reg_read(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_WRITE_ONE_REG:
		rv = cortexm_txml_one_reg_write(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_WATCHBREAK:
		{
			struct target_watchbreak *tw = arg;

			if (tw->tw_set)
				rv = cortexm_core_watchbreak_set(cm, tw);
			else
				rv = cortexm_core_watchbreak_clear(cm, tw);
		}
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_MEM_READWRITE:
		if ((cm->cm_flags & CORTEXM_FLAG_APPLET_RUNNING) == 0 &&
		    (cm->cm_flags & CORTEXM_FLAG_HAS_ARM_CACHE) != 0 &&
		    (cm->cm_ccr & (CMSCS_CCR_DC | CMSCS_CCR_IC)) != 0) {
			struct target_mem_readwrite *mr = arg;
			unsigned int type;

			if (target_get_memory_type(cm->cm_target,
			    mr->mr_target_addr, &type) != 0 &&
			    type == TARGET_MEM_RAM) {
				cortexm_sync_cache(cm, mr->mr_target_addr,
				    mr->mr_length, mr->mr_write);
			}
		}
		break;

	case TARGET_CTL_GET_XML_REGS:
		rv = cortexm_txml_generate(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_TARGET_INFO:
		cortexm_target_info(cm, arg);
		break;

	case TARGET_CTL_APPLET_RUN:
		rv = cortexm_applet_run(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_APPLET_STOP:
		rv = cortexm_applet_stop(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

#ifdef CONFIG_GDB_RTOS_SUPPORT
	case TARGET_CTL_SET_RTOS_FRAME:
		rv = cortexm_rtos_set_frame(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;

	case TARGET_CTL_LOAD_RTOS_PARAMS:
		rv = cortexm_rtos_load_params(cm, arg);
		*cmd = TARGET_CTL_NOP;
		break;
#endif

	default:
		break;
	}

	return rv;
}

static int
cortexm_rte_addr(target_t t, target_addr_t rom_table, u_int idx, uint32_t *rv)
{
	uint32_t rte;

	if (TARGET_REG_READ32(t, rom_table + (idx * 4), &rte) < 0) {
		(void) TARGET_LINK_ERROR(t);
		return 0;
	}

	if (ADIV5_ROMENTRY_PRESENT(rte) == 0 ||
	    ADIV5_ROMENTRY_FORMAT_32(rte) == 0) {
		return 0;
	}

	*rv = (uint32_t) ADIV5_RTE2ADDR(rom_table, rte);

	return 1;
}

static int
cortexm_v6m_attach(cortexm_t cm, uint32_t cpuid, target_addr_t rom_table)
{
	target_t t = cm->cm_target;
	const char *cn;
	u_int i;
	int rv;

	if (CMSCS_CPUID_ARCHITECTURE(cpuid) != CPUID_ARCH_ARMV6M)
		return -1;

	switch (CMSCS_CPUID_PARTNO(cpuid)) {
	case CPUID_PARTNO_CM0_PLUS:
		cn = "0+";
		break;
	default:
		cn = "0";
		break;
	}

	sprintf(cm->cm_core_name, "Cortex-M%s", cn);

	DBPRINTF("\nAttaching %s (CPUID %08" PRIx32 ")\n", cm->cm_core_name,
	    cpuid);
	tmon_log("Attaching %s\n", cm->cm_core_name);

	/*
	 * Ensure AP.CSW "prot" bits are set as per the spec.
	 */
	uint8_t prot = ADIV5_MEMAP_CSW_PROT(0) | ADIV5_MEMAP_CSW_PROT(1);
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_CSW_PROT, &prot) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBPRINTF("Failed to configure AP.CSW\n");
		return -1;
	}

	/*
	 * Fetch the DWT and BPU addresses from the ROM table.
	 */
	if (cortexm_rte_addr(t, rom_table, CORTEXM_RTE_FPB_IDX,
	    &cm->cm_fpb_base) < 0 ||
	    cortexm_rte_addr(t, rom_table, CORTEXM_RTE_DWT_IDX,
	    &cm->cm_dwt_base) < 0) {
		return -1;
	}

	DBFPRINTF("FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32 "\n",
	    cm->cm_fpb_base, cm->cm_dwt_base);
	tmon_log("Debug resources: FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32
	    "\n", cm->cm_fpb_base, cm->cm_dwt_base);

	t->t_free = cortexm_free;
	t->t_ctl = cortexm_core_ctl;

	cm->cm_flags = CORTEXM_FLAG_ARCH_V6M;
	cm->cm_features = CORTEXM_FEATURE_BASELINE;
	cm->cm_regmap_arch = cortexm_txml_regmap_v6m;

	cm->cm_demcr_cfg_bits = CMSCS_DEMCR_VC_CFG_V6M;
	cm->cm_demcr = (cortexm_vc & CMSCS_DEMCR_VC_CFG_V6M) |
	    CMSCS_DEMCR_VC_CORERESET;
	if (cm->cm_dwt_base != 0) {
		/* We have a DWT. */
		cm->cm_demcr |= CMSCS_DEMCR_DWTENA;
	}

	/*
	 * Now probe the SoC.
	 */
	rv = CORTEXM_SOC_ATTACH_NO_MATCH;
	for (i = 0; i < CORTEXM_V6M_NSOCS; i++) {
		if (cortexm_v6m_socs[i].sp_part == CMSCS_CPUID_PARTNO(cpuid) &&
		    (rv = (cortexm_v6m_socs[i].sp_attach)(cm->cm_target)) != 0){
			/* Found one. */
			break;
		}

		(void) TARGET_LINK_ERROR(cm->cm_target);
	}

	/*
	 * It's not fatal if we have no SoC support. The user can still poke
	 * around with GDB.
	 */
	if (i == CORTEXM_V6M_NSOCS) {
		DBFPRINTF("Warning: unsupported SoC\n");
		tmon_log("Note: Unsupported SoC\n");
	}

	return rv;
}

static bool
cortexm_probe_fp(cortexm_t cm)
{
	uint32_t cpacr, r;

	/*
	 * Follow the procedure outlined in the TRM to determine
	 * if the FPU is implemented:
	 *
	 *	Read CPACR
	 *	Set bits 20-23 to enable CP10 and CP11 coprocessors
	 *	Write back the modified value to the CPACR
	 *
	 * If the bits remain set then we have an FPU.
	 */
	cpacr = cortexm_scs_read(cm, CMSCS_REG_CPACR);
	if (cpacr == 0 && TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("DP error during CPACR read\n");
		return false;
	}
	r = cpacr | (0xfu << 20);
	cortexm_scs_write(cm, CMSCS_REG_CPACR, r);
	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("DP error during CPACR write\n");
		return false;
	}
	r = cortexm_scs_read(cm, CMSCS_REG_CPACR);
	if (r == 0 && TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("DP error during CPACR re-read\n");
	}

	/* Restore original CPACR. */
	cortexm_scs_write(cm, CMSCS_REG_CPACR, cpacr);
	if (TARGET_LINK_ERROR(cm->cm_target))
		DBFPRINTF("DP error during CPACR restore\n");

	/* FPU available? */
	if ((r & (0xfu << 20)) != (0xfu << 20))
		return false;

	/*
	 * CP10 and CP11 indicate presence of FPU.
	 * Probe MVFR0 to determine what type of FPU.
	 */
	r = cortexm_scs_read(cm, CMSCS_REG_MVFR0);

	if (CMSCS_MVFR0_SP_SUPPORTED(r) == 0) {
		/* CP10 and CP11 lied. */
		return false;
	}

	if (CMSCS_MVFR0_DP_SUPPORTED(r))
		cm->cm_flags |= CORTEXM_FLAG_FPU_DP;

	return true;
}

static void
cortexm_probe_arm_cache(cortexm_t cm)
{

	cm->cm_ctr = cortexm_scs_read(cm, CMSCS_REG_CTR);
	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("DP error during CTR read\n");
		return;
	}

	if (CMSCS_CTR_FORMAT_ARM(cm->cm_ctr)) {
		cm->cm_clidr = cortexm_scs_read(cm, CMSCS_REG_CLIDR);
		cm->cm_ccr = cortexm_scs_read(cm, CMSCS_REG_CCR);

		if (CMSCS_CLIDR_CTYPE(cm->cm_clidr, 0) !=
		    CMSCS_CLIDR_CTYPE_NONE) {
			/* Cache exists */
			cm->cm_flags |= CORTEXM_FLAG_HAS_ARM_CACHE;
		}
	} else {
		cm->cm_clidr = 0;
		cm->cm_ccr = 0;
	}
}

static int
cortexm_v7m_attach(cortexm_t cm, uint32_t cpuid, target_addr_t rom_table)
{
	target_t t = cm->cm_target;
	u_int i;
	int rv;

	if (CMSCS_CPUID_ARCHITECTURE(cpuid) != CPUID_ARCH_ARMV7M)
		return -1;

	sprintf(cm->cm_core_name, "Cortex-M%u",
	    CMSCS_CPUID_PARTNO(cpuid) & 0xfu);

	DBPRINTF("\nAttaching %s (CPUID %08" PRIx32 ")\n", cm->cm_core_name,
	    cpuid);
	tmon_log("Attaching %s\n", cm->cm_core_name);

	/*
	 * Ensure AP.CSW "prot" bits are set as per the spec.
	 */
	uint8_t prot = ADIV5_MEMAP_CSW_PROT(0) | ADIV5_MEMAP_CSW_PROT(1) |
	    ADIV5_MEMAP_CSW_PROT(5);
	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_CSW_PROT, &prot) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBPRINTF("Failed to configure AHB3\n");
		return -1;
	}

	/*
	 * Fetch the DWT and FBP addresses from the ROM table.
	 */
	if (cortexm_rte_addr(t, rom_table, CORTEXM_RTE_FPB_IDX,
	    &cm->cm_fpb_base) < 0 ||
	    cortexm_rte_addr(t, rom_table, CORTEXM_RTE_DWT_IDX,
	    &cm->cm_dwt_base) < 0) {
		return -1;
	}

	DBFPRINTF("FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32 "\n",
	    cm->cm_fpb_base, cm->cm_dwt_base);
	tmon_log("Debug resources: FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32
	    "\n", cm->cm_fpb_base, cm->cm_dwt_base);

	t->t_free = cortexm_free;
	t->t_ctl = cortexm_core_ctl;

	cm->cm_flags = CORTEXM_FLAG_ARCH_V7M;
	cm->cm_features = CORTEXM_FEATURE_MAINLINE;
	cm->cm_regmap_arch = cortexm_txml_regmap_v78m;

	cm->cm_demcr_cfg_bits = CMSCS_DEMCR_VC_CFG_V7M;
	cm->cm_demcr = (cortexm_vc | CMSCS_DEMCR_VC_CFG_V7M) |
	    CMSCS_DEMCR_VC_CORERESET;
	if (cm->cm_dwt_base != 0) {
		/* We have a DWT. */
		cm->cm_demcr |= CMSCS_DEMCR_TRCENA;
	}

	/*
	 * Floating point is an option for v7m CPU cores.
	 */
	if (cortexm_probe_fp(cm)) {
		/* FPU is present */
		cm->cm_flags |= CORTEXM_FLAG_HAS_FPU;
	}

	/*
	 * Probe for cache
	 */
	cortexm_probe_arm_cache(cm);

	if (TARGET_LINK_ERROR(cm->cm_target)) {
		DBFPRINTF("DP error while probing\n");
		return -1;
	}

	/*
	 * Now probe the SoC.
	 */
	rv = CORTEXM_SOC_ATTACH_NO_MATCH;
	for (i = 0; i < CORTEXM_V7M_NSOCS; i++) {
		if (cortexm_v7m_socs[i].sp_part == CMSCS_CPUID_PARTNO(cpuid) &&
		    (rv = (cortexm_v7m_socs[i].sp_attach)(cm->cm_target)) != 0){
			/* Found one. */
			break;
		}

		(void) TARGET_LINK_ERROR(cm->cm_target);
	}

	DBFPRINTF("Core has %sFPU\n",
	    (cm->cm_flags & CORTEXM_FLAG_HAS_FPU) ? "" : "no ");
	DBFPRINTF("Core has %scache\n",
	    (cm->cm_flags & CORTEXM_FLAG_HAS_ARM_CACHE) ? "" : "no ");

	/*
	 * It's not fatal if we have no SoC support. The user can still poke
	 * around with GDB.
	 */
	if (i == CORTEXM_V7M_NSOCS) {
		DBFPRINTF("Warning: unsupported SoC\n");
		tmon_log("Note: Unsupported SoC\n");
	}

	return rv;
}

bool
cortexm_v8m_secure_debug_available(cortexm_t cm)
{

	return (cm->cm_features & CORTEXM_FEATURE_SECURITY) != 0 &&
	    (cortexm_dhcsr_read(cm) & CMSCS_DHCSR_S_SDE) != 0;
}

static int
cortexm_v8m_attach(cortexm_t cm, uint32_t cpuid, target_addr_t rom_table)
{
	target_t t = cm->cm_target;
	const char *cn;
	bool has_sec;
	uint8_t prot;
	uint32_t r;
	u_int i;
	int rv;

	switch (CMSCS_CPUID_PARTNO(cpuid)) {
	case CPUID_PARTNO_CM23:
		cn = "23";
		break;
	case CPUID_PARTNO_CM33:
		cn = "33";
		break;
	default:
		return -1;
	}

	switch (CMSCS_CPUID_ARCHITECTURE(cpuid)) {
	case CPUID_ARCH_ARMV8M_WITHOUT_MAIN:
		cm->cm_flags = CORTEXM_FLAG_V8M_EXT_BASE;
		cm->cm_features = CORTEXM_FEATURE_BASELINE;
		break;
	case CPUID_ARCH_ARMV8M_WITH_MAIN:
		cm->cm_flags = CORTEXM_FLAG_V8M_EXT_MAIN;
		cm->cm_features = CORTEXM_FEATURE_MAINLINE;
		break;
	default:
		return -1;
	}

	sprintf(cm->cm_core_name, "Cortex-M%s", cn);

	DBPRINTF("\nAttaching %s (CPUID %08" PRIx32 ")\n", cm->cm_core_name,
	    cpuid);
	tmon_log("Attaching %s\n", cm->cm_core_name);

	/*
	 * Fetch the DWT and BPU addresses from the ROM table.
	 */
	if (cortexm_rte_addr(t, rom_table, CORTEXM_RTE_FPB_IDX,
	    &cm->cm_fpb_base) < 0 ||
	    cortexm_rte_addr(t, rom_table, CORTEXM_RTE_DWT_IDX,
	    &cm->cm_dwt_base) < 0) {
		DBFPRINTF("FPB or DWT not found.\n");
		return -1;
	}

	DBFPRINTF("FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32 "\n",
	    cm->cm_fpb_base, cm->cm_dwt_base);
	tmon_log("Debug resources: FPB @ 0x%08" PRIx32 ", DWT @ 0x%08" PRIx32
	    "\n", cm->cm_fpb_base, cm->cm_dwt_base);

	t->t_free = cortexm_free;
	t->t_ctl = cortexm_core_ctl;
	cm->cm_flags |= CORTEXM_FLAG_ARCH_V8M;
	cm->cm_regmap_arch = cortexm_txml_regmap_v78m;

	cm->cm_demcr_cfg_bits = CMSCS_DEMCR_VC_CFG_V8M;
	cm->cm_demcr = cortexm_vc & CMSCS_DEMCR_VC_CFG_V8M;
	if (cm->cm_dwt_base != 0) {
		/* We have a DWT. */
		cm->cm_demcr |= CMSCS_DEMCR_DWTENA;
	}

	DBFPRINTF("DSCSR %08" PRIx32 ", DHCSR %08" PRIx32 "\n",
	    cortexm_scs_read(cm, CMSCS_REG_DSCSR), cortexm_dhcsr_read(cm));

	/*
	 * Determine if this core has the Security Extension.
	 *
	 * Note that we can only auto-detect on targets with
	 * the main extension (ID_PFR1 is not implemented on
	 * the base platform).
	 *
	 * On baseline targets, SoC code is responsible for
	 * setting the features correctly.
	 */
	if (CORTEXM_V8M_HAS_EXT_MAIN(cm)) {
		r = cortexm_scs_read(cm, CMSCS_REG_ID_PFR1);
		if (r == 0 && TARGET_LINK_ERROR(cm->cm_target)) {
			DBFPRINTF("DP error during ID_PFR1 read\n");
			return -1;
		}
		has_sec = CMSCS_ID_PFR1_SECURITY(r) ==
		    CMSCS_ID_PFR1_SECURITY_AVAIL;
	} else {
		/*
		 * We intuit the Security Extension by examining DHCSR.S_SDE.
		 * This isn't conclusive, so SoC-specific will need to
		 * override the flag if secure debug is unavailable.
		 */
		has_sec = (cortexm_dhcsr_read(cm) & CMSCS_DHCSR_S_SDE) != 0;
	}

	prot = ADIV5_MEMAP_CSW_PROT(0) | ADIV5_MEMAP_CSW_PROT(1);
	if (has_sec) {
		cm->cm_regmap_security = cortexm_txml_regmap_v8m_s;
		cm->cm_flags |= CORTEXM_FLAG_V8M_EXT_SECURITY;
		cm->cm_features |= CORTEXM_FEATURE_SECURITY;
	} else {
		cm->cm_regmap_security = cortexm_txml_regmap_v8m_ns;
		prot |= ADIV5_MEMAP_CSW_PROT(6);	/* SLVNONSEC */
	}

	if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_CSW_PROT, &prot) < 0) {
		(void) TARGET_LINK_ERROR(t);
		DBPRINTF("Failed to configure Mem-AP.CSW\n");
		return -1;
	}

	/*
	 * Floating point is only available with the Main extension.
	 */
	if (CORTEXM_V8M_HAS_EXT_MAIN(cm) && cortexm_probe_fp(cm)) {
		/* FPU is present */
		cm->cm_flags |= CORTEXM_FLAG_HAS_FPU;
	}

	/*
	 * Probe for cache
	 */
	cortexm_probe_arm_cache(cm);

	/*
	 * Now probe the SoC.
	 */
	rv = CORTEXM_SOC_ATTACH_NO_MATCH;
	for (i = 0; i < CORTEXM_V8M_NSOCS; i++) {
		if (cortexm_v8m_socs[i].sp_part == CMSCS_CPUID_PARTNO(cpuid) &&
		    (rv = (cortexm_v8m_socs[i].sp_attach)(cm->cm_target)) != 0){
			/* Found one. */
			break;
		}

		(void) TARGET_LINK_ERROR(cm->cm_target);
	}

	/*
	 * It's not fatal if we have no SoC support. The user can still poke
	 * around with GDB.
	 */
	if (i == CORTEXM_V8M_NSOCS) {
		DBFPRINTF("Warning: unsupported SoC\n");
		tmon_log("Note: Unsupported SoC\n");
	}

	if (i == CORTEXM_V8M_NSOCS || rv == CORTEXM_SOC_ATTACH_OK) {
		/*
		 * SoC code for baseline v8m devices will have determined
		 * whether or nor the security extension is available.
		 * In which case, we need to check if secure debug is
		 * available.
		 */
		if ((cm->cm_flags & CORTEXM_FLAG_V8M_EXT_SECURITY) == 0) {
			cm->cm_features &= ~CORTEXM_FEATURE_SECURITY;
			cm->cm_regmap_security = cortexm_txml_regmap_v8m_ns;
			prot |= ADIV5_MEMAP_CSW_PROT(6);	/* SLVNONSEC */

			if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_CSW_PROT,
			    &prot) < 0) {
				(void) TARGET_LINK_ERROR(t);
				DBPRINTF("Failed to configure Mem-AP.CSW\n");
				return -1;
			}
		} else
		if (cortexm_v8m_secure_debug_available(cm)) {
			cm->cm_demcr |= CMSCS_DEMCR_VC_SFERR;
			cortexm_vc |= CMSCS_DEMCR_VC_SFERR;
		}
	}

	return rv;
}

int
cortexm_attach(target_t t, target_addr_t rom_table)
{
	cortexm_t cm;
	uint32_t cpuid;
	unsigned int a;
	int rv;

	if ((cm = zone_calloc(1, sizeof(*cm))) == NULL)
		return 0;

	cm->cm_target = t;

#ifdef CONFIG_USE_CONFIGDB
	if (!cortexm_cf_done) {
		cortexm_cf_done = true;
		if (cf_get_uint32(CF_KEY_UINT32_CORTEXM_FLAGS,
		    &cortexm_gflags) == 0 ||
		    cf_get_uint32(CF_KEY_UINT32_CORTEXM_VC,
		    &cortexm_vc) == 0) {
			cortexm_load_defaults();
			cf_get_uint32(CF_KEY_UINT32_CORTEXM_FLAGS,
			    &cortexm_gflags);
			cf_get_uint32(CF_KEY_UINT32_CORTEXM_VC,
			    &cortexm_vc);
		}
	}
#endif

	/*
	 * The first ROM table entry is the SCS. It must exist and must
	 * be enabled.
	 */
	if (cortexm_rte_addr(t, rom_table, CORTEXM_RTE_SCS_IDX,
	    &cm->cm_scs_base) == 0) {
		DBFPRINTF("no SCS component!\n");
		(void) TARGET_LINK_ERROR(t);
		zone_free(cm);
		tmon_log("No SCS component. Probably not a CPU core.\n");
		return -1;
	}

	/* Now that we have SCS space available, read the CPUID */
	cpuid = cortexm_scs_read(cm, CMSCS_REG_CPUID);
	if (TARGET_LINK_ERROR(t)) {
		DBFPRINTF("failed to read CPUID!\n");
		zone_free(cm);
		tmon_log("Failed to read the CPU ID!\n");
		return -1;
	}

	DBFPRINTF("CPUID %08" PRIx32 "\n", cpuid);
	tmon_log("Cortex-M CPUID: %08" PRIx32 " - ", cpuid);

	if (CMSCS_CPUID_IMPLEMENTER(cpuid) != CPUID_IMPLEMENTER_ARM) {
		DBFPRINTF("unsupported CPUID: %08" PRIx32 "\n", cpuid);
		zone_free(cm);
		tmon_log("Not an ARM device.\n");
		return -1;
	}

	t->t_core = cm;
	cm->cm_cpuid = cpuid;
	cm->cm_regmap_security = NULL;
	cm->cm_last_halt_reason = TARGET_HALT_STATE_RUNNING;
	cm->cm_core_reset = cortexm_core_reset_generic;

	switch (CMSCS_CPUID_PARTNO(cpuid)) {
	case CPUID_PARTNO_CM0:
	case CPUID_PARTNO_CM0_PLUS:
		tmon_log("armv6m\n");
		rv = cortexm_v6m_attach(cm, cpuid, rom_table);
		break;

	case CPUID_PARTNO_CM3:
	case CPUID_PARTNO_CM4:
		/* Cortex-M3/M4 have 4KB TAR Auto-increment. */
		a = 12;
		TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_SET_TAR_BITS, &a);
		/*FALLTHROUGH*/

	case CPUID_PARTNO_CM7:
		tmon_log("armv7m\n");
		rv = cortexm_v7m_attach(cm, cpuid, rom_table);
		break;

	case CPUID_PARTNO_CM23:
	case CPUID_PARTNO_CM33:
		tmon_log("armv8m\n");

		rv = cortexm_v8m_attach(cm, cpuid, rom_table);
		break;

	default:
		DBFPRINTF("Cortex-M? not supported (CPUID %08" PRIx32 ")\n",
		    cpuid);
		tmon_log("Unsupported core.\n");
		rv = -1;
		break;
	}

	/*
	 * Clear any lingering errors after probing for SoCs.
	 */
	(void) TARGET_LINK_ERROR(cm->cm_target);

	if (rv < 0) {
		t->t_core = NULL;
		zone_free(cm);
	} else {
		rv = target_degap_memory(0x0, 0xffffffffu);
	}

	if (rv >= 0) {
		uint32_t mpu_type;

		mpu_type = cortexm_scs_read(cm, CMSCS_REG_MPU_TYPE);
		if (TARGET_LINK_ERROR(cm->cm_target) == 0 &&
		    CMSCS_MPU_TYPE_DREGION(mpu_type) != 0) {
			cm->cm_flags |= CORTEXM_FLAG_HAS_MPU;
			cm->cm_mpu_regions = CMSCS_MPU_TYPE_DREGION(mpu_type);
		}

		cortexm_core_fpb_init(cm);
		cortexm_core_dwt_init(cm);
		if (cortexm_cmd_count++ == 0)
			SHELL_CMD_ADD(cortexm);
	}

	return rv;
}

static const char cortexm_cmd_usage[] =
"usage:\n"
"cortexm maskisr <on | off>\n"
"\tConfigures interrupt masking behaviour during single-step.\n"
"\tDefault is 'on'.\n"
#if 0
"cortexm reset_config <srst | sysresetreq | vectreset>\n"
"\tDetermines which method to use to reset the target.\n"
"\tDefault is 'sysresetreq'.\n"
#endif
"cortexm vector_catch <vector> [...]\n"
"\tConfigures which error vectors to hook when target is running.\n"
"\tOptions for <vector> are:\n"
"\t  harderr  Debug trap on HardFault exception\n"
"\t  interr   Debug trap on fault during exception entry/exit\n"
"\t  buserr   Debug trap on BusFault exception\n"
"\t  staterr  Debug trap on UsageFault exception (state error)\n"
"\t  chkerr   Debug trap on UsageFault exception (check error)\n"
"\t  nocperr  Debug trap on UsageFault exception (copro error)\n"
"\t  mmerr    Debug trap on MemManage exception\n"
"\t  all      Enable all of the above\n"
"\t  none     Disable all of the above\n"
"\tPreceding a vector name with a dash (-) will remove the hook for\n"
"\tthat vector. The default setting is 'all'. You can specify multiple\n"
"\tvectors, each separated by a space. Note that ARMv6-M devices\n"
"\t(Cortex-M0, for example) only support debug trap on 'harderr'; all\n"
"\tother trap types are ignored.\n";

static const struct {
	const char *name;
	uint32_t bits;
} cortexm_vc_map[] = {
	{"all",
	 CMSCS_DEMCR_VC_HARDERR | CMSCS_DEMCR_VC_INTERR |
	 CMSCS_DEMCR_VC_BUSERR | CMSCS_DEMCR_VC_STATERR |
	 CMSCS_DEMCR_VC_CHKERR | CMSCS_DEMCR_VC_NOCPERR |
	 CMSCS_DEMCR_VC_MMERR},
	{"none", 0},
	{"harderr", CMSCS_DEMCR_VC_HARDERR},
	{"interr", CMSCS_DEMCR_VC_INTERR},
	{"buserr", CMSCS_DEMCR_VC_BUSERR},
	{"staterr", CMSCS_DEMCR_VC_STATERR},
	{"chkerr", CMSCS_DEMCR_VC_CHKERR},
	{"nocperr", CMSCS_DEMCR_VC_NOCPERR},
	{"mmerr", CMSCS_DEMCR_VC_MMERR}
};
#define	CORTEXM_VC_MAP_COUNT	(sizeof(cortexm_vc_map) / \
				 sizeof(cortexm_vc_map[0]))

static int
cortexm_cmd_parse_vc(FILE *os, uint8_t argc, const char * const *argv,
    uint32_t *pset, uint32_t *pclr)
{
	uint32_t setbits = 0, clrbits = 0;

	while (argc--) {
		const char *str;
		unsigned int i;
		bool clear;
		size_t l;

		str = *argv++;
		clear = str[0] == '-';
		if (clear)
			str++;
		l = strlen(str);
		if (l == 0)
			break;

		for (i = 0; i < CORTEXM_VC_MAP_COUNT; i++) {
			if (strncasecmp(cortexm_vc_map[i].name, str, l) == 0)
				break;
		}

		if (i == CORTEXM_VC_MAP_COUNT) {
			fprintf(os, "Unrecognised vector: '%s'\n", str);
			return -1;
		}

		if (clear) {
			clrbits |= cortexm_vc_map[i].bits;
			setbits &= ~cortexm_vc_map[i].bits;
		} else {
			setbits |= cortexm_vc_map[i].bits;
			clrbits &= ~cortexm_vc_map[i].bits;
		}
	}

	*pset = setbits;
	*pclr = clrbits;

	return 0;
}

static void
cortexm_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	static const char *endis[2] = {"Disabled", "Enabled"};
	static const char *autoonoff[4] = {"Auto", "On", "Off", "??"};
	static const char *restcfg[4] = {"sRST", "sysresetreq", "vectreset","?"};
	uint32_t setbits, clrbits, y;
	unsigned int x;
	size_t l;

	if (argc) {
		l = strlen(argv[0]);
		if (argc == 2 && strncasecmp("maskisr", argv[0], l) == 0) {
			l = strlen(argv[1]);
#if 0
			if (strncasecmp(autoonoff[0], argv[1], l) == 0)
				x = CORTEXM_GFLAG_MASK_ISR_AUTO;
			else
#endif
			if (strncasecmp(autoonoff[1], argv[1], l) == 0)
				x = CORTEXM_GFLAG_MASK_ISR_ON;
			else
			if (strncasecmp(autoonoff[2], argv[1], l) == 0)
				x = CORTEXM_GFLAG_MASK_ISR_OFF;
			else
				goto usage;

			if (x != (cortexm_gflags &
			    CORTEXM_GFLAG_MASK_ISR_MASK)) {
				rtos_scheduler_suspend();
				cortexm_gflags &= ~CORTEXM_GFLAG_MASK_ISR_MASK;
				cortexm_gflags |= x;
				rtos_scheduler_resume();
#ifndef CONFIG_USE_CONFIGDB
				glob_save();
#else
				cf_set_uint32(CF_KEY_UINT32_CORTEXM_FLAGS,
				    cortexm_gflags);
				configdb_persist();
#endif
			}
		} else
		if (argc == 2 && strncasecmp("reset_config", argv[0], l) == 0) {
			l = strlen(argv[1]);
			if (strncasecmp(restcfg[0], argv[1], l) == 0)
				x = CORTEXM_GFLAG_RESET_CFG_SRST;
			else
			if (strncasecmp(restcfg[1], argv[1], l) == 0)
				x = CORTEXM_GFLAG_RESET_CFG_SYSRESETREQ;
			else
			if (strncasecmp(restcfg[2], argv[1], l) == 0)
				x = CORTEXM_GFLAG_RESET_CFG_VECTRESET;
			else
				goto usage;

			if (x != (cortexm_gflags &
			    CORTEXM_GFLAG_RESET_CFG_MASK)) {
				rtos_scheduler_suspend();
				cortexm_gflags &= ~CORTEXM_GFLAG_RESET_CFG_MASK;
				cortexm_gflags |= x;
				rtos_scheduler_resume();
#ifndef CONFIG_USE_CONFIGDB
				glob_save();
#else
				cf_set_uint32(CF_KEY_UINT32_CORTEXM_FLAGS,
				    cortexm_gflags);
				configdb_persist();
#endif
			}
		} else
		if (argc >= 2 && strncasecmp("vector_catch", argv[0], l) == 0) {
			if (cortexm_cmd_parse_vc(os, argc-1, &argv[1], &setbits,
			    &clrbits) < 0)
				goto usage;

			y = (cortexm_vc | setbits) & ~clrbits;
			if (y != cortexm_vc) {
				rtos_scheduler_suspend();
				cortexm_vc = y;
				rtos_scheduler_resume();
#ifndef CONFIG_USE_CONFIGDB
				glob_save();
#else
				cf_set_uint32(CF_KEY_UINT32_CORTEXM_VC,
				    cortexm_vc);
				configdb_persist();
#endif
			}
		} else
#ifdef DEBUG_ENABLED
		if (argc == 2 && strncasecmp("debug", argv[0], l) == 0) {
			DEBUG_FLAG = strtoul(argv[1], NULL, 0);
			fprintf(os, "Debug O%s\n", DEBUG_FLAG ? "n" : "ff");
		} else
#endif
		{
 usage:
			fputs(cortexm_cmd_usage, os);
			return;
		}
	}

	x = cortexm_gflags & CORTEXM_GFLAG_MASK_ISR_MASK;
	x >>= CORTEXM_GFLAG_MASK_ISR_SHIFT;
	fprintf(os, "Mask ISR: %s\n", autoonoff[x]);

#if 0
	x = cortexm_gflags & CORTEXM_GFLAG_RESET_CFG_MASK;
	x >>= CORTEXM_GFLAG_RESET_CFG_SHIFT;
	fprintf(os, "Reset config: %s\n", restcfg[x]);
#endif

	y = cortexm_vc;
	fprintf(os, "Vector catch config:\n"
	    "\tharderr: %s\n"
	    "\t interr: %s\n"
	    "\t buserr: %s\n"
	    "\tstaterr: %s\n"
	    "\t chkerr: %s\n"
	    "\tnocperr: %s\n"
	    "\t  mmerr: %s\n",
	    endis[(y & CMSCS_DEMCR_VC_HARDERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_INTERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_BUSERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_STATERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_CHKERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_NOCPERR) ? 1 : 0],
	    endis[(y & CMSCS_DEMCR_VC_MMERR) ? 1 : 0]);
}

#ifdef NETWORK_OPT_HTTPD
int
cortexm_update_settings(int nkeys,
    const struct network_http_furl_key_value *kv)
{
	uint32_t vc;
	uint8_t isr_mask;
	bool vc_seen, isr_seen;
	unsigned int v, i;
	int changed;

	vc = 0;
	vc_seen = false;
	isr_mask = 0;
	isr_seen = false;
	changed = 0;

	while (nkeys--) {
		if (strcmp(kv->kv_key, "gdb-vcatch") == 0 &&
		   strncmp("vc-", kv->kv_value, 3) == 0) {
			for (i = 0; i < CORTEXM_VC_MAP_COUNT; i++) {
				if (strcmp(cortexm_vc_map[i].name,
				    &kv->kv_value[3]) == 0) {
					vc |= cortexm_vc_map[i].bits;
					vc_seen = true;
					break;
				}
			}
		} else
		if (strcmp(kv->kv_key, "gdb-stepmask") == 0 &&
		    sscanf(kv->kv_value, "%u", &v) == 1 && (v == 0 || v == 1)) {
			isr_mask = v ? CORTEXM_GFLAG_MASK_ISR_ON :
			    CORTEXM_GFLAG_MASK_ISR_OFF;
			isr_seen = true;
		}

		kv++;
	}


	if (vc_seen) {
		uint32_t cur_vc;

		if (cf_get_uint32(CF_KEY_UINT32_CORTEXM_VC, &cur_vc) == 0) {
			cortexm_load_defaults();
			cf_get_uint32(CF_KEY_UINT32_CORTEXM_VC, &cur_vc);
		}

		if (vc != cur_vc) {
			changed++;

			cf_set_uint32(CF_KEY_UINT32_CORTEXM_VC, vc);
			rtos_scheduler_suspend();
			cortexm_vc = vc;
			rtos_scheduler_resume();
		}
	}

	if (isr_seen) {
		uint32_t gflags;

		if (cf_get_uint32(CF_KEY_UINT32_CORTEXM_FLAGS, &gflags) == 0) {
			cortexm_load_defaults();
			cf_get_uint32(CF_KEY_UINT32_CORTEXM_FLAGS, &gflags);
		}

		if (isr_mask != (gflags & CORTEXM_GFLAG_MASK_ISR_MASK)) {
			gflags &= ~CORTEXM_GFLAG_MASK_ISR_MASK;
			gflags |= isr_mask;
			changed++;

			cf_set_uint32(CF_KEY_UINT32_CORTEXM_VC, gflags);
			rtos_scheduler_suspend();
			cortexm_gflags = gflags;
			rtos_scheduler_resume();
		}
	}

	return changed;
}

void
cortexm_debug_update(stringio_t ss)
{
	uint32_t gflags, vc;

	if (cf_get_uint32(CF_KEY_UINT32_CORTEXM_FLAGS, &gflags) == 0)
		gflags = cortexm_gflags;
	gflags &= CORTEXM_GFLAG_MASK_ISR_MASK;

	if (cf_get_uint32(CF_KEY_UINT32_CORTEXM_VC, &vc) == 0)
		vc = cortexm_vc;

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("stepisr",
	    (gflags == CORTEXM_GFLAG_MASK_ISR_ON) ? "mask" :
	    ((gflags == CORTEXM_GFLAG_MASK_ISR_OFF) ? "unmask" : "auto")));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_hard", vc & CMSCS_DEMCR_VC_HARDERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_int", vc & CMSCS_DEMCR_VC_INTERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_bus", vc & CMSCS_DEMCR_VC_BUSERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_stat", vc & CMSCS_DEMCR_VC_STATERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_chk", vc & CMSCS_DEMCR_VC_CHKERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_nocp", vc & CMSCS_DEMCR_VC_NOCPERR));

	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("vc_mm", vc & CMSCS_DEMCR_VC_MMERR));
}
#endif /* NETWORK_OPT_HTTPD */

void
cortexm_load_defaults(void)
{
	const unsigned int gflags = CORTEXM_GFLAG_MASK_ISR_ON |
	    CORTEXM_GFLAG_RESET_CFG_SYSRESETREQ;
	const uint32_t vc = CMSCS_DEMCR_VC_HARDERR | CMSCS_DEMCR_VC_INTERR |
	    CMSCS_DEMCR_VC_BUSERR | CMSCS_DEMCR_VC_STATERR |
	    CMSCS_DEMCR_VC_CHKERR | CMSCS_DEMCR_VC_NOCPERR |
	    CMSCS_DEMCR_VC_MMERR;

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_cortexm_flags = gflags;
	glob.glob_cortexm_vc = vc;
#else
	cf_set_uint32(CF_KEY_UINT32_CORTEXM_FLAGS, gflags);
	cf_set_uint32(CF_KEY_UINT32_CORTEXM_VC, vc);
#endif
}
