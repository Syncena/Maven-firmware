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
#include <stdlib.h>
#include <string.h>

#include "cortexm.h"
#include "cortexm_regs.h"
#include "cortexm_txml.h"
#include "cortexm_rtos.h"
#include "cortexm_rtos_nuttx.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * First entry in the NuttX core register set is the SP value *before* the
 * exception, without taking into account xPSR[9]. It's of no use to us
 * so we simply ignore it.
 *
 * The second entry is normally PRIMASK, but could in fact be BASEPRI on
 * V7m or V8m cores if CONFIG_ARMV7M_USEBASEPRI/CONFIG_ARMV8M_USEBASEPRI are
 * defined.
 *
 * EXC_LR is always stored in the core frame on V7m/V8m.
 * On V6m cores, EXC_LR is stored if CONFIG_BUILD_PROTECTED is defined.
 * We don't need EXC_LR on V6m, but we still need to grok the layout.
 *
 * On V8m, PSPLIM or MSPLIM is stored after EXC_LR in the core frame, but
 * only if CONFIG_ARMV8M_STACKCHECK_HARDWARE is defined. Which one is stored
 * depends on which SP was active before the exception.
 *
 * I'm sure there's a reason for the complexity...
 */

#define	CORTEXM_FRAME_NUTTX_CORE(base)		\
	{CORTEXM_REG_PRIMASK, (base) + 0x00},	\
	{CORTEXM_REG_R4,      (base) + 0x04},	\
	{CORTEXM_REG_R5,      (base) + 0x08},	\
	{CORTEXM_REG_R6,      (base) + 0x0c},	\
	{CORTEXM_REG_R7,      (base) + 0x10},	\
	{CORTEXM_REG_R8,      (base) + 0x14},	\
	{CORTEXM_REG_R9,      (base) + 0x18},	\
	{CORTEXM_REG_R10,     (base) + 0x1c},	\
	{CORTEXM_REG_R11,     (base) + 0x20}
#define	CORTEXM_FRAME_NUTTX_EXC_LR		0x24
#define	CORTEXM_FRAME_NUTTX_V6_CORE_LEN		0x24
#define	CORTEXM_FRAME_NUTTX_CORE_LEN		0x28

/*
 * The V8m layout. Used only if NuttX is built with the
 * CONFIG_ARMV8M_STACKCHECK_HARDWARE option.
 * Largely the same as CORTEXM_FRAME_NUTTX_CORE(), but with either
 * MSPLIM or PSPLIM following EXC_LR. Which one is stored depends which
 * stack was active before the exception. Hence EXC_LR will need to
 * be consulted. We will default to PSPLIM and fix-up the correct
 * register enum at runtime.
 *
 * Note that CORTEXM_REG_PSPLIM_S and CORTEXM_REG_PSPLIM_NS *must* be
 * the first two elements of CORTEXM_FRAME_NUTTX_V8_CORE().
 */
#define	CORTEXM_FRAME_NUTTX_V8_CORE(base)	\
	{CORTEXM_REG_PSPLIM_S,  (base) + CORTEXM_FRAME_NUTTX_CORE_LEN}, \
	{CORTEXM_REG_PSPLIM_NS, (base) + CORTEXM_FRAME_NUTTX_CORE_LEN}, \
	CORTEXM_FRAME_NUTTX_CORE(base)
#define	CORTEXM_FRAME_NUTTX_V8_CORE_LEN	(CORTEXM_FRAME_NUTTX_CORE_LEN + 4)

#define	CORTEXM_FRAME_NUTTX_FP(base)		\
	{CORTEXM_REG_VFP_D8,  (base) + 0x00},	\
	{CORTEXM_REG_VFP_D9,  (base) + 0x08},	\
	{CORTEXM_REG_VFP_D10, (base) + 0x10},	\
	{CORTEXM_REG_VFP_D11, (base) + 0x18},	\
	{CORTEXM_REG_VFP_D12, (base) + 0x20},	\
	{CORTEXM_REG_VFP_D13, (base) + 0x28},	\
	{CORTEXM_REG_VFP_D14, (base) + 0x30},	\
	{CORTEXM_REG_VFP_D15, (base) + 0x38}
#define	CORTEXM_FRAME_NUTTX_FP_LEN		0x40

/*
 * Used for V6m if CONFIG_BUILD_PROTECTED is *not* defined.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_v6m[] = {
	CORTEXM_FRAME_NUTTX_CORE(0x00),
	CORTEXM_FRAME_CPU(CORTEXM_FRAME_NUTTX_V6_CORE_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_V6_CORE_LEN +
			 CORTEXM_FRAME_CPU_LEN}
};
#define	CORTEXM_RTOS_NUTTX_V6M_COUNT	\
	(sizeof(cortexm_rtos_nuttx_v6m) / sizeof(cortexm_rtos_nuttx_v6m[0]))

/*
 * - Used for V6m if CONFIG_BUILD_PROTECTED is defined
 * - Used for V7m with no FPU available/used
 * - Used for V8m with no FPU available/used and
 *   CONFIG_ARMV8M_STACKCHECK_HARDWARE is not defined.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_no_fp[] = {
	CORTEXM_FRAME_NUTTX_CORE(0x00),
	CORTEXM_FRAME_CPU(CORTEXM_FRAME_NUTTX_CORE_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_CORE_LEN +
			 CORTEXM_FRAME_CPU_LEN}
};
#define	CORTEXM_RTOS_NUTTX_NO_FP_COUNT	\
	(sizeof(cortexm_rtos_nuttx_no_fp) / sizeof(cortexm_rtos_nuttx_no_fp[0]))

/*
 * FPU used in non-lazy mode.
 *
 * - Used for V7m.
 * - Used for V8m when CONFIG_ARMV8M_STACKCHECK_HARDWARE is not defined.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_fp[] = {
	CORTEXM_FRAME_NUTTX_CORE(0x00),
	CORTEXM_FRAME_NUTTX_FP(CORTEXM_FRAME_NUTTX_CORE_LEN),
	CORTEXM_FRAME_EXT(CORTEXM_FRAME_NUTTX_CORE_LEN +
			  CORTEXM_FRAME_NUTTX_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_CORE_LEN +
			 CORTEXM_FRAME_NUTTX_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
/*
 * FPU used in lazy-save mode.
 *
 * - Used for V7m.
 * - Used for V8m when CONFIG_ARMV8M_STACKCHECK_HARDWARE is not defined.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_lazyfp[] = {
	CORTEXM_FRAME_NUTTX_CORE(0x00),
	CORTEXM_FRAME_NUTTX_FP(CORTEXM_FRAME_NUTTX_CORE_LEN),
	CORTEXM_FRAME_EXT_LAZY(CORTEXM_FRAME_NUTTX_CORE_LEN +
			       CORTEXM_FRAME_NUTTX_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_CORE_LEN +
			 CORTEXM_FRAME_NUTTX_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static_assert(sizeof(cortexm_rtos_nuttx_fp) ==
	      sizeof(cortexm_rtos_nuttx_lazyfp),
	      "FP regframe size mismatch");
#define	CORTEXM_RTOS_NUTTX_FP_COUNT	\
	(sizeof(cortexm_rtos_nuttx_fp) / sizeof(cortexm_rtos_nuttx_fp[0]))

/*
 * Used for V8m with no FPU available/used and
 * CONFIG_ARMV8M_STACKCHECK_HARDWARE is defined.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_v8m_no_fp[] = {
	CORTEXM_FRAME_NUTTX_V8_CORE(0x00),
	CORTEXM_FRAME_CPU(CORTEXM_FRAME_NUTTX_V8_CORE_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_V8_CORE_LEN +
			 CORTEXM_FRAME_CPU_LEN}
};
#define	CORTEXM_RTOS_NUTTX_V8M_NO_FP_COUNT	\
	(sizeof(cortexm_rtos_nuttx_v8m_no_fp) /	\
	 sizeof(cortexm_rtos_nuttx_v8m_no_fp[0]))

/*
 * V8m CONFIG_ARMV8M_STACKCHECK_HARDWARE defined, FPU used in non-lazy mode.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_v8m_fp[] = {
	CORTEXM_FRAME_NUTTX_V8_CORE(0x00),
	CORTEXM_FRAME_NUTTX_FP(CORTEXM_FRAME_NUTTX_V8_CORE_LEN),
	CORTEXM_FRAME_EXT(CORTEXM_FRAME_NUTTX_V8_CORE_LEN +
			  CORTEXM_FRAME_NUTTX_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_V8_CORE_LEN +
			 CORTEXM_FRAME_NUTTX_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
/*
 * V8m CONFIG_ARMV8M_STACKCHECK_HARDWARE defined, FPU used in lazy-save mode.
 */
static const struct cortexm_txml_regframe cortexm_rtos_nuttx_v8m_lazyfp[] = {
	CORTEXM_FRAME_NUTTX_V8_CORE(0x00),
	CORTEXM_FRAME_NUTTX_FP(CORTEXM_FRAME_NUTTX_V8_CORE_LEN),
	CORTEXM_FRAME_EXT_LAZY(CORTEXM_FRAME_NUTTX_V8_CORE_LEN +
			       CORTEXM_FRAME_NUTTX_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_NUTTX_V8_CORE_LEN +
			 CORTEXM_FRAME_NUTTX_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static_assert(sizeof(cortexm_rtos_nuttx_v8m_fp) ==
	      sizeof(cortexm_rtos_nuttx_v8m_lazyfp),
	      "V8M FP regframe size mismatch");
#define	CORTEXM_RTOS_NUTTX_V8M_FP_COUNT	\
	(sizeof(cortexm_rtos_nuttx_v8m_fp) /	\
	 sizeof(cortexm_rtos_nuttx_v8m_fp[0]))

static struct cortexm_txml_regframe *
cortexm_nuttx_copy_regframe(const struct cortexm_txml_regframe *src,
    size_t size)
{
	struct cortexm_txml_regframe *rf;

	if ((rf = zone_malloc(size)) != NULL)
		memcpy(rf, src, size);

	return rf;
}

const struct cortexm_txml_regframe *
cortexm_nuttx_get_regframe(cortexm_t cm, const struct target_rtos *tr,
    unsigned int *frame_count, bool *dynamic)
{
	const struct cortexm_txml_regframe *rf = NULL;
	struct cortexm_txml_regframe *new_rf;
	bool has_fpu, ext_frame, state_valid, use_basepri;
	struct target_mem_readwrite mr;
	unsigned int count = 0;
	gdb_rtos_params_t rp;
	target_addr_t addr;
	uint32_t exc_lr;
	size_t rfsize;

	/* Skip the SP saved at the start of the frame. */
	addr = tr->tr_frame + 4;
	cm->cm_rtos_frame_base = addr;
	rp = tr->tr_params;
	assert(rp != NULL);

	has_fpu = cortexm_rtos_get_fp_status(cm,
	    addr + CORTEXM_FRAME_NUTTX_EXC_LR,
	    addr + CORTEXM_FRAME_NUTTX_CORE_LEN + CORTEXM_FRAME_NUTTX_FP_LEN,
	    &ext_frame, &state_valid);

	/*
	 * Default to the lowest common denominator.
	 */
	count = CORTEXM_RTOS_NUTTX_NO_FP_COUNT;
	rf = cortexm_rtos_nuttx_no_fp;
	rfsize = sizeof(cortexm_rtos_nuttx_no_fp);
	*dynamic = false;
	use_basepri = rp->rp_val[GDB_RTOS_NUTTX_config_armv78m_usebasepri] != 0;

	switch (CORTEXM_FLAG_ARCH(cm)) {
	case CORTEXM_FLAG_ARCH_V6M:
		if (!rp->rp_val[GDB_RTOS_NUTTX_config_build_protected]) {
			count = CORTEXM_RTOS_NUTTX_V6M_COUNT;
			rf = cortexm_rtos_nuttx_v6m;
		}
		break;

	case CORTEXM_FLAG_ARCH_V7M:
 normal_frame:
		if (has_fpu && ext_frame) {
			/* Extended frame. */
			count = CORTEXM_RTOS_NUTTX_FP_COUNT;

			if (state_valid) {
				rf = cortexm_rtos_nuttx_fp;
				rfsize = sizeof(cortexm_rtos_nuttx_fp);
			} else {
				/*
				 * FPU state was not saved, therefore the
				 * current FPU registers should be used.
				 */
				rf = cortexm_rtos_nuttx_lazyfp;
				rfsize = sizeof(cortexm_rtos_nuttx_lazyfp);
			}
		}

		if (use_basepri &&
		    (new_rf = cortexm_nuttx_copy_regframe(rf, rfsize)) != NULL){
			*dynamic = true;
			/* Change first item to BASEPRI. */
			new_rf[0].rf_reg = CORTEXM_REG_BASEPRI;
			rf = new_rf;
		}
		break;

	case CORTEXM_FLAG_ARCH_V8M:
		if (!rp->rp_val[GDB_RTOS_NUTTX_config_armv8m_stackcheck_hardware])
			goto normal_frame;

		count = CORTEXM_RTOS_NUTTX_V8M_NO_FP_COUNT;
		rf = cortexm_rtos_nuttx_v8m_no_fp;
		rfsize = sizeof(cortexm_rtos_nuttx_v8m_no_fp);

		if (has_fpu && ext_frame) {
			/* Extended frame. */
			count = CORTEXM_RTOS_NUTTX_V8M_FP_COUNT;

			if (state_valid) {
				rf = cortexm_rtos_nuttx_v8m_fp;
				rfsize = sizeof(cortexm_rtos_nuttx_v8m_fp);
			} else {
				/*
				 * FPU state was not saved, therefore the
				 * current FPU registers should be used.
				 */
				rf = cortexm_rtos_nuttx_v8m_lazyfp;
				rfsize = sizeof(cortexm_rtos_nuttx_v8m_lazyfp);
			}
		}

		/*
		 * Examine EXC_LR to determine if we need to change
		 * PSPLIM -> MSPLIM in the regframe.
		 */
		mr.mr_write = false;
		mr.mr_target_addr = addr + CORTEXM_FRAME_NUTTX_EXC_LR;
		mr.mr_length = sizeof(uint32_t);
		mr.mr_dest = &exc_lr;
		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE,
		    &mr) == 0 &&
		    ((exc_lr & (1u << 2)) != 0 || use_basepri) &&
		    (new_rf = cortexm_nuttx_copy_regframe(rf, rfsize))!= NULL) {
			if ((exc_lr & (1u << 2)) != 0) {
				/* Need to change PSPLIM* to MSMLIM */
				new_rf[0].rf_reg = CORTEXM_REG_MSPLIM_S;
				new_rf[1].rf_reg = CORTEXM_REG_MSPLIM_NS;
			}
			if (use_basepri)
				new_rf[2].rf_reg = CORTEXM_REG_BASEPRI;

			*dynamic = true;
			rf = new_rf;
		}
		break;

	default:
		return NULL;
	}

	*frame_count = count;
	return rf;
}

int
cortexm_nuttx_load_params(cortexm_t cm,
    const struct gdb_rtos_nuttx_target_params *tp)
{
	struct gdb_rtos_nuttx_debugger_hints *hints = tp->tp_hints;
	uint16_t u16;

	(void) cm;

        u16 = hints->sizeof_far_pointer;
        if (u16 != 4) {
                DBFPRINTF("Bad sizeof_far_pointer: %" PRIu16 "\n", u16);
                return -1;
        }

        u16 = hints->offsetof_xcptcontext_regs;
        if (u16 > 255) {
                DBFPRINTF("Bad offsetof_xcptcontext_regs: %" PRIu16 "\n", u16);
                return -1;
        }

	/* XXX: We should be able to nail this size perfectly... */
        u16 = hints->sizeof_xcptcontext_regs;
        if (u16 > 511) {
                DBFPRINTF("Bad sizeof_xcptcontext_regs: %" PRIu16 "\n", u16);
                return false;
        }

	return 0;
}
