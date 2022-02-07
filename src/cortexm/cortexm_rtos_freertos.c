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
#include "cortexm_rtos_freertos.h"

#define	CORTEXM_FRAME_FREERTOS_CORE(base)	\
	{CORTEXM_REG_R4,  (base) + 0x00},	\
	{CORTEXM_REG_R5,  (base) + 0x04},	\
	{CORTEXM_REG_R6,  (base) + 0x08},	\
	{CORTEXM_REG_R7,  (base) + 0x0c},	\
	{CORTEXM_REG_R8,  (base) + 0x10},	\
	{CORTEXM_REG_R9,  (base) + 0x14},	\
	{CORTEXM_REG_R10, (base) + 0x18},	\
	{CORTEXM_REG_R11, (base) + 0x1c}
	/* EXC_LR is stored at 0x20, but is of no interest to GDB. */
#define	CORTEXM_FRAME_FREERTOS_EXC_LR		0x20
#define	CORTEXM_FRAME_FREERTOS_CORE_LEN		0x24

#define	CORTEXM_FRAME_FREERTOS_FP(base)		\
	{CORTEXM_REG_VFP_D8,  (base) + 0x00},	\
	{CORTEXM_REG_VFP_D9,  (base) + 0x08},	\
	{CORTEXM_REG_VFP_D10, (base) + 0x10},	\
	{CORTEXM_REG_VFP_D11, (base) + 0x18},	\
	{CORTEXM_REG_VFP_D12, (base) + 0x20},	\
	{CORTEXM_REG_VFP_D13, (base) + 0x28},	\
	{CORTEXM_REG_VFP_D14, (base) + 0x30},	\
	{CORTEXM_REG_VFP_D15, (base) + 0x38}
#define	CORTEXM_FRAME_FREERTOS_FP_LEN		0x40

static const struct cortexm_txml_regframe cortexm_rtos_freertos_v67m[] = {
	CORTEXM_FRAME_FREERTOS_CORE(0x00),
	CORTEXM_FRAME_CPU(CORTEXM_FRAME_FREERTOS_CORE_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_CPU_LEN}
};
#define	CORTEXM_RTOS_FREERTOS_V67M_COUNT	\
	(sizeof(cortexm_rtos_freertos_v67m) /	\
	 sizeof(cortexm_rtos_freertos_v67m[0]))

static const struct cortexm_txml_regframe cortexm_rtos_freertos_v7m_fp[] = {
	CORTEXM_FRAME_FREERTOS_CORE(0x00),
	CORTEXM_FRAME_FREERTOS_FP(CORTEXM_FRAME_FREERTOS_CORE_LEN),
	CORTEXM_FRAME_EXT(CORTEXM_FRAME_FREERTOS_CORE_LEN +
			  CORTEXM_FRAME_FREERTOS_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_FREERTOS_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static const struct cortexm_txml_regframe cortexm_rtos_freertos_v7m_lazyfp[] = {
	CORTEXM_FRAME_FREERTOS_CORE(0x00),
	CORTEXM_FRAME_FREERTOS_FP(CORTEXM_FRAME_FREERTOS_CORE_LEN),
	CORTEXM_FRAME_EXT_LAZY(CORTEXM_FRAME_FREERTOS_CORE_LEN +
			       CORTEXM_FRAME_FREERTOS_FP_LEN),
	{CORTEXM_REG_SP, CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_FREERTOS_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static_assert(sizeof(cortexm_rtos_freertos_v7m_fp) ==
	      sizeof(cortexm_rtos_freertos_v7m_lazyfp),
	      "V7M FP regframe size mismatch");
#define	CORTEXM_RTOS_FREERTOS_V7M_FP_COUNT	\
	(sizeof(cortexm_rtos_freertos_v7m_fp) /	\
	 sizeof(cortexm_rtos_freertos_v7m_fp[0]))

static const struct cortexm_txml_regframe cortexm_rtos_freertos_v8m[] = {
	{CORTEXM_REG_PSPLIM_S, 0x00},	/* For task in Secure mode */
	{CORTEXM_REG_PSPLIM_NS, 0x00},	/* For task in Non-Secure mode */
	CORTEXM_FRAME_FREERTOS_CORE(0x04),
	CORTEXM_FRAME_CPU(0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN),
	{CORTEXM_REG_SP, 0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_CPU_LEN}
};
#define	CORTEXM_RTOS_FREERTOS_V8M_COUNT	\
	(sizeof(cortexm_rtos_freertos_v8m) /	\
	 sizeof(cortexm_rtos_freertos_v8m[0]))

static const struct cortexm_txml_regframe cortexm_rtos_freertos_v8m_fp[] = {
	{CORTEXM_REG_PSPLIM_S, 0x00},	/* For task in Secure mode */
	{CORTEXM_REG_PSPLIM_NS, 0x00},	/* For task in Non-Secure mode */
	CORTEXM_FRAME_FREERTOS_CORE(0x04),
	CORTEXM_FRAME_FREERTOS_FP(0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN),
	CORTEXM_FRAME_EXT(0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN +
			  CORTEXM_FRAME_FREERTOS_FP_LEN),
	{CORTEXM_REG_SP, 0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_FREERTOS_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static const struct cortexm_txml_regframe cortexm_rtos_freertos_v8m_lazyfp[] = {
	{CORTEXM_REG_PSPLIM_S, 0x00},	/* For task in Secure mode */
	{CORTEXM_REG_PSPLIM_NS, 0x00},	/* For task in Non-Secure mode */
	CORTEXM_FRAME_FREERTOS_CORE(0x04),
	CORTEXM_FRAME_FREERTOS_FP(0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN),
	CORTEXM_FRAME_EXT_LAZY(0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN +
			       CORTEXM_FRAME_FREERTOS_FP_LEN),
	{CORTEXM_REG_SP, 0x04 + CORTEXM_FRAME_FREERTOS_CORE_LEN +
			 CORTEXM_FRAME_FREERTOS_FP_LEN +
			 CORTEXM_FRAME_EXT_LEN}
};
static_assert(sizeof(cortexm_rtos_freertos_v8m_fp) ==
	      sizeof(cortexm_rtos_freertos_v8m_lazyfp),
	      "V8M FP regframe size mismatch");
#define	CORTEXM_RTOS_FREERTOS_V8M_FP_COUNT	\
	(sizeof(cortexm_rtos_freertos_v8m_fp) /	\
	 sizeof(cortexm_rtos_freertos_v8m_fp[0]))

const struct cortexm_txml_regframe *
cortexm_freertos_get_regframe(cortexm_t cm, const struct target_rtos *tr,
    unsigned int *frame_count, bool *dynamic)
{
	const struct cortexm_txml_regframe *rf = NULL;
	bool has_fpu, ext_frame, state_valid;
	unsigned int count = 0;
	target_addr_t addr;

	/*
	 * FreeRTOS frame layout is pretty consistant for all Cortex-M
	 * ports. We use the regframe layouts, declared above, as-is.
	 */
	*dynamic = false;
	cm->cm_rtos_frame_base = tr->tr_frame;

	addr = tr->tr_frame;
	if (CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V8M) {
		/* Skip PSPLIM */
		addr += 4;
	}

	has_fpu = cortexm_rtos_get_fp_status(cm,
	    addr + CORTEXM_FRAME_FREERTOS_EXC_LR,
	    addr + CORTEXM_FRAME_FREERTOS_CORE_LEN +
		   CORTEXM_FRAME_FREERTOS_FP_LEN,
	    &ext_frame, &state_valid);

	switch (CORTEXM_FLAG_ARCH(cm)) {
	case CORTEXM_FLAG_ARCH_V6M:
		count = CORTEXM_RTOS_FREERTOS_V67M_COUNT;
		rf = cortexm_rtos_freertos_v67m;
		break;

	case CORTEXM_FLAG_ARCH_V7M:
		if (has_fpu && ext_frame) {
			/* Extended frame. */
			count = CORTEXM_RTOS_FREERTOS_V7M_FP_COUNT;

			if (state_valid) {
				rf = cortexm_rtos_freertos_v7m_fp;
			} else {
				/*
				 * FPU state was not saved, therefore the
				 * current FPU registers should be used.
				 */
				rf = cortexm_rtos_freertos_v7m_lazyfp;
			}
		} else {
			count = CORTEXM_RTOS_FREERTOS_V67M_COUNT;
			rf = cortexm_rtos_freertos_v67m;
		}
		break;

	case CORTEXM_FLAG_ARCH_V8M:
		if (has_fpu && ext_frame) {
			/* Extended frame. */
			count = CORTEXM_RTOS_FREERTOS_V8M_FP_COUNT;
			if (state_valid) {
				rf = cortexm_rtos_freertos_v8m_fp;
			} else {
				/*
				 * FPU state was not saved, therefore the
				 * current FPU registers should be used.
				 */
				rf = cortexm_rtos_freertos_v8m_lazyfp;
			}
		} else {
			count = CORTEXM_RTOS_FREERTOS_V8M_COUNT;
			rf = cortexm_rtos_freertos_v8m;
		}
		break;

	default:
		return NULL;
	}

	*frame_count = count;
	return rf;
}

void
cortexm_freertos_load_params(cortexm_t cm,
    const struct gdb_rtos_freertos_target_params *tp)
{
	const gdb_rtos_freertos_params_t *cfp;
	gdb_rtos_param_t task_name_offset = 0;
	bool probed = false;

	assert(tp->tp_params->rp_source <= GDB_RTOS_PARAM_PROBED);

	if (tp->tp_integrity) {
		if (tp->tp_short_ticktype)
			cfp = &gdb_freertos_params_ilp32_tt2_int;
		else
			cfp = &gdb_freertos_params_ilp32_tt4_int;
	} else {
		if (tp->tp_short_ticktype)
			cfp = &gdb_freertos_params_ilp32_tt2;
		else
			cfp = &gdb_freertos_params_ilp32_tt4;
	}

	if ((probed = memcmp(tp->tp_params, cfp, sizeof(*cfp))) != 0) {
		/* Make the parameters current. */
		memcpy(tp->tp_params, cfp, sizeof(*cfp));
	}

	/*
	 * Account for the xMPU_SETTINGS variable in TCB_t if the target's
	 * symbol table hints that MPU wrappers are being used.
	 *
	 * The size of this is fairly consistent in the default ports, but
	 * does vary according to Cortex-M family and number of MPU regions.
	 */
	if (tp->tp_mpu_wrappers) {
		gdb_rtos_param_t mpu_regions;

		/*
		 * If MPU wrappers are enabled, but we detected no MPU exists,
		 * then assume firmware is configured for 8 regions.
		 *
		 * XXX: It would be polite to inform the user somehow...
		 */
		if ((cm->cm_flags & CORTEXM_FLAG_HAS_MPU) != 0)
			mpu_regions = (gdb_rtos_param_t)cm->cm_mpu_regions;
		else
			mpu_regions = 8;

		switch (CORTEXM_FLAG_ARCH(cm)) {
		case CORTEXM_FLAG_ARCH_V8M:
			/*
			 * Cortex-M23/M33 ...
			 *   typedef struct MPU_SETTINGS {
			 *     uint32_t ulMAIR0;
			 *     struct {
			 *   	 uint32_t ulRegionBaseAddress;
			 *       uint32_t ulRegionAttribute;
			 *     } xRegions[ portTOTAL_NUM_REGIONS ];
			 *   } xMPU_SETTINGS;
			 */
			task_name_offset += sizeof(uint32_t);
			/*FALLTHROUGH*/

		case CORTEXM_FLAG_ARCH_V7M:
			/*
			 * Cortex-M3/M4 ...
			 *   typedef struct MPU_SETTINGS {
			 *     struct {
			 *   	 uint32_t ulRegionBaseAddress;
			 *       uint32_t ulRegionAttribute;
			 *     } xRegions[ portTOTAL_NUM_REGIONS ];
			 *   } xMPU_SETTINGS;
			 */
			task_name_offset += sizeof(uint32_t) * 2 * mpu_regions;
			break;

		default:
			/* XXX: WTH? */
			break;
		}
	}

	/* Adjust the offset to TCB_t->pcTaskName as necessary. */
	tp->tp_params->rp_val[GDB_RTOS_FREERTOS_offsetof_TCB_t_pcTaskName] +=
	    task_name_offset;
	if (task_name_offset)
		probed = true;

	/* Indicate if no longer using the default params. */
	if (probed)
		tp->tp_params->rp_source = GDB_RTOS_PARAM_PROBED;
}
