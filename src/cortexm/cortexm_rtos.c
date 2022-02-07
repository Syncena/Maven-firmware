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
#include "cortexm_rtos_nuttx.h"

bool
cortexm_rtos_get_fp_status(cortexm_t cm, target_addr_t exc_lr_addr,
    target_addr_t ext_addr, bool *ext_frame, bool *state_valid)
{
	struct target_mem_readwrite mr;
	uint32_t v;

	*ext_frame = false;
	*state_valid = false;

	/* If this core has no FPU, we're done. */
	if ((cm->cm_flags & CORTEXM_FLAG_HAS_FPU) == 0)
		return false;

	/*
	 * Check EXC_LR, bit#4 to determine if we are dealing with
	 * an extended stack frame.
	 */
	mr.mr_write = false;
	mr.mr_target_addr = exc_lr_addr;
	mr.mr_length = sizeof(uint32_t);
	mr.mr_dest = &v;
	if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE, &mr) < 0)
		return false;

	/* Determine if an extended frame was created. */
	*ext_frame = (v & 0x10) == 0;

	/*
	 * Determine if the FPU register state was actually saved.
	 */
	if ((cm->cm_fpccr & CMSCS_FPCCR_LSPEN) == 0) {
		/* Lazy-save not in effect. FPU state is in the frame. */
		*state_valid = true;
		return true;
	}

	if ((cm->cm_fpccr & CMSCS_FPCCR_LSPACT) == 0) {
		/* There are no pending lazy-saves. FPU state is in frame. */
		*state_valid = true;
		return true;
	}

	/*
	 * There's a pending lazy-save. Check if it's for us.
	 *
	 * If FPCAR holds the address of our extended save area, then
	 * the core has yet to write FPU state to it. Therefore FPU
	 * registers need to be read from the core.
	 */
	*state_valid = cm->cm_fpcar != ext_addr;

	return true;
}

int
cortexm_rtos_set_frame(cortexm_t cm, const struct target_rtos *tr)
{
	const struct cortexm_txml_regframe *rf = NULL;
	bool dynamic = false;

	if (tr != NULL) {
		switch (tr->tr_rtos) {
		default:
			assert(0);
			break;

		case TARGET_RTOS_NONE:
			break;

		case TARGET_RTOS_FREERTOS:
			rf = cortexm_freertos_get_regframe(cm, tr,
			    &cm->cm_rtos_frame_count, &dynamic);
			break;

		case TARGET_RTOS_NUTTX:
			rf = cortexm_nuttx_get_regframe(cm, tr,
			    &cm->cm_rtos_frame_count, &dynamic);
			break;
		}
	}

	if (cm->cm_rtos_frame_dynamic && cm->cm_rtos_frame != NULL &&
	    cm->cm_rtos_frame != rf) {
		/* Free the old frame; new one is different. */
		zone_free((void *)(uintptr_t)cm->cm_rtos_frame);
	}

	cm->cm_rtos_frame = rf;
	cm->cm_rtos_frame_dynamic = dynamic;

	return 0;
}

int
cortexm_rtos_load_params(cortexm_t cm, const struct target_rtos_params *rp)
{
	int rv = 0;

	switch (rp->rp_rtos) {
	default:
		assert(0);
		break;

	case TARGET_RTOS_FREERTOS:
		cortexm_freertos_load_params(cm, rp->rp_params);
		break;

	case TARGET_RTOS_NUTTX:
		rv = cortexm_nuttx_load_params(cm, rp->rp_params);
		break;
	}

	return rv;
}
