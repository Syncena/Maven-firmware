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

#ifndef CORTEXM_RTOS_H
#define CORTEXM_RTOS_H

/*
 * Standard Cortex-M hardware-stacked frame.
 */
#define	CORTEXM_FRAME_CPU(base)			\
	{CORTEXM_REG_R0,   (base) + 0x00},	\
	{CORTEXM_REG_R1,   (base) + 0x04},	\
	{CORTEXM_REG_R2,   (base) + 0x08},	\
	{CORTEXM_REG_R3,   (base) + 0x0c},	\
	{CORTEXM_REG_R12,  (base) + 0x10},	\
	{CORTEXM_REG_LR,   (base) + 0x14},	\
	{CORTEXM_REG_PC,   (base) + 0x18},	\
	{CORTEXM_REG_XPSR, (base) + 0x1c}
#define	CORTEXM_FRAME_CPU_LEN			0x20

/*
 * Extended Cortex-M hardware-stacked frame.
 */
#define	CORTEXM_FRAME_EXT(base)						\
	CORTEXM_FRAME_CPU(base),					\
	{CORTEXM_REG_VFP_D0,    (base) + CORTEXM_FRAME_CPU_LEN + 0x00},	\
	{CORTEXM_REG_VFP_D1,    (base) + CORTEXM_FRAME_CPU_LEN + 0x04},	\
	{CORTEXM_REG_VFP_D2,    (base) + CORTEXM_FRAME_CPU_LEN + 0x08},	\
	{CORTEXM_REG_VFP_D3,    (base) + CORTEXM_FRAME_CPU_LEN + 0x0c},	\
	{CORTEXM_REG_VFP_D4,    (base) + CORTEXM_FRAME_CPU_LEN + 0x10},	\
	{CORTEXM_REG_VFP_D5,    (base) + CORTEXM_FRAME_CPU_LEN + 0x14},	\
	{CORTEXM_REG_VFP_D6,    (base) + CORTEXM_FRAME_CPU_LEN + 0x18},	\
	{CORTEXM_REG_VFP_D7,    (base) + CORTEXM_FRAME_CPU_LEN + 0x1c},	\
	{CORTEXM_REG_VFP_FPSCR, (base) + CORTEXM_FRAME_CPU_LEN + 0x20}
#define	CORTEXM_FRAME_EXT_LEN	(CORTEXM_FRAME_CPU_LEN + 24)

/*
 * Extended Cortex-M hardware-stacked frame, but with lazy-save in effect.
 * Using CORTEXM_REG_MAX will ensure the current FPU registers are read
 * rather than being retrieved from the frame.
 */
#define	CORTEXM_FRAME_EXT_LAZY(base)					\
	CORTEXM_FRAME_CPU(base),					\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x00},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x04},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x08},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x0c},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x10},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x14},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x18},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x1c},	\
	{CORTEXM_REG_MAX, (base) + CORTEXM_FRAME_CPU_LEN + 0x20}

extern int cortexm_rtos_set_frame(cortexm_t, const struct target_rtos *);
extern int cortexm_rtos_load_params(cortexm_t,const struct target_rtos_params *);
extern bool cortexm_rtos_get_fp_status(cortexm_t, target_addr_t exc_lr_addr,
		target_addr_t ext_addr, bool *ext_frame, bool *state_valid);

#endif /* CORTEXM_RTOS_H */
