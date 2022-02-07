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

#ifndef FAKE_SOC_H
#define FAKE_SOC_H

/*
 * For use by the platform templates... Pretend we're a lowly Cortex-M4.
 */

typedef enum IRQn {
	NonMaskableInt_IRQn	= -14,
	HardFault_IRQn		= -13,
	MemoryManagement_IRQn	= -12,
	BusFault_IRQn		= -11,
	UsageFault_IRQn		= -10,
	SVCall_IRQn		= -5,
	DebugMonitor_IRQn	= -4,
	PendSV_IRQn		= -2,
	SysTick_IRQn		= -1
} IRQn_Type;

#define __CM4_REV              1         /* Core revision r0p1 */
#define __DEBUG_LVL            3         /* Full debug plus DWT data matching */
#define __FPU_PRESENT          0         /* No FPU */
#define __MPU_PRESENT          1         /* MPU present */
#define __NVIC_PRIO_BITS       3         /* 3 bits used for Priority Levels */
#define __TRACE_LVL            1         /* Basic trace */
#define __VTOR_PRESENT         1         /* VTOR present */
#define __Vendor_SysTickConfig 0         /* Use standard SysTick */

#include <core_cm4.h>

#endif /* FAKE_SOC_H */
