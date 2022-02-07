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

#ifndef SAM_EFC_H
#define SAM_EFC_H

#include "sam4s.h"

//#include "parts.h"	/* atmel/asf/common/utils/parts.h */
extern void mpu_efc_begin(void);
extern void mpu_efc_end(void);

#define	SAM_EFC_OPT_MPU_BEGIN()	mpu_efc_begin()
#define	SAM_EFC_OPT_MPU_END()	mpu_efc_end()

#include <stdlib.h>
#include "rtos.h"
#include "FreeRTOS.h"

#define __no_inline	__attribute__((__noinline__))

#if 0
typedef uint32_t irqflags_t;

#define	cpu_irq_save	__arm_ints_disable
#define	cpu_irq_restore	__arm_ints_restore
#endif

#include "efc.h"

extern void sam_efc_init(uint32_t waits);
extern int sam_efc_program_page(uint32_t page, const void *psrc, int lock);
extern void sam_efc_wipe_user_area(void);

typedef union {
	uint32_t u32[4];
	uint8_t u8[16];
} sam_efc_unique_id_t;
extern int sam_efc_get_unique_id(sam_efc_unique_id_t *);

#endif /* SAM_EFC_H */
