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

#ifndef SAM_EIC_H
#define SAM_EIC_H

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

typedef enum {
	SAM_EIC_IRQ_TYPE_EDGE_RISING = 1,
	SAM_EIC_IRQ_TYPE_EDGE_FALLING,
	SAM_EIC_IRQ_TYPE_EDGE_BOTH,
	SAM_EIC_IRQ_TYPE_LEVEL_HIGH,
	SAM_EIC_IRQ_TYPE_LEVEL_LOW,
} sam_eic_irq_type_t;

typedef struct {
	const char *ei_name;
	uint32_t ei_ipl;
	unsigned int ei_extint;
	sam_eic_irq_type_t ei_type;
	bool ei_async;
	bool ei_filter;
	bool ei_debounce;
} sam_eic_irq_t;
#define	SAM_EIC_EXTINT_MAX	(EIC_EXTINT_NUM - 1)

extern void *sam_eic_irq_register(const sam_eic_irq_t *ei,
		void (*func)(void *, bool), void *arg);
extern void sam_eic_irq_unregister(void *handle);
extern void sam_eic_irq_control(void *handle, bool enable);

typedef enum {
	SAM_EIC_FILTER_CLOCK_GCLK,
	SAM_EIC_FILTER_CLOCK_ULP32K
} sam_eic_filter_clock_t;

typedef enum {
	SAM_EIC_DEBOUNCE_SAMPLES_3,
	SAM_EIC_DEBOUNCE_SAMPLES_7
} sam_eic_debounce_samples_t;

typedef enum {
	SAM_EIC_DEBOUNCE_PRESCALER_NONE,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV2,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV4,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV8,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV16,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV32,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV64,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV128,
	SAM_EIC_DEBOUNCE_PRESCALER_DIV256
} sam_eic_debounce_prescaler_t;

extern void sam_eic_init(unsigned int gen, sam_eic_filter_clock_t,
		sam_eic_debounce_samples_t, sam_eic_debounce_prescaler_t);

#endif /* SAM_EIC_H */
