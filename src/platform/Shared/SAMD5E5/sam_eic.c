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
#include <stdint.h>
#include <stdlib.h>

#include "rtos.h"
#include "platform.h"
#include "sam_eic.h"
#include "sam_clocks.h"
#include "zone_alloc.h"

#define	SAM_EIC_CFG_REG(i)	((i) / 8u)
#define	SAM_EIC_CFG_SHIFT(i)	(((i) % 8u) * 4u)
#define	SAM_EIC_CFG_MASK(i)	(0xfu << SAM_EIC_CFG_SHIFT(i))
#define	SAM_EIC_CFG(i,s,f)	((EIC_CONFIG_SENSE0(s) | \
				((f) ? EIC_CONFIG_FILTEN0 : 0)) << \
				SAM_EIC_CFG_SHIFT(i))

struct sam_eic_handler {
	void (*eh_func)(void *, bool);
	void *eh_arg;
	unsigned int eh_extint;
	void *eh_irq_handle;
};

static void
sam_eic_wait_sync(uint32_t mask)
{

	while ((EIC->SYNCBUSY.reg & mask) != 0)
		;
}

void
sam_eic_init(unsigned int gen, sam_eic_filter_clock_t fc,
    sam_eic_debounce_samples_t ds, sam_eic_debounce_prescaler_t dp)
{
	uint32_t reg;
	uint8_t ctrl;

	/* Validate parameters. */
	assert(fc == SAM_EIC_FILTER_CLOCK_GCLK ||
	    fc == SAM_EIC_FILTER_CLOCK_ULP32K);
	assert(ds == SAM_EIC_DEBOUNCE_SAMPLES_3 ||
	    ds == SAM_EIC_DEBOUNCE_SAMPLES_7);
	assert(dp <= SAM_EIC_DEBOUNCE_PRESCALER_DIV256);

	/* Enable EIC's APB clock. */
	sam_periph_clock_enable(ID_EIC);

	if (EIC->CTRLA.bit.ENABLE) {
		/* Already enabled. Disable. */
		EIC->CTRLA.reg = 0;
		(void) EIC->CTRLA.reg;
		sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);
	}

	/* Software reset. */
	EIC->CTRLA.reg = EIC_CTRLA_SWRST;
	(void) EIC->CTRLA.reg;
	sam_eic_wait_sync(EIC_SYNCBUSY_SWRST);

	/* Select required filter clock. */
	if (fc == SAM_EIC_FILTER_CLOCK_ULP32K) {
		ctrl = EIC_CTRLA_CKSEL;
	} else {
		/* Route the assigned GCLK generator. */
		unsigned int clock;
		clock = sam_gclk_periph_chan_enable(EIC_GCLK_ID, gen);
		assert(clock <= 100000000u);
		(void) clock;
		ctrl = 0;
	}

	if (ds == SAM_EIC_DEBOUNCE_SAMPLES_7)
		reg = EIC_DPRESCALER_STATES0 | EIC_DPRESCALER_STATES1;
	else
		reg = 0;

	if (dp != SAM_EIC_DEBOUNCE_PRESCALER_NONE) {
		uint32_t p = (uint32_t)dp - SAM_EIC_DEBOUNCE_PRESCALER_DIV2;
		reg |= EIC_DPRESCALER_TICKON;
		reg |= EIC_DPRESCALER_PRESCALER0(p);
		reg |= EIC_DPRESCALER_PRESCALER1(p);
	}

	EIC->DPRESCALER.reg = reg;
	EIC->CTRLA.reg = ctrl | EIC_CTRLA_ENABLE;
	(void) EIC->CTRLA.reg;
	sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);
}

static void
sam_eic_interrupt(void *arg)
{
	struct sam_eic_handler *eh = arg;
	uint32_t bit, flag, pstate;

	bit = 1u << eh->eh_extint;
	flag = EIC->INTFLAG.reg & EIC->INTENSET.reg;
	flag &= bit;

	while (flag != 0) {
		pstate = EIC->PINSTATE.reg & bit;
		EIC->INTFLAG.reg = flag;

		(eh->eh_func)(eh->eh_arg, pstate != 0);

		flag = EIC->INTFLAG.reg & EIC->INTENSET.reg;
		flag &= bit;
	}
}

void *
sam_eic_irq_register(const sam_eic_irq_t *ei, void (*func)(void *, bool),
    void *arg)
{
	struct sam_eic_handler *eh;
	volatile uint32_t *pcfg;
	uint32_t cfg, async, deb;

	assert(ei->ei_extint <= SAM_EIC_EXTINT_MAX);
	assert(ei->ei_type >= SAM_EIC_IRQ_TYPE_EDGE_RISING);
	assert(ei->ei_type <= SAM_EIC_IRQ_TYPE_LEVEL_LOW);
	assert(ei->ei_filter == false || ei->ei_async == false);

	pcfg = &EIC->CONFIG[SAM_EIC_CFG_REG(ei->ei_extint)].reg;
	cfg = *pcfg;
	assert((cfg & SAM_EIC_CFG_MASK(ei->ei_extint)) == 0);

	cfg |= SAM_EIC_CFG(ei->ei_extint, (uint32_t)ei->ei_type, ei->ei_filter);

	async = EIC->ASYNCH.reg;
	if (ei->ei_async)
		async |= 1u << ei->ei_extint;
	else
		async &= ~(1u << ei->ei_extint);

	deb = EIC->DEBOUNCEN.reg;
	if (ei->ei_debounce)
		deb |= 1u << ei->ei_extint;
	else
		deb &= ~(1u << ei->ei_extint);

	if ((eh = zone_malloc(sizeof(*eh))) == NULL)
		return NULL;

	eh->eh_func = func;
	eh->eh_arg = arg;
	eh->eh_extint = ei->ei_extint;

	eh->eh_irq_handle = rtos_irq_register_named(ei->ei_name,
	    EIC_0_IRQn + ei->ei_extint, ei->ei_ipl,
	    sam_eic_interrupt, eh);

	if (eh->eh_irq_handle == NULL) {
		zone_free(eh);
		eh = NULL;
	} else {
		rtos_saved_ipl_t ipl;
		uint8_t ctrl;

		ipl = rtos_ipl_raise(HW_IPL_MAX);

		ctrl = EIC->CTRLA.reg;
		EIC->CTRLA.reg = ctrl & ~EIC_CTRLA_ENABLE;
		sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);
		*pcfg = cfg;
		EIC->ASYNCH.reg = async;
		EIC->DEBOUNCEN.reg = deb;
		EIC->CTRLA.reg = ctrl;
		sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);

		rtos_ipl_restore(ipl);

		EIC->INTFLAG.reg = 1u << ei->ei_extint;
	}

	return eh;
}

void
sam_eic_irq_unregister(void *handle)
{
	struct sam_eic_handler *eh = handle;
	volatile uint32_t *pcfg;
	rtos_saved_ipl_t ipl;
	uint32_t cfg;
	uint8_t ctrl;

	pcfg = &EIC->CONFIG[SAM_EIC_CFG_REG(eh->eh_extint)].reg;
	cfg = *pcfg;
	assert((cfg & SAM_EIC_CFG_MASK(eh->eh_extint)) != 0);

	EIC->INTENCLR.reg = 1u << eh->eh_extint;
	EIC->INTFLAG.reg = 1u << eh->eh_extint;

	cfg &= ~SAM_EIC_CFG_MASK(eh->eh_extint);

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	ctrl = EIC->CTRLA.reg;
	EIC->CTRLA.reg = ctrl & ~EIC_CTRLA_ENABLE;
	sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);
	*pcfg = cfg;
	EIC->CTRLA.reg = ctrl;
	sam_eic_wait_sync(EIC_SYNCBUSY_ENABLE);
	rtos_ipl_restore(ipl);

	if (eh->eh_irq_handle != NULL)
		rtos_irq_unregister(eh->eh_irq_handle);

	zone_free(eh);
}

void
sam_eic_irq_control(void *handle, bool enable)
{
	struct sam_eic_handler *eh = handle;

	if (enable)
		EIC->INTENSET.reg = 1u << eh->eh_extint;
	else
		EIC->INTENCLR.reg = 1u << eh->eh_extint;
}
