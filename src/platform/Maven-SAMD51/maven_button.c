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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "platform.h"
#include "hardware.h"

/* User button is here. */
HAL_GPIO_PIN(BUTTON, B, 23)

#include "rtos.h"
#include "sam_clocks.h"
#include "sam_eic.h"
#include "display.h"
#include "timer.h"

#define	BUTTON_IRQ_PMUX		'A'
#define	BUTTON_IRQ_EXTINT	7

#define	BUTTON_EVENT_NONE	0u
#define	BUTTON_EVENT_WAKE	(1u << 0)

struct maven_button_state {
	volatile unsigned int bs_events;
	volatile bool bs_pressed;
	bool bs_wait_inactive;
	struct timeval bs_stamp;
	void *bs_eic;
	rtos_timer_t bs_timer;
};
static struct maven_button_state maven_button_state;

static void
maven_button_interrupt(void *cookie, bool pressed)
{
	struct maven_button_state *bs = cookie;
	struct timeval cur;

	if (bs->bs_wait_inactive) {
		/*
		 * Button might be pressed at startup, so we wait until
		 * it is released.
		 */
		if (!pressed)
			bs->bs_wait_inactive = false;
		bs->bs_pressed = pressed;
	}

	if (pressed == bs->bs_pressed)
		return;
	bs->bs_pressed = pressed;

	timer_get_mono_time(&cur);

	if (pressed) {
		rtos_timer_change_wait_period_isr(bs->bs_timer, 1);
		rtos_timer_start_isr(bs->bs_timer);
		bs->bs_events |= BUTTON_EVENT_WAKE;
		bs->bs_stamp = cur;
	}
}

static void
maven_button_timer(rtos_timer_t t)
{
	struct maven_button_state *bs = rtos_timer_get_id(t);
	unsigned int events;
	rtos_saved_ipl_t ipl;

	/* Fetch current state */
	ipl = rtos_ipl_raise(HW_IPL_BUTTON);
	events = bs->bs_events;
	bs->bs_events = BUTTON_EVENT_NONE;
	rtos_ipl_restore(ipl);

	if ((events & BUTTON_EVENT_WAKE) != 0) {
		display_ip_address();
		rtos_timer_change_wait_period(bs->bs_timer, 4999);
		return;
	}

	/* If still pressed after 5 seconds, start provisioning. */
	if (bs->bs_pressed) {
		bs->bs_wait_inactive = true;
		maven_wifi_provision_start();
	}
}

void
maven_button_attach(void)
{
	static const sam_eic_irq_t ei = {
		.ei_name = "Button",
		.ei_ipl = HW_IPL_BUTTON,
		.ei_extint = BUTTON_IRQ_EXTINT,
		.ei_type = SAM_EIC_IRQ_TYPE_EDGE_BOTH,
		.ei_async = true,
		.ei_filter = false,
		.ei_debounce = true
	};
	struct maven_button_state *bs = &maven_button_state;

	bs->bs_timer = rtos_timer_create("Button", 1, 0, maven_button_timer,bs);
	assert(bs->bs_timer != NULL);

	HAL_GPIO_BUTTON_in();
	HAL_GPIO_BUTTON_inen(1);
	HAL_GPIO_BUTTON_pmux(BUTTON_IRQ_PMUX);

	bs->bs_pressed = HAL_GPIO_BUTTON_read() == 0;
	bs->bs_wait_inactive = bs->bs_pressed;
	bs->bs_events = BUTTON_EVENT_NONE;
	bs->bs_eic = sam_eic_irq_register(&ei, maven_button_interrupt, bs);
	assert(bs->bs_eic != NULL);
	sam_eic_irq_control(bs->bs_eic, true);
}
