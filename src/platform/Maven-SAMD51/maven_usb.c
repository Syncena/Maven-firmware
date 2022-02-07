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
#include <stdint.h>

#include "rtos.h"
#include "platform.h"
#include "platform_usb.h"
#include "hardware.h"
#include "usb_common.h"
#include "sam_clocks.h"

HAL_GPIO_PIN(USB_DM, A, 24)
HAL_GPIO_PIN(USB_DP, A, 25)
#define MAVEN_USB_PMUX  'H'

void
sam_usb_platform_init(void)
{
	static unsigned int usb_clk_gen;

	if (usb_clk_gen == 0) {
		unsigned int clock;
		usb_clk_gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		assert(usb_clk_gen > 0);

		clock = sam_gclk_gen_config(usb_clk_gen, SAM_GCLK_GEN_SRC_DFLL,
		    SAM_GCLK_GEN_DIVSEL_DIRECT, 1, 0);
		assert(clock == 48000000);
		(void) clock;
	}

#if (CONFIG_USE_TINYUSB != 0)
	/* Hook the interrupts here, but keep them disabled. */
	void *irq;

	irq = rtos_irq_register_named("USB", USB_0_IRQn, HW_IPL_USB,
	    usb_interrupt, NULL);
        assert(irq != NULL);
	(void) irq;

        irq = rtos_irq_register_named("USBSOF", USB_1_IRQn, HW_IPL_USB,
	    usb_interrupt, NULL);
        assert(irq != NULL);
	(void) irq;

        irq = rtos_irq_register_named("USBTC0", USB_2_IRQn, HW_IPL_USB,
	    usb_interrupt, NULL);
        assert(irq != NULL);
	(void) irq;

        irq = rtos_irq_register_named("USBTC1", USB_3_IRQn, HW_IPL_USB,
	    usb_interrupt, NULL);
        assert(irq != NULL);
	(void) irq;
#endif

	sam_periph_clock_enable(ID_USB);
	sam_gclk_periph_chan_enable(USB_GCLK_ID, usb_clk_gen);

	HAL_GPIO_USB_DM_pmux(MAVEN_USB_PMUX);
	HAL_GPIO_USB_DP_pmux(MAVEN_USB_PMUX);
}

void
sam_usb_platform_deinit(void)
{

	HAL_GPIO_USB_DP_pmux(HAL_GPIO_PIN_DISABLE_FUNCTION);
	HAL_GPIO_USB_DM_pmux(HAL_GPIO_PIN_DISABLE_FUNCTION);

	sam_gclk_periph_chan_disable(USB_GCLK_ID);
	sam_periph_clock_disable(ID_USB);
}
