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
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "sam-udp.h"
#include "usb_common.h"

extern uint8_t sysconfig_xtal;

void
sam_udp_platform_init(void)
{
	void *irq;

	udd_enable_periph_ck();

	/*
	 * Use PLLB to provide 48 MHz USB clock.
	 */
	if (sysconfig_xtal == 12) {
		/*
		 * We have a 12 MHz xtal feeding the main oscillator.
		 * Divide this by 2 and multiply by 16 to give 96 MHz PLL
		 * output.
		 */
		PMC->CKGR_PLLBR = CKGR_PLLBR_MULB(16-1) |
		    CKGR_PLLBR_PLLBCOUNT(0x3f) |
		    CKGR_PLLBR_DIVB(2);
	} else {
		/*
		 * We have a 16 MHz xtal feeding the main oscillator.
		 * Multiply by 6 to give 96 MHz PLL output.
		 */
		PMC->CKGR_PLLBR = CKGR_PLLBR_MULB(6-1) |
		    CKGR_PLLBR_PLLBCOUNT(0x3f) |
		    CKGR_PLLBR_DIVB(1);
	}

	/* Wait for lock */
	while ((PMC->PMC_SR & PMC_SR_LOCKB) == 0)
		;

	/* USB clock = PLLB / 2 */
	PMC->PMC_USB = PMC_USB_USBDIV(1) | PMC_USB_USBS;

	/* Enable USB clock */
	PMC->PMC_SCER = PMC_SCER_UDP;

	irq = rtos_irq_register_named("USB", UDP_IRQn, HW_IPL_USB,
	    sam_udp_interrupt_handler, NULL);
	assert(irq != NULL);
	(void) irq;

        /* Allow asynchronous USB interrupts to exit of sleep mode */
	PMC->PMC_FSMR |= PMC_FSMR_USBAL;

	/*
	 * Note:
	 * There's no need to enable the USB functions DDM/DDP on GPIO
	 * pins PB10/PB11 since USB is the default function after reset.
	 */
}

void
sam_udp_platform_deinit(void)
{
}

void
pmc_config_periph_clk(uint32_t devid, bool enable)
{
	volatile uint32_t *preg;

	assert(devid < ID_PERIPH_COUNT);

	if (enable)
		preg = (devid < 32) ? &PMC->PMC_PCER0 : &PMC->PMC_PCER1;
	else
		preg = (devid < 32) ? &PMC->PMC_PCDR0 : &PMC->PMC_PCDR1;

	*preg = 1ul << (devid % 32u);
}
