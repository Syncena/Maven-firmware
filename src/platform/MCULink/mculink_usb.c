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

#include <stdlib.h>
#include <stdint.h>

#include "rtos.h"
#include "platform.h"
#include "platform_usb.h"
#include "usb_common.h"
#include "timer.h"

void
mculink_usb_init(void)
{
	extern int _siusbram, _susbram, _eusbram;
	volatile int *src, *dst;
	void *irq;

	/* Power up USB1's PHY. */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_USBHSPHY_MASK;
	timer_buzz_delay_us(100);

	/*
	 * Reset the hardware - if only because that's what the TinyUSB
	 * example does...
	 */
	mculink_reset_ctrl(2, SYSCON_PRESETCTRL2_USB1_HOST_RST_MASK);
	mculink_reset_ctrl(2, SYSCON_PRESETCTRL2_USB1_DEV_RST_MASK);
	mculink_reset_ctrl(2, SYSCON_PRESETCTRL2_USB1_PHY_RST_MASK);
	mculink_reset_ctrl(2, SYSCON_PRESETCTRL2_USB1_RAM_RST_MASK);

	/* Temporarily enable USB1 host clock. */
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_USB1_HOST_MASK, true);

	/* Put PHY powerdown under software control. */
	USBHSH->PORTMODE = USBHSH_PORTMODE_SW_PDCOM_MASK;
	USBHSH->PORTMODE |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/* Disable USB1 host clock. */
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_USB1_HOST_MASK, false);

	/* Power up USB1 LDO */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_LDOUSBHS_MASK;
	__DSB();
	__ISB();

	/* Wait for the PHY/LDO to fully power-up. */
	timer_buzz_delay_us(100);

	/* Enable clock to USB1 SRAM. */
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_USB1_RAM_MASK, true);
	__DSB();
	__ISB();

	/* Copy .usbram initialised data. */
	src = &_siusbram;
	dst = &_susbram;
	while (dst < &_eusbram)
		*dst++ = *src++;
	__DSB();
	__ISB();

	/* Enable clock to USB1 PHY. */
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_USB1_PHY_MASK, true);

	/* Issue a oftware reset to the PHY... */
	USBPHY->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/*
	 * Configure USB1's PLL to generate 480 MHz using our 16 MHz xtal
	 * as a reference.
	 */
	USBPHY->PLL_SIC = (USBPHY->PLL_SIC & ~USBPHY_PLL_SIC_PLL_DIV_SEL_MASK) |
	    USBPHY_PLL_SIC_PLL_DIV_SEL(6U);	/* 16 MHz */
	USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_SET_PLL_REG_ENABLE_MASK;
	/*
	 * The data sheet states: "Reserved. User must set this bit to 0x0".
	 * The device header files define USBPHY_PLL_SIC_SET_PLL_BYPASS_MASK
	 * for this bit. We'll use the latter.
	 */
	USBPHY->PLL_SIC_CLR = USBPHY_PLL_SIC_SET_PLL_BYPASS_MASK;
	USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_SET_PLL_POWER_MASK;
	USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS_MASK;
	USBPHY->CTRL_CLR = USBPHY_CTRL_CLR_CLKGATE_MASK;
	USBPHY->PWD_SET  = 0x0;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/* Enable clock to USB1 device. */
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_USB1_DEV_MASK, true);

	/*
	 * Connect 'clkin' to USB HS PLL.
	 * Note: 'clkin' from Xtal32M was done in mculink_init_core_clock().
	 */
	ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_PLL_USB_OUT_MASK;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/* Enable PHY support for Low speed device + LS via FS Hub. */
	USBPHY->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK |
	    USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/* Enable all power for normal operation. */
	USBPHY->PWD = 0;
	USBPHY->CTRL_SET = USBPHY_CTRL_SET_ENAUTOCLR_CLKGATE_MASK;
	USBPHY->CTRL_SET = USBPHY_CTRL_SET_ENAUTOCLR_PHY_PWD_MASK;
	__DSB();
	__ISB();
	timer_buzz_delay_us(100);

	/*
	 * Hook the device interrupts.
	 */
	irq = rtos_irq_register_named("USB", USB1_IRQn, HW_IPL_USB,
	     usb_interrupt, NULL);
	assert(irq != NULL);
	(void) irq;
}
