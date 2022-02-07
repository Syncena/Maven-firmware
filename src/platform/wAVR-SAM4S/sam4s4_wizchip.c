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
#include <string.h>
#include <stdio.h>

#include "rtos.h"
#include "spi-interface.h"
#include "sam4s4_wizchip.h"
#include "wizchip_driver.h"
#include "serial_number.h"
#include "display.h"
#include "hardware.h"
#include "platform.h"
#include "crypto.h"
#include "timer.h"

static void
sam4s_wiz_interrupt(void *arg)
{

	for (;;) {
		uint32_t isr = (GPIO_NETWORK_IRQ_PIO->PIO_ISR &
		    GPIO_NETWORK_IRQ_PIO->PIO_IMR) & GPIO_NETWORK_IRQ_BIT;

		if (isr == 0)
			break;

		if ((GPIO_NETWORK_IRQ_PIO->PIO_PDSR & isr) == 0)
			wizchip_interrupt(arg);
	}
}

static void
sam4s_wiz_irq_control(uint8_t enable)
{

	if (enable)
		GPIO_NETWORK_IRQ_PIO->PIO_IER = GPIO_NETWORK_IRQ_BIT;
	else
		GPIO_NETWORK_IRQ_PIO->PIO_IDR = GPIO_NETWORK_IRQ_BIT;
}

static void
sam4s_wiz_reset(void)
{

	/*
	 * The WIZ reset pin and display controller reset pin share the
	 * same GPIO pin, so tell the display task what we're about to do.
	 */
	display_stop();

	HAL_GPIO_NETWORK_RST_clr();
	timer_buzz_delay_ms(100);
	HAL_GPIO_NETWORK_RST_set();
	timer_buzz_delay_ms(100);

	/*
	 * The display will need to be re-initialised.
	 */
	display_restart();
}

static void
sam4s_wiz_control(void *cookie, uint8_t cmd, void *arg)
{

	(void) cookie;

	switch (cmd) {
	case WIZ5X00_CTL_IRQ_CTL:
		sam4s_wiz_irq_control(*((uint8_t *)arg));
		break;

	case WIZ5X00_CTL_IRQ_STATUS:
		*((uint8_t *)arg) = HAL_GPIO_NETWORK_IRQ_read() == 0;
		break;

	case WIZ5X00_CTL_RESET:
		sam4s_wiz_reset();
		break;

	default:
		break;
	}
}

void
sam4s_wizchip_attach(const struct spi_interface *spi)
{
	const serial_number_t *sn;
	uint8_t mac[6];
	uint32_t crc;
	void *drv;

	/* Configure GPIOs for Wiz reset, enable and irq pins */
	HAL_GPIO_NETWORK_RST_clr();
	HAL_GPIO_NETWORK_RST_out();
	HAL_GPIO_NETWORK_EN_set();
	HAL_GPIO_NETWORK_EN_out();
	HAL_GPIO_NETWORK_IRQ_pullup(1);
	HAL_GPIO_NETWORK_IRQ_in();

	/*
	 * Enable interrupt for the WizChip controller, configured for
	 * low-level.
	 */
	GPIO_NETWORK_IRQ_PIO->PIO_IER = GPIO_NETWORK_IRQ_BIT;
	GPIO_NETWORK_IRQ_PIO->PIO_AIMER = GPIO_NETWORK_IRQ_BIT;
	GPIO_NETWORK_IRQ_PIO->PIO_LSR = GPIO_NETWORK_IRQ_BIT;
	GPIO_NETWORK_IRQ_PIO->PIO_FELLSR = GPIO_NETWORK_IRQ_BIT;

	sn = serial_number_fetch();
	if (sn == NULL)
		return;

	mac[0] = 0x6u;
	mac[1] = 0x0u;
	crc = crypto_crc32(sn, sizeof(*sn));
	memcpy(&mac[2], &crc, sizeof(crc));

	drv = wizchip_driver_attach(spi, mac, sam4s_wiz_control, NULL);
	if (drv == NULL)
		return;

	/* Hook the WzChip interrupt */
	rtos_irq_register_named("WizNet", GPIO_NETWORK_IRQ_NUMBER,
	    HW_IPL_NETWORK, sam4s_wiz_interrupt, drv);

	/* Enable interrupt */
	sam4s_wiz_irq_control(1);
}
