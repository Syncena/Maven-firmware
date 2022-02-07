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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "rtos.h"
#include "wifi_driver.h"
#include "winc_wifi.h"
#include "network.h"
#include "sam4s4_wifi.h"
#include "timer.h"
#include "hardware.h"
#include "display.h"

struct sam4s_wifi_state {
	void *ws_driver;
	uint32_t ws_provision_count;
	rtos_timer_t ws_ptimer;
};
static struct sam4s_wifi_state sam4s_wifi_state;

static void
sam4s_wifi_interrupt(void *arg)
{

	(void) GPIO_NETWORK_IRQ_PIO->PIO_ISR;

	wifi_driver_interrupt(arg);
}

static void
sam4s_wifi_irq_control(uint8_t enable)
{

	if (enable)
		GPIO_NETWORK_IRQ_PIO->PIO_IER = GPIO_NETWORK_IRQ_BIT;
	else
		GPIO_NETWORK_IRQ_PIO->PIO_IDR = GPIO_NETWORK_IRQ_BIT;
}

static void
sam4s_wifi_reset(void)
{

	/*
	 * The WiFi reset pin and display controller reset pin share the
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
sam4s_wifi_control(void *cookie, uint8_t cmd, void *argp)
{

	(void) cookie;

	switch (cmd) {
	case WINC_WIFI_IRQ_CTL:
		sam4s_wifi_irq_control(*((uint8_t *)argp));
		break;

	case WINC_WIFI_RESET:
		sam4s_wifi_reset();
		break;

#ifndef WINC_PROVISION_SW
	case WIFI_CTL_READ_PROVISION:
		*(uint8_t *)argp = HAL_GPIO_PROVISION_BUTTON_read() == 0;
		break;
#endif

	default:
		break;
	}
}

static void
sam4s_poll_provision_switch(rtos_timer_t handle)
{
	struct sam4s_wifi_state *ws = rtos_timer_get_id(handle);
	struct winc_gpio wg;
	bool pressed;

	wg.wg_pin = WINC_PROVISION_SW;
	wg.wg_value = 0;

	if (wifi_ioctl(ws->ws_driver, WINC_IOCTL_READ_GPIO, &wg) < 0)
		return;

	/* Switch is active low. */
	pressed = wg.wg_value == 0;

	if (pressed == false) {
		if (ws->ws_provision_count >= 4 && ws->ws_provision_count < 20)
			display_ip_address();
		ws->ws_provision_count = 0;
		return;
	}

	display_activity(DISPLAY_ACTIVITY_WAKE);

	if (++(ws->ws_provision_count) == 20) {
		if (wifi_driver_provision_start(ws->ws_driver) < 0)
			ws->ws_provision_count = 0;
	}
}

void
sam4s_wifi_attach(const struct spi_interface *spi)
{
	struct sam4s_wifi_state *ws = &sam4s_wifi_state;

	/*
	 * Configure GPIOs for WiFi reset, enable and irq pins.
	 *
	 * Note that timing is important here. The documentation states
	 * that there must be at least 5mS between the rising edges of
	 * CHIP_EN and RESET, with the former rising first.
	 *
	 * We'll be a little more accomodating.
	 */
	/* Keep RESET asserted until sam4s_wifi_reset() is invoked. */
	HAL_GPIO_NETWORK_RST_clr();
	HAL_GPIO_NETWORK_RST_out();

	/* Keep CHIP_EN deasserted  for a bit longer. */
	HAL_GPIO_NETWORK_EN_clr();
	HAL_GPIO_NETWORK_EN_out();
	timer_buzz_delay_ms(50);

	/* Now assert it. */
	HAL_GPIO_NETWORK_EN_set();
	timer_buzz_delay_ms(50);

	/*
	 * Enable interrupt for the WiFi controller, configured for
	 * falling-edge or low-level as appropriate.
	 *
	 * XXX: I soo need a GPIO driver in ../../../Embedded/arm/stuff
	 */
	HAL_GPIO_NETWORK_IRQ_pullup(1);
	HAL_GPIO_NETWORK_IRQ_in();
	GPIO_NETWORK_IRQ_PIO->PIO_IER = GPIO_NETWORK_IRQ_BIT;
	GPIO_NETWORK_IRQ_PIO->PIO_AIMER = GPIO_NETWORK_IRQ_BIT;
#ifdef NM_EDGE_INTERRUPT
	GPIO_NETWORK_IRQ_PIO->PIO_ESR = GPIO_NETWORK_IRQ_BIT;
#else
	GPIO_NETWORK_IRQ_PIO->PIO_LSR = GPIO_NETWORK_IRQ_BIT;
#endif
	GPIO_NETWORK_IRQ_PIO->PIO_FELLSR = GPIO_NETWORK_IRQ_BIT;

	/*
	 * Attach the WiFi controller
	 */
	ws->ws_driver = wifi_driver_attach(spi, sam4s_wifi_control, NULL);
	if (ws->ws_driver == NULL)
		return;

	/* Hook the wifi interrupt */
	rtos_irq_register_named("WiFi", GPIO_NETWORK_IRQ_NUMBER, HW_IPL_NETWORK,
	    sam4s_wifi_interrupt, ws->ws_driver);

	/* Enable WiFi interrupt */
	sam4s_wifi_irq_control(1);

	/*
	 * Start a timer to monitor the provision switch
	 */
	ws->ws_ptimer = rtos_timer_create("Provision", 250, 1,
	    sam4s_poll_provision_switch, ws);
	assert(ws->ws_ptimer != NULL);
	rtos_timer_start(ws->ws_ptimer);
}
