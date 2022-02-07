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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "target_comms.h"
#include "uart.h"
#include "lpc_flexcomm_usart.h"
#include "swo_uart.h"
#include "timer.h"

/*
 * Install the "VCOM Disable" jumper to redirect MCU-Link's own console
 * to the target serial port.
 */
HAL_GPIO_PIN(VCOM_DISABLE, 0, 18)

/*
 * MCULink uses PIO0.10, function 6 for SWO.
 */
HAL_GPIO_PIN(CONSOLE_TX, 0, 10)
#define	MCULINK_CONSOLE_TX_FUNCTION	6
#define	MCULINK_CONSOLE_TX_ITM_PORT	0u
#define	MCULINK_CONSOLE_TX_BAUD		1000000u

static void
dummy_uart_tx_avail_cb(ringbuff_t rb, void *arg)
{

	(void) arg;

	while (!ringbuff_is_empty(rb))
		(void) ringbuff_consume(rb);

	ringbuff_consume_done(rb);
}

static uint8_t
dummy_uart_open(void *arg, ringbuff_t from_uart, ringbuff_t to_uart)
{

	(void) from_uart;

	if (to_uart != NULL)
		ringbuff_consumer_init(to_uart, dummy_uart_tx_avail_cb, arg);

	return 1;
}

static void
dummy_uart_close(void *arg)
{

	(void) arg;
}

static uint32_t
dummy_uart_configure(void *arg, uint16_t ucfg, uint32_t custom_baud)
{
	(void) arg;

	return uart_cfg_to_baud(ucfg, custom_baud);
}

static void
dummy_uart_get_stats(void *arg, struct uart_stats *s)
{

	(void) arg;
	memset(s, 0, sizeof(*s));
}

static void
dummy_uart_polled_tx(void *arg, uint8_t ch)
{

	(void) arg;
	(void) ch;
}

static const struct uart_funcs dummy_uart = {
	.uf_open = dummy_uart_open,
	.uf_close = dummy_uart_close,
	.uf_configure = dummy_uart_configure,
	.uf_get_stats = dummy_uart_get_stats,
	.uf_polled_tx = dummy_uart_polled_tx
};

static int
attach_flexcomm(const char *name)
{
	lpc_flexcomm_usart_attach_args_t aa;

	HAL_GPIO_EXT7_pullup(1);
	HAL_GPIO_EXT7_digimode(1);
	HAL_GPIO_EXT9_pullup(1);
	HAL_GPIO_EXT9_digimode(1);

	/*
	 * Target serial port is on Flexcomm-USART #0
	 */
	mculink_clock_ctrl(1, SYSCON_AHBCLKCTRL1_FC0_MASK, true);

	/* Clock source is pll0_clk_div, configured to 6.25 MHz at startup. */
	SYSCON->FCCLKSELX[0] = SYSCON_FCCLKSEL0_SEL(1);

	/* Switch Flexcomm0 to USART mode. */
	FLEXCOMM0->PSELID = FLEXCOMM_PSELID_PERSEL(1) |
	    FLEXCOMM_PSELID_LOCK_MASK;

	HAL_GPIO_EXT7_func(1);	/* Flexcomm0 TxD */
	HAL_GPIO_EXT9_func(1);	/* Flexcomm0 RxD */

	aa.aa_instance = 0;
	aa.aa_clock = PLATFORM_FLEXCOMM_CLOCK;
	aa.aa_baud = 0;
	aa.aa_cfg = UART_DEFAULT_CFG;
	aa.aa_name = name;
	aa.aa_ipl = HW_IPL_USART_TARGET;
	aa.aa_use_softirq = true;

	return lpc_flexcomm_usart_attach(&aa);
}

static int
attach_swo_console(void)
{

	/*
	 * There's no available Flexcomm device for a traditional
	 * interactive console. Best we can do is write-only SWO...
	 */
	HAL_GPIO_CONSOLE_TX_digimode(1);
	HAL_GPIO_CONSOLE_TX_out();
	HAL_GPIO_CONSOLE_TX_func(MCULINK_CONSOLE_TX_FUNCTION);

	/*
	 * Enable Trace clock.
	 */
	SYSCON->TRACECLKSEL = SYSCON_TRACECLKSEL_SEL(0);
	SYSCON->TRACECLKDIV = SYSCON_TRACECLKDIV_DIV(0);

	/* Use the generic SWO UART driver. */
	return swo_uart_attach("Console (SWO)", MCULINK_CONSOLE_TX_ITM_PORT,
	    MCULINK_CONSOLE_TX_BAUD);
}

static int
attach_dummy(const char *name)
{

	return uart_register(name, &dummy_uart, NULL);
}

static int
attach_swo(void)
{
	lpc_flexcomm_usart_attach_args_t aa;

	/*
	 * Target SWO port is on Flexcomm-USART #3
	 */
	mculink_clock_ctrl(1, SYSCON_AHBCLKCTRL1_FC3_MASK, true);

	/* Clock source is pll0_clk_div, configured to 6.25 MHz at startup. */
	SYSCON->FCCLKSELX[3] = SYSCON_FCCLKSEL3_SEL(1);

	/* Switch Flexcomm3 to USART mode. */
	FLEXCOMM3->PSELID = FLEXCOMM_PSELID_PERSEL(1) |
	    FLEXCOMM_PSELID_LOCK_MASK;

	HAL_GPIO_EXT1_func(1);	/* Flexcomm3 RxD */

	aa.aa_instance = 3;
	aa.aa_clock = PLATFORM_FLEXCOMM_CLOCK;
	aa.aa_baud = 0;
	aa.aa_cfg = UART_DEFAULT_CFG;
	aa.aa_name = "SWO";
	aa.aa_ipl = HW_IPL_USART_TARGET;
	aa.aa_use_softirq = true;

	return lpc_flexcomm_usart_attach(&aa);
}

void
platform_uart_target_enable(void)
{

	/* No-op. Can't control Tx/Rx direction on MCU-Link. */
}

void
platform_uart_target_disable(void)
{

	/* No-op. Can't control Tx/Rx direction on MCU-Link. */
}

int
platform_uart_attach(platform_uart_t which)
{
	static int console_mode;
#define	CONSOLE_MODE_INIT	0
#define	CONSOLE_MODE_SWO	1
#define	CONSOLE_MODE_FLEXCOMM	2
#define	CONSOLE_MODE_DUMMY	2

	if (console_mode == CONSOLE_MODE_INIT) {
		HAL_GPIO_VCOM_DISABLE_pullup(1);
		HAL_GPIO_VCOM_DISABLE_digimode(1);
		HAL_GPIO_VCOM_DISABLE_in();
		timer_buzz_delay_us(100);
		console_mode = HAL_GPIO_VCOM_DISABLE_read() ?
		    CONSOLE_MODE_SWO : CONSOLE_MODE_FLEXCOMM;
	}

	if (which == PLATFORM_UART_CONSOLE) {
		if (console_mode == CONSOLE_MODE_DUMMY)
			return attach_dummy("Console");
		else
		if (console_mode == CONSOLE_MODE_SWO)
			return attach_swo_console();
		else
			return attach_flexcomm("Console");
	}

	if (which == PLATFORM_UART_TARGET) {
		if (console_mode == CONSOLE_MODE_FLEXCOMM)
			return attach_dummy("Target Serial");
		else
			return attach_flexcomm("Target Serial");
	}

	if (which == PLATFORM_UART_SWO)
		return attach_swo();

	return -1;
}
