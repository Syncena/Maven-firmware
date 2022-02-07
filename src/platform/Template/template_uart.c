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

#include <string.h>
#include <stdio.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "target_comms.h"
#include "ringbuff.h"
#include "uart.h"

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
attach_console(void)
{

	return uart_register("Console", &dummy_uart, NULL);
}

static int
attach_target(void)
{

	return uart_register("Target Serial", &dummy_uart, NULL);
}

static int
attach_swo(void)
{

	return uart_register("SWO", &dummy_uart, NULL);
}

void
platform_uart_target_enable(void)
{
}

void
platform_uart_target_disable(void)
{
}

int
platform_uart_attach(platform_uart_t which)
{

	if (which == PLATFORM_UART_CONSOLE)
		return attach_console();

	if (which == PLATFORM_UART_TARGET)
		return attach_target();

	if (which == PLATFORM_UART_SWO)
		return attach_swo();

	return -1;
}
