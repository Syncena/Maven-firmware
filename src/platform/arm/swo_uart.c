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
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "swo_uart.h"
#include "uart.h"
#include "ringbuff.h"

/*
 * Work-around some naming inconsistencies in ARM's CMSIS headers...
 */
#ifndef ITM_TCR_TraceBusID_Msk
#define	ITM_TCR_TraceBusID_Msk	ITM_TCR_TRACEBUSID_Msk
#endif

/*
 * SAMD5x device header defines 'PORT'. This clashes with ITM->PORT.
 */
#ifdef	PORT
#undef	PORT
#endif

struct swo_uart_state {
	unsigned int us_swo_port;
	uint32_t us_speed;
	ringbuff_t us_rb;
	struct uart_funcs us_uf;
	struct uart_stats us_stats;
};
static struct swo_uart_state swo_uart_state;

static void
swo_uart_tx(struct swo_uart_state *us, uint8_t ch)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_USART_CONSOLE);

	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0 &&
	    (ITM->TER & (1u << us->us_swo_port)) != 0) {
		while (ITM->PORT[us->us_swo_port].u32 == 0)
			;
		ITM->PORT[us->us_swo_port].u8 = ch;
	}

	rtos_ipl_restore(ipl);
}

static void
swo_uart_polled_tx(void *arg, uint8_t ch)
{
	struct swo_uart_state *us = arg;

	swo_uart_tx(us, ch);
	us->us_stats.us_tx_bytes_polled++;
}

static void
swo_uart_tx_avail_cb(ringbuff_t rb, void *arg)
{
	struct swo_uart_state *us = arg;

	while (ringbuff_get_count(rb)) {
		swo_uart_tx(us, ringbuff_consume(rb));
		us->us_stats.us_tx_bytes++;
	}

	ringbuff_consume_done(rb);
}

static uint8_t
swo_uart_open(void *arg, ringbuff_t uart2app, ringbuff_t app2uart)
{
	struct swo_uart_state *us = arg;

	(void) uart2app;

	if ((us->us_rb = app2uart) != NULL)
		ringbuff_consumer_init(app2uart, swo_uart_tx_avail_cb, us);

	return 1;
}

static void
swo_uart_close(void *arg)
{

	(void) arg;
}

static uint32_t
swo_uart_configure(void *arg, uint16_t cfg, uint32_t custom_baud)
{
	struct swo_uart_state *us = arg;

	(void) cfg;
	(void) custom_baud;

	return us->us_speed;
}

static void
swo_uart_get_stats(void *arg, struct uart_stats *s)
{
	struct swo_uart_state *us = arg;

	memcpy(s, &us->us_stats, sizeof(*s));
}

int
swo_uart_attach(const char *name, unsigned int swo_port, uint32_t speed)
{
	struct swo_uart_state *us = &swo_uart_state;

	assert(swo_port <= 31);
	us->us_swo_port = swo_port;
	us->us_speed = speed;

	/*
	 * "Selected PIN Protocol Register": Select which protocol to use
	 * for trace output (2: SWO NRZ, 1: SWO Manchester encoding)
	 */
	// 0xE00400F0 = 2
	TPI->SPPR = 2;

	/*
	 * Bypass the TPIU formatter
	 */
	// 0xE0040304 = 0
	TPI->FFCR = 0;

	/*
	 * "Async Clock Prescaler Register". Scale the baud rate of the
	 * asynchronous output
	 */
	// 0xE0040010 = 53
	TPI->ACPR = (F_CPU / speed) - 1;

	/* enable trace in core debug */
	// 0xE000EDFC |= 1 << 24
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* DWT_CTRL */
	// 0xE0001000 = 0x400003FE
	DWT->CTRL = 0x400003FE;

	/*
	 * ITM Lock Access Register, C5ACCE55 enables more write access to
	 * Control Register 0xE00 :: 0xFFC
	 */
	// 0xE0000FB0
	ITM->LAR = 0xC5ACCE55;

	/* ITM Trace Control Register. XXX:  Validate with Marple's code */
	// 0xE0000E80 = 0x007f0015
	ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk |
	    ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;

	/* ITM Trace Privilege Register */
	// 0xE0000E40
	ITM->TPR = ITM_TPR_PRIVMASK_Msk;

	/*
	 * ITM Trace Enable Register. Enabled tracing on stimulus ports.
	 * One bit per stimulus port.
	 */
	// 0xE0000E00
	ITM->TER = 1u << swo_port;

	/* Now attach as a generic UART. */
	us->us_uf.uf_open = swo_uart_open;
	us->us_uf.uf_close = swo_uart_close;
	us->us_uf.uf_configure = swo_uart_configure;
	us->us_uf.uf_polled_tx = swo_uart_polled_tx;
	us->us_uf.uf_get_stats = swo_uart_get_stats;

	return uart_register(name, &us->us_uf, us);
}
