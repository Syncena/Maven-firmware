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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "console.h"
#include "timer.h"
#include "rtos.h"
#include "platform.h"
#include "uart.h"
#include "hardware.h"
#include "zone_alloc.h"

#define	CONSOLE_RX_RING_SIZE		16
#define	CONSOLE_TX_RING_SIZE_DEFAULT	512

#ifndef CONSOLE_TX_RING_SIZE
#define	CONSOLE_TX_RING_SIZE		CONSOLE_TX_RING_SIZE_DEFAULT
#endif

struct cons_state {
	void *cs_uart;
	ringbuff_t cs_rb_to_cons;
	ringbuff_t cs_rb_from_cons;
#if (CONSOLE_TX_RING_SIZE <= CONSOLE_TX_RING_SIZE_DEFAULT)
	uint8_t cs_tx_ring[CONSOLE_TX_RING_SIZE];
#else
	uint8_t *cs_tx_ring;
#endif
	uint8_t cs_rx_ring[CONSOLE_RX_RING_SIZE];
};
static struct cons_state cons_state;

static void
cons_putchar(char c, struct cons_state *cs)
{

	if (cs->cs_rb_to_cons == NULL) {
		/* No ringbuff. Send direct to UART. */
		uart_polled_tx(cs->cs_uart, c);
		return;
	}

	if (ringbuff_is_full(cs->cs_rb_to_cons)) {
		/*
		 * Ringbuff is full. Free up a space by consuming and
		 * sending the next available character. This needs to be
		 * done with the UART driver's Tx interrupt handler masked;
		 * we could be racing it to the next character...
		 */
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_USART_CONSOLE);
		uart_polled_tx(cs->cs_uart,
		    ringbuff_consume(cs->cs_rb_to_cons));
		rtos_ipl_restore(ipl);

		/*
		 * XXX: We should invoke ringbuff_consume_done() here, but
		 * I'm not convinced it's prudent to do so. In any case, all
		 * current console UART drivers ignore the call.
		 */
	}

	ringbuff_produce(cs->cs_rb_to_cons, (uint8_t)c);
}

void
console_write_stdout(const char *buf, int len)
{
	struct cons_state *cs = &cons_state;
	struct timeval tv;
	const char *sp;
	char s[16], ch;
	unsigned int x;
	static bool newline = true;

	if (cs->cs_uart == NULL)
		return;

	while (len--) {
		if (newline) {
			timer_get_mono_time(&tv);

			sprintf(s, "%06u.%06u ", (unsigned int)tv.tv_sec,
			    (unsigned int)tv.tv_usec);
			for (sp = s; *sp != '\0'; sp++)
				(void) cons_putchar(*sp, cs);

			if (_rtos_interrupt_context())
				sp = "-- IRQ --";
			else
				sp = rtos_task_get_name(rtos_task_current());

			for (x = 0; *sp != '\0'; sp++, x++)
				(void) cons_putchar(*sp, cs);

			while (x++ < 12)
				(void) cons_putchar(' ', cs);
		}

		ch = *buf++;
		newline = (ch == '\n');
		if (newline)
			(void) cons_putchar('\r', cs);
		cons_putchar(ch, cs);
		if (newline)
			ringbuff_produce_done(cs->cs_rb_to_cons);
	}

#ifndef CONSOLE_FLUSH_IN_IDLE_TASK
	if (cs->cs_rb_to_cons != NULL)
		ringbuff_produce_done(cs->cs_rb_to_cons);
#endif
}

void
console_flush(bool force)
{
	struct cons_state *cs = &cons_state;

	if (force) {
		while (!ringbuff_is_empty(cs->cs_rb_to_cons)) {
			uart_polled_tx(cs->cs_uart,
			    ringbuff_consume(cs->cs_rb_to_cons));
		}
	}

	if (cs->cs_rb_to_cons != NULL)
		ringbuff_produce_done(cs->cs_rb_to_cons);
}

ringbuff_t
console_init(const char *str)
{
	struct cons_state *cs = &cons_state;
	int uart_id;

#ifdef WAVR_BOARD
	/*
	 * The original SAM4S-based wAVR board is very short on SRAM (64 KB).
	 *
	 * We want to minimise the size of buffers used by stdio's buffered
	 * I/O, but at the moment newlib's stdio file pointers are pointing
	 * at __sf_fake_std<in|out|err> structures (in Flash!) so we can't set
	 * fp->_flags.__SMBF until the real stdio structures are configured.
	 *
	 * By default stdio will allocate 1 KB per buffer. But by setting the
	 * __SMBF flag *before* performing any actual I/O, we can force the
	 * library routines to use 64-byte buffers. Note that this also relies
	 * on our _fstat() stub in newlib_stubs.c returning -1 for all the
	 * std<in|out|err> file descriptors.
	 *
	 * Fortunately we can get things initialised without causing I/O by
	 * invoking the function version of feof() for each file pointer.
	 *
	 * Note that feof() is a pre-processor macro defined in stdio.h but
	 * there is also a function version available in the library.
	 *
	 * Yes, it's an egregious hack.
	 */
#ifdef feof
#undef feof
#endif
	(void) feof(stdout);
	stdout->_flags |= __SMBF;

	(void) feof(stderr);
	stderr->_flags |= __SMBF;

	(void) feof(stdin);
	stdin->_flags |= __SMBF;
#endif	/* WAVR_BOARD */

	if ((uart_id = platform_uart_attach(PLATFORM_UART_CONSOLE)) < 0)
		return NULL;

	if ((cs->cs_rb_from_cons = ringbuff_alloc(cs->cs_rx_ring,
	    CONSOLE_RX_RING_SIZE)) == NULL) {
		return NULL;
	}

#if (CONSOLE_TX_RING_SIZE > CONSOLE_TX_RING_SIZE_DEFAULT)
	cs->cs_tx_ring = zone_malloc(CONSOLE_TX_RING_SIZE);
	if (cs->cs_tx_ring == NULL)
		return NULL;
#endif

	if ((cs->cs_rb_to_cons = ringbuff_alloc(cs->cs_tx_ring,
	    CONSOLE_TX_RING_SIZE)) == NULL) {
#if (CONSOLE_TX_RING_SIZE > CONSOLE_TX_RING_SIZE_DEFAULT)
		zone_free(cs->cs_tx_ring);
#endif
		ringbuff_free(cs->cs_rb_from_cons);
		return NULL;
	}

	if ((cs->cs_uart = uart_claim(uart_id)) == NULL ||
	    !uart_open(cs->cs_uart, cs->cs_rb_from_cons, cs->cs_rb_to_cons)) {
		ringbuff_free(cs->cs_rb_from_cons);
		ringbuff_free(cs->cs_rb_to_cons);
#if (CONSOLE_TX_RING_SIZE > CONSOLE_TX_RING_SIZE_DEFAULT)
		zone_free(cs->cs_tx_ring);
#endif
		cs->cs_rb_from_cons = NULL;
		cs->cs_rb_to_cons = NULL;
		return NULL;
	}

	uart_configure(cs->cs_uart, UART_DEFAULT_CFG, 0);

	if (*str != '\0')
		console_write_stdout(str, strlen(str));

	return cs->cs_rb_from_cons;
}
