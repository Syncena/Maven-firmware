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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "uart.h"

#ifdef CONFIG_SHELL
#include "shell.h"
#endif

struct uart {
	const char *u_name;
	const struct uart_funcs *u_uf;
	void *u_uf_arg;
	uint16_t u_cfg;
	uint32_t u_wanted_baud;
	uint32_t u_actual_baud;
};

static struct uart uarts[UART_NUARTS];

/* Private bits in u_cfg */
#define	UART_CFG_OPEN		(1u << 14)
#define	UART_CFG_CLAIMED	(1u << 15)

#ifdef CONFIG_SHELL
SHELL_CMD_DECL(uart, uart_cmd, "Displays/modifies UART config");
#endif

static struct uart *
uart_get_uart(unsigned int instance)
{

	if (instance >= UART_NUARTS)
		return NULL;

	return &uarts[instance];
}

uint32_t
uart_configure(void *arg, uint16_t cfg, uint32_t custom_baud)
{
	struct uart *u = arg;
	uint32_t actual_baud;

	cfg &= UART_CFG_MASK;

#ifdef UART_OPT_NO_FLOW
	cfg = (cfg & ~UART_FLOW_MASK) | UART_FLOW_NONE;
#endif

#ifdef UART_OPT_FIXED_DATABITS
	cfg = (cfg & ~UART_CHARSIZE_MASK) | UART_CHARSIZE_8;
#endif

#ifdef UART_OPT_FIXED_STOPBITS
	cfg = (cfg & ~UART_STOPBITS_MASK) | UART_STOPBITS_1;
#endif

	actual_baud = (u->u_uf->uf_configure)(u->u_uf_arg, cfg, custom_baud);
	if (actual_baud != 0) {
		if (custom_baud == 0)
			custom_baud = actual_baud;
		u->u_wanted_baud = custom_baud;
		u->u_actual_baud = actual_baud;
		u->u_cfg = (u->u_cfg & ~UART_CFG_MASK) | cfg;
	}

	return actual_baud;
}

int
uart_register(const char *pname, const struct uart_funcs *uf, void *arg)
{
	int i;


	for (i = 0; i < UART_NUARTS && uarts[i].u_uf != NULL; i++)
		;

	if (i == UART_NUARTS)
		return -1;

#ifdef CONFIG_SHELL
	if (i == 0)
		SHELL_CMD_ADD(uart);
#endif

	uarts[i].u_uf = uf;
	uarts[i].u_uf_arg = arg;
	uarts[i].u_cfg = UART_DEFAULT_CFG;
	uarts[i].u_actual_baud = 38400u;
	uarts[i].u_name = pname;

	return i;
}

void *
uart_claim(int which)
{
	struct uart *u = uart_get_uart((unsigned int)which);

	if (u == NULL || u->u_uf == NULL || (u->u_cfg & UART_CFG_CLAIMED) != 0)
		return NULL;

	u->u_cfg |= UART_CFG_CLAIMED;

	return u;
}

void
uart_unclaim(void *arg)
{
	struct uart *u = arg;

	if ((u->u_cfg & UART_CFG_CLAIMED) == 0)
		return;

	if ((u->u_cfg & UART_CFG_OPEN) != 0)
		(u->u_uf->uf_close)(u->u_uf_arg);

	u->u_cfg &= UART_CFG_MASK;
}

uint8_t
uart_open(void *arg, ringbuff_t from_uart, ringbuff_t to_uart)
{
	struct uart *u = arg;
	uint8_t rv;

	uart_configure(u, u->u_cfg, u->u_wanted_baud);

	rv = (u->u_uf->uf_open)(u->u_uf_arg, from_uart, to_uart);
	if (rv)
		u->u_cfg |= UART_CFG_OPEN;

	return rv;
}

void
uart_close(void *arg)
{
	struct uart *u = arg;

	if ((u->u_cfg & UART_CFG_OPEN) != 0) {
		(u->u_uf->uf_close)(u->u_uf_arg);
		u->u_cfg &= ~UART_CFG_OPEN;
	}
}

void
uart_polled_tx(void *arg, uint8_t ch)
{
	struct uart *u = arg;

	if (u->u_uf->uf_polled_tx)
		(u->u_uf->uf_polled_tx)(u->u_uf_arg, ch);
}

uint16_t
uart_current_config(void *arg, uint32_t *pbaud)
{
	struct uart *u = arg;

	*pbaud = u->u_actual_baud;
	return u->u_cfg;
}

uint32_t
uart_cfg_to_baud(uint16_t cfg, uint32_t custom_baud)
{
	static const uint16_t speeds[16] = {
		0, 3, 6, 12, 24, 48, 96, 192,
		384, 576, 1152, 2304, 4608, 9216, 18432, 0
	};
	uint32_t rv;

	if ((cfg & UART_BAUDRATE_MASK) != UART_BAUDRATE_CUSTOM) {
		rv = (uint32_t)speeds[cfg & UART_BAUDRATE_MASK];
		if (rv == 0)
			rv = 150;
		else
			rv *= 100u;
	} else {
		rv = custom_baud;
	}

	return rv;
}

void
uart_config_strings(uint16_t cfg, uint32_t baud, char *cstr, char *bstr)
{

	if (bstr != NULL) {
		baud = uart_cfg_to_baud(cfg, baud);
		sprintf(bstr, "%" PRIu32, baud);
	}

	if (cstr == NULL)
		return;

	cstr[3] = '\0';

#ifndef UART_OPT_FIXED_STOPBITS
	switch (cfg & UART_STOPBITS_MASK) {
	case UART_STOPBITS_1:
		cstr[2] = '1';
		break;
	case UART_STOPBITS_2:
		cstr[2] = '2';
		break;
	default:
		cstr[2] = '?';
		break;
	}
#else
	cstr[2] = '1';
#endif /* UART_OPT_FIXED_STOPBITS */

	switch (cfg & UART_PARITY_MASK) {
	case UART_PARITY_NONE:
		cstr[1] = 'N';
		break;
	case UART_PARITY_ODD:
		cstr[1] = 'O';
		break;
	case UART_PARITY_EVEN:
		cstr[1] = 'E';
		break;
	default:
		cstr[1] = '?';
		break;
	}

#ifndef UART_OPT_FIXED_DATABITS
	switch (cfg & UART_CHARSIZE_MASK) {
	case UART_CHARSIZE_5:
		cstr[0] = '5';
		break;
	case UART_CHARSIZE_6:
		cstr[0] = '6';
		break;
	case UART_CHARSIZE_7:
		cstr[0] = '7';
		break;
	case UART_CHARSIZE_8:
		cstr[0] = '8';
		break;
	default:
		cstr[0] = '?';
		break;
	}
#else
	cstr[0] = '8';
#endif /* UART_OPT_FIXED_DATABITS */
}

uint16_t
uart_baud_to_cfg(uint32_t baud)
{
	uint16_t cfg;

	switch (baud) {
	case 150:
		cfg = UART_BAUDRATE_150;
		break;
	case 300:
		cfg = UART_BAUDRATE_300;
		break;
	case 600:
		cfg = UART_BAUDRATE_600;
		break;
	case 1200:
		cfg = UART_BAUDRATE_1200;
		break;
	case 2400:
		cfg = UART_BAUDRATE_2400;
		break;
	case 4800:
		cfg = UART_BAUDRATE_4800;
		break;
	case 9600:
		cfg = UART_BAUDRATE_9600;
		break;
	case 19200:
		cfg = UART_BAUDRATE_19200;
		break;
	case 38400:
		cfg = UART_BAUDRATE_38400;
		break;
	case 57600:
		cfg = UART_BAUDRATE_57600;
		break;
	case 115200:
		cfg = UART_BAUDRATE_115200;
		break;
	case 230400:
		cfg = UART_BAUDRATE_230400;
		break;
	case 460800:
		cfg = UART_BAUDRATE_460800;
		break;
	case 921600:
		cfg = UART_BAUDRATE_921600;
		break;
	case 1843200:
		cfg = UART_BAUDRATE_1843200;
		break;
	default:
		cfg = UART_BAUDRATE_CUSTOM;
		break;
	}

	return cfg;
}

#ifdef CONFIG_SHELL
static void
uart_get_stats(void *arg, struct uart_stats *st)
{
	struct uart *u = arg;

	(u->u_uf->uf_get_stats)(u->u_uf_arg, st);
}

static void
uart_print_cfg(FILE *os, struct uart *u)
{
	char cfg_str[4], baud_str[16];
#ifndef UART_OPT_NO_FLOW
	const char *flow = NULL;
#endif

	uart_config_strings(u->u_cfg, u->u_wanted_baud, cfg_str, baud_str);

#ifndef UART_OPT_NO_FLOW
	switch (u->u_cfg & UART_FLOW_MASK) {
	case UART_FLOW_NONE:
		flow = "None";
		break;
	case UART_FLOW_SOFTWARE:
		flow = "XON/XOFF";
		break;
	case UART_FLOW_SOFTWARE_ANY:
		flow = "ANY/XOFF";
		break;
	case UART_FLOW_HARDWARE:
		flow = "RTS/CTS";
		break;
	}

	fprintf(os, "Format: %s, Flow: %s, Baudrate: %s", cfg_str,
	    flow, baud_str);
#else
	fprintf(os, "Format: %s, Baudrate: %s", cfg_str, baud_str);
#endif

	fprintf(os, " (Actual: %" PRIu32 ")\n", u->u_actual_baud);
}

static void
uart_dump(FILE *os, int which)
{
	struct uart *u = uart_get_uart(which);
	struct uart_stats st;

	if (u == NULL || u->u_uf == NULL || u->u_name == NULL)
		return;

	fprintf(os, "uart%d (%s): ", which, u->u_name);
	uart_print_cfg(os, u);
	uart_get_stats(u, &st);

	fprintf(os, "\t         Tx Bytes: %" PRIu32, st.us_tx_bytes);

	if (st.us_tx_bytes_polled)
		fprintf(os, " (%" PRIu32 " polled)", st.us_tx_bytes_polled);

	fprintf(os, "\n\t         Rx Bytes: %" PRIu32 "\n", st.us_rx_bytes);

	if (st.us_tx_breaks) {
		fprintf(os, "\t        Tx BREAKs: %" PRIu32 "\n",
		    st.us_tx_breaks);
	}

	if (st.us_rx_pe)
		fprintf(os, "\t Rx Parity Errors: %" PRIu32 "\n", st.us_rx_pe);

	if (st.us_rx_fe)
		fprintf(os, "\tRx Framing Errors: %" PRIu32 "\n", st.us_rx_fe);

	if (st.us_rx_noise) {
		fprintf(os, "\t  Rx Noise Events: %" PRIu32 "\n",
		    st.us_rx_noise);
	}

	if (st.us_rx_fifo_overruns) {
		fprintf(os, "\t Rx Fifo Overruns: %" PRIu32 "\n",
		    st.us_rx_fifo_overruns);
	}

	if (st.us_rx_ring_overruns) {
		fprintf(os, "\t Rx Ring Overruns: %" PRIu32 "\n",
		    st.us_rx_ring_overruns);
	}

#ifndef UART_OPT_NO_FLOW
	if ((u->u_cfg & UART_FLOW_MASK) != UART_FLOW_NONE) {
		fprintf(os, "\t    Tx Xoff Bytes: %" PRIu32 "\n",st.us_tx_xoff);
		fprintf(os, "\t     Tx Xon Bytes: %" PRIu32 "\n", st.us_tx_xon);
		fprintf(os, "\t    Rx Xoff Bytes: %" PRIu32 "\n",st.us_rx_xoff);
		fprintf(os, "\t     Rx Xon Bytes: %" PRIu32 "\n", st.us_rx_xon);
	}
#endif
}

static void
uart_cmd(FILE *os, uint8_t argc, const char * const *argv)
{

	(void) argc;
	(void) argv;

	for (int i = 0; i < UART_NUARTS; i++)
		uart_dump(os, i);
}
#endif /* CONFIG_SHELL */
