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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "target_console.h"
#include "target_comms.h"
#include "hardware.h"
#include "platform.h"
#include "uart.h"
#include "platform.h"
#include "product.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

#ifndef	TARGET_CONSOLE_RINGBUFF_SIZE_BOUNCE
#define	TARGET_CONSOLE_RINGBUFF_SIZE_BOUNCE	6144
#endif

struct target_console_state {
	port_daemon_instance_t ts_port_daemon;
	ringbuff_t ts_to_host;
	ringbuff_t ts_from_host;

	void *ts_uart_handle;
	int ts_uart_id;

	ringbuff_t ts_bounce;
	uint8_t ts_rb_bounce[TARGET_CONSOLE_RINGBUFF_SIZE_BOUNCE];
};
static struct target_console_state	target_console_state;

static void
target_console_rb_callback(ringbuff_t rb, void *arg)
{
	struct target_console_state *ts = arg;

	(void) rb;

	port_daemon_attention(ts->ts_port_daemon);
}

static uint16_t
target_console_get_uart_cfg(uint32_t *custom_baud)
{
	struct target_comms_config tcc;

	if (!target_comms_get_config(TARGET_COMMS_PORT_TARGET_CONSOLE, &tcc)) {
		*custom_baud = 0;
		return UART_DEFAULT_CFG;
	}

	*custom_baud = tcc.tcc_uart_baud;
	return tcc.tcc_uart_cfg;
}

static bool
target_console_connect(void *arg, port_daemon_instance_t si,
    ringbuff_t to_host, ringbuff_t from_host, unsigned int unused)
{
	target_comms_arg_t ca = arg;
	struct target_console_state *ts = ca->ca_ops_arg;

	(void)unused;

	if (ts->ts_port_daemon != NULL)
		return false;

	ts->ts_port_daemon = si;
	ts->ts_to_host = to_host;
	ts->ts_from_host = from_host;

	/*
	 * ts_bounce:
	 *	Consumer - us.
	 *	Producer - Target UART, SWO UART, or Semihosting.
	 *
	 * ts_to_host:
	 *	Consumer - netusb_mux via port_daemon.
	 *	Producer - us.
	 *
	 * As consumer, we read incoming bytes from the target UART,
	 * SWO UART. The callback notifies us that data is waiting.
	 *
	 * As producer, we use the notification to signal there is
	 * space in the network/USB buffer.
	 */
	ringbuff_init(ts->ts_bounce);
	ringbuff_consumer_init(ts->ts_bounce, target_console_rb_callback, ts);
	ringbuff_producer_init(ts->ts_to_host, target_console_rb_callback, ts);

	/* Enable the TxD output buffer */
	platform_uart_target_enable();

	uart_open(ts->ts_uart_handle, ts->ts_bounce, ts->ts_from_host);

	return true;
}

static void
target_console_disconnect(void *arg)
{
	target_comms_arg_t ca = arg;
	struct target_console_state *ts = ca->ca_ops_arg;
	uint32_t baud;
	uint16_t cfg;

	/*
	 * Restore the UART configuration.
	 * We need to restore it following a USB disconnect since most
	 * hosts will configure a 'default' value of their own choosing
	 * upon close.
	 */
	cfg = target_console_get_uart_cfg(&baud);
	uart_configure(ts->ts_uart_handle, cfg, baud);

	/*
	 * Disable the TxD output buffer if we're using
	 * the regular target UART.
	 */
	platform_uart_target_disable();

	DBFPRINTF("closing target uart\n");
	uart_close(ts->ts_uart_handle);

	ts->ts_port_daemon = NULL;
}

static int
target_console_ctrl(void *arg, port_daemon_ctrl_t op, const void *ptr)
{
	target_comms_arg_t ca = arg;
	struct target_console_state *ts = ca->ca_ops_arg;
	const netusb_mux_event_data_t *ed = ptr;
	int rv = 0;

	switch (op) {
	case PORT_DAEMON_CTRL_BREAK:
		break;

	case PORT_DAEMON_CTRL_UART_CONFIG:
	case PORT_DAEMON_CTRL_USB_CONFIG:
		if (uart_configure(ts->ts_uart_handle, ed->ed_usb_cfg.mc_cfg,
		    ed->ed_usb_cfg.mc_baud) == 0) {
			rv = -1;
		}
		break;

	default:
		break;
	}

	return rv;
}

static void
target_console_worker(void *arg)
{
	target_comms_arg_t ca = arg;
	struct target_console_state *ts = ca->ca_ops_arg;

	target_comms_generic_worker(ts->ts_bounce, ts->ts_to_host,
	    ca->ca_flags);
}

void
target_console_init(void)
{
	static const struct port_daemon_ops target_console_ops = {
		.op_connect = target_console_connect,
		.op_disconnect = target_console_disconnect,
		.op_worker = target_console_worker,
		.op_ctrl = target_console_ctrl,
		.op_hangup = NULL
	};
	static struct target_comms_source cs;
	struct target_console_state *ts;
	uint32_t baud;
	uint16_t cfg;

	ts = &target_console_state;
	cs.cs_port = TARGET_COMMS_PORT_TARGET_CONSOLE;
	cs.cs_ops = &target_console_ops;
	cs.cs_arg.ca_ops_arg = ts;

	ts->ts_bounce = ringbuff_alloc(ts->ts_rb_bounce,
	    sizeof(ts->ts_rb_bounce));
	assert(ts->ts_bounce != NULL);

	/* Attach the target serial port */
	ts->ts_uart_id = platform_uart_attach(PLATFORM_UART_TARGET);
	assert(ts->ts_uart_id >= 0);

	ts->ts_uart_handle = uart_claim(ts->ts_uart_id);
	assert(ts->ts_uart_handle != NULL);

	cfg = target_console_get_uart_cfg(&baud);
	uart_configure(ts->ts_uart_handle, cfg, baud);

	target_comms_register(&cs);

#ifdef PLATFORM_WILC_UART
	extern void platform_wilc_uart_attach(void);
	platform_wilc_uart_attach();
#endif
}
