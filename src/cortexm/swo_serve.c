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
#include <stdint.h>
#include <stdio.h>

#include "swo_serve.h"
#include "target_comms.h"
#include "port_daemon.h"
#include "ringbuff.h"
#include "uart.h"

#define	SWO_SERVE_BOUNCE_RB_SIZE	6144u

struct swo_state {
	port_daemon_instance_t ss_port_daemon;
	ringbuff_t ss_to_host;
	ringbuff_t ss_from_host;
	ringbuff_t ss_bounce;
	target_comms_config_flags_t ss_flags;
	void *ss_swo_handle;
	uint16_t ss_uart_cfg;
	uint32_t ss_uart_baud;
	uint8_t ss_rb_bounce[SWO_SERVE_BOUNCE_RB_SIZE];
};
static struct swo_state swo_state;

static void
swo_serve_rb_callback(ringbuff_t rb, void *arg)
{
	struct swo_state *ss = arg;

	(void) rb;

	port_daemon_attention(ss->ss_port_daemon);
}

static bool
swo_connect(void *arg, port_daemon_instance_t si, ringbuff_t to_host,
    ringbuff_t from_host, unsigned int unused)
{
	target_comms_arg_t ca = arg;
	struct swo_state *ss = ca->ca_ops_arg;
	ringbuff_t uart_rb;

	(void) unused;

	if (ss->ss_port_daemon != NULL)
		return false;

	ss->ss_port_daemon = si;
	ss->ss_to_host = to_host;
	ss->ss_from_host = from_host;

	/*
	 * We maintain a copy of the original flags because the connection
	 * configuration depends on them remaining unchanged for the
	 * duration. The user may tweak them via the 'tc' command, and
	 * that would break our worker function quite badly.
	 */
	ss->ss_flags = ca->ca_flags;

	if (TC_FLAGS_IS_RAW(ss->ss_flags)) {
		/*
		 * For a raw connection, we tie the UART and host together
		 * directly, for maximum performance. For USB connections,
		 * the SWO data is handled entirely in interrupt mode with
		 * zero copy.
		 */
		uart_rb = to_host;
	} else {
		/*
		 * Cooked connections go via the bounce buffer in the
		 * usual way.
		 */
		uart_rb = ss->ss_bounce;
		ringbuff_init(ss->ss_bounce);
		ringbuff_consumer_init(ss->ss_bounce, swo_serve_rb_callback, ss);
		ringbuff_producer_init(to_host, swo_serve_rb_callback, ss);
	}

	/* We need to eat anything sent by the host. */
	ringbuff_consumer_init(from_host, swo_serve_rb_callback, ss);

	/* Bring up the SWO UART */
	uart_open(ss->ss_swo_handle, uart_rb, NULL);

	/* Change configuration if necessary. */
	if (ss->ss_uart_cfg != 0) {
		if (uart_configure(ss->ss_swo_handle, ss->ss_uart_cfg,
		    ss->ss_uart_baud) == 0) {
#if (RELEASE_BUILD == 0)
			printf("swo_connect: cannot configure cfg %" PRIx16
			    ", baud %" PRIu32 "\n", ss->ss_uart_cfg,
			    ss->ss_uart_baud);
#endif
		}
	}

	return true;
}

static void
swo_disconnect(void *arg)
{
	target_comms_arg_t ca = arg;
	struct swo_state *ss = ca->ca_ops_arg;

	uart_close(ss->ss_swo_handle);

	ss->ss_port_daemon = NULL;
}

static int
swo_ctrl(void *arg, port_daemon_ctrl_t op, const void *ptr)
{
	target_comms_arg_t ca = arg;
	struct swo_state *ss = ca->ca_ops_arg;
	const netusb_mux_event_data_t *ed = ptr;
	uint16_t cfg;

	if (op != PORT_DAEMON_CTRL_UART_CONFIG)
		return 0;

	cfg = ed->ed_usb_cfg.mc_cfg & ~(UART_CHARSIZE_MASK | UART_PARITY_MASK |
	    UART_STOPBITS_MASK);
	cfg |= UART_CHARSIZE_8 | UART_PARITY_NONE | UART_STOPBITS_1;
	ss->ss_uart_cfg = cfg;
	ss->ss_uart_baud = ed->ed_usb_cfg.mc_baud;

	if (uart_configure(ss->ss_swo_handle, cfg, ss->ss_uart_baud) == 0) {
#if (RELEASE_BUILD == 0)
		printf("swo_ctrl: cannot configure cfg %" PRIx16
		    ", baud %" PRIu32 "\n", cfg, ss->ss_uart_baud);
#endif
		return -1;
	}

	return 0;
}

static void
swo_worker(void *arg)
{
	target_comms_arg_t ca = arg;
	struct swo_state *ss = ca->ca_ops_arg;
	ringbuff_len_t l;
	bool consumed;

	if (!TC_FLAGS_IS_RAW(ss->ss_flags)) {
		/* Handle target->host data in cooked mode. */
		target_comms_generic_worker(ss->ss_bounce, ss->ss_to_host,
		    ca->ca_flags);
	}

	/* Just eat anything the host sends our way. */
	consumed = false;
	while ((l = ringbuff_consume_contig_size(ss->ss_from_host)) != 0) {
		consumed = true;
		ringbuff_consume_contig(ss->ss_from_host, l);
	}

	if (consumed)
		ringbuff_consume_done(ss->ss_from_host);
}

static void *
swo_serve_attach(void)
{
	struct target_comms_config tcc;
	void *rv;
	int id;

	if ((id = platform_uart_attach(PLATFORM_UART_SWO)) < 0)
		return NULL;

	if ((rv = uart_claim((uint8_t)id)) == NULL)
		return NULL;

	if (!target_comms_get_config(TARGET_COMMS_PORT_SWO, &tcc))
		uart_configure(rv, UART_DEFAULT_CFG, 0);
	else
		uart_configure(rv, tcc.tcc_uart_cfg, tcc.tcc_uart_baud);

	return rv;
}

void
swo_serve_init(void)
{
	static const struct port_daemon_ops swo_ops = {
		.op_connect = swo_connect,
		.op_disconnect = swo_disconnect,
		.op_worker = swo_worker,
		.op_ctrl = swo_ctrl,
		.op_hangup = NULL
	};
	static struct target_comms_source cs;
	struct swo_state *ss;

	ss = &swo_state;

	ss->ss_swo_handle = swo_serve_attach();
	assert(ss->ss_swo_handle != NULL);

	ss->ss_bounce = ringbuff_alloc(ss->ss_rb_bounce,
	    sizeof(ss->ss_rb_bounce));
	assert(ss->ss_bounce != NULL);

	cs.cs_port = TARGET_COMMS_PORT_SWO;
	cs.cs_ops = &swo_ops;
	cs.cs_arg.ca_ops_arg = ss;

	target_comms_register(&cs);
}
