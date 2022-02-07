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
#include <strings.h>

#include "rtos.h"
#include "port_daemon.h"
#include "netusb_mux.h"
#include "linked-lists.h"
#include "zone_alloc.h"

/*
 * Our built-in chooser requires some state.
 */
struct port_daemon_chooser_state {
	/* Array of chooser menu options. */
	const char * const *cs_options;
	unsigned int cs_num_options;

	/* Pointers to the source's original ops structure and context. */
	struct port_daemon_ops const *cs_orig_ops;
	void *cs_orig_ops_arg;

	/* USB UART config received while in chooser mode. */
	struct netusb_mux_usb_config cs_usb_cfg;
};

/*
 * One source instance.
 */
struct port_daemon_instance {
	/* Describes the source connections and flags. */
	struct port_daemon_settings const *si_settings;

	/* Provides the source operations API. */
	struct port_daemon_ops const *si_ops;
#define	si_op_connect		si_ops->op_connect
#define	si_op_disconnect	si_ops->op_disconnect
#define	si_op_worker		si_ops->op_worker
#define	si_op_hangup		si_ops->op_hangup
#define	si_op_ctrl		si_ops->op_ctrl
	void *si_ops_arg;

	/* True if a connection to the source is established. */
	bool si_connected;

	/* Bitmap of events for this source. */
	volatile uint32_t si_events;
#define	PORT_DAEMON_EVENT_NETUSB_MUX_POLL	(1u << 0)
#define	PORT_DAEMON_EVENT_WORKER		(1u << 1)

	/* Ring buffers to/from the host, via netusb_mux. */
	ringbuff_t si_to_mux;
	ringbuff_t si_from_mux;

	/* This source's link to the netusb_mux. */
	netusb_mux_state_t si_mux;

	/* Work-queue link when source has events. */
	TAILQ_ENTRY(port_daemon_instance) si_qent;

	/* Chooser state. */
	struct port_daemon_chooser_state si_chooser;

	/* Source's host-facing ring buffers. */
	uint8_t *si_to_mux_ring;
	uint8_t *si_from_mux_ring;
};
/* Returns 'true' is the source needs to use the chooser. */
#define	si_need_chooser(si)	((si)->si_chooser.cs_options != NULL)

TAILQ_HEAD(port_daemon_source_instance_qhead, port_daemon_instance);

/*
 * Overall port_daemon state
 */
typedef struct port_daemon_state {
	/* Queue of sources requesting service (si_events != 0) */
	struct port_daemon_source_instance_qhead ds_work;

	/* Context for the PortDaemon thread. */
	rtos_task_t ds_task;
} *port_daemon_state_t;
static struct port_daemon_state port_daemon_state;

/*
 * Source 'ops' for our built-in chooser.
 * The chooser ops override the source's real ops, until the user has
 * made a valid choice.
 */
static bool chooser_connect(void *, port_daemon_instance_t, ringbuff_t,
		ringbuff_t, unsigned int);
static void chooser_disconnect(void *);
static void chooser_worker(void *);
static int chooser_ctrl(void *, port_daemon_ctrl_t, const void *);

static const struct port_daemon_ops chooser_ops = {
	.op_connect = chooser_connect,
	.op_disconnect = chooser_disconnect,
	.op_worker = chooser_worker,
	.op_ctrl = chooser_ctrl,
	.op_hangup = NULL
};

static void
port_daemon_set_event(port_daemon_instance_t si, uint32_t event)
{
	port_daemon_state_t ds = &port_daemon_state;
	rtos_saved_ipl_t ipl;
	uint32_t old;

	/* Ensure we can't be interrupted. */
	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	/* Record the new event. */
	old = si->si_events;
	si->si_events = old | event;

	/* If this is the first event, add the source to the work queue. */
	if (old == 0)
		TAILQ_INSERT_TAIL(&ds->ds_work, si, si_qent);

	/* Interrupts are ok now. */
	rtos_ipl_restore(ipl);

	/* If this is the first event, wakeup the PortDaemon thread. */
	if (old == 0)
		rtos_notify_give_any(ds->ds_task);
}

static port_daemon_instance_t
port_daemon_get_event(port_daemon_state_t ds, uint32_t *pevents)
{
	port_daemon_instance_t si;
	rtos_saved_ipl_t ipl;

	/* No interrupts please. */
	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	/* Fetch the first item on the work queue. */
	if ((si = TAILQ_FIRST(&ds->ds_work)) != NULL) {
		/* Got one. Remove it from the queue. */
		TAILQ_REMOVE(&ds->ds_work, si, si_qent);

		/* Grab its event bitmap, and reset to zero. */
		*pevents = si->si_events;
		si->si_events = 0;
	}

	/* Interrupts are ok now. */
	rtos_ipl_restore(ipl);

	return si;
}

static void
port_daemon_netusb_mux_poll_cb(void *arg)
{
	port_daemon_instance_t si = arg;

	/*
	 * The netusb_mux API requests we invoke netusb_mux_poll() from
	 * PortDaemon's thread.
	 */
	port_daemon_set_event(si, PORT_DAEMON_EVENT_NETUSB_MUX_POLL);
}

static void
port_daemon_netusb_mux_hangup_cb(void *arg)
{
	port_daemon_instance_t si = arg;

	/*
	 * The netusb_mux API is providing early warning of a hang-up
	 * event. If the source has supplied us with a hang-up method,
	 * invoke it now.
	 */
	if (si->si_op_hangup != NULL)
		si->si_op_hangup(si->si_ops_arg);
}

static void
port_daemon_disconnect_cleanup(port_daemon_instance_t si)
{

	/*
	 * An established connection has been disconnected.
	 * Perform all the necessary clean-up in preparation for
	 * new connections.
	 */

	/* Flag the new status. */
	si->si_connected = false;

	/* Start with fresh ring buffers. */
	ringbuff_init(si->si_to_mux);
	ringbuff_init(si->si_from_mux);

	/* Re-install the chooser API if required. */
	if (si_need_chooser(si)) {
		si->si_ops = &chooser_ops;
		si->si_ops_arg = si;
	}
}

static void
port_daemon_poll_event(port_daemon_state_t ds, port_daemon_instance_t si)
{
	netusb_mux_event_data_t ed;
	netusb_mux_event_t ev;

	(void) ds;

	/*
	 * The netusb_mux API has requested service. Invoke netusb_mux_poll()
	 * until it returns NETUSB_MUX_EVENT_NONE.
	 */

	while ((ev = netusb_mux_poll(si->si_mux, &ed)) !=
	    NETUSB_MUX_EVENT_NONE) {
		/*
		 * We have some work to do.
		 */
		switch (ev) {
		case NETUSB_MUX_EVENT_CONNECT_NET:
		case NETUSB_MUX_EVENT_CONNECT_USB:
			/*
			 * We have a connection. This can only happen
			 * while we are disconnected...
			 */
			assert(si->si_connected == false);

			/* Invoke the source's "connect" op. */
			si->si_connected = si->si_op_connect(si->si_ops_arg,
			    si, si->si_to_mux, si->si_from_mux, 0);

			/* Success? */
			if (si->si_connected == false) {
				/* Failed to. Clean up. */
				port_daemon_disconnect_cleanup(si);

				/*
				 * For non-chooser connections, we
				 * drop the link.
				 */
				if (si_need_chooser(si))
					netusb_mux_close(si->si_mux);
			}
			break;

		case NETUSB_MUX_EVENT_DISCONNECT:
			/* Host has disconnected. */
			if (si->si_connected)
				si->si_op_disconnect(si->si_ops_arg);
			port_daemon_disconnect_cleanup(si);
			break;

		case NETUSB_MUX_EVENT_BREAK:
			if (si->si_connected && si->si_op_ctrl != NULL) {
				si->si_op_ctrl(si->si_ops_arg,
				    PORT_DAEMON_CTRL_BREAK, NULL);
			}
			break;

		case NETUSB_MUX_EVENT_USB_CONFIG:
			if (si->si_connected && si->si_op_ctrl != NULL) {
				si->si_op_ctrl(si->si_ops_arg,
				    PORT_DAEMON_CTRL_USB_CONFIG, &ed);
			}
			break;

		default:
			break;
		}
	}
}

static void
port_daemon_task(void *arg)
{
	port_daemon_state_t ds = arg;
	port_daemon_instance_t si;
	uint32_t events;

	/*
	 * The main PortDaemon work loop.
	 */

	for (;;) {
		/* Sit around until we have some work to do. */
		rtos_notify_take(1);

		/* Run through all entries on the work queue. */
		while ((si = port_daemon_get_event(ds, &events)) != NULL) {
			if (events & PORT_DAEMON_EVENT_NETUSB_MUX_POLL) {
				/* Handle netusb_mux events. */
				port_daemon_poll_event(ds, si);
			}

			if (events & PORT_DAEMON_EVENT_WORKER) {
				/* Invoke the source's 'worker' op. */
				if (si->si_connected)
					si->si_op_worker(si->si_ops_arg);
			}
		}
	}
}

static void
chooser_from_mux_cb(ringbuff_t rb, void *arg)
{
	port_daemon_instance_t si = arg;

	(void) rb;

	/*
	 * We've received input from the host in response to our
	 * chooser prompt. Tell PortDaemon this source needs
	 * some attention.
	 */

	port_daemon_attention(si);
}

static void
chooser_write_string(ringbuff_t rb, const char *str)
{

	/* Output a string to the specified ring buffer. */
	while (*str != '\0')
		ringbuff_produce(rb, (uint8_t)*str++);
}

static void
chooser_menu(port_daemon_instance_t si)
{
	static const char chooser_header[] =
	    "\r\nPlease choose from one of the following options:";
	static const char chooser_prompt[] = "\r\nChoice: ";
	ringbuff_t rb = si->si_to_mux;
	unsigned int i;
	char buff[8];

	/*
	 * Send the 'chooser' menu to the host.
	 */

	/* Header */
	chooser_write_string(rb, chooser_header);

	/* Each of the numbered options. */
	for (i = 0; i < si->si_chooser.cs_num_options; i++) {
		assert(si->si_chooser.cs_options[i] != NULL);
		snprintf(buff, sizeof(buff), "\r\n %u  ", i + 1);
		chooser_write_string(rb, buff);
		chooser_write_string(rb, si->si_chooser.cs_options[i]);
	}

	/* Finally the prompt. */
	chooser_write_string(rb, chooser_prompt);

	/* Send it on its way. */
	ringbuff_produce_done(rb);
}

static bool
chooser_connect(void *arg, port_daemon_instance_t si, ringbuff_t to_mux,
    ringbuff_t from_mux, unsigned int unused)
{

	(void) arg;
	(void) to_mux;
	(void) unused;

	/*
	 * The chooser service has received a new connection.
	 */

	/* Reset the stashed USB config. */
	si->si_chooser.cs_usb_cfg.mc_cfg = 0;
	si->si_chooser.cs_usb_cfg.mc_baud = 0;

	/* We need notification when data is available from the netusb_mux. */
	ringbuff_consumer_init(from_mux, chooser_from_mux_cb, si);

	/* Send the chooser menu. */
	chooser_menu(si);

	return true;
}

static void
chooser_disconnect(void *arg)
{

	(void) arg;

	/* For the chooser, this is a no-op. */
}

static void
chooser_worker(void *arg)
{
	port_daemon_instance_t si = arg;
	static const char failure_str[] = "Connection failed\r\n";
	unsigned int chosen = 0;
	uint8_t ch;

	/*
	 * Process characters received from the host.
	 */

	/*
	 * Consume all data in the ring buffer, checking for a valid
	 * choice as we go.
	 */
	while (ringbuff_get_count(si->si_from_mux)) {
		ch = ringbuff_consume(si->si_from_mux);

		/* If we already found a valid option, just consume. */
		if (chosen)
			continue;

		/* Otherwise, validate the input. */
		if (ch < (uint8_t)'1')
			continue;
		ch -= (uint8_t)'0';

		if (ch > si->si_chooser.cs_num_options)
			continue;

		/* Found a valid choice. */
		chosen = ch;
	}

	/* If no valid choice, we're done. */
	if (chosen == 0)
		return;

	/*
	 * We have a valid choice.
	 * Reconfigure to use the chosen source ops.
	 */
	si->si_ops = si->si_chooser.cs_orig_ops;
	si->si_ops_arg = si->si_chooser.cs_orig_ops_arg;
	ringbuff_consumer_init(si->si_from_mux, NULL, si);

	/* Echo the choice, followed by a newline. */
	ringbuff_produce(si->si_to_mux, (uint8_t)'0' + chosen);
	ringbuff_produce(si->si_to_mux, (uint8_t)'\r');
	ringbuff_produce(si->si_to_mux, (uint8_t)'\n');
	ringbuff_produce_done(si->si_to_mux);

	/* Attempt to connect to the real source. */
	if (si->si_op_connect(si->si_ops_arg, si, si->si_to_mux,
	    si->si_from_mux, chosen - 1) == false) {
		/* Connection failed. Perhaps it's busy. */
		si->si_ops = &chooser_ops;
		si->si_ops_arg = si;

		/* Consumer side of ringbuff might need re-init'ing. */
		ringbuff_consumer_init(si->si_from_mux, chooser_from_mux_cb,si);

		/* Tell the user the connection failed. */
		chooser_write_string(si->si_to_mux, failure_str);

		/* Re-write the chooser prompt. */
		chooser_menu(si);
	} else
	if (si->si_chooser.cs_usb_cfg.mc_cfg != 0 && si->si_op_ctrl != NULL) {
		/*
		 * Connect succeeded, but we saw a USB config message from the
		 * mux which now needs to be relayed to the real source.
		 */
		si->si_op_ctrl(si->si_ops_arg, PORT_DAEMON_CTRL_USB_CONFIG,
		    &si->si_chooser.cs_usb_cfg);
	}
}

static int
chooser_ctrl(void *arg, port_daemon_ctrl_t op, const void *ptr)
{
	port_daemon_instance_t si = arg;

	/*
	 * When our connection is via USB, we will likely see a UART config
	 * while in chooser mode. Stash it here; we will relay it to the
	 * chosen source later.
	 */
	if (op == PORT_DAEMON_CTRL_USB_CONFIG) {
		const struct netusb_mux_usb_config *cfg = ptr;
		si->si_chooser.cs_usb_cfg = *cfg;
		return 1;
	}

	return 0;
}

port_daemon_instance_t
port_daemon_register_source(const port_daemon_settings_t settings,
    const port_daemon_ops_t ops, void *ops_arg,
    const char * const *options, unsigned int num_options)
{
	struct netusb_mux_attach_args aa;
	port_daemon_instance_t si;

	/*
	 * Ensure the API is being used correctly.
	 */
	assert(settings->s_usb_port != PORT_DAEMON_USB_PORT_UNUSED ||
	    settings->s_tcp_port != PORT_DAEMON_TCP_PORT_UNUSED);
	assert(options == NULL || num_options > 0);

	si = zone_calloc(1, sizeof(*si));
	if (si == NULL)
		return NULL;

	/* Enable the chooser, if so desired. */
	if (options != NULL) {
		si->si_chooser.cs_options = options;
		si->si_chooser.cs_num_options = num_options;
		si->si_chooser.cs_orig_ops = ops;
		si->si_chooser.cs_orig_ops_arg = ops_arg;
		si->si_ops = &chooser_ops;
		si->si_ops_arg = si;
	} else {
		/* Record the source's ops. */
		si->si_chooser.cs_options = NULL;
		si->si_chooser.cs_num_options = 0;
		si->si_ops = ops;
		si->si_ops_arg = ops_arg;
	}

	si->si_settings = settings;
	si->si_events = 0;

	/* Prepare the ring buffers */
	assert(settings->s_rb_size_to_host >= 4);
	si->si_to_mux_ring = zone_malloc(settings->s_rb_size_to_host);
	if (si->si_to_mux_ring == NULL) {
		zone_free(si);
		return NULL;
	}
	si->si_to_mux = ringbuff_alloc(si->si_to_mux_ring,
	    settings->s_rb_size_to_host);
	if (si->si_to_mux == NULL) {
		zone_free(si->si_to_mux_ring);
		zone_free(si);
		return NULL;
	}

	assert(settings->s_rb_size_from_host >= 4);
	si->si_from_mux_ring = zone_malloc(settings->s_rb_size_from_host);
	if (si->si_from_mux_ring == NULL) {
		ringbuff_free(si->si_to_mux);
		zone_free(si->si_to_mux_ring);
		zone_free(si);
		return NULL;
	}
	si->si_from_mux = ringbuff_alloc(si->si_from_mux_ring,
	    settings->s_rb_size_from_host);
	if (si->si_from_mux == NULL) {
		zone_free(si->si_from_mux_ring);
		ringbuff_free(si->si_to_mux);
		zone_free(si->si_to_mux_ring);
		zone_free(si);
		return NULL;
	}

	/*
	 * Our host connection state is handled via the netusb_mux API.
	 * Construct the required attach data.
	 */
	aa.aa_poll_cb = port_daemon_netusb_mux_poll_cb;
	if (si->si_op_hangup != NULL)
		aa.aa_hangup_cb = port_daemon_netusb_mux_hangup_cb;
	else
		aa.aa_hangup_cb = NULL;
	aa.aa_cb_arg = si;
	aa.aa_rb_to_host = si->si_to_mux;
	aa.aa_rb_from_host = si->si_from_mux;
#ifdef CONFIG_USE_NETWORK
	aa.aa_tcp_port = settings->s_tcp_port;
	aa.aa_do_telnet =
	    (settings->s_flags & PORT_DAEMON_SOURCE_FLAG_DO_TELNET) != 0;
	aa.aa_do_nagle =
	    (settings->s_flags & PORT_DAEMON_SOURCE_FLAG_DO_NAGLE) != 0;
#endif
	aa.aa_usb_port = settings->s_usb_port;
	aa.aa_usb_cfg = 0;
	aa.aa_usb_custom_baud = 0;

	si->si_mux = netusb_mux_attach(&aa);
	if (si->si_mux == NULL) {
		ringbuff_free(si->si_from_mux);
		zone_free(si->si_from_mux_ring);
		ringbuff_free(si->si_to_mux);
		zone_free(si->si_to_mux_ring);
		zone_free(si);
		return NULL;
	}

	/*
	 * The netusb_mux API requires an initial call in order to
	 * set up the network listener and/or claim the USB port.
	 */
	port_daemon_set_event(si, PORT_DAEMON_EVENT_NETUSB_MUX_POLL);

	return si;
}

void
port_daemon_attention(port_daemon_instance_t si)
{

	/*
	 * The source requires a call to its 'worker' op.
	 * Arrange for the call to be made from the port_daemon's thread.
	 */
	port_daemon_set_event(si, PORT_DAEMON_EVENT_WORKER);
}

void
port_daemon_change_flags(port_daemon_instance_t si, uint8_t flags)
{
	bool do_telnet, do_nagle;

	do_telnet = (flags & PORT_DAEMON_SOURCE_FLAG_DO_TELNET) != 0;
	do_nagle = (flags & PORT_DAEMON_SOURCE_FLAG_DO_NAGLE) != 0;

	netusb_mux_tcp_options(si->si_mux, do_telnet, do_nagle);
}

void
port_daemon_init(void)
{
	port_daemon_state_t ds = &port_daemon_state;

	TAILQ_INIT(&ds->ds_work);

	ds->ds_task = rtos_task_create("PortDaemon", port_daemon_task, ds,
	    RTOS_TASK_PRIORITY_MED, RTOS_STACK_SIZE(1200));
	assert(ds->ds_task != NULL);
}
