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
#include "netusb_mux.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#endif
#include "uart.h"
#include "usb_common.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

typedef enum {
	NETUSB_MUX_OWNER_NOBODY,
	NETUSB_MUX_OWNER_NETWORK,
	NETUSB_MUX_OWNER_USB
} netusb_mux_owner_t;

typedef enum {
	NETUSB_MUX_NOTIFY_NONE = 0,
	NETUSB_MUX_NOTIFY_USB_ATTACH,
	NETUSB_MUX_NOTIFY_USB_DETACH,

	NETUSB_MUX_NOTIFY_NET_HANGUP,
	NETUSB_MUX_NOTIFY_USB_HANGUP,

	NETUSB_MUX_NOTIFY_NET_CONNECT,
	NETUSB_MUX_NOTIFY_NET_TELNET,
	NETUSB_MUX_NOTIFY_USB_CONNECT,

	NETUSB_MUX_NOTIFY_NET_BREAK,
	NETUSB_MUX_NOTIFY_USB_BREAK,

	NETUSB_MUX_NOTIFY_USB_CONFIG,
	NETUSB_MUX_NOTIFY__COUNT
} netusb_mux_notify_t;

#define	NETUSB_MUX_NOTIFY_NET	((1u << NETUSB_MUX_NOTIFY_NET_HANGUP) | \
				 (1u << NETUSB_MUX_NOTIFY_NET_CONNECT) | \
				 (1u << NETUSB_MUX_NOTIFY_NET_TELNET) | \
				 (1u << NETUSB_MUX_NOTIFY_NET_BREAK))
#define	NETUSB_MUX_NOTIFY_USB	((1u << NETUSB_MUX_NOTIFY_USB_ATTACH) | \
				 (1u << NETUSB_MUX_NOTIFY_USB_DETACH) | \
				 (1u << NETUSB_MUX_NOTIFY_USB_CONNECT) | \
				 (1u << NETUSB_MUX_NOTIFY_USB_BREAK) | \
				 (1u << NETUSB_MUX_NOTIFY_USB_CONFIG))

struct netusb_mux_state {
	struct netusb_mux_attach_args ms_aa;
#define	ms_poll_cb		ms_aa.aa_poll_cb
#define	ms_hangup_cb		ms_aa.aa_hangup_cb
#define	ms_cb_arg		ms_aa.aa_cb_arg
#define	ms_rb_to_host		ms_aa.aa_rb_to_host
#define	ms_rb_from_host		ms_aa.aa_rb_from_host
#define	ms_tcp_port		ms_aa.aa_tcp_port
#define	ms_do_telnet		ms_aa.aa_do_telnet
#define	ms_do_nagle		ms_aa.aa_do_nagle
#ifdef CONFIG_USE_USB
#define	ms_usb_port		ms_aa.aa_usb_port
#define	ms_usb_cfg		ms_aa.aa_usb_cfg
#define	ms_usb_custom_baud	ms_aa.aa_usb_custom_baud
#endif

	netusb_mux_owner_t ms_owner;
	volatile uint32_t ms_notify_mask;
	rtos_timer_t ms_timer;

#ifdef CONFIG_USE_NETWORK
	void *ms_socket;
	bool ms_wait_telnet;
	network_sock_params_t ms_peer;
#endif

#ifdef CONFIG_USE_USB
	void *ms_usb;
#endif
};

static void
netusb_mux_notify(netusb_mux_state_t ms, netusb_mux_notify_t notification)
{
	rtos_saved_ipl_t ipl;
	uint32_t mask, old;

	mask = 1u << (unsigned int)notification;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	old = ms->ms_notify_mask;
	ms->ms_notify_mask = old | mask;
	rtos_ipl_restore(ipl);

	if ((old & mask) == 0)
		(ms->ms_poll_cb)(ms->ms_cb_arg);
}

static netusb_mux_notify_t
netusb_mux_get_notification(netusb_mux_state_t ms)
{
	rtos_saved_ipl_t ipl;
	uint32_t mask;
	int bit;

	assert(ms->ms_notify_mask != 0);

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	mask = ms->ms_notify_mask;
	bit = ffs((int)mask) - 1;
	mask &= ~(1u << bit);
	ms->ms_notify_mask = mask;

	rtos_ipl_restore(ipl);

	return (netusb_mux_notify_t)bit;
}

static void
netusb_mux_tcp_timer_cb(rtos_timer_t t)
{
	netusb_mux_state_t ms = rtos_timer_get_id(t);

	/* Notify our consumer that we need attention. */ 
	(ms->ms_poll_cb)(ms->ms_cb_arg);
}

#ifdef CONFIG_USE_NETWORK
static void
netusb_mux_socket_status_cb(network_sock_status_t st, void *arg)
{
	netusb_mux_state_t ms = arg;
	netusb_mux_notify_t notification;

	switch (st) {
	case NETWORK_STATUS_CONNECT:
		notification = NETUSB_MUX_NOTIFY_NET_CONNECT;
		break;

	case NETWORK_STATUS_TELNET_READY:
		notification = NETUSB_MUX_NOTIFY_NET_TELNET;
		break;

	case NETWORK_STATUS_TELNET_BREAK:
		notification = NETUSB_MUX_NOTIFY_NET_BREAK;
		break;

	case NETWORK_STATUS_DISCONNECT:
	case NETWORK_STATUS_TIMEOUT:
	case NETWORK_STATUS_LINK_DOWN:
	case NETWORK_STATUS_UNREACHABLE:
		DBFPRINTF("event %u\n", st);
		notification = NETUSB_MUX_NOTIFY_NET_HANGUP;
		if (ms->ms_owner == NETUSB_MUX_OWNER_NETWORK &&
		    ms->ms_hangup_cb != NULL) {
			DBFPRINTF("call hangup\n");
			(ms->ms_hangup_cb)(ms->ms_cb_arg);
		}
		break;

	default:
		notification = NETUSB_MUX_NOTIFY_NONE;
		break;
	}

	if (notification != NETUSB_MUX_NOTIFY_NONE)
		netusb_mux_notify(ms, notification);
}

static void
netusb_mux_restart_network(netusb_mux_state_t ms)
{

	if (ms->ms_tcp_port == NETUSB_MUX_TCP_PORT_UNUSED)
		return;

	if (ms->ms_socket != NULL) {
		network_close(ms->ms_socket);
		ms->ms_socket = NULL;
	}
	ms->ms_wait_telnet = false;

	if (ms->ms_owner == NETUSB_MUX_OWNER_NOBODY) {
		ms->ms_socket = network_socket_listen(ms->ms_tcp_port,
		    netusb_mux_socket_status_cb, ms);

		/*
		 * The above call will fail if the network is down.
		 * Schedule a poll event so we can retry later.
		 */
		if (ms->ms_socket == NULL)
			rtos_timer_start(ms->ms_timer);
	}
}

static netusb_mux_event_t
netusb_mux_notify_net_hangup(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{
	netusb_mux_event_t rv = NETUSB_MUX_EVENT_NONE;

	(void) ed;

	DBFPRINTF("owner %s, do_telnet %u, wait_telnet %u\n",
	    (ms->ms_owner == NETUSB_MUX_OWNER_NETWORK) ? "net" :
	    ((ms->ms_owner == NETUSB_MUX_OWNER_USB) ? "usb" : "none"),
	    ms->ms_do_telnet, ms->ms_wait_telnet);

	/*
	 * Return NETUSB_MUX_EVENT_DISCONNECT iff a valid connection
	 * was established.
	 */
	if (ms->ms_owner == NETUSB_MUX_OWNER_NETWORK) {
		DBFPRINTF("clearing ownership\n");
		ms->ms_owner = NETUSB_MUX_OWNER_NOBODY;
		if (ms->ms_do_telnet == false || ms->ms_wait_telnet == false)
			rv = NETUSB_MUX_EVENT_DISCONNECT;
		else
			DBFPRINTF("ignored\n");
	}

	/* Handle network restart. */
	netusb_mux_restart_network(ms);

	return rv;
}

static netusb_mux_event_t
netusb_mux_notify_net_connect(netusb_mux_state_t ms,
    netusb_mux_event_data_t *ed)
{

	assert(ms->ms_tcp_port != NETUSB_MUX_TCP_PORT_UNUSED);

	/* We should never see this while already owned by the network. */
	assert(ms->ms_owner != NETUSB_MUX_OWNER_NETWORK);

	/* Ignore if already owned by someone else. */
	if (ms->ms_owner != NETUSB_MUX_OWNER_NOBODY)
		return NETUSB_MUX_EVENT_NONE;

	/* A valid network connection attempt. Try to establish the link. */
	if (network_establish(ms->ms_socket, ms->ms_rb_to_host,
	    ms->ms_rb_from_host, ms->ms_do_telnet, &ms->ms_peer) != 0) {
		/* Connection failed for some reason. */
		netusb_mux_restart_network(ms);
		return NETUSB_MUX_EVENT_NONE;
	}

	/* Success. Network now owns the connection. */
	ms->ms_owner = NETUSB_MUX_OWNER_NETWORK;

	/* Configure the required settings. */
	network_tcp_nagle(ms->ms_socket, ms->ms_do_nagle);

	/* Non-telnet connections are now established. */
	if (ms->ms_do_telnet == false) {
		/* Provide peer details to user, if required. */
		if (ed != NULL)
			ed->ed_tcp_peer = ms->ms_peer;
		return NETUSB_MUX_EVENT_CONNECT_NET;
	}

	/* Otherwise, we need to wait for telnet negotiation to complete. */
	ms->ms_wait_telnet = true;
	return NETUSB_MUX_EVENT_NONE;
}

static netusb_mux_event_t
netusb_mux_notify_net_telnet(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	assert(ms->ms_owner == NETUSB_MUX_OWNER_NETWORK);
	assert(ms->ms_wait_telnet == true);

	/* No longer waiting for telnet negotiation. */
	ms->ms_wait_telnet = false;

	/* Provide peer details to user, if required. */
	if (ed != NULL)
		ed->ed_tcp_peer = ms->ms_peer;

	return NETUSB_MUX_EVENT_CONNECT_NET;
}

static netusb_mux_event_t
netusb_mux_notify_net_break(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	assert(ms->ms_owner == NETUSB_MUX_OWNER_NETWORK);
	assert(ms->ms_do_telnet == true);

	(void) ms;
	(void) ed;

	return NETUSB_MUX_EVENT_BREAK;
}
#endif /* CONFIG_USE_NETWORK */

#ifdef CONFIG_USE_USB
static void
netusb_mux_usb_status_cb(usb_cdc_status_t st, void *arg)
{
	netusb_mux_state_t ms = arg;
	netusb_mux_notify_t notification;

	/* Note: This can be invoked from interrupt mode for Atmel USB driver */

	assert(ms->ms_usb != NULL);

	switch (st) {
	case USB_CDC_STATUS_ATTACHED:
		/* USB interface has just been plugged into a host */
		notification = NETUSB_MUX_NOTIFY_USB_ATTACH;
		break;

	case USB_CDC_STATUS_DETACHED:
		/* USB interface has just been unplugged */
		notification = NETUSB_MUX_NOTIFY_USB_DETACH;
		break;

	case USB_CDC_STATUS_CONNECTED:
		/* User has opened the USB device on host */
		notification = NETUSB_MUX_NOTIFY_USB_CONNECT;
		break;

	case USB_CDC_STATUS_HANGUP:
		/* User has closed the USB device on host */
		notification = NETUSB_MUX_NOTIFY_USB_HANGUP;
		if (ms->ms_owner == NETUSB_MUX_OWNER_USB &&
		    ms->ms_hangup_cb != NULL) {
			(ms->ms_hangup_cb)(ms->ms_cb_arg);
		}
		break;

	case USB_CDC_STATUS_CONFIG:
		/* User has changed the UART settings on host */
		notification = NETUSB_MUX_NOTIFY_USB_CONFIG;
		break;

	case USB_CDC_STATUS_SEND_BREAK:
		/* User has requested a BREAK character be sent. */
		notification = NETUSB_MUX_NOTIFY_USB_BREAK;
		break;

	default:
		notification = NETUSB_MUX_NOTIFY_NONE;
		break;
	}

	if (notification != NETUSB_MUX_NOTIFY_NONE)
		netusb_mux_notify(ms, notification);
}

static netusb_mux_event_t
netusb_mux_notify_usb_attach(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	/* Not interested in these for now. */
	(void) ms;
	(void) ed;

	return NETUSB_MUX_EVENT_NONE;
}

static netusb_mux_event_t
netusb_mux_notify_usb_detach(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	/* Not interested in these for now. */
	(void) ms;
	(void) ed;

	return NETUSB_MUX_EVENT_NONE;
}

static netusb_mux_event_t
netusb_mux_notify_usb_connect(netusb_mux_state_t ms,
    netusb_mux_event_data_t *ed)
{

	(void) ed;

	/* Ignore if connection established. */
	if (ms->ms_owner != NETUSB_MUX_OWNER_NOBODY)
		return NETUSB_MUX_EVENT_NONE;

	assert(ms->ms_usb != NULL);

	if (usb_cdc_open(ms->ms_usb, ms->ms_usb_cfg, ms->ms_usb_custom_baud,
	    ms->ms_rb_to_host, ms->ms_rb_from_host) < 0) {
		/* Failed to establish a connection. */
		return NETUSB_MUX_EVENT_NONE;
	}

	/* USB now owns the connection. */
	ms->ms_owner = NETUSB_MUX_OWNER_USB;

#ifdef CONFIG_USE_NETWORK
	/* Stop listening on the network port. */
	netusb_mux_restart_network(ms);
#endif

	return NETUSB_MUX_EVENT_CONNECT_USB;
}

static netusb_mux_event_t
netusb_mux_notify_usb_hangup(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	(void) ed;

#ifdef CONFIG_USE_NETWORK
	DBFPRINTF("owner %s, do_telnet %u, wait_telnet %u\n",
	    (ms->ms_owner == NETUSB_MUX_OWNER_NETWORK) ? "net" :
	    ((ms->ms_owner == NETUSB_MUX_OWNER_USB) ? "usb" : "none"),
	    ms->ms_do_telnet, ms->ms_wait_telnet);
#endif

	assert(ms->ms_usb != NULL);

	/* Ignore if not owned by USB */
	if (ms->ms_owner != NETUSB_MUX_OWNER_USB)
		return NETUSB_MUX_EVENT_NONE;

	ms->ms_owner = NETUSB_MUX_OWNER_NOBODY;
	usb_cdc_close(ms->ms_usb);

#ifdef CONFIG_USE_NETWORK
	/* Need to restart the network listener. */
	netusb_mux_restart_network(ms);
#endif

	return NETUSB_MUX_EVENT_DISCONNECT;
}

static netusb_mux_event_t
netusb_mux_notify_usb_config(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	/* Ignore config events if not owned by USB. */
	if (ms->ms_owner != NETUSB_MUX_OWNER_USB)
		return NETUSB_MUX_EVENT_NONE;

	assert(ms->ms_usb != NULL);

	if (ed != NULL) {
		ed->ed_usb_cfg.mc_cfg = usb_cdc_get_config(ms->ms_usb,
		    &ed->ed_usb_cfg.mc_baud);
	}

	return NETUSB_MUX_EVENT_USB_CONFIG;
}

static netusb_mux_event_t
netusb_mux_notify_usb_break(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{

	(void) ed;

	/* Ignore config events if not owned by USB. */
	if (ms->ms_owner != NETUSB_MUX_OWNER_USB)
		return NETUSB_MUX_EVENT_NONE;

	return NETUSB_MUX_EVENT_BREAK;
}
#endif /* CONFIG_USE_USB */

static netusb_mux_event_t
netusb_mux_handle_notifications(netusb_mux_state_t ms,
    netusb_mux_event_data_t *ed)
{
	typedef netusb_mux_event_t (*netusb_mux_notify_handler_t)(
	    netusb_mux_state_t, netusb_mux_event_data_t *);
	static const netusb_mux_notify_handler_t
	    notify_handlers[NETUSB_MUX_NOTIFY__COUNT] = {
		[NETUSB_MUX_NOTIFY_NONE]        = NULL,
#ifdef CONFIG_USE_NETWORK
		[NETUSB_MUX_NOTIFY_NET_HANGUP]  = netusb_mux_notify_net_hangup,
		[NETUSB_MUX_NOTIFY_NET_CONNECT] = netusb_mux_notify_net_connect,
		[NETUSB_MUX_NOTIFY_NET_TELNET]  = netusb_mux_notify_net_telnet,
		[NETUSB_MUX_NOTIFY_NET_BREAK]   = netusb_mux_notify_net_break,
#endif
#ifdef CONFIG_USE_USB
		[NETUSB_MUX_NOTIFY_USB_ATTACH]  = netusb_mux_notify_usb_attach,
		[NETUSB_MUX_NOTIFY_USB_DETACH]  = netusb_mux_notify_usb_detach,
		[NETUSB_MUX_NOTIFY_USB_CONNECT] = netusb_mux_notify_usb_connect,
		[NETUSB_MUX_NOTIFY_USB_HANGUP]  = netusb_mux_notify_usb_hangup,
		[NETUSB_MUX_NOTIFY_USB_CONFIG]  = netusb_mux_notify_usb_config,
		[NETUSB_MUX_NOTIFY_USB_BREAK]   = netusb_mux_notify_usb_break,
#endif
	};

	netusb_mux_notify_t notification;
	netusb_mux_event_t rv;

	while (ms->ms_notify_mask != 0) {
		notification = netusb_mux_get_notification(ms);
		assert(notification < NETUSB_MUX_NOTIFY__COUNT);

		if (notify_handlers[notification] != NULL) {
			rv = (notify_handlers[notification])(ms, ed);
			if (rv != NETUSB_MUX_EVENT_NONE)
				return rv;
		}
	}

	return NETUSB_MUX_EVENT_NONE;
}

netusb_mux_event_t
netusb_mux_poll(netusb_mux_state_t ms, netusb_mux_event_data_t *ed)
{
	netusb_mux_event_t rv;

	rv = netusb_mux_handle_notifications(ms, ed);
	if (rv != NETUSB_MUX_EVENT_NONE)
		return rv;

	if (ms->ms_owner != NETUSB_MUX_OWNER_NOBODY)
		return NETUSB_MUX_EVENT_NONE;

#ifdef CONFIG_USE_NETWORK
	/*
	 * If idle, and we have no listening socket, restart network.
	 */
	if (ms->ms_socket == NULL)
		netusb_mux_restart_network(ms);
#endif

#ifdef CONFIG_USE_USB
	/*
	 * It's not an error for this to fail - a CDC instance may not be
	 * assigned for the requested port. We'll try again later.
	 */
	if (ms->ms_usb_port != NETUSB_MUX_USB_PORT_UNUSED &&
	    ms->ms_usb == NULL) {
		ms->ms_usb = usb_cdc_claim(ms->ms_usb_port,
		    netusb_mux_usb_status_cb, ms);
		if (ms->ms_usb == NULL)
			rtos_timer_start(ms->ms_timer);
	}
#endif

	return NETUSB_MUX_EVENT_NONE;
}

netusb_mux_state_t
netusb_mux_attach(const struct netusb_mux_attach_args *aa)
{
	netusb_mux_state_t ms;

	if ((ms = zone_malloc(sizeof(*ms))) == NULL)
		return NULL;

	DBFPRINTF("attaching %p\n", (void *)ms);

	assert(aa->aa_usb_port == NETUSB_MUX_USB_PORT_UNUSED ||
	    aa->aa_usb_port < PLATFORM_USB_CDC_INSTANCE_NB);

	ms->ms_aa = *aa;
	ms->ms_owner = NETUSB_MUX_OWNER_NOBODY;
	ms->ms_notify_mask = 0;

#ifdef CONFIG_USE_NETWORK
	ms->ms_wait_telnet = false;
	ms->ms_socket = NULL;
#endif
#ifdef CONFIG_USE_USB
	ms->ms_usb = NULL;
	if (ms->ms_usb_cfg == 0)
		ms->ms_usb_cfg = UART_DEFAULT_CFG;
#endif

	ms->ms_timer = rtos_timer_create("netusb_mux", 1053, 0,
	    netusb_mux_tcp_timer_cb, ms);
	assert(ms->ms_timer != NULL);

	return ms;
}

void
netusb_mux_detach(netusb_mux_state_t ms)
{

	DBFPRINTF("closing %p\n", (void *)ms);

#ifdef CONFIG_USE_USB
	if (ms->ms_usb != NULL) {
		usb_cdc_close(ms->ms_usb);
		usb_cdc_unclaim(ms->ms_usb);
		ms->ms_usb = NULL;
	}
#endif

#ifdef CONFIG_USE_NETWORK
	if (ms->ms_socket != NULL) {
		network_close(ms->ms_socket);
		ms->ms_socket = NULL;
	}
#endif

	rtos_timer_stop(ms->ms_timer);
	rtos_timer_delete(ms->ms_timer);
	zone_free(ms);
}

#ifdef CONFIG_USE_NETWORK
void
netusb_mux_tcp_change(netusb_mux_state_t ms, uint16_t new_port)
{
	rtos_saved_ipl_t ipl;

	if (ms->ms_socket != NULL) {
		network_close(ms->ms_socket);
		ms->ms_socket = NULL;
	}

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	ms->ms_notify_mask &= ~NETUSB_MUX_NOTIFY_NET;

	if (ms->ms_owner == NETUSB_MUX_OWNER_NETWORK)
		ms->ms_owner = NETUSB_MUX_OWNER_NOBODY;

	ms->ms_tcp_port = new_port;
	rtos_ipl_restore(ipl);

	rtos_timer_start(ms->ms_timer);
}

void
netusb_mux_tcp_options(netusb_mux_state_t ms, bool do_telnet, bool do_nagle)
{

	ms->ms_do_telnet = do_telnet;

	if (do_nagle != ms->ms_do_nagle) {
		ms->ms_do_nagle = do_nagle;
		/* XXX: locking */
		if (ms->ms_socket != NULL)
			network_tcp_nagle(ms->ms_socket, ms->ms_do_nagle);
	}
}
#endif /* CONFIG_USE_NETWORK */

void
netusb_mux_usb_change(netusb_mux_state_t ms, int8_t new_port)
{
#ifdef CONFIG_USE_USB
	rtos_saved_ipl_t ipl;

	if (ms->ms_usb != NULL) {
		usb_cdc_close(ms->ms_usb);
		if (new_port != ms->ms_usb_port) {
			usb_cdc_unclaim(ms->ms_usb);
			ms->ms_usb = NULL;
		}
	}

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	ms->ms_notify_mask &= ~NETUSB_MUX_NOTIFY_USB;

	if (ms->ms_owner == NETUSB_MUX_OWNER_USB)
		ms->ms_owner = NETUSB_MUX_OWNER_NOBODY;

	ms->ms_usb_port = new_port;
	rtos_ipl_restore(ipl);

	rtos_timer_start(ms->ms_timer);
#else
	(void) ms;
	(void) new_port;
#endif /* CONFIG_USE_USB */
}

void
netusb_mux_close(netusb_mux_state_t ms)
{

#ifdef CONFIG_USE_USB
	netusb_mux_usb_change(ms, ms->ms_usb_port);
#endif
#ifdef CONFIG_USE_NETWORK
	netusb_mux_tcp_change(ms, ms->ms_tcp_port);
#endif
}
