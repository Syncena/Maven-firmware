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

#ifndef NETUSB_MUX_H
#define NETUSB_MUX_H

#include <stdbool.h>
#include <stdint.h>

#include "rtos.h"
#include "ringbuff.h"
#include "usb_cdc.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#endif

typedef enum {
	NETUSB_MUX_EVENT_NONE = 0,
	NETUSB_MUX_EVENT_DISCONNECT,
	NETUSB_MUX_EVENT_CONNECT_NET,
	NETUSB_MUX_EVENT_CONNECT_USB,
	NETUSB_MUX_EVENT_USB_CONFIG,
	NETUSB_MUX_EVENT_BREAK,
} netusb_mux_event_t;

struct netusb_mux_usb_config {
	uint16_t mc_cfg;
	uint32_t mc_baud;
};

typedef union {
#ifdef CONFIG_USE_NETWORK
	network_sock_params_t ed_tcp_peer;
#endif
	struct netusb_mux_usb_config ed_usb_cfg;
} netusb_mux_event_data_t;

struct netusb_mux_attach_args {
	void (*aa_poll_cb)(void *);
	void (*aa_hangup_cb)(void *);
	void *aa_cb_arg;

	/* Shared params. */
	ringbuff_t aa_rb_to_host;
	ringbuff_t aa_rb_from_host;

#ifdef CONFIG_USE_NETWORK
	/* Network params. */
	uint16_t aa_tcp_port;
	bool aa_do_telnet;
	bool aa_do_nagle;
#endif

	/* USB params. */
	int8_t aa_usb_port;
	uint16_t aa_usb_cfg;
	uint32_t aa_usb_custom_baud;
};
#define	NETUSB_MUX_TCP_PORT_UNUSED	((uint16_t)0u)
#define	NETUSB_MUX_USB_PORT_UNUSED	((int8_t)-1)

struct netusb_mux_state;
typedef struct netusb_mux_state *netusb_mux_state_t;

extern netusb_mux_state_t netusb_mux_attach(const struct netusb_mux_attach_args*);
extern void netusb_mux_detach(netusb_mux_state_t);

extern netusb_mux_event_t netusb_mux_poll(netusb_mux_state_t,
				netusb_mux_event_data_t *);

extern void netusb_mux_close(netusb_mux_state_t);

extern void netusb_mux_tcp_change(netusb_mux_state_t, uint16_t);
extern void netusb_mux_tcp_options(netusb_mux_state_t, bool, bool);
extern void netusb_mux_usb_change(netusb_mux_state_t, int8_t);

#endif /* NETUSB_MUX_H */
