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

#ifndef PORT_DAEMON_H
#define PORT_DAEMON_H

#include <stdbool.h>

#include "ringbuff.h"
#include "usb_cdc.h"
#include "netusb_mux.h"

typedef enum {
	PORT_DAEMON_CTRL_USB_CONFIG,	/* arg -> netusb_mux_event_data_t */
	PORT_DAEMON_CTRL_UART_CONFIG,	/* arg -> netusb_mux_event_data_t */
	PORT_DAEMON_CTRL_BREAK,		/* arg -> NULL */
} port_daemon_ctrl_t;

struct port_daemon_instance;
typedef struct port_daemon_instance *port_daemon_instance_t;

typedef bool (*port_daemon_op_connect_t)(void *, port_daemon_instance_t,
		ringbuff_t to_host, ringbuff_t from_host, unsigned int);
typedef void (*port_daemon_op_disconnect_t)(void *);
typedef void (*port_daemon_op_worker_t)(void *);
typedef int (*port_daemon_op_ctrl_t)(void *, port_daemon_ctrl_t, const void *);
typedef void (*port_daemon_op_hangup_t)(void *);

typedef struct port_daemon_ops {
	port_daemon_op_connect_t op_connect;
	port_daemon_op_disconnect_t op_disconnect;
	port_daemon_op_worker_t op_worker;
	port_daemon_op_ctrl_t op_ctrl;
	port_daemon_op_hangup_t op_hangup;
} const *port_daemon_ops_t;

typedef struct port_daemon_settings {
	uint8_t s_flags;
#define	PORT_DAEMON_SOURCE_FLAG_DO_TELNET	(1u << 0)
#define	PORT_DAEMON_SOURCE_FLAG_DO_NAGLE	(1u << 1)

	int8_t s_usb_port;
#define	PORT_DAEMON_USB_PORT_UNUSED		NETUSB_MUX_USB_PORT_UNUSED

	uint16_t s_tcp_port;
#define	PORT_DAEMON_TCP_PORT_UNUSED		NETUSB_MUX_TCP_PORT_UNUSED

	uint16_t s_rb_size_to_host;
	uint16_t s_rb_size_from_host;
} const *port_daemon_settings_t;


extern void port_daemon_init(void);
extern port_daemon_instance_t port_daemon_register_source(
		const port_daemon_settings_t, const port_daemon_ops_t, void *,
		const char * const *options, unsigned int num_options);
extern void port_daemon_attention(port_daemon_instance_t);
extern void port_daemon_change_flags(port_daemon_instance_t, uint8_t);

#endif /* PORT_DAEMON_H */
