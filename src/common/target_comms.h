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

#ifndef TARGET_COMMS_H
#define TARGET_COMMS_H

#include <stdbool.h>
#include <stdio.h>

#include "ringbuff.h"
#include "port_daemon.h"

typedef enum {
	TARGET_COMMS_PORT_TARGET_CONSOLE = 0,
	TARGET_COMMS_PORT_SWO,
	TARGET_COMMS_PORT_SEMIHOST0,
	TARGET_COMMS_PORT_GDB_SERVER0,
	TARGET_COMMS_PORT_SEMIHOST1,
	TARGET_COMMS_PORT_GDB_SERVER1,
	TARGET_COMMS_PORT_AVR_PROGRAMMER,
	TARGET_COMMS_PORT_WIFI_UART,
	TARGET_COMMS_PORT__COUNT,
	TARGET_COMMS_PORT_NONE,
} target_comms_port_t;

typedef uint16_t target_comms_config_flags_t;

struct target_comms_config {
	target_comms_config_flags_t tcc_flags;
#define	TARGET_COMMS_FLAGS_CAN_SHARE		(1u << 0)
#define	TARGET_COMMS_FLAGS_TELNET		(1u << 1)
#define	TARGET_COMMS_FLAGS_CRLF			(1u << 2)
#define	TARGET_COMMS_FLAGS_MUTABLE_MODE		(1u << 3)
#define	TARGET_COMMS_FLAGS_UART			(1u << 4)
#define	TARGET_COMMS_FLAGS_MUTABLE_CFG		(1u << 5)
#define	TARGET_COMMS_FLAGS_MUTABLE_BAUD		(1u << 6)
#define	TARGET_COMMS_FLAGS_DO_NAGLE		(1u << 7)
#define	TARGET_COMMS_FLAGS_MUTABLE_NAGLE	(1u << 8)
#define	TARGET_COMMS_FLAGS_NAGLE_RESTART	(1u << 9)
#define	TARGET_COMMS_FLAGS_VALID		(1u << 15)
#define	TARGET_COMMS_PORT_VALID(tcc)		\
	(((tcc)->tcc_flags & TARGET_COMMS_FLAGS_VALID) != 0)

	int8_t tcc_usb_port;
#define	TARGET_COMMS_USB_PORT_UNUSED	PORT_DAEMON_USB_PORT_UNUSED

	uint16_t tcc_tcp_port;
#define	TARGET_COMMS_TCP_PORT_UNUSED	PORT_DAEMON_TCP_PORT_UNUSED

	uint16_t tcc_rb_size_to_host;
	uint16_t tcc_rb_size_from_host;

	uint16_t tcc_uart_cfg;
	uint32_t tcc_uart_baud;
	uint32_t tcc_uart_max_baud;
#define	TARGET_COMMS_UART_BAUD_STANDARD	0u
};

typedef struct target_comms_arg {
	void *ca_ops_arg;
	target_comms_config_flags_t ca_flags;	/* Same as tcc_flags, above. */
} *target_comms_arg_t;

/*
 * Convenience macroes for target_comms_config.tcc_flags and
 * target_comms_arg.ca_flags fields.
 */
#define	TC_FLAGS_DO_CRLF(f)	    (((f) & TARGET_COMMS_FLAGS_CRLF) != 0)
#define	TC_FLAGS_DO_TELNET(f)	    (((f) & TARGET_COMMS_FLAGS_TELNET) != 0)
#define	TC_FLAGS_IS_RAW(f)	    (!TC_FLAGS_DO_CRLF(f) && \
				     !TC_FLAGS_DO_TELNET(f))

struct target_comms_source {
	port_daemon_ops_t cs_ops;
	struct target_comms_arg cs_arg;
	target_comms_port_t cs_port;
};

extern void target_comms_init(void);
extern void target_comms_register(struct target_comms_source *);
extern void target_comms_start(void);

extern void target_comms_generic_worker(ringbuff_t from_target,
		ringbuff_t to_host, uint8_t flags);
extern bool target_comms_get_config(target_comms_port_t port,
		struct target_comms_config *tcc);
extern bool target_comms_set_config(target_comms_port_t port,
		const struct target_comms_config *tcc, bool persist);
extern void target_comms_load_defaults(void);

#ifdef PLATFORM_AUTOBAUD
extern uint32_t platform_autobaud(FILE *, target_comms_port_t);
#endif

#ifndef CONFIG_USE_CONFIGDB
#define	TARGET_COMMS_GLOBAL_VARS		\
	struct target_comms_config glob_target_comms[TARGET_COMMS_PORT__COUNT];
#endif

#endif /* TARGET_COMMS_H */
