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

#ifndef PROGRAMMER_H
#define PROGRAMMER_H

#include "rtos.h"
#include "ringbuff.h"
#include "network.h"

#define	PROGRAMMER_STACK_LEN		400

#define	PROGRAMMER_MAX_MESSAGE_LEN	320
#define	PROGRAMMER_RB_2HOST_SIZE	300
#define	PROGRAMMER_RB_2PROG_SIZE	300

struct programmer_protocol;

struct programmer_state {
	rtos_task_t ps_task;

	uint32_t ps_events;
	int ps_owner;
#define	PROGRAMMER_OWNER_NOBODY		0
#define	PROGRAMMER_OWNER_NETWORK	1
#ifdef CONFIG_USE_USB
#define	PROGRAMMER_OWNER_USB_CDC	2
#define	PROGRAMMER_OWNER_USB_VENDOR	3
#endif

	void *ps_socket;
	network_sock_params_t ps_peer;

#ifdef CONFIG_USE_USB
	void *ps_usb;
	uint16_t ps_usb_rx_len;
#endif

	uint32_t ps_rescue_clock;

	const struct programmer_protocol *ps_protocol;

	ringbuff_t ps_prog2host;
	ringbuff_t ps_host2prog;
	ringbuff_blocking_t ps_prog2host_blk;
	ringbuff_blocking_t ps_host2prog_blk;

	void *ps_isp;
	void *ps_pdi;
	void *ps_updi;

	void *ps_proto_state;

	uint32_t ps_request_len;
	uint32_t ps_request_cursor;
	uint32_t ps_reply_len;

	uint8_t ps_request[PROGRAMMER_MAX_MESSAGE_LEN] __attribute__((aligned(16)));
	uint8_t ps_reply[PROGRAMMER_MAX_MESSAGE_LEN] __attribute__((aligned(16)));

	uint8_t ps_rb_prog2host[PROGRAMMER_RB_2HOST_SIZE];
	uint8_t ps_rb_host2prog[PROGRAMMER_RB_2PROG_SIZE];
};

struct programmer_protocol {
	uint32_t pp_header_len;
	uint32_t (*pp_got_header)(struct programmer_state *);
	void (*pp_got_body)(struct programmer_state *);
	void (*pp_got_frame)(struct programmer_state *);
	void (*pp_closed)(struct programmer_state *);
};

#if 0
extern uint32_t programmer_curr_clock_rate;
extern uint32_t programmer_curr_clock_ticks;
extern const char *programmer_curr_clock_name;
#endif

extern void programmer_init(void);
extern uint8_t programmer_get_proto(void);
extern void programmer_load_defaults(void);
extern void programmer_task(rtos_task_t);
extern void programmer_status_tick(void);
extern const char *programmer_status(uint32_t *);

extern uint32_t programmer_request_get_byte(struct programmer_state *,
			uint8_t *);
extern uint32_t programmer_request_get_buff(struct programmer_state *, void *,
			uint32_t);
extern uint32_t programmer_reply_add_byte(struct programmer_state *, uint8_t);
extern uint32_t programmer_reply_add_buff(struct programmer_state *,
			const void *,uint32_t);

extern uint32_t programmer_curr_clock_rate;
extern uint32_t programmer_curr_clock_ticks;
extern const char *programmer_curr_clock_name;

#define	PROGRAMMER_DEV_ID_SIZE	16
#ifndef CONFIG_USE_CONFIGDB
#define	PROGRAMMER_DEV_ID	glob.glob_programmer_dev_id
#else
extern char programmer_dev_id[PROGRAMMER_DEV_ID_SIZE];
#define	PROGRAMMER_DEV_ID	programmer_dev_id
#endif

/* Values for glob.glob_programmer_proto */
#define	PROG_PROTO_STK500V2	0
#define	PROG_PROTO_DRAGON	1
#define	PROG_PROTO_WAVR		2

#include "proto_jtag2.h"
#include "proto_stk500v2.h"
#include "prog_isp.h"
#include "prog_pdi.h"
#include "prog_updi.h"

#ifndef CONFIG_USE_CONFIGDB
#define	PROGRAMMER_GLOBAL_VARS		\
	char glob_programmer_dev_id[PROGRAMMER_DEV_ID_SIZE];\
	uint8_t glob_programmer_proto;	\
	PROTO_JTAG2_GLOBAL_VARS		\
	PROGO_STK500_GLOBAL_VARS	\
	PROG_ISP_GLOBAL_VARS		\
	PROG_PDI_GLOBAL_VARS		\
	PROG_UPDI_GLOBAL_VARS
#endif

#endif /* PROGRAMMER_H */
