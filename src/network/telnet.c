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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "telnet.h"

#define	IAC	255u
#define	DONT	254u
#define	DO	253u
#define	WONT	252u
#define	WILL	251u
#define	BREAK	243u
#define	NOP	241u

#define	TELOPT_ECHO	1
#define	TELOPT_SGA	3
#define	TELOPT_LINEMODE	34

static __always_inline void
telnet_opt(ringbuff_t txb, uint8_t cmd, uint8_t opt)
{

	ringbuff_produce(txb, IAC);
	ringbuff_produce(txb, cmd);
	ringbuff_produce(txb, opt);
}

void
telnet_init(struct telnet_state *ts, ringbuff_t txb)
{

	ts->ts_cmd = 0;
	ts->ts_skip_count = 0;
	telnet_opt(txb, DO, TELOPT_SGA);
	telnet_opt(txb, WILL, TELOPT_SGA);
	telnet_opt(txb, WILL, TELOPT_ECHO);
	telnet_opt(txb, DONT, TELOPT_LINEMODE);
}

static uint16_t
telnet_handle_cmd(struct telnet_state *ts, uint8_t cmd)
{
	uint16_t rv = 0;

	ts->ts_cmd = 0;

	switch (cmd) {
	case DONT:
	case DO:
	case WONT:
	case WILL:
		ts->ts_skip_count = 1;
		break;

	case BREAK:
		rv = TELNET_SEND_BREAK;
		break;

	default:
		break;
	}

	return rv;
}

uint16_t
telnet_input(struct telnet_state *ts, uint8_t ch)
{
	uint16_t rv = 0;

	if (ts->ts_cmd)
		rv = telnet_handle_cmd(ts, ch);
	else
	if (ts->ts_skip_count)
		ts->ts_skip_count -= 1;
	else
	if (ch == IAC)
		ts->ts_cmd = 1;
	else
	if (ch != '\n' && ch != '\0')
		rv = TELNET_VALID_DATA | ch;

	return rv;
}

void
telnet_output(struct telnet_state *ts, ringbuff_t txb, uint8_t ch)
{
	uint32_t timeout;

	(void) ts;

	/* XXX: We should try harder here */
	if (ch == IAC)
		return;

	timeout = 500;
	while (ringbuff_is_full(txb) && timeout) {
		rtos_task_sleep(2);
		timeout--;
	}

	if (timeout) {
		ringbuff_produce(txb, ch);
		if (ch == '\n')
			ringbuff_produce_done(txb);
	}
}
