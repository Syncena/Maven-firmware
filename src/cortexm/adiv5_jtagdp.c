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
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "adiv5.h"
#include "adiv5regs.h"
#include "adiv5_ll.h"
#include "adiv5_swdp.h"
#include "jtag_if.h"
#include "timer.h"
#include "tmon.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

struct jtagdp_state {
	jtag_if_t js_if;
	jtag_state_t js_cookie;
	int js_device;
	uint32_t js_ir;
};
#define	JTAG_IF_DONE(js)	(js)->js_if->ji_done((js)->js_cookie)
#define	JTAG_IF_RESET(js)	(js)->js_if->ji_reset((js)->js_cookie)
#define	JTAG_IF_PROBE(js)	(js)->js_if->ji_probe((js)->js_cookie)
#define	JTAG_IF_IDCODE(js,d)	(js)->js_if->ji_get_idcode((js)->js_cookie,d)
#define	JTAG_IF_SELECT(js,d)	(js)->js_if->ji_select_device((js)->js_cookie,d)
#define	JTAG_IF_WRITE_IR(js,ir)	(js)->js_if->ji_write_ir((js)->js_cookie,(ir))
#define	JTAG_IF_RDWR_DR(js,wr,rd,bits)					       \
	do {								       \
		(js)->js_if->ji_rdwr_dr((js)->js_cookie,(wr),(rd),(bits));     \
	} while (/*CONSTCOND*/0)

static int
adiv5_jtagdp_init(adiv5_dp_t dp, uint32_t cfg)
{
	static struct jtagdp_state db_jtag_state;
	struct jtagdp_state *js = &db_jtag_state;
	unsigned int speed, idles;
	int devs;

	adiv5_dp_get_speed_idles(&speed, &idles);

	/* Bring up the platform's JTAG interface. */
	js->js_cookie = platform_jtag_init(&js->js_if, speed, idles);
	if (js->js_cookie == NULL)
		return 0;

	dp->dp_cfg = adiv5_speed_2cfg[speed] << ADIV5_LL_CLKDELAY_SHIFT;
	dp->dp_cfg |= cfg;
	ADIV5_DP_FORCE_SELECT_SET(dp);	/* XXX: Make this optional. */

	/* Target may need to switch from SWD to JTAG. */
	adiv5_swdp_switch_to_jtag(dp->dp_cfg | ADIV5_LL_SWITCH_TO_JTAG);

	/* Anything out there? */
	if ((devs = JTAG_IF_PROBE(js)) != 0) {
		DBFPRINTF("Found JTAG scan chain with %d devices.\n", devs);
		tmon_log("Found %d TAPs on JTAG scan chain\n", devs);
		for (int d = 0; d < devs; d++) {
			tmon_log("TAP %d, IDCODE %08" PRIx32 "\n", d,
			    JTAG_IF_IDCODE(js, d));
		}
		for (int d = 0; d < devs; d++) {
			uint32_t idcode = JTAG_IF_IDCODE(js, d);

			DBFPRINTF("IDCODE#%d: %08" PRIx32 "\n", d, idcode);

			/* Looking for an ARM AVIv5 JTAG-DP Port. */
			if ((idcode & 0x0fff0fffu) == 0x0ba00477u) {
				/*
				 * XXX: Support multiple JTAG-DP TAPs.
				 */
				DBPRINTF("Found JTAG-DP.\n");
				tmon_log("Found a JTAG-DP at TAP %d\n", d);
				JTAG_IF_SELECT(js, d);
				js->js_device = d;
				dp->dp_idr = idcode;
				dp->dp_if_cookie = js;
				return 1;
			}
		}
	}

	JTAG_IF_DONE(js);

	return 0;
}

static void
adiv5_jtagdp_done(adiv5_dp_t dp)
{
	struct jtagdp_state *js = dp->dp_if_cookie;

	JTAG_IF_DONE(js);
}

static uint32_t
adiv5_jtagdp_xfer(adiv5_dp_t dp, uint32_t req, uint32_t *data)
{
	struct jtagdp_state *js = dp->dp_if_cookie;
	uint32_t ack, d, wrbits[2], rdbits[2];
	timer_timeout_t to;

	d = (data != NULL) ? *data : 0;

	/* Construct the 35-bits of data to write to DR. */
	JTAG_DP_CONSTRUCT_DR(wrbits, req, d);

	/* Write DPACC or APACC to IR according to req & DP_REQ_APnDP */
	JTAG_IF_WRITE_IR(js, JTAG_DP_IR_DPACC | (req & DP_REQ_APnDP));

	timer_timeout_start(&to, 2000);

	do {
		JTAG_IF_RDWR_DR(js, wrbits, rdbits, 35);
		JTAG_DP_DECONSTRUCT_DR(rdbits, ack, d);
	} while (ack == DP_ACK_WAIT && !timer_timeout_expired(&to));

	if (ack == DP_ACK_OK && data != NULL)
		*data = d;

	dp->dp_last_ack = ack;
	return ack;
}

static void
adiv5_jtagdp_abort(adiv5_dp_t dp, uint32_t v)
{
	struct jtagdp_state *js = dp->dp_if_cookie;
	uint32_t wrbits[2], rdbits[2];

	/* Construct a DP ABORT request. */
	JTAG_DP_CONSTRUCT_DR(wrbits, 0, v);

	JTAG_IF_WRITE_IR(js, JTAG_DP_IR_ABORT);
	JTAG_IF_RDWR_DR(js, wrbits, rdbits, 35);
}

const struct adiv5_dp_interface adiv5_jtagdp_if = {
	.dpi_name = "JTAG-DP",
	.dpi_type = ADIV5_TYPE_JTAGDP,
	.dpi_init = adiv5_jtagdp_init,
	.dpi_done = adiv5_jtagdp_done,
	.dpi_xfer = adiv5_jtagdp_xfer,
	.dpi_line_reset = NULL,
	.dpi_abort = adiv5_jtagdp_abort,
};
