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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "jtag_if.h"
#include "platform.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

#define	JTAG_MAX_DEVICES	32
#define	JTAG_MAX_IRLEN		32
#define	JTAG_IR_IDCODE		0xfffffffeu
#define	JTAG_IR_BYPASS		0xffffffffu

struct jtag_state {
	jtag_hw_if_t js_hw_if;
	jtag_hw_state_t js_hw_state;
	int js_ndevs;
	int js_current;
	uint32_t js_current_ir;
	unsigned int js_current_ir_bits;
	unsigned int js_current_ir_before;
	unsigned int js_current_ir_after;
	uint16_t js_ir_bits[JTAG_MAX_DEVICES];
	bool js_idle;
	unsigned int js_idle_clocks;
};

#define	JTAG_HW_DONE(js)						       \
	do {								       \
		((js)->js_hw_if->hi_done)((js)->js_hw_state);		       \
	} while (/*CONSTCOND*/0)
#define	JTAG_HW_RESET(js)						       \
	do {								       \
		if ((js)->js_hw_if->hi_reset != NULL)			       \
			((js)->js_hw_if->hi_reset)((js)->js_hw_state);	       \
	} while (/*CONSTCOND*/0)
#define	JTAG_HW_CLK_TMS(js,bits,tms)					       \
	do {								       \
		((js)->js_hw_if->hi_clock_tms)((js)->js_hw_state, (bits),      \
		    (tms));						       \
	} while (/*CONSTCOND*/0)
#define	JTAG_HW_CLK_TDO_TDI(js,bits,tdi,last)				       \
		((js)->js_hw_if->hi_clock_tdo_tdi)((js)->js_hw_state, (bits),  \
		    (tdi),(last))

static void
jtag_done(jtag_state_t js)
{

	DBFPRINTF("Finished.\n");

	JTAG_HW_DONE(js);

	memset(js, 0, sizeof(*js));
}

static void
jtag_reset(jtag_state_t js)
{

	/* Toggle TRST, if available. */
	DBFPRINTF("Toggle TRST.\n");
	JTAG_HW_RESET(js);

	/* Go to Run-Test/Idle via Test-Logic-Reset */
	DBFPRINTF("Test-Logic-Reset -> Run-Test/Idle.\n");
	JTAG_HW_CLK_TMS(js, 6, 0x1fu);

	js->js_idle = true;
	js->js_current = -1;
	js->js_current_ir = JTAG_IR_BYPASS;
}

static __inline void
jtag_idle_2_shift_ir(jtag_state_t js)
{

	assert(js->js_idle);

	/* Goto Shift-IR from Run-Test/Idle. */
	DBFPRINTF("Run-Test/Idle -> Shift-IR.\n");
	JTAG_HW_CLK_TMS(js, 4, 0x3u);

	js->js_idle = false;
}

static __inline void
jtag_idle_2_shift_dr(jtag_state_t js)
{

	assert(js->js_idle);

	/* Goto Shift-DR from Run-Test/Idle. */
	DBFPRINTF("Run-Test/Idle -> Shift-DR.\n");
	JTAG_HW_CLK_TMS(js, 3, 0x1u);

	js->js_idle = false;
}

static __inline void
jtag_exit1_2_idle(jtag_state_t js)
{

	assert(js->js_idle == false);

	/* Starting at Exit1-xR, goto Run-Test/Idle. */
	DBFPRINTF("Exit1-xR -> Run-Test/Idle.\n");
	JTAG_HW_CLK_TMS(js, 2, 0x1u);

	/*
	 * Continue to toggle TCK if required, remaining in Run-Test/Idle.
	 * Note: This is a requirement imposed by ARM ADIv5 TAPs,
	 * and may not necessarily apply to TAPs from other vendors.
	 */
	if (js->js_idle_clocks) {
		DBFPRINTF("Inserting %u idle clock%s.\n", js->js_idle_clocks,
		    (js->js_idle_clocks == 0) ? "" : "s");
		JTAG_HW_CLK_TMS(js, js->js_idle_clocks, 0);
	}

	js->js_idle = true;
}

static int
jtag_probe(jtag_state_t js)
{
	unsigned int bits_set, bits_clr;
	int i, ndevs;
	uint32_t rx;

	DBFPRINTF("Probing JTAG\n");

	js->js_ndevs = 0;

	jtag_reset(js);

	/*
	 * All devices should now have their IDCODE in DR.
	 * Transition to Shift-DR.
	 */
	jtag_idle_2_shift_dr(js);

	/*
	 * Feed a continuous string of TDO=1 into DR, while we read 32-bits
	 * at a time. Eventually we should see 0xffffffff, indicating our
	 * TDO=1 bits have been clocked out.
	 */
	DBFPRINTF("Shifting reset IDCODEs\n");
	bits_set = bits_clr = 0;
	for (i = 0, rx = 0; i < JTAG_MAX_DEVICES && rx != 0xffffffffu; i++) {
		rx = JTAG_HW_CLK_TDO_TDI(js, 32, 0xffffffffu, false);

		/* Count set/clear bits. This will indicate if TDO is stuck. */
		for (uint32_t mask = 1; mask; mask <<= 1) {
			if ((rx & mask) == 0)
				bits_clr++;
			else
				bits_set++;
		}

		DBFPRINTF("TAP %d: IDCODE %08" PRIx32 "\n", i, rx);
	}

	DBFPRINTF("IDCODEs shifted. Count %u, bits_set %u, bits_clr %u\n",
	    i, bits_set, bits_clr);

	/* Back to Run-Test/Idle, via Test-Logic-Reset. */
	jtag_reset(js);

	/* Anything of interest found? */
	if (i == 0 || bits_clr == 0 || bits_set == 0) {
		DBFPRINTF("Bad scan chain.\n");
		return 0;
	}
	ndevs = i - 1;

	/*
	 * Transition to Shift-IR.
	 * As we move to this state, all sensible devices will preload
	 * their IR shift register with 1. As we clock out these single
	 * bits, we count the intervening 0 bits to determine the width
	 * of each device's IR. Two consecutive 1 bits indicate we reached
	 * the end of the chain; the two 1s were fed in by us at the start.
	 */
	jtag_idle_2_shift_ir(js);

	DBFPRINTF("Shifting reset IRs\n");

	/* Clock the first bit out. It ought to be a 1. */
	if (JTAG_HW_CLK_TDO_TDI(js, 1, 1, false) == 0) {
		/* First bit was zero. This shouldn't happen. */
		DBFPRINTF("First bit zero. Abort.\n");
		jtag_reset(js);
		return 0;
	}

	for (i = 0; i < JTAG_MAX_DEVICES; i++) {
		uint32_t ir_bits;

		for (ir_bits = 1; ir_bits <= JTAG_MAX_IRLEN; ir_bits++) {
			/* Found the next 1 bit? */
			if (JTAG_HW_CLK_TDO_TDI(js, 1, 1, false) == 1)
				break;
		}

		if (ir_bits == 1) {
			/*
			 * This implies we found two consecutive 1 bits,
			 * indicating we've shifted the entire chain.
			 */
			DBFPRINTF("Two consecutive TDO=1\n");
			break;
		}

		/*
		 * We've now identified the number of bits in this
		 * device's instruction register.
		 */
		js->js_ir_bits[i] = ir_bits;
		DBFPRINTF("TAP %d: IR bits %u\n", i, ir_bits);
	}

	/* Back to Run-Test/Idle, via Test-Logic-Reset. */
	jtag_reset(js);

	if (i != ndevs) {
		DBFPRINTF("Saw %d IDCODEs, but saw %d reset IRs.\n",
		    ndevs, i);
		return 0;
	}

	js->js_ndevs = i;
	DBFPRINTF("Done. Found %d devices.\n", js->js_ndevs);

	return js->js_ndevs;
}

static void
jtag_select_device(jtag_state_t js, int dev)
{
	unsigned int ir_bits;

	DBFPRINTF("Select device %d\n", dev);

	assert(js->js_ndevs > 0);
	assert(dev >= 0 && dev < js->js_ndevs);

	if (js->js_current == dev)
		return;

	js->js_current = dev;
	js->js_current_ir_bits = js->js_ir_bits[dev];
	js->js_current_ir = JTAG_IR_BYPASS;

	ir_bits = 0;
	for (int i = 0; i < dev; i++)
		ir_bits += js->js_ir_bits[i];
	js->js_current_ir_before = ir_bits;

	ir_bits = 0;
	for (int i = dev + 1; i < js->js_ndevs; i++)
		ir_bits += js->js_ir_bits[i];
	js->js_current_ir_after = ir_bits;

	DBFPRINTF("IR bits %u, before %u, after %u\n", js->js_ir_bits[dev],
	    js->js_current_ir_before, js->js_current_ir_after);
}

static FAST_RAMFUNC void
jtag_write_ir(jtag_state_t js, uint32_t ir);
static void
jtag_write_ir(jtag_state_t js, uint32_t ir)
{
	unsigned int bits, this_bits;

	assert(js->js_current >= 0);

	DBFPRINTF("Device %d, IR 0x%" PRIx32 "\n", js->js_current, ir);

	if (js->js_current_ir == ir) {
		DBFPRINTF("IR unchanged.\n");
		return;
	}
	js->js_current_ir = ir;

	jtag_idle_2_shift_ir(js);

	/* Put all preceding TAPS into BYPASS mode. */
	bits = js->js_current_ir_before;
	while (bits) {
		this_bits = (bits <= 32) ? bits : 32;
		bits -= this_bits;
		JTAG_HW_CLK_TDO_TDI(js, this_bits, JTAG_IR_BYPASS, false);
	}

	/* Send IR to the target device. */
	bits = js->js_current_ir_bits;
	assert(bits > 0 && bits <= 32);
	JTAG_HW_CLK_TDO_TDI(js, bits, ir, js->js_current_ir_after == 0);

	/* Put all following TAPS into BYPASS mode. */
	bits = js->js_current_ir_after;
	while (bits) {
		this_bits = (bits <= 32) ? bits : 32;
		bits -= this_bits;
		JTAG_HW_CLK_TDO_TDI(js, this_bits, JTAG_IR_BYPASS, bits == 0);
	}

	jtag_exit1_2_idle(js);

	DBFPRINTF("Done.\n");
}

static FAST_RAMFUNC void
jtag_rdwr_dr(jtag_state_t js, const uint32_t *wr, uint32_t *rd,
    unsigned int nbits);
static void
jtag_rdwr_dr(jtag_state_t js, const uint32_t *wr, uint32_t *rd,
    unsigned int nbits)
{
	unsigned int bits, this_bits;
	bool last_dev;

	assert(js->js_current >= 0);

	last_dev = (js->js_current + 1) == js->js_ndevs;

	jtag_idle_2_shift_dr(js);

	/* Send one TDO=0 for each preceding device. */
	bits = (unsigned int)js->js_current;
	while (bits) {
		this_bits = (bits <= 32) ? bits : 32;
		bits -= this_bits;
		JTAG_HW_CLK_TDO_TDI(js, this_bits, 0, false);
	}

	/* Shift in/out the supplied data to the target device. */
	while (nbits) {
		bool last;

		if (nbits <= 32) {
			this_bits = nbits;
			last = last_dev;
		} else {
			this_bits = 32;
			last = false;
		}
		nbits -= this_bits;

		*rd++ = JTAG_HW_CLK_TDO_TDI(js, this_bits, *wr++, last);
	}

	/* Send one TDO=0 for each following device. */
	if (!last_dev) {
		bits = (unsigned int)js->js_ndevs - js->js_current;
		bits--;
		while (bits) {
			this_bits = (bits <= 32) ? bits : 32;
			bits -= this_bits;
			JTAG_HW_CLK_TDO_TDI(js, this_bits, 0, bits == 0);
		}
	}

	jtag_exit1_2_idle(js);
}

static uint32_t
jtag_get_idcode(jtag_state_t js, int dev)
{
	uint32_t idcode;

	assert(js->js_ndevs);
	assert(dev < js->js_ndevs);

	DBFPRINTF("Fetch IDCODE for device %d\n", dev);

	jtag_select_device(js, dev);
	jtag_write_ir(js, JTAG_IR_IDCODE);
	idcode = 0;
	jtag_rdwr_dr(js, &idcode, &idcode, 32);

	DBFPRINTF("Read IDCODE 0x%08" PRIx32 "\n", idcode);

	return idcode;
}

jtag_state_t
jtag_init(const jtag_hw_if_t hw_if, jtag_hw_state_t hw_state,
    const struct jtag_if **jif, unsigned int idle_clocks)
{
	static const struct jtag_if jtag_if = {
		.ji_done = jtag_done,
		.ji_reset = jtag_reset,
		.ji_probe = jtag_probe,
		.ji_get_idcode = jtag_get_idcode,
		.ji_select_device = jtag_select_device,
		.ji_write_ir = jtag_write_ir,
		.ji_rdwr_dr = jtag_rdwr_dr,
	};
	static struct jtag_state jtag_state;

	assert(idle_clocks <= 32);

	memset(&jtag_state, 0, sizeof(jtag_state));
	jtag_state.js_hw_if = hw_if;
	jtag_state.js_hw_state = hw_state;
	jtag_state.js_idle = false;
	jtag_state.js_ndevs = 0;
	jtag_state.js_current = -1;
	jtag_state.js_idle_clocks = idle_clocks;
	*jif = &jtag_if;

	return &jtag_state;
}
