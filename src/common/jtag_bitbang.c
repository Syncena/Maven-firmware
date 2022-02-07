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
#include <stdint.h>
#include <stdlib.h>

#include "rtos.h"
#include "timer.h"
#include "hardware.h"
#include "jtag_if.h"
#include "jtag_bitbang.h"
#include "platform.h"

/*
 * Declare pins.
 */
HW_PIN(TDO, EXT1)
HW_PIN(TMS, EXT3)
HW_PIN(TCK, EXT4)
HW_PIN(TDI, EXT8)

struct jtag_hw_state {
	uint32_t hs_del;
};

static void
jtag_bitbang_done(jtag_hw_state_t hs)
{
	rtos_saved_ipl_t ipl;

	assert(hs != NULL);
	(void)hs;

	/* Atomic to minimise pin float time. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HW_PIN_TCK_DIR_clr();
	HW_PIN_TCK_in();
	HW_PIN_TMS_DIR_clr();
	HW_PIN_TMS_in();
	HW_PIN_TDI_DIR_clr();
	HW_PIN_TDI_in();
	rtos_ipl_restore(ipl);

	HW_PIN_TDO_DIR_clr();
	HW_PIN_TDO_in();
}

static FAST_RAMFUNC void
jtag_bitbang_clock_tms(jtag_hw_state_t hs, unsigned int bits, uint32_t tms);
static void
jtag_bitbang_clock_tms(jtag_hw_state_t hs, unsigned int bits, uint32_t tms)
{
	uint32_t del, mask;

	assert(bits > 0 && bits <= 32);

	mask = 1;
	del = hs->hs_del;

	while (bits--) {
		if ((tms & mask) != 0)
			HW_PIN_TMS_set();
		else
			HW_PIN_TMS_clr();

		if (del == 0) {
			HW_PIN_TCK_set();
			__COMPILER_BARRIER();
			(void) HW_PIN_TDO_read();
			HW_PIN_TCK_clr();
			__COMPILER_BARRIER();
		} else {
			HW_PIN_TCK_set();
			timer_delay_cycles(del);
			(void) HW_PIN_TDO_read();
			HW_PIN_TCK_clr();
			timer_delay_cycles(del);
		}

		mask <<= 1;
	}
}

static FAST_RAMFUNC uint32_t
jtag_bitbang_clock_tdo_tdi(jtag_hw_state_t hs, unsigned int bits, uint32_t tx,
    bool last);
static uint32_t
jtag_bitbang_clock_tdo_tdi(jtag_hw_state_t hs, unsigned int bits, uint32_t tx,
    bool last)
{
	uint32_t del, mask, rx;
	bool tdo;

	assert(bits > 0 && bits <= 32);
	assert(HW_PIN_TMS_state() == false);

	rx = 0;
	mask = 1;
	del = hs->hs_del;

	while (bits--) {
		if (bits == 0 && last)
			HW_PIN_TMS_set();

		if ((tx & mask) != 0)
			HW_PIN_TDI_set();
		else
			HW_PIN_TDI_clr();

		if (del == 0) {
			HW_PIN_TCK_set();
			__COMPILER_BARRIER();
			tdo = HW_PIN_TDO_read() != 0;
			HW_PIN_TCK_clr();
			__COMPILER_BARRIER();
		} else {
			HW_PIN_TCK_set();
			timer_delay_cycles(del);
			tdo = HW_PIN_TDO_read() != 0;
			HW_PIN_TCK_clr();
			timer_delay_cycles(del);
		}

		if (tdo)
			rx |= mask;

		mask <<= 1;
	}

	return rx;
}

jtag_state_t
jtag_bitbang_init(jtag_if_t *jif, unsigned int speed, unsigned int idle_clocks)
{
	static const struct jtag_hw_if jtag_bitbang_hw_if = {
		.hi_done = jtag_bitbang_done,
		.hi_reset = NULL,
		.hi_clock_tms = jtag_bitbang_clock_tms,
		.hi_clock_tdo_tdi = jtag_bitbang_clock_tdo_tdi,
	};
	static struct jtag_hw_state jtag_bitbang_state;
	struct jtag_hw_state *hs = &jtag_bitbang_state;
	rtos_saved_ipl_t ipl;

	assert(speed <= 9);

	/*
	 * - TCK, TMS, TDI to output mode and set to low level.
	 * - TDO to input mode.
	 */
	/* Atomic to minimise contention time. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HW_PIN_TCK_clr();
	HW_PIN_TCK_out();
	HW_PIN_TCK_DIR_set();
	HW_PIN_TMS_clr();
	HW_PIN_TMS_out();
	HW_PIN_TMS_DIR_set();
	HW_PIN_TDI_clr();
	HW_PIN_TDI_out();
	HW_PIN_TDI_DIR_set();
	rtos_ipl_restore(ipl);

	HW_PIN_TDO_in();
	HW_PIN_TDO_DIR_clr();

	if (speed > 8)
		speed = 9;
	hs->hs_del = (8 - speed) * 10;

	return jtag_init(&jtag_bitbang_hw_if, hs, jif, idle_clocks);
}
