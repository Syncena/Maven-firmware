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
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "hardware.h"
#include "adiv5.h"
#include "adiv5_ll.h"
#include "adiv5_swdp.h"
#include "adiv5regs.h"
#include "tmon.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "ringbuff.h"
#include "platform.h"
#include "uart.h"
#include "target_comms.h"
#include "timer.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * Declare pins.
 */
HW_PIN(SWO, EXT1)
HW_PIN(SWDIO, EXT3)
HW_PIN(SWCLK, EXT4)

#if (!defined(__OPTIMIZE__) || (__OPTIMIZE__ == 0))
#define	ALWAYS_INLINE	/* nothing */
#else
#define	ALWAYS_INLINE	__always_inline
#endif

#pragma GCC push_options
/*
 * The original 'wAVR' board (SAM4S4-based) is memory-challenged, so always
 * optimise for size. Otherwise, warp factor 3 for best performance.
 */
#ifdef WAVR_BOARD
#pragma GCC optimize ("Os")
#else
#pragma GCC optimize ("O3")
#endif
static ALWAYS_INLINE void
add_nops(unsigned int count)
{
	(void) count;

#if 0
	/* This was necessary at some point in the past. Not so much now. */
	while (count--)
		__asm __volatile("nop\n");
#endif
}

static ALWAYS_INLINE void
swdp_swclk_toggle(uint32_t del)
{

	HW_PIN_SWCLK_clr();
	timer_delay_cycles(del);
	HW_PIN_SWCLK_set();
	timer_delay_cycles(del);
}

static ALWAYS_INLINE void
swdp_swclk_toggle_zero(void)
{

	HW_PIN_SWCLK_clr();
	add_nops(3);
	HW_PIN_SWCLK_set();
	__COMPILER_BARRIER();
}

static ALWAYS_INLINE void
adiv5_swdio_mode_in(void)
{

	__disable_irq();
	/* External buffer to input */
	HW_PIN_SWDIO_DIR_clr();
	/* Our GPIO pin to input */
	HW_PIN_SWDIO_in();
	__COMPILER_BARRIER();
	__enable_irq();
}

static ALWAYS_INLINE void
adiv5_swdio_mode_out(void)
{

	__disable_irq();
	/* Our GPIO pin to output */
	HW_PIN_SWDIO_out();
	/* External buffer to output */
	HW_PIN_SWDIO_DIR_set();
	__COMPILER_BARRIER();
	__enable_irq();
}

static ALWAYS_INLINE void
adiv5_swdio_turnaround_input(uint32_t cfg)
{
	uint32_t ta_ticks;

	adiv5_swdio_mode_in();

	ta_ticks = (cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT;

	/* Cycle the clock */
#pragma GCC unroll 0
	while (ta_ticks--)
		swdp_swclk_toggle(ADIV5_LL_CLKDELAY(cfg));
}

static ALWAYS_INLINE void
adiv5_swdio_turnaround_input_zero(uint32_t cfg)
{
	uint32_t ta_ticks;

	adiv5_swdio_mode_in();

	ta_ticks = (cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT;

	/* Cycle the clock */
#pragma GCC unroll 0
	while (ta_ticks--)
		swdp_swclk_toggle_zero();
}

static ALWAYS_INLINE void
adiv5_swdio_turnaround_output(uint32_t cfg)
{
	uint32_t ta_ticks;

	ta_ticks = (cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT;

	/* Cycle the clock */
#pragma GCC unroll 0
	while (ta_ticks--)
		swdp_swclk_toggle(ADIV5_LL_CLKDELAY(cfg));

	adiv5_swdio_mode_out();
}

static ALWAYS_INLINE void
adiv5_swdio_turnaround_output_zero(uint32_t cfg)
{
	uint32_t ta_ticks;

	ta_ticks = (cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT;

	/* Cycle the clock */
#pragma GCC unroll 0
	while (ta_ticks--)
		swdp_swclk_toggle_zero();

	adiv5_swdio_mode_out();
}

static ALWAYS_INLINE void
adiv5_swdp_clock_out(uint32_t del, uint32_t bit)
{

	HW_PIN_SWCLK_clr();
	__COMPILER_BARRIER();

	if (bit)
		HW_PIN_SWDIO_set();
	else
		HW_PIN_SWDIO_clr();

	timer_delay_cycles(del);

	HW_PIN_SWCLK_set();

	timer_delay_cycles(del);
}

static ALWAYS_INLINE void
adiv5_swdp_clock_out_zero(uint32_t bit)
{

	HW_PIN_SWCLK_clr();
	__COMPILER_BARRIER();

	if (bit)
		HW_PIN_SWDIO_set();
	else
		HW_PIN_SWDIO_clr();
	__COMPILER_BARRIER();

	HW_PIN_SWCLK_set();
	__COMPILER_BARRIER();
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_in(uint32_t del)
{
	uint32_t rv;

	HW_PIN_SWCLK_clr();

	timer_delay_cycles(del);

	rv = HW_PIN_SWDIO_read();

	HW_PIN_SWCLK_set();

	timer_delay_cycles(del);

	return rv;
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_in_zero(void)
{
	uint32_t rv;

	HW_PIN_SWCLK_clr();

	rv = HW_PIN_SWDIO_read();

	HW_PIN_SWCLK_set();

	return rv;
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_out_bulk(uint32_t del, uint32_t data, uint32_t bits)
{
	uint32_t parity = 0;

#pragma GCC unroll 0
	while (bits--) {
		parity += data & 1;
		adiv5_swdp_clock_out(del, data & 1);
		data >>= 1;
	}

	return parity & 1;
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_out_bulk_zero(uint32_t data, uint32_t bits)
{
	uint32_t parity = 0;

#pragma GCC unroll 0
	while (bits--) {
		parity += data & 1;
		adiv5_swdp_clock_out_zero(data & 1);
		data >>= 1;
	}

	return parity & 1;
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_in_bulk(uint32_t del, uint32_t bits, uint32_t *ppar)
{
	uint32_t mask, rv;
	bool parity;

	rv = 0;
	parity = false;
	mask = 1u;

#pragma GCC unroll 0
	while (bits--) {
		if (adiv5_swdp_clock_in(del)) {
			parity = !parity;
			rv |= mask;
		}
		mask <<= 1;
	}

	if (ppar)
		*ppar = (uint32_t)parity;

	return rv;
}

static ALWAYS_INLINE uint32_t
adiv5_swdp_clock_in_bulk_zero(uint32_t bits, uint32_t *ppar)
{
	uint32_t mask, rv;
	bool parity;

	rv = 0;
	parity = false;
	mask = 1u;

#pragma GCC unroll 0
	while (bits--) {
		if (adiv5_swdp_clock_in_zero()) {
			parity = !parity;
			rv |= mask;
		}
		mask <<= 1;
	}

	if (ppar)
		*ppar = (uint32_t)parity;

	return rv;
}

static FAST_RAMFUNC uint32_t
adiv5_swdp_transfer(uint32_t cfg, uint32_t req, uint32_t *data)
{
	uint32_t del, d, v, ack;

	/*
	 * SWD request. Bits are sent LSB first.
	 * +------+------+------+------+------+------+------+------+
	 * |7     |6     |5     |4     |3     |2     |1     |0     |
	 * | Park | Stop |Parity|  A3  |  A2  | RnW  |APnDP |Start |
	 * +------+------+------+------+------+------+------+------+
	 */

	del = ADIV5_LL_CLKDELAY(cfg);

	/* Send the 8 bits of the request */
	(void) adiv5_swdp_clock_out_bulk(del, req, 8);

	/* Turn around the pin */
	adiv5_swdio_turnaround_input(cfg);

	/* Read the ACK */
	ack = adiv5_swdp_clock_in_bulk(del, 3, NULL);
	if (cfg & ADIV5_LL_SWD_IGNORE_ACK)
		ack = DP_ACK_OK;

	if (ack == DP_ACK_OK) {
		/* So far so good. Start the data transfer */

		if (req & (DP_REQ_RnW << 1)) {
			/* Read request */
			uint32_t parity;

			/* Read word and parity */
			v = adiv5_swdp_clock_in_bulk(del, 32, &parity);
			parity ^= adiv5_swdp_clock_in(del) ? 1 : 0;

			/* DIO pin back to output */
			adiv5_swdio_turnaround_output(cfg);

			if (parity)
				ack = DP_ACK_PARITY;
			else
			if (data != NULL)
				*data = v;
		} else {
			uint32_t parity;

			/* DIO pin back to output */
			adiv5_swdio_turnaround_output(cfg);

			/* Write word and parity */
			parity = adiv5_swdp_clock_out_bulk(del, *data, 32);
			adiv5_swdp_clock_out(del, parity);
		}

		/* Insert idle cycles, if required */
		d = ADIV5_LL_IDLE_CYCLES(cfg);
		if (d) {
			HW_PIN_SWDIO_clr();
#pragma GCC unroll 0
			while (d--)
				swdp_swclk_toggle(del);
		}

		/* Park DIO */
		HW_PIN_SWDIO_set();

		return ack;
	}

	if (ack == DP_ACK_WAIT || ack == DP_ACK_FAULT) {
		if (cfg & ADIV5_LL_SWD_DATA_PHASE) {
			if ((req & (DP_REQ_RnW << 1)) == 0) {
				/* Need to switch back to output */
				adiv5_swdio_turnaround_output(cfg);
				HW_PIN_SWDIO_clr();
			}

			/* Dummy read/write: 32 + parity */
#pragma GCC unroll 0
			for (d = 0; d < 33; d++)
				swdp_swclk_toggle(del);

			if ((req & (DP_REQ_RnW << 1)) != 0) {
				/* Need to switch back to output */
				adiv5_swdio_turnaround_output(cfg);
			}
		} else {
			/* DIO pin back to output */
			adiv5_swdio_turnaround_output(cfg);
		}

		/* Park DIO */
		HW_PIN_SWDIO_set();

		return ack;
	}

	/* ACK is not valid */
	d = 33 + ((cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT);
#pragma GCC unroll 0
	while (d--)
		swdp_swclk_toggle(del);

	/* Park DIO */
	HW_PIN_SWDIO_set();
	adiv5_swdio_mode_out();

	return ack;
}

static FAST_RAMFUNC uint32_t
adiv5_swdp_transfer_zero(uint32_t cfg, uint32_t req, uint32_t *data)
{
	uint32_t d, v, ack;

	/*
	 * SWD request. Bits are sent LSB first.
	 * +------+------+------+------+------+------+------+------+
	 * |7     |6     |5     |4     |3     |2     |1     |0     |
	 * | Park | Stop |Parity|  A3  |  A2  | RnW  |APnDP |Start |
	 * +------+------+------+------+------+------+------+------+
	 */

	/* Send the 8 bits of the request */
	(void) adiv5_swdp_clock_out_bulk_zero(req, 8);
	adiv5_swdio_turnaround_input_zero(cfg);
	ack = adiv5_swdp_clock_in_bulk_zero(3, NULL);
	if (cfg & ADIV5_LL_SWD_IGNORE_ACK)
		ack = DP_ACK_OK;

	if (ack == DP_ACK_OK) {
		/* So far so good. Start the data transfer */

		if (req & (DP_REQ_RnW << 1)) {
			/* Read request */
			uint32_t parity;

			/* Read word and parity */
			v = adiv5_swdp_clock_in_bulk_zero(32, &parity);
			parity ^= adiv5_swdp_clock_in_zero() ? 1 : 0;

			if (parity)
				ack = DP_ACK_PARITY;

			/* DIO pin back to output */
			adiv5_swdio_turnaround_output_zero(cfg);

			if (data != NULL)
				*data = v;
		} else {
			uint32_t parity;

			/* DIO pin back to output */
			adiv5_swdio_turnaround_output_zero(cfg);
			/* Write word and parity */
			parity = adiv5_swdp_clock_out_bulk_zero(*data, 32);
			adiv5_swdp_clock_out_zero(parity);
		}

		/* Insert idle cycles, if required */
		d = ADIV5_LL_IDLE_CYCLES(cfg);
		if (d) {
			HW_PIN_SWDIO_clr();
#pragma GCC unroll 0
			while (d--)
				swdp_swclk_toggle_zero();
		}

		/* Park DIO */
		add_nops(1);
		HW_PIN_SWDIO_set();

		return ack;
	}

	if (ack == DP_ACK_WAIT || ack == DP_ACK_FAULT) {
		if (cfg & ADIV5_LL_SWD_DATA_PHASE) {
			if ((req & (DP_REQ_RnW << 1)) == 0) {
				/* Need to switch back to output */
				adiv5_swdio_turnaround_output_zero(cfg);
				HW_PIN_SWDIO_clr();
			}

			/* Dummy read/write: 32 + parity */
#pragma GCC unroll 0
			for (d = 0; d < 33; d++)
				swdp_swclk_toggle_zero();

			if ((req & (DP_REQ_RnW << 1)) != 0) {
				/* Need to switch back to output */
				add_nops(4);
				adiv5_swdio_turnaround_output_zero(cfg);
			}
		} else {
			/* DIO pin back to output */
			adiv5_swdio_turnaround_output_zero(cfg);
		}

		/* Park DIO */
		add_nops(1);
		HW_PIN_SWDIO_set();

		return ack;
	}

	/* ACK is not valid */
	d = 33 + ((cfg & ADIV5_LL_SWD_TURNAROUND_MASK) >>
	    ADIV5_LL_SWD_TURNAROUND_SHIFT);
#pragma GCC unroll 0
	while (d--)
		swdp_swclk_toggle_zero();

	/* Park DIO */
	HW_PIN_SWDIO_set();
	adiv5_swdio_mode_out();

	return ack;
}
#pragma GCC pop_options

static void
adiv5_swdp_line_reset_impl(uint32_t cfg)
{
	uint32_t del = ADIV5_LL_CLKDELAY(cfg);

	/* Send the line reset sequence */
	HW_PIN_SWDIO_set();

	if (del) {
		/* 50 cycles with SWDIO high */
#pragma GCC unroll 0
		for (int i = 0; i < 50; i++)
			swdp_swclk_toggle(del);
		/* 2 cycles with SWDIO low */
		(void) adiv5_swdp_clock_out_bulk(del, 0, 2);
	} else {
		/* 50 cycles with SWDIO high */
#pragma GCC unroll 0
		for (int i = 0; i < 50; i++)
			swdp_swclk_toggle_zero();
		/* 2 cycles with SWDIO low */
		(void) adiv5_swdp_clock_out_bulk_zero(0, 2);
	}
}

static void
adiv5_swdp_start(uint32_t cfg)
{
	rtos_saved_ipl_t ipl;
	uint32_t del;
	int i;

	/* Ensure DIO is high */
	HW_PIN_SWDIO_set();

	/* Atomic to minimise contention time. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);

	/* DIO pin and external buffer to output */
	HW_PIN_SWDIO_out();
	HW_PIN_SWDIO_DIR_set();

	/* SWO to input, connected to UART. */
	HW_PIN_SWO_DIR_clr();
	HW_PIN_SWO_pullup(1);
	HW_PIN_SWO_function(SWO_UART_RX_FUNCTION);

	rtos_ipl_restore(ipl);

	if (((adiv5_ll_config | adiv5_ll_cfg_extra_set) &
	    ADIV5_LL_NRST_EXTENDED) != 0) {
		/* Assert RESET */
		adiv5_ll_nRST_assert();
		timer_buzz_delay_us(50);

		/* Ensure CLK is low */
		HW_PIN_SWCLK_clr();

		/* Atomic to minimise contention time. */
		ipl = rtos_ipl_raise(HW_IPL_MAX);
		/* CLK pin to output */
		HW_PIN_SWCLK_out();
		/* CLK external buffer to output */
		HW_PIN_SWCLK_DIR_set();
		rtos_ipl_restore(ipl);

		timer_buzz_delay_us(50);

		/* Deassert RESET */
		adiv5_ll_nRST_deassert();
		timer_buzz_delay_us(50);

		/* Bring CLK high */
		HW_PIN_SWCLK_set();
		timer_delay_cycles(100);
	} else {
		/* Ensure CLK is high */
		HW_PIN_SWCLK_set();

		/* Atomic to minimise contention time. */
		ipl = rtos_ipl_raise(HW_IPL_MAX);
		/* CLK pin to output */
		HW_PIN_SWCLK_out();
		/* CLK external buffer to output */
		HW_PIN_SWCLK_DIR_set();
		rtos_ipl_restore(ipl);
	}

	del = ADIV5_LL_CLKDELAY(cfg);

	if ((cfg & ADIV5_LL_SWD_SWITCH_TO_SWD) != 0) {
		/* Switch from JTAG to SWD */
		HW_PIN_SWDIO_set();
		if (del) {
#pragma GCC unroll 0
			for (i = 0; i < 64; i++)
				swdp_swclk_toggle(del);
			(void) adiv5_swdp_clock_out_bulk(del, 0xe79e, 16);
			HW_PIN_SWDIO_set();
#pragma GCC unroll 0
			for (i = 0; i < 64; i++)
				swdp_swclk_toggle(del);
		} else {
#pragma GCC unroll 0
			for (i = 0; i < 64; i++)
				swdp_swclk_toggle_zero();
			(void) adiv5_swdp_clock_out_bulk_zero(0xe79e, 16);
			HW_PIN_SWDIO_set();
#pragma GCC unroll 0
			for (i = 0; i < 64; i++)
				swdp_swclk_toggle_zero();
		}

	}

	if ((cfg & ADIV5_LL_SWD_DORMANT) != 0) {
		/* Switch out of dormant state */
		HW_PIN_SWDIO_set();
		if (del) {
			/* 8 cycles with SWDIO high */
#pragma GCC unroll 0
			for (i = 0; i < 8; i++)
				swdp_swclk_toggle(del);
			/* Alert sequence */
			(void) adiv5_swdp_clock_out_bulk(del, 0x6209F392u, 32);
			(void) adiv5_swdp_clock_out_bulk(del, 0x86852D95u, 32);
			(void) adiv5_swdp_clock_out_bulk(del, 0xE3DDAFE9u, 32);
			(void) adiv5_swdp_clock_out_bulk(del, 0x19BC0EA2u, 32);
			/*
			 * 4 cycles with SWDIO low, followed by 0x1a,
			 * followed by 8 cycles with SWDIO high.
			 */
			(void) adiv5_swdp_clock_out_bulk(del, 0xff1a0u, 20);
		} else {
			/* 8 cycles with SWDIO high */
#pragma GCC unroll 0
			for (i = 0; i < 8; i++)
				swdp_swclk_toggle_zero();
			/* Alert sequence */
			(void) adiv5_swdp_clock_out_bulk_zero(0x6209F392u, 32);
			(void) adiv5_swdp_clock_out_bulk_zero(0x86852D95u, 32);
			(void) adiv5_swdp_clock_out_bulk_zero(0xE3DDAFE9u, 32);
			(void) adiv5_swdp_clock_out_bulk_zero(0x19BC0EA2u, 32);
			/*
			 * 4 cycles with SWDIO low, followed by 0x1a,
			 * followed by 8 cycles with SWDIO high.
			 */
			(void) adiv5_swdp_clock_out_bulk_zero(0x1a0u, 12);
		}
	}

	if ((cfg & ADIV5_LL_SWD_LINE_RESET) != 0)
		adiv5_swdp_line_reset_impl(cfg);
}

static void
adiv5_swdp_stop(uint32_t cfg)
{
	rtos_saved_ipl_t ipl;
	uint32_t del;
	int i;

	del = ADIV5_LL_CLKDELAY(cfg);

	if ((cfg & ADIV5_LL_SWD_DORMANT) != 0) {
		/* Switch to dormant state */
		HW_PIN_SWDIO_set();
		if (del) {
#pragma GCC unroll 0
			for (i = 0; i < 50; i++)
				swdp_swclk_toggle(del);
			(void) adiv5_swdp_clock_out_bulk(del, 0xe3bc, 16);
		} else {
#pragma GCC unroll 0
			for (i = 0; i < 50; i++)
				swdp_swclk_toggle_zero();
			(void) adiv5_swdp_clock_out_bulk_zero(0xe3bc, 16);
		}
	}

	if ((cfg & ADIV5_LL_SWD_LINE_RESET) != 0)
		adiv5_swdp_line_reset_impl(cfg);

	/* DIO to input */
	adiv5_swdio_mode_in();

	/* Atomic to minimise contention time. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);

	/* CLK pin and external buffer to input */
	HW_PIN_SWCLK_DIR_clr();
	HW_PIN_SWCLK_in();

	/* SWO pin back to GPIO mode. */
	HW_PIN_SWO_DIR_clr();
	HW_PIN_SWO_gpio();

	rtos_ipl_restore(ipl);
}

void
adiv5_swdp_switch_to_jtag(uint32_t cfg)
{
	unsigned int i;
	uint32_t del;

	if ((cfg & ADIV5_LL_SWITCH_TO_JTAG) == 0)
		return;

	/*
	 * Switch from SWD to JTAG mode.
	 * TMS is the same physical pin as SWDIO, so reuse the SWD
	 * bulk routines.
	 */

	del = ADIV5_LL_CLKDELAY(cfg);;

	HW_PIN_SWDIO_set();

	if (del) {
#pragma GCC unroll 0
		for (i = 0; i < 64; i++)
			swdp_swclk_toggle(del);

		/* Clock out the magic sequence on TMS. */
		(void) adiv5_swdp_clock_out_bulk(del, 0xe73c, 16);
		HW_PIN_SWDIO_set();
#pragma GCC unroll 0
		for (i = 0; i < 6; i++)
			swdp_swclk_toggle(del);
	} else {
#pragma GCC unroll 0
		for (i = 0; i < 64; i++)
			swdp_swclk_toggle_zero();

		/* Clock out the magic sequence on TMS. */
		(void) adiv5_swdp_clock_out_bulk_zero(0xe73c, 16);
		HW_PIN_SWDIO_set();
#pragma GCC unroll 0
		for (i = 0; i < 6; i++)
			swdp_swclk_toggle_zero();
	}
}

static __inline uint32_t
adid5_swdp_construct_swd_request(uint32_t req)
{
	uint32_t bits;

	/*
	 * Given APnDP, RnW, and A2/A3, construct the 8-bit request to
	 * send to the target, including Start/Stop/Park bits and Parity bit.
	 * This compiles down to 7-8 instructions; no branching or look-up
	 * table required.
	 */

	/* Start/Stop/Park bits. */
	bits = 0x81u;

	/* Mask and shift APnDP, RnW, and A2/A3 to the appropriate positions. */
	bits |= (req & 0xfu) << 1;

	/*
	 * Compute the Parity bit.
	 * We use the 4 'data' bits to shift a magic 16-bit constant. After
	 * shifting, bit 0 yields the parity, which is then shifted into place.
	 */
	bits |= ((0x6996u >> (req & 0xfu)) & 1u) << 5;

	return bits;
}

static int
adiv5_swdp_init(adiv5_dp_t dp, uint32_t cfg)
{
	unsigned int speed, idles;

	adiv5_dp_get_speed_idles(&speed, &idles);

	/* Turnaround starts at one until we change DLCR.TURNAROUND */
	dp->dp_cfg = 1 << ADIV5_LL_SWD_TURNAROUND_SHIFT;

	dp->dp_cfg |= adiv5_speed_2cfg[speed] << ADIV5_LL_CLKDELAY_SHIFT;
	dp->dp_cfg |= idles << ADIV5_LL_IDLE_CYCLES_SHIFT;
	dp->dp_cfg |= ADIV5_LL_SWD_LINE_RESET;
	dp->dp_cfg |= cfg;
	ADIV5_DP_FORCE_SELECT_SET(dp);

	adiv5_swdp_start(dp->dp_cfg);

	return 1;
}

static void
adiv5_swdp_done(adiv5_dp_t dp)
{

	adiv5_swdp_stop(dp->dp_cfg | ADIV5_LL_SWD_LINE_RESET);
}

static uint32_t
adiv5_swdp_xfer(adiv5_dp_t dp, uint32_t req, uint32_t *data)
{
	timer_timeout_t to;
	uint32_t ack, dpreq;
	uint32_t (*xfer)(uint32_t, uint32_t, uint32_t *);
	bool retry;

	DBFPRINTF("%cP %s, reg 0x%02" PRIx32 ", *data %08" PRIx32 "\n",
	    (req & DP_REQ_APnDP) ? 'A' : 'D',
	    (req & DP_REQ_RnW) ? "Read" : "Write",
	    req & (DP_REQ_A2 | DP_REQ_A3),
	    (data != NULL) ? *data : 0xfffffffful);

	if (ADIV5_DP_STICKY_ERROR(dp)) {
		DBFPRINTF("%cP %s, req %02" PRIx32 ", *data %08" PRIx32
		    ". Aborted. Pending sticky fault.\n",
		    (req & DP_REQ_APnDP) ? 'A' : 'D',
		    (req & DP_REQ_RnW) ? "Read" : "Write",
		    req & (DP_REQ_A2 | DP_REQ_A3),
		    (data != NULL) ? *data : 0xfffffffful);
		dp->dp_last_ack = DP_ACK_FAULT;
		return DP_ACK_FAULT;
	}

	xfer = ADIV5_LL_CLKDELAY(dp->dp_cfg) ? adiv5_swdp_transfer :
	    adiv5_swdp_transfer_zero;

	dpreq = adid5_swdp_construct_swd_request(req);

	timer_timeout_start(&to, 2000);
	retry = false;

	do {
		ack = xfer(dp->dp_cfg, dpreq, data);
		switch (ack) {
		case DP_ACK_OK:
		case DP_ACK_FAULT:
			retry = false;
			break;
		case DP_ACK_WAIT:
		case DP_ACK_PARITY:
			retry = true;
			break;
		default:
			ack = DP_ACK_FAULT;
			retry = false;
			break;
		}
	} while (retry && !timer_timeout_expired(&to));

	DBFPRINTF("ack %02" PRIx32 ", timer %s\n", ack,
	    timer_timeout_expired(&to) ? "Expired" : "Ok");

	if (ack == DP_ACK_FAULT) {
		ADIV5_DP_STICKY_ERROR_SET(dp);

		DBFPRINTF("%cP %s, req %02" PRIx32 ", *data %08" PRIx32
		    ". Sticky FAULT reported.\n",
		    (req & DP_REQ_APnDP) ? 'A' : 'D',
		    (req & DP_REQ_RnW) ? "Read" : "Write",
		    req & (DP_REQ_A2 | DP_REQ_A3),
		    (data != NULL) ? *data : 0xfffffffful);
	}

	if (ack == DP_ACK_OK && (req & DP_REQ_RnW) && data != NULL)
		DBFPRINTF("data %08" PRIx32 "\n", *data);

	dp->dp_last_ack = ack;

	return ack;
}

static void
adiv5_swdp_line_reset(adiv5_dp_t dp)
{

	adiv5_swdp_line_reset_impl(dp->dp_cfg);
}

/*---------------------------------------------------------------------------*/

static int
adiv5_swjdp_init(adiv5_dp_t dp, uint32_t cfg)
{
	unsigned int speed, idles;

	adiv5_dp_get_speed_idles(&speed, &idles);

	/* Turnaround starts at one until we change DLCR.TURNAROUND */
	dp->dp_cfg = 1 << ADIV5_LL_SWD_TURNAROUND_SHIFT;

	dp->dp_cfg |= adiv5_speed_2cfg[speed] << ADIV5_LL_CLKDELAY_SHIFT;
	dp->dp_cfg |= idles << ADIV5_LL_IDLE_CYCLES_SHIFT;
	dp->dp_cfg |= ADIV5_LL_SWD_LINE_RESET;
	dp->dp_cfg |= cfg;
	ADIV5_DP_FORCE_SELECT_SET(dp);

	/* Put the external interface in SWD mode */
	adiv5_swdp_start(dp->dp_cfg | ADIV5_LL_SWD_SWITCH_TO_SWD);

	return 1;
}

static void
adiv5_swjdp_done(adiv5_dp_t dp)
{

	adiv5_swdp_stop(dp->dp_cfg | ADIV5_LL_SWD_LINE_RESET);
}

const struct adiv5_dp_interface adiv5_swdp_if = {
	.dpi_name = "SW-DP",
	.dpi_type = ADIV5_TYPE_SWDP,
	.dpi_init = adiv5_swdp_init,
	.dpi_done = adiv5_swdp_done,
	.dpi_xfer = adiv5_swdp_xfer,
	.dpi_line_reset = adiv5_swdp_line_reset,
	.dpi_abort = NULL,
};

const struct adiv5_dp_interface adiv5_swjdp_if = {
	.dpi_name = "SWJ-DP",
	.dpi_type = ADIV5_TYPE_SWJDP,
	.dpi_init = adiv5_swjdp_init,
	.dpi_done = adiv5_swjdp_done,
	.dpi_xfer = adiv5_swdp_xfer,
	.dpi_line_reset = adiv5_swdp_line_reset,
	.dpi_abort = NULL,
};
