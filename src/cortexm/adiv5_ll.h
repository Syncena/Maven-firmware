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

#ifndef ADIV5_LL_H
#define ADIV5_LL_H

#include <stdbool.h>

extern void adiv5_ll_init_target(void);
extern void adiv5_ll_load_defaults(void);
extern void adiv5_ll_configure_reset(void);
extern bool adiv5_ll_extended_reset_configured(void);

extern int adiv5_ll_nRST_state(void);
extern void adiv5_ll_nRST_toggle(void);
extern void adiv5_ll_nRST_assert(void);
extern void adiv5_ll_nRST_deassert(void);

/*
 * Timing/state configuration passed to the low-level hw routines..
 */
#define	ADIV5_LL_NRST_DRIVER_MASK	(1u << 0)
#define	ADIV5_LL_NRST_DRIVER_OPEN_DRAIN	(0u << 0)
#define	ADIV5_LL_NRST_DRIVER_PUSH_PULL	(1u << 0)
#define	ADIV5_LL_NRST_EXTENDED		(1u << 1)

#define	ADIV5_LL_CLKDELAY_SHIFT		0
#define	ADIV5_LL_CLKDELAY_MASK		0x000000ffu
#define	ADIV5_LL_CLKDELAY(c)		(((c) & ADIV5_LL_CLKDELAY_MASK) >> \
					 ADIV5_LL_CLKDELAY_SHIFT)
#define	ADIV5_LL_IDLE_CYCLES_SHIFT	8
#define	ADIV5_LL_IDLE_CYCLES_MASK	0x0000ff00u
#define	ADIV5_LL_IDLE_CYCLES(c)		(((c) & ADIV5_LL_IDLE_CYCLES_MASK) >> \
					 ADIV5_LL_IDLE_CYCLES_SHIFT)

/* Specific to the SWD interface */
#define	ADIV5_LL_SWD_TURNAROUND_SHIFT	16
#define	ADIV5_LL_SWD_TURNAROUND_MASK	0x00070000u
#define	ADIV5_LL_SWD_DATA_PHASE		(1u << 20)
#define	ADIV5_LL_SWD_SWITCH_TO_SWD	(1u << 21)
#define	ADIV5_LL_SWITCH_TO_JTAG		ADIV5_LL_SWD_SWITCH_TO_SWD
#define	ADIV5_LL_SWD_LINE_RESET		(1u << 22)
#define	ADIV5_LL_SWD_DORMANT		(1u << 23)
#define	ADIV5_LL_SWD_IGNORE_ACK		(1u << 24)

extern uint32_t adiv5_ll_cfg_extra_set;
extern uint32_t adiv5_ll_cfg_extra_clr;

#ifndef CONFIG_USE_CONFIGDB
#define	ADIV5_LL_GLOBAL_VARS		\
	uint8_t glob_adiv5_ll_config;
#define	adiv5_ll_config		glob.glob_adiv5_ll_config
#else
extern uint8_t adiv5_ll_config;
#endif

#endif /* ADIV5_LL_H */
