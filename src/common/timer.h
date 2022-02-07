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

#ifndef TIMER_H
#define TIMER_H

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "platform.h"
#include "FreeRTOS.h"

#define	TIMER_HZ		configTICK_RATE_HZ

extern void timer_tick(void) FAST_RAMFUNC;
extern void timer_get_mono_time(struct timeval *);
extern uint32_t timer_get_mono_usecs(void);
extern void timer_timeval_to_string(char *, size_t, const struct timeval *);
extern void timer_add(const struct timeval *, const struct timeval *,
		struct timeval *);
extern void timer_sub(const struct timeval *, const struct timeval *,
		struct timeval *);
#define	timer_cmp(tvp, uvp, cmp)					\
	(((tvp)->tv_sec == (uvp)->tv_sec) ?				\
	    ((tvp)->tv_usec cmp (uvp)->tv_usec) :			\
	    ((tvp)->tv_sec cmp (uvp)->tv_sec))

typedef unsigned long long timer_timeout_t;
extern void timer_timeout_start(timer_timeout_t *, uint32_t);
extern bool timer_timeout_expired(const timer_timeout_t *);
extern uint32_t timer_timeout_remaining(const timer_timeout_t *);

#if (PLATFORM_HAS_CPU_DELAY == 0)
static void
timer_cpu_delay_us(uint32_t us)
{

	if (us == 0)
		return;

	/* Assume best case of 3 cycles per loop. */
	us *= F_CPU / 3000000u;

	timer_delay_cycles(us);
}
#else
extern void timer_cpu_delay_us(uint32_t);
#endif	/* PLATFORM_HAS_CPU_DELAY */

static __always_inline void
timer_buzz_delay_us(uint32_t us)
{

	timer_cpu_delay_us(us);
}

static __always_inline void
timer_buzz_delay_ms(uint32_t ms)
{
	while (ms--)
		timer_buzz_delay_us(1000);
}

#endif /* TIMER_H */
