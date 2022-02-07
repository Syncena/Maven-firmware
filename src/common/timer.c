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

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#endif
#include "timer.h"
#include "rtos.h"
#include "platform.h"

/* Number of microseconds per timer interrupt */
#define	USECS_PER_TICK	(1000000ul / TIMER_HZ)

/* Number of microseconds per hardware timer count tick */
#define	TIMER_CLOCK	F_CPU
#define	TIMER_PER_USEC	(TIMER_CLOCK / 1000000ul)
#define	CNT_TO_USECS(c)	(((c) + (TIMER_PER_USEC / 2)) / TIMER_PER_USEC)

struct timer_state {
	struct timeval ts_mono_time;
	timer_timeout_t ts_ticks;
} timer_state;

void
timer_get_mono_time(struct timeval *tv)
{
	rtos_saved_ipl_t ipl;
	timer_cnt_t tcnt;

	ipl = rtos_ipl_raise(HW_IPL_TIMER);
	tcnt = timer_get_elapsed_ticks();
	*tv = timer_state.ts_mono_time;
	rtos_ipl_restore(ipl);

	tv->tv_usec += CNT_TO_USECS(tcnt);

	if (tv->tv_usec >= 1000000l) {
		tv->tv_sec++;
		tv->tv_usec -= 1000000l;
	}
}

uint32_t
timer_get_mono_usecs(void)
{
	struct timeval tv;

	timer_get_mono_time(&tv);

	return ((uint32_t)tv.tv_sec * 1000000ul) + (uint32_t)tv.tv_usec;
}

void
timer_add(const struct timeval *t1, const struct timeval *t2,
    struct timeval *rv)
{

	rv->tv_sec = t1->tv_sec + t2->tv_sec;
	rv->tv_usec = t1->tv_usec + t2->tv_usec;

	if (rv->tv_usec > 1000000l) {
		rv->tv_sec += 1;
		rv->tv_usec -= 1000000l;
	}
}

void
timer_sub(const struct timeval *btv, const struct timeval *ltv,
    struct timeval *rv)
{

	rv->tv_sec = btv->tv_sec - ltv->tv_sec;
	rv->tv_usec = btv->tv_usec - ltv->tv_usec;

	if (rv->tv_usec < 0) {
		rv->tv_usec += 1000000ul;
		rv->tv_sec -= 1;
	}
}

void
timer_timeval_to_string(char *str, size_t blen, const struct timeval *tv)
{
	uint16_t days;
	uint8_t hours, mins, secs;
	ldiv_t l;

	l = ldiv(tv->tv_sec, 60ul * 60ul * 24ul);
	days = (uint16_t)l.quot;
	l = ldiv(l.rem, 60ul * 60ul);
	hours = (uint8_t)l.quot;
	l = ldiv(l.rem, 60ul);
	mins = (uint8_t)l.quot;
	secs = (uint8_t)l.rem;

	snprintf(str, blen, "%3" PRIu16 ":%02u:%02u:%02u.%06lu", days,
	    (unsigned int)hours, (unsigned int)mins, (unsigned int)secs,
	    (unsigned long)tv->tv_usec);
}
 
void
timer_timeout_start(timer_timeout_t *tp, uint32_t ms)
{
	timer_timeout_t cur_ticks;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_TIMER);
	cur_ticks = timer_state.ts_ticks;
	rtos_ipl_restore(ipl);

	*tp = cur_ticks + ms;
}

bool
timer_timeout_expired(const timer_timeout_t *tp)
{
	timer_timeout_t cur_ticks;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_TIMER);
	cur_ticks = timer_state.ts_ticks;
	rtos_ipl_restore(ipl);

	return cur_ticks >= *tp;
}

uint32_t
timer_timeout_remaining(const timer_timeout_t *tp)
{
	timer_timeout_t cur_ticks;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_TIMER);
	cur_ticks = timer_state.ts_ticks;
	rtos_ipl_restore(ipl);

	if (cur_ticks >= *tp)
		return 0ul;

	return (uint32_t)(*tp - cur_ticks);
}

FAST_RAMFUNC void
timer_tick(void)
{
	struct timer_state *ts = &timer_state;
	rtos_saved_ipl_t ipl;
	int32_t m_usec;
	time_t m_sec;

	m_sec = ts->ts_mono_time.tv_sec;
	m_usec = ts->ts_mono_time.tv_usec + USECS_PER_TICK;
	if (m_usec >= 1000000l) {
		m_usec -= 1000000l;
		m_sec++;
	}

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	ts->ts_mono_time.tv_sec = m_sec;
	ts->ts_mono_time.tv_usec = m_usec;
	rtos_ipl_restore(ipl);

	ts->ts_ticks++;

#ifdef POWER_CHECK
	POWER_CHECK();
#endif
}
