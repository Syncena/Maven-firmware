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
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "sam_trng.h"
#include "sam_clocks.h"
#include "timer.h"

static int
read_rng(uint32_t *pv)
{
	static uint32_t previous_value;
	timer_timeout_t to;
	uint32_t rv;

	timer_timeout_start(&to, 500);

	do {
		while ((TRNG->INTFLAG.reg & TRNG_INTFLAG_DATARDY) == 0) {
			if (timer_timeout_expired(&to))
				return -1;
			rtos_yield();
		}

		rv = TRNG->DATA.reg;
	} while (rv == previous_value && !timer_timeout_expired(&to));

	if (timer_timeout_expired(&to))
		return -1;

	*pv = previous_value = rv;

	return 0;
}

void
sam_trng_init(void)
{

	sam_periph_clock_enable(ID_TRNG);

	TRNG->CTRLA.reg = TRNG_CTRLA_ENABLE;
	(void) TRNG->CTRLA.reg;
}

int
sam_trng_getentropy(void *buff, size_t bufflen)
{
	uint32_t v;
	size_t c;

	while (bufflen) {
		if (read_rng(&v) < 0)
			return -1;

		if (bufflen >= sizeof(v))
			c = sizeof(v);
		else
			c = bufflen;

		memcpy(buff, &v, c);
		buff = (void *)((uintptr_t)buff + c);
		bufflen -= c;
	}

	return 0;
}
