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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rtos.h"
#include "platform.h"
#include "platform_usb.h"
#include "hardware.h"
#include "serial_number.h"
#include "timer.h"
#include "zone_alloc.h"

zone_t platform_rtos_zone;
static struct template_gpio template_gpio;
struct template_gpio *GPIO = &template_gpio;

void
platform_early_init(void)
{
	extern int _stack_top, _heap_end;
	size_t hsize;

	hsize = (size_t)(((uintptr_t)&_heap_end) - ((uintptr_t)&_stack_top));

	(void) zone_create(NULL, (void *)&_stack_top, hsize, 0);

	platform_rtos_zone = zone_create_named("RTOS", NULL,
	    (void *)(uintptr_t)0x20040000u, 16u * 1024u,
	    ZONE_FLAGS_ASSERT_ALLOC_FAILURE);
}

void
platform_fabricate_serial(serial_number_t *sn)
{
	uint32_t u;

	/*
	 * Serial number failed validation. Fabricate something which
	 * has a half decent chance of being unique, but add a '-' to
	 * indicate this is a fabrication.
	 */

	u = 0x12345678u;
	sn->zero = 0x00u;
	sprintf(sn->cstr, "TEM1ZZ%07" PRIX32, u >> 4);
	sn->checksum = 0x00u;
}

void
platform_init(void)
{
}

void
platform_reboot(void)
{
}

void
platform_watchdog_disable(void)
{
}

void
platform_attach_devices(void)
{

	template_usb_init();
}

void
platform_hw_init_target(void)
{

	/*
	 * Configure target i/f.
	 */
}

void platform_benchmark(void);
void
platform_benchmark(void)
{
	timer_timeout_t to;
	unsigned int loops;
	volatile uint32_t *m;
	void *b1, *b2;

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to))
		loops++;
	printf("Benchmark: buzzloops/sec %u\n", loops);

#define	BENCH_CHUNK	32768u
	b1 = zone_malloc(BENCH_CHUNK);
	b2 = zone_malloc(BENCH_CHUNK);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memset(b1, 0, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: RAM clear loops/sec %u\n", loops);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memcpy(b2, b1, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: RAM->RAM loops/sec %u\n", loops);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memcpy(b1, (const void *)(uintptr_t)0x30000u, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: ROM->RAM loops/sec %u\n", loops);

	m = (volatile uint32_t *)(uintptr_t)0x30000u;
	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		loops++;
		(void) *m++;
		if (m >= (volatile uint32_t *)(uintptr_t)(0x30000u + BENCH_CHUNK))
			m = (volatile uint32_t *)(uintptr_t)0x30000u;
	}
	printf("Benchmark: ROM read loops/sec %u\n", loops);

	m = (volatile uint32_t *)(uintptr_t)0x20000000u;
	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		loops++;
		(void) *m++;
		if (m >= (volatile uint32_t *)(uintptr_t)(0x20000000u + BENCH_CHUNK))
			m = (volatile uint32_t *)(uintptr_t)0x20000000u;
	}
	printf("Benchmark: RAM read loops/sec %u\n", loops);

	zone_free(b1);
	zone_free(b2);
}
