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

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "rtos.h"
#include "timer.h"
#include "debug.h"
#include "console.h"

static rtos_mutex_static_t debug_mutex_store;
static rtos_mutex_t debug_mutex;

int
debug_vprint(const char *fmt, va_list ap)
{
	rtos_saved_ipl_t ipl;
	int rv;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	if (debug_mutex == NULL)
		debug_mutex = rtos_mutex_create_static(&debug_mutex_store);
	rtos_ipl_restore(ipl);

	if (!_rtos_interrupt_context())
		rtos_mutex_acquire(debug_mutex);

	ipl = rtos_ipl_raise(HW_IPL_USART_CONSOLE);
	rv = vfprintf(stderr, fmt, ap);
	fflush(stderr);
	rtos_ipl_restore(ipl);

	if (!_rtos_interrupt_context())
		rtos_mutex_release(debug_mutex);

	return rv;
}

int
debug_print(const char *fmt, ...)
{
	va_list ap;
	int rv;

	va_start(ap, fmt);

	rv = debug_vprint(fmt, ap);

	va_end(ap);

	return rv;
}

void
debug_flush(void)
{

	console_flush(true);
}
