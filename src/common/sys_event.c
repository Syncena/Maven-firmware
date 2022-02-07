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
#include <stdlib.h>

#include "sys_event.h"
#include "linked-lists.h"
#include "rtos.h"
#include "zone_alloc.h"

struct sys_event_handle {
	void (*eh_cb)(void *, sys_event_t, const void *);
	void *eh_arg;
	LIST_ENTRY(sys_event_handle) eh_qent;
};
LIST_HEAD(sys_event_handle_qhead, sys_event_handle);

static rtos_mutex_t sys_event_mutex;
static struct sys_event_handle_qhead sys_event_handles;

void
sys_event_init(void)
{

	sys_event_mutex = rtos_mutex_create();
	assert(sys_event_mutex != NULL);
	LIST_INIT(&sys_event_handles);
}

sys_event_handle_t
sys_event_register(void (*cb)(void *, sys_event_t, const void *), void *arg)
{
	sys_event_handle_t eh;

	if ((eh = zone_malloc(sizeof(*eh))) == NULL)
		return NULL;

	eh->eh_cb = cb;
	eh->eh_arg = arg;

	rtos_mutex_acquire(sys_event_mutex);
	LIST_INSERT_HEAD(&sys_event_handles, eh, eh_qent);
	rtos_mutex_release(sys_event_mutex);

	return eh;
}

void
sys_event_deregister(sys_event_handle_t eh)
{

	rtos_mutex_acquire(sys_event_mutex);
	LIST_REMOVE(eh, eh_qent);
	rtos_mutex_release(sys_event_mutex);

	zone_free(eh);
}

void
sys_event_notify(sys_event_t ev, const void *ev_info)
{
	sys_event_handle_t eh;

	rtos_mutex_acquire(sys_event_mutex);

	LIST_FOREACH(eh, &sys_event_handles, eh_qent)
		(eh->eh_cb)(eh->eh_arg, ev, ev_info);

	rtos_mutex_release(sys_event_mutex);
}
