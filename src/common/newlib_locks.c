/*
 * Copyright (c) 2023, Steve C. Woodford.
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

#include <sys/lock.h>
#include <stdbool.h>
#include <stdlib.h>

#include "rtos.h"
#include "zone_alloc.h"

/*
 * One of these days newlib might use weak symbols for retargetable locking.
 * Until then, we have to use the linker's "--wrap" feature. What a hack.
 */
#if 1
#define	WRAP(n)		__wrap_ ## n
#else
#define	WRAP(n)		n
#endif

/*
 * Can't use assert() within the locking primitives; infinite recursion
 * could rear its ugly head.
 */
#ifndef	NDEBUG
#define	LOCK_ASSERT(x,msg)						       \
	do {								       \
		if (!(x)) {						       \
			extern int _write(int, char *, int);		       \
			_write(1, (char *)(uintptr_t)(msg), sizeof(msg) - 1);  \
			abort();					       \
		}							       \
	} while (/*CONSTCOND*/0)
#else
#define	LOCK_ASSERT(x,msg)						       \
	do {								       \
		/* Nothing */						       \
	} while (/*CONSTCOND*/0)
#endif

struct __lock {
	bool is_recursive;
	rtos_mutex_static_t mutex_static;
	rtos_mutex_t mutex;
};
static struct __lock lock_common_mutex;
static struct __lock lock_common_mutex_r;

#define	LOCK_ALIAS(n)		\
	extern struct __lock __attribute__((alias("lock_common_mutex"))) WRAP(n)
#define	LOCK_ALIAS_RECURSIVE(n)	\
	extern struct __lock __attribute__((alias("lock_common_mutex_r"))) WRAP(n)
#define	LOCK_FN(t, n, ...)	\
	extern t WRAP(n) ( __VA_ARGS__ ); t WRAP(n) ( __VA_ARGS__ )

LOCK_ALIAS_RECURSIVE(__lock___sinit_recursive_mutex);
LOCK_ALIAS_RECURSIVE(__lock___sfp_recursive_mutex);
LOCK_ALIAS_RECURSIVE(__lock___env_recursive_mutex);

#if 0
/* These are not used. */
LOCK_ALIAS(__lock___tz_mutex);
LOCK_ALIAS_RECURSIVE(__lock___atexit_recursive_mutex);
LOCK_ALIAS(__lock___at_quick_exit_mutex);
LOCK_ALIAS_RECURSIVE(__lock___malloc_recursive_mutex);
LOCK_ALIAS(__lock___dd_hash_mutex);
LOCK_ALIAS(__lock___arc4random_mutex);
#endif

static void
common_lock_init(_LOCK_T l, bool recursive)
{

	LOCK_ASSERT(l != NULL, "NULL lock\n");
	l->is_recursive = recursive;
	if (recursive)
		l->mutex = rtos_mutex_create_recursive_static(&l->mutex_static);
	else
		l->mutex = rtos_mutex_create_static(&l->mutex_static);
	LOCK_ASSERT(l->mutex != NULL, "Mutex is NULL\n");
}

LOCK_FN(void, __retarget_lock_init, _LOCK_T *lock)
{
	_LOCK_T l = zone_malloc(sizeof(struct __lock));

	common_lock_init(l, false);
	*lock = l;
}

LOCK_FN(void, __retarget_lock_init_recursive, _LOCK_T *lock)
{
	_LOCK_T l = zone_malloc(sizeof(struct __lock));

	common_lock_init(l, true);
	*lock = l;
}

LOCK_FN(void, __retarget_lock_close, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == false, "Closing recursive mutex\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_delete(lock->mutex);
	zone_free(lock);
}

LOCK_FN(void, __retarget_lock_close_recursive, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == true, "Closing regular mutex\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_delete(lock->mutex);
	zone_free(lock);
}

LOCK_FN(void, __retarget_lock_acquire, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == false, "Acquire on recursive mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_acquire(lock->mutex);
}

LOCK_FN(void, __retarget_lock_acquire_recursive, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == true, "Acquire on regular mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_acquire_recursive(lock->mutex);
}

LOCK_FN(int, __retarget_lock_try_acquire, _LOCK_T lock)
{

	(void)lock;
	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(0, "__retarget_lock_try_acquire!!\n");
	return 0;
}

LOCK_FN(int, __retarget_lock_try_acquire_recursive, _LOCK_T lock)
{

	(void)lock;
	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(0, "__retarget_lock_try_acquire_recursive!!\n");
	return 0;
}

LOCK_FN(void, __retarget_lock_release, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == false, "release recursive mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "mutex is NULL\n");

	rtos_mutex_release(lock->mutex);
}

LOCK_FN(void, __retarget_lock_release_recursive, _LOCK_T lock)
{

	LOCK_ASSERT(_rtos_interrupt_context() == 0, "Interrupt context");
	LOCK_ASSERT(lock->is_recursive == true, "release regular mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "mutex is NULL\n");

	rtos_mutex_release_recursive(lock->mutex);
}

/* Arrange for static locks to be initialised very early on. */
__attribute__((constructor))
static void
retarget_lock_init(void)
{

	common_lock_init(&lock_common_mutex, false);
	common_lock_init(&lock_common_mutex_r, true);
}
