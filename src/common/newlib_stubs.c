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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rtos.h"
#include "FreeRTOSConfig.h"
#include "console.h"
#include "timer.h"
#include "zone_alloc.h"

int _open(const char *, int, int);
int
_open(const char *fname, int flags, int mode)
{

	(void) fname;
	(void) flags;
	(void) mode;

	return -1;
}

int _close(int);
int
_close(int fd)
{

	(void) fd;

	return -1;
}

int _read(int, char *, int);
int
_read(int fd, char *ptr, int len)
{

	(void) fd;
	(void) ptr;
	(void) len;

	return 0;
}

int _write(int, char *, int);
int
_write(int fd, char *ptr, int len)
{
	int rv = len;

	if (fd == 1 || fd == 2)
		console_write_stdout(ptr, len);
	else
		rv = -1;

	return rv;
}

int _lseek(int, int, int);
int
_lseek(int fd, int ptr, int dir)
{

	(void) fd;
	(void) ptr;
	(void) dir;

	return 0;
}

extern int _fstat(int, struct stat *) __attribute__ ((used));
int
_fstat(int fd, struct stat *st)
{

	(void) fd;

	/*
	 * Return an error for std<in|out|err>. This helps avoid newlib
	 * allocating large buffers.
	 */
	if (fd < 3)
		return -1;

	st->st_mode = S_IFCHR;
	st->st_blksize = 64;
	return 0;
}

extern int _isatty(int)  __attribute__ ((used));
int
_isatty(int fd)
{

	(void) fd;

	return 1;
}

int _getpid(void);
int
_getpid(void)
{

	return 1;
}

int _kill(int, int);
int
_kill(int pid, int sig)
{

	(void) pid;
	(void) sig;

	return 0;
}

void _exit(int);
void
_exit(int status)
{

	(void) status;
	for (;;);
}

int _gettimeofday(struct timeval *tv, void *tzp);
int
_gettimeofday(struct timeval *tv, void *tzp)
{

	if (tv != NULL) {
		timer_get_mono_time(tv);
	} else
	if (tzp != NULL) {
		struct timezone *tz = tzp;

		tz->tz_minuteswest = 0;
		tz->tz_dsttime = 0;
	}

	return 0;
}

extern void *__wrap__malloc_r(struct _reent *r, size_t size)
     __attribute__ ((used));
void *
__wrap__malloc_r(struct _reent *r, size_t size)
{

	(void) r;

	return zone_malloc(size);
}

extern void __wrap__free_r(struct _reent *r, void *ptr)
     __attribute__ ((used));
void
__wrap__free_r(struct _reent *r, void *ptr)
{

	(void) r;

	zone_free(ptr);
}

extern void *__wrap__calloc_r(struct _reent *r, size_t n, size_t size)
     __attribute__ ((used));
void *
__wrap__calloc_r(struct _reent *r, size_t n, size_t size)
{

	(void) r;

	return zone_calloc(n, size);
}

extern void * __wrap__realloc_r(struct _reent *r, void *ptr, size_t size)
     __attribute__ ((used));
void *
__wrap__realloc_r(struct _reent *r, void *ptr, size_t size)
{
	void *rv;

	(void) r;

	if (size == 0)
		return ptr;

	rv = zone_malloc(size);
	if (rv != NULL && ptr != NULL) {
		memcpy(rv, ptr, size);
		zone_free(ptr);
	}

	return rv;
}

extern void __wrap__mstats_r(struct _reent *r, char *p)
     __attribute__ ((used));
void
__wrap__mstats_r(struct _reent *r, char *p)
{

	(void) r;
	(void) p;
}

/*
 * Can't use assert() within the locking primitives; infinite recursion
 * could rear its ugly head.
 */
#ifndef	NDEBUG
#define	LOCK_ASSERT(x,msg)						       \
	do {								       \
		if (!(x)) {						       \
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
	rtos_mutex_t mutex;
};

struct __lock __wrap___lock___sinit_recursive_mutex;
struct __lock __wrap___lock___sfp_recursive_mutex;
struct __lock __wrap___lock___env_recursive_mutex;

#if 0
/* These are not used. */
struct __lock __lock___atexit_recursive_mutex;
struct __lock __lock___at_quick_exit_mutex;
struct __lock __lock___malloc_recursive_mutex;
struct __lock __lock___dd_hash_mutex;
struct __lock __lock___arc4random_mutex;
struct __lock __lock___tz_mutex;
#endif

static void
common_lock_init(_LOCK_T l, bool recursive)
{

	LOCK_ASSERT(l != NULL, "NULL lock\n");
	l->is_recursive = recursive;
	l->mutex = recursive ? rtos_mutex_create_recursive() : rtos_mutex_create();
	LOCK_ASSERT(l->mutex != NULL, "NULL mutex\n");
}

void __wrap___retarget_lock_init (_LOCK_T *lock);
void
__wrap___retarget_lock_init (_LOCK_T *lock)
{
	_LOCK_T l = zone_malloc(sizeof(struct __lock));

	common_lock_init(l, false);
	*lock = l;
}

void __wrap___retarget_lock_init_recursive(_LOCK_T *lock);
void
__wrap___retarget_lock_init_recursive(_LOCK_T *lock)
{
	_LOCK_T l = zone_malloc(sizeof(struct __lock));

	common_lock_init(l, true);
	*lock = l;
}

void __wrap___retarget_lock_close(_LOCK_T lock);
void
__wrap___retarget_lock_close(_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == false, "Closing recursive mutex\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_delete(lock->mutex);
	zone_free(lock);
}

void __wrap___retarget_lock_close_recursive(_LOCK_T lock);
void
__wrap___retarget_lock_close_recursive(_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == true, "Closing regular mutex\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_delete(lock->mutex);
	zone_free(lock);
}

void __wrap___retarget_lock_acquire (_LOCK_T lock);
void
__wrap___retarget_lock_acquire (_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == false, "Acquire on recursive mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_acquire(lock->mutex);
}

void __wrap___retarget_lock_acquire_recursive (_LOCK_T lock);
void
__wrap___retarget_lock_acquire_recursive (_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == true, "Acquire on regular mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "Mutex is NULL\n");

	rtos_mutex_acquire_recursive(lock->mutex);
}

int __wrap___retarget_lock_try_acquire(_LOCK_T lock);
int
__wrap___retarget_lock_try_acquire(_LOCK_T lock)
{

	(void)lock;
	LOCK_ASSERT(0, "__retarget_lock_try_acquire!!\n");
	return 0;
}

int __wrap___retarget_lock_try_acquire_recursive(_LOCK_T lock);
int
__wrap___retarget_lock_try_acquire_recursive(_LOCK_T lock)
{

	(void)lock;
	LOCK_ASSERT(0, "__retarget_lock_try_acquire_recursive!!\n");
	return 0;
}

void __wrap___retarget_lock_release (_LOCK_T lock);
void
__wrap___retarget_lock_release (_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == false, "release recursive mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "mutex is NULL\n");

	rtos_mutex_release(lock->mutex);
}

void __wrap___retarget_lock_release_recursive (_LOCK_T lock);
void
__wrap___retarget_lock_release_recursive (_LOCK_T lock)
{

	LOCK_ASSERT(lock->is_recursive == true, "release regular mtx\n");
	LOCK_ASSERT(lock->mutex != NULL, "mutex is NULL\n");

	rtos_mutex_release_recursive(lock->mutex);
}

/*
 * Arrange for locks to be initialised very early on.
 */
static void
retarget_lock_init(void)
{

	common_lock_init(&__wrap___lock___sinit_recursive_mutex, true);
	common_lock_init(&__wrap___lock___sfp_recursive_mutex, true);
	common_lock_init(&__wrap___lock___env_recursive_mutex, true);
//	common_lock_init(&__wrap___lock___tz_mutex, false);
}

__attribute__((section(".preinit_array")))
    void (* const retarget_lock_init_entry[])(void) = {retarget_lock_init};
