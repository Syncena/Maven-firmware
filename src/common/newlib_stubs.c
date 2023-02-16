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
