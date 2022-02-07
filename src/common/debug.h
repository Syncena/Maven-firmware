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

#ifndef DEBUG_H
#define DEBUG_H

#include <stdarg.h>

#if (RELEASE_BUILD != 0)
#define	DBPRINTF(fmt, ...)	do {/* Nothing */} while (/*CONSTCOND*/0)
#define	DBFPRINTF(fmt, ...)	do {/* Nothing */} while (/*CONSTCOND*/0)
#define	DBMPRINTF(m,fmt, ...)	do {/* Nothing */} while (/*CONSTCOND*/0)
#define	DBMFPRINTF(m,fmt, ...)	do {/* Nothing */} while (/*CONSTCOND*/0)
#else

#ifndef DEBUG_FLAG
#ifndef	DEBUG_FLAG_INIT
#define	DEBUG_FLAG_INIT	0
#else
#define	DEBUG_ENABLED	1
#endif
#define	DEBUG_FLAG	DEBUG_FLAG_INIT
#else
#define	DEBUG_ENABLED	1
unsigned int DEBUG_FLAG
#ifdef DEBUG_FLAG_INIT
	= DEBUG_FLAG_INIT
#endif
	;
#endif	/* DEBUG_FLAG */

#define	DBPRINTF(fmt, ...)						\
	do {								\
		if (DEBUG_FLAG)						\
			debug_print(fmt, ##__VA_ARGS__);		\
	} while (/*CONSTCOND*/0)

#define	DBFPRINTF(fmt, ...)						\
	do {								\
		if (DEBUG_FLAG)						\
			debug_print("%s: " fmt, __func__, ##__VA_ARGS__);\
	} while (/*CONSTCOND*/0)

#define	DBMPRINTF(m,fmt, ...)						\
	do {								\
		if (DEBUG_FLAG & (m))					\
			debug_print(fmt, ##__VA_ARGS__);		\
	} while (/*CONSTCOND*/0)

#define	DBMFPRINTF(m,fmt, ...)						\
	do {								\
		if (DEBUG_FLAG & (m))					\
			debug_print("%s: " fmt, __func__, ##__VA_ARGS__);\
	} while (/*CONSTCOND*/0)

#endif /* (RELEASE_BUILD != 0) */

extern int debug_print(const char *, ...);
extern int debug_vprint(const char *, va_list);
extern void debug_flush(void);

#endif /* DEBUG_H */
