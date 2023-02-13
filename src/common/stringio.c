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
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stringio.h"
#include "zone_alloc.h"

struct stringio_state {
	FILE *ss_fp;
	char *ss_buff;
	uint16_t ss_bytes;
	uint16_t ss_buff_size;
	bool ss_error;
};
#define	STRINGIO_INCREMENT	128u

static int
stringio_write(void *cookie, const char *buff, int count)
{
	stringio_t ss = cookie;
	size_t new_bytes;

	if (count <= 0)
		return count;

	new_bytes = (size_t)count + (size_t)ss->ss_bytes;
	if (new_bytes >= 0x8000u)
		return -1;

	/*
	 * Note: '>=' is correct here. We need to leave one spare byte
	 * for the '\0' terminator.
	 */
	if (new_bytes >= (size_t)ss->ss_buff_size) {
		char *new_buff;
		size_t need_size;

		for (need_size = ss->ss_buff_size + STRINGIO_INCREMENT;
		    need_size <= new_bytes;
		    need_size += STRINGIO_INCREMENT) {
		}

		new_buff = zone_malloc(need_size);
		if (new_buff == NULL) {
			errno = ENOMEM;
			ss->ss_error = true;
			return -1;
		}

		if (ss->ss_buff != NULL) {
			memcpy(new_buff, ss->ss_buff, (size_t)ss->ss_bytes);
			zone_free(ss->ss_buff);
		}

		ss->ss_buff = new_buff;
		ss->ss_buff_size = need_size;
	}

	memcpy(&ss->ss_buff[ss->ss_bytes], buff, (size_t)count);
	ss->ss_bytes = (uint16_t)new_bytes;

	return count;
}

stringio_t
stringio_new(void)
{
	struct stringio_state *ss;

	if ((ss = zone_malloc(sizeof(*ss))) == NULL)
		return NULL;

	ss->ss_buff = NULL;
	ss->ss_bytes = 0;
	ss->ss_buff_size = 0;
	ss->ss_error = false;

	if ((ss->ss_fp = fwopen(ss, stringio_write)) == NULL) {
		zone_free(ss);
		ss = NULL;
	}

	return ss;
}

const char *
stringio_get_buff(stringio_t ss)
{

	fflush(ss->ss_fp);

	if (ss->ss_buff != NULL) {
		assert(ss->ss_bytes < ss->ss_buff_size);
		ss->ss_buff[ss->ss_bytes] = '\0';
	}

	return ss->ss_error ? NULL : ss->ss_buff;
}

size_t
stringio_get_len(stringio_t ss)
{

	fflush(ss->ss_fp);

	if (ss->ss_buff != NULL) {
		assert(ss->ss_bytes < ss->ss_buff_size);
		return (size_t)ss->ss_bytes;
	}

	return (size_t)0;
}
void
stringio_done(stringio_t ss)
{

	fclose(ss->ss_fp);

	if (ss->ss_buff != NULL)
		zone_free(ss->ss_buff);
	zone_free(ss);
}

void
stringio_printf(stringio_t ss, const char *fmt, ...)
{
	va_list ap;
	int rv;

	if (ss->ss_error == false) {
		va_start(ap, fmt);
		rv = vfprintf(ss->ss_fp, fmt, ap);
		va_end(ap);

		if (rv < 0)
			ss->ss_error = true;
	}
}

void
stringio_cat(stringio_t ss, const char *str)
{
	int rv;

	if (ss->ss_error == false) {
		rv = fputs(str, ss->ss_fp);
		if (rv < 0)
			ss->ss_error = true;
	}
}
