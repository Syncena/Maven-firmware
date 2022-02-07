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
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>

#include "hexdump.h"
#include "debug.h"

/*
 * TODO: Collapse repeating lines to a single '*'
 */

static void
hexdump_fprintf(FILE *os, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (os == NULL)
		debug_vprint(fmt, ap);
	else
		vfprintf(os, fmt, ap);
	va_end(ap);
}

void
hexdump(FILE *os, const char *prefix, unsigned per_line,
    const uint8_t *buff, size_t len)
{
	unsigned int i, total;
	char pr[17];

	if (prefix == NULL)
		prefix = "";

	total = 0;
	pr[0] = '\0';

	while (len) {
		hexdump_fprintf(os, "%s%04x:", prefix, total);

		for (i = 0; i < per_line && len; i++, len--) {
			uint8_t ch = *buff++;
			hexdump_fprintf(os, " %02x", (unsigned int)ch);
			if (per_line < sizeof(pr))
				pr[i] = (char)(isprint(ch) ? ch : '.');
		}

		if (per_line < sizeof(pr)) {
			pr[i] = '\0';
			while (i++ < per_line)
				hexdump_fprintf(os, "   ");
		}

		hexdump_fprintf(os, " %s\n", pr);

		total += per_line;
	}
}
