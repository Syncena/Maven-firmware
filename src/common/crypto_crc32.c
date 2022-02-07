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

/*
 * Get the libtomcrypto footprint as small as possible.
 */
#define	LTC_NOTHING
#define	LTC_MD5
#define	LTC_RIJNDAEL
#define	LTC_CBC_MODE
#define	LTC_PKCS_5
#define	LTC_HMAC
#define	LTC_HASH_HELPERS
#define	LTC_CRC32
#define	LTC_NO_TEST
#define	LTC_NO_FILE
#define	LTC_SMALL_CODE
#include <tomcrypt.h>

#include "crypto_crc32.h"
#include "zone_alloc.h"

uint32_t
crypto_crc32(const void *buff, uint32_t len)
{
	crc32_state ctx;
	uint32_t crc;

	crc32_init(&ctx);
	crc32_update(&ctx, buff, len);
	crc32_finish(&ctx, &crc, sizeof(crc));

	return crc;
}

void *
crypto_crc32_init(void)
{
	crc32_state *ctx;

	ctx = zone_calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		return NULL;

	crc32_init(ctx);
	return ctx;
}

uint32_t
crypto_crc32_finish(void *cookie)
{
	crc32_state *ctx = cookie;
	uint32_t crc;

	crc32_finish(ctx, &crc, sizeof(crc));
	zone_free(ctx);

	return crc;
}

void
crypto_crc32_add(void *cookie, const void *buff, uint32_t len)
{
	crc32_state *ctx = cookie;

	crc32_update(ctx, buff, len);
}
