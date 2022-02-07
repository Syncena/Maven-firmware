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
#include <stdint.h>
#include <string.h>
#include <machine/endian.h>

/*
 * Get the libtomcrypto footprint as small as possible.
 */
#define LTC_NOTHING
#define LTC_MD5
#define LTC_RIJNDAEL
#define LTC_CBC_MODE
#define LTC_PKCS_5
#define LTC_HMAC
#define LTC_HASH_HELPERS
#define LTC_CRC32
#define LTC_NO_TEST
#define LTC_NO_FILE
#define LTC_SMALL_CODE
#include <tomcrypt.h>

#include "rtos.h"
#include "platform.h"
#include "sam_clocks.h"
#include "sam_trng.h"
#include "timer.h"

#define	CRC32_ERR_BITS	(DSU_STATUSA_BERR | DSU_STATUSA_FAIL | DSU_STATUSA_PERR)

void
crc32_init(crc32_state *ctx)
{

	/*
	 * The DSU is write-protected in the PAC by default, so we need
	 * to enable write access to it.
	 */
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | ID_DSU;

	/* The CRC's starting value has all bits set. */
	ctx->crc = (ulong32)0xffffffffu;
}

void
crc32_update(crc32_state *ctx, const unsigned char *input, unsigned long length)
{
	timer_timeout_t to;

	/*
	 * Input buffer and length should always be aligned and a multiple
	 * of 32-bits.
	 */
	assert((length % sizeof(uint32_t)) == 0);
	assert(((uintptr_t)input % sizeof(uint32_t)) == 0);

	/* Ensure status bits are clear before starting. */
	DSU->STATUSA.reg = CRC32_ERR_BITS | DSU_STATUSA_DONE;

	/*
	 * Load the buffer parameters and initial CRC value.
	 * Note that we don't care about cache coherency since
	 * our cache is write-through and the DSU only reads
	 * from memory.
	 */
	DSU->ADDR.reg = (uint32_t)(uintptr_t)input;
	DSU->LENGTH.reg = (uint32_t)length;
	DSU->DATA.reg = ctx->crc;

	/* Start the transfer. */
	DSU->CTRL.reg = DSU_CTRL_CRC;

	/* The DSU is pretty fast, but a timeout is useful. */
	timer_timeout_start(&to, 2000u);

	/* Wait for DONE, or an error. */
	while ((DSU->STATUSA.reg & (CRC32_ERR_BITS | DSU_STATUSA_DONE)) == 0 &&
	    !timer_timeout_expired(&to)) {
		/*
		 * When used in the bootloader, we can be called before the
		 * scheduler has started. In fact, the systick timer is not
		 * yet enabled, so our timeout will never expire. Let's hope
		 * the DSU never fails...
		 */
		if (rtos_scheduler_running())
			rtos_yield();
	}

	/* Something's afoot if the DSU is taking too long. */
	if (timer_timeout_expired(&to))
		return;

	/* On success, save the computed CRC32. */
	if ((DSU->STATUSA.reg & CRC32_ERR_BITS) == 0)
		ctx->crc = (ulong32) DSU->DATA.reg;
}

void
crc32_finish(crc32_state *ctx, void *hash, unsigned long size)
{
	uint32_t crc;

	/* Write-protect the DSU. */
	PAC->WRCTRL.reg = PAC_WRCTRL_KEY_SET | ID_DSU;

	/* These two conditions should never happen. Maybe use assert? */
	if (size == 0ul)
		return;
	if (size > sizeof(crc))
		size = (unsigned long)sizeof(crc);

	/* The DSU's output is big-endian. We're little-endian. */
	crc = __bswap32(ctx->crc);

	/* The final result needs to be bitwise inverted. */
	crc ^= 0xffffffffu;

	/* Return the result to the caller. */
	if (size == sizeof(uint32_t) &&
	    (((uintptr_t)hash) % sizeof(uint32_t)) == 0) {
		*((uint32_t *)hash) = crc;
	} else {
		memcpy(hash, &crc, (size_t)size);
	}
}
