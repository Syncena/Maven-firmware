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
#include <stdint.h>

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

static int
sam_aes_setup(const unsigned char *key, int keylen, int num_rounds,
    symmetric_key *skey)
{
	uint32_t ctrla, v;
	unsigned int i;
	int rv;

	(void) key;
	(void) keylen;
	(void) num_rounds;
	(void) skey;

	assert(keylen == 16 || keylen == 24 || keylen == 32);

	ctrla = AES_CTRLA_ENABLE | AES_CTRLA_AESMODE_CBC |
	    AES_CTRLA_CIPHER_DEC | AES_CTRLA_STARTMODE_AUTO;

	switch (keylen) {
	case 16:
		assert(num_rounds == 10);
		ctrla |= AES_CTRLA_KEYSIZE_128BIT;
		break;
	case 24:
		assert(num_rounds == 12);
		ctrla |= AES_CTRLA_KEYSIZE_192BIT;
		break;
	case 32:
		assert(num_rounds == 14);
		ctrla |= AES_CTRLA_KEYSIZE_256BIT;
		break;
	default:
		assert(0);
		return CRYPT_INVALID_KEYSIZE;
		break;
	}

	/* Enable AES clock */
	sam_periph_clock_enable(ID_AES);

	/* SWRST and enable hardware */
	AES->CTRLA.reg = AES_CTRLA_SWRST;
	while ((AES->CTRLA.reg & AES_CTRLA_SWRST) != 0)
		rtos_yield();

	/* Handle random seed, and enable countermeasures */
	rv = sam_trng_getentropy(&v, sizeof(v));
	if (rv == 0) {
		/* Got a seed. Enable all countermeasures. */
		ctrla |= AES_CTRLA_CTYPE_Msk;
		AES->CTRLA.reg = ctrla;
		AES->RANDSEED.reg = v;
	} else {
		AES->CTRLA.reg = ctrla;
	}

	/* Write key. */
	for (i = 0; i < (keylen / sizeof(uint32_t)); i++) {
		if (((uintptr_t)key % sizeof(uint32_t)) != 0)
			memcpy(&v, key, sizeof(v));
		else
			v = *((const uint32_t *)(uintptr_t)key);

		AES->KEYWORD[i].reg = v;
		key += sizeof(uint32_t);
	}

	AES->CTRLA.reg = ctrla | AES_CTRLA_KEYGEN_LAST;

	skey->data = NULL;

	return CRYPT_OK;
}

static void
sam_aes_done(symmetric_key *skey)
{

	/* SWRST hardware. */
	AES->CTRLA.reg = AES_CTRLA_SWRST;

	/* Disable AES clock */
	sam_periph_clock_disable(ID_AES);

	skey->data = NULL;
}

static int
sam_aes_test(void)
{

	return CRYPT_NOP;
}

static int
sam_aes_keysize(int *keysize)
{

	assert(keysize != NULL);
	assert(*keysize >= 16);

	if (*keysize < 24)
		*keysize = 16;
	else
	if (*keysize < 32)
		*keysize = 24;
	else
		*keysize = 32;

	return CRYPT_OK;
}

static int
sam_aes_accel_decrypt(const unsigned char *ct, unsigned char *pt,
    unsigned long blocks, unsigned char *iv, symmetric_key *skey)
{
	uint32_t aligned[4];
	uint32_t *p;

	if (skey->data == NULL) {
		skey->data = (void *)(uintptr_t)iv;

		/* Write IV to hardware */
		if (((uintptr_t)iv % sizeof(uint32_t)) != 0) {
			memcpy(aligned, iv, sizeof(aligned));
			p = aligned;
		} else {
			p = (uint32_t *)(uintptr_t)iv;
		}

		AES->INTVECTV[0].reg = *p++;
		AES->INTVECTV[1].reg = *p++;
		AES->INTVECTV[2].reg = *p++;
		AES->INTVECTV[3].reg = *p;

		/* Set CTRLB.NEWMSG */
		AES->CTRLB.reg = AES_CTRLB_NEWMSG;
	}

	while (blocks--) {
		/* Copy ciphertext to input data registers */
		if (((uintptr_t)ct % sizeof(uint32_t)) != 0) {
			memcpy(aligned, ct, sizeof(aligned));
			p = aligned;
		} else {
			p = (uint32_t *)(uintptr_t)ct;
		}

		AES->INDATA.reg = *p++;
		AES->INDATA.reg = *p++;
		AES->INDATA.reg = *p++;
		AES->INDATA.reg = *p;

		/* Wait for completion */
		while (AES->INTFLAG.bit.ENCCMP == 0)
			rtos_yield();

		/* Copy output data registers to plaintext */
		if (((uintptr_t)ct % sizeof(uint32_t)) != 0) {
			aligned[0] = AES->INDATA.reg;
			aligned[1] = AES->INDATA.reg;
			aligned[2] = AES->INDATA.reg;
			aligned[3] = AES->INDATA.reg;
			memcpy(pt, aligned, sizeof(aligned));
		} else {
			p = (uint32_t *)(uintptr_t)pt;
			p[0] = AES->INDATA.reg;
			p[1] = AES->INDATA.reg;
			p[2] = AES->INDATA.reg;
			p[3] = AES->INDATA.reg;
		}

		ct += 16;
		pt += 16;
	}

	return CRYPT_OK;
}

/* Override the 'aes_desc' structure provided by tomcrypt. */
const struct ltc_cipher_descriptor aes_desc = {
	.name = "aes",
	.ID = 6,
	.min_key_length = 16,
	.max_key_length = 32,
	.block_length = 16,
	.default_rounds = 0,
	.setup = sam_aes_setup,
	.ecb_encrypt = NULL,
	.ecb_decrypt = NULL,
	.test = sam_aes_test,
	.done = sam_aes_done,
	.keysize = sam_aes_keysize,
	.accel_ecb_encrypt = NULL,
	.accel_ecb_decrypt = NULL,
	.accel_cbc_encrypt = NULL,
	.accel_cbc_decrypt = sam_aes_accel_decrypt,
	.accel_ctr_encrypt = NULL,
	.accel_lrw_encrypt = NULL,
	.accel_lrw_decrypt = NULL,
	.accel_ccm_memory = NULL,
	.accel_gcm_memory = NULL,
	.omac_memory = NULL,
	.xcbc_memory = NULL,
	.f9_memory = NULL,
	.accel_xts_encrypt = NULL,
	.accel_xts_decrypt = NULL
};
