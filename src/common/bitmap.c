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
#include <string.h>

#include "bitmap.h"

#define BITMAP_BIT_TO_ELEMENT_IDX(bit)	((bit) / BITMAP_BITS_PER_ELEMENT)
#define	BITMAP_BIT_MASK(bit)		(1u << ((bit)%BITMAP_BITS_PER_ELEMENT))

void
bitmap_init(bitmap_t bm, unsigned int nbits)
{

	assert(nbits > 0);

	memset(bm, 0, sizeof(bitmap_element_t) *
	    BITMAP_BITS_TO_ELEMENTS(nbits));
}

void
bitmap_set_state(bitmap_t bm, unsigned int bit, bool state)
{
	bitmap_element_t mask, v;
	unsigned int idx;

	mask = BITMAP_BIT_MASK(bit);
	idx = BITMAP_BIT_TO_ELEMENT_IDX(bit);

	v = bm[idx] & ~mask;

	if (state)
		v |= mask;

	bm[idx] = v;
}

bool
bitmap_get_state(const bitmap_t bm, unsigned int bit)
{
	bitmap_element_t mask, v;

	mask = BITMAP_BIT_MASK(bit);
	v = bm[BITMAP_BIT_TO_ELEMENT_IDX(bit)] & mask;

	return v != 0;
}

unsigned int
bitmap_find_first(const bitmap_t bm, unsigned int nbits, unsigned int bit,
    bool state)
{

	while (bit < nbits) {
		/* Fetch the element containing the next bit. */
		bitmap_element_t v = bm[BITMAP_BIT_TO_ELEMENT_IDX(bit)];

		/*
		 * If we're looking for unset bits, flip the state of
		 * all bits in the element.
		 */
		if (state == false)
			v ^= (bitmap_element_t)(~0u);

		/*
		 * Get the starting bit down to bit#0. Also ensures any
		 * unused upper bits are unset.
		 */
		v >>= bit % BITMAP_BITS_PER_ELEMENT;

		/*
		 * 'v' will be non-zero if there is at least one bit of
		 * interest.
		 */
		if (v) {
			int f;

			/* Find the first interesting bit. */
			f = ffs(v);
			assert(f > 0);	/* It *must* exist. */

			/* Convert into actual bit number. */
			bit = bit + (unsigned int)f;

			/* All being well, this will never exceed nbits. */
			assert(bit <= nbits);

			/*
			 * Note: we return bit number + 1, which happens
			 * to work just fine with the result from the
			 * above ffs() call.
			 */
			return bit;
		}

		/*
		 * We get here if there is nothing of interest in this
		 * element. Advance to the next one.
		 */
		bit += BITMAP_BITS_PER_ELEMENT;
		bit &= ~(BITMAP_BITS_PER_ELEMENT - 1u);
	}

	/*
	 * We get here if the bitmap contains no further bits with the
	 * required state.
	 */

	return 0;
}
