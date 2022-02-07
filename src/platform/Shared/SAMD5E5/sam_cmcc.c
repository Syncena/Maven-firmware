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

#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "sam_cmcc.h"

/*
 * The SAM's CMCC is 4 KB of unified cache, line size is 16 bytes.
 * Policy is write-through, no write-allocate, 4 ways per set.
 *
 * As it's write-through, "clean" operations are not required.
 */

#define	SAM_CACHE_SIZE		4096u
#define	SAM_CACHE_LINE_SIZE	16u
#define	SAM_CACHE_WAYS		4u

#define	CACHE_INDEX_MASK	((SAM_CACHE_SIZE / SAM_CACHE_WAYS) - 1u)
#define	CACHE_INVALL_THRESHOLD	(SAM_CACHE_SIZE / SAM_CACHE_LINE_SIZE)

void
CMCC_CleanInvalidateDCache_by_Addr(const void *addr, size_t len)
{
	uint32_t idx, a = (uint32_t)(uintptr_t)addr;
	uint32_t l = (uint32_t)len;

	__DSB();
	__DMB();
	__ISB();

	/* Do nothing if the cache is disabled. */
	if ((CMCC->SR.reg & CMCC_SR_CSTS) == 0)
		return;

	/* Also do nothing if caller-supplied length is zero. */
	if (l == 0)
		return;

	/* Quicker to invalidate the entire cache for large buffers. */
	if (l >= CACHE_INVALL_THRESHOLD) {
		CMCC->CTRL.reg = 0;
		while ((CMCC->SR.reg & CMCC_SR_CSTS) != 0)
			;
		CMCC->MAINT0.reg = CMCC_MAINT0_INVALL;
		__DSB();
		__ISB();
		CMCC->CTRL.reg = CMCC_CTRL_CEN;
		return;
	}

	/* Otherwise, round length/address to cache line boundaries. */
	l += a & (SAM_CACHE_LINE_SIZE - 1u);
	l += SAM_CACHE_LINE_SIZE - 1u;
	l &= ~(SAM_CACHE_LINE_SIZE - 1u);
	a &= ~(SAM_CACHE_LINE_SIZE - 1u);

	/* Cache-ops must be performed with the cache disabled. (!!) */
	CMCC->CTRL.reg = 0;
	while ((CMCC->SR.reg & CMCC_SR_CSTS) != 0)
		;

	/* For each cache line... */
	while (l) {
		/* Invalidate the ways for this index. */
		idx = a & CACHE_INDEX_MASK;
		for (unsigned int way = 0; way < SAM_CACHE_WAYS; way++) {
			CMCC->MAINT1.reg = CMCC_MAINT1_INDEX(idx) |
			    CMCC_MAINT1_WAY(way);
		}

		a += SAM_CACHE_LINE_SIZE;
		l -= SAM_CACHE_LINE_SIZE;
	}

	/* Sync and re-enable the cache. */
	__ISB();
	CMCC->CTRL.reg = CMCC_CTRL_CEN;
	__DSB();
	__ISB();
}

void
CMCC_Enable(void)
{

	CMCC_Disable();

	/* Use all CMCC RAM for cache. */
	CMCC->CFG.reg = CMCC_CFG_CSIZESW_CONF_CSIZE_4KB;

	/* Enable. */
	CMCC->CTRL.reg = CMCC_CTRL_CEN;

	/* Don't return until we're sure it's enabled. */
	while ((CMCC->SR.reg & CMCC_SR_CSTS) == 0)
		;
}

void
CMCC_Disable(void)
{

	__DSB();
	__DMB();
	__ISB();

	/* If the cache is enabled, disable it. */
	if ((CMCC->SR.reg & CMCC_SR_CSTS) != 0) {
		CMCC->CTRL.reg = 0;
		while ((CMCC->SR.reg & CMCC_SR_CSTS) != 0)
			;
	}

	/* Ensure it is invalidated. */
	CMCC->MAINT0.reg = CMCC_MAINT0_INVALL;
	__DSB();
	__ISB();
}
