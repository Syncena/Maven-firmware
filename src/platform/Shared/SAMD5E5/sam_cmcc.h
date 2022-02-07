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

#ifndef SAM_CMCC_H
#define SAM_CMCC_H

/*
 * The SAMD5x/E5x's CMCC is 4 KB of unified cache, line size is 16 bytes.
 * Policy is write-through, no write-allocate, 4 ways per set.
 *
 * As it's write-through, "clean" operations are not required.
 */

static __inline void
CMCC_CleanDCache_by_Addr(const void *addr, size_t len)
{

	/* Write-through Dcache. Nothing to do. */
	(void) addr;
	(void) len;
	__DSB();
	__DMB();
	__ISB();
}

/* Write-through Dcache. Use 'CMCC_CleanInvalidateDCache_by_Addr()' */
#define CMCC_InvalidateDCache_by_Addr(a,l) \
	CMCC_CleanInvalidateDCache_by_Addr(a,l)

extern void CMCC_CleanInvalidateDCache_by_Addr(const void *addr, size_t len);
extern void CMCC_Enable(void);
extern void CMCC_Disable(void);

#endif /* SAM_CMCC_H */
