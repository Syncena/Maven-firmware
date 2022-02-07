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

#include "applet-shared.h"
#include <sys/cdefs.h>

#define	STM_FPEC_REG_SR(md)		(md)
#define	 STM_FPEC_SR_BSY		(1u << 0)
#define	 STM_FPEC_SR_EOP		(1u << 5)
#define	 STM_FPEC_SR_ERROR		((1u << 2) | (1u << 4))
#define	STM_FPEC_REG_CR(md)		((md) + 4u)
#define	 STM_FPEC_CR_PG			((1u << 0) | (1u << 9))

static __always_inline void
fpec_write_cr(uint32_t hw, uint32_t md, uint32_t cr)
{

	*(volatile uint32_t *)((uintptr_t)hw + STM_FPEC_REG_CR(md)) = cr;
}

static __always_inline void
fpec_write_sr(uint32_t hw, uint32_t md, uint32_t sr)
{

	*(volatile uint32_t *)((uintptr_t)hw + STM_FPEC_REG_SR(md)) = sr;
}

static __always_inline uint32_t
fpec_read_sr(uint32_t hw, uint32_t md)
{

	return *(volatile uint32_t *)((uintptr_t)hw + STM_FPEC_REG_SR(md));
}

static __always_inline int
flash_applet_md_write_page(struct flash_applet_descriptor *fad)
{
	volatile uint16_t *dest, *dend;
	volatile uint32_t *vdest;
	uint16_t *src;
	uint32_t d, sr, *vsrc;

	d = fad->fad_dest;
	dest = (volatile uint16_t *)(uintptr_t)d;
	vdest = (volatile uint32_t *)(uintptr_t)d;
	dend = (volatile uint16_t *)(uintptr_t)(d + fad->fad_page_size);
	src = flash_applet_read_ptr(fad);
	vsrc = (uint32_t *)(uintptr_t)src;

	while (dest < dend) {
		fpec_write_cr(fad->fad_controller, fad->fad_md_flags,
		    STM_FPEC_CR_PG);
		membar();
		*dest++ = *src++;
		membar();

		do {
			sr = fpec_read_sr(fad->fad_controller,
			    fad->fad_md_flags);
		} while ((sr & STM_FPEC_SR_BSY) != 0);

		fpec_write_sr(fad->fad_controller, fad->fad_md_flags,
		    STM_FPEC_SR_EOP | STM_FPEC_SR_ERROR);
		membar();

		if (sr & STM_FPEC_SR_ERROR)
			return (int)sr;
	}

	if ((fad->fad_flags & FLASH_APPLET_FLAGS_VERIFY) != 0) {
		while (vdest < (volatile uint32_t *)(uintptr_t)dend) {
			if (*vdest++ != *vsrc++)
				return -1;
		}
	}

	flash_applet_advance_read_page(fad, (uint32_t)(uintptr_t)dend);

	return 0;
}

#include "common.c"
