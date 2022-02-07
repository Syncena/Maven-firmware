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


#define	STM_EFMI_SR			0x0cu
#define	 STM_EFMI_SR_EOP		(1u << 0)
#define	 STM_EFMI_SR_OPERR		(1u << 1)
#define	 STM_EFMI_SR_WRPERR		(1u << 4)
#define	 STM_EFMI_SR_PGAERR		(1u << 5)
#define	 STM_EFMI_SR_PGPERR		(1u << 6)
#define	 STM_EFMI_SR_PGSERR		(1u << 7)
#define	 STM_EFMI_SR_RDERR		(1u << 8)
#define	 STM_EFMI_SR_ALL_ERRS		(STM_EFMI_SR_OPERR | \
					 STM_EFMI_SR_WRPERR | \
					 STM_EFMI_SR_PGAERR | \
					 STM_EFMI_SR_PGPERR | \
					 STM_EFMI_SR_PGSERR)
#define	 STM_EFMI_SR_BSY		(1u << 16)
#define	STM_EFMI_CR			0x10u
#define	 STM_EFMI_CR_PG			(1u << 0)
#define	 STM_EFMI_CR_PSIZE_x8		(0u << 8)
#define	 STM_EFMI_CR_PSIZE_x16		(1u << 8)
#define	 STM_EFMI_CR_PSIZE_x32		(2u << 8)

static __always_inline void
efmi_write_cr(uint32_t hw, uint32_t cr)
{

	*(volatile uint32_t *)((uintptr_t)hw + STM_EFMI_CR) = cr;
}

static __always_inline void
efmi_write_sr(uint32_t hw, uint32_t sr)
{

	*(volatile uint32_t *)((uintptr_t)hw + STM_EFMI_SR) = sr;
}

static __always_inline uint32_t
efmi_read_sr(uint32_t hw)
{

	return *(volatile uint32_t *)((uintptr_t)hw + STM_EFMI_SR);
}

static __always_inline int
flash_applet_md_write_page(struct flash_applet_descriptor *fad)
{
	uintptr_t src, dest, dend;
	volatile uint32_t *vdest;
	uint32_t d, sr, *vsrc;

	d = fad->fad_dest;
	dest = (uintptr_t)d;
	vdest = (volatile uint32_t *)dest;
	dend = (uintptr_t)(d + fad->fad_page_size);
	src = (uintptr_t)flash_applet_read_ptr(fad);
	vsrc = (uint32_t *)src;

	efmi_write_cr(fad->fad_controller, fad->fad_md_flags | STM_EFMI_CR_PG);
	membar();

	while (dest < dend) {
		if (fad->fad_md_flags == STM_EFMI_CR_PSIZE_x8) {
			*(volatile uint8_t *)dest = *(uint8_t *)src;
			dest += sizeof(uint8_t);
			src += sizeof(uint8_t);
		} else
		if (fad->fad_md_flags == STM_EFMI_CR_PSIZE_x16) {
			*(volatile uint16_t *)dest = *(uint16_t *)src;
			dest += sizeof(uint16_t);
			src += sizeof(uint16_t);
		} else {
			*(volatile uint32_t *)dest = *(uint32_t *)src;
			dest += sizeof(uint32_t);
			src += sizeof(uint32_t);
		}

		membar();

		do {
			sr = efmi_read_sr(fad->fad_controller);
		} while ((sr & STM_EFMI_SR_BSY) != 0);

		if (sr & STM_EFMI_SR_ALL_ERRS) {
			efmi_write_sr(fad->fad_controller,
			    STM_EFMI_SR_ALL_ERRS | STM_EFMI_SR_EOP);
			return (int)sr;
		}
	}

	if ((fad->fad_flags & FLASH_APPLET_FLAGS_VERIFY) != 0) {
		while (vdest < (volatile uint32_t *)dend) {
			if (*vdest++ != *vsrc++)
				return -1;
		}
	}

	flash_applet_advance_read_page(fad, (uint32_t)dend);

	efmi_write_cr(fad->fad_controller, fad->fad_md_flags);
	membar();

	return 0;
}

#include "common.c"
