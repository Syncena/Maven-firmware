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
#include "applet-sam-efc.h"
#include <sys/cdefs.h>

#define	SAM_EFC_FCR			0x04u
#define	SAM_EFC_FSR			0x08u
#define	 SAM_EFC_FSR_FRDY		(1u << 0)
#define	 SAM_EFC_FSR_ERRS		0x0eu
#define	 SAM_EFC_FSR_BUSY(fsr)		(((fsr) & SAM_EFC_FSR_FRDY) == 0)

static __always_inline void
efc_write_cmd(uint32_t hw, uint32_t cmd)
{

	*(volatile uint32_t *)((uintptr_t)hw + SAM_EFC_FCR) = cmd;
	membar();
}

static __always_inline uint32_t
efc_read_status(uint32_t hw)
{

	return *(volatile uint32_t *)((uintptr_t)hw + SAM_EFC_FSR);
}

static __always_inline int
flash_applet_md_write_page(struct flash_applet_descriptor *fad)
{
	volatile uint32_t *dest, *vdest, *dend;
	uint32_t d, *src, *vsrc, cmd, fsr;

	d = fad->fad_dest;
	vdest = dest = (volatile uint32_t *)(uintptr_t)d;
	dend = (volatile uint32_t *)(uintptr_t)(d + fad->fad_page_size);
	vsrc = src = flash_applet_read_ptr(fad);

	/* Load the page buffer */
	while (likely(dest < dend)) {
		*dest++ = *src++;
		membar();
	}

	cmd = SAM_EFC_APPLET_MAKE_CMD(fad->fad_md_flags, d);
	efc_write_cmd(fad->fad_controller, cmd);

	do {
		fsr = efc_read_status(fad->fad_controller);
	} while (SAM_EFC_FSR_BUSY(fsr));

	if ((fsr & SAM_EFC_FSR_ERRS) == 0) {
		if ((fad->fad_flags & FLASH_APPLET_FLAGS_VERIFY) != 0) {
			while (vdest < dend) {
				if (*vdest++ != *vsrc++)
					return -1;
			}
		}

		flash_applet_advance_read_page(fad, (uint32_t)(uintptr_t)dend);
	}

	return (fsr & SAM_EFC_FSR_ERRS) ? (int)fsr : 0;
}

#include "common.c"
