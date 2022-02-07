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

#define	FMC_REG_CMD			0x00u
#define	 FMC_CMD_WRITE_PROG		10u
#define	FMC_REG_STARTA			0x10u
#define	 FMC_ADDRESS(a)			(((a) >> 4) & 0x3ffffu)
#define	FMC_REG_DATAW(n)		(0x80u + ((n) * 4u))
#define	FMC_REG_INT_STATUS		0xfe0u
#define	FMC_REG_INT_CLR_STATUS		0xfe8u
#define	 FMC_INT_FAIL			(1u << 0)
#define	 FMC_INT_ERR			(1u << 1)
#define	 FMC_INT_DONE			(1u << 2)
#define	 FMC_INT_ECC_ERR		(1u << 3)
#define	 FMC_INT_ANY			(FMC_INT_FAIL | FMC_INT_ERR | \
					 FMC_INT_DONE | FMC_INT_ECC_ERR)
#define	 FMC_INT_ERRORS			(FMC_INT_FAIL | FMC_INT_ERR | \
					 FMC_INT_ECC_ERR)

#define	FMC_FLASH_BYTES_PER_WORD	0x10u

static __always_inline void
fmc_write_cmd(uint32_t hw, uint32_t cmd)
{

	*(volatile uint32_t *)((uintptr_t)hw + FMC_REG_CMD) = cmd;
	membar();
}

static __always_inline void
fmc_write_starta(uint32_t hw, uint32_t addr)
{

	*(volatile uint32_t *)((uintptr_t)hw + FMC_REG_STARTA) =
	    FMC_ADDRESS(addr);
	membar();
}

static __always_inline void
fmc_write_dataw(uint32_t hw, unsigned int which, uint32_t value)
{

	*(volatile uint32_t *)((uintptr_t)hw + FMC_REG_DATAW(which)) = value;
	membar();
}

static __always_inline uint32_t
fmc_read_int_status(uint32_t hw)
{

	return *(volatile uint32_t *)((uintptr_t)hw + FMC_REG_INT_STATUS);
}

static __always_inline void
fmc_write_int_status_clr(uint32_t hw, uint32_t st)
{

	*(volatile uint32_t *)((uintptr_t)hw + FMC_REG_INT_CLR_STATUS) = st;
	membar();
}

static __always_inline int
flash_applet_md_write_page(struct flash_applet_descriptor *fad)
{
	uint32_t dest, dend, st;
	volatile uint32_t *vdest;
	uint32_t *src, *vsrc;

	dest = fad->fad_dest;
	vdest = (volatile uint32_t *)(uintptr_t)dest;
	dend = dest + fad->fad_page_size;
	vsrc = src = flash_applet_read_ptr(fad);
	st = FMC_INT_FAIL;

	while (likely(dest < dend)) {
		/* Write the start address of the next word. */
		fmc_write_starta(fad->fad_controller, dest);
		dest += FMC_FLASH_BYTES_PER_WORD;

		/* Fill the word buffer. */
		for (unsigned int i = 0;
		    i < (FMC_FLASH_BYTES_PER_WORD / sizeof(uint32_t)); i++) {
			fmc_write_dataw(fad->fad_controller, i, *src++);
		}
		membar();

		fmc_write_cmd(fad->fad_controller, FMC_CMD_WRITE_PROG);

		do {
			st = fmc_read_int_status(fad->fad_controller);
		} while ((st & FMC_INT_ANY) == 0);

		fmc_write_int_status_clr(fad->fad_controller, st);
		if ((st & FMC_INT_ERRORS) != 0)
			break;
	}

	if ((st & FMC_INT_ERRORS) == 0) {
		if ((fad->fad_flags & FLASH_APPLET_FLAGS_VERIFY) != 0) {
			while (vdest < (volatile uint32_t *)(uintptr_t)dend) {
				if (*vdest++ != *vsrc++)
					return -1;
			}
		}
		flash_applet_advance_read_page(fad, dend);
	}

	return (st & FMC_INT_ERRORS) ? (int)st : 0;
}

#include "common.c"
