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

#define	NVMCTRL_REG_CMD		0x04u		/* 16-bits */
#define	 NVMCTRL_CMD_WP		0xa503u
#define	NVMCTRL_REG_INTFLAG	0x10u		/* 16-bits */
#define	 NVMCTRL_INTFLAG_DONE	(1u << 0)
#define	 NVMCTRL_INTFLAG_ADDRE	(1u << 1)
#define	 NVMCTRL_INTFLAG_PROGE	(1u << 2)
#define	 NVMCTRL_INTFLAG_LOCKE	(1u << 3)
#define	 NVMCTRL_INTFLAG_ECCSE	(1u << 4)
#define	 NVMCTRL_INTFLAG_ECCDE	(1u << 5)
#define	 NVMCTRL_INTFLAG_NVME	(1u << 6)
#define	 NVMCTRL_INTFLAG_ERROR	(NVMCTRL_INTFLAG_ADDRE | \
				 NVMCTRL_INTFLAG_PROGE | \
				 NVMCTRL_INTFLAG_LOCKE | \
				 NVMCTRL_INTFLAG_ECCSE | \
				 NVMCTRL_INTFLAG_ECCDE | \
				 NVMCTRL_INTFLAG_NVME)
#define	NVMCTRL_REG_STATUS	0x12u		/* 16-bits */
#define	 NVMCTRL_STATUS_READY	(1u << 0)
#define	NVMCTRL_REG_ADDR	0x1cu		/* 32-bits */
#define	 NVMCTRL_ADDR(a)	((a) & 0x00ffffffu)

static __always_inline void
nvm_write_addr(uint32_t hw, uint32_t addr)
{

	*(volatile uint32_t *)((uintptr_t)hw + NVMCTRL_REG_ADDR) =
	    NVMCTRL_ADDR(addr);
}

static __always_inline void
nvm_write_cmd(uint32_t hw, uint16_t cmd)
{

	*(volatile uint16_t *)((uintptr_t)hw + NVMCTRL_REG_CMD) = cmd;
}

static __always_inline uint16_t
nvm_read_status(uint32_t hw)
{

	return *(volatile uint16_t *)((uintptr_t)hw + NVMCTRL_REG_STATUS);
}

static __always_inline uint16_t
nvm_read_intflag(uint32_t hw)
{

	return *(volatile uint16_t *)((uintptr_t)hw + NVMCTRL_REG_INTFLAG);
}

static __always_inline void
nvm_write_intflag(uint32_t hw, uint8_t flag)
{

	*(volatile uint16_t *)((uintptr_t)hw + NVMCTRL_REG_INTFLAG) = flag;
}

static __always_inline void
nvm_wait_ready(uint32_t hw)
{

	while ((nvm_read_status(hw) & NVMCTRL_STATUS_READY) == 0)
		;
}

static __always_inline int
flash_applet_md_write_page(struct flash_applet_descriptor *fad)
{
	volatile uint32_t *dest, *vdest, *dend;
	uint32_t *src, *vsrc;
	uint32_t d, off;
	uint16_t flag;

	nvm_write_intflag(fad->fad_controller, NVMCTRL_INTFLAG_ERROR);

	d = fad->fad_dest;
	vdest = dest = (volatile uint32_t *)(uintptr_t)d;
	dend = (volatile uint32_t *)(uintptr_t)(d + fad->fad_page_size);
	vsrc = src = flash_applet_read_ptr(fad);

	/* Load the page buffer. */
	nvm_wait_ready(fad->fad_controller);
	while (dest < dend) {
		*dest++ = *src++;
		membar();
	}

	nvm_wait_ready(fad->fad_controller);
	nvm_write_addr(fad->fad_controller, d);
	nvm_wait_ready(fad->fad_controller);
	nvm_write_cmd(fad->fad_controller, NVMCTRL_CMD_WP);
	membar();

	do {
		flag = nvm_read_intflag(fad->fad_controller);
	} while ((flag & NVMCTRL_INTFLAG_DONE) == 0);

	nvm_write_intflag(fad->fad_controller, flag);
	membar();

	if (flag & NVMCTRL_INTFLAG_ERROR)
		return (int)flag;

	if ((fad->fad_flags & FLASH_APPLET_FLAGS_VERIFY) != 0) {
		while (vdest < dend) {
			if (*vdest++ != *vsrc++)
				return -1;
		}
	}

	flash_applet_advance_read_page(fad, (uint32_t)(uintptr_t)dend);

	return 0;
}

#include "common.c"
