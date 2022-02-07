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

#ifndef APPLET_SHARED_H
#define APPLET_SHARED_H

#include <stdlib.h>
#include <stdint.h>

#ifndef APPLET_VOLATILE
#define APPLET_VOLATILE	/* Nothing */
#define	membar()	do { } while (0)
#else
#ifdef likely
#undef likely
#endif
#ifdef unlikely
#undef unlikely
#endif
#define likely(x)	__builtin_expect((x), 1)
#define unlikely(x)	__builtin_expect((x), 0)
#define	membar()	__asm __volatile("\tdsb\n" ::: "memory")
#endif	/* APPLET_VOLATILE */

struct flash_applet_descriptor {
	APPLET_VOLATILE uint32_t fad_write_idx;		/* #00 */
	APPLET_VOLATILE uint32_t fad_read_idx;		/* #04 */
	APPLET_VOLATILE uint32_t fad_target_idle;	/* #08 */
	APPLET_VOLATILE uint32_t fad_status;		/* #12 */
	APPLET_VOLATILE uint32_t fad_dest;		/* #16 */
	uint32_t fad_controller;			/* #20 */
	uint32_t fad_page_size;				/* #24 */
	uint32_t fad_md_flags;				/* #28 */
	uint32_t fad_buffer_size;			/* #32 */
	uint32_t fad_flags;				/* #36 */
	uint8_t fad_buffer[];				/* #40 */
};
#define	FLASH_APPLET_FLAGS_VERIFY	(1u << 0)

static __inline uint32_t
flash_applet_buffer_count(const struct flash_applet_descriptor *fad)
{
	uint32_t ridx, widx;

	ridx = fad->fad_read_idx;
	widx = fad->fad_write_idx;

	if (widx >= ridx)
		return widx - ridx;

	return (fad->fad_buffer_size - ridx) + widx;
}

static __inline uint32_t
flash_applet_buffer_space(const struct flash_applet_descriptor *fad,
    uint32_t needed)
{
	uint32_t space = fad->fad_buffer_size - flash_applet_buffer_count(fad);

	return (space > needed);
}

static __inline void
flash_applet_advance_read_page(struct flash_applet_descriptor *fad,
    uint32_t dest)
{
	uint32_t ridx = fad->fad_read_idx + fad->fad_page_size;

	if (ridx >= fad->fad_buffer_size)
		ridx = 0;
	fad->fad_read_idx = ridx;
	fad->fad_dest = dest;
	membar();
}

static __inline void *
flash_applet_read_ptr(const struct flash_applet_descriptor *fad)
{

	return (void *)(uintptr_t)&((fad)->fad_buffer[fad->fad_read_idx]);
}

#endif /* APPLET_SHARED_H */
