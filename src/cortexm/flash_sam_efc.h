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

#ifndef FLASH_SAM_EFC_H
#define FLASH_SAM_EFC_H

#include "target.h"

/* No SAM3/4 EFC device has more than 2 planes */
#define	SAM_EFC_MAX_PLANES		2

/* Planes are contiguous on most devices, but there's always an exception */
#define	SAM_EFC_FLASH_BASE_CONTIG	0u
#define	SAM_EFC_FLASH_BASE_INVALID	((uint32_t)(~0u))

extern void *flash_sam_efc_attach(target_t t, const target_addr_t *flash_base,
		target_addr_t efc_regs, uint32_t efc_offset,
		uint32_t wait_states, int use_ewp, int add_flash);

extern void flash_sam_efc_detach(void *cookie);

/* Query/Set/Clear the GPNVM bits */
extern int flash_sam_efc_gpnvm(void *, uint32_t op);
#define	FLASH_SAM_EFC_GPNVM_QUERY	0
#define	FLASH_SAM_EFC_GPNVM_SET(b)	(((b) << 8) | 1)
#define	FLASH_SAM_EFC_GPNVM_CLEAR(b)	(((b) << 8) | 2)

/* Returns the number of Flash planes on this device */
extern int flash_sam_efc_nplanes(void *);

#endif /* FLASH_SAM_EFC_H */
