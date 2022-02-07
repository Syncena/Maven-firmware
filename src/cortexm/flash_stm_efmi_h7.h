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

#ifndef FLASH_STM_EFMI_H7_H
#define FLASH_STM_EFMI_H7_H

#include <stdbool.h>
#include <stdio.h>

#include "target.h"

struct flash_stm_efmi_h7_desc {
	target_t ed_target;
	uint32_t ed_regbase;
	target_addr_t ed_flash_base;
	uint32_t ed_flash_size;
	uint32_t ed_sector_size;
	uint32_t ed_max_page_size;
	uint32_t ed_mass_erase_time;
	uint32_t ed_sector_erase_time;
	bool ed_swapped;
};

extern void *flash_stm_efmi_h7_attach(const struct flash_stm_efmi_h7_desc *);
extern int flash_stm_efmi_h7_rdp_query(void *, struct target_protection_info *);
extern int flash_stm_efmi_h7_rdp_set(void *, struct target_protection_info *);
extern int flash_stm_efmi_h7_rdp_clear(void *, FILE *);

extern const char stm_efmi_h7_rdp_query_text[];

#endif /* FLASH_STM_EFMI_H7_H */
