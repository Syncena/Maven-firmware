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

#ifndef FLASH_STM_EFMI_H
#define FLASH_STM_EFMI_H

#include <stdbool.h>
#include <stdio.h>

#include "target.h"

enum flash_stm_efmi_nvm_type {
	FLASH_STM_EFMI_FLASH,
	FLASH_STM_EFMI_OTP,
	FLASH_STM_EFMI_OBYTES,
};

typedef bool (*flash_stm_efmi_dual_bank_enabled_t)(uint32_t);

#define	FLASH_STM_EFMI_MAX_OPTCR	3

struct flash_stm_efmi_desc {
	target_t ed_target;
	enum flash_stm_efmi_nvm_type ed_type;
	uint32_t ed_regbase;
	target_addr_t ed_flash_base;
	uint32_t ed_flash_size;
	uint32_t ed_max_page_size;
#define	ed_otp_blocks	ed_max_page_size
	uint32_t ed_sector_small;
	uint32_t ed_sector_med;
	uint32_t ed_sector_large;
	uint32_t ed_mass_erase_time;
	uint32_t ed_sector_erase_time;
	flash_stm_efmi_dual_bank_enabled_t ed_dual_bank;
	uint32_t ed_optcr_mask[FLASH_STM_EFMI_MAX_OPTCR];
	uint8_t ed_nopts;
	uint8_t ed_size_shift;
	uint8_t ed_instance;
};

extern void *flash_stm_efmi_attach(const struct flash_stm_efmi_desc *);
extern void flash_stm_efmi_detach(void *);
extern int flash_stm_efmi_rdp_query(void *, struct target_protection_info *);
extern int flash_stm_efmi_rdp_set(void *, struct target_protection_info *);
extern int flash_stm_efmi_rdp_clear(void *, FILE *);

extern const char stm_efmi_rdp_query_text[];

#endif /* FLASH_STM_EFMI_H */
