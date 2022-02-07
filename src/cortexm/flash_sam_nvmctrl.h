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

#ifndef FLASH_SAM_NVMCTRL_H
#define FLASH_SAM_NVMCTRL_H

#include "target.h"
#include "microchip_dsu.h"

#define	FLASH_SAM_NVMCTRL_TYPE_0	0	/* SAMD5x */
#define	FLASH_SAM_NVMCTRL_TYPE_1	1	/* SAM0, except saml1x */
#define	FLASH_SAM_NVMCTRL_TYPE_2	2	/* SAML1x */
#define	FLASH_SAM_NVMCTRL_TYPE(x,ws)	((x) | ((ws) << 8))

#define	FLASH_SAM_NVMCTRL_FLAG_EEP	(1u << 0)	/* Has [Smart]EEPROM */
#define	FLASH_SAM_NVMCTRL_FLAG_RWWEEP	(1u << 1)	/* Has RWWEEProm */

struct flash_sam_nvmctrl_attach_args {
	target_t aa_target;
	target_addr_t aa_nvmctrl_regs;
	target_addr_t aa_flash_base;
	target_addr_t aa_rwweep_base;
	target_addr_t aa_seep_base;
	target_addr_t aa_user_base;
	uint32_t aa_user_size;
	int (*aa_user_erase)(void *, uint8_t *, uint32_t);
	int (*aa_user_check)(void *, uint8_t *, uint32_t, uint8_t *);
	void *aa_user_cookie;
	uint16_t aa_type_ws;
	uint16_t aa_flags;
};

extern void *flash_sam_nvmctrl_attach(
		const struct flash_sam_nvmctrl_attach_args *);

extern int flash_sam_nvmctrl0_prot_set(void *, struct target_protection_info *,
		microchip_dsu_protection_t);
extern int flash_sam_nvmctrl1_prot_set(void *, struct target_protection_info *,
		microchip_dsu_protection_t);
extern int flash_sam_nvmctrl2_prot_set(void *, struct target_protection_info *,
		microchip_dsu_protection_t);

#endif /* FLASH_SAM_NVMCTRL_H */
