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

#ifndef MPU_COMMON_H
#define MPU_COMMON_H

#include "platform.h"

#ifdef MPU_RASR_ATTRS_Msk
struct mpu_entry {
	uint32_t me_base;
	uint32_t me_len;
	uint32_t me_attr;
};
#endif /* MPU_RASR_ATTRS_Msk */

#ifdef MPU_RLAR_EN_Msk
struct mpu_entry {
	uint32_t me_rbar;
	uint32_t me_rlar;
	uint8_t me_mair;
};

#define	MPU_ENTRY(base, end, xn, ap, outer, inner) {			       \
	.me_rbar = ((uint32_t)(uintptr_t)(base) & MPU_RBAR_BASE_Msk) |	       \
		   (((xn) << MPU_RBAR_XN_Pos) & MPU_RBAR_XN_Msk) |	       \
		   (((ap) << MPU_RBAR_AP_Pos) & MPU_RBAR_AP_Msk),	       \
	.me_rlar = ((uint32_t)(uintptr_t)(end) & MPU_RLAR_LIMIT_Msk) |	       \
		   MPU_RLAR_EN_Msk,					       \
	.me_mair = (((outer) & 0xfu) << 4) | ((inner) & 0xfu)}

#define	MPU_PERM_EXEC_OK		0u
#define	MPU_PERM_NO_EXEC		1u
#define	MPU_PERM_PRIV_READ_WRITE	0u
#define	MPU_PERM_ANY_READ_WRITE		1u
#define	MPU_PERM_PRIV_READ_ONLY		2u
#define	MPU_PERM_ANY_READ_ONLY		3u

#define	MPU_ATTR_DEVICE			0x0u
#define	MPU_ATTR_WRITE_THRU_TRANS_WA	0x1u
#define	MPU_ATTR_WRITE_THRU_TRANS_RA	0x2u
#define	MPU_ATTR_WRITE_THRU_TRANS_RWA	0x3u
#define	MPU_ATTR_NON_CACHEABLE		0x4u
#define	MPU_ATTR_WRITE_BACK_TRANS_WA	0x5u
#define	MPU_ATTR_WRITE_BACK_TRANS_RA	0x6u
#define	MPU_ATTR_WRITE_BACK_TRANS_RWA	0x7u
#define	MPU_ATTR_WRITE_THRU_NO_ALLOC	0x8u
#define	MPU_ATTR_WRITE_THRU_WA		0x9u
#define	MPU_ATTR_WRITE_THRU_RA		0xau
#define	MPU_ATTR_WRITE_THRU_RWA		0xbu
#define	MPU_ATTR_WRITE_BACK_NO_ALLOC	0xcu
#define	MPU_ATTR_WRITE_BACK_WA		0xdu
#define	MPU_ATTR_WRITE_BACK_RA		0xeu
#define	MPU_ATTR_WRITE_BACK_RWA		0xfu

#define	MPU_DEVICE_nGnRnE		0x0u
#define	MPU_DEVICE_nGnRE		0x4u
#define	MPU_DEVICE_nGRE			0x8u
#define	MPU_DEVICE_GRE			0xcu
#endif /* MPU_RLAR_EN_Msk */

extern void mpu_init(const struct mpu_entry *me, unsigned int count);
extern void mpu_enable(void);
extern unsigned int mpu_add_region(const struct mpu_entry *me);
extern void mpu_disable_region(unsigned int);
extern void mpu_enable_region(unsigned int);

#endif /* MPU_COMMON_H */
