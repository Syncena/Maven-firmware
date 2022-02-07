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

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "mpu_common.h"
#include "shell.h"

#ifndef MPU_RLAR_EN_Msk
#error "This doesn't look like an ARMv8M MPU ..."
#endif

static unsigned int mpu_first_free_region;
static unsigned int mpu_first_free_attr;

#define	MPU_N_ATTR	((sizeof(MPU->MAIR) / sizeof(MPU->MAIR[0])) * 4u)
#define	MPU_ATTR(n)	((uint8_t)((MPU->MAIR[(n)/4u]>>(((n)%4u)*8))&0xffu))

#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL(mpu, mpu_cmd, "Dump MPU");

static void
mpu_dump(FILE *fp)
{
	static const char *sh_str[] = {
	    "Non-shareable", "Invalid", "Outer shareable", "Inner shareable"
	};
	static const char *ap_str[] = {
	    "Priv R/W", "R/W", "Priv RO", "RO"
	};
	static const char *dm_str[] = {
	    "nGnRnE", "nGnRE", "nGRE", "GRE"
	};
	static const char *at_str[] = {
	    /* 0000 */ "Unpredictable",
	    /* 0001 */ "Write-through transient, write-allocate",
	    /* 0010 */ "Write-through transient, read-allocate",
	    /* 0011 */ "Write-through transient, read/write-allocate",
	    /* 0100 */ "Non-cacheable",
	    /* 0101 */ "Write-back transient, write-allocate",
	    /* 0110 */ "Write-back transient, read-allocate",
	    /* 0111 */ "Write-back transient, read/write-allocate",
	    /* 1000 */ "Write-through non-transient, no read/write-allocate",
	    /* 1001 */ "Write-through non-transient, write-allocate",
	    /* 1010 */ "Write-through non-transient, read-allocate",
	    /* 1011 */ "Write-through non-transient, read/write-allocate",
	    /* 1100 */ "Write-back non-transient, no read/write-allocate",
	    /* 1101 */ "Write-back non-transient, write-allocate",
	    /* 1110 */ "Write-back non-transient, read-allocate",
	    /* 1111 */ "Write-back non-transient, read/write-allocate",
	};
	uint32_t rbar, rlar, attr, aidx;
	unsigned int regions;

	regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;

	fprintf(fp, "MPU regions: %u, used %u\n", regions,
	    mpu_first_free_region);
	fprintf(fp, "MPU attributes: %u, used %u\n", MPU_N_ATTR,
	    mpu_first_free_attr);
	fprintf(fp, "MPU_CTRL 0x%" PRIx32 "\n", MPU->CTRL);

	for (unsigned int i = 0; i < regions; i++) {
		MPU->RNR = i;
		rlar = MPU->RLAR;
		if ((rlar & MPU_RLAR_EN_Msk) == 0)
			continue;
		rbar = MPU->RBAR;

		fprintf(fp, "%u: 0x%08" PRIx32 " -> 0x%08" PRIx32 "\n", i,
		    rbar & MPU_RBAR_BASE_Msk,
		    (rlar & MPU_RLAR_LIMIT_Msk) | 0x1fu);

		fprintf(fp, "   %s, %s, %sexecutable\n",
		    sh_str[(rbar & MPU_RBAR_SH_Msk) >> MPU_RBAR_SH_Pos],
		    ap_str[(rbar & MPU_RBAR_AP_Msk) >> MPU_RBAR_AP_Pos],
		    (rbar & MPU_RBAR_XN_Msk) ? "Non-" : "");

		aidx = (rlar & MPU_RLAR_AttrIndx_Msk) >> MPU_RLAR_AttrIndx_Pos;
		attr = MPU_ATTR(aidx);

		if ((attr & 0xf0u) == 0u) {
			fprintf(fp, "   Device-%s\n",
			    dm_str[(attr >> 2) & 0x3u]);
		} else {
			fprintf(fp, "   Normal memory, Outer %s\n",
			    at_str[(attr >> 4) & 0xfu]);
			fprintf(fp, "   Normal memory, Inner %s\n",
			    at_str[attr & 0x0fu]);
		}
	}
}

static void
mpu_cmd(FILE *fp, uint8_t argc, const char * const *argv)
{
	uint32_t ptr;
	uint8_t v;

	if (argc == 0) {
		mpu_dump(fp);
		return;
	}

	ptr = strtoul(argv[0], NULL, 0);
	fprintf(fp, "Will read from 0x%08" PRIx32 " ... ", ptr);
	fflush(fp);
	rtos_task_sleep(100);

	__asm __volatile("" ::: "memory");
	v = *((volatile uint8_t *)(uintptr_t)ptr);
	__asm __volatile("" ::: "memory");

	fprintf(fp, "Success. Byte 0x%02u\n", (unsigned int)v);
}
#endif /* (RELEASE_BUILD == 0) */

unsigned int
mpu_add_region(const struct mpu_entry *me)
{
	unsigned int aidx;
	uint32_t rlar;

#if (RELEASE_BUILD == 0)
	unsigned int regions;

	regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
	assert(mpu_first_free_region < regions);
#endif

	for (aidx = 0; aidx < mpu_first_free_attr; aidx++) {
		if (MPU_ATTR(aidx) == me->me_mair)
			break;
	}

	assert(aidx < MPU_N_ATTR);
	if (aidx == mpu_first_free_attr) {
		ARM_MPU_SetMemAttr(aidx, me->me_mair);
		mpu_first_free_attr++;
	}

	rlar = me->me_rlar & ~MPU_RLAR_AttrIndx_Msk;
	rlar |= (aidx << MPU_RLAR_AttrIndx_Pos) & MPU_RLAR_AttrIndx_Msk;

	ARM_MPU_SetRegion(mpu_first_free_region, me->me_rbar, rlar);

	return mpu_first_free_region++;
}

void
mpu_disable_region(unsigned int region)
{

	MPU->RNR = region;
	MPU->RLAR &= ~MPU_RLAR_EN_Msk;
}

void
mpu_enable_region(unsigned int region)
{

	MPU->RNR = region;
	MPU->RLAR |= MPU_RLAR_EN_Msk;
}

void
mpu_init(const struct mpu_entry *me, unsigned int count)
{
	unsigned int i, regions;

	regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
	assert(count <= regions);

	/* Ensure MPU is off */
	ARM_MPU_Disable();

	mpu_first_free_region = 0;
	mpu_first_free_attr = 0;

	/* Configure the MPU as requested by the SOC. */
	for (i = 0; i < count; i++, me++)
		(void) mpu_add_region(me);

	for (; i < regions; i++)
		ARM_MPU_ClrRegion(i);
}

void
mpu_enable(void)
{

	ARM_MPU_Enable(0);

#if (RELEASE_BUILD == 0)
	SHELL_CMD_ADD(mpu);
#endif
}
