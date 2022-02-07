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
#include <stdlib.h>
#include <strings.h>

#include "platform.h"
#include "mpu_common.h"
#include "shell.h"

#ifndef MPU_RASR_ATTRS_Msk
#error "This doesn't look like an ARMv7M MPU..."
#endif

static unsigned int mpu_first_free;

#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL(mpu, mpu_cmd, "Dump MPU");

static void
mpu_dump(FILE *fp)
{
	unsigned int regions;
	uint32_t rasr, srd, rsize;

	regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;

	fprintf(fp, "MPU regions: %u, used %u\n", regions, mpu_first_free);
	fprintf(fp, "MPU_CTRL 0x%" PRIx32 "\n", MPU->CTRL);
	for (unsigned int i = 0; i < regions; i++) {
		MPU->RNR = i;
		rasr = MPU->RASR;
		if ((rasr & MPU_RASR_ENABLE_Msk) == 0)
			continue;
		fprintf(fp, "%u: RBAR 0x%08" PRIx32 ", RASR 0x%08" PRIx32 "\n",
		    i, MPU->RBAR & MPU_RBAR_ADDR_Msk, rasr);
		srd = (rasr & MPU_RASR_SRD_Msk) >> MPU_RASR_SRD_Pos;
		rsize = 1lu << (1+((rasr & MPU_RASR_SIZE_Msk)>>MPU_RASR_SIZE_Pos));
		for (uint32_t a = MPU->RBAR & MPU_RBAR_ADDR_Msk;
		    srd != 0; a += (rsize / 8u), srd >>= 1) {
			if (srd & 1u) {
				fprintf(fp, "   Sub-region 0x%08" PRIx32
				    "-0x%08" PRIx32 " disabled\n", a,
				    a + (rsize / 8u) - 1);
			}
		}
		fprintf(fp, "   SRD%02" PRIx32 " B%" PRIu32 " C%" PRIu32
		    " S%" PRIu32 " TEX%" PRIu32 " AP%" PRIu32 " XN%" PRIu32
		    " Size 0x%" PRIx32 "\n",
		    (rasr & MPU_RASR_SRD_Msk) >> MPU_RASR_SRD_Pos,
		    (rasr & MPU_RASR_C_Msk) >> MPU_RASR_C_Pos,
		    (rasr & MPU_RASR_B_Msk) >> MPU_RASR_B_Pos,
		    (rasr & MPU_RASR_S_Msk) >> MPU_RASR_S_Pos,
		    (rasr & MPU_RASR_TEX_Msk) >> MPU_RASR_TEX_Pos,
		    (rasr & MPU_RASR_AP_Msk) >> MPU_RASR_AP_Pos,
		    (rasr & MPU_RASR_XN_Msk) >> MPU_RASR_XN_Pos,
		    rsize);
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

static uint32_t
mpu_calc_size(uint32_t size)
{

	assert(size != 0);
	assert((size & (size - 1u)) == 0);

	return ((uint32_t)ffs(size) - 2u) << MPU_RASR_SIZE_Pos;
}

unsigned int
mpu_add_region(const struct mpu_entry *me)
{
#if (RELEASE_BUILD == 0)
	assert(mpu_first_free < ((MPU->TYPE & MPU_TYPE_DREGION_Msk) >>
	    MPU_TYPE_DREGION_Pos));
#endif

	assert((me->me_base & ~MPU_RBAR_ADDR_Msk) == 0);
	assert((me->me_base & (me->me_len - 1)) == 0);
	assert(me->me_len >= 32);

	ARM_MPU_SetRegionEx(mpu_first_free, me->me_base,
	    mpu_calc_size(me->me_len) | me->me_attr);

	return mpu_first_free++;
}

void
mpu_disable_region(unsigned int region)
{

	MPU->RNR = region;
	MPU->RASR &= ~(1u << MPU_RASR_ENABLE_Pos);
}

void
mpu_enable_region(unsigned int region)
{

	MPU->RNR = region;
	MPU->RASR |= 1u << MPU_RASR_ENABLE_Pos;
}

void
mpu_init(const struct mpu_entry *me, unsigned int count)
{
	unsigned int i, regions;

	regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
	assert(count <= regions);

	/* Ensure MPU is off */
	ARM_MPU_Disable();

	mpu_first_free = 0;

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
