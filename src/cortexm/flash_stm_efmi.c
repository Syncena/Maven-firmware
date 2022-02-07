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
#include <stdlib.h>
#include <stdio.h>

#include "flash_stm_efmi.h"
#include "flash_applet.h"
#include "platform.h"
#include "timer.h"
#include "rtos.h"
#include "twiddle.h"
#include "zone_alloc.h"

/*
 * Driver for the Embedded Flash Memory Interface found on some STM32F4 parts.
 *
 * The Flash controller is extremely inefficient from a performance
 * perspective. A Flash Applet is pretty much essential to get decent
 * programming speed.
 *
 * Maybe one day ST will implement a better Flash controller...
 */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

struct stm_efmi_state {
	struct target_flash es_tf;
	uint32_t es_psize;
	uint32_t es_optcr;
	uint32_t es_new_optcr[FLASH_STM_EFMI_MAX_OPTCR];
	u_int es_flags;
	void *es_applet;
	struct flash_stm_efmi_desc es_desc;
};
#define	es_target		es_desc.ed_target
#define	es_type			es_desc.ed_type
#define	es_reg_base		es_desc.ed_regbase
#define	es_flash_base		es_desc.ed_flash_base
#define	es_flash_size		es_desc.ed_flash_size
#define	es_max_page_size	es_desc.ed_max_page_size
#define	es_sector_small		es_desc.ed_sector_small
#define	es_sector_med		es_desc.ed_sector_med
#define	es_sector_large		es_desc.ed_sector_large
#define	es_mass_erase_time	es_desc.ed_mass_erase_time
#define	es_sector_erase_time	es_desc.ed_sector_erase_time
#define	es_dual_bank		es_desc.ed_dual_bank
#define	es_optcr_mask		es_desc.ed_optcr_mask
#define	es_nopts		es_desc.ed_nopts
#define	es_size_shift		es_desc.ed_size_shift
#define	es_instance		es_desc.ed_instance

#define	STM_EFMI_FLAGS_UNLOCKED		(1u << 0)
#define	STM_EFMI_FLAGS_FLUSH_OPTS	(1u << 1)

#define	STM_EFMI_IS_UNLOCKED(es)	\
		(((es)->es_flags & STM_EFMI_FLAGS_UNLOCKED) != 0)

/*
 * Lock/Unlock state is shared between driver instances,
 * so it must be global. Sucks, but no other way.
 */
static int stm_efmi_unlocked;

/*
 * For some devices, the other 'option bytes' instances are recorded
 * here so that the first instance can access their stashed values.
 */
static struct stm_efmi_state *stm_efmi_option[FLASH_STM_EFMI_MAX_OPTCR];

/* EFMI Registers; offset from es_reg_base */
#define	STM_EFMI_ACR			0x00u
#define	 STM_EFMI_ACR_LATENCY_MASK	0xfu
#define	 STM_EFMI_ACR_LATENCY_SHIFT	0
#define	 STM_EFMI_ACR_PRFTEN		(1u << 8)
#define	 STM_EFMI_ACR_ICEN		(1u << 9)
#define	 STM_EFMI_ACR_DCEN		(1u << 10)
#define	 STM_EFMI_ACR_ICRST		(1u << 11)
#define	 STM_EFMI_ACR_DCRST		(1u << 12)
#define	STM_EFMI_KEYR			0x04u
#define	 STM_EFMI_KEYR_KEY1		0x45670123u
#define	 STM_EFMI_KEYR_KEY2		0xcdef89abu
#define	STM_EFMI_OPTKEYR		0x08u
#define	 STM_EFMI_OPTKEYR_KEY1		0x08192a3bu
#define	 STM_EFMI_OPTKEYR_KEY2		0x4c5d6e7fu
#define	STM_EFMI_SR			0x0cu
#define	 STM_EFMI_SR_EOP		(1u << 0)
#define	 STM_EFMI_SR_OPERR		(1u << 1)
#define	 STM_EFMI_SR_WRPERR		(1u << 4)
#define	 STM_EFMI_SR_PGAERR		(1u << 5)
#define	 STM_EFMI_SR_PGPERR		(1u << 6)
#define	 STM_EFMI_SR_PGSERR		(1u << 7)
#define	 STM_EFMI_SR_RDERR		(1u << 8)
#define	 STM_EFMI_SR_ALL_ERRS		(STM_EFMI_SR_OPERR | \
					 STM_EFMI_SR_WRPERR | \
					 STM_EFMI_SR_PGAERR | \
					 STM_EFMI_SR_PGPERR | \
					 STM_EFMI_SR_PGSERR)
#define	 STM_EFMI_SR_BSY		(1u << 16)
#define	STM_EFMI_CR			0x10u
#define	 STM_EFMI_CR_PG			(1u << 0)
#define	 STM_EFMI_CR_SER		(1u << 1)
#define	 STM_EFMI_CR_MER		(1u << 2)
#define	 STM_EFMI_CR_SNB_MASK		0x1fu
#define	 STM_EFMI_CR_SNB_SHIFT		3
#define	 STM_EFMI_CR_PSIZE_x8		(0u << 8)
#define	 STM_EFMI_CR_PSIZE_x16		(1u << 8)
#define	 STM_EFMI_CR_PSIZE_x32		(2u << 8)
#define	 STM_EFMI_CR_PSIZE_x64		(3u << 8)
#define	 STM_EFMI_CR_PSIZE_MASK		(3u << 8)
#define	 STM_EFMI_CR_STRT		(1u << 16)
#define	 STM_EFMI_CR_EOPIE		(1u << 24)
#define	 STM_EFMI_CR_ERRIE		(1u << 25)
#define	 STM_EFMI_CR_LOCK		(1u << 31)
#define	STM_EFMI_OPTCR			0x14u
#define	 STM_EFMI_OPTCR_OPTLOCK		(1u << 0)
#define	 STM_EFMI_OPTCR_OPTSTRT		(1u << 1)
#define	 STM_EFMI_OPTCR_RDP_MASK	0xff00u
#define	 STM_EFMI_OPTCR_RDP(x)		((x) & 0xff00u)
#define	 STM_EFMI_OPTCR_RDP_MAGIC(l)	(((l) == 0) ? 0xaa00 : \
					  ((l) == 2) ? 0xcc00u : 0x0000u)
#define	 STM_EFMI_OPTCR_RDP_LEVEL(x)	((STM_EFMI_OPTCR_RDP(x)==0xaa00u)?0:\
					 ((STM_EFMI_OPTCR_RDP(x)==0xcc00u)?2:1))
#define	STM_EFMI_OPTCR1			0x18u
#define	STM_EFMI_OPTCR2			0x1cu


#define	STM_EFMI_DUAL_BANK_ENABLED(es)	((es)->es_dual_bank && \
					 (es)->es_dual_bank(es->es_optcr))
#define	STM_EFMI_SSZ_SHIFT(es)		((es)->es_size_shift * \
					 STM_EFMI_DUAL_BANK_ENABLED(es))
#define	STM_EFMI_SECTOR_SIZE_SMALL(es)	((es)->es_sector_small >> \
					 STM_EFMI_SSZ_SHIFT(es))
#define	STM_EFMI_SECTOR_SIZE_MED(es)	((es)->es_sector_med >> \
					 STM_EFMI_SSZ_SHIFT(es))
#define	STM_EFMI_SECTOR_SIZE_LARGE(es)	((es)->es_sector_large >> \
					 STM_EFMI_SSZ_SHIFT(es))
#define	STM_EFMI_PAGE_SIZE(es)		((es)->es_max_page_size << \
					 STM_EFMI_SSZ_SHIFT(es))

static const uint8_t stm_efmi_flash_applet[] = {
#include "flash-applets/applet-stm-efmi.applet"
};

/*
 * Convenience functions for read/write of EFMI registers
 */
static int
stm_efmi_read_reg(struct stm_efmi_state *es, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(es->es_target, es->es_reg_base + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(es->es_target);

	return rv;
}

static int
stm_efmi_write_reg(struct stm_efmi_state *es, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(es->es_target, es->es_reg_base + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(es->es_target);

	return rv;
}

static uint32_t
stm_efmi_wait_bsy(struct stm_efmi_state *es, FILE *os)
{
	timer_timeout_t to;
	uint32_t sr = 0, err, erase_time;
	int rv;

	if (os != NULL)
		erase_time = es->es_mass_erase_time;
	else
		erase_time = es->es_sector_erase_time;
	timer_timeout_start(&to, erase_time);

	do {
		rv = stm_efmi_read_reg(es, STM_EFMI_SR, &sr);
		if (rv < 0) {
			rv = TARGET_LINK_ERROR(es->es_target);
			DBFPRINTF("DP err 0x%08x\n", rv);
			break;
		}

		if ((sr & STM_EFMI_SR_BSY) != 0) {
			if (os != NULL)
				twiddle(os);
			rtos_task_sleep(1);
		}
	} while ((sr & STM_EFMI_SR_BSY) != 0 && !timer_timeout_expired(&to));

	DBFPRINTF("Waited for %" PRIu32 " mS, rv %d, SR 0x%" PRIx32 "\n",
	    erase_time - timer_timeout_remaining(&to), rv, sr);

	err = STM_EFMI_SR_ALL_ERRS;

	if (rv == 0 && (sr & STM_EFMI_SR_BSY) != 0) {
		DBFPRINTF("Timeout waiting for BSY\n");
		return err;
	}

	if (rv < 0)
		return err;

	if ((err & sr) != 0 &&
	    stm_efmi_write_reg(es, STM_EFMI_SR, STM_EFMI_SR_EOP | err) < 0) {
		DBFPRINTF("Failed to clear SR\n");
		return ~0u;
	}

	if (sr != 0)
		DBFPRINTF("Non-zero SR: 0x%08" PRIx32 "\n", sr);

	return sr & err;
}

static int
stm_efmi_unlock(struct stm_efmi_state *es)
{
	uint32_t cr;

	if (STM_EFMI_IS_UNLOCKED(es))
		return 0;

	/*
	 * The FLASH_CR.PSIZE field depends on the target's Vcc.
	 * We set it up here rather than at attach time because
	 * it's far more likely that power_target_vcc is stable
	 * by this point.
	 */
	if (power_target_vcc < F16(2.1)) {
		DBFPRINTF("PSIZE = x8\n");
		es->es_psize = STM_EFMI_CR_PSIZE_x8;
	} else
	if (power_target_vcc < F16(2.7)) {
		DBFPRINTF("PSIZE = x16\n");
		es->es_psize = STM_EFMI_CR_PSIZE_x16;
	} else {
		DBFPRINTF("PSIZE = x32\n");
		es->es_psize = STM_EFMI_CR_PSIZE_x32;
	}

	if (stm_efmi_read_reg(es, STM_EFMI_CR, &cr) < 0) {
		DBFPRINTF("Failed to read CR\n");
		return -1;
	}

	DBFPRINTF("CR: 0x%08" PRIx32 "\n");

	if (stm_efmi_write_reg(es, STM_EFMI_SR,
	    STM_EFMI_SR_EOP | STM_EFMI_SR_ALL_ERRS) < 0) {
		DBFPRINTF("Failed to clear SR bits.\n");
	}

	if (cr & STM_EFMI_CR_LOCK) {
		/* Perform an unlock sequence */
		if (stm_efmi_write_reg(es, STM_EFMI_KEYR,
		    STM_EFMI_KEYR_KEY1) < 0 ||
		    stm_efmi_write_reg(es, STM_EFMI_KEYR,
		    STM_EFMI_KEYR_KEY2) < 0) {
			DBFPRINTF("keys failed\n");
			return -1;
		}

		if (stm_efmi_wait_bsy(es, NULL) != 0) {
			DBFPRINTF("BSY failed\n");
			return -1;
		}

		if (stm_efmi_read_reg(es, STM_EFMI_CR, &cr) < 0) {
			DBFPRINTF("CR read-back failed\n");
			return -1;
		}

		if (cr & STM_EFMI_CR_LOCK) {
			/* Flash remained locked. We're stuffed. */
			DBFPRINTF("Flash remained locked\n");
			return -1;
		}
	}

	stm_efmi_unlocked++;
	es->es_flags |= STM_EFMI_FLAGS_UNLOCKED;
	DBFPRINTF("Unlocked\n");

	return 0;
}

static void
stm_efmi_lock(struct stm_efmi_state *es)
{
	uint32_t cr;

	if (!STM_EFMI_IS_UNLOCKED(es))
		return;

	es->es_flags &= ~STM_EFMI_FLAGS_UNLOCKED;

	if (stm_efmi_unlocked && --stm_efmi_unlocked) {
		DBFPRINTF("unlocked count not yet zero\n");
		return;
	}

	(void) TARGET_LINK_ERROR(es->es_target);
	if (stm_efmi_read_reg(es, STM_EFMI_CR, &cr) < 0)
		return;
	cr |= STM_EFMI_CR_LOCK;
	(void) stm_efmi_write_reg(es, STM_EFMI_CR, cr);
	DBFPRINTF("locked\n");
}

static int
stm_efmi_sector_desc_flash(target_flash_t tf, target_addr_t *poffset,
    struct target_flash_sector *fs)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	uint32_t offset = *poffset;
	uint32_t number, base;

	if (STM_EFMI_DUAL_BANK_ENABLED(es) &&
	    (offset >= (es->es_flash_size / 2u))) {
		offset -= es->es_flash_size / 2u;
		base = es->es_flash_base + (es->es_flash_size / 2u);
		number = 12u;
		DBFPRINTF("dual bank enabled\n");
	} else {
		base = es->es_flash_base;
		number = 0u;
	}

	DBFPRINTF("start base 0x%08" PRIx32 ", offset 0x%08" PRIx32
	    ", number %" PRIu32 "\n", base, offset, number);

	if (offset < STM_EFMI_SECTOR_SIZE_MED(es)) {
		fs->fs_size = STM_EFMI_SECTOR_SIZE_SMALL(es);
		fs->fs_pages = STM_EFMI_SECTOR_SIZE_SMALL(es) /
		    STM_EFMI_PAGE_SIZE(es);
		*poffset = offset & (STM_EFMI_SECTOR_SIZE_SMALL(es) - 1);
		base += offset & ~(STM_EFMI_SECTOR_SIZE_SMALL(es) - 1);
		number += offset / STM_EFMI_SECTOR_SIZE_SMALL(es);
		DBFPRINTF("small sector\n");
	} else
	if (offset < STM_EFMI_SECTOR_SIZE_LARGE(es)) {
		fs->fs_size = STM_EFMI_SECTOR_SIZE_MED(es);
		fs->fs_pages = STM_EFMI_SECTOR_SIZE_MED(es) /
		    STM_EFMI_PAGE_SIZE(es);
		*poffset = offset & (STM_EFMI_SECTOR_SIZE_MED(es) - 1);
		base += offset & ~(STM_EFMI_SECTOR_SIZE_MED(es) - 1);
		number += 4u;
		DBFPRINTF("medium sector\n");
	} else {
		fs->fs_size = STM_EFMI_SECTOR_SIZE_LARGE(es);
		fs->fs_pages = STM_EFMI_SECTOR_SIZE_LARGE(es) /
		    STM_EFMI_PAGE_SIZE(es);
		*poffset = offset & (STM_EFMI_SECTOR_SIZE_LARGE(es) - 1);
		base += offset & ~(STM_EFMI_SECTOR_SIZE_LARGE(es) - 1);
		number += 4u + (offset / STM_EFMI_SECTOR_SIZE_LARGE(es));
		DBFPRINTF("large sector\n");
	}

	fs->fs_number = number;
	fs->fs_cookie = (uintptr_t)base;

	DBFPRINTF("final base 0x%08" PRIx32 ", offset 0x%08" PRIxTADDR
	    ", number %" PRIu32 "\n", base, *poffset, number);

	return 0;
}

static int
stm_efmi_sector_desc_otp(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;

	fs->fs_size = es->es_flash_size / es->es_max_page_size;
	fs->fs_pages = 1;
	fs->fs_number = *offset / fs->fs_size;
	*offset &= fs->fs_size - 1;

	fs->fs_cookie = es->es_flash_base;
	if (fs->fs_number == (es->es_flash_size / es->es_max_page_size))
		fs->fs_cookie += es->es_flash_size;

	return 0;
}

static int
stm_efmi_erase_sector(target_flash_t tf,
    const struct target_flash_sector *fs)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	uint32_t cr;

	if (stm_efmi_unlock(es) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	DBFPRINTF("Sector %" PRIu32 " @ 0x%08" PRIx32 "\n", fs->fs_number,
	    (uint32_t)fs->fs_cookie);

	if (es->es_type == FLASH_STM_EFMI_FLASH) {
		cr = es->es_psize | STM_EFMI_CR_SER |
		    (fs->fs_number << STM_EFMI_CR_SNB_SHIFT);

		if (stm_efmi_write_reg(es, STM_EFMI_CR, cr) < 0 ||
		    stm_efmi_write_reg(es, STM_EFMI_CR,
		    cr | STM_EFMI_CR_STRT) < 0) {
			DBFPRINTF("Error\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		cr = stm_efmi_wait_bsy(es, NULL);

		(void) stm_efmi_write_reg(es, STM_EFMI_CR, es->es_psize);

		DBFPRINTF("%s (cr = 0x%" PRIx32 ")\n",
		    (cr == 0) ? "Done" : "Failed", cr);
	} else {
		DBFPRINTF("skipped for non-Flash\n");
		cr = 0;
	}

	return (cr == 0) ? TARGET_FLASH_RESULT_OK : TARGET_FLASH_RESULT_ERROR;
}

static int
stm_efmi_write_page(target_flash_t tf, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	target_addr_t dest;
	const void *src;
	unsigned int len, pagelen, inc;
	uint32_t sr;
	int rv;

	DBFPRINTF("sector %" PRIu32 ", page %" PRIu32 "\n", fs->fs_number,
	    page);

	if (stm_efmi_unlock(es) < 0) {
		DBFPRINTF("failed to unlock Flash\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	pagelen = (es->es_type == FLASH_STM_EFMI_OTP) ? fs->fs_size :
	    STM_EFMI_PAGE_SIZE(es);

	src = tf->tf_buffer;
	dest = (target_addr_t)(fs->fs_cookie + (page * pagelen));
	DBFPRINTF("addr 0x%" PRIxTADDR ", pagelen 0x%x\n", dest, pagelen);

	if (es->es_type == FLASH_STM_EFMI_FLASH && es->es_applet != NULL) {
		rv = flash_applet_write_page(tf, dest, pagelen,
		    es->es_reg_base, es->es_psize, es->es_applet);
	} else {
		if (es->es_psize == STM_EFMI_CR_PSIZE_x8)
			inc = 1;
		else
		if (es->es_psize == STM_EFMI_CR_PSIZE_x16)
			inc = 2;
		else
			inc = 4;

		if (stm_efmi_write_reg(es, STM_EFMI_CR,
		    es->es_psize | STM_EFMI_CR_PG) < 0) {
			DBFPRINTF("failed to set PG\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

		rv = 0;
		for (len = 0; rv >= 0 && len < pagelen; len += inc) {
			if (inc == 1)
				rv = TARGET_MEM_WRITE(es->es_target, dest, 1, src);
			else if (inc == 2) {
				rv = TARGET_REG_WRITE16(es->es_target, dest,
				    *(const uint16_t *)src);
			} else {
				rv = TARGET_REG_WRITE32(es->es_target, dest,
				    *(const uint32_t *)src);
			}

			if (rv < 0)
				break;

			src = (const void *)((uintptr_t)src + inc);
			dest += inc;

			sr = stm_efmi_wait_bsy(es, NULL);
			if (sr != 0) {
				DBFPRINTF("sr 0x%" PRIx32, sr);
				rv = -1;
				break;
			}
		}

		rv = (rv < 0) ? TARGET_FLASH_RESULT_ERROR :
		    TARGET_FLASH_RESULT_OK;

		(void) stm_efmi_write_reg(es, STM_EFMI_CR, es->es_psize);
	}

	DBFPRINTF("%s\n", (rv != TARGET_FLASH_RESULT_ERROR) ? "Done" :
	    "Failed");

	return rv;
}

static int
stm_efmi_sector_desc_option(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *fs)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;

	fs->fs_size = es->es_max_page_size;
	fs->fs_pages = 1;
	fs->fs_number = 0;
	fs->fs_cookie = 0;
	*offset &= fs->fs_size - 1;

	return 0;
}

static int
stm_efmi_write_page_option(target_flash_t tf,
    const struct target_flash_sector *fs, uint32_t page)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	uint32_t optcr;
	uint16_t *p;
	u_int i, idx;

	(void) fs;
	(void) page;

	assert(page == 0);

	/* Get a pointer to the option bytes to be writen. */
	p = (uint16_t *)(void *)tf->tf_buffer;
	idx = es->es_instance;

	for (i = 0; i < es->es_flash_size; i += 16u, p += 8, idx++) {
		/* Merge the two 16-bit halves */
		optcr = (uint32_t)p[0];
		optcr |= ((uint32_t)p[4]) << 16;

		/* Sanitise and update the cached value. */
		es->es_new_optcr[idx] = optcr & es->es_optcr_mask[idx];
	}

	/* Ensure the new value will be written in stm_efmi_finish() */
	stm_efmi_option[0]->es_flags |= STM_EFMI_FLAGS_FLUSH_OPTS;

	return TARGET_FLASH_RESULT_VERIFIED;	/* XXX: Not strictly true... */
}

static int
stm_efmi_opt_unlock(struct stm_efmi_state *es)
{
	uint32_t ocr;

	DBFPRINTF("unlocking\n");

	if (stm_efmi_read_reg(es, STM_EFMI_OPTCR, &ocr) < 0) {
		DBFPRINTF("Failed to read OPTCR\n");
		return -1;
	}

	if ((ocr & STM_EFMI_OPTCR_OPTLOCK) == 0) {
		DBFPRINTF("already unlocked\n");
		return 0;
	}

	if (stm_efmi_write_reg(es, STM_EFMI_OPTKEYR,
	                       STM_EFMI_OPTKEYR_KEY1) < 0 ||
	    stm_efmi_write_reg(es, STM_EFMI_OPTKEYR,
	                       STM_EFMI_OPTKEYR_KEY2) < 0) {
		DBFPRINTF("OPTCR unlock failed.\n");
		return -1;
	}

	if (stm_efmi_wait_bsy(es, NULL) != 0) {
		DBFPRINTF("BSY after OPTCR unlock failed.\n");
		return -1;
	}

	if (stm_efmi_read_reg(es, STM_EFMI_OPTCR, &ocr) < 0) {
		DBFPRINTF("failed to re-read OPTCR after unlock\n");
		return -1;
	}

	if (ocr & STM_EFMI_OPTCR_OPTLOCK) {
		/* OPTCR remained locked. We're stuffed. */
		DBFPRINTF("OPTCR still locked\n");
		return -1;
	}

	return 0;
}

static int
stm_efmi_opt_lock(struct stm_efmi_state *es)
{
	uint32_t ocr;

	if (stm_efmi_read_reg(es, STM_EFMI_OPTCR, &ocr) < 0) {
		DBFPRINTF("Failed to read OPTCR\n");
		return -1;
	}

	if ((ocr & STM_EFMI_OPTCR_OPTLOCK) != 0)
		return 0;

	if (stm_efmi_write_reg(es, STM_EFMI_OPTCR,
	    STM_EFMI_OPTCR_OPTLOCK) < 0) {
		DBFPRINTF("Failed to set OPTLOCK\n");
		return -1;
	}

	return 0;
}

static int
stm_efmi_flush_options(struct stm_efmi_state *es, FILE *os)
{
	u_int i;
	int rv;

	assert(es->es_type == FLASH_STM_EFMI_OBYTES);
	assert(es->es_instance == 0);

	DBFPRINTF("Flushing options\n");

	if (stm_efmi_opt_unlock(es) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	es->es_optcr = es->es_new_optcr[0];

	for (i = 1; i < FLASH_STM_EFMI_MAX_OPTCR; i++) {
		if (stm_efmi_option[i] == NULL)
			break;

		/* Synchronise option values. */
		es->es_new_optcr[i] = stm_efmi_option[i]->es_new_optcr[i];
		stm_efmi_option[i]->es_new_optcr[0] = es->es_new_optcr[0];
		stm_efmi_option[i]->es_optcr = es->es_optcr;
	}

	DBFPRINTF("will flush %u option%s\n", (unsigned int)es->es_nopts,
	    (es->es_nopts > 1) ? "s" : "");

	rv = TARGET_FLASH_RESULT_OK;
	switch (es->es_nopts) {
	case 3:
		DBFPRINTF("OPTCR2 %08" PRIx32 "\n", es->es_new_optcr[2]);
		rv = stm_efmi_write_reg(es, STM_EFMI_OPTCR2,
		    es->es_new_optcr[2]);
		if (rv < 0) {
			DBFPRINTF("OPTCR2 WRITE FAILED\n");
			rv = TARGET_FLASH_RESULT_ERROR;
			break;
		}
		/*FALLTHROUGH*/
	case 2:
		DBFPRINTF("OPTCR1 %08" PRIx32 "\n", es->es_new_optcr[1]);
		rv = stm_efmi_write_reg(es, STM_EFMI_OPTCR1,
		    es->es_new_optcr[1]);
		if (rv < 0) {
			DBFPRINTF("OPTCR1 WRITE FAILED\n");
			rv = TARGET_FLASH_RESULT_ERROR;
			break;
		}
		/*FALLTHROUGH*/
	default:
		break;
	}

	if (rv == TARGET_FLASH_RESULT_ERROR)
		goto unlock;

	DBFPRINTF("OPTCR %08" PRIx32 "\n", es->es_optcr);

	rv = stm_efmi_write_reg(es, STM_EFMI_OPTCR, es->es_optcr);
	if (rv < 0) {
		DBFPRINTF("OPTCR WRITE FAILED\n");
		rv = TARGET_FLASH_RESULT_ERROR;
		goto unlock;
	}

	rv = stm_efmi_write_reg(es, STM_EFMI_OPTCR,
	    es->es_optcr | STM_EFMI_OPTCR_OPTSTRT);
	if (rv < 0) {
		DBFPRINTF("OPTCR WRITE START FAILED\n");
		rv = TARGET_FLASH_RESULT_ERROR;
		goto unlock;
	}

	if (stm_efmi_wait_bsy(es, os) != 0) {
		DBFPRINTF("WRITE FAILED (BSY)\n");
		rv = TARGET_FLASH_RESULT_ERROR;
		/*
		 * Changing certain option bits causes a
		 * JTAG/SWD disconnection event.
		 */
	}

 unlock:
	stm_efmi_opt_lock(es);

	return rv;
}

static int
stm_efmi_finish(target_flash_t tf)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	int rv = TARGET_FLASH_RESULT_OK;

	if (es->es_type == FLASH_STM_EFMI_FLASH && es->es_applet != NULL)
		rv = flash_applet_finish(es->es_applet);

	if (es->es_type == FLASH_STM_EFMI_OBYTES &&
	    (es->es_flags & STM_EFMI_FLAGS_FLUSH_OPTS) != 0) {
		es->es_flags &= ~STM_EFMI_FLAGS_FLUSH_OPTS;
		rv = stm_efmi_flush_options(es, NULL);
	}

	stm_efmi_lock(es);

	return rv;
}

static void
stm_efmi_free(target_flash_t tf)
{
	struct stm_efmi_state *es = (struct stm_efmi_state *)tf;
	u_int i;

	if (es->es_type == FLASH_STM_EFMI_FLASH && es->es_applet != NULL)
		flash_applet_finish(es->es_applet);

	for (i = 0; i < FLASH_STM_EFMI_MAX_OPTCR; i++) {
		if (es == stm_efmi_option[i]) {
			stm_efmi_option[i] = NULL;
			break;
		}
	}

	zone_free(es);
}

static int
flash_stm_efmi_attach_impl(struct stm_efmi_state *es)
{
	unsigned int flags = 0;
	const char *desc = NULL;
	uint32_t optcr, flash_size;

	es->es_flags = 0;
	flash_size = es->es_flash_size;

	(void) TARGET_LINK_ERROR(es->es_target);
	if (stm_efmi_read_reg(es, STM_EFMI_OPTCR, &optcr) < 0) {
		DBFPRINTF("failed to read OPTCR\n");
		return -1;
	}
	optcr &= ~(STM_EFMI_OPTCR_OPTLOCK | STM_EFMI_OPTCR_OPTSTRT);

	switch (es->es_type) {
	case FLASH_STM_EFMI_FLASH:
		es->es_tf.tf_sector_desc = stm_efmi_sector_desc_flash;
		es->es_tf.tf_write_page = stm_efmi_write_page;
		break;

	case FLASH_STM_EFMI_OTP:
		desc = "[one-time programmable]";
		es->es_tf.tf_sector_desc = stm_efmi_sector_desc_otp;
		es->es_tf.tf_write_page = stm_efmi_write_page;
		flash_size += es->es_flash_size / es->es_max_page_size;
		break;

	case FLASH_STM_EFMI_OBYTES:
		assert(es->es_instance < FLASH_STM_EFMI_MAX_OPTCR);
		flags = TARGET_MEM_OPTIONS;
		es->es_tf.tf_sector_desc = stm_efmi_sector_desc_option;
		es->es_tf.tf_write_page = stm_efmi_write_page_option;
		break;

	default:
		return -1;
	}

	es->es_tf.tf_max_page_size = es->es_max_page_size;
	es->es_tf.tf_erase_sector = stm_efmi_erase_sector;
	es->es_tf.tf_size = es->es_flash_size;
	es->es_tf.tf_erase_byte = 0xffu;
	es->es_tf.tf_finish = stm_efmi_finish;
	es->es_tf.tf_free = stm_efmi_free;
	es->es_optcr = optcr;

	DBFPRINTF("OPTCR %08" PRIx32 "\n", optcr);

	if (es->es_instance == 0) {
		es->es_new_optcr[0] = optcr;
	} else {
		if (stm_efmi_read_reg(es, (es->es_instance == 1) ?
		    STM_EFMI_OPTCR1 : STM_EFMI_OPTCR2, &optcr) < 0) {
			DBFPRINTF("failed to read OPTCR%u\n",
			    (unsigned int)es->es_instance);
			return -1;
		}
		es->es_new_optcr[es->es_instance] = optcr;
		DBFPRINTF("OPTCR%u %08" PRIx32 "\n",
		    (unsigned int)es->es_instance, optcr);
	}

	DBFPRINTF("base 0x%08" PRIxTADDR ", size 0x%08" PRIx32 "\n",
	    es->es_flash_base, es->es_flash_size);

	if (es->es_type == FLASH_STM_EFMI_FLASH) {
		DBFPRINTF("Small 0x%" PRIx32 ", Med 0x%" PRIx32 ", Large "
		    "0x%" PRIx32 "\n", STM_EFMI_SECTOR_SIZE_SMALL(es),
		    STM_EFMI_SECTOR_SIZE_MED(es),
		    STM_EFMI_SECTOR_SIZE_LARGE(es));
	}

	if (target_add_flash(es->es_target, &es->es_tf, es->es_flash_base,
	    flash_size, desc, flags) < 0){
		DBFPRINTF("failed to add flash @ %08" PRIxTADDR "\n",
		    es->es_flash_base);
		return -1;
	}

	if (es->es_type == FLASH_STM_EFMI_OBYTES) {
		stm_efmi_option[es->es_instance] = es;

		if (es->es_instance == 0) {
			es->es_target->t_flags |=
			    (TARGET_FLAG_SUPPORTS_PROTECTION |
			     TARGET_FLAG_SUPPORTS_PERM_PROT);

			if (STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr)  > 0)
				es->es_target->t_flags |= TARGET_FLAG_IS_LOCKED;
		}
	}

	return 0;
}

void *
flash_stm_efmi_attach(const struct flash_stm_efmi_desc *ed)
{
	struct stm_efmi_state *es;
	void *flash_applet;

	DBFPRINTF("nt %u, reg 0x%08" PRIx32 ", size 0x%08" PRIx32 "\n",
	    (unsigned int)ed->ed_type, ed->ed_regbase, ed->ed_flash_size);

	if (ed->ed_type == FLASH_STM_EFMI_FLASH) {
		struct flash_applet_params fp;
		assert(ed->ed_instance == 0);

		fp.fp_applet = stm_efmi_flash_applet;
		fp.fp_applet_size = sizeof(stm_efmi_flash_applet);
		fp.fp_applet_stack_size = 0;
		fp.fp_max_page_size = ed->ed_max_page_size;
		flash_applet = flash_applet_attach(ed->ed_target, &fp);
	} else {
		flash_applet = NULL;
	}

	if ((es = zone_calloc(1, sizeof(*es))) == NULL) {
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		return NULL;
	}

	es->es_applet = flash_applet;
	es->es_desc = *ed;

	if (flash_stm_efmi_attach_impl(es) < 0) {
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		zone_free(es);
		return NULL;
	}

	return es;
}

void
flash_stm_efmi_detach(void *arg)
{

	stm_efmi_free(arg);
}

const char stm_efmi_rdp_query_text[] =
"Possible level values:\n"
"\t0  Device is unlocked; RDP == 0xAA.\n"
"\t1  Read-out protection of memories (Debug features limited)\n"
"\t2  Chip protection; RDP == 0xCC. (Debug & boot from RAM disabled).\n"
"Note that changing the level will require the target to be power-cycled\n"
"for the change to take effect.\n";

int
flash_stm_efmi_rdp_query(void *arg, struct target_protection_info *pi)
{
	struct stm_efmi_state *es = arg;

	pi->pi_max_level = 2;
	pi->pi_level = STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr);
	pi->pi_text = stm_efmi_rdp_query_text;

	return 0;
}

int
flash_stm_efmi_rdp_set(void *arg, struct target_protection_info *pi)
{
	struct stm_efmi_state *es = arg;

	if (pi->pi_level > 2)
		return -1;

	/* Short circuit if there's no change */
	if (STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr) == pi->pi_level)
		return TARGET_PI_UNCHANGED;

	if (STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr) > pi->pi_level)
		return TARGET_PI_USE_UNLOCK;

	if (pi->pi_level == 2 && pi->pi_permanent == 0)
		return TARGET_PI_NEED_PERM_FLAG;

	if (stm_efmi_unlock(es) < 0)
		return -1;

	es->es_new_optcr[0] &= ~STM_EFMI_OPTCR_RDP_MASK;
	es->es_new_optcr[0] |= STM_EFMI_OPTCR_RDP_MAGIC(pi->pi_level);
	es->es_flags |= STM_EFMI_FLAGS_FLUSH_OPTS;

	stm_efmi_flush_options(es, NULL);

	return TARGET_PI_OK_POWER_CYCLE;
}

int
flash_stm_efmi_rdp_clear(void *arg, FILE *os)
{
	struct stm_efmi_state *es = arg;

	/* Short circuit if there's no change */
	if (STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr) == 0)
		return TARGET_PI_UNCHANGED;

	if (STM_EFMI_OPTCR_RDP_LEVEL(es->es_optcr) == 2) {
		fprintf(os, "RDP level 2 cannot be unlocked.\n");
		return -1;
	}

	if (stm_efmi_unlock(es) < 0)
		return -1;

	/* Reset all OPTCR values to default. */
	es->es_new_optcr[0] = es->es_optcr_mask[0];
	if (stm_efmi_option[1] != NULL) {
		stm_efmi_option[1]->es_new_optcr[1] =
		    stm_efmi_option[1]->es_optcr_mask[1];
	}
	if (stm_efmi_option[2] != NULL) {
		stm_efmi_option[2]->es_new_optcr[2] =
		    stm_efmi_option[2]->es_optcr_mask[2];
	}

	es->es_new_optcr[0] &= ~STM_EFMI_OPTCR_RDP_MASK;
	es->es_new_optcr[0] |= STM_EFMI_OPTCR_RDP_MAGIC(0);
	es->es_flags |= STM_EFMI_FLAGS_FLUSH_OPTS;

	fputs("Performing full-chip erase. This could take up to 32 seconds, "
	    "depending\non target Vcc and Flash size... ", os);

	stm_efmi_flush_options(es, os);

	fprintf(os, "\bDone.\n");

	return TARGET_PI_OK_POWER_CYCLE;
}
