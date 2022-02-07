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

#include "flash_stm_efmi_h7.h"
#include "flash_applet.h"
#include "platform.h"
#include "timer.h"
#include "rtos.h"
#include "twiddle.h"
#include "zone_alloc.h"

/*
 * Driver for the Embedded Flash Memory Interface found on STM32H7 parts.
 */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

struct stm_efmi_h7_state;

struct stm_efmi_h7_target_flash {
	struct target_flash tf_tf;
	struct stm_efmi_h7_state *tf_es;
	target_addr_t tf_flash_base;
	uint32_t tf_flash_size;
	uint32_t tf_reg_base;
	uint32_t tf_psize;
};

struct stm_efmi_h7_state {
	target_t es_target;
	u_int es_flags;
	u_int es_ref_cnt;
	void *es_applet;
	struct flash_stm_efmi_h7_desc es_desc;
	uint32_t es_max_page_size;
	uint32_t es_sector_size;
	uint32_t es_mass_erase_time;
	uint32_t es_sector_erase_time;
	struct stm_efmi_h7_target_flash es_tf[2];
};

#define	STM_EFMI_H7_BANK1(tf)		((tf) == &((tf)->tf_es->es_tf[0]))
#define	STM_EFMI_H7_FLAGS_UNLOCKED(tf)	(1u << (STM_EFMI_H7_BANK1(tf) ? 0 : 1))
#define	STM_EFMI_H7_FLAGS_FLUSH_OPTS	(1u << 2)

#define	STM_EFMI_H7_IS_UNLOCKED(tf)	\
		(((tf)->tf_es->es_flags & STM_EFMI_H7_FLAGS_UNLOCKED(tf)) != 0)

/*
 * Lock/Unlock state is shared between driver instances,
 * so it must be global. Sucks, but no other way.
 */
static int stm_efmi_h7_unlocked;

/* EFMI Registers; offset from es_reg_base */
#define	STM_EFMI_H7_ACR			0x00u
#define	STM_EFMI_H7_KEYR		0x04u
#define	 STM_EFMI_H7_KEYR_KEY1		0x45670123u
#define	 STM_EFMI_H7_KEYR_KEY2		0xcdef89abu
#define	STM_EFMI_H7_OPTKEYR		0x08u
#define	 STM_EFMI_H7_OPTKEYR_KEY1	0x08192a3bu
#define	 STM_EFMI_H7_OPTKEYR_KEY2	0x4c5d6e7fu
#define	STM_EFMI_H7_CR			0x0cu
#define	 STM_EFMI_H7_CR_LOCK		(1u << 0)
#define	 STM_EFMI_H7_CR_PG		(1u << 1)
#define	 STM_EFMI_H7_CR_SER		(1u << 2)
#define	 STM_EFMI_H7_CR_BER		(1u << 3)
#define	 STM_EFMI_H7_CR_PSIZE_x8	(0u << 4)
#define	 STM_EFMI_H7_CR_PSIZE_x16	(1u << 4)
#define	 STM_EFMI_H7_CR_PSIZE_x32	(2u << 4)
#define	 STM_EFMI_H7_CR_PSIZE_x64	(3u << 4)
#define	 STM_EFMI_H7_CR_PSIZE_MASK	(3u << 4)
#define	 STM_EFMI_H7_CR_FW		(1u << 6)
#define	 STM_EFMI_H7_CR_START		(1u << 7)
#define	 STM_EFMI_H7_CR_SNB_MASK	0x7u
#define	 STM_EFMI_H7_CR_SNB_SHIFT	8
#define	 STM_EFMI_H7_CR_CRC_EN		(1u << 15)
#define	 STM_EFMI_H7_CR_EOPIE		(1u << 16)
#define	 STM_EFMI_H7_CR_WRPERRIE	(1u << 17)
#define	 STM_EFMI_H7_CR_PGSERRIE	(1u << 18)
#define	 STM_EFMI_H7_CR_STRBERRIE	(1u << 19)
#define	 STM_EFMI_H7_CR_INCERRIE	(1u << 21)
#define	 STM_EFMI_H7_CR_OPERRIE		(1u << 22)
#define	 STM_EFMI_H7_CR_RDPERRIE	(1u << 23)
#define	 STM_EFMI_H7_CR_RDSERRIE	(1u << 24)
#define	 STM_EFMI_H7_CR_SNECCERRIE	(1u << 25)
#define	 STM_EFMI_H7_CR_DBECCERRIE	(1u << 26)
#define	 STM_EFMI_H7_CR_CRCENDIE	(1u << 27)
#define	 STM_EFMI_H7_CR_CRCRDERRIE	(1u << 28)
#define	STM_EFMI_H7_SR			0x10u
#define	 STM_EFMI_H7_SR_BSY		(1u << 0)
#define	 STM_EFMI_H7_SR_WBNE		(1u << 1)
#define	 STM_EFMI_H7_SR_QW		(1u << 2)
#define	 STM_EFMI_H7_SR_CRC_BUSY	(1u << 3)
#define	 STM_EFMI_H7_SR_EOP		(1u << 16)
#define	 STM_EFMI_H7_SR_WRPERR		(1u << 17)
#define	 STM_EFMI_H7_SR_PGSERR		(1u << 18)
#define	 STM_EFMI_H7_SR_STRBERR		(1u << 19)
#define	 STM_EFMI_H7_SR_INCERR		(1u << 21)
#define	 STM_EFMI_H7_SR_OPERR		(1u << 22)
#define	 STM_EFMI_H7_SR_RDPERR		(1u << 23)
#define	 STM_EFMI_H7_SR_RDSERR		(1u << 24)
#define	 STM_EFMI_H7_SR_SNECCERR	(1u << 25)
#define	 STM_EFMI_H7_SR_DBECCERR	(1u << 26)
#define	 STM_EFMI_H7_SR_CRCEND		(1u << 27)
#define	 STM_EFMI_H7_SR_CRCRDERR	(1u << 28)
#define	 STM_EFMI_H7_SR_ALL_ERRS	(STM_EFMI_H7_SR_WRPERR | \
					 STM_EFMI_H7_SR_PGSERR | \
					 STM_EFMI_H7_SR_STRBERR | \
					 STM_EFMI_H7_SR_INCERR | \
					 STM_EFMI_H7_SR_OPERR | \
					 STM_EFMI_H7_SR_RDPERR | \
					 STM_EFMI_H7_SR_RDSERR | \
					 STM_EFMI_H7_SR_SNECCERR | \
					 STM_EFMI_H7_SR_DBECCERR | \
					 STM_EFMI_H7_SR_CRCRDERR)
#define	STM_EFMI_H7_CCR			0x14u
#define	 STM_EFMI_H7_CCR_CLR(sr)	((sr & STM_EFMI_H7_SR_ALL_ERRS) | \
					 STM_EFMI_H7_SR_EOP)
#define	STM_EFMI_H7_OPTCR		0x18u
#define	 STM_EFMI_H7_OPTCR_OPTLOCK	(1u << 0)
#define	 STM_EFMI_H7_OPTCR_OPTSTRT	(1u << 1)
#define	 STM_EFMI_H7_OPTCR_MER		(1u << 4)
#define	 STM_EFMI_H7_OPTCR_OPTCHANGEERRIE	(1u << 30)
#define	 STM_EFMI_H7_OPTCR_SWAP_BANK	(1u << 31)


#define	STM_EFMI_H7_OPTSR_CUR		0x1cu
#define	STM_EFMI_H7_OPTSR_PRG		0x20u
#define	 STM_EFMI_H7_OPTSR_MASK		0xa3fefffcu	/* XXX: SoC-specific? */
#define	 STM_EFMI_H7_OPTSR_OPT_BUSY	(1u << 0)
#define	 STM_EFMI_H7_OPTSR_RDP_MASK	0xff00u
#define	 STM_EFMI_H7_OPTSR_RDP(x)	((x) & 0xff00u)
#define	 STM_EFMI_H7_OPTSR_RDP_MAGIC(l)	(((l) == 0) ? 0xaa00 : \
					  ((l) == 2) ? 0xcc00u : 0x0000u)
#define	 STM_EFMI_H7_OPTSR_RDP_LEVEL(x)	((STM_EFMI_H7_OPTCR_RDP(x)==0xaa00u)?0:\
					 ((STM_EFMI_H7_OPTCR_RDP(x)==0xcc00u)?2:1))
#define	STM_EFMI_H7_OPTCCR		0x24u
#define	 STM_EFMI_H7_OPTCCR_CLR_OPTCHANGEERR	(1u << 30)
#define	STM_EFMI_H7_PRAR1_CUR		0x28u
#define	STM_EFMI_H7_PRAR1_PRG		0x2cu		/* <--- */
#define	STM_EFMI_H7_PRAR1_MASK		0x8fff0fffu	/* XXX: SoC-specific? */
#define	STM_EFMI_H7_SCAR1_CUR		0x30u
#define	STM_EFMI_H7_SCAR1_PRG		0x34u		/* <--- */
#define	STM_EFMI_H7_SCAR1_MASK		0x8fff0fffu	/* XXX: SoC-specific? */
#define	STM_EFMI_H7_WPSN_CUR1		0x38u
#define	STM_EFMI_H7_WPSN_PRG1		0x3cu		/* <--- */
#define	STM_EFMI_H7_WPSN_MASK		0x000000ffu	/* XXX: SoC-specific? */
#define	STM_EFMI_H7_BOOT7_CUR		0x40u
#define	STM_EFMI_H7_BOOT7_PRG		0x44u		/* <--- */
#define	STM_EFMI_H7_BOOT4_CUR		0x48u
#define	STM_EFMI_H7_BOOT4_PRG		0x4cu		/* <--- */
#define	STM_EFMI_H7_PRAR2_CUR		0x128u
#define	STM_EFMI_H7_PRAR2_PRG		0x12cu		/* <--- */
#define	STM_EFMI_H7_PRAR2_MASK		0x8fff0fffu	/* XXX: SoC-specific? */
#define	STM_EFMI_H7_SCAR2_CUR		0x130u
#define	STM_EFMI_H7_SCAR2_PRG		0x134u		/* <--- */
#define	STM_EFMI_H7_SCAR2_MASK		0x8fff0fffu	/* XXX: SoC-specific? */
#define	STM_EFMI_H7_WPSN_CUR2		0x138u
#define	STM_EFMI_H7_WPSN_PRG2		0x13cu		/* <--- */

#if 0
static const uint8_t stm_efmi_h7_flash_applet[] = {
#include "flash-applets/applet-stm-efmi_h7.applet"
};
#endif

/*
 * Convenience functions for read/write of EFMI registers
 */
static int
stm_efmi_h7_read_reg(struct stm_efmi_h7_target_flash *tf, uint32_t reg,
    uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(tf->tf_es->es_target, tf->tf_reg_base + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(tf->tf_es->es_target);

	return rv;
}

static int
stm_efmi_h7_write_reg(struct stm_efmi_h7_target_flash *tf, uint32_t reg,
    uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(tf->tf_es->es_target, tf->tf_reg_base + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(tf->tf_es->es_target);

	return rv;
}

static uint32_t
stm_efmi_h7_wait_bsy(struct stm_efmi_h7_target_flash *tf, FILE *os)
{
	struct stm_efmi_h7_state *es = tf->tf_es;
	timer_timeout_t to;
	uint32_t sr, err, erase_time;
	int rv;

	if (os != NULL)
		erase_time = es->es_mass_erase_time;
	else
		erase_time = es->es_sector_erase_time;
	timer_timeout_start(&to, erase_time);

	do {
		rv = stm_efmi_h7_read_reg(tf, STM_EFMI_H7_SR, &sr);
		if (rv < 0) {
			rv = TARGET_LINK_ERROR(es->es_target);
			DBFPRINTF("DP err 0x%08x\n", rv);
			break;
		}

		if ((sr & STM_EFMI_H7_SR_BSY) != 0) {
			if (os != NULL)
				twiddle(os);
			rtos_task_sleep(1);
		}
	} while ((sr & STM_EFMI_H7_SR_BSY) != 0 && !timer_timeout_expired(&to));

	DBFPRINTF("Waited for %" PRIu32 " mS\n",
	    erase_time - timer_timeout_remaining(&to));

	err = STM_EFMI_H7_SR_ALL_ERRS;

	if (rv == 0 && (sr & STM_EFMI_H7_SR_BSY) != 0) {
		DBFPRINTF("Timeout waiting for BSY\n");
		return err;
	}

	if (rv < 0)
		return err;

	if ((err & sr) != 0 &&
	    stm_efmi_h7_write_reg(tf, STM_EFMI_H7_SR,
				  STM_EFMI_H7_SR_EOP | err) < 0) {
		DBFPRINTF("Failed to clear SR\n");
		return ~0u;
	}

	if (sr != 0)
		DBFPRINTF("Non-zero SR: 0x%08" PRIx32 "\n", sr);

	return sr & err;
}

static int
stm_efmi_h7_unlock(struct stm_efmi_h7_target_flash *tf)
{
	uint32_t cr;

	if (STM_EFMI_H7_IS_UNLOCKED(tf))
		return 0;

	/*
	 * The FLASH_CR.PSIZE field depends on the target's Vcc.
	 * We set it up here rather than at attach time because
	 * it's far more likely that power_target_vcc is stable
	 * by this point.
	 */
	if (power_target_vcc < F16(2.1)) {
		DBFPRINTF("PSIZE = x8\n");
		tf->tf_psize = STM_EFMI_H7_CR_PSIZE_x8;
	} else
	if (power_target_vcc < F16(2.7)) {
		DBFPRINTF("PSIZE = x16\n");
		tf->tf_psize = STM_EFMI_H7_CR_PSIZE_x16;
	} else {
		DBFPRINTF("PSIZE = x32\n");
		tf->tf_psize = STM_EFMI_H7_CR_PSIZE_x32;
	}

	if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_CR, &cr) < 0) {
		DBFPRINTF("Failed to read CR\n");
		return -1;
	}

	if (cr & STM_EFMI_H7_CR_LOCK) {
		/* Perform an unlock sequence */
		if (stm_efmi_h7_write_reg(tf, STM_EFMI_H7_KEYR,
		    STM_EFMI_H7_KEYR_KEY1) < 0 ||
		    stm_efmi_h7_write_reg(tf, STM_EFMI_H7_KEYR,
		    STM_EFMI_H7_KEYR_KEY2) < 0) {
			DBFPRINTF("keys failed\n");
			return -1;
		}

		if (stm_efmi_h7_wait_bsy(tf, NULL) != 0) {
			DBFPRINTF("BSY failed\n");
			return -1;
		}

		if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_CR, &cr) < 0) {
			DBFPRINTF("CR read-back failed\n");
			return -1;
		}

		if (cr & STM_EFMI_H7_CR_LOCK) {
			/* Flash remained locked. We're stuffed. */
			DBFPRINTF("Flash remained locked\n");
			return -1;
		}
	}

	stm_efmi_h7_unlocked++;
	tf->tf_es->es_flags |= STM_EFMI_H7_FLAGS_UNLOCKED(tf);
	DBFPRINTF("Unlocked\n");

	return 0;
}

static void
stm_efmi_h7_lock(struct stm_efmi_h7_target_flash *tf)
{
	uint32_t cr;

	if (!STM_EFMI_H7_IS_UNLOCKED(tf))
		return;

	tf->tf_es->es_flags &= ~(STM_EFMI_H7_FLAGS_UNLOCKED(tf));

	if (stm_efmi_h7_unlocked && --stm_efmi_h7_unlocked) {
		DBFPRINTF("unlocked count not yet zero\n");
		return;
	}

	(void) TARGET_LINK_ERROR(tf->tf_es->es_target);
	if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_CR, &cr) < 0)
		return;
	cr |= STM_EFMI_H7_CR_LOCK;
	(void) stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR, cr);
	DBFPRINTF("locked\n");
}

static int
stm_efmi_h7_sector_desc_flash(target_flash_t tft, target_addr_t *poffset,
    struct target_flash_sector *fs)
{
	struct stm_efmi_h7_target_flash *tf;
	struct stm_efmi_h7_state *es;
	uint32_t offset = *poffset;
	uint32_t number, base;

	tf = (struct stm_efmi_h7_target_flash *)tft;
	es = tf->tf_es;

	base = tf->tf_flash_base;
	number = 0u;

	DBFPRINTF("start base 0x%08" PRIx32 ", offset 0x%08" PRIx32
	    ", number %" PRIu32 "\n", base, offset, number);

	fs->fs_size = es->es_sector_size;
	fs->fs_pages = es->es_sector_size / es->es_max_page_size;
	*poffset = offset & (es->es_sector_size - 1);
	base += offset & ~(es->es_sector_size - 1);
	number += offset / es->es_sector_size;

	fs->fs_number = number;
	fs->fs_cookie = (uintptr_t)base;

	DBFPRINTF("final base 0x%08" PRIx32 ", offset 0x%08" PRIxTADDR
	    ", number %" PRIu32 "\n", base, *poffset, number);

	return 0;
}

static int
stm_efmi_h7_erase_sector(target_flash_t tft,
    const struct target_flash_sector *fs)
{
	struct stm_efmi_h7_target_flash *tf;
	uint32_t cr;

	tf = (struct stm_efmi_h7_target_flash *)tft;

	if (stm_efmi_h7_unlock(tf) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	DBFPRINTF("Sector %" PRIu32 " @ 0x%08" PRIx32 "\n", fs->fs_number,
	    (uint32_t)fs->fs_cookie);

	cr = tf->tf_psize | STM_EFMI_H7_CR_SER |
	    (fs->fs_number << STM_EFMI_H7_CR_SNB_SHIFT);

	if (stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR, cr) < 0 ||
	    stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR,
	    cr | STM_EFMI_H7_CR_START) < 0) {
		DBFPRINTF("Error\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	cr = stm_efmi_h7_wait_bsy(tf, NULL);

	(void) stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR, tf->tf_psize);

	DBFPRINTF("%s (cr = 0x%" PRIx32 ")\n", (cr == 0) ? "Done" : "Failed",
	    cr);

	return (cr == 0) ? TARGET_FLASH_RESULT_OK : TARGET_FLASH_RESULT_ERROR;
}

static int
stm_efmi_h7_write_page(target_flash_t tft, const struct target_flash_sector *fs,
    uint32_t page)
{
	struct stm_efmi_h7_target_flash *tf;
	struct stm_efmi_h7_state *es;
	target_addr_t dest;
	const void *src;
	unsigned int len, pagelen, inc;
	uint32_t sr;
	int rv = 0;

	tf = (struct stm_efmi_h7_target_flash *)tft;
	es = tf->tf_es;

	DBFPRINTF("sector %" PRIu32 ", page %" PRIu32 "\n", fs->fs_number,
	    page);

	if (stm_efmi_h7_unlock(tf) < 0) {
		DBFPRINTF("failed to unlock Flash\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	pagelen = es->es_max_page_size;

	src = tft->tf_buffer;
	dest = (target_addr_t)(fs->fs_cookie + (page * pagelen));
	DBFPRINTF("addr 0x%" PRIxTADDR ", pagelen 0x%x\n", dest, pagelen);

	if (es->es_applet != NULL) {
		rv = flash_applet_write_page(tft, dest, pagelen,
		    tf->tf_reg_base, tf->tf_psize, es->es_applet);
	} else {
		if (tf->tf_psize == STM_EFMI_H7_CR_PSIZE_x8)
			inc = 1;
		else
		if (tf->tf_psize == STM_EFMI_H7_CR_PSIZE_x16)
			inc = 2;
		else
			inc = 4;

		if (stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR,
		    tf->tf_psize | STM_EFMI_H7_CR_PG) < 0) {
			DBFPRINTF("failed to set PG\n");
			return TARGET_FLASH_RESULT_ERROR;
		}

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

			sr = stm_efmi_h7_wait_bsy(tf, NULL);
			if (sr != 0) {
				DBFPRINTF("sr 0x%" PRIx32, sr);
				rv = -1;
				break;
			}
		}

		rv = (rv < 0) ? TARGET_FLASH_RESULT_ERROR :
		    TARGET_FLASH_RESULT_OK;

		(void) stm_efmi_h7_write_reg(tf, STM_EFMI_H7_CR, tf->tf_psize);
	}

	DBFPRINTF("%s\n", (rv != TARGET_FLASH_RESULT_ERROR) ? "Done" :
	    "Failed");

	return rv;
}

#if 0
static int
stm_efmi_h7_opt_unlock(struct stm_efmi_h7_state *es)
{
	struct stm_efmi_h7_target_flash *tf = &es->es_tf[0];
	uint32_t ocr;

	DBFPRINTF("unlocking\n");

	if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_OPTCR, &ocr) < 0) {
		DBFPRINTF("Failed to read OPTCR\n");
		return -1;
	}

	if ((ocr & STM_EFMI_H7_OPTCR_OPTLOCK) == 0) {
		DBFPRINTF("already unlocked\n");
		return 0;
	}

	if (stm_efmi_h7_write_reg(tf, STM_EFMI_H7_OPTKEYR,
	                       STM_EFMI_H7_OPTKEYR_KEY1) < 0 ||
	    stm_efmi_h7_write_reg(tf, STM_EFMI_H7_OPTKEYR,
	                       STM_EFMI_H7_OPTKEYR_KEY2) < 0) {
		DBFPRINTF("OPTCR unlock failed.\n");
		return -1;
	}

	if (stm_efmi_h7_wait_bsy(tf, NULL) != 0) {
		DBFPRINTF("BSY after OPTCR unlock failed.\n");
		return -1;
	}

	if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_OPTCR, &ocr) < 0) {
		DBFPRINTF("failed to re-read OPTCR after unlock\n");
		return -1;
	}

	if (ocr & STM_EFMI_H7_OPTCR_OPTLOCK) {
		/* OPTCR remained locked. We're stuffed. */
		DBFPRINTF("OPTCR still locked\n");
		return -1;
	}

	return 0;
}

static int
stm_efmi_h7_opt_lock(struct stm_efmi_h7_state *es)
{
	struct stm_efmi_h7_target_flash *tf = &es->es_tf[0];
	uint32_t ocr;

	if (stm_efmi_h7_read_reg(tf, STM_EFMI_H7_OPTCR, &ocr) < 0) {
		DBFPRINTF("Failed to read OPTCR\n");
		return -1;
	}

	if ((ocr & STM_EFMI_H7_OPTCR_OPTLOCK) != 0)
		return 0;

	if (stm_efmi_h7_write_reg(tf, STM_EFMI_H7_OPTCR,
	    STM_EFMI_H7_OPTCR_OPTLOCK) < 0) {
		DBFPRINTF("Failed to set OPTLOCK\n");
		return -1;
	}

	return 0;
}
#endif

static void
stm_efmi_h7_flush_options(struct stm_efmi_h7_state *es, FILE *os)
{

	(void) es;
	(void) os;
}

static int
stm_efmi_h7_finish(target_flash_t tft)
{
	struct stm_efmi_h7_target_flash *tf;
	struct stm_efmi_h7_state *es;
	int rv = TARGET_FLASH_RESULT_OK;

	tf = (struct stm_efmi_h7_target_flash *)tft;
	es = tf->tf_es;

	if (es->es_applet != NULL)
		rv = flash_applet_finish(es->es_applet);

	if ((es->es_flags & STM_EFMI_H7_FLAGS_FLUSH_OPTS) != 0) {
		es->es_flags &= ~STM_EFMI_H7_FLAGS_FLUSH_OPTS;
		stm_efmi_h7_flush_options(es, NULL);
	}

	stm_efmi_h7_lock(tf);

	return rv;
}

static void
stm_efmi_h7_free(target_flash_t tft)
{
	struct stm_efmi_h7_target_flash *tf;
	struct stm_efmi_h7_state *es;

	tf = (struct stm_efmi_h7_target_flash *)tft;
	es = tf->tf_es;

	es->es_ref_cnt -= 1;
	if (es->es_ref_cnt)
		return;

	if (es->es_applet != NULL)
		flash_applet_finish(es->es_applet);

	zone_free(es);
}

static int
flash_stm_efmi_h7_attach_impl(struct stm_efmi_h7_state *es,
    struct stm_efmi_h7_target_flash *tf, const char *desc)
{

	tf->tf_tf.tf_sector_desc = stm_efmi_h7_sector_desc_flash;
	tf->tf_tf.tf_write_page = stm_efmi_h7_write_page;
	tf->tf_tf.tf_erase_sector = stm_efmi_h7_erase_sector;
	tf->tf_tf.tf_max_page_size = es->es_max_page_size;
	tf->tf_tf.tf_finish = stm_efmi_h7_finish;
	tf->tf_tf.tf_free = stm_efmi_h7_free;
	tf->tf_tf.tf_size = tf->tf_flash_size;
	tf->tf_tf.tf_erase_byte = 0xffu;

	DBFPRINTF("base 0x%08" PRIxTADDR ", size 0x%08" PRIx32 "\n",
	    tf->tf_flash_base, tf->tf_flash_size);

	if (target_add_flash(es->es_target, &tf->tf_tf, tf->tf_flash_base,
	    tf->tf_flash_size, desc, 0) < 0){
		DBFPRINTF("failed to add flash @ %08" PRIxTADDR "\n",
		    tf->tf_flash_base);
		return -1;
	}

	es->es_ref_cnt++;

	return 0;
}

void *
flash_stm_efmi_h7_attach(const struct flash_stm_efmi_h7_desc *ed)
{
	struct stm_efmi_h7_target_flash *tf;
	struct stm_efmi_h7_state *es;
	void *flash_applet;

	DBFPRINTF("reg 0x%08" PRIx32 ", size 0x%08" PRIx32 "\n", ed->ed_regbase,
	    ed->ed_flash_size);

#if 0
	if (0) {
		struct flash_applet_params fp;
		fp.fp_applet = stm_efmi_h7_flash_applet;
		fp.fp_applet_size = sizeof(stm_efmi_h7_flash_applet);
		fp.fp_applet_stack_size = 0;
		fp.fp_max_page_size = ed->ed_max_page_size;
		flash_applet = flash_applet_attach(ed->ed_target, &fp);
	} else {
		flash_applet = NULL;
	}
#else
	flash_applet = NULL;
#endif

	if ((es = zone_calloc(1, sizeof(*es))) == NULL) {
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		return NULL;
	}

	es->es_target = ed->ed_target;
	es->es_applet = flash_applet;
	es->es_max_page_size = ed->ed_max_page_size;
	es->es_sector_size = ed->ed_sector_size;
	es->es_mass_erase_time = ed->ed_mass_erase_time;
	es->es_sector_erase_time = ed->ed_sector_erase_time;

	tf = &es->es_tf[0];
	tf->tf_es = es;
	tf->tf_flash_size = ed->ed_flash_size / 2u;
	tf->tf_reg_base = ed->ed_regbase;
	tf->tf_flash_base = ed->ed_flash_base;
	if (ed->ed_swapped) {
		tf->tf_reg_base += 0x100u;
		tf->tf_flash_base += 1024u * 1024u;
	}

	tf = &es->es_tf[1];
	tf->tf_es = es;
	tf->tf_flash_size = ed->ed_flash_size / 2u;
	tf->tf_reg_base = ed->ed_regbase;
	tf->tf_flash_base = ed->ed_flash_base;
	if (!ed->ed_swapped) {
		tf->tf_reg_base += 0x100u;
		tf->tf_flash_base += 1024u * 1024u;
	}

	if (flash_stm_efmi_h7_attach_impl(es, &es->es_tf[0], "Bank 1") < 0 ||
	    flash_stm_efmi_h7_attach_impl(es, &es->es_tf[1], "Bank 2") < 0) {
		if (es->es_ref_cnt == 0) {
			if (flash_applet != NULL)
				flash_applet_detach(flash_applet);
			zone_free(es);
		}
		return NULL;
	}

	return es;
}

const char stm_efmi_h7_rdp_query_text[] =
"Possible level values:\n"
"\t0  Device is unlocked; RDP == 0xAA.\n"
"\t1  Read-out protection of memories (Debug features limited)\n"
"\t2  Chip protection; RDP == 0xCC. (Debug & boot from RAM disabled).\n"
"Note that changing the level will require the target to be power-cycled\n"
"for the change to take effect.\n";

int
flash_stm_efmi_h7_rdp_query(void *arg, struct target_protection_info *pi)
{

	(void) arg;

	pi->pi_max_level = 2;
	pi->pi_level = 0;
	pi->pi_text = stm_efmi_h7_rdp_query_text;

	return 0;
}

int
flash_stm_efmi_h7_rdp_set(void *arg, struct target_protection_info *pi)
{

	(void) arg;

	if (pi->pi_level > 2)
		return -1;

	return TARGET_PI_OK_POWER_CYCLE;
}

int
flash_stm_efmi_h7_rdp_clear(void *arg, FILE *os)
{

	(void) arg;
	(void) os;

	return TARGET_PI_OK_POWER_CYCLE;
}
