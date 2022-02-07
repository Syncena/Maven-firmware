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

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

#include "flash_stm_fpec.h"
#include "flash_stm_efmi.h"	/* For const char stm_efmi_rdp_query_text[] */
#include "flash_applet.h"
#include "timer.h"
#include "zone_alloc.h"

/*
 * Driver for the Flash Program and Erase Controller found on some STM43F parts.
 *
 * Maybe one day ST will implement a better Flash controller...
 */

//#define DEBUG_FLAG_INIT	1
#include "debug.h"

struct stm_fpec_state {
	struct target_flash fs_tf;
	target_t fs_target;
	void *fs_applet;
	enum flash_stm_fpec_nvm_type fs_nt;
	uint32_t fs_base;
	uint32_t fs_pages;
	uint32_t fs_bank2_base;
	int fs_unlocked;
};

/*
 * Lock/Unlock state is shared between Flash and Option Byte instances,
 * so it must be global. Sucks, but no other way.
 */
static int stm_fpec_unlocked;

#define	STM_FPEC_REG_BASE	0x40022000u

/* FPEC Registers; offset from STM_FPEC_REG_BASE */
#define	STM_FPEC_ACR			0x00u
#define	STM_FPEC_KEYR			0x04u
#define	 STM_FPEC_KEYR_KEY1		0x45670123u
#define	 STM_FPEC_KEYR_KEY2		0xcdef89abu
#define	STM_FPEC_OPTKEYR		0x08u
#define	STM_FPEC_SR			0x0cu
#define	 STM_FPEC_SR_BSY		(1u << 0)
#define	 STM_FPEC_SR_PGERR		(1u << 2)
#define	 STM_FPEC_SR_WRPRTERR		(1u << 4)
#define	 STM_FPEC_SR_EOP		(1u << 5)
#define	 STM_FPEC_SR_ALL_ERRS		(STM_FPEC_SR_PGERR | \
					 STM_FPEC_SR_WRPRTERR)
#define	STM_FPEC_CR			0x10u
#define	 STM_FPEC_CR_PG			(1u << 0)
#define	 STM_FPEC_CR_PER		(1u << 1)
#define	 STM_FPEC_CR_MER		(1u << 2)
#define	 STM_FPEC_CR_OPTPG		(1u << 4)
#define	 STM_FPEC_CR_OPTER		(1u << 5)
#define	 STM_FPEC_CR_STRT		(1u << 6)
#define	 STM_FPEC_CR_LOCK		(1u << 7)
#define	 STM_FPEC_CR_OPTWRE		(1u << 9)
#define	 STM_FPEC_CR_ERRIE		(1u << 10)
#define	 STM_FPEC_CR_EOPIE		(1u << 12)
#define	STM_FPEC_AR			0x14u

#define	STM_FPEC_OBR			0x1cu
#define	 STM_FPEC_OBR_OPTERR		(1u << 0)
#define	 STM_FPEC_OBR1_RDPRT		(1u << 1)
#define	 STM_FPEC_OBR1_WDG_SW		(1u << 2)
#define	 STM_FPEC_OBR1_nRST_STOP	(1u << 3)
#define	 STM_FPEC_OBR1_nRST_STDBY	(1u << 4)
#define	 STM_FPEC_OBR1_USER(x)		(((x) >> 2) & 0x1fu)
#define	 STM_FPEC_OBR1_DATA0(x)		(((x) >> 10) & 0xffu)
#define	 STM_FPEC_OBR1_DATA1(x)		(((x) >> 18) & 0xffu)
#define	 STM_FPEC_OBR2_RDPRT(x)		(((x) >> 1) & 3u)
#define	 STM_FPEC_OBR2_WDG_SW		(1u << 8)
#define	 STM_FPEC_OBR2_nRST_STOP	(1u << 9)
#define	 STM_FPEC_OBR2_nRST_STDBY	(1u << 10)
#define	 STM_FPEC_OBR2_USER(x)		(((x) >> 8) & 0xffu)
#define	 STM_FPEC_OBR2_DATA0(x)		(((x) >> 16) & 0xffu)
#define	 STM_FPEC_OBR2_DATA1(x)		(((x) >> 24) & 0xffu)
#define	STM_FPEC_WRPR			0x20u
#define	 STM_FPEC_WRPR_BITS(a,x)	(((x) >> ((a) * 8)) & 0xffu)
#define	STM_FPEC_KEYR2			0x44u
#define	STM_FPEC_SR2			0x4cu
#define	STM_FPEC_CR2			0x50u
#define	STM_FPEC_AR2			0x54u

#define	STM_FPEC_OB16(v)		((v) | (((v) ^ 0xffu) << 8))

#define	STM_FPEC_RDP1_LEVEL_0		0xa5u
#define	STM_FPEC_RDP1_LEVEL_1		0x00u
#define	STM_FPEC_RDP2_LEVEL_0		0xaau
#define	STM_FPEC_RDP2_LEVEL_1		0x00u
#define	STM_FPEC_RDP2_LEVEL_2		0xccu

static const uint8_t stm_fpec_flash_applet[] = {
#include "flash-applets/applet-stm-fpec.applet"
};

/*
 * Convenience functions for read/write of FPEC registers
 */
static int
stm_fpec_read_reg(struct stm_fpec_state *fs, uint32_t reg, uint32_t *pv)
{
	int rv;

	rv = TARGET_REG_READ32(fs->fs_target, STM_FPEC_REG_BASE + reg, pv);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(fs->fs_target);

	return rv;
}

static int
stm_fpec_write_reg(struct stm_fpec_state *fs, uint32_t reg, uint32_t v)
{
	int rv;

	rv = TARGET_REG_WRITE32(fs->fs_target, STM_FPEC_REG_BASE + reg, v);
	if (rv < 0)
		(void) TARGET_LINK_ERROR(fs->fs_target);

	return rv;
}

static int
stm_fpec_unlock(struct stm_fpec_state *fs)
{
	uint32_t cr, cr2 = 0;

	if (fs->fs_unlocked)
		return 0;

	if (stm_fpec_read_reg(fs, STM_FPEC_CR, &cr) < 0)
		return -1;

	if (cr & STM_FPEC_CR_LOCK) {
		/* Perform an unlock sequence */
		DBFPRINTF("Unlock sequence on bank 0\n");
		stm_fpec_write_reg(fs, STM_FPEC_KEYR, STM_FPEC_KEYR_KEY1);
		stm_fpec_write_reg(fs, STM_FPEC_KEYR, STM_FPEC_KEYR_KEY2);
		if (stm_fpec_read_reg(fs, STM_FPEC_CR, &cr) < 0) {
			DBFPRINTF("read after unlock failed\n");
			return -1;
		}
		if (cr & STM_FPEC_CR_LOCK) {
			/* Flash remained locked. We're stuffed. */
			DBFPRINTF("Unlock failed\n");
			return -1;
		}
	}

	if (fs->fs_bank2_base) {
		DBFPRINTF("Unlock sequence on bank 1\n");
		if (stm_fpec_read_reg(fs, STM_FPEC_CR2, &cr2) < 0) {
			stm_fpec_write_reg(fs, STM_FPEC_CR, cr |
			    STM_FPEC_CR_LOCK);
			return -1;
		}

		if (cr2 & STM_FPEC_CR_LOCK) {
			/* Perform an unlock sequence */
			stm_fpec_write_reg(fs, STM_FPEC_KEYR2,
			    STM_FPEC_KEYR_KEY1);
			stm_fpec_write_reg(fs, STM_FPEC_KEYR2,
			    STM_FPEC_KEYR_KEY2);
			if (stm_fpec_read_reg(fs, STM_FPEC_CR2, &cr2) < 0) {
				DBFPRINTF("read after unlock failed\n");
				return -1;
			}
			if (cr2 & STM_FPEC_CR_LOCK) {
				/* Flash remained locked. We're stuffed. */
				stm_fpec_write_reg(fs, STM_FPEC_CR, cr |
				    STM_FPEC_CR_LOCK);
				DBFPRINTF("Unlock failed\n");
				return -1;
			}
		}
	}

	if (fs->fs_nt != FLASH_STM_FPEC_FLASH &&
	    (cr & STM_FPEC_CR_OPTWRE) == 0) {
		/* Perform option byte unlock sequence */
		DBFPRINTF("Unlock sequence on option bytes\n");
		stm_fpec_write_reg(fs, STM_FPEC_OPTKEYR, STM_FPEC_KEYR_KEY1);
		stm_fpec_write_reg(fs, STM_FPEC_OPTKEYR, STM_FPEC_KEYR_KEY2);
		if (stm_fpec_read_reg(fs, STM_FPEC_CR, &cr) < 0 ||
		    (cr & STM_FPEC_CR_OPTWRE) == 0) {
			/* Option bytes remained locked. We're stuffed. */
			DBFPRINTF("Unlock failed\n");
			stm_fpec_write_reg(fs, STM_FPEC_CR, cr |
			    STM_FPEC_CR_LOCK);
			if (fs->fs_bank2_base) {
				stm_fpec_write_reg(fs, STM_FPEC_CR2, cr2 |
				    STM_FPEC_CR_LOCK);
			}
			return -1;
		}
	}

	stm_fpec_unlocked++;
	fs->fs_unlocked = 1;

	DBFPRINTF("Unlock succeeded\n");

	return 0;
}

static void
stm_fpec_lock(struct stm_fpec_state *fs)
{
	uint32_t cr;

	if (fs->fs_unlocked == 0)
		return;
	fs->fs_unlocked = 0;

	if (stm_fpec_unlocked && --stm_fpec_unlocked)
		return;

	if (stm_fpec_read_reg(fs, STM_FPEC_CR, &cr) < 0)
		return;

	DBFPRINTF("Locking\n");

	cr &= ~STM_FPEC_CR_OPTWRE;
	cr |= STM_FPEC_CR_LOCK;
	stm_fpec_write_reg(fs, STM_FPEC_CR, cr);

	if (fs->fs_bank2_base) {
		if (stm_fpec_read_reg(fs, STM_FPEC_CR2, &cr) < 0)
			return;
		cr |= STM_FPEC_CR_LOCK;
		stm_fpec_write_reg(fs, STM_FPEC_CR2, cr);
	}
}

static int
stm_fpec_wait_bsy(struct stm_fpec_state *fs, uint32_t sr_reg)
{
	timer_timeout_t to;
	uint32_t sr;

	timer_timeout_start(&to, 8000);
	do {
		if (stm_fpec_read_reg(fs, sr_reg, &sr) < 0) {
			DBFPRINTF("Failed to read SR #1\n");
			return -1;
		}
	} while ((sr & STM_FPEC_SR_BSY) != 0 && !timer_timeout_expired(&to));

	if ((sr & STM_FPEC_SR_BSY) != 0) {
		DBFPRINTF("Timeout waiting for BSY\n");
		return -1;
	}

	stm_fpec_write_reg(fs, sr_reg, STM_FPEC_SR_EOP |
	    STM_FPEC_SR_ALL_ERRS);

	if ((sr & (STM_FPEC_SR_EOP | STM_FPEC_SR_ALL_ERRS)) != STM_FPEC_SR_EOP){
		DBFPRINTF("SR has errors set\n");
		return -1;
	}

	return 0;
}

static int
stm_fpec_sector_desc(target_flash_t tf, target_addr_t *offset,
    struct target_flash_sector *tfs)
{

	/* Write and erase sizes are identical (for now) */
	tfs->fs_size = tf->tf_max_page_size;
	tfs->fs_pages = 1;
	tfs->fs_number = *offset / tfs->fs_size;
	tfs->fs_cookie = tfs->fs_number * tfs->fs_size;
	*offset %= tfs->fs_size;

	return 0;
}

static int
stm_fpec_erase_sector(target_flash_t tf, const struct target_flash_sector *tfs)
{
	struct stm_fpec_state *fs = (struct stm_fpec_state *)tf;
	uint32_t ar, cr, cr_reg, ar_reg, sr_reg, cmd;
	int rv;

	if (stm_fpec_unlock(fs) < 0)
		return TARGET_FLASH_RESULT_ERROR;

	ar = fs->fs_base + (uint32_t)tfs->fs_cookie;

	DBFPRINTF("Sector @ 0x%08" PRIx32 ": ", ar);

	if (fs->fs_bank2_base == 0 || ar < fs->fs_bank2_base) {
		cr_reg = STM_FPEC_CR;
		ar_reg = STM_FPEC_AR;
		sr_reg = STM_FPEC_SR;
	} else {
		cr_reg = STM_FPEC_CR2;
		ar_reg = STM_FPEC_AR2;
		sr_reg = STM_FPEC_SR2;
	}

	if (fs->fs_nt == FLASH_STM_FPEC_FLASH)
		cmd = STM_FPEC_CR_PER;
	else
		cmd = STM_FPEC_CR_OPTER;

#if 0
	if (stm_fpec_read_reg(fs, cr_reg, &cr) < 0) {
		DBPRINTF("failed to read CR\n");
		return -1;
	}

	cr &= STM_FPEC_CR_OPTWRE;
#else
	cr = STM_FPEC_CR_OPTWRE;
#endif

	stm_fpec_write_reg(fs, cr_reg, cmd | cr);
	stm_fpec_write_reg(fs, ar_reg, ar);
	stm_fpec_write_reg(fs, cr_reg, cmd | cr | STM_FPEC_CR_STRT);

	rv = stm_fpec_wait_bsy(fs, sr_reg);

	stm_fpec_write_reg(fs, cr_reg, cr);

	DBPRINTF(" %s\n", (rv == 0) ? "Done" : "Failed");

	return (rv < 0) ? TARGET_FLASH_RESULT_ERROR : TARGET_FLASH_RESULT_OK;
}

static int
stm_fpec_verify_ob(struct stm_fpec_state *fs, const uint16_t *src, size_t cnt)
{

	if ((cnt * sizeof(uint16_t)) != fs->fs_tf.tf_size) {
		DBFPRINTF("cnt %" PRIu32 " != size %" PRIu32 "\n",
		    (uint32_t)cnt, fs->fs_tf.tf_size);
		return -1;
	}

	for (int i = 0; i < (int)cnt; i++, src++) {
		if (*src != STM_FPEC_OB16(*src & 0xffu)) {
			DBFPRINTF("%d 0x%04" PRIx16 " != 0x%04" PRIx16 "\n", i,
			    *src, STM_FPEC_OB16(*src & 0xffu));
			return -1;
		}
	}

	return 0;
}

static int
stm_fpec_write_page(target_flash_t tf, const struct target_flash_sector *tfs,
    uint32_t page)
{
	struct stm_fpec_state *fs = (struct stm_fpec_state *)tf;
	const uint16_t *src, *last;
	uint32_t addr, cr, sr_reg, cmd;
	int rv = 0;

	DBFPRINTF("sector %" PRIu32 ", page %" PRIu32 "\n", tfs->fs_number,
	    page);

	if (page != 0)
		return TARGET_FLASH_RESULT_ERROR;

	if (stm_fpec_unlock(fs) < 0) {
		DBFPRINTF("failed to unlock Flash\n");
		return TARGET_FLASH_RESULT_ERROR;
	}

	src = (const uint16_t *)(void *)tf->tf_buffer;
	last = src + (tf->tf_max_page_size / sizeof(*src));
	addr = fs->fs_base + (uint32_t)tfs->fs_cookie;

	/*
	 * We won't be crossing a page boundary, so the controller registers
	 * are constant for the entire write operation.
	 */
	if (fs->fs_bank2_base == 0 || addr < fs->fs_bank2_base)
		sr_reg = STM_FPEC_SR;
	else
		sr_reg = STM_FPEC_SR2;

	DBFPRINTF("sr %02" PRIx32 ", src %p, last %p, addr 0x%08" PRIx32 "\n",
	    sr_reg, (const void *)src, (const void *)last, addr);

	if (fs->fs_nt == FLASH_STM_FPEC_FLASH && fs->fs_applet != NULL) {
		rv = flash_applet_write_page(tf, addr, tf->tf_max_page_size,
		    STM_FPEC_REG_BASE, sr_reg, fs->fs_applet);
	} else {
		if (fs->fs_nt == FLASH_STM_FPEC_FLASH) {
			DBFPRINTF("Writing Flash bytes\n");
			cmd = STM_FPEC_CR_PG;
		} else {
			DBFPRINTF("Writing option bytes\n");
			cmd = STM_FPEC_CR_OPTPG;
			if (stm_fpec_verify_ob(fs, src, last - src) < 0) {
				DBFPRINTF("option bytes failed to verify\n");
				return TARGET_FLASH_RESULT_ERROR;
			}
		}

#if 0
		if (stm_fpec_read_reg(fs, sr_reg + 4u, &cr) < 0) {
			DBFPRINTF("failed to read CR\n");
			return -1;
		}

		cr &= STM_FPEC_CR_OPTWRE;
#else
		cr = STM_FPEC_CR_OPTWRE;
#endif

		while (rv >= 0 && src < last) {
			stm_fpec_write_reg(fs, sr_reg + 4u, cmd | cr);
			TARGET_REG_WRITE16(fs->fs_target, addr, *src);

			src++;
			addr += 2;

			rv = stm_fpec_wait_bsy(fs, sr_reg);
		}

		rv = (rv < 0) ? TARGET_FLASH_RESULT_ERROR :
		    TARGET_FLASH_RESULT_OK;
	}

	DBFPRINTF("rv %d\n", rv);

	return rv;
}

static int
stm_fpec_finish(target_flash_t tf)
{
	struct stm_fpec_state *fs = (struct stm_fpec_state *)tf;
	int rv = TARGET_FLASH_RESULT_OK;

	if (fs->fs_applet != NULL)
		rv = flash_applet_finish(fs->fs_applet);

	stm_fpec_lock(fs);

	return rv;
}

static void
stm_fpec_free(target_flash_t tf)
{
	struct stm_fpec_state *fs = (struct stm_fpec_state *)tf;

	if (fs->fs_applet != NULL)
		flash_applet_detach(fs->fs_applet);

	zone_free(fs);
}

static int
stm_fpec_query_rdp(struct stm_fpec_state *fs, uint32_t *obr)
{

	if ((*obr & STM_FPEC_OBR_OPTERR) != 0)
		*obr = 0xffffffffu;

	if (fs->fs_nt == FLASH_STM_FPEC_OBYTES_TYPE1) {
		/*
		 * These devices have a two-level lockdown.
		 */
		return (*obr & STM_FPEC_OBR1_RDPRT) != 0;
	} else {
		return STM_FPEC_OBR2_RDPRT(*obr);
	}
}

static void
stm_fpec_configure_rdp(struct stm_fpec_state *fs)
{
	uint32_t obr;

	/* All devices have some form of protection. */
	fs->fs_target->t_flags |= TARGET_FLAG_SUPPORTS_PROTECTION;

	if (stm_fpec_read_reg(fs, STM_FPEC_OBR, &obr) < 0) {
		DBFPRINTF("Failed to read OBR\n");
		return;
	}

	DBFPRINTF("OBR 0x%08" PRIx32 "\n", obr);

	if (stm_fpec_query_rdp(fs, &obr) > 0)
		fs->fs_target->t_flags |= TARGET_FLAG_IS_LOCKED;

	if (fs->fs_nt == FLASH_STM_FPEC_OBYTES_TYPE2) {
		/*
		 * These devices have a three-level lockdown.
		 */
		fs->fs_target->t_flags |= TARGET_FLAG_SUPPORTS_PERM_PROT;
	}
}

void *
flash_stm_fpec_attach(target_t t, enum flash_stm_fpec_nvm_type nt,
    uint32_t bank1_base, uint32_t bank2_base,
    uint16_t page_size, uint16_t pages)
{
	struct stm_fpec_state *fs;
	unsigned int flags;
	void *flash_applet;

	if (nt == FLASH_STM_FPEC_FLASH) {
		struct flash_applet_params fp;
		fp.fp_applet = stm_fpec_flash_applet;
		fp.fp_applet_size = sizeof(stm_fpec_flash_applet);
		fp.fp_applet_stack_size = 0;
		fp.fp_max_page_size = page_size;
		flash_applet = flash_applet_attach(t, &fp);
	} else {
		flash_applet = NULL;
		bank2_base = 0;
	}

	if ((fs = zone_calloc(1, sizeof(*fs))) == NULL) {
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		return NULL;
	}

	fs->fs_target = t;
	fs->fs_applet = flash_applet;
	fs->fs_nt = nt;
	fs->fs_base = bank1_base;
	fs->fs_pages = pages;
	fs->fs_bank2_base = bank2_base;
	fs->fs_tf.tf_size = page_size * pages;
	fs->fs_tf.tf_max_page_size = page_size;
	fs->fs_tf.tf_erase_byte = 0xffu;
	fs->fs_tf.tf_sector_desc = stm_fpec_sector_desc;
	fs->fs_tf.tf_erase_sector = stm_fpec_erase_sector;
	fs->fs_tf.tf_write_page = stm_fpec_write_page;
	fs->fs_tf.tf_finish = stm_fpec_finish;
	fs->fs_tf.tf_free = stm_fpec_free;

	flags = (nt == FLASH_STM_FPEC_FLASH) ? 0 : TARGET_MEM_OPTIONS;

	DBFPRINTF("base1 0x%08" PRIx32 ", base2 0x%08" PRIx32 ", size "
	    "0x%08" PRIx32 ", page size 0x%" PRIx16 "\n", bank1_base,
	    bank2_base, fs->fs_tf.tf_size, page_size);

	if (target_add_flash(t, &fs->fs_tf, (target_addr_t)bank1_base,
	    (uint32_t)page_size * (uint32_t)pages, NULL, flags) < 0){
		DBFPRINTF("failed to add flash @ %08" PRIx32 "\n", bank1_base);
		if (flash_applet != NULL)
			flash_applet_detach(flash_applet);
		zone_free(fs);
		return NULL;
	}

	if (nt != FLASH_STM_FPEC_FLASH)
		stm_fpec_configure_rdp(fs);

	return fs;
}

static const char stm_fpec_rdp_query_text[] =
"Possible level values:\n"
"\t0  Device is unlocked. RDP is set to 0xA5.\n"
"\t1  Device is locked. RDP is set to a value other than 0xA5.\n"
"Target must be reset after changing level.\n";

int
flash_stm_fpec_rdp_query(void *arg, struct target_protection_info *pi)
{
	struct stm_fpec_state *fs = arg;
	uint32_t obr;

	if (stm_fpec_read_reg(fs, STM_FPEC_OBR, &obr) < 0)
		return -1;

	pi->pi_level = stm_fpec_query_rdp(fs, &obr);

	if (fs->fs_nt == FLASH_STM_FPEC_OBYTES_TYPE1) {
		pi->pi_max_level = 1;
		pi->pi_text = stm_fpec_rdp_query_text;
	} else {
		pi->pi_max_level = 2;
		pi->pi_text = stm_efmi_rdp_query_text;	/* Re-use EFMI text. */
	}

	return TARGET_PI_OK;
}

static int
flash_stm_fpec_write_ob(struct stm_fpec_state *fs, const uint16_t *ob, FILE *os)
{
	struct target_flash_sector tfs;
	target_addr_t off;
	int rv;

	DBPRINTF("Options: %04" PRIx16 " %04" PRIx16 " %04" PRIx16
	    " %04" PRIx16 " %04" PRIx16 " %04" PRIx16 " %04" PRIx16
	    " %04" PRIx16 "\n",
	    ob[0], ob[1], ob[2], ob[3], ob[4], ob[5], ob[6], ob[7]);

	/* Fetch descriptor for option byte sector */
	off = 0;
	(void) stm_fpec_sector_desc(&fs->fs_tf, &off, &tfs);

	/* Erase option bytes. */
	rv = stm_fpec_erase_sector(&fs->fs_tf, &tfs);
	if (rv < 0 && os != NULL)
		fprintf(os, "Failed to erase option bytes\n");

	/* If that worked, write the updated option bytes. */
	if (rv == 0) {
		if (ob[0] == STM_FPEC_OB16(STM_FPEC_RDP1_LEVEL_0) ||
		    ob[0] == STM_FPEC_OB16(STM_FPEC_RDP2_LEVEL_0)) {
			fprintf(os, "Mass erase in progress. Please wait...\n");
		}

		rv = stm_fpec_write_page(&fs->fs_tf, &tfs, 0);
		if (rv < 0 && os != NULL)
			fprintf(os, "Failed to write new option bytes\n");
	}

	stm_fpec_finish(&fs->fs_tf);

	return rv;
}

int
flash_stm_fpec_rdp_set(void *arg, struct target_protection_info *pi)
{
	struct stm_fpec_state *fs = arg;
	uint32_t obr, wrpr;
	u_int cur_level;
	uint16_t *ob;
	int rv;

	if (pi->pi_level > pi->pi_max_level)
		return -1;

	/* Fetch the current option bytes */
	if (stm_fpec_read_reg(fs, STM_FPEC_OBR, &obr) < 0 ||
	    stm_fpec_read_reg(fs, STM_FPEC_WRPR, &wrpr) < 0) {
		return -1;
	}

	/* Short circuit if there's no change. */
	cur_level = stm_fpec_query_rdp(fs, &obr);
	if (cur_level == pi->pi_level)
		return TARGET_PI_UNCHANGED;
	if (cur_level > pi->pi_level)
		return TARGET_PI_USE_UNLOCK;

	/* Fabricate the new option bytes */
	ob = (uint16_t *)(void *)fs->fs_tf.tf_buffer;

	/* Raise the level, preserving other bits. */
	if (fs->fs_nt == FLASH_STM_FPEC_OBYTES_TYPE1) {
		ob[0] = STM_FPEC_OB16(STM_FPEC_RDP1_LEVEL_1);
		ob[1] = STM_FPEC_OB16(STM_FPEC_OBR1_USER(obr));
		ob[2] = STM_FPEC_OB16(STM_FPEC_OBR1_DATA0(obr));
		ob[3] = STM_FPEC_OB16(STM_FPEC_OBR1_DATA1(obr));
	} else {
		if (pi->pi_level == 2) {
			if (pi->pi_permanent == 0)
				return TARGET_PI_NEED_PERM_FLAG;
			ob[0] = STM_FPEC_OB16(STM_FPEC_RDP2_LEVEL_2);
		} else {
			ob[0] = STM_FPEC_OB16(STM_FPEC_RDP2_LEVEL_1);
		}
		ob[1] = STM_FPEC_OB16(STM_FPEC_OBR2_USER(obr));
		ob[2] = STM_FPEC_OB16(STM_FPEC_OBR2_DATA0(obr));
		ob[3] = STM_FPEC_OB16(STM_FPEC_OBR2_DATA1(obr));
	}

	ob[4] = STM_FPEC_OB16(STM_FPEC_WRPR_BITS(0, wrpr));
	ob[5] = STM_FPEC_OB16(STM_FPEC_WRPR_BITS(1, wrpr));
	ob[6] = STM_FPEC_OB16(STM_FPEC_WRPR_BITS(2, wrpr));
	ob[7] = STM_FPEC_OB16(STM_FPEC_WRPR_BITS(3, wrpr));

	rv = flash_stm_fpec_write_ob(fs, ob, pi->pi_out_file);

	if (rv < 0)
		return rv;

	return TARGET_PI_OK_POWER_CYCLE;
}

int
flash_stm_fpec_rdp_clear(void *arg, FILE *os)
{
	struct stm_fpec_state *fs = arg;
	uint32_t obr;
	uint16_t *ob;

	/* Fetch the current option byte */
	if (stm_fpec_read_reg(fs, STM_FPEC_OBR, &obr) < 0)
		return -1;

	/* Short circuit if there's no change. Should never happen. */
	if (stm_fpec_query_rdp(fs, &obr) == 0)
		return TARGET_PI_UNCHANGED;

	/* Fabricate the new option bytes */
	ob = (uint16_t *)(void *)fs->fs_tf.tf_buffer;

	/* Set RDP disable protection. */
	if (fs->fs_nt == FLASH_STM_FPEC_OBYTES_TYPE1)
		ob[0] = STM_FPEC_OB16(STM_FPEC_RDP1_LEVEL_0);
	else
		ob[0] = STM_FPEC_OB16(STM_FPEC_RDP2_LEVEL_0);

	/* Erase all other bits. */
	ob[1] = STM_FPEC_OB16(0xffu);
	ob[2] = STM_FPEC_OB16(0xffu);
	ob[3] = STM_FPEC_OB16(0xffu);
	ob[4] = STM_FPEC_OB16(0xffu);
	ob[5] = STM_FPEC_OB16(0xffu);
	ob[6] = STM_FPEC_OB16(0xffu);
	ob[7] = STM_FPEC_OB16(0xffu);

	if (flash_stm_fpec_write_ob(fs, ob, os) < 0)
		return -1;

	return TARGET_PI_OK_POWER_CYCLE;
}
