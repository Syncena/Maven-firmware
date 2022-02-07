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
#include <string.h>

#include "sam-efc.h"
#include "globals.h"
#include "timer.h"

#include "efc.c"

#define	USER_SIGNATURE_WORDS	(512 / sizeof(uint32_t))
static uint32_t user_signature_cache[USER_SIGNATURE_WORDS];
static uint32_t user_sig_free_words, user_sig_current_idx;
static uint32_t user_sig_status;
static uint32_t user_sig_writes;
static uint32_t user_sig_erases;

#define	USER_SIG_GOOD		0
#define	USER_SIG_READ_FAILED	1
#define	USER_SIG_ERASE_FAILED	2
#define	USER_SIG_WRITE_FAILED	3

#define	UNIQUE_IDENTIFIER_WORDS	(16 / sizeof(uint32_t))

static __always_inline void
efc_barrier(void)
{

	__asm __volatile("\tdmb\n\tdsb\nisb\n" ::: "memory");
#ifdef CMCC
	CMCC->CMCC_MAINT0 = CMCC_MAINT0_INVALL;
	__asm __volatile("\tdmb\n\tdsb\nisb\n" ::: "memory");
#endif
}

static void
glob_read_user_sig(void)
{
	uint32_t rv, used_words, *first_blank;
	rtos_saved_ipl_t ipl;

#ifdef CMCC
	CMCC->CMCC_MAINT0 = CMCC_MAINT0_INVALL;
#endif

	/*
	 * Load the user signature area into our local cache.
	 */
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif
	rv = efc_perform_read_sequence(EFC0, EFC_FCMD_STUS,
	    EFC_FCMD_SPUS, user_signature_cache, USER_SIGNATURE_WORDS);
#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	efc_barrier();
	rtos_ipl_restore(ipl);

	if (rv != EFC_RC_OK) {
		user_sig_status = USER_SIG_READ_FAILED;
		return;
	}

	user_sig_status = USER_SIG_GOOD;

	/*
	 * Search for the last unwritten word, starting from the
	 * end of the user signature area.
	 */
	first_blank = &user_signature_cache[USER_SIGNATURE_WORDS - 1];

	while (*first_blank == 0xfffffffful &&
	    first_blank >= user_signature_cache) {
		first_blank -= 1;
	}

	/*
	 * Note: this may point to the location *after* the end of the
	 * user signature area, if there are no blank words.
	 */
	first_blank += 1;

	/*
	 * We maintain the erase count in the first word, so skip if the
	 * flash is completely blank.
	 */
	if (first_blank == user_signature_cache)
		first_blank += 1;

	/*
	 * How many words are free.
	 */
	used_words = (uint32_t)(first_blank - user_signature_cache);
	user_sig_free_words = USER_SIGNATURE_WORDS - used_words;
	user_sig_erases = user_signature_cache[0] + 1;
}

uint8_t
glob_read_storage(struct global_vars *gv)
{

	/* Grab the user signature area */
	glob_read_user_sig();

	if (user_sig_status != USER_SIG_GOOD)
		return GLOB_RDWR_ERROR;

	user_sig_current_idx = 1;
	memcpy(gv, &user_signature_cache[1], sizeof(*gv));

	return GLOB_RDWR_OK;
}

uint8_t
glob_erase_storage(void)
{
	rtos_saved_ipl_t ipl;
	uint32_t rv;

	/*
	 * Need to erase the whole user signature area
	 */
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif
	rv = efc_perform_command(EFC0, EFC_FCMD_EUS, 0);
#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	efc_barrier();
	rtos_ipl_restore(ipl);

	if (rv != 0) {
		user_sig_status = USER_SIG_ERASE_FAILED;
		return GLOB_RDWR_ERROR;
	}

	memset(&user_signature_cache[1], 0xff,
	    sizeof(user_signature_cache) - 4);
	user_sig_free_words = USER_SIGNATURE_WORDS - 1;
	user_sig_current_idx = 1;
	user_signature_cache[0] = user_sig_erases;
	user_sig_erases += 1;

	return GLOB_RDWR_OK;
}

uint8_t
glob_write_storage(const struct global_vars *gv)
{
	uint32_t rv, need_words, first_blank, *src, *dst;
	rtos_saved_ipl_t ipl;

	if (user_sig_status != USER_SIG_GOOD)
		return GLOB_RDWR_ERROR;

	/* How many words needed, rounded up to nearest word */
	need_words = ((sizeof(*gv) + 3) & ~3ul) / sizeof(uint32_t);

	if (need_words > user_sig_free_words) {
		uint8_t erv = glob_erase_storage();
		if (erv != GLOB_RDWR_OK)
			return erv;
	}

	first_blank = USER_SIGNATURE_WORDS - user_sig_free_words;

	/* Just return if there's no change */
	if (memcmp(&user_signature_cache[user_sig_current_idx], gv,
	    sizeof(*gv)) == 0) {
		return GLOB_RDWR_OK;
	}

	timer_buzz_delay_ms(50);

	ipl = rtos_ipl_raise(HW_IPL_MAX);
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif
	/* Otherwise copy the new data in place */
	memcpy(&user_signature_cache[first_blank], gv, sizeof(*gv));

	/* Write it to the staging area */
	for (src = user_signature_cache, dst = (uint32_t *)IFLASH0_ADDR;
	    src < &user_signature_cache[USER_SIGNATURE_WORDS]; ) {
		*dst++ = *src++;
	}

	efc_barrier();
	rv = efc_perform_command(EFC0, EFC_FCMD_WUS, 0);
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	rtos_ipl_restore(ipl);

	if (rv != 0) {
		user_sig_status = USER_SIG_WRITE_FAILED;
		return GLOB_RDWR_ERROR;
	}

	user_sig_writes++;
	glob_read_user_sig();

	return GLOB_RDWR_OK;
}

#ifdef GLOBAL_OPT_CMD
void
glob_show_storage(FILE *fp)
{
	const char *st;

	switch (user_sig_status) {
	case USER_SIG_GOOD:
		st = "Operational";
		break;
	case USER_SIG_READ_FAILED:
		st = "Read Failure";
		break;
	case USER_SIG_ERASE_FAILED:
		st = "Erase Failure";
		break;
	case USER_SIG_WRITE_FAILED:
		st = "Write Failure";
		break;
	default:
		st = "Memory corruption?";
		break;
	}

	fprintf(fp, "Storage space status: %s\n", st);

	if (user_sig_status != USER_SIG_GOOD)
		return;

	fprintf(fp, "Storage space used: %" PRIu32 " bytes\n",
	    (USER_SIGNATURE_WORDS - user_sig_free_words) * sizeof(uint32_t));
	fprintf(fp, "Storage space free: %" PRIu32 " bytes\n",
	    user_sig_free_words * sizeof(uint32_t));
	fprintf(fp, "Storage writes this session: %" PRIu32 "\n",
	    user_sig_writes);
	fprintf(fp, "Storage lifetime erases: %" PRIu32 "\n", user_sig_erases);
}
#endif /* GLOBAL_OPT_CMD */

void
sam_efc_init(uint32_t wait_states)
{

	/*
	 * Set flash wait states
	 */
	efc_write_fmr(EFC0, EEFC_FMR_FWS(wait_states) | EEFC_FMR_CLOE);
}

void
sam_efc_wipe_user_area(void)
{

	/*
	 * Erase the whole user signature area, including the erase counter.
	 * Used after testing a new board before it is ready for shipping.
	 */
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif
	(void) efc_perform_command(EFC0, EFC_FCMD_EUS, 0);
#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	efc_barrier();
}

int
sam_efc_program_page(uint32_t page, const void *psrc, int lock)
{
	uint32_t fsr, cmd, *dest;
	const uint32_t *src;
	rtos_saved_ipl_t ipl;

#if 0
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	/*
	 * Clear the lock bit
	 */
	(void) EFC0->EEFC_FSR;
	efc_barrier();
	cmd = EEFC_FCR_FCMD_CLB;
	fsr = efc_perform_command(EFC0, cmd, page);
	efc_barrier();
	rtos_ipl_restore(ipl);

	if (fsr != 0)
		return -1;
#endif

	/*
	 * If on an 8-page boundary, we can erase 8 pages
	 */
	if ((page % 8) == 0) {
		/*
		 * The pages may already be erased. Do a quick check
		 */
		dest = (uint32_t *)((page * IFLASH0_PAGE_SIZE) + IFLASH0_ADDR);
		fsr = (IFLASH0_PAGE_SIZE / sizeof(*dest)) * 8;
		while (fsr--) {
			if (*dest != 0xffffffff)
				break;
		}

		if (fsr != 0) {
			/*
			 * Erase the next block of 8 pages
			 */
			ipl = rtos_ipl_raise(HW_IPL_MAX);
			cmd = EEFC_FCR_FCMD_EPA;
#ifdef SAM_EFC_OPT_MPU_BEGIN
			SAM_EFC_OPT_MPU_BEGIN();
#endif
			fsr = efc_perform_command(EFC0, cmd, page | 1);
#ifdef SAM_EFC_OPT_MPU_END
			SAM_EFC_OPT_MPU_END();
#endif
			efc_barrier();
			rtos_ipl_restore(ipl);

			if (fsr != 0)
				return -1;
		}
	}

	/*
	 * We can return here if writing a blank page
	 */
	src = psrc;
	fsr = IFLASH0_PAGE_SIZE / sizeof(*dest);
	while (fsr--) {
		if (*src != 0xffffffff)
			break;
	}

	if (fsr == 0)
		return 0;

	src = psrc;
	fsr = IFLASH0_PAGE_SIZE / sizeof(*dest);
	dest = (uint32_t *)(uintptr_t)IFLASH0_ADDR;
	cmd = lock ? EEFC_FCR_FCMD_WPL : EEFC_FCR_FCMD_WP;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif

	/* Write the page to the latch buffer */
	while (fsr--)
		*dest++ = *src++;

	/* Program the page, setting lock bit if necessary */
	efc_barrier();
	fsr = efc_perform_command(EFC0, cmd, page);

#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	efc_barrier();
	rtos_ipl_restore(ipl);

	return (fsr == 0) ? 0 : -1;
}

int
sam_efc_get_unique_id(sam_efc_unique_id_t *uid)
{
	rtos_saved_ipl_t ipl;
	uint32_t rv;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	efc_barrier();
#ifdef SAM_EFC_OPT_MPU_BEGIN
	SAM_EFC_OPT_MPU_BEGIN();
#endif
	rv = efc_perform_read_sequence(EFC0, EFC_FCMD_STUI,
	    EFC_FCMD_SPUI, uid->u32, UNIQUE_IDENTIFIER_WORDS);
#ifdef SAM_EFC_OPT_MPU_END
	SAM_EFC_OPT_MPU_END();
#endif
	efc_barrier();
	rtos_ipl_restore(ipl);

	if (rv != EFC_RC_OK)
		return -1;

	return 0;
}
