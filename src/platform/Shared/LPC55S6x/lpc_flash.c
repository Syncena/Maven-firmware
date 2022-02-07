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
#include <stdint.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "timer.h"
#include "lpc_flash.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

#define	FMC_FLASH_BYTES_PER_WORD	0x10u

#define	 FMC_CMD_INIT			0u
#define	 FMC_CMD_POWERDOWN		1u
#define	 FMC_CMD_SET_READ_MODE		2u
#define	 FMC_CMD_READ_SINGLE_WORD	3u
#define	 FMC_CMD_ERASE_RANGE		4u
#define	 FMC_CMD_BLANK_CHECK		5u
#define	 FMC_CMD_MARGIN_CHECK		6u
#define	 FMC_CMD_CHECKSUM		7u
#define	 FMC_CMD_WRITE			8u
#define	 FMC_CMD_WRITE_PROG		10u
#define	 FMC_CMD_PROGRAM		12u
#define	 FMC_CMD_REPORT_ECC		13u

#define	FMC_INT_STATUS_ERRORS		(FLASH_INT_STATUS_FAIL_MASK | \
					 FLASH_INT_STATUS_ERR_MASK  | \
					 FLASH_INT_STATUS_ECC_ERR_MASK)
#define	FMC_INT_STATUS_ANY		(FLASH_INT_STATUS_DONE_MASK | \
					 FMC_INT_STATUS_ERRORS)
#define	FMC_WORD_ADDR(a)		FLASH_STARTA_STARTA((a) >> 4)

static FAST_RAMFUNC uint32_t
fmc_issue_command(uint32_t cmd)
{
	rtos_saved_ipl_t ipl;
	uint32_t status;

	ipl = rtos_ipl_raise(HW_IPL_MAX);

#if (F_CPU > 100000000)
	/*
	 * We rely on platform code configuring FRO 96M on MAINCLKSELA.
	 * Simply switch MAINCLKSELB to it.
	 */
	uint32_t main_b = SYSCON->MAINCLKSELB;
	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(0);
	/* Now running at 96 MHz. */
#endif

	uint32_t fmccr;
	fmccr = SYSCON->FMCCR;
	SYSCON->FMCCR = fmccr & ~SYSCON_FMCCR_PREFEN_MASK;

	FLASH->CMD = cmd;
	for (unsigned int i = 0; i < 2000000; i++) {
		status = FLASH->INT_STATUS;
		if ((status & FMC_INT_STATUS_ANY) != 0)
			break;
		timer_buzz_delay_us(1u);
	}

	if ((status & FMC_INT_STATUS_ANY) == 0)
		FLASH->EVENT = FLASH_EVENT_ABORT(1); /* Timeout. */
	else
		FLASH->INT_CLR_STATUS = status;

	SYSCON->FMCCR = fmccr;

#if (F_CPU > 100000000)
	/* Back to the original clock source. */
	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(main_b);
#endif

	rtos_ipl_restore(ipl);

	return status;
}

int
lpc_flash_program_page(uint32_t addr, const void *src)
{
	uint32_t st;

	DBFPRINTF("addr 0x%08" PRIx32 "\n", addr);

	assert(addr < (FLASH_CFPA_SCRATCH_BASE & ~0x10000000u));

	FLASH->STARTA = FMC_WORD_ADDR(addr);
	FLASH->STOPA = FMC_WORD_ADDR(addr);
	st = fmc_issue_command(FMC_CMD_ERASE_RANGE);
	if ((st & FLASH_INT_STATUS_DONE_MASK) == 0) {
		DBFPRINTF("Erase failed (st 0x%08" PRIx32 ").\n", st);
		return -1;
	}

	/* Write the page. */
	for (uint32_t w = 0; w < LPC_FLASH_BYTES_PER_PAGE;
	    w += FMC_FLASH_BYTES_PER_WORD) {
		uint32_t *data = (uint32_t *)((uintptr_t)src + w);

		FLASH->STARTA = FMC_WORD_ADDR(addr + w);

		for (unsigned int i = 0;
		    i < (FMC_FLASH_BYTES_PER_WORD / sizeof(uint32_t)); i++) {
			FLASH->DATAW[i] = data[i];
		}

		st = fmc_issue_command(FMC_CMD_WRITE_PROG);
		if ((st & FLASH_INT_STATUS_DONE_MASK) == 0) {
			DBFPRINTF("Write word 0x%" PRIx32 " failed\n", w);
			return -1;
		}
	}

	DBFPRINTF("Success.\n");
	return 0;
}
