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
#include <stdint.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"

#define	FLASH_ALL_ERRORS	(NVMCTRL_INTFLAG_ADDRE | \
				 NVMCTRL_INTFLAG_PROGE | \
				 NVMCTRL_INTFLAG_LOCKE | \
				 NVMCTRL_INTFLAG_ECCSE | \
				 NVMCTRL_INTFLAG_ECCDE | \
				 NVMCTRL_INTFLAG_NVME)

#define	NVM_RESERVED		64u
#define	NVM_PAGE_SIZE		16u

static uint32_t page_buffer[NVMCTRL_PAGE_SIZE / sizeof(uint32_t)];

static int maven_flash_write_page(uint16_t cmd, uint32_t addr, const uint32_t *src,
		unsigned int page_size);
static int maven_flash_send_command(uint16_t cmd, uint32_t addr);

static int
maven_flash_send_command(uint16_t cmd, uint32_t addr)
{
	uint16_t flag;

	do {
		flag = NVMCTRL->STATUS.reg;
	} while ((flag & NVMCTRL_STATUS_READY) == 0);

	NVMCTRL->ADDR.reg = NVMCTRL_ADDR_ADDR(addr);
	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | cmd;
	__DSB();
	__ISB();

	do {
		flag = NVMCTRL->INTFLAG.reg;
	} while ((flag & NVMCTRL_INTFLAG_DONE) == 0);

	NVMCTRL->INTFLAG.reg = flag;

	return (flag & FLASH_ALL_ERRORS) ? -1 : 0;
}

static int
maven_flash_write_page(uint16_t cmd, uint32_t addr, const uint32_t *src,
    unsigned int page_size)
{
	volatile uint32_t *dest = (volatile uint32_t *)(uintptr_t)addr;
	unsigned int count = page_size / sizeof(*dest);

	for (unsigned int i = 0; i < count; i++)
		dest[i] = src[i];

	__ISB();
	__DSB();

	return maven_flash_send_command(cmd, addr);
}

int
platform_flash_program(uint32_t page, const void *src, uint32_t len)
{
	rtos_saved_ipl_t ipl;
	uint32_t addr;
	int rv = 0;

	assert(len <= NVMCTRL_PAGE_SIZE);

	addr = page * NVMCTRL_PAGE_SIZE;
	if ((addr % NVMCTRL_BLOCK_SIZE) == 0) {
		ipl = rtos_ipl_raise(HW_IPL_MAX);
		rv = maven_flash_send_command(NVMCTRL_CTRLB_CMD_EB, addr);
		if (rv == 0) {
			rv = maven_flash_send_command(
			    NVMCTRL_CTRLB_CMD_PBC, 0);
		}
		rtos_ipl_restore(ipl);

		if (rv != 0)
			return rv;
	}

	if (((uintptr_t)src & (sizeof(uint32_t) - 1)) != 0 ||
	    len < NVMCTRL_PAGE_SIZE) {
		if (len < NVMCTRL_PAGE_SIZE)
			memset(page_buffer, 0xff, sizeof(page_buffer));
		memcpy(page_buffer, src, len);
		src = page_buffer;
	}

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	rv = maven_flash_write_page(NVMCTRL_CTRLB_CMD_WP, addr, src,
	    NVMCTRL_PAGE_SIZE);
	rtos_ipl_restore(ipl);

	return rv;
}

struct maven_nvm_params {
	uint32_t np_dest;
	const uint8_t *np_src;
	unsigned int np_pages;
};

static int
maven_nvm_write(const struct maven_nvm_params *np)
{
	const uint32_t *src;
	uint32_t dest;
	unsigned int page;
	int rv = 0;

	dest = np->np_dest;
	src = (const uint32_t *)(uintptr_t)np->np_src;

	for (page = 0; page < np->np_pages; page++) {
		rv = maven_flash_write_page(NVMCTRL_CTRLB_CMD_WQW, dest, src,
		    NVM_PAGE_SIZE);
		if (rv < 0)
			break;

		dest += NVM_PAGE_SIZE;
		src += NVM_PAGE_SIZE / sizeof(*src);
	}

	return rv;
}

int
maven_nvm_program(uint32_t addr, const void *buff, size_t len)
{
	uint32_t bounce[NVM_PAGE_SIZE / sizeof(uint32_t)];
	struct maven_nvm_params np;
	rtos_saved_ipl_t ipl;
	int rv;

	assert(addr >= (NVMCTRL_USER + NVM_RESERVED));
	assert((addr + len) <= (NVMCTRL_USER + FLASH_USER_PAGE_SIZE));
	assert((addr % NVM_PAGE_SIZE) == 0);
	assert(len > 0 && (len % NVM_PAGE_SIZE) == 0);

	np.np_pages = len / NVM_PAGE_SIZE;
	np.np_dest = addr;
	if (((uintptr_t)buff % sizeof(uint32_t)) == 0) {
		np.np_src = (const uint8_t *)(uintptr_t)buff;
	} else {
		memcpy(bounce, buff, len);
		np.np_src = (const uint8_t *)bounce;
	}

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	rv = maven_nvm_write(&np);
	rtos_ipl_restore(ipl);

	return rv;
}
