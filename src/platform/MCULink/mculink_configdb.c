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
#include "eeprom_emulation.h"
#include "lpc_flash.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

extern int _eeprom_flash_start, _eeprom_flash_end;
#define	FLASH_EE_START_ADDR	((uint32_t)(uintptr_t)&_eeprom_flash_start)
#define	FLASH_EE_END_ADDR	((uint32_t)(uintptr_t)&_eeprom_flash_end)
#define	FLASH_EE_SIZE		(FLASH_EE_END_ADDR - FLASH_EE_START_ADDR)
#define	FLASH_EE_PAGE_SIZE	(2u * LPC_FLASH_BYTES_PER_PAGE)
#define	FLASH_EE_PAGE2ADDR(p)	(((uint32_t)(p) * FLASH_EE_PAGE_SIZE) + \
				 FLASH_EE_START_ADDR)

struct flash_ee_read_ctx {
	const void *rc_src;
	void *rc_dst;
};

static void
flash_ee_copy_page(void *ctx)
{
	struct flash_ee_read_ctx *rc = ctx;

	memcpy(rc->rc_dst, rc->rc_src, FLASH_EE_PAGE_SIZE);
}

static bool
flash_ee_page_read(void *cookie, unsigned int page, void *data)
{
	struct flash_ee_read_ctx rc;
	bool rv;

	(void) cookie;

	/*
	 * Flash memory on this device is ECC protected. Erased or corrupt
	 * pages will generate a hardfault exception on any attempt to
	 * access them. Thus, we must arrange to catch and recover from
	 * a hardfault when reading the page.
	 */
	rc.rc_src = (const void *)(uintptr_t)FLASH_EE_PAGE2ADDR(page);
	rc.rc_dst = data;

	rv = !rtos_catch_fault(flash_ee_copy_page, &rc);

	DBFPRINTF("page %u, src %p, %s\n", page, rc.rc_src,
	    rv ? "Success" : "Fault");

	return rv;
}

static bool
flash_ee_page_program(void *cookie, unsigned int page, const void *data)
{
	uint32_t addr;
	bool rv;

	(void) cookie;

	addr = FLASH_EE_PAGE2ADDR(page);

	for (unsigned int i = 0;
	    i < (FLASH_EE_PAGE_SIZE / LPC_FLASH_BYTES_PER_PAGE); i++) {
		if ((rv = lpc_flash_program_page(addr, data) == 0) == false)
			break;

		addr += LPC_FLASH_BYTES_PER_PAGE;
		data = (const void *)((uintptr_t)data +
		    LPC_FLASH_BYTES_PER_PAGE);
	}

	DBFPRINTF("page %u, dst 0x%08" PRIx32 ", %s\n", page,
	    FLASH_EE_PAGE2ADDR(page), rv ? "Success" : "Failed");

	return rv;
}

void
platform_attach_configdb(void)
{
	static struct eeprom_emulation_driver ed;
	int rv;

	ed.ed_page_size = FLASH_EE_PAGE_SIZE,
	ed.ed_pages = FLASH_EE_SIZE / FLASH_EE_PAGE_SIZE,
	ed.ed_page_read = flash_ee_page_read,
	ed.ed_page_program = flash_ee_page_program,

	rv = eeprom_emulation_attach(&ed, NULL);
	assert(rv == 0);
	(void)rv;
}
