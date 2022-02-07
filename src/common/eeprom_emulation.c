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
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "eeprom_emulation.h"
#include "configdb.h"
#include "crypto_crc32.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

struct eeprom_page {
	uint32_t ep_crc;
	uint32_t ep_magic;
#define	EEPROM_PAGE_MAGIC	0x45455045	/* EEPE */
	uint32_t ep_generation;
#define	EEPROM_GEN_VIRGIN	0u
	uint32_t ep_size;
	uint8_t ep_data[];
};

struct eeprom_emulation {
	struct eeprom_page *ee_pagebuff;
	unsigned int ee_current_page;
	uint32_t ee_generation;
	size_t ee_size;
	const struct eeprom_emulation_driver *ee_driver;
	void *ee_driver_arg;
	struct configdb_driver ee_cd;
};

static bool
ee_validate_page(struct eeprom_emulation *ee, const struct eeprom_page *ep)
{
	uint32_t crc;

	if (ep->ep_magic != EEPROM_PAGE_MAGIC) {
		DBFPRINTF("Bad magic number %08" PRIx32 "\n", ep->ep_magic);
		return false;
	}

	if (ep->ep_generation == 0) {
		DBFPRINTF("generation is zero!\n");
		return false;
	}

	if ((size_t)ep->ep_size != ee->ee_size) {
		DBFPRINTF("ep_size (%" PRIu32 ") != ee_size (%u)\n",
		    ep->ep_size, (unsigned int)ee->ee_size);
		return false;
	}

	crc = crypto_crc32(&ep->ep_magic, ee->ee_driver->ed_page_size -
	    sizeof(crc));

	if (crc != ep->ep_crc) {
		DBFPRINTF("crc (%08" PRIx32 ") != ep_crc (%08" PRIx32 ")\n",
		    crc, ep->ep_crc);
	}

	return crc == ep->ep_crc;
}

static bool
ee_driver_read_page(struct eeprom_emulation *ee, unsigned int page)
{

	DBFPRINTF("page %u\n", page);

	if (ee->ee_driver->ed_page_read(ee->ee_driver_arg, page,
	    ee->ee_pagebuff) == false) {
		DBFPRINTF("Driver failed to read page.\n");
		return false;
	}

	return ee_validate_page(ee, ee->ee_pagebuff);
}

static bool
ee_driver_write_page(struct eeprom_emulation *ee, unsigned int page,
    const void *buff)
{

	return ee->ee_driver->ed_page_program(ee->ee_driver_arg, page, buff);
}

static size_t
ee_read(void *cookie, void *buff)
{
	struct eeprom_emulation *ee = cookie;

	if (ee->ee_generation == EEPROM_GEN_VIRGIN)
		memset(buff, 0xff, ee->ee_size);
	else
		memcpy(buff, ee->ee_pagebuff->ep_data, ee->ee_size);

	return ee->ee_size;
}

static int
ee_write(void *cookie, const void *buff, size_t len)
{
	struct eeprom_emulation *ee = cookie;
	struct eeprom_page *ep;
	unsigned int new_page;
	int rv;

	assert(len > 0);
	assert(len <= ee->ee_size);

	if ((ep = zone_malloc(ee->ee_driver->ed_page_size)) == NULL)
		return -1;

	memset(ep->ep_data, 0xff, ee->ee_size);
	memcpy(ep->ep_data, buff, len);

	ep->ep_magic = EEPROM_PAGE_MAGIC;
	ep->ep_generation = ee->ee_generation + 1;
	ep->ep_size = (uint32_t)ee->ee_size;
	ep->ep_crc = crypto_crc32(&ep->ep_magic, ee->ee_driver->ed_page_size -
	    sizeof(ep->ep_crc));

	if (ee->ee_generation == EEPROM_GEN_VIRGIN) {
		/* Virgin EEPROM. */
		new_page = 0;
	} else {
		new_page = ee->ee_current_page + 1u;
		new_page %= ee->ee_driver->ed_pages;
	}

	if (ee_driver_write_page(ee, new_page, ep)) {
		memcpy(ee->ee_pagebuff, ep, ee->ee_driver->ed_page_size);
		ee->ee_generation += 1u;
		ee->ee_current_page = new_page;
		rv = 0;
	} else {
		rv = -1;
	}

	zone_free(ep);
	return rv;
}

static void
ee_get_current_page(struct eeprom_emulation *ee)
{
	struct eeprom_page *ep = ee->ee_pagebuff;

	ee->ee_current_page = 0;
	ee->ee_generation = EEPROM_GEN_VIRGIN;

	for (unsigned int p = ee->ee_current_page; p < ee->ee_driver->ed_pages;
	    p++) {
		if (ee_driver_read_page(ee, p) &&
		    ep->ep_generation > ee->ee_generation) {
			ee->ee_generation = ep->ep_generation;
			ee->ee_current_page = p;
		}
	}

	/* Ensure the page buffer contains the most recent generation. */
	if (ee->ee_generation != EEPROM_GEN_VIRGIN)
		(void) ee_driver_read_page(ee, ee->ee_current_page);
}

int
eeprom_emulation_attach(const struct eeprom_emulation_driver *ed, void *arg)
{
	struct eeprom_emulation *ee;

	assert(ed->ed_page_size > sizeof(struct eeprom_page));
	assert((ed->ed_page_size & (ed->ed_page_size - 1u)) == 0u);
	assert(ed->ed_pages > 1u);

	if ((ee = zone_malloc(sizeof(*ee))) == NULL)
		return -1;

	ee->ee_pagebuff = zone_malloc(ed->ed_page_size);
	if (ee->ee_pagebuff == NULL) {
		zone_free(ee);
		return -1;
	}

	ee->ee_size = ed->ed_page_size - sizeof(struct eeprom_page);
	ee->ee_driver = ed;
	ee->ee_driver_arg = arg;
	ee->ee_cd.cd_read = ee_read;
	ee->ee_cd.cd_write = ee_write;
	ee->ee_cd.cd_cookie = ee;

	ee_get_current_page(ee);

	if (configdb_attach_driver(&ee->ee_cd, ee->ee_size) < 0) {
		zone_free(ee->ee_pagebuff);
		zone_free(ee);
		return -1;
	}

	return 0;
}
