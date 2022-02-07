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
#include <string.h>

#include "rtos.h"
#include "globals.h"
#ifdef GLOBAL_OPT_CMD
#include "shell.h"
#endif

struct global_vars glob;

#ifdef GLOB_SPLIT_CRC
#define	GLOB_BASE_SIZE	(offsetof(struct global_vars, glob_crc2))
#define	GLOB_MAIN_SIZE	(sizeof(struct global_vars) - GLOB_BASE_SIZE)
#else
#define	GLOB_BASE_SIZE	sizeof(struct global_vars)
#define	GLOB_MAIN_SIZE	sizeof(struct global_vars)
#endif

#ifdef GLOBAL_OPT_CMD
SHELL_CMD_DECL(glob, glob_cmd, "Load/Save persistant global variables");
#endif

#ifndef	GLOB_OP_READ_ONLY
static struct global_vars glob_stash;
static rtos_mutex_t glob_mutex;

static void
glob_lock(void)
{

	rtos_mutex_acquire(glob_mutex);
}

static void
glob_unlock(void)
{

	rtos_mutex_release(glob_mutex);
}
#endif /* GLOB_OP_READ_ONLY */

static uint16_t
glob_calc_crc(int which, const struct global_vars *gv)
{
	const uint8_t *p;
	uint16_t crc, len;
	uint8_t bit;

#ifndef GLOB_SPLIT_CRC
	(void) which;
	p = (const uint8_t *)(const void *)gv;
	len = sizeof(*gv);
#else
	if (which == 0) {
		p = (const uint8_t *)(const void *)gv;
		len = GLOB_BASE_SIZE - sizeof(crc);
	} else {
		p = (const uint8_t *)(const void *)gv;
		p = &p[GLOB_BASE_SIZE];
		len = GLOB_MAIN_SIZE;
	}
#endif

	p += sizeof(crc);
	len -= sizeof(crc);
	crc = 0xffffu;

	while (len--) {
		crc ^= ((uint16_t)*p++) << 8;

		for (bit = 0; bit < 8; bit++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x5031;
			else
				crc = (crc << 1);
		}
	}

	return crc;
}

static uint8_t
glob_read(FILE *fp, struct global_vars *gv)
{
	uint16_t crc;
	uint8_t rv;

#if (RELEASE_BUILD != 0)
	(void) fp;
#endif

	/* Fetch from device/app-specific storage */
	if ((rv = glob_read_storage(gv)) != GLOB_RDWR_OK) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL)
			fprintf(fp, "Failed to read from storage\n");
#endif
		return rv;
	}

	/*
	 * Validate them - persistant storage can go bad, or this may be
	 * first bring-up on a new CPU...
	 */
	if (gv->glob_magic != GLOB_MAGIC || gv->glob_version != GLOB_VERSION) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL) {
			fprintf(fp, "Magic/Version mismatch (%04" PRIx16
			    " / %04" PRIx16 ")\n", gv->glob_magic,
			    gv->glob_version);
		}
#endif
		return GLOB_RDWR_ERROR;
	}

#ifdef GLOB_SPLIT_CRC
	if (gv->glob_size != GLOB_BASE_SIZE) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL) {
			fprintf(fp, "Base size (%" PRIu16 ") != %u\n",
			    gv->glob_size, (unsigned int)GLOB_BASE_SIZE);
		}
#endif
		return GLOB_RDWR_ERROR;
	}

	if ((crc = glob_calc_crc(0, gv)) != gv->glob_crc) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL) {
			fprintf(fp, "Base CRC (%04" PRIx16 ") != %04" PRIx16
			    "\n", gv->glob_crc, crc);
		}
#else
		(void) crc;
#endif
		return GLOB_RDWR_ERROR;
	}

	if (gv->glob_size2 != GLOB_MAIN_SIZE) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL) {
			fprintf(fp, "Main size (%" PRIu16 ") != %u\n",
			    gv->glob_size2, (unsigned int)GLOB_MAIN_SIZE);
		}
#endif
		return GLOB_RDWR_MAIN_ERROR;
	}

	if ((crc = glob_calc_crc(1, gv)) != gv->glob_crc2) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL) {
			fprintf(fp, "Main CRC (%04" PRIx16 ") != %04" PRIx16
			    "\n", gv->glob_crc2, crc);
		}
#else
		(void) crc;
#endif
		return GLOB_RDWR_MAIN_ERROR;
	}
#else
	if ((crc = glob_calc_crc(0, gv)) != gv->glob_crc) {
#if (RELEASE_BUILD == 0)
		if (fp != NULL)
			fprintf(fp, "CRC (%04" PRIx16 ") != %04" PRIx16 "\n",
			    gv->glob_crc, crc);
#else
		(void) crc;
#endif
		return GLOB_RDWR_ERROR;
	}
#endif

	/* Global vars look good */
	return GLOB_RDWR_OK;
}

#ifndef	GLOB_OP_READ_ONLY
static uint8_t
glob_write(struct global_vars *gv)
{
	uint8_t rv;

	gv->glob_crc = glob_calc_crc(0, gv);

#ifdef GLOB_SPLIT_CRC
	gv->glob_crc2 = glob_calc_crc(1, gv);
#endif

	if ((rv = glob_read_storage(&glob_stash)) != GLOB_RDWR_OK)
		return rv;

	/*
	 * Avoid writing if there's no change
	 */
	if (memcmp(gv, &glob_stash, sizeof(*gv)) == 0)
		return GLOB_RDWR_OK;

	return glob_write_storage(gv);
}
#endif

static void
glob_load_defaults(uint8_t full)
{

	if (full) {
		memset(&glob, 0, sizeof(glob));
		glob.glob_magic = GLOB_MAGIC;
		glob.glob_version = GLOB_VERSION;
		glob.glob_size = GLOB_BASE_SIZE;
#ifdef GLOB_SPLIT_CRC
		glob_app_defaults(0);
#endif
	}

#ifndef GLOB_SPLIT_CRC
	glob_app_defaults();
#else
	memset(&((char *)&glob)[GLOB_BASE_SIZE], 0, GLOB_MAIN_SIZE);
	glob.glob_size2 = GLOB_MAIN_SIZE;
	glob_app_defaults(1);
#endif
}

int
glob_init(void)
{
	uint8_t rv;
	int erased = 0;

#ifdef GLOBAL_OPT_CMD
	SHELL_CMD_ADD(glob);
#endif /* GLOBAL_OPT_CMD */

#ifndef	GLOB_OP_READ_ONLY
	glob_mutex = rtos_mutex_create();
	assert(glob_mutex != NULL);
#endif /* GLOB_OP_READ_ONLY */

	if ((rv = glob_read(NULL, &glob)) != GLOB_RDWR_OK) {
		const uint32_t *gv = (const uint32_t *)&glob;

		/* Return value must indicate if glob storage is erased. */
		if (gv[0] == 0xffffffffu && gv[1] == 0xffffffffu &&
		    gv[2] == 0xffffffffu && gv[3] == 0xffffffffu) {
			erased = 1;
		}

		/*
		 * Revert to compile-time defaults
		 */
		glob_load_defaults(rv != GLOB_RDWR_MAIN_ERROR);
	}

	return erased;
}

uint8_t
glob_save(void)
{
#ifndef	GLOB_OP_READ_ONLY
	uint8_t rv;

	glob_lock();

	rv = glob_write(&glob);

	if (rv == GLOB_RDWR_OK)
		rv = glob_read(NULL, &glob_stash);

	if (rv == GLOB_RDWR_OK && memcmp(&glob_stash, &glob,
	    sizeof(glob_stash)) != 0) {
		rv = GLOB_RDWR_ERROR;
	}

	glob_unlock();

	return rv;
#else
	return GLOB_RDWR_OK;
#endif
}

#ifdef GLOBAL_OPT_CMD
static void
glob_erase(void)
{
#ifndef	GLOB_OP_READ_ONLY
	glob_lock();

	glob_erase_storage();

	glob_unlock();
#endif
}

static void
glob_cmd(FILE *fp, uint8_t argc, const char * const *argv)
{
	uint8_t rv, rv2;

	if (argc >= 1 && strcasecmp(argv[0], "erase") == 0) {
		glob_erase();
		fprintf(fp, "You might want to reboot now\n");
	} else
	if (argc >= 1 && strcasecmp(argv[0], "save") == 0) {
		glob_save();
	}

	fprintf(fp, "Global variables size: %u bytes\n",
	    (unsigned int)sizeof(glob));

	glob_lock();
	rv = glob_read(fp, &glob_stash);

	rv2 = GLOB_RDWR_OK;
	if (memcmp(&glob_stash, &glob, sizeof(glob_stash)) != 0)
		rv2 = GLOB_RDWR_ERROR;
	glob_unlock();

	if (rv2 != GLOB_RDWR_OK)
		fprintf(fp, "In-memory globals do not match saved!\n");
	fprintf(fp, "Global variables status: %salid\n",
	    (rv == GLOB_RDWR_OK) ? "V" : "Inv");

	glob_show_storage(fp);
}
#endif /* GLOBAL_OPT_CMD */
