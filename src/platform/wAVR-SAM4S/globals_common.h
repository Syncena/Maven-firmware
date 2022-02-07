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

#ifndef GLOBALS_COMMON_H
#define	GLOBALS_COMMON_H

#ifdef CONFIG_USE_CONFIGDB
#error "You should not be including this."
#endif

#define	GLOB_SPLIT_CRC

#include <stdint.h>
#include "timer.h"
#include "network.h"
#if defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
#include "wifi_driver.h"
#define	NETWORK_DRIVER_GLOBAL_VARS	WIFI_GLOBAL_VARS
#else
#define	NETWORK_DRIVER_GLOBAL_VARS	/* empty */
#endif
#include "display.h"
#include "target_console.h"

#define	GLOBAL_VARS_BASE					\
	NETWORK_GLOBAL_VARS					\
	NETWORK_DRIVER_GLOBAL_VARS				\
	DISPLAY_GLOBAL_VARS					\
	TARGET_CONSOLE_GLOBAL_VARS

#if (RELEASE_BUILD == 0)
#define	GLOBAL_OPT_CMD
#endif

#ifdef GLOBAL_OPT_CMD
#include <stdio.h>
#endif

#define	GLOB_MAGIC	0xbabe
#define	GLOB_VERSION	0x10

struct global_vars {
	uint16_t	glob_crc;
	uint16_t	glob_magic;
	uint16_t	glob_size;
	uint8_t		glob_version;

#ifndef GLOB_SPLIT_CRC
	GLOBAL_VARS
#else
	GLOBAL_VARS_BASE

	uint16_t	glob_crc2;
	uint16_t	glob_size2;

	GLOBAL_VARS_MAIN
#endif /* GLOB_SPLIT_CRC */
};

extern struct global_vars glob;

extern int glob_init(void);
extern uint8_t glob_save(void);

#ifdef GLOB_SPLIT_CRC
extern void glob_app_defaults(uint8_t);	/* Provided by application */
#else
extern void glob_app_defaults(void);	/* Provided by application */
#endif

/* Low-level routines provided by App to read/write persistant storage */
extern uint8_t glob_read_storage(struct global_vars *);
extern uint8_t glob_write_storage(const struct global_vars *);
extern uint8_t glob_erase_storage(void);
#define	GLOB_RDWR_OK		0
#define	GLOB_RDWR_ERROR		1
#define	GLOB_RDWR_MAIN_ERROR	2

#ifdef GLOBAL_OPT_CMD
extern void glob_show_storage(FILE *fp);
#endif

#endif /* GLOBALS_COMMON_H */
