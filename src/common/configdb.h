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

#ifndef CONFIGDB_H
#define CONFIGDB_H

#include <sys/types.h>
#include "cf_registry.h"

typedef uint16_t	cf_id_t;
typedef uint8_t		cf_len_t;

extern cf_len_t cf_get_uint8(cf_id_t, uint8_t *);
extern cf_len_t cf_get_uint16(cf_id_t, uint16_t *);
extern cf_len_t cf_get_uint32(cf_id_t, uint32_t *);
extern cf_len_t cf_get_float(cf_id_t, float *);
extern cf_len_t cf_get_double(cf_id_t, double *);
extern cf_len_t cf_get_string(cf_id_t, size_t, char *);
extern cf_len_t cf_get_data(cf_id_t, size_t, void *);
#if 0
extern cf_len_t cf_get_fix16(cf_id_t, int32_t *);
#endif

extern cf_len_t cf_set_uint8(cf_id_t, uint8_t);
extern cf_len_t cf_set_uint16(cf_id_t, uint16_t);
extern cf_len_t cf_set_uint32(cf_id_t, uint32_t);
extern cf_len_t cf_set_float(cf_id_t, float);
extern cf_len_t cf_set_double(cf_id_t, double);
extern cf_len_t cf_set_string(cf_id_t, const char *);
extern cf_len_t cf_set_data(cf_id_t, size_t, const void *);
#if 0
extern cf_len_t cf_set_fix16(cf_id_t, int32_t);
#endif

extern void cf_del_uint8(cf_id_t);
extern void cf_del_uint16(cf_id_t);
extern void cf_del_uint32(cf_id_t);
extern void cf_del_float(cf_id_t);
extern void cf_del_double(cf_id_t);
extern void cf_del_string(cf_id_t);
extern void cf_del_data(cf_id_t);
#if 0
extern void cf_del_fix16(cf_id_t);
#endif

struct configdb_driver {
	/* Read the entire DB from persistant storage. */
	size_t (*cd_read)(void *cookie, void *buff);

	/* Write to the persistant storage device. */
	int (*cd_write)(void *cookie, const void *buff, size_t len);

	/* Driver-specific cookie. */
	void *cd_cookie;
};

extern int configdb_attach_driver(const struct configdb_driver *, size_t);
extern int configdb_init(void);

#ifndef CONFIGDB_READ_ONLY
extern void configdb_erase_all(void);
extern int configdb_persist(void);
#else
#define	configdb_erase_all()	do { /* nothing */ } while (0)
#define	configdb_persist()	do { /* nothing */ } while (0)
#endif /* CONFIGDB_READ_ONLY */

#endif /* CONFIGDB_H */
