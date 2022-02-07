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

#ifndef FSDATA_CUSTOM_H
#define FSDATA_CUSTOM_H

#include <assert.h>

#include "lwip/apps/httpd_opts.h"
#include "lwip/def.h"
#include "lwip/apps/fs.h"

#if LWIP_HTTPD_CUSTOM_FILES
extern void fsdata_custom_init(void);
extern void fsdata_set_http_root_app(void);
extern void fsdata_set_http_root_provision(void);

typedef int (*fsdata_custom_open_t)(struct fs_file *, const char *, void *);
typedef int (*fsdata_custom_read_t)(struct fs_file *, char *, int, void *);
typedef void (*fsdata_custom_close_t)(struct fs_file *, void *);
extern void *fsdata_add_custom_handler(fsdata_custom_open_t,
	fsdata_custom_read_t, fsdata_custom_close_t, void *);
extern void fsdata_del_custom_handler(void *);

extern void fsdata_create(const char *name, const void *, size_t, bool dup);
extern void fsdata_delete(const char *name);
extern const char *fsdata_create_one_time(const char *suff, const void *data,
	size_t len, bool dup);
#endif

#endif /* FSDATA_CUSTOM_H */
