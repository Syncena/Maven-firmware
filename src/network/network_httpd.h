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

#ifndef NETWORK_HTTPD_H
#define NETWORK_HTTPD_H

#include <stdbool.h>

typedef enum {
	NETWORK_HTTPD_PING_TARGET = 0,
	NETWORK_HTTPD_PING_NETSUM,
	NETWORK_HTTPD_PING_NET,
	NETWORK_HTTPD_PING_COMMS,
	NETWORK_HTTPD_PING_DEBUG,
	NETWORK_HTTPD_PING__COUNT,
} network_httpd_ping_t;

struct network_http_furl_key_value {
	const char *kv_key;
	const char *kv_value;
};

extern void network_httpd_init(void);

typedef const char *(*network_http_post_handler_furl_t)(void *,
		int nkeys, const struct network_http_furl_key_value *kv);

extern void *network_httpd_add_post_handler_furl(
			network_http_post_handler_furl_t, void *,
			const char *uri);

extern void network_httpd_del_post_handler(void *);

extern void network_httpd_update_ping(network_httpd_ping_t);

typedef bool (*network_http_provision_info_t)(void *, const char *ssid,
		const char *pass, const char *name);
extern void network_httpd_set_provision_info_cb(network_http_provision_info_t,
		void *);

#endif /* NETWORK_HTTPD_H */
