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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef NETWORK_LWIP
#error "httpd support requires lwIP"
#endif

#include "linked-lists.h"
#include "network_httpd.h"
#include "configdb.h"
#include "stringio.h"
#include "json_utils.h"
#include "fsdata_custom.h"
#include "zone_alloc.h"
#if (LWIP_HTTPD_CUSTOM_FILES != 0)
#include "wifi_provision.h"
#include "version.h"
#endif

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/apps/httpd.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

#if LWIP_HTTPD_SUPPORT_POST
#define	NETWORK_MAX_POST_CONTENT_LEN	2048

union post_handler_func {
	network_http_post_handler_furl_t hf_furl;
};

typedef enum {
	NETWORK_POST_TYPE_JSON,
	NETWORK_POST_TYPE_FURL,
} post_handler_type_t;

struct post_handler {
	post_handler_type_t ph_type;
	union post_handler_func ph_handler;
	void *ph_arg;
	const char *ph_uri;
	LIST_ENTRY(post_handler) ph_qent;
};
LIST_HEAD(post_handler_qhead, post_handler);
static struct post_handler_qhead post_handlers =
	LIST_HEAD_INITIALIZER(post_handlers);

struct post_connection {
	void *pc_connection;
	struct post_handler *pc_handler;
	unsigned int pc_content_len;
	unsigned int pc_buffer_len;
	unsigned int pc_offset;
	struct furl_key_value *pc_kv;
	char *pc_buffer;
	LIST_ENTRY(post_connection) pc_qent;
};
LIST_HEAD(post_connection_qhead, post_connection);
static struct post_connection_qhead post_connections =
	LIST_HEAD_INITIALIZER(post_connections);

#if (LWIP_HTTPD_CUSTOM_FILES != 0)
static rtos_mutex_t ping_mutex;
static const char *ping_strings[NETWORK_HTTPD_PING__COUNT] = {
	[NETWORK_HTTPD_PING_TARGET] = "target",
	[NETWORK_HTTPD_PING_NETSUM] = "netsum",
	[NETWORK_HTTPD_PING_NET] = "network",
	[NETWORK_HTTPD_PING_COMMS] = "comms",
	[NETWORK_HTTPD_PING_DEBUG] = "debug"
};
static uint8_t ping_numbers[NETWORK_HTTPD_PING__COUNT];

static network_http_provision_info_t set_provision_info;
static void *set_provision_info_arg;
#endif /* (LWIP_HTTPD_CUSTOM_FILES != 0) */

void *
network_httpd_add_post_handler_furl(network_http_post_handler_furl_t h,
    void *arg, const char *uri)
{
	struct post_handler *ph;

	if ((ph = zone_malloc(sizeof(*ph))) == NULL)
		return NULL;

	ph->ph_type = NETWORK_POST_TYPE_FURL;
	ph->ph_handler.hf_furl = h;
	ph->ph_arg = arg;
	ph->ph_uri = uri;

	/* XXX: Locking! */
	/* XXX: Check for duplicates! */
	LIST_INSERT_HEAD(&post_handlers, ph, ph_qent);

	return ph;
}

void
network_httpd_del_post_handler(void *h)
{
	struct post_handler *ph = h;

	/* XXX: Locking! */
	LIST_REMOVE(ph, ph_qent);
	zone_free(ph);
}

static struct post_connection *
find_connection(void *connection)
{
	struct post_connection *pc;

	LIST_FOREACH(pc, &post_connections, pc_qent) {
		if (pc->pc_connection == connection)
			break;
	}

	return pc;
}

err_t
httpd_post_begin(void *connection, const char *uri, const char *http_request,
    u16_t http_request_len, int content_len, char *response_uri,
    u16_t response_uri_len, u8_t *post_auto_wnd)
{
	struct post_handler *ph;
	struct post_connection *pc;
	unsigned int clen = (unsigned int) content_len;
	const char *p, *ct;

	LWIP_UNUSED_ARG(http_request_len);
	LWIP_UNUSED_ARG(post_auto_wnd);

	/* Search for a handler for this URI */
	/* XXX: Locking! */
	LIST_FOREACH(ph, &post_handlers, ph_qent) {
		if (strcmp(uri, ph->ph_uri) == 0)
			break;
	}

	/* Return 404 if there's no handler. */
	if (ph == NULL) {
		snprintf(response_uri, response_uri_len, "/nonexistant.html");
		return ERR_VAL;
	}

	ct = "application/x-www-form-urlencoded";
	if ((p = strstr(http_request, "Content-Type: ")) == NULL ||
	    strncasecmp(p + sizeof("Content-Type:"), ct, strlen(ct)) != 0) {
  bad_err:
		snprintf(response_uri, response_uri_len, "/error.txt");
		return ERR_VAL;
	}

	/* We also only accept sensible values for content_len */
	if (clen == 0 || clen > NETWORK_MAX_POST_CONTENT_LEN)
		goto bad_err;

	/* Do we already know about this connection? */
	if ((pc = find_connection(connection)) == NULL) {
		if ((pc = zone_malloc(sizeof(*pc))) == NULL) {
 mem_err:
			snprintf(response_uri, response_uri_len,
			    "/error.txt");
			return ERR_VAL;
		}

		pc->pc_buffer = zone_malloc(clen + 1);
		if (pc->pc_buffer == NULL) {
			zone_free(pc);
			goto mem_err;
		}
		pc->pc_buffer_len = clen + 1;
		pc->pc_connection = connection;
		pc->pc_handler = ph;
		LIST_INSERT_HEAD(&post_connections, pc, pc_qent);
	} else {
		assert(pc->pc_handler == ph);
		if ((clen + 1) > pc->pc_buffer_len) {
			void *new_data = zone_malloc(clen + 1);
			if (new_data == NULL)
				goto mem_err;
			zone_free(pc->pc_buffer);
			pc->pc_buffer = new_data;
			pc->pc_buffer_len = clen + 1;
		}
	}

	pc->pc_content_len = clen;
	pc->pc_offset = 0;

	return ERR_OK;
}

err_t
httpd_post_receive_data(void *connection, struct pbuf *p)
{
	struct post_connection *pc;
	err_t rv;

	if ((pc = find_connection(connection)) == NULL ||
	    (p->tot_len + pc->pc_offset) > pc->pc_content_len) {
		rv = ERR_VAL;
	} else {
		u16_t copied;

		copied = pbuf_copy_partial(p, &pc->pc_buffer[pc->pc_offset],
		    p->tot_len, 0);
		rv = copied ? ERR_OK : ERR_VAL;
		pc->pc_offset += copied;
	}

	pbuf_free(p);
	return rv;
}

static int
hex_to_uint8(char ch, uint8_t *rp)
{

	if (ch >= '0' && ch <= '9')
		*rp = (uint8_t)(ch - '0');
	else
	if (ch >= 'A' && ch <= 'F')
		*rp = (uint8_t)(ch - 'A') + 10u;
	else
	if (ch >= 'a' && ch <= 'f')
		*rp = (uint8_t)(ch - 'a') + 10u;
	else
		return -1;

	return 0;
}

static int
unescape(char *src)
{
	char ch, *dest;
	uint8_t v[2];

	dest = src;
	while ((ch = *src++) != '\0') {
		if (ch == '%') {
			ch = *src++;
			if (ch == '\0' || hex_to_uint8(ch, &v[0]) < 0)
				return -1;

			ch = *src++;
			if (ch == '\0' || hex_to_uint8(ch, &v[1]) < 0)
				return -1;

			ch = (char)((v[0] << 4) | v[1]);
		} else
		if (ch == '+')
			ch = ' ';

		*dest++ = ch;
	}

	*dest = '\0';
	return 0;
}

static const char *
network_http_post_handle_furl(struct post_connection *pc)
{
	struct post_handler *ph;
	char *key, *value, *x;
	unsigned int nkeys, maxkeys;
	const char *rv;
	struct network_http_furl_key_value *kv = NULL;

	/* Determine how many keys have been provided in the URL. */
	for (maxkeys = 0, key = pc->pc_buffer; *key != '\0'; key++) {
		if (*key == '&')
			maxkeys++;
	}

	nkeys = 0;
	if (maxkeys != 0 &&
	    (kv = zone_malloc((maxkeys + 1) * sizeof(*kv))) != NULL) {
		maxkeys++;

		for (nkeys = 0, key = strtok_r(pc->pc_buffer, "&", &x);
		    key != NULL && nkeys < maxkeys;
		    key = strtok_r(NULL, "&", &x)) {
			value = strchr(key, '=');
			if (value != NULL) {
				*value++ = '\0';
				if (unescape(value) < 0)
					value = NULL;
			}

			if (unescape(key) >= 0) {
				kv[nkeys].kv_key = key;
				kv[nkeys].kv_value = value;
				nkeys++;
			}
		}
	}

	ph = pc->pc_handler;
	rv = (ph->ph_handler.hf_furl)(ph->ph_arg, nkeys, kv);

	if (kv != NULL)
		zone_free(kv);

	return (rv != NULL) ? rv : "/ok.txt";
}

void
httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
	struct post_connection *pc;
	const char *resp;

	if ((pc = find_connection(connection)) == NULL) {
		snprintf(response_uri, response_uri_len, "/error.txt");
		return;
	}

	pc->pc_buffer[pc->pc_offset] = '\0';
	resp = network_http_post_handle_furl(pc);

	/* Submit the optional reply. */
	if (resp != NULL)
		snprintf(response_uri, response_uri_len, "%s", resp);

	/* Done with this connection. */
	LIST_REMOVE(pc, pc_qent);
	zone_free(pc);
}
#endif /* LWIP_HTTPD_SUPPORT_POST */

#if (LWIP_HTTPD_CUSTOM_FILES != 0)
void
network_httpd_update_ping(network_httpd_ping_t p)
{
	const char *json;
	stringio_t ss;

	assert(p < NETWORK_HTTPD_PING__COUNT);

	if ((ss = stringio_new()) == NULL)
		return;

	rtos_mutex_acquire(ping_mutex);

	ping_numbers[p] += 1;

	stringio_printf(ss, JSON_OBJECT_START);

	for (unsigned int i = 0; i < NETWORK_HTTPD_PING__COUNT; i++) {
		if (i > 0)
			stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_NUMBER_V(ping_strings[i],
		    ping_numbers[i]));
	}

	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL)
		fsdata_create("/ping.json", json, strlen(json), true);

	rtos_mutex_release(ping_mutex);

	stringio_done(ss);
}

static void
create_ip_string(char *ipstr, uint32_t ip)
{

	if (ip != 0)
		network_ip2str(ip, ipstr);
	else
		ipstr[0] = '\0';
}

static void
network_config_status_cb(const network_ip_params_t *pip)
{
	network_ip_params_t ip;
	const char *json;
	stringio_t ss;
	uint8_t sec;
	bool dhcp;
	struct {
		char ssid[WIFI_PROVISION_SSID_MAX + 1];
		char pass[WIFI_PROVISION_PASS_MAX + 1];
		char hostname[65];
	} *wifi;

	wifi = zone_calloc(1, sizeof(*wifi));
	assert(wifi != NULL);
	if (wifi == NULL)
		return;

	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &sec) == 0)
		sec = (uint8_t)WIFI_PROVISION_ENC_OPEN;

	if (cf_get_string(CF_KEY_STRING_WIFI_SSID, sizeof(wifi->ssid),
	    wifi->ssid) == 0 || (sec != WIFI_PROVISION_ENC_OPEN &&
	    cf_get_string(CF_KEY_STRING_WIFI_KEY, sizeof(wifi->pass),
	    wifi->pass) == 0)) {
		zone_free(wifi);
		return;
	}

	cf_get_string(CF_KEY_STRING_WIFI_NAME, sizeof(wifi->hostname),
	    wifi->hostname);

	ip.ip_addr = 0;
	cf_get_uint32(CF_KEY_UINT32_NETWORK_IP, &ip.ip_addr);
	dhcp = ip.ip_addr == 0;

	if (pip == NULL) {
		ip.ip_mask = 0;
		ip.ip_gate = 0;
		cf_get_uint32(CF_KEY_UINT32_NETWORK_MASK, &ip.ip_mask);
		cf_get_uint32(CF_KEY_UINT32_NETWORK_GATE, &ip.ip_gate);
	} else {
		ip = *pip;
	}

	/* Recreate network.json */

	if ((ss = stringio_new()) == NULL) {
		zone_free(wifi);
		return;
	}

	stringio_printf(ss, JSON_OBJECT_START);
	stringio_printf(ss, JSON_STRING("ssid", wifi->ssid));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("wifi_pass", wifi->pass));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_NUMBER("wifi_sec", sec));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("hostname", wifi->hostname));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_BOOL("dhcp", dhcp));
	stringio_printf(ss, JSON_SEPARATOR);

	/* Re-use the wifi buffers to format IP strings. */
	create_ip_string(wifi->ssid, ip.ip_addr);
	create_ip_string(wifi->pass, ip.ip_mask);
	create_ip_string(wifi->hostname, ip.ip_gate);

	stringio_printf(ss, JSON_STRING("ip_addr", wifi->ssid));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("ip_mask", wifi->pass));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("ip_gate", wifi->hostname));
	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL) {
		fsdata_create("/network.json", json, strlen(json), true);
		network_httpd_update_ping(NETWORK_HTTPD_PING_NET);
	}

	stringio_done(ss);
	zone_free(wifi);
}

static void
network_link_status_cb(const network_link_status_t *ls, void *arg)
{
	static network_link_status_t cls;
	static uint32_t current_ip;
	const network_ip_params_t *ipp;
	const char *json;
	stringio_t ss;
	char ipstr[16];

	(void) arg;

	if (ls->ls_state == NETWORK_LINK_UP)
		ipp = network_get_ip_params();
	else
		ipp = NULL;

	/* Bail if no change since last report. */
	if (cls.ls_state == ls->ls_state &&
	    cls.ls_chan == ls->ls_chan &&
	    cls.ls_sig == ls->ls_sig &&
	    (ipp != NULL && current_ip == ipp->ip_addr) &&
	    strcmp(cls.ls_bssid, ls->ls_bssid) == 0) {
		return;
	}

	/* Recreate netsum.json */

	if ((ss = stringio_new()) == NULL)
		return;

	if (ipp != NULL)
		network_ip2str(ipp->ip_addr, ipstr);
	else
		ipstr[0] = '\0';

	stringio_printf(ss, JSON_OBJECT_START);
	stringio_printf(ss, JSON_STRING("ssid", ls->ls_bssid));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_NUMBER("chan", ls->ls_chan));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_NUMBER("sig", ls->ls_sig));
	stringio_printf(ss, JSON_SEPARATOR);
	stringio_printf(ss, JSON_STRING("addr", ipstr));
	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL) {
		fsdata_create("/netsum.json", json, strlen(json), true);
		network_httpd_update_ping(NETWORK_HTTPD_PING_NETSUM);
	}

	stringio_done(ss);

	cls = *ls;
	if (ipp != NULL) {
		if (ipp->ip_addr != 0 && current_ip != ipp->ip_addr)
			network_config_status_cb(ipp);
		current_ip = ipp->ip_addr;
	} else {
		current_ip = 0;
	}
}

static const char *
network_config_post_furl(void *cookie, int nkeys,
    const struct network_http_furl_key_value *kv)
{
	const char *ssid, *pass;
	wifi_provision_enc_t enc;
	bool do_dhcp, wifi_changed;
	const char *hostname, *rv;
	const char *ip_addr, *ip_mask, *ip_gate;
	network_ip_params_t ipp;

	(void) cookie;

	ssid = pass = NULL;
	enc = WIFI_PROVISION_ENC_OPEN;
	do_dhcp = true;
	hostname = NULL;
	ip_addr = ip_mask = ip_gate = NULL;

	while (nkeys--) {
		const char *k;

		if (strncmp(kv->kv_key, "net-wifi-", 9) == 0) {
			k = &kv->kv_key[9];
			if (strcmp(k, "ssid") == 0)
				ssid = kv->kv_value;
			else
			if (strcmp(k, "pass") == 0)
				pass = kv->kv_value;
			else
			if (strcmp(k, "security") == 0) {
				if (strcmp(kv->kv_value, "0") == 0)
					enc = WIFI_PROVISION_ENC_OPEN;
				else
					enc = WIFI_PROVISION_ENC_WPA;
			}
		} else
		if (strncmp(kv->kv_key, "net-ip-", 7) == 0) {
			k = &kv->kv_key[7];
			if (strcmp(k, "assignment") == 0)
				do_dhcp = strcmp(kv->kv_value, "0") != 0;
			else
			if (strcmp(k, "hostname") == 0)
				hostname = kv->kv_value;
			else
			if (strcmp(k, "addr") == 0)
				ip_addr = kv->kv_value;
			else
			if (strcmp(k, "mask") == 0)
				ip_mask = kv->kv_value;
			else
			if (strcmp(k, "gate") == 0)
				ip_gate = kv->kv_value;
		}
		kv++;
	}

	/*
	 * All parameters collected. Validate them.
	 */
	rv = NULL;

	if (enc == WIFI_PROVISION_ENC_WPA &&
	    (pass == NULL || strlen(pass) < 8)) {
		DBFPRINTF("enc but bad pass\n");
		rv = "Password must be 8 characters or more.";
	} else
	if (enc == WIFI_PROVISION_ENC_OPEN && pass != NULL &&
	    strlen(pass) != 0) {
		DBFPRINTF("no enc but pass\n");
		rv = "Password not required for an open AP.";
	}

	if (ssid == NULL || strlen(ssid) == 0) {
		DBFPRINTF("no ssid\n");
		rv = "WiFi network name must be supplied.";
	}

	if (do_dhcp) {
		memset(&ipp, 0, sizeof(ipp));
	} else {
		rv = network_parse_ip_params(ip_addr, ip_mask, ip_gate, &ipp);
		if (rv != NULL)
			DBFPRINTF("failed to parse ip params: %s\n", rv);
	}

	if (rv != NULL)
		return fsdata_create_one_time("txt", rv, strlen(rv), false);

	/*
	 * Changing WiFi provisioning details will also reset IP
	 * configuration to DHCP. This is by design, as it prevents
	 * a user painting themselves into a corner by assigning a
	 * static IP which is not usable on the new WiFi network,
	 * resulting in a Bad Experience from which recovery will
	 * be non-trivial.
	 *
	 * Thus, if WiFi configuration changes, we ignore any
	 * attempt to change IP configuration.
	 */
	if (set_provision_info != NULL) {
		wifi_changed = set_provision_info(set_provision_info_arg,
		    ssid, pass, hostname);
	} else {
		wifi_changed = false;
	}

	if (wifi_changed) {
		/* Reset IP configuration to DHCP. */
		memset(&ipp, 0, sizeof(ipp));
	}

	network_set_ip_params_drv_persist(&ipp);

	return rv;
}

void
network_httpd_set_provision_info_cb(network_http_provision_info_t cb,
    void *arg)
{

	set_provision_info = cb;
	set_provision_info_arg = arg;
}
#endif /* (LWIP_HTTPD_CUSTOM_FILES != 0) */

void
network_httpd_init(void)
{

#if (LWIP_HTTPD_CUSTOM_FILES != 0)
	/*
	 * Start the custom fsdata service.
	 */
	fsdata_custom_init();

	/*
	 * Activate the application filesystem.
	 */
	fsdata_set_http_root_app();

	/*
	 * Initialise json ping state.
	 */
	for (network_httpd_ping_t p = NETWORK_HTTPD_PING_TARGET;
	    p < NETWORK_HTTPD_PING__COUNT; p++) {
		ping_numbers[p] = 0u;
	}

	ping_mutex = rtos_mutex_create();
	assert(ping_mutex != NULL);

	/*
	 * Create maven.json with firmware details.
	 */
	char *maven_json = zone_malloc(128);
	assert(maven_json != NULL);
	const serial_number_t *sn = serial_number_fetch();
	assert(sn != NULL);

	sprintf(maven_json, "{\"fwver\":\"%s.%s.%s\","
			    "\"fwdate\":\"%s\","
			    "\"serial\":\"%s\"}",
	    PRODUCT_VERSION_MAJOR, PRODUCT_VERSION_MINOR, PRODUCT_VERSION_TINY,
	    PRODUCT_BUILD_DATE, sn->cstr);
	fsdata_create("/maven.json", maven_json, strlen(maven_json), false);

	/* We handle network status here. */
	network_hook_link_status(network_link_status_cb, NULL);

	/* And network configuration changes. */
	network_httpd_add_post_handler_furl(network_config_post_furl, NULL,
	    "/network.cgi");

	network_config_status_cb(NULL);
#endif /* (LWIP_HTTPD_CUSTOM_FILES != 0) */

	/* Start the LwIP httpd server. */
	httpd_init();
}

