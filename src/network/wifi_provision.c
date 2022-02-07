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

#include "linked-lists.h"
#include "network.h"
#include "network_httpd.h"
#include "wifi_provision.h"
#include "zone_alloc.h"
#if LWIP_HTTPD_CUSTOM_FILES
#include "fsdata_custom.h"
#endif
#ifdef WIFI_PROVISION_SUPPORT_SCAN
#include "stringio.h"
#include "json_utils.h"
#endif

struct wifi_scan_entry {
	wifi_provision_ssid_t se_details;
	LIST_ENTRY(wifi_scan_entry) se_qent;
};
LIST_HEAD(wifi_scan_entry_qhead, wifi_scan_entry);

struct wifi_provision {
	struct wifi_scan_entry_qhead wp_scan_entries;
	int wp_count;
	void *wp_fsdata_cookie;
	void *wp_post_cookie;
	int wp_done;
	wifi_provision_enc_t wp_enc;
	char wp_ssid[WIFI_PROVISION_SSID_MAX + 1];
	char wp_pass[WIFI_PROVISION_PASS_MAX + 1];
};

static const char *
wifi_provision_post_common(struct wifi_provision *wp, const char *ssid,
    const char *pass)
{
	const struct wifi_scan_entry *se;
	wifi_provision_enc_t enc;
	size_t l;

	/* SSID and PASS must exist. */
	if (ssid == NULL || pass == NULL)
		return "WiFi name and/or password missing.";

	/* SSID length must be between 1 and WIFI_PROVISION_SSID_MAX */
	l = strlen(ssid);
	if (l < 1 || l > WIFI_PROVISION_SSID_MAX)
		return "WiFi network name invalid.";

	/* PASS length must be 0, or >= 8 and <= WIFI_PROVISION_PASS_MAX */
	l = strlen(pass);
	if (l > 0 && (l < 8 || l > WIFI_PROVISION_PASS_MAX))
		return "WiFi password invalid.";

	if (pass[0] == '\0')
		enc = WIFI_PROVISION_ENC_OPEN;
	else
		enc = WIFI_PROVISION_ENC_WPA;

	/* Do we already know about this SSID? */
	LIST_FOREACH(se, &wp->wp_scan_entries, se_qent) {
		if (strcmp(se->se_details.ps_ssid, ssid) == 0) {
			enc = se->se_details.ps_enc;
			break;
		}
	}

	/* Error if known SSID needs a password, but none supplied. */
	if (pass[0] == '\0' && enc != WIFI_PROVISION_ENC_OPEN)
		return "WiFi network requires a password.";

	/* Stash the information, and flag provisioning as done. */
	strlcpy(wp->wp_ssid, ssid, sizeof(wp->wp_ssid));
	strlcpy(wp->wp_pass, pass, sizeof(wp->wp_pass));
	wp->wp_enc = enc;
	wp->wp_done = 1;

	return NULL;
}

static const char *
wifi_provision_post_furl(void *cookie, int nkeys,
    const struct network_http_furl_key_value *kv)
{
	struct wifi_provision *wp = cookie;
	const char *ssid, *pass, *rv;

	ssid = pass = NULL;

	while (nkeys--) {
		if (strstr(kv->kv_key, "ssid") != NULL)
			ssid = kv->kv_value;
		else
		if (strstr(kv->kv_key, "pass") != NULL)
			pass = kv->kv_value;

		kv++;
	}

	rv = wifi_provision_post_common(wp, ssid, pass);
	if (rv != NULL)
#if LWIP_HTTPD_CUSTOM_FILES
		return fsdata_create_one_time("txt", rv, strlen(rv), false);
#else
		return "/aperr.txt";
#endif

	return NULL;
}

void *
wifi_provision_start(void)
{
	struct wifi_provision *wp;

	if ((wp = zone_malloc(sizeof(*wp))) == NULL)
		return NULL;

	LIST_INIT(&wp->wp_scan_entries);
	wp->wp_count = 0;

	wp->wp_post_cookie =
	    network_httpd_add_post_handler_furl(wifi_provision_post_furl, wp,
	    "/apconn.cgi");

	if (wp->wp_post_cookie == NULL) {
		zone_free(wp);
		return NULL;
	}

	wp->wp_done = 0;

#if (LWIP_HTTPD_CUSTOM_FILES != 0)
	fsdata_set_http_root_provision();
#endif

	return wp;
}

void
wifi_provision_end(void *arg)
{
	struct wifi_provision *wp = arg;
	struct wifi_scan_entry *se, *tse;

	network_httpd_del_post_handler(wp->wp_post_cookie);

#if (LWIP_HTTPD_CUSTOM_FILES != 0)
	fsdata_delete("/aplist.json");
	fsdata_set_http_root_app();
#endif

	LIST_FOREACH_SAFE(se, &wp->wp_scan_entries, se_qent, tse) {
		LIST_REMOVE(se, se_qent);
		zone_free(se);
		wp->wp_count--;
	}

	assert(wp->wp_count == 0);

	zone_free(wp);

	return;
}

int
wifi_provision_is_done(void *arg, const char **pssid, const char **ppass,
    wifi_provision_enc_t *penc)
{
	struct wifi_provision *wp = arg;

	if (wp->wp_done) {
		*pssid = wp->wp_ssid;
		*ppass = wp->wp_pass;
		*penc = wp->wp_enc;
	}

	return wp->wp_done;
}

int
wifi_provision_add_ssid(void *arg, const char *ssid, uint8_t rssi,
    wifi_provision_enc_t enc)
{
	struct wifi_provision *wp = arg;
	struct wifi_scan_entry *new_se, *list_se, *last_se;

	/*
	 * Have we seen this SSID before?
	 */
	LIST_FOREACH(new_se, &wp->wp_scan_entries, se_qent) {
		if (strcmp(new_se->se_details.ps_ssid, ssid) == 0)
			break;
	}

	if (new_se != NULL) {
		/*
		 * SSID already exists. Remove it temporarily.
		 */
		assert(wp->wp_count > 0);
		wp->wp_count--;
		LIST_REMOVE(new_se, se_qent);
	} else
	if ((new_se = zone_malloc(sizeof(*new_se))) != NULL) {
		/*
		 * New SSID.
		 */
		strlcpy(new_se->se_details.ps_ssid, ssid,
		    sizeof(new_se->se_details.ps_ssid));
	} else {
		/* Oops. */
		return -1;
	}

	/* Update the RSSI and encryption. */
	new_se->se_details.ps_rssi = rssi;
	new_se->se_details.ps_enc = enc;

	/*
	 * We keep the list ordered by RSSI
	 */
	last_se = NULL;
	LIST_FOREACH(list_se, &wp->wp_scan_entries, se_qent) {
		if (new_se->se_details.ps_rssi > list_se->se_details.ps_rssi)
			break;
		last_se = list_se;
	}

	/*
	 * Insert the entry into the appropriate place in the list.
	 */
	if (list_se != NULL)
		LIST_INSERT_BEFORE(list_se, new_se, se_qent);
	else if (last_se != NULL)
		LIST_INSERT_AFTER(last_se, new_se, se_qent);
	else
		LIST_INSERT_HEAD(&wp->wp_scan_entries, new_se, se_qent);

	wp->wp_count++;

	return wp->wp_count;
}

int
wifi_provision_scan_done(void *arg)
{
#ifdef WIFI_PROVISION_SUPPORT_SCAN
	struct wifi_provision *wp = arg;
	const wifi_provision_ssid_t *ps;
	struct wifi_scan_entry *se;
	const char *json;
	bool separator;
	stringio_t ss;

	if (wp->wp_count == 0)
		return 0;

	if ((ss = stringio_new()) == NULL)
		return -1;

	stringio_printf(ss, JSON_ARRAY_START);

	/*
	 * Generate a JSON representation of the scan entries.
	 */
	separator = false;
	LIST_FOREACH(se, &wp->wp_scan_entries, se_qent) {
		ps = &se->se_details;

		if (separator)
			stringio_printf(ss, JSON_SEPARATOR);
		else
			separator = true;

		stringio_printf(ss, JSON_OBJECT_START);
		stringio_printf(ss, JSON_STRING("ssid", ps->ps_ssid));
		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_NUMBER("rssi",(int)ps->ps_rssi - 255));
		stringio_printf(ss, JSON_SEPARATOR);
		stringio_printf(ss, JSON_NUMBER("enc", ps->ps_enc));
		stringio_printf(ss, JSON_OBJECT_END);
	}

	stringio_printf(ss, JSON_ARRAY_END);

	if ((json = stringio_get_buff(ss)) != NULL)
		fsdata_create("/aplist.json", json, strlen(json), true);

	stringio_done(ss);
#else
	(void) arg;
#endif /* WIFI_PROVISION_SUPPORT_SCAN */
	return 0;
}

const wifi_provision_ssid_t *
wifi_provision_get_ssid(void *arg, int idx)
{
	struct wifi_provision *wp = arg;
	struct wifi_scan_entry *se;

	assert(idx >= 0);

	if (idx < 0 || idx >= wp->wp_count)
		return NULL;

	LIST_FOREACH(se, &wp->wp_scan_entries, se_qent) {
		if (idx-- == 0)
			break;
	}

	if (se == NULL)
		return NULL;

	return &se->se_details;
}
