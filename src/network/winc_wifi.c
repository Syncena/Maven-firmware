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
#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "common/include/nm_common.h"
#include "driver/include/m2m_periph.h"
#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_ota.h"
#include "socket/include/socket.h"
#include "driver/source/nmasic.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#include "linked-lists.h"
#include "zone_alloc.h"

#ifdef WINC_OPT_USE_WIFI_TIME
#include "timer.h"
#endif
#include "network.h"
#include "winc_wifi.h"

#ifdef WINC_OPT_COMMAND
#include "shell.h"
#endif

#ifdef CONFIG_USE_CONFIGDB
#include "configdb.h"
#endif

#ifdef NETWORK_LWIP
#ifndef ETH_MODE
#error "NETWORK_LWIP requires ETH_MODE to be defined."
#endif
#include "network_lwip.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"
#endif

//#define DEBUG_FLAG	winc_wifi_debug
//#define DEBUG_FLAG_INIT	1
#include "debug.h"

#ifdef WINC_OPT_USE_WIFI_TIME
#ifndef TIMER_OPT_SYSTEM_TIME
#error "WINC_OPT_USE_WIFI_TIME needs TIMER_OPT_SYSTEM_TIME"
#endif
/*
 * Note: TIMER_OPT_EVENTS is optional, but without it we will query the
 * WiFi module's time only once at startup. Otherwise we query regularly
 * in the (vain) hope the module's clock is better than ours.
 */
#endif /* WINC_OPT_USE_WIFI_TIME */

#ifdef WINC_ADAFRUIT_BREAKOUT
#if 0
/* These are for the Rev.A chip */
#define	AFB_LED_ERR	M2M_PERIPH_GPIO18
#define	AFB_LED_WIFI	M2M_PERIPH_GPIO16
#define	AFB_LED_NET	M2M_PERIPH_GPIO15
#else
/* These are for the Rev.B chip */
#define	AFB_LED_ERR	M2M_PERIPH_GPIO6
#define	AFB_LED_WIFI	M2M_PERIPH_GPIO4
#define	AFB_LED_NET	M2M_PERIPH_GPIO5
#endif
#endif	/* WINC_ADAFRUIT_BREAKOUT */

enum winc_wifi_link_state {
	WINC_WIFI_STATE_INIT = 0,
	WINC_WIFI_STATE_NEW_PROVISION,
	WINC_WIFI_STATE_PROVISIONING,
	WINC_WIFI_STATE_PROVISIONED,
	WINC_WIFI_STATE_CONNECTING,
	WINC_WIFI_STATE_ACQUIRE_ADDRESS,
	WINC_WIFI_STATE_GET_RSSI,
	WINC_WIFI_STATE_CONFIGURED,
	WINC_WIFI_STATE_SHOWTIME,
};

#define	WINC_SOCK_BUFFER_SIZE	1024
#define	WINC_SOCK_BUFFER_MAX	(NETWORK_NSOCKETS * 3u)
struct winc_socket_buffer {
	SIMPLEQ_ENTRY(winc_socket_buffer) sb_qent;
	struct winc_socket_buffer *sb_next;
	unsigned int sb_len;
	uint8_t sb_buff[WINC_SOCK_BUFFER_SIZE];
};
SIMPLEQ_HEAD(winc_socket_buffer_head, winc_socket_buffer);

struct winc_socket {
	uint8_t ws_flags;
#define	WINC_SOCK_FLAG_IN_USE		(1u << 0)
#define	WINC_SOCK_FLAG_ACTIVE		(1u << 1)
#define	WINC_SOCK_FLAG_TCP		(1u << 2)
#define	WINC_SOCK_FLAG_ESTABLISHED	(1u << 3)

	SOCKET ws_sock;
	SOCKET ws_listen_sock;
#define	WINC_SOCK_INVALID	(-1)
#define	WINC_BAD_SOCKET(s)	(((s) < 0) || (s) >= MAX_SOCKET)

	network_sock_status_callback_t ws_status_callback;
	void *ws_status_callback_arg;
	struct winc_wifi *ws_winc;
	struct winc_socket *ws_next;

	uint16_t ws_lport;
	uint16_t ws_dport;
	uint32_t ws_daddr;

	struct winc_socket_buffer_head ws_rx_buffs;
	uint16_t ws_tx_pending;
	uint16_t ws_rx_length;
	uint16_t ws_rx_idx;
};

struct winc_wifi {
	struct network_driver ww_drv;
	struct network_state *ww_network_state;

	void *ww_cookie;
	void (*ww_read_write)(void *, const void *, void *, uint16_t);
	void (*ww_control)(void *, uint8_t, void *);
#ifndef CONFIG_USE_CONFIGDB
	struct winc_wifi_glob_vars *ww_gv;
#endif
	tpfNmBspIsr ww_irq_handler;
	uint8_t ww_prov_blink;
	volatile uint8_t ww_wifi_event;
	enum winc_wifi_link_state ww_link_state;
	enum winc_wifi_link_state ww_prov_old_state;
	network_link_status_callback_t ww_link_cb;
	void *ww_link_cb_arg;
	network_link_status_t ww_link_status;

	struct winc_prov_arg ww_prov_arg;

#ifdef WINC_OPT_USE_WIFI_TIME
	struct timer_event ww_sntp_ev;
/* Adjust period to slightly randomise the next query +/- ~30 minutes */
#define	WINC_SNTP_PERIOD	(TIMER_EVENT_PERIOD_DAILY + (rand() % \
	    ((TIMER_EVENT_PERIOD_HOURLY / 2) - TIMER_EVENT_PERIOD_HOURLY)))
#endif /* WINC_OPT_USE_WIFI_TIME */

#ifdef WINC_OPT_OTA_UPDATE
	uint8_t ww_ota_state;
#endif

	uint8_t ww_nsocket_buffs;
	uint8_t ww_nsockets;
	struct winc_socket_buffer *ww_socket_buffs;
	struct winc_socket_buffer_head ww_free_socket_buffs;
	struct winc_socket *ww_sockets;
	struct winc_socket *ww_asf2sock[MAX_SOCKET];
};
static struct winc_wifi *winc_wifi_state;

#ifdef NETWORK_LWIP
struct winc_pbuf {
	struct pbuf *wp_pbuf;
	STAILQ_ENTRY(winc_pbuf) wp_qent;
};

STAILQ_HEAD(winc_pbuf_qhead, winc_pbuf);
struct winc_wifi_lwip {
	struct winc_wifi wwl_ww;
	struct netif wwl_netif;
	network_link_status_callback_t wwl_link_cb;
	void *wwl_link_cb_arg;
	struct winc_pbuf_qhead wwl_tx_queue;
	struct winc_pbuf_qhead wwl_free_queue;
	struct network_lwip_driver wwl_driver;
	struct pbuf *wwl_rx_pbuf_head;
	struct pbuf *wwl_rx_pbuf_cur;
	uint8_t *wwl_rx_payload;
	struct pbuf *wwl_tx_pbuf;
	uint8_t *wwl_tx_payload;
	uint16_t wwl_tx_payload_len;
	uint8_t wwl_rx_buffer[128];
	uint8_t wwl_tx_buffer[1536];
};

static void winc_wifi_if_rx_cb(uint8, void *, void *);
#endif /* NETWORK_LWIP */

/*
 * Atmel's driver needs this to determine the maximum SPI read/write
 * length we can handle in one chunk. Actual limit is 65525, but
 * stick to 2K for now.
 */
tstrNmBusCapabilities egstrNmBusCapabilities = { 2048 };

#ifdef WINC_OPT_COMMAND
SHELL_CMD_DECL(wifi, winc_wifi_cmd, "Display WiFi Info");
#endif

static void winc_wifi_sock_disconnect(struct winc_socket *);

#ifdef WINC_OPT_GPIO
#ifdef WINC_ADAFRUIT_BREAKOUT
void
winc_wifi_ada_led_init(void)
{

	m2m_periph_init(NULL);
	m2m_periph_gpio_set_dir(AFB_LED_ERR, 1);
	m2m_periph_gpio_set_val(AFB_LED_ERR, 1);
	m2m_periph_gpio_set_dir(AFB_LED_WIFI, 1);
	m2m_periph_gpio_set_val(AFB_LED_WIFI, 1);
	m2m_periph_gpio_set_dir(AFB_LED_NET, 1);
	m2m_periph_gpio_set_val(AFB_LED_NET, 1);
}

void
winc_wifi_ada_led_control(uint8_t led, uint8_t state)
{

	switch (led) {
	case WINC_LED_ERR:
		m2m_periph_gpio_set_val(AFB_LED_ERR, !state);
		break;
	case WINC_LED_WIFI:
		m2m_periph_gpio_set_val(AFB_LED_WIFI, !state);
		break;
	case WINC_LED_NETWORK:
		m2m_periph_gpio_set_val(AFB_LED_NET, !state);
		break;
	}
}
#endif /* WINC_ADAFRUIT_BREAKOUT */
#endif /* WINC_OPT_GPIO */

static __always_inline void
winc_wifi_led_err(uint8_t state)
{

	winc_wifi_led_control(WINC_LED_ERR, state);
}

static __always_inline void
winc_wifi_led_wifi(uint8_t state)
{

	winc_wifi_led_control(WINC_LED_WIFI, state);
}

static void
winc_wifi_led_net(int8_t state)
{
	static uint8_t net_cnt;

	if (state < 0) {
		net_cnt = 1;
		state = WINC_LED_OFF;
	}

	if (state == WINC_LED_ON) {
		if (net_cnt++ == 0)
			winc_wifi_led_control(WINC_LED_NETWORK, WINC_LED_ON);
	} else {
		if (net_cnt == 1) {
			winc_wifi_led_control(WINC_LED_NETWORK, WINC_LED_OFF);
			net_cnt = 0;
		} else
		if (net_cnt > 1)
			net_cnt--;
	}
}

static void
winc_wifi_start_provision(struct winc_wifi *ww)
{
	tstrM2MAPConfig ap;
	char prov[32];
	uint32_t ip;

	if (ww->ww_link_state != WINC_WIFI_STATE_INIT)
		return;

	ww->ww_link_state = WINC_WIFI_STATE_PROVISIONING;
	ww->ww_prov_blink = 1;
	winc_wifi_led_wifi(WINC_LED_ON);

	(ww->ww_control)(ww->ww_cookie, WINC_WIFI_PROVISIONING, NULL);

	memset(&ap, 0, sizeof(ap));
#ifndef CONFIG_USE_CONFIGDB
	strlcpy(ww->ww_link_status.ls_bssid, ww->ww_gv->gv_prov_ssid,
	    sizeof(ww->ww_link_status.ls_bssid));
	ip = 0xc0a80001u;
#else
	if (cf_get_string(CF_KEY_STRING_WIFI_PROV_SSID,
	    sizeof(ww->ww_link_status.ls_bssid),
	    ww->ww_link_status.ls_bssid) == 0) {
		strlcpy(ww->ww_link_status.ls_bssid, "Default SSID",
		    sizeof(ww->ww_link_status.ls_bssid));
	}
	if (cf_get_uint32(CF_KEY_UINT32_WIFI_PROV_IP, &ip) == 0)
		ip = 0xc0a80001u;
#endif
	strlcpy((char *)ap.au8SSID, ww->ww_link_status.ls_bssid,
	    sizeof(ap.au8SSID));
	ap.u8ListenChannel = 1;
	ap.u8SecType = M2M_WIFI_SEC_OPEN;	/* Nobody does WEP now */
	ap.u8SsidHide = 0;
	ap.au8DHCPServerIP[0] = (ip >> 24) & 0xffu;
	ap.au8DHCPServerIP[1] = (ip >> 16) & 0xffu;
	ap.au8DHCPServerIP[2] = (ip >>  8) & 0xffu;
	ap.au8DHCPServerIP[3] = (ip >>  0) & 0xffu;
	sprintf(prov, "%u.%u.%u.%u",
	    (unsigned int)ap.au8DHCPServerIP[0],
	    (unsigned int)ap.au8DHCPServerIP[1],
	    (unsigned int)ap.au8DHCPServerIP[2],
	    (unsigned int)ap.au8DHCPServerIP[3]);

	if (m2m_wifi_start_provision_mode(&ap, prov, 1) != M2M_SUCCESS)
		winc_wifi_led_err(WINC_LED_ON);

	if (ww->ww_link_cb != NULL) {
		ww->ww_link_status.ls_state = NETWORK_LINK_PROVISIONING;
		ww->ww_link_status.ls_key[0] = '\0';
		ww->ww_link_status.ls_rssi = 0;
		(ww->ww_link_cb)(&ww->ww_link_status, ww->ww_link_cb_arg);
	}

	network_driver_soft_timeout(ww->ww_network_state, 500);
}

static void
winc_wifi_new_provision(struct winc_wifi *ww)
{
	struct winc_socket *ws;

	winc_wifi_led_wifi(WINC_LED_OFF);
	winc_wifi_led_net(-1);
	winc_wifi_led_err(WINC_LED_OFF);

	ww->ww_link_status.ls_state = NETWORK_LINK_DOWN;
	ww->ww_link_status.ls_key[0] = '\0';
	ww->ww_link_status.ls_rssi = 0;

	/* Notify all open sockets that link's down */
	for (ws = ww->ww_sockets; ws != NULL; ws = ws->ws_next) {
		if ((ws->ws_flags & WINC_SOCK_FLAG_IN_USE) != 0) {
//			winc_wifi_close(ww, (network_sock_t)ws);
			winc_wifi_sock_disconnect(ws);
		}
	}

	/* Notify network layer that link is down */
	if (ww->ww_prov_old_state > WINC_WIFI_STATE_ACQUIRE_ADDRESS &&
	    ww->ww_link_cb != NULL) {
		(ww->ww_link_cb)(&ww->ww_link_status, ww->ww_link_cb_arg);
	}

#ifndef NETWORK_LWIP
	network_ip_params_t ip;
	ip.ip_addr = 0;
	ip.ip_mask = 0;
	ip.ip_gate = 0;
	network_set_ip_params_drv(&ip);
#endif

	ww->ww_link_state = WINC_WIFI_STATE_INIT;
	network_driver_soft_timeout(ww->ww_network_state, 0);
}

static tenuM2mSecType
winc_wifi_get_enc(struct winc_wifi *ww)
{
	uint8_t enc;

#ifndef CONFIG_USE_CONFIGDB
	enc = ww->ww_gv->gv_enc_type;
#else
	(void) ww;
	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &enc) == 0)
		enc = WINC_WIFI_SEC_INVALID;
#endif

	switch (enc) {
	default:
	case WINC_WIFI_SEC_INVALID:
		return M2M_WIFI_SEC_INVALID;
	case WINC_WIFI_SEC_OPEN:
		return M2M_WIFI_SEC_OPEN;
	case WINC_WIFI_SEC_WPA_PSK:
		return M2M_WIFI_SEC_WPA_PSK;
	case WINC_WIFI_SEC_WEP:
		return M2M_WIFI_SEC_WEP;
	case WINC_WIFI_SEC_802_1X:
		return M2M_WIFI_SEC_802_1X;
	}
}

static uint8_t
winc_wifi_from_enc(uint8_t enc)
{

	switch (enc) {
	default:
	case M2M_WIFI_SEC_INVALID:
		return WINC_WIFI_SEC_INVALID;
	case M2M_WIFI_SEC_OPEN:
		return WINC_WIFI_SEC_OPEN;
	case M2M_WIFI_SEC_WPA_PSK:
		return WINC_WIFI_SEC_WPA_PSK;
	case M2M_WIFI_SEC_WEP:
		return WINC_WIFI_SEC_WEP;
	case M2M_WIFI_SEC_802_1X:
		return WINC_WIFI_SEC_802_1X;
	}
}

static sint8
winc_m2m_wifi_connect(struct winc_wifi *ww)
{

	return m2m_wifi_connect(ww->ww_link_status.ls_bssid,
	    strlen(ww->ww_link_status.ls_bssid),
	    winc_wifi_get_enc(ww),
	    ww->ww_link_status.ls_key,
	    M2M_WIFI_CH_ALL);
}

static void
winc_wifi_start_connecting(struct winc_wifi *ww)
{

	if (ww->ww_prov_arg.pa_cb != NULL) {
		void (*cb)(void *);
		cb = ww->ww_prov_arg.pa_cb;
		ww->ww_prov_arg.pa_cb = NULL;
		DBFPRINTF("Invoking provisioned callback\n");
		(cb)(ww->ww_prov_arg.pa_cookie);
	}

#ifndef CONFIG_USE_CONFIGDB
	strlcpy(ww->ww_link_status.ls_bssid, ww->ww_gv->gv_ssid,
	    sizeof(ww->ww_link_status.ls_bssid));
	strlcpy(ww->ww_link_status.ls_key, ww->ww_gv->gv_key,
	    sizeof(ww->ww_link_status.ls_key));
#else
	if (cf_get_string(CF_KEY_STRING_WIFI_SSID,
	    sizeof(ww->ww_link_status.ls_bssid),
	    ww->ww_link_status.ls_bssid) == 0) {
		ww->ww_link_status.ls_bssid[0] = '\0';
	}
	if (cf_get_string(CF_KEY_STRING_WIFI_KEY,
	    sizeof(ww->ww_link_status.ls_key),
	    ww->ww_link_status.ls_key) == 0) {
		ww->ww_link_status.ls_key[0] = '\0';
	}
#endif

	if (ww->ww_link_status.ls_bssid[0] == '\0' ||
	    winc_m2m_wifi_connect(ww) != M2M_SUCCESS) {
		winc_wifi_led_err(WINC_LED_ON);
		DBFPRINTF("connect failed\n");
		network_driver_soft_timeout(ww->ww_network_state, 1000);
	} else {
		ww->ww_link_state = WINC_WIFI_STATE_CONNECTING;
		DBFPRINTF("connect succeeded\n");
		if (ww->ww_link_cb != NULL) {
			ww->ww_link_status.ls_state = NETWORK_LINK_CONNECTING;
			ww->ww_link_status.ls_rssi = 0;
			(ww->ww_link_cb)(&ww->ww_link_status,
			    ww->ww_link_cb_arg);
		}
	}
}

static void
winc_wifi_handle_provision_info(struct winc_wifi *ww, tstrM2MProvisionInfo *pi)
{
	int valid = 1;

	if (pi->u8Status == M2M_SUCCESS) {
		winc_wifi_led_wifi(WINC_LED_OFF);
#ifndef CONFIG_USE_CONFIGDB
		strlcpy(ww->ww_gv->gv_ssid, (const char *)pi->au8SSID,
		    sizeof(ww->ww_gv->gv_ssid));
		strlcpy(ww->ww_gv->gv_key, (const char *)pi->au8Password,
		    sizeof(ww->ww_gv->gv_key));
		ww->ww_gv->gv_enc_type = winc_wifi_from_enc(pi->u8SecType);
		(ww->ww_control)(ww->ww_cookie, WINC_WIFI_GLOB_SAVE, NULL);
#else
		if (cf_set_string(CF_KEY_STRING_WIFI_SSID,
		    (const char *)pi->au8SSID) == 0 ||
		    cf_set_string(CF_KEY_STRING_WIFI_KEY,
		    (const char *)pi->au8Password) == 0 ||
		    cf_set_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE,
		    winc_wifi_from_enc(pi->u8SecType)) == 0) {
			valid = 0;
		} else {
			configdb_persist();
		}
#endif
	}

	if (valid) {
		ww->ww_link_state = WINC_WIFI_STATE_PROVISIONED;
		DBFPRINTF("provisioned\n");
	} else {
		DBFPRINTF("failed\n");
		ww->ww_link_state = WINC_WIFI_STATE_INIT;
		network_driver_soft_timeout(ww->ww_network_state, 100);
	}
}

static void
winc_wifi_handle_connection_state(struct winc_wifi *ww,
    tstrM2mWifiStateChanged *p)
{

	DBFPRINTF("cur state %u\n", (unsigned int)p->u8CurrState);

	if (ww->ww_link_state < WINC_WIFI_STATE_CONNECTING)
		return;

	if (p->u8CurrState == M2M_WIFI_DISCONNECTED) {
		winc_wifi_led_net(-1);
		winc_wifi_led_wifi(WINC_LED_OFF);
		DBFPRINTF("WiFi disconnect\n");
		if (ww->ww_link_state > WINC_WIFI_STATE_ACQUIRE_ADDRESS &&
		    ww->ww_link_cb != NULL) {
			ww->ww_link_status.ls_state = NETWORK_LINK_DOWN;
			ww->ww_link_status.ls_rssi = 0;
			(ww->ww_link_cb)(&ww->ww_link_status,
			    ww->ww_link_cb_arg);
		}
		ww->ww_link_state = WINC_WIFI_STATE_PROVISIONED;
	} else
	if (p->u8CurrState == M2M_WIFI_CONNECTED) {
		DBFPRINTF("WiFi connect\n");
		winc_wifi_led_err(WINC_LED_OFF);
		winc_wifi_led_wifi(WINC_LED_ON);
		ww->ww_link_state = WINC_WIFI_STATE_ACQUIRE_ADDRESS;
	}
}

static network_signal_strength_t
winc_rssi_to_enum(int8_t rssi)
{

	if (rssi >= -1)
		return NETWORK_SIG_NONE;
	else
	if (rssi >= -63)
		return NETWORK_SIG_EXCELLENT;
	else
	if (rssi >= -70)
		return NETWORK_SIG_GOOD;
	else
	if (rssi >= -78)
		return NETWORK_SIG_FAIR;
	else
		return NETWORK_SIG_POOR;
}

static void
winc_wifi_handle_rssi(struct winc_wifi *ww, const int8_t *rssi)
{

	if (ww->ww_link_state < WINC_WIFI_STATE_GET_RSSI)
		return;

	if (*rssi != ww->ww_link_status.ls_rssi) {
		ww->ww_link_status.ls_sig = winc_rssi_to_enum(*rssi);
		ww->ww_link_status.ls_rssi = *rssi;
		if (ww->ww_link_cb != NULL) {
			ww->ww_link_status.ls_state = NETWORK_LINK_UP;
			(ww->ww_link_cb)(&ww->ww_link_status,
			    ww->ww_link_cb_arg);
		}
	}

	if (ww->ww_link_state == WINC_WIFI_STATE_GET_RSSI)
		ww->ww_link_state = WINC_WIFI_STATE_CONFIGURED;
}

static void
winc_wifi_handle_dhcp_conf(struct winc_wifi *ww, tstrM2MIPConfig *p)
{
#ifndef NETWORK_LWIP
	network_ip_params_t ip;
#else
	(void) p;
#endif

	if (ww->ww_link_state != WINC_WIFI_STATE_ACQUIRE_ADDRESS)
		return;

	ww->ww_link_state = WINC_WIFI_STATE_GET_RSSI;
	winc_wifi_led_err(WINC_LED_OFF);

#ifndef NETWORK_LWIP
	ip.ip_addr = ntoh32(p->u32StaticIP);
	ip.ip_mask = ntoh32(p->u32SubnetMask);
	ip.ip_gate = ntoh32(p->u32Gateway);
	network_set_ip_params_drv(&ip);
#endif

	/* If we fail to query RSSI, just fake it */
	if (m2m_wifi_req_curr_rssi() != M2M_SUCCESS) {
		int8_t rssi = 0;
		ww->ww_link_status.ls_rssi = -1;
		winc_wifi_handle_rssi(ww, &rssi);
	}
}

#ifdef WINC_OPT_USE_WIFI_TIME
static void
winc_wifi_start_sntp(struct winc_wifi *ww)
{

	(void) m2m_wifi_enable_sntp(1);
}

#ifdef TIMER_OPT_EVENTS
/*ARGSUSED*/
static void
winc_wifi_query_sntp(struct timer_event *te)
{

	/*
	 * Adjust period to slightly randomise the next
	 * query +/- ~30 minutes
	 */
	te->te_period = WINC_SNTP_PERIOD;

	/* With any luck, the WINC will send an NTP query */
	m2m_wifi_get_sytem_time();
}
#endif /* TIMER_OPT_EVENTS */

static void
winc_wifi_handle_sys_time(struct winc_wifi *ww, tstrSystemTime *p)
{
	struct timeval tv;
	struct tm tm;

	if (p->u8Second > 59 || p->u8Minute > 59 || p->u8Hour > 23 ||
	    p->u8Day < 1 || p->u8Day > 31 || p->u8Month < 1 ||
	    p->u8Month > 12 || p->u16Year < 2000) {
		return;
	}

	memset(&tm, 0, sizeof(tm));
	tm.tm_sec = p->u8Second;
	tm.tm_min = p->u8Minute;
	tm.tm_hour = p->u8Hour;
	tm.tm_mday = p->u8Day;
	tm.tm_mon = p->u8Month - 1;
	tm.tm_year = p->u16Year - 1900;
	tv.tv_sec = mk_gmtime(&tm);
	tv.tv_usec = 0;
	timer_set_sys_time(&tv);

#ifdef TIMER_OPT_EVENTS
	if (ww->ww_sntp_ev.te_period == 0) {
		ww->ww_sntp_ev.te_period = WINC_SNTP_PERIOD;
		ww->ww_sntp_ev.te_event_cb = winc_wifi_query_sntp;
		ww->ww_sntp_ev.te_cookie = ww;
		timer_add_event(&ww->ww_sntp_ev, ww->ww_sntp_ev.te_period);
	}
#endif /* TIMER_OPT_EVENTS */
}
#endif /* WINC_OPT_USE_WIFI_TIME */

static void
winc_wifi_link_cb(uint8_t msg, void *arg)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);

	switch (msg) {
	case M2M_WIFI_RESP_PROVISION_INFO:
		DBFPRINTF("PROVISION_INFO\n");
		winc_wifi_handle_provision_info(ww, arg);
		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		DBFPRINTF("DHCP_CONF\n");
		winc_wifi_handle_dhcp_conf(ww, arg);
		break;

	case M2M_WIFI_RESP_GET_SYS_TIME:
		DBFPRINTF("GET_SYS_TIME\n");
#ifdef WINC_OPT_USE_WIFI_TIME
		winc_wifi_handle_sys_time(ww, arg);
#endif
		break;

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		DBFPRINTF("CON_STATE_CHANGED\n");
		winc_wifi_handle_connection_state(ww, arg);
		break;

	case M2M_WIFI_RESP_CURRENT_RSSI:
		DBFPRINTF("CURRENT_RSSI\n");
		winc_wifi_handle_rssi(ww, arg);
		break;

	default:
		DBFPRINTF("unhandled event 0x%02x\n", (unsigned int)msg);
		break;
	}
}

static void
winc_wifi_link_check(void *arg)
{
	struct winc_wifi *ww = arg;

	if (ww->ww_link_state == WINC_WIFI_STATE_SHOWTIME)
		m2m_wifi_req_curr_rssi();
}

#ifdef WINC_OPT_OTA_UPDATE
static void
winc_wifi_ota_update_cb(uint8 u8OtaUpdateStatusType, uint8 u8OtaUpdateStatus)
{
	struct winc_wifi *ww = winc_wifi_state;
	int8_t rv;

	assert(ww != NULL);

	switch (u8OtaUpdateStatusType) {
	case DL_STATUS:
		/* Download status */
		if (u8OtaUpdateStatus == OTA_STATUS_SUCSESS) {
			/*
			 * Download succeeded. Switch active image to it.
			 */
			DBFPRINTF("download succeeded.\n");

			if ((rv = m2m_ota_switch_firmware() != M2M_SUCCESS))
				ww->ww_ota_state = rv;
			else
				ww->ww_ota_state = WINC_OTA_STATUS_SWITCHING;
		} else {
			DBFPRINTF("download failed with status %u\n",
			    (unsigned int)u8OtaUpdateStatus);
			ww->ww_ota_state = u8OtaUpdateStatus;
		}
		break;

	case SW_STATUS:
		/* Switch firmware status */
		if (u8OtaUpdateStatus == OTA_STATUS_SUCSESS) {
			DBFPRINTF("switch succeeded. Reboot required.\n");
			ww->ww_ota_state = WINC_OTA_STATUS_UPDATED;
		} else {
			DBFPRINTF("switch failed with status %u\n",
			    (unsigned int)u8OtaUpdateStatus);
			ww->ww_ota_state = u8OtaUpdateStatus;
		}
		break;

	case RB_STATUS:
		/* Rollback status */
		if (u8OtaUpdateStatus == OTA_STATUS_SUCSESS) {
			DBFPRINTF("rollback status %u\n",
			    (unsigned int)u8OtaUpdateStatus);
			ww->ww_ota_state = WINC_OTA_STATUS_UPDATED;
		} else {
			ww->ww_ota_state = u8OtaUpdateStatus;
		}
		break;

	default:
		DBFPRINTF("bad status type %u\n",
		    (unsigned int)u8OtaUpdateStatusType);
		break;
	}
}
#endif	/* WINC_OPT_OTA_UPDATE */

static uint32_t
winc_wifi_process_events(void *arg)
{
	struct winc_wifi *ww = arg;
	rtos_saved_ipl_t ipl;
	uint32_t rv = 20;
	uint8_t event;
#ifdef DEBUG_ENABLED
	static uint8_t old_state = 0xffu;
#endif
	uint8_t state_change;

//	winc_wifi_led_net(WINC_LED_ON);

	do {
		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		event = ww->ww_wifi_event;
		ww->ww_wifi_event = 0;
		rtos_ipl_restore(ipl);

		if (event)
			m2m_wifi_handle_events(NULL);
	} while (event);

#ifdef DEBUG_ENABLED
	state_change = old_state != ww->ww_link_state;
	if (state_change)
		old_state = ww->ww_link_state;
#else
	state_change = 0;
#endif

	switch (ww->ww_link_state) {
	case WINC_WIFI_STATE_INIT:
		if (state_change)
			DBFPRINTF("INIT\n");
		winc_wifi_start_provision(ww);
		break;

	case WINC_WIFI_STATE_NEW_PROVISION:
		if (state_change)
			DBFPRINTF("NEW_PROVISION\n");
		winc_wifi_new_provision(ww);
		break;

	case WINC_WIFI_STATE_PROVISIONING:
		if (state_change)
			DBFPRINTF("PROVISIONING\n");
		ww->ww_prov_blink = 1 - ww->ww_prov_blink;
		winc_wifi_led_wifi(ww->ww_prov_blink ? WINC_LED_ON :
		    WINC_LED_OFF);
		network_driver_soft_timeout(ww->ww_network_state, 500);
		break;

	case WINC_WIFI_STATE_PROVISIONED:
		if (state_change)
			DBFPRINTF("PROVISIONED\n");
		winc_wifi_start_connecting(ww);
		break;

	case WINC_WIFI_STATE_CONNECTING:
		if (state_change)
			DBFPRINTF("CONNECTING\n");
		break;

	case WINC_WIFI_STATE_ACQUIRE_ADDRESS:
		if (state_change)
			DBFPRINTF("ACQUIRE_ADDRESS\n");
#ifdef NETWORK_LWIP
		/* Start the DHCP client */
		winc_wifi_handle_dhcp_conf(ww, NULL);
#endif
		break;

	case WINC_WIFI_STATE_GET_RSSI:
		if (state_change)
			DBFPRINTF("GET_RSSI\n");
		break;

	case WINC_WIFI_STATE_CONFIGURED:
		if (state_change)
			DBFPRINTF("CONFIGURED\n");
		ww->ww_link_state = WINC_WIFI_STATE_SHOWTIME;
#ifdef WINC_OPT_USE_WIFI_TIME
		winc_wifi_start_sntp(ww);
#endif
#ifdef WINC_OPT_OTA_UPDATE
		m2m_ota_init(winc_wifi_ota_update_cb, NULL);
#endif
		break;

	case WINC_WIFI_STATE_SHOWTIME:
		rv = RTOS_MAX_SLEEP_MS;
		break;
	}

//	winc_wifi_led_net(WINC_LED_OFF);

	return rv;
}

static void
winc_wifi_process_events_native(void *arg)
{

	(void) winc_wifi_process_events(arg);
}

static void
winc_wifi_get_mac(void *arg, uint8_t *mac)
{

	(void) arg;

	(void) m2m_wifi_get_mac_address(mac);
}

static void
winc_wifi_set_ip_params(void *arg, const network_ip_params_t *ip)
{

	/*
	 * Do nothing for now - the WINC1500 has its own DHCP client and
	 * it would seem the API to support static IP configuration is in
	 * the process of being deprecated.
	 */
	(void) arg;
	(void) ip;
}

static network_sock_t
winc_wifi_open(void *arg, network_sock_type_t stype,
    network_sock_status_callback_t func, void *cbarg)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws;
	uint8_t flag;
	uint8 sock_type;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED)
		return NETWORK_SOCKET_INVALID;

	switch (stype) {
	case NETWORK_SOCK_TYPE_TCP:
		flag = WINC_SOCK_FLAG_TCP;
		sock_type = SOCK_STREAM;
		break;
	case NETWORK_SOCK_TYPE_UDP:
		flag = 0;
		sock_type = SOCK_DGRAM;
		break;
	default:
		return NETWORK_SOCKET_INVALID;
	}

	for (ws = ww->ww_sockets; ws != NULL; ws = ws->ws_next) {
		if ((ws->ws_flags & WINC_SOCK_FLAG_IN_USE) == 0)
			break;
	}

	if (ws == NULL) {
		if (ww->ww_nsockets == MAX_SOCKET)
			return NETWORK_SOCKET_INVALID;

		ws = zone_calloc(1, sizeof(*ws));
		if (ws == NULL)
			return NETWORK_SOCKET_INVALID;

		ws->ws_winc = ww;
		ws->ws_next = ww->ww_sockets;
		ww->ww_sockets = ws;
		ww->ww_nsockets++;
	}

	ws->ws_sock = socket(AF_INET, sock_type, 0);
	if (WINC_BAD_SOCKET(ws->ws_sock)) {
		if (ws->ws_sock >= MAX_SOCKET) {
			close(ws->ws_sock);
			ws->ws_sock = WINC_SOCK_INVALID;
		}
		return NETWORK_SOCKET_INVALID;
	}

	DBFPRINTF("sock %u, %s\n", (unsigned int)ws->ws_sock,
	    (stype == NETWORK_SOCK_TYPE_TCP) ? "TCP" : "UDP");

	ws->ws_listen_sock = WINC_SOCK_INVALID;
	ws->ws_flags = flag | WINC_SOCK_FLAG_IN_USE;
	ws->ws_status_callback = func;
	ws->ws_status_callback_arg = cbarg;
	ww->ww_asf2sock[ws->ws_sock] = ws;

	SIMPLEQ_INIT(&ws->ws_rx_buffs);

	return ws;
}

static void
winc_wifi_close(void *arg, network_sock_t sock)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;
	struct winc_socket_buffer *sb;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_flags & WINC_SOCK_FLAG_IN_USE) == 0) {
		return;
	}

	if (!WINC_BAD_SOCKET(ws->ws_sock)) {
		DBFPRINTF("sock %u: close\n", (unsigned int)ws->ws_sock);
		ww->ww_asf2sock[ws->ws_sock] = NULL;
		close(ws->ws_sock);
		ws->ws_sock = WINC_SOCK_INVALID;
	}

	if (!WINC_BAD_SOCKET(ws->ws_listen_sock)) {
		DBFPRINTF("sock %u: close listen sock %u\n",
		    (unsigned int)ws->ws_sock,
		    (unsigned int)ws->ws_listen_sock);
		ww->ww_asf2sock[ws->ws_listen_sock] = NULL;
		close(ws->ws_listen_sock);
		ws->ws_listen_sock = WINC_SOCK_INVALID;
	}

	while ((sb = SIMPLEQ_FIRST(&ws->ws_rx_buffs)) != NULL) {
		SIMPLEQ_REMOVE_HEAD(&ws->ws_rx_buffs, sb_qent);
		sb->sb_len = 0;
		SIMPLEQ_INSERT_HEAD(&ww->ww_free_socket_buffs, sb, sb_qent);
	}

	ws->ws_flags = 0;
	ws->ws_lport = 0;
	ws->ws_dport = 0;
	ws->ws_daddr = 0;
	ws->ws_rx_length = 0;
	ws->ws_tx_pending = 0;
}

static uint8_t
winc_wifi_connect(void *arg, network_sock_t sock,
    const network_sock_params_t *sp)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;
	struct sockaddr_in sa;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED)
		return 0;

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ACTIVE) != 0 ||
	    ws->ws_lport != 0 || ws->ws_dport != 0 || sp->sp_dport == 0) {
		/* Not open, or we're already busy on this socket */
		return 0;
	}

	sa.sin_family = AF_INET;
	sa.sin_port = hton16(sp->sp_dport);
	sa.sin_addr.s_addr = hton32(sp->sp_dest_ip);

	if (ws->ws_flags & WINC_SOCK_FLAG_TCP) {
		if (connect(ws->ws_sock, (struct sockaddr *)&sa, sizeof(sa)) !=
		    SOCK_ERR_NO_ERROR) {
			return 0;
		}
	} else {
		if (bind(ws->ws_sock, (struct sockaddr *)&sa, sizeof(sa)) !=
		    SOCK_ERR_NO_ERROR) {
			return 0;
		}
		ws->ws_lport = sp->sp_sport;
	}

	/*
	 * We'll get a CONNECT/BIND callback when this completes.
	 */
	ws->ws_dport = sp->sp_dport;
	ws->ws_daddr = sp->sp_dest_ip;
	ws->ws_flags |= WINC_SOCK_FLAG_ACTIVE;

	return 1;
}

static uint8_t
winc_wifi_listen(void *arg, network_sock_t sock, uint16_t lport)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;
	struct sockaddr_in sa;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED)
		return 0;

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ACTIVE) != 0 ||
	    ws->ws_lport != 0 || ws->ws_dport != 0 || lport == 0) {
		/* Not open, or we're already busy on this socket */
		return 0;
	}

	/*
	 * Atmel's library requires us to go through the bind/listen dance
	 * rather than the single 'listen' that we're used to here.
	 */
	sa.sin_family = AF_INET;
	sa.sin_port = hton16(lport);
	sa.sin_addr.s_addr = 0;
	if (bind(ws->ws_sock, (struct sockaddr *)&sa, sizeof(sa)) !=
	    SOCK_ERR_NO_ERROR) {
		return 0;
	}

	/*
	 * We'll get a BIND callback when this completes.
	 */
	ws->ws_lport = lport;
	ws->ws_flags |= WINC_SOCK_FLAG_ACTIVE;

	return 1;
}

static uint16_t
winc_wifi_rx_avail(void *arg, network_sock_t sock)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED)
		return 0;

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) == 0) {
		/* Not open, or we're already busy on this socket */
		return 0;
	}

	return ws->ws_rx_length;
}

static struct winc_socket_buffer *
winc_get_socket_buffer(struct winc_wifi *ww)
{
	struct winc_socket_buffer *sb;

	if ((sb = SIMPLEQ_FIRST(&ww->ww_free_socket_buffs)) != NULL) {
		SIMPLEQ_REMOVE_HEAD(&ww->ww_free_socket_buffs, sb_qent);
	} else
	if (ww->ww_nsocket_buffs == WINC_SOCK_BUFFER_MAX ||
	    (sb = zone_calloc(1, sizeof(*sb))) == NULL) {
		return NULL;
	} else {
		sb->sb_next = ww->ww_socket_buffs;
		ww->ww_socket_buffs = sb;
		ww->ww_nsocket_buffs++;
	}

	return sb;
}

static uint8_t
winc_wifi_post_recv(struct winc_wifi *ww, struct winc_socket *ws)
{
	struct winc_socket_buffer *sb;
	sint16 x;

	assert(SIMPLEQ_EMPTY(&ws->ws_rx_buffs));

	if ((sb = winc_get_socket_buffer(ww)) == NULL)
		return 0;

	SIMPLEQ_INSERT_HEAD(&ws->ws_rx_buffs, sb, sb_qent);

	ws->ws_rx_length = 0;
	ws->ws_rx_idx = 0;

	/* Initiate a receive into our Rx bounce buffer */
	if ((ws->ws_flags & WINC_SOCK_FLAG_TCP) != 0) {
		DBFPRINTF("sock %u: TCP\n", (unsigned int)ws->ws_sock);
		x = recv(ws->ws_sock, sb->sb_buff, sizeof(sb->sb_buff), 0);
	} else {
		DBFPRINTF("sock %u: UDP\n", (unsigned int)ws->ws_sock);
		x = recvfrom(ws->ws_sock, sb->sb_buff, sizeof(sb->sb_buff), 0);
	}

#ifdef DEBUG_ENABLED
	if (x != SOCK_ERR_NO_ERROR) {
		DBFPRINTF("sock %u: error %" PRId16" \n",
		    (unsigned int)ws->ws_sock, x);
	}
#endif

	return x == SOCK_ERR_NO_ERROR;
}

static uint16_t
winc_wifi_read_buffer(struct winc_wifi *ww, struct winc_socket *ws,
    uint8_t *buff, uint16_t len)
{
	struct winc_socket_buffer *sb;
	unsigned int l, rxlen, rxidx;

	DBFPRINTF("sock %u: len %" PRIu16 "\n", (unsigned int)ws->ws_sock, len);

	if (ws->ws_rx_length == 0) {
		DBFPRINTF("sock %u: Rx buffer empty\n",
		    (unsigned int)ws->ws_sock);
		return 0;
	}

	winc_wifi_led_net(WINC_LED_ON);

	rxlen = (unsigned int)ws->ws_rx_length;
	rxidx = (unsigned int)ws->ws_rx_idx;

	while (len && rxlen) {
		DBFPRINTF("sock %u: len %" PRIu16 ", rxlen %u, rxidx %u\n",
		    (unsigned int)ws->ws_sock, len, rxlen, rxidx);

		sb = SIMPLEQ_FIRST(&ws->ws_rx_buffs);
		assert(sb != NULL);
		assert(sb->sb_len <= rxlen);

		l = sb->sb_len;
		if (l > len)
			l = len;
		len -= l;

		memcpy(buff, &sb->sb_buff[rxidx], l);

		rxidx += l;
		rxlen -= l;
		buff += l;

		sb->sb_len -= l;
		if (sb->sb_len == 0) {
			DBFPRINTF("sock %u: done with socket buff @ %p\n",
			    (unsigned int)ws->ws_sock, (void *)sb);
			SIMPLEQ_REMOVE_HEAD(&ws->ws_rx_buffs, sb_qent);
			SIMPLEQ_INSERT_HEAD(&ww->ww_free_socket_buffs, sb,
			    sb_qent);
			rxidx = 0;
		}
	}

	l = (unsigned int)ws->ws_rx_length - rxlen;
	ws->ws_rx_length = (uint16_t)rxlen;
	ws->ws_rx_idx = (uint16_t)rxidx;

	if (rxlen == 0 && winc_wifi_post_recv(ww, ws) == 0) {
		DBFPRINTF("sock %u: post next failed\n",
		    (unsigned int)ws->ws_sock);
		l = NETWORK_SOCK_ERR_EOF;
	}

	winc_wifi_led_net(WINC_LED_OFF);

	DBFPRINTF("sock %u: read %u bytes\n", (unsigned int)ws->ws_sock, l);
	return (uint16_t)l;
}

static uint16_t
winc_wifi_read(void *arg, network_sock_t sock, void *buff, uint16_t len)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;

	DBFPRINTF("sock %u: len %" PRIu16 "\n", (unsigned int)ws->ws_sock, len);

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED) {
		DBFPRINTF("sock %u: link down\n", (unsigned int)ws->ws_sock);
		return NETWORK_SOCK_ERR_NO_LINK;
	}

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) == 0) {
		DBFPRINTF("sock %u: not open\n", (unsigned int)ws->ws_sock);
		return NETWORK_SOCK_ERR_NOT_OPEN;
	}

	len = winc_wifi_read_buffer(ww, ws, buff, len);

	return len;
}

static uint16_t
winc_wifi_recv(void *arg, network_sock_t sock, void *buff, uint16_t len,
    network_sock_params_t *sp)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;

	DBFPRINTF("sock %u: len %" PRIu16 "\n", (unsigned int)ws->ws_sock, len);

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED) {
		DBFPRINTF("sock %u: link down\n", (unsigned int)ws->ws_sock);
		return NETWORK_SOCK_ERR_NO_LINK;
	}

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) == 0) {
		DBFPRINTF("sock %u: not open\n", (unsigned int)ws->ws_sock);
		return NETWORK_SOCK_ERR_NOT_OPEN;
	}

	if ((ws->ws_flags & WINC_SOCK_FLAG_TCP) != 0) {
		DBFPRINTF("sock %u: not UDP\n", (unsigned int)ws->ws_sock);
		return NETWORK_SOCK_ERR_BAD_CALL;
	}

	if (sp) {
		sp->sp_dest_ip = ws->ws_daddr;
		sp->sp_dport = ws->ws_dport;
	}

	len = winc_wifi_read_buffer(ww, ws, buff, len);

	return len;
}

static uint16_t
winc_wifi_tx_free(void *arg, network_sock_t sock)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED)
		return NETWORK_SOCK_ERR_NO_LINK;

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) == 0) {
		return NETWORK_SOCK_ERR_NOT_OPEN;
	}

	if (ws->ws_tx_pending)
		return 0;

	return SOCKET_BUFFER_MAX_LENGTH;
}

static network_sock_state_t
winc_wifi_sock_status(void *arg, network_sock_t sock,
    network_sock_params_t *sp)
{
//	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;
	network_sock_state_t st;

	(void) arg;

	memset(sp, 0, sizeof(*sp));

	if ((ws->ws_flags & WINC_SOCK_FLAG_TCP) == 0)
		st = NETWORK_SOCKS_UDP;
	else
	if ((ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) != 0)
		st = NETWORK_SOCKS_ESTABLISHED;
	else
	if (WINC_BAD_SOCKET(ws->ws_sock) && WINC_BAD_SOCKET(ws->ws_listen_sock))
		st = NETWORK_SOCKS_CLOSING;
	else
	if (!WINC_BAD_SOCKET(ws->ws_listen_sock) && ws->ws_lport)
		st = NETWORK_SOCKS_LISTEN;
	else
	if (!WINC_BAD_SOCKET(ws->ws_sock) && ws->ws_dport)
		st = NETWORK_SOCKS_SYNSENT;
	else
		st = NETWORK_SOCKS_NONE;

	switch (st) {
	case NETWORK_SOCKS_ESTABLISHED:
	case NETWORK_SOCKS_SYNSENT:
	case NETWORK_SOCKS_CLOSING:
		sp->sp_dest_ip = ws->ws_daddr;
		sp->sp_dport = ws->ws_dport;
		/*FALLTHROUGH*/
	case NETWORK_SOCKS_UDP:
	case NETWORK_SOCKS_LISTEN:
		sp->sp_sport = ws->ws_lport;
		break;
	default:
		break;
	}

	return st;
}

static uint16_t
winc_wifi_write(void *arg, network_sock_t sock, const void *buff, uint16_t len,
    const uint8_t *dstmac)
{
	struct winc_wifi *ww = arg;
	struct winc_socket *ws = sock;
	sint16 x;

	if (ww->ww_link_state < WINC_WIFI_STATE_CONFIGURED) {
		DBFPRINTF("no link\n");
		return NETWORK_SOCK_ERR_NO_LINK;
	}

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    (ws->ws_flags & WINC_SOCK_FLAG_ESTABLISHED) == 0) {
		DBFPRINTF("not open\n");
		return NETWORK_SOCK_ERR_NOT_OPEN;
	}

	if (dstmac || len > SOCKET_BUFFER_MAX_LENGTH) {
		DBFPRINTF("bad call\n");
		return NETWORK_SOCK_ERR_BAD_CALL;
	}

	if (len == 0)
		return 0;

	if (ws->ws_tx_pending) {
		DBFPRINTF("busy\n");
		return NETWORK_SOCK_ERR_BUSY;
	}

	winc_wifi_led_net(WINC_LED_ON);
	if (ws->ws_flags & WINC_SOCK_FLAG_TCP) {
		x = send(ws->ws_sock, (void *)(uintptr_t)buff, len, 0);
		DBFPRINTF("send rv = %" PRId16 "\n", x);
	} else {
		struct sockaddr_in sa;

		sa.sin_family = AF_INET;
		sa.sin_port = hton16(ws->ws_dport);
		sa.sin_addr.s_addr = hton32(ws->ws_daddr);

		x = sendto(ws->ws_sock, (void *)(uintptr_t)buff, len, 0,
		    (struct sockaddr *)&sa, sizeof(sa));
		DBFPRINTF("sendto rv = %" PRId16 "\n", x);
	}
	if (x == SOCK_ERR_NO_ERROR)
		ws->ws_tx_pending = len;
	else
	if (x == SOCK_ERR_BUFFER_FULL)
		len = NETWORK_SOCK_ERR_BUSY;
	else
		len = NETWORK_SOCK_ERR_BAD_CALL;
	winc_wifi_led_net(WINC_LED_OFF);

	return len;
}

static void
winc_wifi_link_status(void *arg, network_link_status_callback_t cb, void *cbarg)
{
	struct winc_wifi *ww = arg;

	ww->ww_link_cb = cb;
	ww->ww_link_cb_arg = cbarg;
}

#ifdef WINC_OPT_GPIO
static int
winc_wifi_gpio2m2m(uint8_t gp, uint8_t *m2m)
{
	switch (gp) {
	case WINC_GPIO_PIN_3:
		*m2m = M2M_PERIPH_GPIO3;
		return 0;
	case WINC_GPIO_PIN_4:
		*m2m = M2M_PERIPH_GPIO4;
		return 0;
	case WINC_GPIO_PIN_5:
		*m2m = M2M_PERIPH_GPIO5;
		return 0;
	case WINC_GPIO_PIN_6:
		*m2m = M2M_PERIPH_GPIO6;
		return 0;
	}
	return -1;
}
#endif /* WINC_OPT_GPIO */

static int8_t
winc_wifi_locked_ioctl(void *cookie, uint8_t cmd, void *arg)
{
	int8_t rv = 0;
	struct winc_wifi *ww = cookie;
#ifdef WINC_OPT_GPIO
	struct winc_gpio *wg = arg;
	uint8_t m2m;
#else
	(void) arg;
#endif

	switch (cmd) {
	case NETWORK_IOCTL_DHCP_CLIENT_START:
	case NETWORK_IOCTL_DHCP_CLIENT_STOP:
		/*
		 * We'll only see these in "native" mode. They're handled
		 * on our behalf in network_lwip.c when in LWIP mode.
		 *
		 * In native mode, the WINC module has an internal DHCP
		 * client which is outwith our control, so just ignore.
		 */
		break;

	case NETWORK_IOCTL_WIFI_PROVISION:
		if (arg == NULL)
			return -1;
		ww->ww_prov_arg = *((struct winc_prov_arg *)arg);
		ww->ww_prov_old_state = ww->ww_link_state;
		ww->ww_link_state = WINC_WIFI_STATE_NEW_PROVISION;
		network_driver_soft_timeout(ww->ww_network_state, 0);
		break;

#ifdef WINC_OPT_GPIO
	case WINC_IOCTL_INIT_GPIO:
		if (winc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		if (wg->wg_value & WINC_GPIO_DIRECTION_OUT)
			m2m_periph_gpio_set_dir(m2m, 1);
		else
			m2m_periph_gpio_set_dir(m2m, 0);

		if (wg->wg_value & WINC_GPIO_PULLUP_ON)
			m2m_periph_gpio_pullup_ctrl(m2m, 1);
		else
			m2m_periph_gpio_pullup_ctrl(m2m, 0);
		break;

	case WINC_IOCTL_WRITE_GPIO:
		if (winc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		m2m_periph_gpio_set_val(m2m, wg->wg_value ? 1 : 0);
		break;

	case WINC_IOCTL_READ_GPIO:
		if (winc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		m2m_periph_gpio_get_val(m2m, &wg->wg_value);
		break;
#endif /* WINC_OPT_GPIO */

#ifdef WINC_OPT_OTA_UPDATE
	case WINC_IOCTL_OTA_START:
		if (ww->ww_link_state != WINC_WIFI_STATE_SHOWTIME)
			rv = WINC_OTA_STATUS_NOT_ASSOCIATED;
		else
		if (ww->ww_ota_state == WINC_OTA_STATUS_IDLE) {
			DBFPRINTF("Initiate OTA\n");
			if ((rv = m2m_ota_start_update((uint8 *)arg)) ==
			    OTA_STATUS_SUCSESS) {
				DBFPRINTF("OTA in progress\n");
				ww->ww_ota_state = rv =
				    WINC_OTA_STATUS_IN_PROGRESS;
			} else {
				DBFPRINTF("OTA error %d\n", (int)rv);
			}
		} else
		if (ww->ww_ota_state == WINC_OTA_STATUS_IN_PROGRESS)
			rv = WINC_OTA_ERR_BUSY;
		else
			rv = ww->ww_ota_state;
		break;

	case WINC_IOCTL_OTA_STATUS:
		rv = ww->ww_ota_state;
		switch (rv) {
		case WINC_OTA_STATUS_IDLE:
		case WINC_OTA_STATUS_IN_PROGRESS:
		case WINC_OTA_STATUS_SWITCHING:
			break;

		default:
			ww->ww_ota_state = WINC_OTA_STATUS_IDLE;
			break;
		}
		break;

	case WINC_IOCTL_OTA_ROLLBACK:
		if (ww->ww_ota_state == WINC_OTA_STATUS_IDLE) {
			rv = m2m_ota_rollback();
			if (rv == OTA_STATUS_SUCSESS) {
				DBFPRINTF("OTA rollback in progress\n");
				ww->ww_ota_state = rv =
				    WINC_OTA_STATUS_IN_PROGRESS;
			} else {
				DBFPRINTF("OTA rollback error %d\n", (int)rv);
			}
		} else
			rv = ww->ww_ota_state;
		break;
#endif

	default:
		rv = -1;
		break;
	}

	return rv;
}

static void
winc_wifi_sock_callback_common(struct winc_socket *ws, network_sock_status_t st)
{
	network_sock_status_callback_t cb;

	if ((cb = ws->ws_status_callback) != NULL)
		(cb)(st, ws->ws_status_callback_arg);
}

static void
winc_wifi_sock_disconnect(struct winc_socket *ws)
{

//	ws->ws_flags &= ~WINC_SOCK_FLAG_ESTABLISHED;
	winc_wifi_sock_callback_common(ws, NETWORK_STATUS_DISCONNECT);
}

#ifndef NETWORK_LWIP
static void
winc_wifi_sock_connect(struct winc_socket *ws)
{

	ws->ws_flags |= WINC_SOCK_FLAG_ESTABLISHED;
	winc_wifi_sock_callback_common(ws, NETWORK_STATUS_CONNECT);
}

static void
winc_wifi_sock_timeout(struct winc_socket *ws)
{

//	ws->ws_flags &= ~WINC_SOCK_FLAG_ESTABLISHED;
	winc_wifi_sock_callback_common(ws, NETWORK_STATUS_TIMEOUT);
}

static void
winc_wifi_sock_recv(struct winc_socket *ws)
{

	winc_wifi_sock_callback_common(ws, NETWORK_STATUS_RX_READY);
}

static void
winc_wifi_sock_send_ok(struct winc_socket *ws)
{

	winc_wifi_sock_callback_common(ws, NETWORK_STATUS_TX_DONE);
}

static void
winc_wifi_socket_handle_bind(struct winc_socket *ws,
    tstrSocketBindMsg *msg)
{

	if (msg == NULL || msg->status != 0) {
		/* Can't do much else here */
		DBFPRINTF("error %d\n", msg ? (int)msg->status : -1);
		winc_wifi_sock_disconnect(ws);
	} else
	if (!WINC_BAD_SOCKET(ws->ws_listen_sock)) {
		DBFPRINTF("dup BIND?\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (ws->ws_lport == 0) {
		DBFPRINTF("bogons!\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (ws->ws_flags & WINC_SOCK_FLAG_TCP) {
		/* Initiate the listen */
		DBFPRINTF("sock %u: initiating TCP listen\n",
		    (unsigned int)ws->ws_sock);
		ws->ws_listen_sock = ws->ws_sock;
		ws->ws_sock = WINC_SOCK_INVALID;
		listen(ws->ws_listen_sock, 0);
	} else {
		DBFPRINTF("sock %u: posting UDP recv\n",
		    (unsigned int)ws->ws_sock);
		if (winc_wifi_post_recv(ws->ws_winc, ws) == 0) {
			close(ws->ws_sock);
			ws->ws_sock = WINC_SOCK_INVALID;
		} else {
			winc_wifi_sock_connect(ws);
		}
	}
}

static void
winc_wifi_socket_handle_listen(struct winc_socket *ws,
    tstrSocketListenMsg *msg)
{

	/*
	 * Judging from the Atmel source code, this is simply
	 * an ACK that the listen succeeded or failed.
	 * We don't need to do anything else at this point.
	 */
	if (msg == NULL || msg->status != 0) {
		/* Can't do much else here */
		DBFPRINTF("error %d, port %u\n", msg ? (int)msg->status : -1,
		    (unsigned int)ws->ws_lport);
		winc_wifi_sock_disconnect(ws);
	} else
	if (WINC_BAD_SOCKET(ws->ws_listen_sock)) {
		DBFPRINTF("spurious LISTEN?\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (ws->ws_lport == 0) {
		DBFPRINTF("bogons!\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (accept(ws->ws_listen_sock, NULL, NULL) != SOCK_ERR_NO_ERROR) {
		DBFPRINTF("accept failed\n");
		winc_wifi_sock_disconnect(ws);
	}
}

static void
winc_wifi_socket_handle_accept(struct winc_socket *ws,
    tstrSocketAcceptMsg *msg)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);

	/*
	 * Someone's connecting to our listener.
	 */
	if (WINC_BAD_SOCKET(ws->ws_listen_sock)) {
		DBFPRINTF("spurious ACCEPT?\n");
	} else
	if (ws->ws_lport == 0) {
		DBFPRINTF("bogons!\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (!WINC_BAD_SOCKET(ws->ws_sock)) {
		/*
		 * We don't permit multiple connections for now
		 */
		DBFPRINTF("no multi\n");
		close(msg->sock);
	} else
	if (WINC_BAD_SOCKET(msg->sock)) {
		DBFPRINTF("bad sock\n");
	} else
	if (ww->ww_asf2sock[msg->sock] != NULL) {
		DBFPRINTF("duplicate socket\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (accept(ws->ws_listen_sock, NULL, NULL) != SOCK_ERR_NO_ERROR) {
		/*
		 * No idea why we do two accept() calls, but that's what
		 * Atmel's example does...
		 */
		DBFPRINTF("accept failed\n");
		winc_wifi_sock_disconnect(ws);
	} else {
		ws->ws_sock = msg->sock;
		ws->ws_dport = ntoh16(msg->strAddr.sin_port);
		ws->ws_daddr = ntoh32(msg->strAddr.sin_addr.s_addr);
		ww->ww_asf2sock[ws->ws_sock] = ws;

		DBFPRINTF("sock %u: posting TCP recv\n",
		    (unsigned int)ws->ws_sock);
		if (winc_wifi_post_recv(ww, ws) == 0) {
			ww->ww_asf2sock[ws->ws_sock] = NULL;
			ws->ws_sock = WINC_SOCK_INVALID;
			close(msg->sock);
		} else {
			winc_wifi_sock_connect(ws);
		}
	}
}

static void
winc_wifi_socket_handle_connect(struct winc_socket *ws,
    tstrSocketConnectMsg *msg)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);

	if (WINC_BAD_SOCKET(ws->ws_sock)) {
		DBFPRINTF("spurious CONNECT?\n");
	} else
	if (msg == NULL || msg->s8Error < 0) {
		/* Can't do much else here */
		DBFPRINTF("error %d\n", msg ? (int)msg->s8Error : -1);
		winc_wifi_sock_disconnect(ws);
	} else
	if (ws->ws_dport == 0) {
		DBFPRINTF("bogons!\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (WINC_BAD_SOCKET(msg->sock)) {
		DBFPRINTF("bad sock\n");
		winc_wifi_sock_disconnect(ws);
	} else
	if (ww->ww_asf2sock[msg->sock] != ws || msg->sock != ws->ws_sock) {
		DBFPRINTF("socket mismatch\n");
	} else {
		ws->ws_lport = 0xffffu;	/* Wish we could find this out... */

		if (winc_wifi_post_recv(ww, ws) == 0) {
			winc_wifi_sock_disconnect(ws);
		} else {
			winc_wifi_sock_connect(ws);
		}
	}
}

static void
winc_wifi_recv_cleanup(struct winc_wifi *ww, struct winc_socket *ws)
{
	struct winc_socket_buffer *sb;

	while ((sb = SIMPLEQ_FIRST(&ws->ws_rx_buffs)) != NULL) {
		SIMPLEQ_REMOVE_HEAD(&ws->ws_rx_buffs, sb_qent);
		SIMPLEQ_INSERT_HEAD(&ww->ww_free_socket_buffs, sb, sb_qent);
	}

	ws->ws_rx_length = 0;
	ws->ws_rx_idx = 0;
}

static void
winc_wifi_socket_handle_recv(struct winc_wifi *ww, struct winc_socket *ws,
    uint8_t rcv_from, tstrSocketRecvMsg *msg)
{
	struct winc_socket_buffer *sb;

	if (msg == NULL || WINC_BAD_SOCKET(ws->ws_sock) ||
	    ws->ws_winc->ww_asf2sock[ws->ws_sock] != ws) {
		DBFPRINTF("msg %p, sock %u\n", (void *)msg,
		    (unsigned int)ws->ws_sock);
		winc_wifi_recv_cleanup(ww, ws);
		return;
	}

	if (msg->s16BufferSize < 0) {
		/* Some kind of error */
		switch (msg->s16BufferSize) {
		case SOCK_ERR_TIMEOUT:
			DBFPRINTF("sock %u: TIMEOUT\n",
			    (unsigned int)ws->ws_sock);
			winc_wifi_sock_timeout(ws);
			break;

		default:
			DBFPRINTF("sock %u: err %" PRId16 "\n",
			    (unsigned int)ws->ws_sock, msg->s16BufferSize);
			goto clean_discon;

		case SOCK_ERR_CONN_ABORTED:
			DBFPRINTF("sock %u: ABORTED\n",
			    (unsigned int)ws->ws_sock);

		clean_discon:
			winc_wifi_recv_cleanup(ww, ws);
			winc_wifi_sock_disconnect(ws);
			break;
		}
		return;
	}

	if (msg->s16BufferSize == 0) {
		/* I believe this is to ACK a close() */
		DBFPRINTF("zero buff\n");
		winc_wifi_recv_cleanup(ww, ws);
		return;
	}

	if (rcv_from && ws->ws_rx_length == 0 &&
	    msg->strRemoteAddr.sin_port != 0) {
		/* Copy peer details the first time through */
		ws->ws_dport = ntoh16(msg->strRemoteAddr.sin_port);
		ws->ws_daddr = ntoh32(msg->strRemoteAddr.sin_addr.s_addr);
	}

	DBFPRINTF("sock %u: len %" PRId16 ", remaining %" PRIu16 "\n",
	    (unsigned int)ws->ws_sock, msg->s16BufferSize,
	    msg->u16RemainingSize);

	sb = SIMPLEQ_LAST(&ws->ws_rx_buffs, winc_socket_buffer, sb_qent);
	assert(sb != NULL);
	assert((unsigned int)msg->s16BufferSize <= sizeof(sb->sb_buff));
	assert(msg->pu8Buffer == sb->sb_buff);

	sb->sb_len = msg->s16BufferSize;

	DBPRINTF("message(8): %02x %02x %02x %02x %02x %02x %02x %02x\n",
	    (unsigned int)sb->sb_buff[0], (unsigned int)sb->sb_buff[1],
	    (unsigned int)sb->sb_buff[2], (unsigned int)sb->sb_buff[3],
	    (unsigned int)sb->sb_buff[4], (unsigned int)sb->sb_buff[5],
	    (unsigned int)sb->sb_buff[6], (unsigned int)sb->sb_buff[7]);

	if (msg->u16RemainingSize != 0) {
		/*
		 * There's more to come, so we need another socket buffer
		 */
		if ((sb = winc_get_socket_buffer(ww)) == NULL) {
			DBFPRINTF("sock %u: out of socket buffers\n",
			    (unsigned int)ws->ws_sock);
			return;
		}

		SIMPLEQ_INSERT_TAIL(&ws->ws_rx_buffs, sb, sb_qent);

		ws->ws_rx_length += msg->s16BufferSize;
		msg->pu8Buffer = sb->sb_buff;
	} else {
		/*
		 * No more data to come. Invoke the callback
		 */
		ws->ws_rx_length += msg->s16BufferSize;
		ws->ws_rx_idx = 0;
		winc_wifi_sock_recv(ws);
	}
}

static void
winc_wifi_socket_handle_send(struct winc_socket *ws, int16_t sent_len)
{

	if (WINC_BAD_SOCKET(ws->ws_sock) ||
	    ws->ws_winc->ww_asf2sock[ws->ws_sock] != ws) {
		return;
	}

	if (sent_len < 0) {
		/* Some kind of error */
		switch (sent_len) {
		case SOCK_ERR_TIMEOUT:
			winc_wifi_sock_timeout(ws);
			break;
		default:
			DBFPRINTF("err %" PRId16 "\n", sent_len);
			/*FALLTHROUGH*/
		case SOCK_ERR_CONN_ABORTED:
			winc_wifi_sock_disconnect(ws);
			break;
		}
		return;
	}

	if (sent_len == 0) {
		/* I believe this is to ACK a close() */
		DBFPRINTF("zero buff\n");
		return;
	}

	if (sent_len > ws->ws_tx_pending) {
		/* Should never happen... */
		DBFPRINTF("%" PRId16 " > %" PRIu16 "\n", sent_len,
		    ws->ws_tx_pending);
		return;
	}

	if (ws->ws_tx_pending == 0) {
		/* Should never happen... */
		DBFPRINTF("idle\n");
		return;
	}

	ws->ws_tx_pending = 0;

	winc_wifi_sock_send_ok(ws);
}

static void
winc_wifi_socket_cb(SOCKET sock, uint8 msg, void *arg)
{
	struct winc_wifi *ww = winc_wifi_state;
	struct winc_socket *ws;

	assert(ww != NULL);

	if (WINC_BAD_SOCKET(sock))
		return;

	ws = ww->ww_asf2sock[sock];
	if (ws == NULL) {
		DBFPRINTF("bogus socket\n");
		return;
	}

	winc_wifi_led_net(WINC_LED_ON);

	switch (msg) {
	case SOCKET_MSG_BIND:
		DBFPRINTF("sock %u: BIND\n", (unsigned int)sock);
		winc_wifi_socket_handle_bind(ws, arg);
		break;
	case SOCKET_MSG_LISTEN:
		DBFPRINTF("sock %u: LISTEN\n", (unsigned int)sock);
		winc_wifi_socket_handle_listen(ws, arg);
		break;
	case SOCKET_MSG_DNS_RESOLVE:
		/* We don't support this */
		DBFPRINTF("sock %u: RESOLVE\n", (unsigned int)sock);
		break;
	case SOCKET_MSG_ACCEPT:
		DBFPRINTF("sock %u: ACCEPT\n", (unsigned int)sock);
		winc_wifi_socket_handle_accept(ws, arg);
		break;
	case SOCKET_MSG_CONNECT:
		DBFPRINTF("sock %u: CONNECT\n", (unsigned int)sock);
		winc_wifi_socket_handle_connect(ws, arg);
		break;
	case SOCKET_MSG_RECV:
		DBFPRINTF("sock %u: RECV\n", (unsigned int)sock);
		goto handle_recv;
	case SOCKET_MSG_RECVFROM:
		DBFPRINTF("sock %u: RECVFROM\n", (unsigned int)sock);
	handle_recv:
		winc_wifi_socket_handle_recv(ww, ws,
		    (msg == SOCKET_MSG_RECVFROM), arg);
		break;
	case SOCKET_MSG_SEND:
		DBFPRINTF("sock %u: SEND\n", (unsigned int)sock);
		goto handle_send;
	case SOCKET_MSG_SENDTO:
		DBFPRINTF("sock %u: SENDTO\n", (unsigned int)sock);
	handle_send:
		winc_wifi_socket_handle_send(ws, (int16_t)*((sint16 *)arg));
		break;
	default:
		DBFPRINTF("sock %u: unsupported %u\n", (unsigned int)sock,
		    (unsigned int)msg);
		break;
	}

	winc_wifi_led_net(WINC_LED_OFF);
}
#endif /* NETWORK_LWIP */

static uint8_t
winc_wifi_reset(struct winc_wifi *ww, bool do_provision)
{
	struct winc_socket_buffer *sb;
	tstrWifiInitParam param;

	DBFPRINTF("do_provision = %u\n", do_provision);

	SIMPLEQ_INIT(&ww->ww_free_socket_buffs);

	for (sb = ww->ww_socket_buffs; sb != NULL; sb = sb->sb_next) {
		sb->sb_len = 0;
		SIMPLEQ_INSERT_HEAD(&ww->ww_free_socket_buffs, sb, sb_qent);
	}

	if (do_provision)
		ww->ww_link_state = WINC_WIFI_STATE_INIT;
	else
		ww->ww_link_state = WINC_WIFI_STATE_PROVISIONED;

	(ww->ww_control)(ww->ww_cookie, WINC_WIFI_RESET, NULL);

	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = winc_wifi_link_cb;

#ifdef NETWORK_LWIP
//	if (do_provision == 0) {
		/* This implies we reboot following provisioning mode. */
		param.strEthInitParam.u8EthernetEnable = M2M_WIFI_MODE_ETHERNET;
		param.strEthInitParam.pfAppEthCb = winc_wifi_if_rx_cb;
//	}
#endif

	if (m2m_wifi_init(&param) != M2M_SUCCESS)
		return 0;

#if !defined(NETWORK_LWIP)
	socketInit();
	registerSocketCallback(winc_wifi_socket_cb, NULL);
#endif

	winc_wifi_led_init();

#ifndef CONFIG_USE_CONFIGDB
	if (ww->ww_gv->gv_name[0] != 0xff && ww->ww_gv->gv_name[0] != '\0') {
		m2m_wifi_set_device_name((uint8 *)ww->ww_gv->gv_name,
		    strlen(ww->ww_gv->gv_name));
	}
#else
	char name[M2M_DEVICE_NAME_MAX];
	if (cf_get_string(CF_KEY_STRING_WIFI_NAME, sizeof(name), name) != 0)
		m2m_wifi_set_device_name((uint8 *)name, strlen(name));
#endif

	return 1;
}

void
winc_wifi_interrupt(void *arg)
{
	struct winc_wifi *ww = arg;

	if (ww->ww_irq_handler == NULL) {
		uint8_t enable = 0;
		(ww->ww_control)(ww->ww_cookie, WINC_WIFI_IRQ_CTL, &enable);
	} else {
		(ww->ww_irq_handler)();
		if (ww->ww_wifi_event == 0) {
			ww->ww_wifi_event = 1;
			network_driver_hw_interrupt(ww->ww_network_state);
		}
	}
}

int8_t
winc_wifi_ioctl(void *arg, uint8_t op, void *oparg)
{
	struct winc_wifi *ww = arg;

	return network_ioctl(ww->ww_network_state, op, oparg);
}

void *
winc_wifi_attach(const struct winc_wifi_attach_args *waa)
{
	struct winc_wifi *ww;
	tenuM2mSecType enc;

	assert(winc_wifi_state == NULL);

	ww = zone_calloc(1, sizeof(*ww));
	if (ww == NULL)
		return NULL;

	winc_wifi_state = ww;

	ww->ww_cookie = waa->waa_cookie;
	ww->ww_read_write = waa->waa_read_write;
	ww->ww_control = waa->waa_control;
#ifndef CONFIG_USE_CONFIGDB
	ww->ww_gv = waa->waa_glob_vars;
#endif
	ww->ww_drv.nd_cookie = ww;
	ww->ww_drv.nd_worker = winc_wifi_process_events_native;
	ww->ww_drv.nd_link_check = winc_wifi_link_check;
	ww->ww_drv.nd_get_mac = winc_wifi_get_mac;
	ww->ww_drv.nd_set_ip_params = winc_wifi_set_ip_params;
	ww->ww_drv.nd_open = winc_wifi_open;
	ww->ww_drv.nd_close = winc_wifi_close;
	ww->ww_drv.nd_connect = winc_wifi_connect;
	ww->ww_drv.nd_listen = winc_wifi_listen;
	ww->ww_drv.nd_rx_avail = winc_wifi_rx_avail;
	ww->ww_drv.nd_read = winc_wifi_read;
	ww->ww_drv.nd_recv = winc_wifi_recv;
	ww->ww_drv.nd_tx_free = winc_wifi_tx_free;
	ww->ww_drv.nd_sock_status = winc_wifi_sock_status;
	ww->ww_drv.nd_write = winc_wifi_write;
	ww->ww_drv.nd_link_status = winc_wifi_link_status;
	ww->ww_drv.nd_ioctl = winc_wifi_locked_ioctl;

#ifdef WINC_OPT_OTA_UPDATE
	ww->ww_ota_state = WINC_OTA_STATUS_IDLE;
#endif

	ww->ww_socket_buffs = NULL;
	ww->ww_nsocket_buffs = 0;
	ww->ww_sockets = NULL;
	ww->ww_nsockets = 0;

	enc = winc_wifi_get_enc(ww);
	if (winc_wifi_reset(ww, enc == M2M_WIFI_SEC_INVALID) == 0) {
		winc_wifi_state = NULL;
		zone_free(ww->ww_socket_buffs);
		zone_free(ww);
		return NULL;
	}

	ww->ww_network_state = network_register(&ww->ww_drv);
	if (ww->ww_network_state == NULL) {
		winc_wifi_state = NULL;
		zone_free(ww->ww_socket_buffs);
		zone_free(ww);
		return NULL;
	}

#ifdef WINC_OPT_COMMAND
	SHELL_CMD_ADD(wifi);
#endif

	return ww;
}

#ifdef NETWORK_LWIP
static void
winc_wifi_new_rx_pbuf(struct winc_wifi_lwip *wwl)
{
	struct pbuf *p;

	p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
	if (p == NULL) {
		DBFPRINTF("failed to alloc pbuf\n");
		if (wwl->wwl_rx_pbuf_head != NULL) {
			pbuf_free(wwl->wwl_rx_pbuf_head);
			wwl->wwl_rx_pbuf_head = NULL;
		}
		wwl->wwl_rx_pbuf_cur = NULL;
		wwl->wwl_rx_payload = wwl->wwl_rx_buffer;
		m2m_wifi_set_receive_buffer(wwl->wwl_rx_buffer,
		    sizeof(wwl->wwl_rx_buffer));
		return;
	}

	assert(p->next == NULL);

	if (wwl->wwl_rx_pbuf_head == NULL) {
#if ETH_PAD_SIZE
		pbuf_remove_header(p, ETH_PAD_SIZE);
#endif
		wwl->wwl_rx_pbuf_head = p;
	}

	if (wwl->wwl_rx_pbuf_cur != NULL)
		wwl->wwl_rx_pbuf_cur->next = p;

	wwl->wwl_rx_pbuf_cur = p;
	m2m_wifi_set_receive_buffer(p->payload, p->len);
}

static void
winc_wifi_if_rx_cb(uint8 mtype, void *buff, void *ctrl)
{
	struct winc_wifi_lwip *wwl = (struct winc_wifi_lwip *)winc_wifi_state;
	tstrM2mIpCtrlBuf *cb = ctrl;
	struct pbuf *p;

	DBFPRINTF("mtype %u\n", (unsigned int)mtype);

	(void) buff;
	(void) mtype;

	winc_wifi_led_net(WINC_LED_ON);

	assert(mtype == M2M_WIFI_RESP_ETHERNET_RX_PACKET);

	/*
	 * WINC driver has copied cb->u16DataSize bytes into the
	 * payload of the current pbuf.
	 */
	if (wwl->wwl_rx_payload == wwl->wwl_rx_buffer) {
		assert(buff == wwl->wwl_rx_payload);
		if (cb->u16RemainigDataSize != 0) {
			/*
			 * We're dropping this packet due to an earlier error
			 * while allocating another pbuf.
			 */
			DBFPRINTF("continue dropping packet\n");
			goto done;
		}
		wwl->wwl_rx_payload = NULL;
		DBFPRINTF("Finished dropping packet\n");
	}

	if ((p = wwl->wwl_rx_pbuf_cur) != NULL) {
		DBFPRINTF("Rx packet. Len %" PRIu16 "\n", cb->u16DataSize);
		assert(buff == p->payload);
		assert(wwl->wwl_rx_pbuf_head != NULL);
		p->len = cb->u16DataSize;
		wwl->wwl_rx_pbuf_head->tot_len += cb->u16DataSize;

		if (cb->u16RemainigDataSize == 0) {
			/* Packet is complete. */
			p = wwl->wwl_rx_pbuf_head;
			wwl->wwl_rx_pbuf_head = NULL;
			wwl->wwl_rx_pbuf_cur = NULL;
#if ETH_PAD_SIZE
			pbuf_add_header(p, ETH_PAD_SIZE);
#endif
			if (wwl->wwl_netif.input(p, &wwl->wwl_netif) != ERR_OK)
				pbuf_free(p);
		}
	}

	/*
	 * Allocate another Rx buffer for the WINC driver. On failure,
	 * winc_wifi_new_rx_pbuf() will configure a dummy buffer to
	 * absorb the remainder of the packet.
	 */
	winc_wifi_new_rx_pbuf(wwl);

 done:
	winc_wifi_led_net(WINC_LED_OFF);
}

static int8_t
winc_wifi_if_locked_ioctl(struct netif *netif, uint8_t cmd, void *arg)
{
	struct winc_wifi_lwip *wwl = netif->state;

	return winc_wifi_locked_ioctl(&wwl->wwl_ww, cmd, arg);
}

static void
winc_wifi_if_get_mac(struct netif *netif, uint8_t *mac)
{

	winc_wifi_get_mac(netif, mac);
}

static int8_t
winc_wifi_if_start_stop(struct netif *netif, int8_t do_start)
{
	struct winc_wifi_lwip *wwl = netif->state;

	if (do_start) {
		STAILQ_INIT(&wwl->wwl_tx_queue);
		STAILQ_INIT(&wwl->wwl_free_queue);
		if (wwl->wwl_rx_pbuf_cur == NULL)
			winc_wifi_new_rx_pbuf(wwl);
		netif_set_up(netif);
	} else {
		struct winc_pbuf *wp;

		m2m_wifi_set_receive_buffer(&wwl->wwl_rx_buffer,
		    sizeof(wwl->wwl_rx_buffer));
		if (wwl->wwl_rx_pbuf_head != NULL) {
			pbuf_free(wwl->wwl_rx_pbuf_head);
			wwl->wwl_rx_pbuf_head = NULL;
		}
		wwl->wwl_rx_pbuf_cur = NULL;

		/* Discard any queued Tx pbufs */
		wwl->wwl_tx_pbuf = NULL;
		while ((wp = STAILQ_FIRST(&wwl->wwl_tx_queue)) != NULL) {
			STAILQ_REMOVE_HEAD(&wwl->wwl_tx_queue, wp_qent);
			assert(wp->wp_pbuf != NULL);
			pbuf_free(wp->wp_pbuf);
			wp->wp_pbuf = NULL;
			STAILQ_INSERT_TAIL(&wwl->wwl_free_queue, wp, wp_qent);
		}

		/* Now free the free list */
		while ((wp = STAILQ_FIRST(&wwl->wwl_free_queue)) != NULL) {
			STAILQ_REMOVE_HEAD(&wwl->wwl_free_queue, wp_qent);
			zone_free(wp);
		}

		netif_set_down(netif);
	}

	return 0;
}

static void
winc_wifi_if_link_worker(struct netif *netif)
{

	winc_wifi_link_check(netif->state);
}

static void
winc_wifi_if_link_status(struct netif *netif, network_link_status_callback_t cb,
    void *cbarg)
{
	struct winc_wifi_lwip *wwl = netif->state;

	wwl->wwl_link_cb = cb;
	wwl->wwl_link_cb_arg = cbarg;
}

static void
winc_wifi_if_link_callback(const network_link_status_t *ls, void *arg)
{
	struct winc_wifi_lwip *wwl = arg;

	if (wwl->wwl_link_cb != NULL)
		wwl->wwl_link_cb(ls, wwl->wwl_link_cb_arg);

	if (ls->ls_state == NETWORK_LINK_DOWN)
		netif_set_link_down(&wwl->wwl_netif);
	else
	if (ls->ls_state == NETWORK_LINK_UP)
		netif_set_link_up(&wwl->wwl_netif);
}

static int
winc_wifi_if_tx(struct winc_wifi_lwip *wwl, struct pbuf *p)
{

	/*
	 * Brain-dead WINC driver can't do scatter/gather. What a ball-ache.
	 */
	if (wwl->wwl_tx_pbuf == NULL) {
		wwl->wwl_tx_pbuf = p;
#if ETH_PAD_SIZE
		pbuf_remove_header(p, ETH_PAD_SIZE);
#endif
		wwl->wwl_tx_payload_len = p->tot_len;
		if (p->tot_len == p->len) {
			wwl->wwl_tx_payload = p->payload;
		} else {
			struct pbuf *q;
			uint8_t *payload = wwl->wwl_tx_buffer;
			uint16_t l = p->tot_len;

			assert(l <= sizeof(wwl->wwl_tx_buffer));

			for (q = p; q != NULL; q = q->next) {
				assert(q->len <= l);
				memcpy(payload, q->payload, q->len);
				payload += q->len;
				l -= q->len;
			}

			assert(l == 0);
			wwl->wwl_tx_payload = wwl->wwl_tx_buffer;
		}

#if ETH_PAD_SIZE
		pbuf_add_header(p, ETH_PAD_SIZE);
#endif
	} else {
		assert(wwl->wwl_tx_pbuf == p);
	}


	if (m2m_wifi_send_ethernet_pkt(wwl->wwl_tx_payload,
	    wwl->wwl_tx_payload_len) != M2M_SUCCESS) {
		return 0;
	}

	wwl->wwl_tx_pbuf = NULL;
	pbuf_free(p);

	return 1;
}

static uint32_t
winc_wifi_if_worker(struct netif *netif)
{
	struct winc_wifi_lwip *wwl = netif->state;
	struct winc_wifi *ww = &wwl->wwl_ww;
	struct winc_pbuf *wp;
	uint32_t rv;

	rv = winc_wifi_process_events(&wwl->wwl_ww);

	if (ww->ww_link_state != WINC_WIFI_STATE_SHOWTIME)
		return rv;

	while ((wp = STAILQ_FIRST(&wwl->wwl_tx_queue)) != NULL) {
		if (winc_wifi_if_tx(wwl, wp->wp_pbuf) == 0) {
			network_driver_soft_timeout(ww->ww_network_state, 1);
			break;
		} else {
			STAILQ_REMOVE_HEAD(&wwl->wwl_tx_queue, wp_qent);
			wp->wp_pbuf = NULL;
			STAILQ_INSERT_TAIL(&wwl->wwl_free_queue, wp, wp_qent);
		}
	}

	return rv;
}

static err_t
winc_wifi_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct winc_wifi_lwip *wwl = netif->state;
	struct winc_pbuf *wp;

	pbuf_ref(p);

	if (STAILQ_EMPTY(&wwl->wwl_tx_queue) && winc_wifi_if_tx(wwl, p) == 1)
		return ERR_OK;

	if ((wp = STAILQ_FIRST(&wwl->wwl_free_queue)) != NULL) {
		STAILQ_REMOVE_HEAD(&wwl->wwl_free_queue, wp_qent);
	} else {
		wp = zone_malloc(sizeof(*wp));
		if (wp == NULL) {
			pbuf_free(p);
			return ERR_MEM;
		}
	}

	wp->wp_pbuf = p;
	STAILQ_INSERT_TAIL(&wwl->wwl_tx_queue, wp, wp_qent);
	network_driver_soft_timeout(wwl->wwl_ww.ww_network_state, 0);

	return ERR_OK;
}

static err_t
winc_wifi_netif_init(struct netif *netif)
{
	uint8_t mac[ETH_HWADDR_LEN];

#if LWIP_NETIF_HOSTNAME
	cf_len_t l;

	netif_set_hostname(netif, NULL);
	l = cf_get_string(CF_KEY_STRING_WIFI_NAME, M2M_DEVICE_NAME_MAX, NULL);
	if (l > 1) {
		char *name = zone_malloc((size_t)l);
		if (name != NULL) {
			cf_get_string(CF_KEY_STRING_WIFI_NAME,
			    M2M_DEVICE_NAME_MAX, name);
			netif_set_hostname(netif, name);
		}
	}
#endif

	netif->name[0] = 'm';
	netif->name[1] = 'w';
	netif->output = etharp_output;
	netif->linkoutput = winc_wifi_low_level_output;
	winc_wifi_get_mac(netif, mac);
	memcpy(netif->hwaddr, mac, ETH_HWADDR_LEN);
        netif->hwaddr_len = ETH_HWADDR_LEN;
        netif->mtu = 1500;
        netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
            NETIF_FLAG_ETHERNET;

	return ERR_OK;
}

void *
winc_wifi_lwip_attach(const struct winc_wifi_attach_args *waa)
{
	struct network_lwip_driver *nld;
	struct winc_wifi_lwip *wwl;
	struct winc_wifi *ww;
	tenuM2mSecType enc;

	assert(winc_wifi_state == NULL);

	wwl = zone_calloc(1, sizeof(*wwl));
	if (wwl == NULL)
		return NULL;

	nld = &wwl->wwl_driver;
	nld->nld_netif = &wwl->wwl_netif;
	nld->nld_handle_interrupt = winc_wifi_if_worker;
	nld->nld_link_check = winc_wifi_if_link_worker;
	nld->nld_link_status = winc_wifi_if_link_status;
	nld->nld_get_mac = winc_wifi_if_get_mac;
	nld->nld_ioctl = winc_wifi_if_locked_ioctl;

	ww = winc_wifi_state = &wwl->wwl_ww;
	ww->ww_link_cb = winc_wifi_if_link_callback;
	ww->ww_link_cb_arg = wwl;
	ww->ww_cookie = waa->waa_cookie;
	ww->ww_read_write = waa->waa_read_write;
	ww->ww_control = waa->waa_control;
#ifndef CONFIG_USE_CONFIGDB
	ww->ww_gv = waa->waa_glob_vars;
#endif
#ifdef WINC_OPT_OTA_UPDATE
	ww->ww_ota_state = WINC_OTA_STATUS_IDLE;
#endif

	ww->ww_socket_buffs = NULL;
	ww->ww_nsocket_buffs = 0;
	ww->ww_sockets = NULL;
	ww->ww_nsockets = 0;

	enc = winc_wifi_get_enc(ww);
	if (winc_wifi_reset(ww, enc == M2M_WIFI_SEC_INVALID) == 0) {
		winc_wifi_state = NULL;
		zone_free(ww);
		return NULL;
	}

	netif_add(&wwl->wwl_netif, NULL, NULL, NULL, wwl,
	    winc_wifi_netif_init, netif_input);
	netif_set_default(&wwl->wwl_netif);

	if (winc_wifi_if_start_stop(&wwl->wwl_netif, 1) < 0 ||
	    (ww->ww_network_state = network_lwip_attach(nld)) == NULL) {
		winc_wifi_state = NULL;
		zone_free(ww);
		return NULL;
	}

#ifdef WINC_OPT_COMMAND
	SHELL_CMD_ADD(wifi);
#endif

	return ww;
}
#endif /* NETWORK_LWIP */

#ifdef WINC_OPT_COMMAND
#ifndef PRODUCT_NAME
#define	PRODUCT_NAME	"the device"
#endif
static const char * const wifi_cmd_usage =
"Usage:\n"
"wifi ssid <wifi network name>\n"
"\tSets the name of the WiFi network to which " PRODUCT_NAME " should "
	"connect.\n"
"\tEnclose the network name in double quotes, such as \"net name\" if\n"
"\tit contains spaces.\n"
"wifi key <encryption type> <wifi network passphrase>\n"
"\tConfigure the encryption type and passphrase. Encryption type is\n"
"\tone of \"wep\" or \"wpa\". Note that \"wep\" is insecure and\n"
"\tsupport is likely to be dropped in future firmware updates.\n"
"\tThe wifi network passphrase is the password used to secure your\n"
"\tWiFi network. Again, enclose the password in double-quotes if\n"
"\tthe password contains spaces.\n"
"wifi name <client name>\n"
"\tSpecifies a human-friendly name for " PRODUCT_NAME ". Most good DHCP "
	"servers\n"
"\twill log the client name against the assigned IP address and may\n"
"\tpopulate the local DNS with the name for ease of access. This is\n"
"\tblank by default.\n"
;

static void
winc_wifi_cmd(FILE *fp, uint8_t argc, const char * const *argv)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);
	(void) ww;

	if (argc >= 2) {
#ifdef DEBUG_ENABLED
		if (strcasecmp(argv[0], "debug") == 0) {
			if (strcasecmp(argv[1], "on") == 0)
				winc_wifi_debug = 1;
			else
			if (strcasecmp(argv[1], "off") == 0)
				winc_wifi_debug = 0;
		} else
#endif
		if (strcasecmp(argv[0], "name") == 0) {
#ifndef CONFIG_USE_CONFIGDB
			strlcpy(ww->ww_gv->gv_name, argv[1],
			    sizeof(ww->ww_gv->gv_name));
#else
			cf_set_string(CF_KEY_STRING_WIFI_NAME, argv[1]);
#endif
		} else
		if (strcasecmp(argv[0], "ssid") == 0) {
#ifndef CONFIG_USE_CONFIGDB
			strlcpy(ww->ww_gv->gv_ssid, argv[1],
			    sizeof(ww->ww_gv->gv_ssid));
#else
			cf_set_string(CF_KEY_STRING_WIFI_SSID, argv[1]);
#endif
		} else
		if (strcasecmp(argv[0], "key") == 0) {
			uint8_t st;

			if (strcasecmp(argv[1], "none") == 0) {
				if (argc != 2) {
					fprintf(fp, "Ignoring supplied"
					    " passphrasen\n");
				}
				st = WINC_WIFI_SEC_OPEN;
			} else
			if (strcasecmp(argv[1], "wep") == 0) {
				st = WINC_WIFI_SEC_WEP;
			} else
			if (strcasecmp(argv[1], "wpa") == 0) {
				st = WINC_WIFI_SEC_WPA_PSK;
			} else {
				fprintf(fp, "Unrecognised key type: "
				    "%s\n", argv[1]);
				return;
			}

			if (st != WINC_WIFI_SEC_OPEN) {
				if (argc != 3) {
					fprintf(fp, "You must specify"
					    " a passphrasen\n");
					return;
				}
#ifndef CONFIG_USE_CONFIGDB
				strlcpy(ww->ww_gv->gv_key, argv[2],
				    sizeof(ww->ww_gv->gv_key));
#else
				cf_set_string(CF_KEY_STRING_WIFI_KEY, argv[2]);
#endif
			} else {
#ifndef CONFIG_USE_CONFIGDB
				ww->ww_gv->gv_key[0] = '\0';
#else
				cf_set_string(CF_KEY_STRING_WIFI_KEY, "");
#endif
			}
#ifndef CONFIG_USE_CONFIGDB
			ww->ww_gv->gv_enc_type = st;
#else
			cf_set_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, st);
#endif
		} else {
			goto usage;
		}

#ifndef CONFIG_USE_CONFIGDB
		(ww->ww_control)(ww->ww_cookie, WINC_WIFI_GLOB_SAVE, NULL);
#else
		configdb_persist();
#endif
	} else
	if (argc != 0) {
 usage:
		fprintf(fp, "%s", wifi_cmd_usage);
		return;
	}

	tstrM2mRev strRev;
	sint8 x;
	if ((x = m2m_wifi_get_firmware_version(&strRev)) != M2M_SUCCESS) {
		fprintf(fp, "m2m_wifi_get_firmware_version: returned %d\n",
		    (int)x);
		return;
	}

	fprintf(fp, "WINC1500 ChipId 0x%" PRIx32 ", "
	    "Firmware %u.%u.%u, Driver %u.%u.%u\n"
	    "Firmware Date %s, %s\n", strRev.u32Chipid,
	    (unsigned int)strRev.u8FirmwareMajor,
	    (unsigned int)strRev.u8FirmwareMinor,
	    (unsigned int)strRev.u8FirmwarePatch,
	    (unsigned int)strRev.u8DriverMajor,
	    (unsigned int)strRev.u8DriverMinor,
	    (unsigned int)strRev.u8DriverPatch,
	    strRev.BuildDate, strRev.BuildTime);

	char *name = NULL, *ssid = NULL;
	uint8_t enc;

#ifndef CONFIG_USE_CONFIGDB
	if (ww->ww_gv->gv_name[0] != 0xff && ww->ww_gv->gv_name[0] != '\0')
		name = ww->ww_gv->gv_name;
	if (ww->ww_gv->gv_ssid[0] != 0xff && ww->ww_gv->gv_ssid[0] != '\0')
		ssid = ww->ww_gv->gv_ssid;
	enc = ww->ww_gv->gv_enc_type;
#else
	cf_len_t l;

	l = cf_get_string(CF_KEY_STRING_WIFI_NAME, M2M_DEVICE_NAME_MAX, NULL);
	if (l != 0) {
		name = zone_malloc((size_t)l);
		if (name != NULL) {
			cf_get_string(CF_KEY_STRING_WIFI_NAME,
			    M2M_DEVICE_NAME_MAX, name);
		}
	}
	l = cf_get_string(CF_KEY_STRING_WIFI_SSID, 33, NULL);
	if (l != 0) {
		ssid = zone_malloc((size_t)l);
		if (ssid != NULL)
			cf_get_string(CF_KEY_STRING_WIFI_SSID, 33, ssid);
	}
	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &enc) == 0)
		enc = WINC_WIFI_SEC_INVALID;
#endif

	if (name != NULL) {
		fprintf(fp, "WiFi Device Name: %s\n", name);
#ifdef CONFIG_USE_CONFIGDB
		zone_free(name);
#endif
	}

	if (ssid != NULL) {
		fprintf(fp, "WiFi SSID: %s\n", ssid);
#ifdef CONFIG_USE_CONFIGDB
		zone_free(ssid);
#endif
	}

	const char *en;
	switch (enc) {
	case WINC_WIFI_SEC_OPEN:
		en = "Open";
		break;
	case WINC_WIFI_SEC_WPA_PSK:
		en = "WPA/WPA2";
		break;
	case WINC_WIFI_SEC_WEP:
		en = "WEP";
		break;
	case WINC_WIFI_SEC_802_1X:
		en = "802.1X";
		break;
	default:
		en = "Unknown";
		break;
	}
	fprintf(fp, "WiFi encryption: %s\n", en);

#if (RELEASE_BUILD == 0)
	fprintf(fp, "Max sockets: %u\n", (unsigned int)ww->ww_nsockets);
	fprintf(fp, "Socket buffs: %u\n", (unsigned int)ww->ww_nsocket_buffs);
#endif /* (RELEASE_BUILD == 0) */
}
#endif /* WINC_OPT_COMMAND */

/*
 * These are the BSP functions required by Atmel's driver.
 */
void
nm_bsp_sleep(uint32 u32TimeMsec)
{
	struct winc_wifi *ww = winc_wifi_state;
	uint32_t msecs = (uint32_t) u32TimeMsec;

	assert(ww != NULL);

	(ww->ww_control)(ww->ww_cookie, WINC_WIFI_SUSPEND, &msecs);
}

void
nm_bsp_register_isr(tpfNmBspIsr handler)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);

	ww->ww_irq_handler = handler;
}

void
nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	struct winc_wifi *ww = winc_wifi_state;

	assert(ww != NULL);

	(ww->ww_control)(ww->ww_cookie, WINC_WIFI_IRQ_CTL, &u8Enable);
}

sint8
nm_bus_ioctl(uint8 cmd, void *arg)
{
	struct winc_wifi *ww = winc_wifi_state;
	tstrNmSpiRw *srw;

	assert(ww != NULL);

	if (cmd != NM_BUS_IOCTL_RW)
		return -1;

	srw = (tstrNmSpiRw *)arg;

	(ww->ww_read_write)(ww->ww_cookie, srw->pu8InBuf, srw->pu8OutBuf,
	    srw->u16Sz);

	return 0;
}

sint8
nm_bus_init(void *pvinit)
{

	(void) pvinit;

	nm_bsp_sleep(1);
	return M2M_SUCCESS;
}

sint8
nm_bus_deinit(void)
{

	return M2M_SUCCESS;
}

sint8 spi_flash_enable(uint8 enable);
sint8
spi_flash_enable(uint8 enable)
{

	(void) enable;

	return M2M_SUCCESS;
}
