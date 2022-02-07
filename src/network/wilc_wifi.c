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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "common/include/nm_common.h"
#include "driver/include/m2m_periph.h"
#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"

#include "rtos.h"
#include "linked-lists.h"
#include "configdb.h"
#include "timer.h"
#include "zone_alloc.h"

#include "network.h"
#include "network_httpd.h"
#include "wilc_wifi.h"
#include "wifi_provision.h"

#ifdef WILC_OPT_COMMAND
#include "shell.h"
#endif

#include "network_lwip.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"

#include "conf_tinyservices.h"
#include "tinyservices.h"
#include "dns_hijack.h"
#include "fix16.h"


//#define DEBUG_FLAG_INIT	1
//#define DEBUG_FLAG	wilc_wifi_debug
#include "debug.h"

#define WILC_RECORD_STATS	1

_Static_assert(PBUF_LINK_ENCAPSULATION_HLEN == M2M_ETHERNET_HDR_OFFSET,
    "PBUF_LINK_ENCAPSULATION_HLEN != M2M_ETHERNET_HDR_OFFSET");

#define	WILC_WIFI_SCAN_LIFETIME	(30u * 1000u)

enum wilc_wifi_link_state {
	WILC_WIFI_STATE_INIT = 0,
	WILC_WIFI_STATE_NEW_PROVISION,
	WILC_WIFI_STATE_PROVISIONING,
	WILC_WIFI_STATE_PROVISION_AP_OFF,
	WILC_WIFI_STATE_PROVISIONED,
	WILC_WIFI_STATE_NEW_NETWORK,
	WILC_WIFI_STATE_CONNECTING,
	WILC_WIFI_STATE_GET_RSSI,
	WILC_WIFI_STATE_WAIT_RSSI,
	WILC_WIFI_STATE_START_STA,
	WILC_WIFI_STATE_ACQUIRE_ADDRESS,
	WILC_WIFI_STATE_CONFIGURED,
	WILC_WIFI_STATE_SHOWTIME,
};

enum wilc_wifi_if_id {
	WILC_IF_STA,
	WILC_IF_AP
};

struct wilc_pbuf {
	struct pbuf *wp_pbuf;
	enum wilc_wifi_if_id wp_if;
	TAILQ_ENTRY(wilc_pbuf) wp_qent;
};
TAILQ_HEAD(wilc_pbuf_qhead, wilc_pbuf);

struct wilc_wifi {
	struct network_lwip_driver ww_driver;
	struct network_state *ww_network_state;
	void *ww_cookie;
	void (*ww_read_write)(void *, const void *, void *, uint16_t);
	void (*ww_control)(void *, uint8_t, void *);
	tpfNmBspIsr ww_irq_handler;
	volatile uint8_t ww_wifi_event;
	enum wilc_wifi_link_state ww_link_state;
	enum wilc_wifi_link_state ww_prov_old_state;
	struct wilc_prov_arg ww_prov_arg;
	fix16_t ww_rssi;
	network_link_status_t ww_link_status;
	network_link_status_callback_t ww_link_cb;
	void *ww_link_cb_arg;

#define	WILC_LINK_CHECK_PERIOD	(600u * TIMER_HZ)
	bool ww_wilc_link_rssi_dead;
	timer_timeout_t ww_link_timer;

	struct pbuf *ww_rx_pbuf;

	struct wilc_pbuf_qhead ww_tx_free_queue;
	struct wilc_pbuf_qhead ww_tx_queue;
	int ww_tx_pending;
	bool ww_tx_bogons;
	bool ww_tx_just_queue;
#define	WILC_MAX_TX_QUEUE_SIZE	48
	struct wilc_pbuf ww_tx_pbufs[WILC_MAX_TX_QUEUE_SIZE];

	struct netif ww_netif_sta;
	bool ww_sta_if_enabled;

	struct netif ww_netif_ap;

	timer_timeout_t ww_prov_timeout;
	uint8_t ww_prov_blink;
	void *ww_prov_handle;
	int ww_prov_ssid_count;

	rtos_timer_t ww_new_network_timer;

#ifdef WIFI_PROVISION_SUPPORT_SCAN
	uint8_t ww_scan_count;
	uint8_t ww_scan_channel;
	uint8_t ww_scan_index;
	uint8_t ww_channel;
	uint16_t ww_chan_usage[14];
#endif
	bool ww_scan_done;
	bool ww_ap_enabled;
	bool ww_ap_if_enabled;

	timer_timeout_t ww_connect_timeout;

#ifdef WILC_RECORD_STATS
	unsigned int ww_stat_tx_busy;
	unsigned int ww_stat_tx_queued;
	unsigned int ww_stat_tx_sent;
	unsigned int ww_stat_tx_deferred;
	unsigned int ww_stat_tx_dropped;
	unsigned int ww_stat_tx_contig;
	unsigned int ww_stat_tx_frag;

	unsigned int ww_stat_rx_packets;
	unsigned int ww_stat_rx_dropped;
	unsigned int ww_stat_rx_no_pbufs;

	unsigned int ww_stat_rssi_busy;
	unsigned int ww_stat_rssi_reqs;
	unsigned int ww_stat_rssi_stats;
	unsigned int ww_stat_rssi_early;
#endif

	tstrM2mRev ww_fw_info;
	uint8_t ww_rx_buffer[1536];
	uint8_t ww_tx_buffer[1536 + M2M_ETHERNET_HDR_OFFSET];
};
static struct wilc_wifi *wilc_wifi_state;

#define	WILC_RSSI(ww)		fix16_div((ww)->ww_rssi, F16(8.0))
#define	WILC_RSSI_INIT(ww,rssi)	fix16_mul((rssi), F16(8.0))

#ifdef WILC_RECORD_STATS
#define	WILC_STATS(n)	do { (n)++; } while (0)
#else
#define	WILC_STATS(n)	do { /* nothing */ } while (0)
#endif

static void wilc_wifi_if_eth_cb(uint8 mtype, void *buff, void *ctrl);
static void wilc_wifi_if_start_stop(struct netif *netif, bool do_start);
static bool wilc_wifi_if_tx(struct wilc_wifi *, struct pbuf *,
		enum wilc_wifi_if_id);

#define	WILC_RX_PBUF_SZ	(PBUF_POOL_BUFSIZE - M2M_ETHERNET_HDR_OFFSET)

/*
 * Atmel's driver needs this to determine the maximum SPI read/write
 * length we can handle in one chunk. Actual limit is 65525, but
 * stick to 2K for now.
 */
tstrNmBusCapabilities egstrNmBusCapabilities = { 2048 };

#ifdef WILC_OPT_COMMAND
SHELL_CMD_DECL(wifi, wilc_wifi_cmd, "Display WiFi Info");
#endif

static __always_inline void
wilc_wifi_led_err(uint8_t state)
{

	wilc_wifi_led_control(WILC_LED_ERR, state);
}

static __always_inline void
wilc_wifi_led_wifi(uint8_t state)
{

	wilc_wifi_led_control(WILC_LED_WIFI, state);
}

static void
wilc_wifi_led_net(int8_t state)
{
	static uint8_t net_cnt;

	if (state < 0) {
		net_cnt = 1;
		state = WILC_LED_OFF;
	}

	if (state == WILC_LED_ON) {
		if (net_cnt++ == 0)
			wilc_wifi_led_control(WILC_LED_NETWORK, WILC_LED_ON);
	} else {
		if (net_cnt == 1) {
			wilc_wifi_led_control(WILC_LED_NETWORK, WILC_LED_OFF);
			net_cnt = 0;
		} else
		if (net_cnt > 1)
			net_cnt--;
	}
}

static tenuM2mSecType
wilc_wifi_get_enc(struct wilc_wifi *ww)
{
	uint8_t enc;

	(void) ww;
	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &enc) == 0)
		return M2M_WIFI_SEC_INVALID;

	switch ((wifi_provision_enc_t)enc) {
	default:
	case WIFI_PROVISION_ENC_OPEN:
		return M2M_WIFI_SEC_OPEN;
	case WIFI_PROVISION_ENC_WPA:
		return M2M_WIFI_SEC_WPA_PSK;
	case WIFI_PROVISION_ENC_WEP:
		return M2M_WIFI_SEC_WEP;
	case WIFI_PROVISION_ENC_802_1X:
		return M2M_WIFI_SEC_802_1X;
	}
}

static sint8
wilc_m2m_wifi_connect(struct wilc_wifi *ww)
{
	tuniM2MWifiAuth wa;

	memset(&wa, 0, sizeof(wa));
	strlcpy((char *)wa.au8PSK, ww->ww_link_status.ls_key,
	    sizeof(wa.au8PSK));

	return m2m_wifi_connect(ww->ww_link_status.ls_bssid,
	    strlen(ww->ww_link_status.ls_bssid), wilc_wifi_get_enc(ww),
	    &wa, M2M_WIFI_CH_ALL);
}

static void
wilc_wifi_invoke_link_callback(struct wilc_wifi *ww)
{

	if (ww->ww_link_cb != NULL)
		(ww->ww_link_cb)(&ww->ww_link_status, ww->ww_link_cb_arg);
}

static tenuM2mScanCh
wilc_wifi_pick_ap_chan(struct wilc_wifi *ww)
{
#ifdef WIFI_PROVISION_SUPPORT_SCAN
	uint16_t cl[3];

	/*
	 * We pick channel 1, 6, or 11 based on an approximation to
	 * the congestion level found during the scan.
	 */

	/* Congestion level around channel 1 */
	cl[0] = ww->ww_chan_usage[M2M_WIFI_CH_1] * 3;
	cl[0] += ww->ww_chan_usage[M2M_WIFI_CH_2] * 2;
	cl[0] += ww->ww_chan_usage[M2M_WIFI_CH_3];

	/* Congestion level around channel 6 */
	cl[1] = ww->ww_chan_usage[M2M_WIFI_CH_4];
	cl[1] += ww->ww_chan_usage[M2M_WIFI_CH_5] * 2;
	cl[1] += ww->ww_chan_usage[M2M_WIFI_CH_6] * 3;
	cl[1] += ww->ww_chan_usage[M2M_WIFI_CH_7] * 2;
	cl[1] += ww->ww_chan_usage[M2M_WIFI_CH_8];

	/* Congestion level around channel 11 */
	cl[2] = ww->ww_chan_usage[M2M_WIFI_CH_9];
	cl[2] += ww->ww_chan_usage[M2M_WIFI_CH_10] * 2;
	cl[2] += ww->ww_chan_usage[M2M_WIFI_CH_11] * 3;

	if (cl[0] < cl[1] && cl[0] < cl[2])
		return M2M_WIFI_CH_1;

	if (cl[1] < cl[0] && cl[1] < cl[2])
		return M2M_WIFI_CH_6;
#else
	(void) ww;
#endif

	return M2M_WIFI_CH_11;
}

static void
wilc_wifi_start_ap(struct wilc_wifi *ww)
{
	tstrM2MAPConfig apcfg;

	/*
	 * netif_sta prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 * netif_ap prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 *
	 * Bring up AP.
	 */

	/* Build configuration for AP mode. */
	memset(&apcfg, 0, sizeof(apcfg));

	/* Configure the access point's SSID. */
	if (cf_get_string(CF_KEY_STRING_WIFI_PROV_SSID,
	    sizeof(apcfg.au8SSID), (char *)apcfg.au8SSID) == 0) {
		strlcpy((char *)apcfg.au8SSID, "Maverick",
		    sizeof(apcfg.au8SSID));
	}

	/* Configure the WiFi channel number. */
	if (cf_get_uint8(CF_KEY_UINT8_WIFI_PROV_CHANNEL,
	    &apcfg.u8ListenChannel) == 0) {
		apcfg.u8ListenChannel = wilc_wifi_pick_ap_chan(ww);
	}

	/* Support a single STA. */
	apcfg.u8MaxSupportedSta = 1;

	/* Configure passphrase. */
	if (cf_get_string(CF_KEY_STRING_WIFI_PROV_PASS,
	    sizeof(apcfg.uniAuth.au8PSK), (char *)apcfg.uniAuth.au8PSK) != 0) {
		/* WPA passphrase. */
		apcfg.u8SecType = M2M_WIFI_SEC_WPA_PSK;
		DBFPRINTF("PSK\n");
	} else {
		/* Open access point. */
		apcfg.u8SecType = M2M_WIFI_SEC_OPEN;
		DBFPRINTF("Open AP\n");
	}

	/* Put the WILC1000 into AP mode. */
	if (m2m_wifi_enable_ap(&apcfg) != M2M_SUCCESS) {
		DBFPRINTF("failed to start AP\n");
		return;
	}

	ww->ww_ap_enabled = true;
	ww->ww_ap_if_enabled = false;
}

static void
wilc_wifi_start_provision(struct wilc_wifi *ww)
{

	DBFPRINTF("start provision\n");
	ww->ww_link_state = WILC_WIFI_STATE_PROVISIONING;
	ww->ww_link_status.ls_state = NETWORK_LINK_PROVISIONING;
	ww->ww_link_status.ls_rssi = 0;
	ww->ww_rssi = F16(0.0);
	wilc_wifi_invoke_link_callback(ww);

	(ww->ww_control)(ww->ww_cookie, WILC_WIFI_PROVISIONING, NULL);

	ww->ww_prov_handle = wifi_provision_start();
	assert(ww->ww_prov_handle != NULL);
	ww->ww_prov_ssid_count = 0;

#if 0
	/* This doesn't work; subsequent m2m_wifi_request_scan() calls fail. */
	if (m2m_wifi_set_scan_options(2, 20) != M2M_SUCCESS)
		DBFPRINTF("Failed to set scan options.\n");
#endif

	ww->ww_ap_enabled = false;
	ww->ww_ap_if_enabled = false;

#ifdef WIFI_PROVISION_SUPPORT_SCAN
	ww->ww_scan_index = 0;
	ww->ww_scan_done = false;
	ww->ww_scan_count = 3;
	ww->ww_scan_channel = M2M_WIFI_CH_1;
	memset(ww->ww_chan_usage, 0, sizeof(ww->ww_chan_usage));

	if (m2m_wifi_request_scan(ww->ww_scan_channel) != M2M_SUCCESS)
		DBFPRINTF("Failed to start scan.\n");
#else
	ww->ww_scan_done = true;
#endif

	timer_timeout_start(&ww->ww_prov_timeout, 400);
}

static void
wilc_wifi_new_provision(struct wilc_wifi *ww)
{

	/*
	 * netif_sta prerequisites:
	 *  - Link up.
	 * netif_ap prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 */

	/* De-configure the STA network interface. */
	ww->ww_link_status.ls_state = NETWORK_LINK_DOWN;
	ww->ww_link_status.ls_rssi = 0;
	ww->ww_rssi = F16(0.0);
	wilc_wifi_invoke_link_callback(ww);
	wilc_wifi_if_start_stop(&ww->ww_netif_sta, false);
	ww->ww_sta_if_enabled = false;

	/* Disconnect the STA. */
	m2m_wifi_disconnect();

	/* Start provision mode. */
	wilc_wifi_start_provision(ww);
}

static void
wilc_wifi_provisioning(struct wilc_wifi *ww)
{
	const char *ssid, *pass;
	wifi_provision_enc_t enc;

	/*
	 * netif_sta prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 * netif_ap prerequisites:
	 *  - Link up.
	 *  - Interface up.
	 *  - Static IP assigned.
	 */

	if (wifi_provision_is_done(ww->ww_prov_handle, &ssid, &pass, &enc)) {
		DBFPRINTF("Provision complete.\n");
		if (ww->ww_ap_if_enabled) {
			lwip_tiny_dhcpserver_stop();
			dns_hijack_stop();
			wilc_wifi_if_start_stop(&ww->ww_netif_ap, false);
			ww->ww_ap_if_enabled = false;
		}

		ww->ww_link_state = WILC_WIFI_STATE_PROVISION_AP_OFF;

		cf_set_string(CF_KEY_STRING_WIFI_SSID, ssid);
		cf_set_string(CF_KEY_STRING_WIFI_KEY, pass);
		cf_set_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, (uint8_t)enc);

		/*
		 * Delete any static IP settings. This will ensure we're
		 * starting with a clean slate, and provides a rescue
		 * service if the user messed up.
		 */
		cf_del_uint32(CF_KEY_UINT32_NETWORK_IP);
		cf_del_uint32(CF_KEY_UINT32_NETWORK_MASK);
		cf_del_uint32(CF_KEY_UINT32_NETWORK_GATE);

		configdb_persist();

		wifi_provision_end(ww->ww_prov_handle);
		ww->ww_prov_handle = NULL;
		return;
	}

	if (ww->ww_scan_done) {
#ifdef DEBUG_ENABLED
		const wifi_provision_ssid_t *ps;
		int i;

		DBPRINTF("Scan complete\n");
		for (i = 0; i < ww->ww_prov_ssid_count; i++) {
			ps = wifi_provision_get_ssid(ww->ww_prov_handle, i);
			assert(ps != NULL);

			DBPRINTF("RSSI %2u, Auth %u, SSID %s\n",
			    (unsigned int)ps->ps_rssi, (unsigned int)ps->ps_enc,
			    ps->ps_ssid);
		}
#endif /* DEBUG_ENABLED */

		ww->ww_scan_done = false;

		if (ww->ww_ap_enabled == false) {
			wifi_provision_scan_done(ww->ww_prov_handle);
			wilc_wifi_start_ap(ww);
		}
	}

	if (!timer_timeout_expired(&ww->ww_prov_timeout))
		return;

	ww->ww_prov_blink = 1 - ww->ww_prov_blink;
	wilc_wifi_led_wifi(ww->ww_prov_blink ? WILC_LED_ON : WILC_LED_OFF);
	timer_timeout_start(&ww->ww_prov_timeout, 400);
}

static void
wilc_wifi_start_connecting(struct wilc_wifi *ww)
{

	/*
	 * netif_sta prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 * netif_ap prerequisites:
	 *  - Link down.
	 *  - Interface down.
	 *  - No IP assigned.
	 */

	if (ww->ww_prov_arg.pa_cb != NULL) {
		void (*cb)(void *);
		cb = ww->ww_prov_arg.pa_cb;
		ww->ww_prov_arg.pa_cb = NULL;
		DBFPRINTF("Invoking provisioned callback\n");
		(cb)(ww->ww_prov_arg.pa_cookie);
	}

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

	if (ww->ww_link_status.ls_bssid[0] == '\0' ||
	    wilc_m2m_wifi_connect(ww) != M2M_SUCCESS) {
		wilc_wifi_led_err(WILC_LED_ON);
		DBFPRINTF("connect failed\n");
	} else {
		ww->ww_link_state = WILC_WIFI_STATE_CONNECTING;
		DBFPRINTF("connect succeeded\n");
		ww->ww_link_status.ls_state = NETWORK_LINK_CONNECTING;
		ww->ww_link_status.ls_rssi = 0;
		ww->ww_rssi = F16(0.0);
		wilc_wifi_invoke_link_callback(ww);
		timer_timeout_start(&ww->ww_connect_timeout, 5000);
	}
}

static void
wilc_wifi_handle_connection_state(struct wilc_wifi *ww,
    tstrM2mWifiStateChanged *p)
{

	DBFPRINTF("state %s, err %u, if %s, ch %u\n",
	    (p->u8CurrState == M2M_WIFI_DISCONNECTED) ? "DIS" :
	      ((p->u8CurrState == M2M_WIFI_CONNECTED) ? "CON" : "??"),
	    (unsigned int)p->u8ErrCode,
	    (p->u8IfcId == STATION_INTERFACE) ? "STA" :
	      ((p->u8IfcId == AP_INTERFACE) ? "AP" : "P2P"),
	    (unsigned int)p->u8Ch + 1);

#ifdef WIFI_PROVISION_SUPPORT_SCAN
	if (p->u8CurrState == M2M_WIFI_CONNECTED)
		ww->ww_channel = p->u8Ch + 1;
	else
		ww->ww_channel = 0;
#endif

	if (p->u8IfcId == AP_INTERFACE && ww->ww_ap_enabled &&
	    ww->ww_ap_if_enabled == false) {
		ww->ww_ap_if_enabled = true;

		/* Bring up the AP network interface. */
		wilc_wifi_if_start_stop(&ww->ww_netif_ap, true);

		/* Configure a static IP. */
		ip_addr_t ip, gw, mask;
		ip = AP_ADDR_IP;
		gw = GW_ADDR_IP;
		mask = SN_MASK_IP;
		netif_set_addr(&ww->ww_netif_ap, &ip, &mask, &gw);

		dns_hijack_start(ip);

		/* Enable the DHCP server on the ap interface. */
		ip = BC_ADDR_IP;
		lwip_tiny_dhcpserver_start(&ip);
	}

	if (p->u8IfcId != STATION_INTERFACE ||
	    ww->ww_link_state < WILC_WIFI_STATE_CONNECTING) {
		return;
	}

	if (p->u8CurrState == M2M_WIFI_DISCONNECTED) {
		wilc_wifi_led_net(-1);
		wilc_wifi_led_wifi(WILC_LED_OFF);
		DBFPRINTF("WiFi disconnect\n");
		if (ww->ww_link_state >= WILC_WIFI_STATE_ACQUIRE_ADDRESS) {
			ww->ww_link_status.ls_state = NETWORK_LINK_DOWN;
			ww->ww_link_status.ls_rssi = 0;
			ww->ww_rssi = F16(0.0);
			wilc_wifi_invoke_link_callback(ww);
		}
		ww->ww_link_state = WILC_WIFI_STATE_PROVISIONED;
		ww->ww_sta_if_enabled = false;
		wilc_wifi_if_start_stop(&ww->ww_netif_sta, false);
	} else
	if (p->u8CurrState == M2M_WIFI_CONNECTED) {
		DBFPRINTF("WiFi connect\n");
		wilc_wifi_led_err(WILC_LED_OFF);
		wilc_wifi_led_wifi(WILC_LED_ON);
		ww->ww_link_state = WILC_WIFI_STATE_GET_RSSI;
	}
}

static void
wilc_wifi_new_network(struct wilc_wifi *ww)
{

	wilc_wifi_led_net(-1);
	wilc_wifi_led_wifi(WILC_LED_OFF);

	DBFPRINTF("WiFi disconnect\n");
	if (ww->ww_link_state >= WILC_WIFI_STATE_ACQUIRE_ADDRESS) {
		ww->ww_link_status.ls_state = NETWORK_LINK_DOWN;
		ww->ww_link_status.ls_rssi = 0;
		ww->ww_rssi = F16(0.0);
		wilc_wifi_invoke_link_callback(ww);
	}
	ww->ww_link_state = WILC_WIFI_STATE_PROVISIONED;
	ww->ww_sta_if_enabled = false;
	wilc_wifi_if_start_stop(&ww->ww_netif_sta, false);
}

static network_signal_strength_t
wilc_rssi_to_enum(int8_t rssi)
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
wilc_wifi_handle_rssi(struct wilc_wifi *ww, const int8_t *prssi)
{
	fix16_t rssi;
	int8_t i;

	if (ww->ww_wilc_link_rssi_dead)
		printf("wilc_wifi_handle_rssi: WILC RSSI recovered.\n");

	WILC_STATS(ww->ww_stat_rssi_stats);
	ww->ww_wilc_link_rssi_dead = false;

	if (ww->ww_link_state < WILC_WIFI_STATE_GET_RSSI) {
		WILC_STATS(ww->ww_stat_rssi_early);
		return;
	}

	rssi = fix16_from_int(*prssi);
	ww->ww_rssi = WILC_RSSI_INIT(ww, rssi);
	i = (int8_t) fix16_to_int(WILC_RSSI(ww));

	ww->ww_link_status.ls_rssi = i;
	ww->ww_link_status.ls_sig = wilc_rssi_to_enum(i);
#ifdef WIFI_PROVISION_SUPPORT_SCAN
	ww->ww_link_status.ls_chan = ww->ww_channel;
#else
	ww->ww_link_status.ls_chan = 0;
#endif

	if (ww->ww_link_state == WILC_WIFI_STATE_WAIT_RSSI)
		ww->ww_link_state = WILC_WIFI_STATE_START_STA;
}

#ifdef WIFI_PROVISION_SUPPORT_SCAN
static void
wilc_wifi_scan_next_channel(struct wilc_wifi *ww)
{

	if (ww->ww_scan_channel == M2M_WIFI_CH_ALL) {
		if (ww->ww_scan_count == 0) {
			ww->ww_scan_done = true;
		} else {
			ww->ww_scan_channel = M2M_WIFI_CH_1;
			ww->ww_scan_count -= 1;
		}
	} else
	if (ww->ww_scan_channel == M2M_WIFI_CH_14) {
		ww->ww_scan_channel = M2M_WIFI_CH_ALL;
	} else {
		ww->ww_scan_channel += 1;
	}

	if (ww->ww_scan_done == false) {
		/* Initiate a scan of this channel. */
		m2m_wifi_request_scan(ww->ww_scan_channel);
	}
}

static void
wilc_wifi_handle_scan_done(struct wilc_wifi *ww, tstrM2mScanDone *p)
{

	ww->ww_scan_index = p->u8NumofCh;

	if (ww->ww_scan_index == 0) {
		/* Nothing on this channel. Scan the next. */
		wilc_wifi_scan_next_channel(ww);
	} else {
		/* Start reading the scan results. */
		m2m_wifi_req_scan_result(ww->ww_scan_index - 1);
	}
}

static void
wilc_wifi_handle_scan_result(struct wilc_wifi *ww, tstrM2mWifiscanResult *p)
{
	wifi_provision_enc_t enc;
	int i;

	switch (p->u8AuthType) {
	default:
		enc = WIFI_PROVISION_ENC_OPEN;
		break;
	case M2M_WIFI_SEC_WPA_PSK:
		enc = WIFI_PROVISION_ENC_WPA;
		break;
	case M2M_WIFI_SEC_WEP:
		enc = WIFI_PROVISION_ENC_WEP;
		break;
	case M2M_WIFI_SEC_802_1X:
		enc = WIFI_PROVISION_ENC_802_1X;
		break;
	}

	i = wifi_provision_add_ssid(ww->ww_prov_handle,
	    (const char *)p->au8SSID, p->s8rssi, enc);
	if (i > 0)
		ww->ww_prov_ssid_count = i;

	/*
	 * Update the channel congestion value based on the AP's RSSI.
	 * We ignore anything under -100 dbm.
	 */
	if (p->u8ch > 0 && p->u8ch <= 14 && p->s8rssi <= 0 && p->s8rssi >= -100)
		ww->ww_chan_usage[p->u8ch - 1] += (uint16_t)(100 + p->s8rssi);

	ww->ww_scan_index -= 1;
	if (ww->ww_scan_index == 0)
		wilc_wifi_scan_next_channel(ww);
	else
		m2m_wifi_req_scan_result(ww->ww_scan_index - 1);
}
#endif /* WIFI_PROVISION_SUPPORT_SCAN */

static bool
wilc_check_rssi(struct wilc_wifi *ww)
{
	sint8 res;

	if (ww->ww_wilc_link_rssi_dead)
		return false;

	res = m2m_wifi_req_curr_rssi();
	if (res != M2M_SUCCESS) {
		printf("wilc_check_rssi: request failed. %d\n", (int)res);
		WILC_STATS(ww->ww_stat_rssi_busy);
		return false;
	}

	WILC_STATS(ww->ww_stat_rssi_reqs);
	timer_timeout_start(&ww->ww_link_timer, TIMER_HZ);

	return true;
}

static void
wilc_wifi_request_rssi(struct wilc_wifi *ww)
{

	/* If WILC firmware is in a huff, skip trying to read RSSI. */
	if (wilc_check_rssi(ww) == false)
		ww->ww_link_state = WILC_WIFI_STATE_START_STA;
	else
		ww->ww_link_state = WILC_WIFI_STATE_WAIT_RSSI;
}

static void
wilc_wifi_handle_dhcp_conf(struct wilc_wifi *ww)
{

	if (ww->ww_link_state != WILC_WIFI_STATE_ACQUIRE_ADDRESS)
		return;

	ww->ww_link_state = WILC_WIFI_STATE_CONFIGURED;
	wilc_wifi_led_err(WILC_LED_OFF);

	ww->ww_sta_if_enabled = true;
	ww->ww_link_status.ls_state = NETWORK_LINK_UP;
	wilc_wifi_invoke_link_callback(ww);
}

#ifdef DEBUG_ENABLED
static const char *wilc_wifi_link_msg_sta[] = {
	"REQ_CONNECT",
	"REQ_GET_CONN_INFO",
	"RESP_CONN_INFO",
	"REQ_DISCONNECT",
	"RESP_CON_STATE_CHANGED",
	"REQ_SLEEP",
	"REQ_SCAN",
	"RESP_SCAN_DONE",
	"REQ_SCAN_RESULT",
	"RESP_SCAN_RESULT",
	"REQ_START_WPS",
	"REQ_DISABLE_WPS",
	"RES_WPS",
	"RESP_IP_CONFIGURED",
	"RESP_IP_CONFLICT",
	"REQ_ENABLE_MONITORING",
	"REQ_DISABLE_MONITORING",
	"RESP_WIFI_RX_PACKET",
	"REQ_SEND_WIFI_PACKET",
	"REQ_LSN_INT",
	"REQ_SEND_ETHERNET_PACKET",
	"RESP_ETHERNET_RX_PACKET",
	"REQ_SET_SCAN_OPTION",
	"REQ_SET_SCAN_REGION",
	"REQ_SET_SCAN_LIST",
	"REQ_SET_MAC_MCAST",
	"REQ_SET_P2P_IFC_ID",
	"RESP_PACKET_SENT",
	"REQ_CERT_ADD_CHUNK",
	"REQ_CERT_DOWNLOAD_DONE",
	"REQ_CHG_MONITORING_CHNL",
	"RESP_ANT_SWITCH_MODE",
	"REQ_P2P_AUTH"
};
static const char *wilc_wifi_link_msg_config[] = {
	"REQ_RESTART",
	"REQ_SET_MAC_ADDRESS",
	"REQ_CURRENT_RSSI",
	"RESP_CURRENT_RSSI",
	"REQ_SET_DEVICE_NAME",
	"REQ_CUST_INFO_ELEMENT",
	"RESP_FIRMWARE_STRTED",
	"REQ_SET_TX_POWER",
	"REQ_SET_MAX_TX_RATE",
	"REQ_ENABLE_MCAST_FILTER",
	"REQ_DISABLE_MCAST_FILTER",
	"REQ_SET_ANT_SWITCH_MODE",
	"P2P_AUTH_RES"
};

static const char *wilc_wifi_link_msg_ap[] = {
	"REQ_ENABLE_AP",
	"REQ_DISABLE_AP",
	"REQ_AP_ASSOC_INFO",
	"RESP_AP_ASSOC_INFO",
	"REQ_AP_BLACK_LIST"
};
#endif /* DEBUG_ENABLED */

static void
wilc_wifi_link_cb(uint8_t msg, void *arg)
{
	struct wilc_wifi *ww = wilc_wifi_state;

#ifdef DEBUG_ENABLED
	const char *str;

	if (msg >= M2M_WIFI_REQ_CONNECT && msg <= M2M_WIFI_REQ_P2P_AUTH)
		str = wilc_wifi_link_msg_sta[msg - M2M_WIFI_REQ_CONNECT];
	else
	if (msg >= M2M_WIFI_REQ_RESTART && msg <= M2M_WIFI_P2P_AUTH_RES)
		str = wilc_wifi_link_msg_config[msg - M2M_WIFI_REQ_RESTART];
	else
	if (msg >= M2M_WIFI_REQ_ENABLE_AP && msg <= M2M_WIFI_REQ_AP_BLACK_LIST)
		str = wilc_wifi_link_msg_ap[msg - M2M_WIFI_REQ_ENABLE_AP];
	else
		str = "?????";

	if (msg != M2M_WIFI_RESP_CURRENT_RSSI)
		DBFPRINTF("msg M2M_WIFI_%s\n", str);
#endif /* DEBUG_ENABLED */

	assert(ww != NULL);

	switch (msg) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		wilc_wifi_handle_connection_state(ww, arg);
		break;

	case M2M_WIFI_RESP_CURRENT_RSSI:
		wilc_wifi_handle_rssi(ww, arg);
		break;

	case M2M_WIFI_RESP_FIRMWARE_STRTED:
		break;

#ifdef WIFI_PROVISION_SUPPORT_SCAN
	case M2M_WIFI_RESP_SCAN_DONE:
		wilc_wifi_handle_scan_done(ww, arg);
		break;

	case M2M_WIFI_RESP_SCAN_RESULT:
		wilc_wifi_handle_scan_result(ww, arg);
		break;
#endif

	default:
		DBFPRINTF("unhandled event 0x%02x\n", (unsigned int)msg);
		break;
	}
}

static void
wilc_wifi_process_tx(struct wilc_wifi *ww)
{
	struct wilc_pbuf *wp;

	while (ww->ww_tx_pending) {
		wp = TAILQ_FIRST(&ww->ww_tx_queue);
		assert(wp != NULL);

		if (wilc_wifi_if_tx(ww, wp->wp_pbuf, wp->wp_if)) {
			/* Packet was sent. */
			ww->ww_tx_pending--;
			TAILQ_REMOVE(&ww->ww_tx_queue, wp, wp_qent);
			pbuf_free(wp->wp_pbuf);
			TAILQ_INSERT_TAIL(&ww->ww_tx_free_queue, wp, wp_qent);
		} else {
			/*
			 * Failed to send (crap WILC firmware).
			 * Try again shortly.
			 */
			WILC_STATS(ww->ww_stat_tx_busy);
			break;
		}
	}
}

static uint32_t
wilc_wifi_if_worker(struct netif *netif)
{
	struct wilc_wifi *ww = netif->state;
//	rtos_saved_ipl_t ipl;
#define	WILC_IF_WORKER_DEFAULT_SLEEP	20
	uint32_t rv = WILC_IF_WORKER_DEFAULT_SLEEP;
//	uint8_t event;
#ifdef DEBUG_ENABLED
	static uint8_t old_state = 0xffu;
#endif
	uint8_t state_change;

	do {
//		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
//		event = ww->ww_wifi_event;
		ww->ww_wifi_event = 0;
//		rtos_ipl_restore(ipl);

		/*
		 * Process events received from the WILC firmware. This will:
		 *  - Pass Rx data packets (if the link is up) to lwIP.
		 *  - Invoke event callbacks.
		 */
//		if (event) {
			ww->ww_tx_just_queue = true;
			m2m_wifi_handle_events(NULL);
			ww->ww_tx_just_queue = false;
//		}

		if (ww->ww_link_status.ls_state == NETWORK_LINK_UP ||
		    ww->ww_link_status.ls_state == NETWORK_LINK_PROVISIONING) {
			wilc_wifi_process_tx(ww);
		}
	} while (ww->ww_wifi_event);

#ifdef DEBUG_ENABLED
	state_change = old_state != ww->ww_link_state;
	if (state_change)
		old_state = ww->ww_link_state;
#else
	state_change = 0;
#endif

	switch (ww->ww_link_state) {
	case WILC_WIFI_STATE_INIT:
		if (state_change)
			DBFPRINTF("INIT\n");
		if (wilc_wifi_get_enc(ww) == M2M_WIFI_SEC_INVALID)
			wilc_wifi_start_provision(ww);
		else
			wilc_wifi_start_connecting(ww);
		break;

	case WILC_WIFI_STATE_NEW_PROVISION:
		if (state_change)
			DBFPRINTF("NEW_PROVISIONED\n");
		wilc_wifi_new_provision(ww);
		break;

	case WILC_WIFI_STATE_PROVISIONING:
		if (state_change)
			DBFPRINTF("PROVISIONING\n");
		wilc_wifi_provisioning(ww);
		if (ww->ww_tx_pending)
			rv = 1;
		break;

	case WILC_WIFI_STATE_PROVISION_AP_OFF:
		if (state_change)
			DBFPRINTF("PROVISION AP OFF\n");
		m2m_wifi_disable_ap();
		ww->ww_ap_enabled = false;
		ww->ww_link_state = WILC_WIFI_STATE_PROVISIONED;
		break;

	case WILC_WIFI_STATE_PROVISIONED:
		if (state_change)
			DBFPRINTF("PROVISIONED\n");
		wilc_wifi_start_connecting(ww);
		break;

	case WILC_WIFI_STATE_NEW_NETWORK:
		if (state_change)
			DBFPRINTF("NEW_NETWORK\n");
		wilc_wifi_new_network(ww);
		break;

	case WILC_WIFI_STATE_CONNECTING:
		if (state_change)
			DBFPRINTF("CONNECTING\n");
		if (timer_timeout_expired(&ww->ww_connect_timeout))
			ww->ww_link_state = WILC_WIFI_STATE_INIT;
		break;

	case WILC_WIFI_STATE_GET_RSSI:
		if (state_change)
			DBFPRINTF("GET_RSSI\n");
		wilc_wifi_request_rssi(ww);
		break;

	case WILC_WIFI_STATE_WAIT_RSSI:
		if (state_change)
			DBFPRINTF("WAIT_RSSI\n");
		if (timer_timeout_expired(&ww->ww_link_timer)) {
			DBFPRINTF("RSSI dead\n");
			ww->ww_wilc_link_rssi_dead = true;
			ww->ww_link_state = WILC_WIFI_STATE_START_STA;
		}
		break;

	case WILC_WIFI_STATE_START_STA:
		if (state_change)
			DBFPRINTF("START_STA\n");
		wilc_wifi_if_start_stop(&ww->ww_netif_sta, true);
		ww->ww_link_state = WILC_WIFI_STATE_ACQUIRE_ADDRESS;
		break;

	case WILC_WIFI_STATE_ACQUIRE_ADDRESS:
		if (state_change)
			DBFPRINTF("ACQUIRE_ADDRESS\n");
		/* Start the DHCP client */
		wilc_wifi_handle_dhcp_conf(ww);
		break;

	case WILC_WIFI_STATE_CONFIGURED:
		if (state_change)
			DBFPRINTF("CONFIGURED\n");
		ww->ww_link_state = WILC_WIFI_STATE_SHOWTIME;
		break;

	case WILC_WIFI_STATE_SHOWTIME:
		if (state_change)
			DBFPRINTF("SHOWTIME\n");
		rv = (ww->ww_tx_pending) ? 1: RTOS_MAX_SLEEP_MS;
		break;
	}

	return rv;
}

#ifdef WILC_OPT_GPIO
static int
wilc_wifi_gpio2m2m(uint8_t gp, uint8_t *m2m)
{
	switch (gp) {
	case WILC_GPIO_PIN_3:
		*m2m = M2M_PERIPH_GPIO3;
		return 0;
	case WILC_GPIO_PIN_4:
		*m2m = M2M_PERIPH_GPIO4;
		return 0;
	case WILC_GPIO_PIN_5:
		*m2m = M2M_PERIPH_GPIO5;
		return 0;
	case WILC_GPIO_PIN_6:
		*m2m = M2M_PERIPH_GPIO6;
		return 0;
	}
	return -1;
}
#endif /* WILC_OPT_GPIO */

static int8_t
wilc_wifi_locked_ioctl(void *cookie, uint8_t cmd, void *arg)
{
	struct wilc_wifi *ww = cookie;
	struct wilc_prov_arg *pa;
	int8_t rv = 0;
#ifdef WILC_OPT_GPIO
	struct wilc_gpio *wg;
	uint8_t m2m;
#endif

	(void) cookie;

	switch (cmd) {
	case NETWORK_IOCTL_WIFI_PROVISION:
		if (arg == NULL)
			return -1;

		pa = (struct wilc_prov_arg *)arg;
		ww->ww_prov_arg = *pa;
		ww->ww_prov_old_state = ww->ww_link_state;
		ww->ww_link_state = WILC_WIFI_STATE_NEW_PROVISION;
		network_driver_soft_timeout(ww->ww_network_state, 0);
		break;

#ifdef WILC_OPT_GPIO
	case WILC_IOCTL_INIT_GPIO:
		wg = arg;
		if (wilc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		if (wg->wg_value & WILC_GPIO_DIRECTION_OUT)
			m2m_periph_gpio_set_dir(m2m, 1);
		else
			m2m_periph_gpio_set_dir(m2m, 0);

		if (wg->wg_value & WILC_GPIO_PULLUP_ON)
			m2m_periph_gpio_pullup_ctrl(m2m, 1);
		else
			m2m_periph_gpio_pullup_ctrl(m2m, 0);
		break;

	case WILC_IOCTL_WRITE_GPIO:
		wg = arg;
		if (wilc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		m2m_periph_gpio_set_val(m2m, wg->wg_value ? 1 : 0);
		break;

	case WILC_IOCTL_READ_GPIO:
		wg = arg;
		if (wilc_wifi_gpio2m2m(wg->wg_pin, &m2m) < 0)
			return 1;
		m2m_periph_gpio_get_val(m2m, &wg->wg_value);
		break;
#endif /* WILC_OPT_GPIO */

	default:
		rv = -1;
		break;
	}

	return rv;
}

static uint8_t
wilc_wifi_reset(struct wilc_wifi *ww)
{
	tstrWifiInitParam param;

	DBFPRINTF("\n");

	(ww->ww_control)(ww->ww_cookie, WILC_WIFI_RESET, NULL);

	memset(&param, 0, sizeof(param));
	param.pfAppWifiCb = wilc_wifi_link_cb;

	param.strEthInitParam.pfAppEthCb = wilc_wifi_if_eth_cb;
	param.strEthInitParam.au8ethRcvBuf = ww->ww_rx_buffer;
	param.strEthInitParam.u16ethRcvBufSize = sizeof(ww->ww_rx_buffer);

	if (m2m_wifi_init(&param, &ww->ww_fw_info) != M2M_SUCCESS) {
		DBFPRINTF("m2m_wifi_init failed\n");
		return 0;
	}

	wilc_wifi_led_init();

	char name[M2M_DEVICE_NAME_MAX];
	if (cf_get_string(CF_KEY_STRING_WIFI_NAME, sizeof(name), name) != 0)
		m2m_wifi_set_device_name((uint8 *)name, strlen(name));

	return 1;
}

void
wilc_wifi_interrupt(void *arg)
{
	struct wilc_wifi *ww = arg;

	if (ww->ww_irq_handler != NULL) {
		(ww->ww_irq_handler)();
		ww->ww_wifi_event = 1;
		network_driver_hw_interrupt(ww->ww_network_state);
	} else {
		uint8_t enable = 0;
		(ww->ww_control)(ww->ww_cookie, WILC_WIFI_IRQ_CTL, &enable);
	}
}

int8_t
wilc_wifi_ioctl(void *arg, uint8_t op, void *oparg)
{
	struct wilc_wifi *ww = arg;

	return network_ioctl(ww->ww_network_state, op, oparg);
}

static void
wilc_wifi_if_tx_cb(struct wilc_wifi *ww)
{

	(void) ww;
}

static void
wilc_wifi_if_rx_cb(struct wilc_wifi *ww, tstrM2MDataBufCtrl *cb)
{
	struct netif *netif;
	struct pbuf *p;
	uint16_t plen = cb->u16DataSize + cb->u8DataOffset;

	wilc_wifi_led_net(WILC_LED_ON);

	if ((p = ww->ww_rx_pbuf) == NULL) {
		/*
		 * Packet is in ww->ww_rx_buffer.
		 */
		p = pbuf_alloc(PBUF_RAW, WILC_RX_PBUF_SZ, PBUF_POOL);
		if (p == NULL) {
			WILC_STATS(ww->ww_stat_rx_no_pbufs);
			goto done;
		}

		memcpy(p->payload, ww->ww_rx_buffer, plen);
	} else {
		ww->ww_rx_pbuf = NULL;
	}

	netif = (cb->u8IfcId == STATION_INTERFACE) ? &ww->ww_netif_sta :
	    &ww->ww_netif_ap;

	p->tot_len = p->len = plen;

	if (pbuf_header(p, -(cb->u8DataOffset - ETH_PAD_SIZE)) == 0) {
		if (netif->input(p, netif) != ERR_OK) {
			WILC_STATS(ww->ww_stat_rx_dropped);
			pbuf_free(p);
			DBFPRINTF("netif->input failed\n");
		} else {
			WILC_STATS(ww->ww_stat_rx_packets);
		}
	} else {
		DBFPRINTF("pbuf_header() failed\n");
		pbuf_free(p);
		WILC_STATS(ww->ww_stat_rx_dropped);
	}

	ww->ww_rx_pbuf = pbuf_alloc(PBUF_RAW, WILC_RX_PBUF_SZ, PBUF_POOL);
	if (ww->ww_rx_pbuf == NULL) {
		DBFPRINTF("refill pbuf_alloc() failed\n");
		WILC_STATS(ww->ww_stat_rx_no_pbufs);
		m2m_wifi_set_receive_buffer(ww->ww_rx_buffer,
		    sizeof(ww->ww_rx_buffer));
	} else {
		m2m_wifi_set_receive_buffer(ww->ww_rx_pbuf->payload,
		    WILC_RX_PBUF_SZ);
	}

 done:
	wilc_wifi_led_net(WILC_LED_OFF);
}

static void
wilc_wifi_if_eth_cb(uint8 mtype, void *buff, void *ctrl)
{
	struct wilc_wifi *ww = wilc_wifi_state;

	(void) buff;

	if (mtype == M2M_WIFI_RESP_PACKET_SENT)
		wilc_wifi_if_tx_cb(ww);
	else
	if (mtype == M2M_WIFI_RESP_ETHERNET_RX_PACKET)
		wilc_wifi_if_rx_cb(ww, (tstrM2MDataBufCtrl *)ctrl);
}

static int8_t
wilc_wifi_if_locked_ioctl(struct netif *netif, uint8_t cmd, void *arg)
{
	struct wilc_wifi *ww = netif->state;

	return wilc_wifi_locked_ioctl(ww, cmd, arg);
}

static void
wilc_wifi_if_get_mac(struct netif *netif, uint8_t *mac)
{
	struct wilc_wifi *ww = netif->state;
	uint8_t dummy_mac[6];

	if (netif == &ww->ww_netif_sta)
		m2m_wifi_get_mac_address(dummy_mac, mac);
	else
		m2m_wifi_get_mac_address(mac, dummy_mac);
}

static void
wilc_wifi_if_start_stop(struct netif *netif, bool do_start)
{
	struct wilc_wifi *ww = netif->state;
	struct wilc_pbuf *wp;

	m2m_wifi_set_receive_buffer(&ww->ww_rx_buffer,sizeof(ww->ww_rx_buffer));

	if (do_start) {
		TAILQ_INIT(&ww->ww_tx_queue);
		TAILQ_INIT(&ww->ww_tx_free_queue);

		for (unsigned int i = 0; i < WILC_MAX_TX_QUEUE_SIZE; i++) {
			wp = &ww->ww_tx_pbufs[i];
			TAILQ_INSERT_TAIL(&ww->ww_tx_free_queue, wp, wp_qent);
		}

		netif_set_link_up(netif);
		netif_set_up(netif);
		netif_set_default(netif);
	} else {
		netif_set_addr(netif, NULL, NULL, NULL);
		netif_set_down(netif);
		netif_set_link_down(netif);
		netif_set_default(NULL);

		if (ww->ww_rx_pbuf != NULL) {
			pbuf_free(ww->ww_rx_pbuf);
			ww->ww_rx_pbuf = NULL;
		}

		ww->ww_wilc_link_rssi_dead = false;

		/* Discard any queued Tx pbufs */
		while ((wp = TAILQ_FIRST(&ww->ww_tx_queue)) != NULL) {
			assert(ww->ww_tx_pending > 0);
			ww->ww_tx_pending--;
			TAILQ_REMOVE(&ww->ww_tx_queue, wp, wp_qent);
			assert(wp->wp_pbuf != NULL);
			pbuf_free(wp->wp_pbuf);
			wp->wp_pbuf = NULL;
			TAILQ_INSERT_TAIL(&ww->ww_tx_free_queue, wp, wp_qent);
		}
		assert(ww->ww_tx_pending == 0);
	}
}

static void
wilc_wifi_if_link_status(struct netif *netif, network_link_status_callback_t cb,
    void *cbarg)
{
	struct wilc_wifi *ww = netif->state;

	ww->ww_link_cb = cb;
	ww->ww_link_cb_arg = cbarg;
}

/*
 * Returns 'false' if packet was accepted by the WILC firmware, or if the
 * interface is down.
 * Otherwise returns 'true' if the WILC module's Tx queue is full.
 */
static bool
wilc_wifi_if_tx(struct wilc_wifi *ww, struct pbuf *p, enum wilc_wifi_if_id id)
{
	uint8_t *tx_packet;
	uint16_t tx_len;
	bool up, tx_twice;
	sint8 hw;

	/* Do nothing if the target interface is down. */
	up = (id == WILC_IF_STA) ? ww->ww_sta_if_enabled : ww->ww_ap_if_enabled;
	if (!up) {
		DBFPRINTF("interface down\n");
		return false;
	}

	/*
	 * Brain-dead Atmel/Microchip WILC driver is brain-dead...
	 *
	 *  1. It can't do scatter/gather over SPI.
	 *  2. There must be space in the Tx buffer *BEFORE THE PAYLOAD* for
	 *     it to add some additional headers. The number of bytes of
	 *     space is given by M2M_ETHERNET_HDR_OFFSET, nominally 36 bytes.
	 *  3. It's just a pile of shite.
	 *  4. Don't get me started on the firmware *inside* the device...
	 */

	/* PBUF_LINK_ENCAPSULATION_HLEN is 36 in lwipopts.h */
	pbuf_header(p, M2M_ETHERNET_HDR_OFFSET);

#if ETH_PAD_SIZE
	/*
	 * WILC driver deals with this internally via M2M_ETH_PAD_SIZE
	 */
#endif

	tx_len = p->tot_len - ETH_PAD_SIZE - M2M_ETHERNET_HDR_OFFSET;

	if (p->tot_len == p->len) {
		/*
		 * Entire packet is contained in a single pbuf.
		 * No need to defragment.
		 */
		tx_packet = p->payload;
		WILC_STATS(ww->ww_stat_tx_contig);
	} else {
		/*
		 * Packet is scattered across multiple pbufs.
		 * Copy into our contiguous bounce buffer.
		 */
		uint8_t *buff = ww->ww_tx_buffer;
		uint16_t l = p->tot_len;

		assert(l <= sizeof(ww->ww_tx_buffer));
		WILC_STATS(ww->ww_stat_tx_frag);

		for (struct pbuf *q = p; q != NULL; q = q->next) {
			assert(q->len <= l);
			memcpy(buff, q->payload, q->len);
			buff += q->len;
			l -= q->len;
		}

		assert(l == 0);
		tx_packet = ww->ww_tx_buffer;
	}

	tx_twice = false;
	if (id == WILC_IF_STA && ww->ww_tx_bogons) {
		/*
		 * When operating in STA mode, we need to work-around a WILC
		 * firmware bug which causes random Tx packets to be dropped
		 * when the WiFi channel is busy. There is no way to determine
		 * that a packet was dropped; firmware does not return an
		 * error code - the packet simply disappears.
		 * This happens frequently enough to break TCP throughput, to
		 * the point where streams can pause for many seconds while
		 * TCP tries to compensate for the packet loss.
		 * A partial solution is to send all TCP packets twice. It's
		 * not perfect, but there is no other way to ensure TCP
		 * throughput doesn't fall through the floor.
		 */
		const uint16_t *etype;

		/* Point to the Ethernet packet type... */
		etype = (const uint16_t *)(uintptr_t)&tx_packet[
		    M2M_ETHERNET_HDR_OFFSET + ETH_PAD_SIZE + 12];
		/* Payload must be IPv4 */
		if (*etype == 0x8) {
			const uint8_t *proto;

			/* Point to the IPv4 protocol number */
			proto = &tx_packet[M2M_ETHERNET_HDR_OFFSET +
			    ETH_PAD_SIZE + 14 + 9];

			/* Apply the work-around for TCP packets only. */
			tx_twice = (*proto == 6);
		}
	}

	hw = m2m_wifi_send_ethernet_pkt(tx_packet, tx_len,
	    (id == WILC_IF_STA) ? STATION_INTERFACE : AP_INTERFACE);

	if (hw == M2M_SUCCESS) {
		WILC_STATS(ww->ww_stat_tx_sent);

		if (tx_twice) {
			assert(id == WILC_IF_STA);

			/* Re-send once more for luck. */
			(void) m2m_wifi_send_ethernet_pkt(tx_packet, tx_len,
			    STATION_INTERFACE);
		}
	} else
		WILC_STATS(ww->ww_stat_tx_deferred);

	return hw == M2M_SUCCESS;
}

static err_t
wilc_wifi_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct wilc_wifi *ww = netif->state;
	struct wilc_pbuf *wp;
	enum wilc_wifi_if_id if_id;

	if_id = (netif == &ww->ww_netif_sta) ? WILC_IF_STA : WILC_IF_AP;

	if (ww->ww_tx_just_queue == false && ww->ww_tx_pending == 0 &&
	    wilc_wifi_if_tx(ww, p, if_id)) {
		return ERR_OK;
	}

	if ((wp = TAILQ_FIRST(&ww->ww_tx_free_queue)) != NULL) {
		TAILQ_REMOVE(&ww->ww_tx_free_queue, wp, wp_qent);
	} else {
		WILC_STATS(ww->ww_stat_tx_dropped);
		return ERR_MEM;
	}

	pbuf_ref(p);
	wp->wp_pbuf = p;
	wp->wp_if = if_id;
	TAILQ_INSERT_TAIL(&ww->ww_tx_queue, wp, wp_qent);
	WILC_STATS(ww->ww_stat_tx_queued);
	ww->ww_tx_pending++;

	if (ww->ww_tx_just_queue == false)
		network_driver_soft_timeout(ww->ww_network_state, 0);

	return ERR_OK;
}

static err_t
wilc_wifi_common_netif_init(struct netif *netif)
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

	wilc_wifi_if_get_mac(netif, mac);
	memcpy(netif->hwaddr, mac, ETH_HWADDR_LEN);
	netif->output = etharp_output;
	netif->linkoutput = wilc_wifi_low_level_output;
        netif->hwaddr_len = ETH_HWADDR_LEN;
        netif->mtu = 1500;
        netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
            NETIF_FLAG_ETHERNET;

	return ERR_OK;
}

static err_t
wilc_wifi_sta_netif_init(struct netif *netif)
{

	netif->name[0] = 's';
	netif->name[1] = 't';

	return wilc_wifi_common_netif_init(netif);
}

static err_t
wilc_wifi_ap_netif_init(struct netif *netif)
{

	netif->name[0] = 'a';
	netif->name[1] = 'p';

	return wilc_wifi_common_netif_init(netif);
}

static bool
wilc_wifi_manual_provision(void *arg, const char *ssid, const char *pass,
    const char *name)
{
	struct wilc_wifi *ww = arg;
	wifi_provision_enc_t enc;
	char *hostname = NULL;
	uint8_t penc;
	cf_len_t l;
	bool changed = false;

	assert(ssid != NULL);
	assert(pass == NULL || strlen(pass) >= 8);

	enc = (pass != NULL) ? WIFI_PROVISION_ENC_WPA : WIFI_PROVISION_ENC_OPEN;

	l = cf_get_string(CF_KEY_STRING_WIFI_NAME, M2M_DEVICE_NAME_MAX, NULL);
	if (l != 0) {
		hostname = zone_malloc((size_t)l);
		if (hostname != NULL) {
			cf_get_string(CF_KEY_STRING_WIFI_NAME,
			    M2M_DEVICE_NAME_MAX, hostname);
		}
	}

	/* Update credentials. */
	if (strcmp(ww->ww_link_status.ls_bssid, ssid) != 0) {
		cf_set_string(CF_KEY_STRING_WIFI_SSID, ssid);
		changed = true;
	}

	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &penc) == 0 ||
	    enc != penc) {
		cf_set_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, (uint8_t)enc);
		changed = true;
	}

	if (pass == NULL) {
		cf_del_string(CF_KEY_STRING_WIFI_KEY);
		changed = ww->ww_link_status.ls_key[0] != '\0';
	} else
	if (strcmp(ww->ww_link_status.ls_key, pass) != 0) {
		cf_set_string(CF_KEY_STRING_WIFI_KEY, pass);
		changed = true;
	}

	/* Note: Changing hostname doesn't affect our return value. */
	if (name != NULL) {
		if (hostname == NULL || strcmp(name, hostname) != 0)
			cf_set_string(CF_KEY_STRING_WIFI_NAME, name);
	} else
	if (hostname != NULL) {
		cf_del_string(CF_KEY_STRING_WIFI_NAME);
	}

	configdb_persist();

	if (hostname != NULL)
		zone_free(hostname);

	/*
	 * We're likely being called via an httpd post handler. Thus, it
	 * would not be a good idea to tear down the network immediately.
	 * Schedule an event for later...
	 */
	rtos_timer_start(ww->ww_new_network_timer);

	return changed;
}

static void
wilc_network_change_timer_callback(rtos_timer_t handle)
{
	struct wilc_wifi *ww = rtos_timer_get_id(handle);

	/* Danger: We are not holding the network driver mutex... */

	/*
	 * Change state - we'll deal with this next time the driver
	 * is invoked.
	 */
	ww->ww_link_state = WILC_WIFI_STATE_NEW_NETWORK;
}

void *
wilc_wifi_lwip_attach(const struct wilc_wifi_attach_args *waa)
{
	struct network_lwip_driver *nld;
	struct wilc_wifi *ww;

	assert(wilc_wifi_state == NULL);

	wilc_wifi_state = ww = zone_calloc(1, sizeof(*ww));
	if (ww == NULL)
		return NULL;

	nld = &ww->ww_driver;
	nld->nld_netif = &ww->ww_netif_sta;
	nld->nld_handle_interrupt = wilc_wifi_if_worker;
	nld->nld_link_check = NULL;
	nld->nld_link_status = wilc_wifi_if_link_status;
	nld->nld_get_mac = wilc_wifi_if_get_mac;
	nld->nld_ioctl = wilc_wifi_if_locked_ioctl;

	ww->ww_cookie = waa->waa_cookie;
	ww->ww_read_write = waa->waa_read_write;
	ww->ww_control = waa->waa_control;

	uint8_t bug;
	if (cf_get_uint8(CF_KEY_UINT8_WILC_TX_BOGONS, &bug) == 0)
		bug = 0;
	ww->ww_tx_bogons = bug != 0;

	if (wilc_wifi_reset(ww) == 0) {
		wilc_wifi_state = NULL;
		zone_free(ww);
		return NULL;
	}

	netif_add(&ww->ww_netif_sta, NULL, NULL, NULL, ww,
	    wilc_wifi_sta_netif_init, netif_input);

	netif_add(&ww->ww_netif_ap, NULL, NULL, NULL, ww,
	    wilc_wifi_ap_netif_init, netif_input);

	if ((ww->ww_network_state = network_lwip_attach(nld)) == NULL) {
		wilc_wifi_state = NULL;
		zone_free(ww);
		return NULL;
	}

	ww->ww_new_network_timer = rtos_timer_create("WILCTimer", 2000, 0,
	    wilc_network_change_timer_callback, ww);
	assert(ww->ww_new_network_timer != NULL);
	network_httpd_set_provision_info_cb(wilc_wifi_manual_provision, ww);

#ifdef WILC_OPT_COMMAND
	SHELL_CMD_ADD(wifi);
#endif

	return ww;
}

void
wilc_wifi_load_defaults(void)
{

	cf_del_uint8(CF_KEY_UINT8_WILC_TX_BOGONS);
}

#ifdef WILC_OPT_COMMAND
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
"wifi pass <wifi network passphrase>\n"
"\tConfigure the WiFi passphrase. Specify \"none\" if the WiFi network\n"
"\tdoes not require a passphrase. Otherwise, the passphrase must be at\n"
"\tleast 8 characters in length. Again, enclose the passphrase in\n"
"\tdouble quotes if it contains spaces.\n"
"wifi name <client name>\n"
"\tSpecifies a human-friendly name for " PRODUCT_NAME ". Most good DHCP "
	"servers\n"
"\twill log the client name against the assigned IP address and may\n"
"\tpopulate the local DNS with the name for ease of access. This is\n"
"\tblank by default.\n"
;

static void
wilc_wifi_cmd(FILE *fp, uint8_t argc, const char * const *argv)
{
	struct wilc_wifi *ww = wilc_wifi_state;

	assert(ww != NULL);
	(void) ww;

	if (argc >= 2) {
#ifdef DEBUG_ENABLED
		if (strcasecmp(argv[0], "debug") == 0) {
			if (strcasecmp(argv[1], "on") == 0)
				wilc_wifi_debug = 1;
			else
			if (strcasecmp(argv[1], "off") == 0)
				wilc_wifi_debug = 0;
		} else
#endif
		if (strcasecmp(argv[0], "txbug") == 0) {
			bool old = ww->ww_tx_bogons;
			if (strcasecmp(argv[1], "on") == 0)
				ww->ww_tx_bogons = true;
			else
			if (strcasecmp(argv[1], "off") == 0)
				ww->ww_tx_bogons = false;
			else {
				fprintf(fp, "\"on\" or \"off\" expected.\n");
				return;
			}
			if (old != ww->ww_tx_bogons) {
				cf_set_uint8(CF_KEY_UINT8_WILC_TX_BOGONS,
				    (uint8_t)ww->ww_tx_bogons);
			}
		} else
		if (strcasecmp(argv[0], "name") == 0) {
			cf_set_string(CF_KEY_STRING_WIFI_NAME, argv[1]);
		} else
		if (strcasecmp(argv[0], "ssid") == 0) {
			cf_set_string(CF_KEY_STRING_WIFI_SSID, argv[1]);
		} else
		if (strcasecmp(argv[0], "pass") == 0) {
			wifi_provision_enc_t st;

			if (strcasecmp(argv[1], "none") == 0) {
				st = WIFI_PROVISION_ENC_OPEN;
				cf_del_string(CF_KEY_STRING_WIFI_KEY);
			} else {
				st = WIFI_PROVISION_ENC_WPA;
				if (strlen(argv[1]) < 8) {
					fprintf(fp, "Passphrase must be at "
					    "least 8 characters in length.\n");
					return;
				}
				cf_set_string(CF_KEY_STRING_WIFI_KEY, argv[1]);
			}
			cf_set_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, (uint8_t)st);
		} else {
			goto usage;
		}

		configdb_persist();
	} else
	if (argc != 0) {
 usage:
		fprintf(fp, "%s", wifi_cmd_usage);
		return;
	}

	struct wilc_fw_timestamp ts;
	(ww->ww_control)(ww->ww_cookie, WILC_WIFI_GET_FW_TIMESTAMP, &ts);

	fprintf(fp, "WILC1000 ChipId 0x%" PRIx32 "\n"
	    "Firmware %u.%u.%u, Driver %u.%u.%u\nFirmware Date %s, %s\n",
	    ww->ww_fw_info.u32Chipid,
	    (unsigned int)ww->ww_fw_info.u8FirmwareMajor,
	    (unsigned int)ww->ww_fw_info.u8FirmwareMinor,
	    (unsigned int)ww->ww_fw_info.u8FirmwarePatch,
	    (unsigned int)ww->ww_fw_info.u8DriverMajor,
	    (unsigned int)ww->ww_fw_info.u8DriverMinor,
	    (unsigned int)ww->ww_fw_info.u8DriverPatch,
	    ts.ts_date, ts.ts_time);

	char *name = NULL, *ssid = NULL;
	wifi_provision_enc_t enc;
	uint8_t enc8;
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
	if (cf_get_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE, &enc8) == 0)
		enc = WIFI_PROVISION_ENC_OPEN;
	else
		enc = (wifi_provision_enc_t)enc8;

	if (name != NULL) {
		fprintf(fp, "WiFi Device Name: %s\n", name);
		zone_free(name);
	}

	if (ssid != NULL) {
		fprintf(fp, "WiFi SSID: %s\n", ssid);
		zone_free(ssid);
	}

	const char *en;
	switch (enc) {
	case WIFI_PROVISION_ENC_OPEN:
		en = "Open";
		break;
	case WIFI_PROVISION_ENC_WPA:
		en = "WPA/WPA2";
		break;
	case WIFI_PROVISION_ENC_WEP:
		en = "WEP";
		break;
	case WIFI_PROVISION_ENC_802_1X:
		en = "802.1X";
		break;
	default:
		en = "Unknown";
		break;
	}
	fprintf(fp, "WiFi encryption: %s\n", en);
#ifdef WIFI_PROVISION_SUPPORT_SCAN
	if (ww->ww_channel != 0) {
		fprintf(fp, "WiFi channel: %" PRIu32 "\n",
		    (uint32_t)ww->ww_channel);

		if (!ww->ww_ap_enabled && ww->ww_link_status.ls_rssi < -1) {
			static const char *sig_str[] = {
			    "None", "Poor", "Fair", "Good", "Excellent"
			};
			network_signal_strength_t sig;
#if (RELEASE_BUILD == 0)
			char sig_avg[16];

			fix16_to_str(WILC_RSSI(ww), sig_avg, 2);
			sig = wilc_rssi_to_enum(ww->ww_link_status.ls_rssi);
			fprintf(fp, "WiFi RSSI: %s %s (%d)\n", sig_avg,
			    sig_str[sig], (int)ww->ww_link_status.ls_rssi);
#else
			sig = wilc_rssi_to_enum(ww->ww_link_status.ls_rssi);
			fprintf(fp, "WiFi Signal: %s\n", sig_str[sig]);
#endif
		}
	}
#endif /* WIFI_PROVISION_SUPPORT_SCAN */

#ifdef WILC_RECORD_STATS
	fprintf(fp, "  Tx: Sent %u, Deferred %u, Dropped %u\n      "
	    "Busy %u, Queued %u, Contig %u, Frag %u\n",
	    ww->ww_stat_tx_sent, ww->ww_stat_tx_deferred,
	    ww->ww_stat_tx_dropped, ww->ww_stat_tx_busy, ww->ww_stat_tx_queued,
	    ww->ww_stat_tx_contig, ww->ww_stat_tx_frag);
	fprintf(fp, "  Rx: Received %u, Dropped %u, NoBufs %u\n",
	    ww->ww_stat_rx_packets, ww->ww_stat_rx_dropped,
	    ww->ww_stat_rx_no_pbufs);
	fprintf(fp, "RSSI: Reqs %u, Replies %u, Early %u, Busy %u\n",
	    ww->ww_stat_rssi_reqs, ww->ww_stat_rssi_stats,
	    ww->ww_stat_rssi_early, ww->ww_stat_rssi_busy);
#endif
#if (RELEASE_BUILD == 0)
{
	extern unsigned int gCountOptMax;
	extern unsigned int gCountOptTotal;
	extern unsigned int gCountOptCalls;
	extern unsigned int gCountOptNonZero;
	extern unsigned int gCountOptErrors;

	extern unsigned int gCountMax;
	extern unsigned int gCountTotal;
	extern unsigned int gCountCalls;
	extern unsigned int gCountNonZero;
	extern unsigned int gCountErrors;

	fprintf(fp, "hif_send_optimized:\n\tCalls: %u, NonZero %u, Max %u, Avg %u, Errors %u\n",
	    gCountOptCalls, gCountOptNonZero, gCountOptMax, gCountOptTotal / gCountOptCalls, gCountOptErrors);
	fprintf(fp, "hif_send:\n\tCalls: %u, NonZero %u, Max %u, Avg %u, Errors %u\n",
	    gCountCalls, gCountNonZero, gCountMax, gCountTotal / gCountCalls, gCountErrors);
}

	fprintf(fp, "Tx TCP work-around: %s\n", ww->ww_tx_bogons ? "Enabled" :
	    "Disabled");
#endif
}
#endif /* WILC_OPT_COMMAND */

/*
 * These are the BSP functions required by Atmel's driver.
 */
void
nm_bsp_sleep(uint32 u32TimeMsec)
{

	rtos_task_sleep((rtos_tick_t) u32TimeMsec);
}

void
nm_bsp_register_isr(tpfNmBspIsr handler)
{
	struct wilc_wifi *ww = wilc_wifi_state;

	assert(ww != NULL);

	ww->ww_irq_handler = handler;
}

void
nm_bsp_interrupt_ctrl(uint8 u8Enable)
{
	struct wilc_wifi *ww = wilc_wifi_state;

	assert(ww != NULL);

	(ww->ww_control)(ww->ww_cookie, WILC_WIFI_IRQ_CTL, &u8Enable);
}

sint8
nm_bus_ioctl(uint8 cmd, void *arg)
{
	struct wilc_wifi *ww = wilc_wifi_state;
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
