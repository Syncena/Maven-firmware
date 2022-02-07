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
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "network.h"
#include "network_lwip.h"
#include "linked-lists.h"
#include "rtos.h"
#include "zone_alloc.h"

#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/err.h"
#include "lwip/dhcp.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/sys.h"

struct network_lwip_socket {
	network_sock_type_t ns_type;
	network_sock_status_callback_t ns_callback;
	void *ns_callback_arg;
	void *ns_pcb;
	struct tcp_pcb *ns_listen_pcb;
	struct pbuf *ns_pbuf_cur;
	struct pbuf *ns_pbuf_queue;
	ip_addr_t ns_remote_addr;
	u16_t ns_remote_port;
	uint16_t ns_rd_offset;
	int16_t ns_sock_state;
};
#define	NETWORK_LWIP_STATE_DEAD 	(-1)
#define	NETWORK_LWIP_STATE_INIT 	0
#define	NETWORK_LWIP_STATE_OPEN 	1
#define	NETWORK_LWIP_STATE_LISTEN 	2
#define	NETWORK_LWIP_STATE_CONNECTED 	3

struct network_lwip_state {
	struct network_driver nls_net_driver;
	struct network_state *nls_network_state;
	struct network_lwip_driver *nls_hw_driver;
	uint8_t nls_nsockets;
	uint8_t nls_dhcp_client_enabled;
	uint8_t nls_link_state;
	netif_ext_callback_t nls_ext_callback;
	network_link_status_callback_t nls_link_status_cb;
	void *nls_link_status_cb_arg;
};

static struct network_lwip_socket *
network_lwip_new_socket(struct network_lwip_state *nls,
    network_sock_type_t t, network_sock_status_callback_t cb, void *cb_arg)
{
	struct network_lwip_socket *ns;

	if ((ns = zone_calloc(1, sizeof(*ns))) == NULL)
		return NULL;

	ns->ns_type = t;
	ns->ns_callback = cb;
	ns->ns_callback_arg = cb_arg;
	ns->ns_sock_state = NETWORK_LWIP_STATE_INIT;

	nls->nls_nsockets++;

	return ns;
}

static void
network_lwip_free_socket(struct network_lwip_state *nls,
    struct network_lwip_socket *ns)
{

	if (ns->ns_pbuf_cur != NULL) {
		pbuf_free(ns->ns_pbuf_cur);
		ns->ns_pbuf_cur = NULL;
	}

	if (ns->ns_pbuf_queue != NULL) {
		pbuf_free(ns->ns_pbuf_queue);
		ns->ns_pbuf_queue = NULL;
	}

	if (ns->ns_pcb != NULL) {
		if (ns->ns_type == NETWORK_SOCK_TYPE_TCP) {
			err_t err = tcp_close((struct tcp_pcb *)ns->ns_pcb);
			assert(err == ERR_OK);
			(void)err;
			tcp_arg(ns->ns_pcb, NULL);
		} else {
			udp_remove((struct udp_pcb *)ns->ns_pcb);
		}
		ns->ns_pcb = NULL;
	}

	if (ns->ns_listen_pcb != NULL) {
		err_t err = tcp_close(ns->ns_listen_pcb);
		assert(err == ERR_OK);
		(void)err;
		ns->ns_listen_pcb = NULL;
	}

	zone_free(ns);

	assert(nls->nls_nsockets != 0);
	nls->nls_nsockets--;
}

_Static_assert(SYS_TIMEOUTS_SLEEPTIME_INFINITE == RTOS_MAX_SLEEP_MS,
    "SYS_TIMEOUTS_SLEEPTIME_INFINITE != RTOS_MAX_SLEEP_MS");

static void
network_lwip_worker(void *arg)
{
	struct network_lwip_state *nls = arg;
	struct network_lwip_driver *nld = nls->nls_hw_driver;
	uint32_t next_lwip_poll, next_poll;

	do {
		/*
		 * Handle interrupt-related work for the hardware.
		 */
		next_poll = nld->nld_handle_interrupt(nld->nld_netif);

		/*
		 * Process any pending LwIP timeouts.
		 */
		if ((next_lwip_poll = sys_timeouts_sleeptime()) == 0) {
			sys_check_timeouts();
			next_lwip_poll = sys_timeouts_sleeptime();
			next_poll = 0;
		}
	} while (next_poll == 0 || next_lwip_poll == 0);

	/*
	 * Calculate earliest (non-interrupt) wakeup.
	 */
	if (next_lwip_poll < next_poll)
		next_poll = next_lwip_poll;

	/*
	 * Schedule a soft timeout, if necessary.
	 */
	if (next_poll != RTOS_MAX_SLEEP_MS)
		network_driver_soft_timeout(nls->nls_network_state, next_poll);
}

static void
network_lwip_link_check(void *arg)
{
	struct network_lwip_state *nls = arg;
	struct network_lwip_driver *nld = nls->nls_hw_driver;

	nld->nld_link_check(nld->nld_netif);
}

static void
network_lwip_link_status(void *arg, network_link_status_callback_t cb,
    void *cb_arg)
{
	struct network_lwip_state *nls = arg;

	nls->nls_link_status_cb = cb;
	nls->nls_link_status_cb_arg = cb_arg;
}

static void
network_lwip_ext_callback(struct netif* netif, netif_nsc_reason_t reason,
    const netif_ext_callback_args_t* args)
{
	network_ip_params_t ip;

	(void) args;

	if ((reason & LWIP_NSC_IPV4_SETTINGS_CHANGED) == 0)
		return;

	if (ip4_addr_isany(&netif->ip_addr))
		return;

	ip.ip_addr = ntoh32(netif->ip_addr.addr);
	ip.ip_mask = ntoh32(netif->netmask.addr);
	ip.ip_gate = ntoh32(netif->gw.addr);
#ifdef NETWORK_OPT_SNTP_CLIENT
	ip.ip_sntp = 0;
#endif

	network_set_ip_params_drv(&ip);
}

static void
network_lwip_dhcp_start(struct network_lwip_state *nls)
{

	if (nls->nls_dhcp_client_enabled)
		return;

	nls->nls_dhcp_client_enabled = 1;
	netif_add_ext_callback(&nls->nls_ext_callback,
	    network_lwip_ext_callback);

	dhcp_start(nls->nls_hw_driver->nld_netif);
}

static void
network_lwip_dhcp_stop(struct network_lwip_state *nls)
{

	if (nls->nls_dhcp_client_enabled == 0)
		return;

	nls->nls_dhcp_client_enabled = 0;
	netif_remove_ext_callback(&nls->nls_ext_callback);

	dhcp_stop(nls->nls_hw_driver->nld_netif);
}

static void
network_lwip_link_status_cb(const network_link_status_t *ls, void *arg)
{
	struct network_lwip_state *nls = arg;

	if (nls->nls_link_status_cb != NULL)
		(nls->nls_link_status_cb)(ls, nls->nls_link_status_cb_arg);

	if (nls->nls_link_state == ls->ls_state)
		return;

	if (ls->ls_state != NETWORK_LINK_DOWN &&
	    ls->ls_state != NETWORK_LINK_UP) {
		return;
	}

	nls->nls_link_state = ls->ls_state;

	if (ls->ls_state == NETWORK_LINK_UP)
		network_lwip_dhcp_start(nls);
	else
		network_lwip_dhcp_stop(nls);
}

static void
network_lwip_get_mac(void *arg, uint8_t *mac)
{
	struct network_lwip_state *nls = arg;
	struct network_lwip_driver *nld = nls->nls_hw_driver;

	nld->nld_get_mac(nld->nld_netif, mac);
}

static int8_t
network_lwip_ioctl(void *arg, uint8_t op, void *oparg)
{
	struct network_lwip_state *nls = arg;
	struct network_lwip_driver *nld = nls->nls_hw_driver;
	struct network_lwip_socket *ns;
	int8_t rv = 0;

	switch (op) {
	case NETWORK_IOCTL_DHCP_CLIENT_START:
		network_lwip_dhcp_start(nls);
		break;

	case NETWORK_IOCTL_DHCP_CLIENT_STOP:
		network_lwip_dhcp_stop(nls);
		break;

	case NETWORK_IOCTL_TCP_NAGLE_OFF:
		ns = oparg;
		if (ns->ns_pcb != NULL)
			tcp_nagle_disable((struct tcp_pcb *)ns->ns_pcb);
		break;

	case NETWORK_IOCTL_TCP_NAGLE_ON:
		ns = oparg;
		if (ns->ns_pcb != NULL)
			tcp_nagle_enable((struct tcp_pcb *)ns->ns_pcb);
		break;

#if (LWIP_STATS != 0)
	case NETWORK_IOCTL_STATS:
		stats_display();
		if (nld->nld_ioctl != NULL)
			rv = nld->nld_ioctl(nld->nld_netif, op, oparg);
		break;
#endif

	default:
		if (nld->nld_ioctl != NULL)
			rv = nld->nld_ioctl(nld->nld_netif, op, oparg);
		break;
	}

	return rv;
}

static void
network_lwip_rx_pbuf(struct network_lwip_socket *ns, struct pbuf *p)
{

	if (ns->ns_pbuf_cur == NULL) {
		assert(ns->ns_pbuf_queue == NULL);
		ns->ns_pbuf_cur = p;
	} else {
		if (ns->ns_pbuf_queue == NULL)
			ns->ns_pbuf_queue = p;
		else
			pbuf_chain(ns->ns_pbuf_queue, p);
	}
}

static void
network_lwip_udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port)
{
	struct network_lwip_socket *ns = arg;

	(void) pcb;

	if (ns == NULL)
		return;

#if (RELEASE_BUILD == 0)
if (ns->ns_type != NETWORK_SOCK_TYPE_UDP ||
    ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED ||
    ns->ns_pcb != pcb) {
	printf("udp_recv: pcb %p, pbuf %p, ns_type %u, ns_sock_state %" PRId16
	    ", ns_pcb %p\n", (void *)pcb, (void *)p,
	    (unsigned int)ns->ns_type, ns->ns_sock_state, (void *)ns->ns_pcb);
}
#endif

	assert(ns->ns_type == NETWORK_SOCK_TYPE_UDP);
	assert(ns->ns_pcb == pcb);
	assert(ns->ns_sock_state == NETWORK_LWIP_STATE_CONNECTED);

	if (p == NULL) {
		ns->ns_callback(NETWORK_STATUS_DISCONNECT, ns->ns_callback_arg);
		ns->ns_sock_state = NETWORK_LWIP_STATE_DEAD;
		return;
	}

	network_lwip_rx_pbuf(ns, p);

	ns->ns_remote_addr = *addr;
	ns->ns_remote_port = port;

	ns->ns_callback(NETWORK_STATUS_RX_READY, ns->ns_callback_arg);
}

static int
network_lwip_open_tcp(struct network_lwip_socket *ns)
{
	struct tcp_pcb *pcb;

	if ((pcb = tcp_new_ip_type(IPADDR_TYPE_ANY)) == NULL)
		return -1;

	ns->ns_pcb = (void *)pcb;
	return 0;
}

static int
network_lwip_open_udp(struct network_lwip_socket *ns)
{
	struct udp_pcb *pcb;

	if ((pcb = udp_new_ip_type(IPADDR_TYPE_ANY)) == NULL)
		return -1;

	ns->ns_pcb = (void *)pcb;
	udp_recv(pcb, network_lwip_udp_recv_callback, ns);
	return 0;
}

static network_sock_t
network_lwip_open(void *cookie, network_sock_type_t stype,
    network_sock_status_callback_t func, void *arg)
{
	struct network_lwip_state *nls = cookie;
	struct network_lwip_socket *ns;
	int rv;

	ns = network_lwip_new_socket(nls, stype, func, arg);
	if (ns == NULL)
		return NULL;

	if (stype == NETWORK_SOCK_TYPE_TCP)
		rv = network_lwip_open_tcp(ns);
	else
		rv = network_lwip_open_udp(ns);

	if (rv < 0) {
		network_lwip_free_socket(nls, ns);
		ns = NULL;
	} else {
		ns->ns_sock_state = NETWORK_LWIP_STATE_OPEN;
	}

	return (network_sock_t)ns;
}

static void
network_lwip_close(void *cookie, network_sock_t sock)
{
	struct network_lwip_state *nls = cookie;
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;

	network_lwip_free_socket(nls, ns);
}

static uint8_t
network_lwip_connect(void *cookie, network_sock_t sock,
    const network_sock_params_t *sp)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	ip_addr_t dest;

	(void) cookie;

	/* XXX: UDP only for the time being. */
	assert(ns->ns_type == NETWORK_SOCK_TYPE_UDP);

	if (udp_bind((struct udp_pcb *)ns->ns_pcb, IP_ANY_TYPE,
	    sp->sp_sport) != ERR_OK) {
		return 0;
	}

	dest.addr = lwip_htonl(sp->sp_dest_ip);
	if (udp_connect((struct udp_pcb *)ns->ns_pcb, &dest,
	    sp->sp_dport) != ERR_OK) {
		return 0;
	}

	ns->ns_sock_state = NETWORK_LWIP_STATE_CONNECTED;

	return 1;
}

static err_t
network_lwip_tcp_recv_callback(void *arg, struct tcp_pcb *pcb,
    struct pbuf *p, err_t err)
{
	struct network_lwip_socket *ns = arg;
	network_sock_status_t st;
	err_t rv;

	(void) pcb;

	if (ns == NULL)
		return ERR_OK;

#if (RELEASE_BUILD == 0)
if (ns->ns_type != NETWORK_SOCK_TYPE_TCP ||
    ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED ||
    ns->ns_pcb != pcb) {
	printf("tcp_recv: pcb %p, pbuf %p, err %u, ns_type %u, ns_sock_state "
	    "%" PRId16 ", ns_pcb %p\n", (void *)pcb, (void *)p, err,
	    (unsigned int)ns->ns_type, ns->ns_sock_state, (void *)ns->ns_pcb);
}
#endif

	assert(ns->ns_type == NETWORK_SOCK_TYPE_TCP);
	assert(ns->ns_sock_state == NETWORK_LWIP_STATE_CONNECTED);
	assert(ns->ns_pcb == pcb);

	if (err != ERR_OK) {
		ns->ns_sock_state = NETWORK_LWIP_STATE_DEAD;
		return ERR_OK;
	}

	if (p == NULL) {
		if (ns->ns_pbuf_cur != NULL) {
			pbuf_free(ns->ns_pbuf_cur);
			ns->ns_pbuf_cur = NULL;
		}
		if (ns->ns_pbuf_queue != NULL) {
			pbuf_free(ns->ns_pbuf_queue);
			ns->ns_pbuf_queue = NULL;
		}
		ns->ns_sock_state = NETWORK_LWIP_STATE_DEAD;
		st = NETWORK_STATUS_DISCONNECT;
		rv = ERR_ABRT;
	} else {
		network_lwip_rx_pbuf(ns, p);
		st = NETWORK_STATUS_RX_READY;
		rv = ERR_OK;
	}

	ns->ns_callback(st, ns->ns_callback_arg);

	return rv;
}

static err_t
network_lwip_tcp_sent_callback(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	struct network_lwip_socket *ns = arg;

	(void) pcb;
	(void) len;

	if (ns == NULL)
		return ERR_OK;

#if (RELEASE_BUILD == 0)
if (ns->ns_type != NETWORK_SOCK_TYPE_TCP ||
    ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED ||
    ns->ns_pcb != pcb) {
	printf("tcp_sent: pcb %p, ns_type %u, ns_sock_state %" PRId16
	    ", ns_pcb %p\n", (void *)pcb, (unsigned int)ns->ns_type,
	    ns->ns_sock_state, (void *)ns->ns_pcb);
}
#endif

	assert(ns->ns_type == NETWORK_SOCK_TYPE_TCP);
	assert(ns->ns_sock_state == NETWORK_LWIP_STATE_CONNECTED);
	assert(ns->ns_pcb == pcb);

	ns->ns_callback(NETWORK_STATUS_TX_DONE, ns->ns_callback_arg);

	return ERR_OK;
}

static void
network_lwip_tcp_err_callback(void *arg, err_t err)
{
	struct network_lwip_socket *ns = arg;
	network_sock_status_t st;

#if (RELEASE_BUILD == 0)
	printf("network_lwip_tcp_err_callback: ns %p, err %d\n", (void *)ns,
	    err);
#endif

	if (ns == NULL)
		return;

	assert(ns->ns_type == NETWORK_SOCK_TYPE_TCP);

	if (ns->ns_pbuf_cur != NULL) {
		pbuf_free(ns->ns_pbuf_cur);
		ns->ns_pbuf_cur = NULL;
	}
	if (ns->ns_pbuf_queue != NULL) {
		pbuf_free(ns->ns_pbuf_queue);
		ns->ns_pbuf_queue = NULL;
	}
	ns->ns_sock_state = NETWORK_LWIP_STATE_DEAD;

	if (err == ERR_ABRT)
		st = NETWORK_STATUS_TIMEOUT;
	else
		st = NETWORK_STATUS_DISCONNECT;

	ns->ns_callback(st, ns->ns_callback_arg);
}

static err_t
network_lwip_tcp_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	struct network_lwip_socket *ns = arg;

	(void) err;

	if (ns == NULL)
		return ERR_ABRT;

	if (newpcb == NULL) {
		/* TCP stack could not allocate new PCB */
		return ERR_OK;
	}

	/*
	 * If the socket is anything other than listening,
	 * then we don't accept this connection.
	 */
	if (ns->ns_sock_state != NETWORK_LWIP_STATE_LISTEN)
		return ERR_ISCONN;

#if (RELEASE_BUILD == 0)
if (ns->ns_type != NETWORK_SOCK_TYPE_TCP ||
    ns->ns_pcb != NULL || ns->ns_pbuf_cur != NULL) {
	printf("tcp_accept: newpcb %p, err %u, ns_type %u, ns_pcb %p, "
	    "pbuf_cur %p\n", (void *)newpcb, err, ns->ns_type,
	    (void *)ns->ns_pcb, (void *)ns->ns_pbuf_cur);
}
#endif

	assert(ns->ns_type == NETWORK_SOCK_TYPE_TCP);
	assert(ns->ns_pcb == NULL);
	assert(ns->ns_pbuf_cur == NULL);

#if (LWIP_TCP_KEEPALIVE != 0)
	ip_set_option(newpcb, SOF_KEEPALIVE);
#endif

	ns->ns_pcb = newpcb;
	ns->ns_sock_state = NETWORK_LWIP_STATE_CONNECTED;

	tcp_arg(newpcb, ns);
	tcp_recv(newpcb, network_lwip_tcp_recv_callback);
	tcp_sent(newpcb, network_lwip_tcp_sent_callback);
	tcp_err(newpcb, network_lwip_tcp_err_callback);

	ns->ns_callback(NETWORK_STATUS_CONNECT, ns->ns_callback_arg);

	return ERR_OK;
}

static uint8_t
network_lwip_listen(void *cookie, network_sock_t sock, uint16_t sport)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;

	(void) cookie;

#if SO_REUSE
	ip_set_option((struct tcp_pcb *)ns->ns_pcb, SOF_REUSEADDR);
#endif

	if (tcp_bind((struct tcp_pcb *)ns->ns_pcb, IP_ANY_TYPE,sport) != ERR_OK)
		return 0;

	ns->ns_listen_pcb = tcp_listen((struct tcp_pcb *)ns->ns_pcb);
	if (ns->ns_listen_pcb == NULL)
		return 0;

	ns->ns_pcb = NULL;
	ns->ns_sock_state = NETWORK_LWIP_STATE_LISTEN;

	tcp_arg((struct tcp_pcb *)ns->ns_listen_pcb, ns);
	tcp_accept((struct tcp_pcb *)ns->ns_listen_pcb,
	    network_lwip_tcp_accept_callback);

	return 1;
}

static uint16_t
network_lwip_read(void *cookie, network_sock_t sock, void *buff, uint16_t len)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	struct pbuf *p;
	uint16_t this_len, copied_len = 0;
	uint8_t *cbuff = buff;

	(void) cookie;

	if (ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED)
		return NETWORK_SOCK_ERR_EOF;

	p = ns->ns_pbuf_cur;

	while (len && p != NULL) {
		this_len = p->len - ns->ns_rd_offset;
		if (this_len > len)
			this_len = len;

		memcpy(cbuff, (uint8_t *)p->payload + ns->ns_rd_offset,
		    this_len);

		cbuff += this_len;
		len -= this_len;
		copied_len += this_len;
		ns->ns_rd_offset += this_len;

		if (ns->ns_rd_offset == p->len) {
			ns->ns_rd_offset = 0;
			ns->ns_pbuf_cur = NULL;
			pbuf_free(p);
			if ((p = ns->ns_pbuf_queue) != NULL) {
				ns->ns_pbuf_queue = pbuf_dechain(p);
				ns->ns_pbuf_cur = p;
			}
		}
	}

	if (ns->ns_type == NETWORK_SOCK_TYPE_TCP && copied_len)
		tcp_recved((struct tcp_pcb *)ns->ns_pcb, copied_len);

	return copied_len;
}

static uint16_t
network_lwip_recv(void *cookie, network_sock_t sock, void *buff, uint16_t len,
    network_sock_params_t *sp)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	uint16_t rv;

	(void) cookie;

	assert(ns->ns_type == NETWORK_SOCK_TYPE_UDP);

	rv = network_lwip_read(cookie, sock, buff, len);
	if (rv == 0 || NETWORK_SOCK_ERR(rv))
		return rv;

	if (sp != NULL) {
		sp->sp_dest_ip = lwip_ntohl(ns->ns_remote_addr.addr);
		sp->sp_dport = ns->ns_remote_port;
	}

	return rv;
}

static uint16_t
network_lwip_rx_available(void *cookie, network_sock_t sock)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	uint16_t rv;

	(void) cookie;

	if (ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	rv = 0;

	if (ns->ns_pbuf_cur != NULL)
		rv += ns->ns_pbuf_cur->len - ns->ns_rd_offset;

	if (ns->ns_pbuf_queue != NULL)
		rv += ns->ns_pbuf_queue->tot_len;

	return rv;
}

static uint16_t
network_lwip_tcp_write(struct network_lwip_socket *ns, const void *buff,
    uint16_t len)
{
	uint16_t snd;
	err_t err;

	if (len == 0) {
		if (tcp_output((struct tcp_pcb *)ns->ns_pcb) != ERR_OK)
			return NETWORK_SOCK_ERR_EOF;
		return 0;
	}

	snd = (uint16_t)tcp_sndbuf((struct tcp_pcb *)ns->ns_pcb);
	if (len > snd)
		len = snd;

	err = tcp_write((struct tcp_pcb *)ns->ns_pcb, buff, len,
	    TCP_WRITE_FLAG_COPY);
	if (err != ERR_OK) {
		if (err == ERR_MEM)
			len = 0;
		else
			return NETWORK_SOCK_ERR_EOF;
	}

	return len;
}

static uint16_t
network_lwip_udp_write(struct network_lwip_socket *ns, const void *buff,
    uint16_t len, const uint8_t *dstmac)
{
	struct pbuf *p;
	err_t err;

	(void) dstmac;

	if (len == 0)
		return 0;

	if ((p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM)) == NULL)
		return 0;

	memcpy(p->payload, buff, len);

	err = udp_send((struct udp_pcb *)ns->ns_pcb, p);
	if (err != ERR_OK)
		pbuf_free(p);

	return (err == ERR_OK) ? len : NETWORK_SOCK_ERR_EOF;
}

static uint16_t
network_lwip_write(void *cookie, network_sock_t sock, const void *buff,
    uint16_t len, const uint8_t *dstmac)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	uint16_t rv;

	(void) cookie;

	if (ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	if (ns->ns_type == NETWORK_SOCK_TYPE_TCP) {
		assert(dstmac == NULL);
		rv = network_lwip_tcp_write(ns, buff, len);
	} else {
		rv = network_lwip_udp_write(ns, buff, len, dstmac);
	}

	return rv;
}

static uint16_t
network_lwip_tx_free(void *cookie, network_sock_t sock)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;

	(void) cookie;

	assert(ns->ns_type == NETWORK_SOCK_TYPE_TCP);

	if (ns->ns_sock_state != NETWORK_LWIP_STATE_CONNECTED)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	return (uint16_t)tcp_sndbuf((struct tcp_pcb *)ns->ns_pcb);
}

static network_sock_state_t
network_lwip_sock_status(void *cookie, network_sock_t sock,
    network_sock_params_t *sp)
{
	struct network_lwip_socket *ns = (struct network_lwip_socket *)sock;
	network_sock_state_t ss;

	(void) cookie;

	if (ns->ns_sock_state < NETWORK_LWIP_STATE_LISTEN)
		return NETWORK_SOCKS_NONE;

	if (ns->ns_type == NETWORK_SOCK_TYPE_TCP) {
		struct tcp_pcb *pcb;

		if (ns->ns_sock_state == NETWORK_LWIP_STATE_LISTEN) {
			assert(ns->ns_listen_pcb != NULL);
			pcb = ns->ns_listen_pcb;
			sp->sp_dport = 0;
			sp->sp_dest_ip = IPADDR_ANY;
			ss = NETWORK_SOCKS_LISTEN;
		} else {
			assert(ns->ns_pcb != NULL);
			pcb = ns->ns_pcb;
			sp->sp_dport = pcb->remote_port;
			sp->sp_dest_ip = ntoh32(pcb->remote_ip.addr);
			ss = NETWORK_SOCKS_ESTABLISHED;
		}

		sp->sp_sport = pcb->local_port;
		sp->sp_local_ip = ntoh32(pcb->local_ip.addr);
	} else {
		struct udp_pcb *pcb = ns->ns_pcb;

		sp->sp_sport = pcb->local_port;
		sp->sp_dport = pcb->remote_port;
		if (ns->ns_remote_addr.addr != 0)
			sp->sp_dest_ip = lwip_ntohl(ns->ns_remote_addr.addr);
		else
			sp->sp_dest_ip = ntoh32(pcb->remote_ip.addr);
		sp->sp_local_ip = 0;
		ss = NETWORK_SOCKS_UDP;
	}

	return ss;
}

static void
network_lwip_set_ip_params(void *cookie, const network_ip_params_t *ip)
{
	struct network_lwip_state *nls = cookie;
	ip4_addr_t addr, mask, gw;

	if (nls->nls_dhcp_client_enabled)
		return;

	addr.addr = hton32(ip->ip_addr);
	mask.addr = hton32(ip->ip_mask);
	gw.addr = hton32(ip->ip_gate);
	netif_set_addr(nls->nls_hw_driver->nld_netif, &addr, &mask, &gw);
}

struct network_state *
network_lwip_attach(struct network_lwip_driver *nld)
{
	struct network_lwip_state *nls;

#if (LWIP_STATS != 0)
	stats_init();
#endif

	if ((nls = zone_malloc(sizeof(*nls))) == NULL)
		return NULL;

	nls->nls_net_driver.nd_cookie = nls;
	nls->nls_net_driver.nd_worker = network_lwip_worker;
	if (nld->nld_link_check != NULL)
		nls->nls_net_driver.nd_link_check = network_lwip_link_check;
	else
		nls->nls_net_driver.nd_link_check = NULL;
	nls->nls_net_driver.nd_link_status = network_lwip_link_status;
	nls->nls_net_driver.nd_get_mac = network_lwip_get_mac;
	nls->nls_net_driver.nd_ioctl = network_lwip_ioctl;
	nls->nls_net_driver.nd_open = network_lwip_open;
	nls->nls_net_driver.nd_close = network_lwip_close;
	nls->nls_net_driver.nd_connect = network_lwip_connect;
	nls->nls_net_driver.nd_listen = network_lwip_listen;
	nls->nls_net_driver.nd_read = network_lwip_read;
	nls->nls_net_driver.nd_recv = network_lwip_recv;
	nls->nls_net_driver.nd_rx_avail = network_lwip_rx_available;
	nls->nls_net_driver.nd_write = network_lwip_write;
	nls->nls_net_driver.nd_tx_free = network_lwip_tx_free;
	nls->nls_net_driver.nd_sock_status = network_lwip_sock_status;
	nls->nls_net_driver.nd_set_ip_params = network_lwip_set_ip_params;

	nls->nls_hw_driver = nld;
	nls->nls_nsockets = 0;
	nls->nls_dhcp_client_enabled = 0;
	nls->nls_link_state = ~0;

	nls->nls_network_state = network_register(&nls->nls_net_driver);

	if (nls->nls_network_state == NULL) {
		printf("network_lwip_attach: bogons at 12 o'clock!\n");
		zone_free(nls);
		return NULL;
	}

	nld->nld_link_status(nld->nld_netif, network_lwip_link_status_cb, nls);

	return nls->nls_network_state;
}

/*
 * Service functions for LWIP itself.
 */
void
sys_init(void)
{
}

u32_t
sys_now(void)
{

	return (u32_t) xTaskGetTickCount();
}

u32_t
sys_jiffies(void)
{

	return sys_now();
}
