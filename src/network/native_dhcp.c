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

#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "rtos.h"
#include "network.h"
#include "native_dhcp.h"
#include "timer.h"
#include "hexdump.h"

#ifdef DHCP_OPT_COMMAND
#include "shell.h"
SHELL_CMD_DECL(dhcp, dhcp_cmd, "Displays/renews DHCP lease");
#endif /* DHCP_OPT_COMMAND */

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	dhcp_debug_flag
#include "debug.h"

#define DHCP_STATE_IDLE		0
#define DHCP_STATE_INIT		1
#define DHCP_STATE_START	2
#define DHCP_STATE_DISCOVER	3
#define DHCP_STATE_REQUEST	4
#define DHCP_STATE_DECLINE	5
#define DHCP_STATE_LEASED	6
#define DHCP_STATE_RENEW	7

#define BOOTP_BROADCAST		0x8000

#define DHCP_DISCOVER		1
#define DHCP_REPLY		2
#define DHCP_OFFER		2
#define DHCP_REQUEST		3
#define DHCP_DECLINE		2
#define DHCP_ACK		5
#define DHCP_NACK		6
#define DHCP_RELEASE		7

#define DHCP_HTYPE_ETHERNET	1
#define DHCP_HLEN_ETHERNET	6
#define DHCP_MSG_LEN		236

#define DHCP_SERVER_PORT	67
#define DHCP_CLIENT_PORT	68

#define DHCP_OPTION_SUBNET_MASK	1
#define DHCP_OPTION_ROUTER	3
#define DHCP_OPTION_REQ_IPADDR	50
#define DHCP_OPTION_LEASE_TIME	51
#define DHCP_OPTION_MSG_TYPE	53
#define DHCP_OPTION_SERVER_ID	54
#define DHCP_OPTION_REQ_LIST	55
#define DHCP_OPTION_T1		58
#define DHCP_OPTION_T2		59
#define DHCP_OPTION_NTP_SERVERS	42
#define DHCP_OPTION_END		255

#define DHCP_BUFFER_SIZE	576	/* !!!! */

#define	mymin(a,b)		(((a) < (b)) ? (a) : (b))

struct dhcp_msg {
	uint8_t dm_op;
	uint8_t dm_htype;
	uint8_t dm_hlen;
	uint8_t dm_hops;
	uint32_t dm_xid;
	uint16_t dm_secs;
	uint16_t dm_flags;
	uint32_t dm_ciaddr;
	uint32_t dm_yiaddr;
	uint32_t dm_siaddr;
	uint32_t dm_giaddr;
	uint8_t dm_chaddr[16];
	uint8_t dm_servname[64];
	uint8_t dm_bootfile[128];
	uint32_t dm_cookie;
	uint8_t dm_options[];
};

struct dhcp_offer {
	uint32_t do_addr;
	uint32_t do_mask;
	uint32_t do_gate;
	uint32_t do_ntp_server;
	uint32_t do_lease;
	uint32_t do_t1;
	uint32_t do_t2;
	uint32_t do_server_ip;
	uint32_t do_server_id;
};

struct dhcp_state {
	struct dhcp_offer ds_offer;
	uint32_t ds_xid;
	uint8_t ds_state;
	uint8_t ds_link_state;
	uint8_t ds_rx_ready;
	uint8_t ds_backoff;
	uint8_t ds_hook_cookie;
	volatile uint8_t ds_running;
	uint8_t ds_initialised;
	void *ds_sock;
	rtos_task_t ds_task;
	network_sock_params_t ds_peer;	/* Peer we're currently talking to */
	uint32_t ds_peer_xid;		/* Peer's XID (optional) */
	time_t ds_start_time;
	uint32_t ds_lease_mono_time;
	uint32_t ds_renew_time;
	uint32_t ds_rebind_time;
	union {
		uint32_t uds_dummy;
		uint8_t uds_buff[DHCP_BUFFER_SIZE];
	} ds_u;
};
#define	ds_msg_buff	ds_u.uds_buff
#define	DHCP_MSG(ds)	((struct dhcp_msg *)(void *)&(ds)->ds_msg_buff)
static struct dhcp_state dhcp_state;

static void dhcp_task(void *);
#ifndef DHCP_STACK_SIZE
#define	DHCP_STACK_SIZE	1000
#endif

static void
dhcp_link_status_cb(const network_link_status_t *ls, void *arg)
{
	struct dhcp_state *ds = arg;

	DBFPRINTF("ls_state %u, ds_state %u\n", ls->ls_state,
	    ds->ds_link_state);

	if (ls->ls_state == ds->ds_link_state)
		return;
	ds->ds_link_state = ls->ls_state;

	switch (ls->ls_state) {
	case NETWORK_LINK_UP:
		ds->ds_state = DHCP_STATE_INIT;
		rtos_notify_give(ds->ds_task);
		break;

	case NETWORK_LINK_DOWN:
		if (ds->ds_sock != NULL) {
			network_close(ds->ds_sock);
			ds->ds_sock = NULL;
		}
		ds->ds_state = DHCP_STATE_IDLE;
		break;
	}
}

void
native_dhcp_init(void)
{
	struct dhcp_state *ds = &dhcp_state;

	DBFPRINTF("current running state is %u\n", ds->ds_running);

	if (ds->ds_running)
		return;

	if (ds->ds_initialised == 0) {
		DBFPRINTF("starting dhcp_task\n");
		ds->ds_task = rtos_task_create("DHCP", dhcp_task, ds,
		    RTOS_TASK_PRIORITY_LOW, RTOS_STACK_SIZE(DHCP_STACK_SIZE));
		assert(ds->ds_task != NULL);

		ds->ds_hook_cookie =
		    network_hook_link_status(dhcp_link_status_cb, ds);
#ifdef DHCP_OPT_COMMAND
		SHELL_CMD_ADD(dhcp);
#endif
		ds->ds_initialised = 1;
	}

	DBFPRINTF("notifying task that we're running\n");
	ds->ds_running = 1;
	rtos_notify_give(ds->ds_task);
}

void
native_dhcp_deinit(void)
{
	struct dhcp_state *ds = &dhcp_state;

	if (ds->ds_initialised == 0)
		return;

	DBFPRINTF("current running state is %u\n", ds->ds_running);

	ds->ds_running = 0;
	rtos_notify_give(ds->ds_task);
}

static void
dhcp_socket_cb(network_sock_status_t st, void *arg)
{
	struct dhcp_state *ds = arg;

	switch (st) {
	case NETWORK_STATUS_TIMEOUT:
	case NETWORK_STATUS_UNREACHABLE:
		DBFPRINTF("err %u\n", st);
		if (ds->ds_sock != NULL) {
			network_close(ds->ds_sock);
			ds->ds_sock = NULL;
		}
		ds->ds_state = DHCP_STATE_INIT;
		break;

	case NETWORK_STATUS_RX_READY:
		DBFPRINTF("Rx ready\n");
		ds->ds_rx_ready = 1;
		rtos_notify_give(ds->ds_task);
		break;

	default:
		break;
	}
}

static uint8_t *
dhcp_addmsgtype(uint8_t *optptr, uint8_t type)
{

	*optptr++ = DHCP_OPTION_MSG_TYPE;
	*optptr++ = 1;
	*optptr++ = type;

	return optptr;
}

static uint8_t *
dhcp_addserverip(uint32_t serverip, uint8_t *optptr)
{
	uint32_t naddr;

	naddr = hton32(serverip);

	*optptr++ = DHCP_OPTION_SERVER_ID;
	*optptr++ = 4;
	memcpy(optptr, &naddr, 4);

	return optptr + 4;
}

static uint8_t *
dhcp_addreqipaddr(uint32_t ipaddr, uint8_t *optptr)
{
	uint32_t naddr;

	naddr = hton32(ipaddr);

	*optptr++ = DHCP_OPTION_REQ_IPADDR;
	*optptr++ = 4;
	memcpy(optptr, &naddr, 4);

	return optptr + 4;
}

static uint8_t *
dhcp_addreqoptions(uint8_t *optptr)
{

	*optptr++ = DHCP_OPTION_REQ_LIST;
	*optptr++ = 3;
	*optptr++ = DHCP_OPTION_SUBNET_MASK;
	*optptr++ = DHCP_OPTION_ROUTER;
	*optptr++ = DHCP_OPTION_LEASE_TIME;

	return optptr;
}

static uint8_t *
dhcp_addreqntpservers(uint8_t *optptr)
{

	*optptr++ = DHCP_OPTION_NTP_SERVERS;
	*optptr++ = 4;
	*optptr++ = 0;
	*optptr++ = 0;
	*optptr++ = 0;
	*optptr++ = 0;

	return optptr;
}

static uint8_t *
dhcp_addend(uint8_t *optptr)
{

	*optptr++ = DHCP_OPTION_END;

	return optptr;
}

static int8_t
dhcp_open_socket(struct dhcp_state *ds)
{
	network_sock_params_t sp;

	if (ds->ds_sock != NULL)
		network_close(ds->ds_sock);

	sp.sp_dest_ip = ds->ds_peer.sp_dest_ip;
	sp.sp_dport = DHCP_SERVER_PORT;
	sp.sp_sport = DHCP_CLIENT_PORT;

	ds->ds_sock = network_socket_udp(&sp, dhcp_socket_cb, ds);
	if (ds->ds_sock == NETWORK_SOCKET_INVALID)
		return -1;

	return 0;
}

static int8_t
dhcp_send(struct dhcp_state *ds, uint8_t mtype)
{
	struct dhcp_msg *dm;
	uint8_t bcast_mac[6], *mac;
	uint16_t len;
	uint8_t *next_op;
	time_t t;

	time(&t);
	dm = DHCP_MSG(ds);

	dm->dm_op = mtype;
	dm->dm_htype = DHCP_HTYPE_ETHERNET;
	dm->dm_hlen = DHCP_HLEN_ETHERNET;
	dm->dm_xid = hton32(ds->ds_xid);
	dm->dm_secs = (uint16_t)(t - ds->ds_start_time);
	dm->dm_flags = 0;
	network_get_mac(dm->dm_chaddr);

#ifdef DEBUG_ENABLED
	if (DEBUG_FLAG &&
	    dm->dm_chaddr[0] == 0 && dm->dm_chaddr[1] == 0 &&
	    dm->dm_chaddr[2] == 0 && dm->dm_chaddr[3] == 0 &&
	    dm->dm_chaddr[4] == 0 && dm->dm_chaddr[5] == 0) {
		DBFPRINTF("MAC address is zero!\n");
	}
#endif

	dm->dm_cookie = hton32(0x63825363);
	next_op = dm->dm_options;
	next_op = dhcp_addmsgtype(next_op, mtype);

	switch (mtype) {
	case DHCP_DISCOVER:
		/* Broadcast DISCOVER message to all servers */
		ds->ds_peer.sp_dest_ip = 0xfffffffful;
		memset(bcast_mac, 0xff, sizeof(bcast_mac));
		mac = bcast_mac;
		if (dhcp_open_socket(ds) < 0)
			return -1;
		next_op = dhcp_addreqoptions(next_op);
		break;

	case DHCP_REQUEST:
		/*
		 * Send REQUEST message to the server that sent the
		 * *first* OFFER
		 */
		if (dhcp_open_socket(ds) < 0)
			return -1;
		dm->dm_ciaddr = 0;
		dm->dm_siaddr = hton32(ds->ds_peer.sp_dest_ip);
		next_op = dhcp_addreqipaddr(ntoh32(dm->dm_yiaddr), next_op);
		dm->dm_yiaddr = 0;
		next_op = dhcp_addserverip(ds->ds_peer.sp_dest_ip, next_op);
		next_op = dhcp_addreqntpservers(next_op);
		mac = NULL;
		break;

	case DHCP_DECLINE:
		/*
		 * Send DECLINE message to the server that sent the
		 * *last* OFFER
		 */
		if (dhcp_open_socket(ds) < 0)
			return -1;
		dm->dm_ciaddr = hton32(ds->ds_peer.sp_dest_ip);
		next_op = dhcp_addserverip(ds->ds_peer.sp_dest_ip, next_op);
		mac = NULL;
		break;

	default:
		return -1;
	}

	next_op = dhcp_addend(next_op);
	len = (uint16_t)(next_op - ds->ds_msg_buff);

	if (ds->ds_sock == NULL || 
	    network_sock_sendto(ds->ds_sock, dm, len, mac) != len) {
		DBFPRINTF("send failed\n");
		return -1;
	}

	return 0;
}

static int8_t
dhcp_parse(struct dhcp_state *ds, uint16_t len, struct dhcp_offer *pdo)
{
	const struct dhcp_msg *dm = DHCP_MSG(ds);
	uint8_t mymac[6], mt;
	const uint8_t *p;

#ifdef DEBUG_ENABLED
	if (DEBUG_FLAG) {
		char ipstr[NETWORK_IP_STR_LEN];

		network_ip2str(ds->ds_peer.sp_dest_ip, ipstr);
		DBFPRINTF("reply from %s\n", ipstr);

		hexdump(NULL, " ", 16, ds->ds_msg_buff, len);
	}
#endif

	if (dm->dm_op != DHCP_REPLY) {
		DBFPRINTF("bad reply (op %u)\n", dm->dm_op);
		return -1;
	}

	if (dm->dm_cookie != hton32(0x63825363)) {
		DBFPRINTF("bad magic (0x%08" PRIx32 ")\n", dm->dm_cookie);
		return -1;
	}

	if (ntoh32(dm->dm_xid) != ds->ds_xid) {
		DBFPRINTF("bad xid. Wanted %08" PRIx32 " got %08" PRIx32 "\n",
		    ds->ds_xid, ntoh32(dm->dm_xid));
		return -1;
	}

	network_get_mac(mymac);
	if (memcmp(dm->dm_chaddr, mymac, sizeof(mymac)) != 0) {
		DBFPRINTF("bad MAC in reply\n");
		return -1;
	}

	DBFPRINTF("got message, len %" PRIu16 ":\n", len);
	DBPRINTF("\tdm_op:     %u\n", (unsigned int)dm->dm_op);
	DBPRINTF("\tdm_htype:  %u\n", (unsigned int)dm->dm_htype);
	DBPRINTF("\tdm_hlen:   %u\n", (unsigned int)dm->dm_hlen);
	DBPRINTF("\tdm_hops:   %u\n", (unsigned int)dm->dm_hops);
	DBPRINTF("\tdm_xid:    %08" PRIx32 "\n", dm->dm_xid);
	DBPRINTF("\tdm_secs:   %" PRIu16" \n", dm->dm_secs);
	DBPRINTF("\tdm_flags:  %04" PRIu16" \n", dm->dm_flags);
	DBPRINTF("\tdm_ciaddr: %08" PRIx32 "\n", ntoh32(dm->dm_ciaddr));
	DBPRINTF("\tdm_yiaddr: %08" PRIx32 "\n", ntoh32(dm->dm_yiaddr));
	DBPRINTF("\tdm_siaddr: %08" PRIx32 "\n", ntoh32(dm->dm_siaddr));
	DBPRINTF("\tdm_giaddr: %08" PRIx32 "\n", ntoh32(dm->dm_giaddr));
	DBPRINTF("\tdm_chaddr: %02x:%02x:%02x:%02x:%02x:%02x\n",
	    (unsigned int)dm->dm_chaddr[0], (unsigned int)dm->dm_chaddr[1],
	    (unsigned int)dm->dm_chaddr[2], (unsigned int)dm->dm_chaddr[2],
	    (unsigned int)dm->dm_chaddr[4], (unsigned int)dm->dm_chaddr[5]);
	DBPRINTF("\tdm_serv:   %s\n", (const char *)dm->dm_servname);
	DBPRINTF("\tdm_boot    %s\n", (const char *)dm->dm_bootfile);

	mt = 0;

	if (pdo) {
		memset(pdo, 0, sizeof(*pdo));
		pdo->do_addr = ntoh32(dm->dm_yiaddr);
		pdo->do_server_ip = ds->ds_peer.sp_dest_ip;
	}

	for (p=dm->dm_options; p < &ds->ds_msg_buff[len] &&
	    *p != DHCP_OPTION_END;){
		uint8_t op_len;
		uint32_t v;

		op_len = p[1] + 2;

		DBFPRINTF("op %2u, len %u\n", (unsigned int)p[0],
		    (unsigned int)op_len);

		if (op_len == 0)
			break;

		switch (*p) {
		case DHCP_OPTION_MSG_TYPE:
			mt = p[2];
			break;

		case DHCP_OPTION_SUBNET_MASK:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_mask = ntoh32(v);
			}
			break;
			
		case DHCP_OPTION_ROUTER:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_gate = ntoh32(v);
			}
			break;
			
		case DHCP_OPTION_REQ_IPADDR:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_addr = ntoh32(v);
			}
			break;

		case DHCP_OPTION_LEASE_TIME:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_lease = ntoh32(v);
			}
			break;

		case DHCP_OPTION_NTP_SERVERS:
			if (pdo && op_len >= 4) {
				memcpy(&v, &p[2], 4);
				pdo->do_ntp_server = ntoh32(v);
			}
			break;

		case DHCP_OPTION_T1:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_t1 = ntoh32(v);
			}
			break;

		case DHCP_OPTION_T2:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_t2 = ntoh32(v);
			}
			break;

		case DHCP_OPTION_SERVER_ID:
			if (pdo) {
				memcpy(&v, &p[2], 4);
				pdo->do_server_id = ntoh32(v);
			}
			break;

		default:
			break;
		}

		p += op_len;
	}

	return (int8_t)mt;
}

static uint32_t
dhcp_state_init(struct dhcp_state *ds)
{
	network_ip_params_t ip;

	DBFPRINTF("INIT\n");

	ds->ds_xid = (uint32_t)rand() * (uint32_t)rand();
	if (ds->ds_sock != NULL) {
		network_close(ds->ds_sock);
		ds->ds_sock = NULL;
	}

	memset(&ds->ds_offer, 0, sizeof(ds->ds_offer));
	memset(&ip, 0, sizeof(ip));
	network_set_ip_params(&ip);

	ds->ds_peer.sp_dest_ip = 0xfffffffful;
	ds->ds_state = DHCP_STATE_START;
	ds->ds_backoff = 1;

	return 0;
}

static uint32_t
dhcp_state_start(struct dhcp_state *ds)
{
	uint32_t rv;

	DBFPRINTF("START\n");

	ds->ds_xid++;
	time(&ds->ds_start_time);

	if (dhcp_open_socket(ds) < 0)
		return TIMER_HZ;

	if (dhcp_send(ds, DHCP_DISCOVER) < 0) {
		ds->ds_state = DHCP_STATE_INIT;
		rv = TIMER_HZ;
	} else {
		ds->ds_state = DHCP_STATE_DISCOVER;
		rv = TIMER_HZ * 2;
	}

	return rv;
}

static int8_t
dhcp_read_response(struct dhcp_state *ds, struct dhcp_offer *pdo)
{
	uint16_t len;

	ds->ds_rx_ready = 0;

	DBFPRINTF("reading response\n");

	if (ds->ds_sock == NULL) {
		DBFPRINTF("socket was closed under us!\n");
		return -1;
	}

	len = network_sock_rx_avail(ds->ds_sock);
	if (NETWORK_SOCK_ERR(len)) {
		DBFPRINTF("err1 %" PRIx16 "\n", len);
		return -1;
	}

	DBFPRINTF("rx len %" PRIu16 "\n", len);

	if (len == 0)
		return -1;

	len = mymin(len, sizeof(ds->ds_msg_buff));
	len = network_sock_recv(ds->ds_sock, ds->ds_msg_buff, len,&ds->ds_peer);
	if (NETWORK_SOCK_ERR(len))
		return -1;

	return dhcp_parse(ds, len, pdo);
}

static uint32_t
dhcp_state_discover(struct dhcp_state *ds)
{
	struct dhcp_offer dof;
	uint32_t rv;

	DBFPRINTF("DISCOVER\n");

	if (ds->ds_rx_ready == 0) {
		DBFPRINTF("DISCOVER timeout.\n");
		ds->ds_state = DHCP_STATE_START;
		if (ds->ds_backoff < 60)
			ds->ds_backoff *= 2;
		return TIMER_HZ * (uint32_t)ds->ds_backoff;
	}

	ds->ds_backoff = 1;

	if (dhcp_read_response(ds, &dof) != DHCP_OFFER) {
		ds->ds_state = DHCP_STATE_INIT;
		return TIMER_HZ;
	}

	DBFPRINTF("Response good. Sending REQUEST\n");

	if (dhcp_send(ds, DHCP_REQUEST) < 0) {
		DBFPRINTF("REQUEST send failed\n");
		ds->ds_state = DHCP_STATE_INIT;
		rv = TIMER_HZ;
	} else {
		ds->ds_state = DHCP_STATE_REQUEST;
		rv = TIMER_HZ * 2;
	}

	return rv;
}

#if defined(DEBUG_ENABLED) || defined(DHCP_OPT_COMMAND)
static void
dhcp_fprintf(FILE *os, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);

	if (os == NULL)
		debug_vprint(fmt, ap);
	else
		vfprintf(os, fmt, ap);

	va_end(ap);
}

static void
dhcp_show_lease(FILE *os, struct dhcp_state *ds)
{
	char ipstr[NETWORK_IP_STR_LEN];
	struct timeval tv;
	time_t rt;

	/*
	 * Work out how much time has elapsed since we acquired the lease
	 */
	timer_get_mono_time(&tv);
	tv.tv_sec -= ds->ds_lease_mono_time;
	rt = time(NULL);

	if (ds->ds_state == DHCP_STATE_LEASED) {
		if (ds->ds_offer.do_t1 >= tv.tv_sec)
			rt += ds->ds_offer.do_t1 - tv.tv_sec;
		network_ip2str(ds->ds_offer.do_server_ip, ipstr);
		dhcp_fprintf(os, "Leased from %s, renew in %" PRIu32" secs "
		    "(%s)\n", ipstr,
		    (uint32_t)(ds->ds_offer.do_t1 - tv.tv_sec), ctime(&rt));
	} else
	if (ds->ds_state == DHCP_STATE_RENEW) {
		if (ds->ds_offer.do_t2 >= tv.tv_sec)
			rt += ds->ds_offer.do_t2 - tv.tv_sec;
		network_ip2str(ds->ds_offer.do_server_ip, ipstr);
		dhcp_fprintf(os, "Renewing from server %s, rebind in "
		    "%" PRIu32" secs (%s)\n", ipstr,
		    (uint32_t)(ds->ds_offer.do_t2 - tv.tv_sec),
		    ctime(&rt));
	} else {
		dhcp_fprintf(os, "No active lease\n");
		return;
	}

	network_ip2str(ds->ds_offer.do_addr, ipstr);
	dhcp_fprintf(os, "Addr %s/%u, ", ipstr,
	    33 - ffsl(ds->ds_offer.do_mask));

	network_ip2str(ds->ds_offer.do_gate, ipstr);
	dhcp_fprintf(os, "Gate %s", ipstr);

	if (ds->ds_offer.do_ntp_server != 0) {
		network_ip2str(ds->ds_offer.do_ntp_server, ipstr);
		dhcp_fprintf(os, ", NTP %s", ipstr);
	}

	dhcp_fprintf(os, "\n");

	if (os != NULL)
		fflush(os);
}
#endif /* defined(DEBUG_ENABLED) || defined(DHCP_OPT_COMMAND) */

static uint32_t
dhcp_sleep_time(uint32_t *tp, uint32_t *rbp)
{
	uint32_t rv;

	if (*tp > 300)
		rv = 300;
	else
		rv = *tp;

	*tp -= rv;

	if (rbp != NULL) {
		if (*rbp > rv)
			*rbp -= rv;
		else
			*rbp = 0;
	}

	return rv * TIMER_HZ;
}

static uint32_t
dhcp_state_request(struct dhcp_state *ds)
{
	struct dhcp_offer dof;
	struct timeval tv;

	DBFPRINTF("REQUEST\n");

	if (ds->ds_rx_ready == 0) {
		DBFPRINTF("REQUEST timeout\n");
		ds->ds_state = DHCP_STATE_INIT;
		return 0;
	}

	if (dhcp_read_response(ds, &dof) != DHCP_ACK) {
		ds->ds_state = DHCP_STATE_INIT;
		return TIMER_HZ;
	}

	if (dof.do_lease == 0)
		dof.do_lease = 3600;
	if (dof.do_t1 == 0)
		dof.do_t1 = dof.do_lease / 2;
	if (dof.do_t2 == 0)
		dof.do_t2 = dof.do_lease - ((dof.do_lease * 8) / 7);

	ds->ds_renew_time = dof.do_t1;
	ds->ds_rebind_time = dof.do_t2;
	timer_get_mono_time(&tv);
	ds->ds_lease_mono_time = tv.tv_sec;

	network_close(ds->ds_sock);
	ds->ds_sock = NULL;

	if (dof.do_addr != ds->ds_offer.do_addr ||
	    dof.do_mask != ds->ds_offer.do_mask ||
	    dof.do_gate != ds->ds_offer.do_gate) {
		network_ip_params_t ip;

		ip.ip_addr = dof.do_addr;
		ip.ip_mask = dof.do_mask;
		ip.ip_gate = dof.do_gate;
		network_set_ip_params(&ip);
	}

	ds->ds_offer = dof;
	ds->ds_state = DHCP_STATE_LEASED;

#ifdef DEBUG_ENABLED
	if (DEBUG_FLAG)
		dhcp_show_lease(NULL, ds);
#endif

	return dhcp_sleep_time(&ds->ds_renew_time, &ds->ds_rebind_time);
}

static uint32_t
dhcp_state_leased(struct dhcp_state *ds)
{

	DBFPRINTF("LEASED. %" PRIu32 " seconds remaining.\n",
	    ds->ds_renew_time);

	if (ds->ds_renew_time == 0) {
		ds->ds_state = DHCP_STATE_RENEW;
		return 0;
	}

	return dhcp_sleep_time(&ds->ds_renew_time, &ds->ds_rebind_time);
}

static uint32_t
dhcp_state_renew(struct dhcp_state *ds)
{

	DBFPRINTF("RENEW. %" PRIu32 " seconds remaining.\n", ds->ds_renew_time);

	if (ds->ds_rx_ready == 0 && ds->ds_rebind_time == 0) {
		ds->ds_state = DHCP_STATE_INIT;
		return 0;
	}

	if (ds->ds_rx_ready == 0) {
		struct dhcp_msg *dm = DHCP_MSG(ds);
		ds->ds_peer.sp_dest_ip = ds->ds_offer.do_server_ip;
		dm->dm_yiaddr = hton32(ds->ds_offer.do_addr);
		if (dhcp_send(ds, DHCP_REQUEST) < 0) {
			ds->ds_state = DHCP_STATE_INIT;
			return TIMER_HZ;
		}

		return dhcp_sleep_time(&ds->ds_rebind_time, NULL);
	}

	return dhcp_state_request(ds);
}

static rtos_tick_t
dhcp_state_machine(struct dhcp_state *ds)
{
	uint32_t tm;

	tm = 0;

	switch (ds->ds_state) {
	case DHCP_STATE_INIT:
		if ((tm = dhcp_state_init(ds)) != 0)
			break;
		/*FALLTHROUGH*/

	case DHCP_STATE_START:
		tm = dhcp_state_start(ds);
		break;

	case DHCP_STATE_DISCOVER:
		tm = dhcp_state_discover(ds);
		break;

	case DHCP_STATE_REQUEST:
		tm = dhcp_state_request(ds);
		break;

	case DHCP_STATE_LEASED:
		tm = dhcp_state_leased(ds);
		break;

	case DHCP_STATE_RENEW:
		tm = dhcp_state_renew(ds);
		break;

	case DHCP_STATE_IDLE:
		/* Usually only happens when there's no Ethernet link */
		tm = TIMER_HZ;
		break;
	}

	return (rtos_tick_t)tm;
}

static void
dhcp_task(void *arg)
{
	struct dhcp_state *ds = arg;
	uint8_t last_running = ds->ds_running;
	uint8_t this_running;

	for (;;) {
		this_running = ds->ds_running;

		if (this_running != last_running) {
			if (last_running != 0) {
				DBFPRINTF("closing down\n");
				if (ds->ds_sock != NULL) {
					network_close(ds->ds_sock);
					ds->ds_sock = NULL;
				}
			} else {
				DBFPRINTF("starting up\n");
				ds->ds_state = DHCP_STATE_INIT;
			}

			last_running = this_running;
		}

		if (this_running) {
			rtos_tick_t ms;

			do {
				ms = dhcp_state_machine(ds);
			} while (ms == 0);
			rtos_notify_take_timed(1, ms);
		} else {
			DBFPRINTF("idle\n");
			rtos_notify_take(1);
		}
	}
}

#ifdef DHCP_OPT_COMMAND
static void
dhcp_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct dhcp_state *ds = &dhcp_state;

	if (ds->ds_running == 0) {
		fputs("DHCP is not enabled (static IP configured)\n", os);
		return;
	}

	if (argc >= 1) {
		if (strcasecmp(argv[0], "renew") == 0) {
			if (ds->ds_state != DHCP_STATE_LEASED) {
				fputs("No lease to renew!\n", os);
				return;
			}

			ds->ds_state = DHCP_STATE_RENEW;
			ds->ds_backoff = 1;
			rtos_notify_give(ds->ds_task);
			return;
		} else
		if (strcasecmp(argv[0], "rebind") == 0) {
			ds->ds_state = DHCP_STATE_INIT;
			ds->ds_backoff = 1;
			rtos_notify_give(ds->ds_task);
			return;
		}
#ifdef DEBUG_ENABLED
		else
		if (strcasecmp(argv[0], "debug") == 0) {
			DEBUG_FLAG = 1 - DEBUG_FLAG;
			fprintf(os, "DHCP debug O%s\n",
			    DEBUG_FLAG ? "n" : "ff");
		}
#endif
	}

	dhcp_show_lease(os, ds);
}
#endif /* DHCP_OPT_COMMAND */
