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

#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>
#include "ringbuff.h"

#ifdef NETWORK_LWIP
#include "lwipopts.h"
#endif

#ifdef NETWORK_DEV_WIZCHIP
/*
 * SPI Ethernet
 */
#ifndef NETWORK_LWIP
#define	NETWORK_OPT_DHCP_CLIENT
#else
#define	NETWORK_OPT_EXTERNAL_DHCP_CLIENT
#define	NETWORK_OPT_HTTPD	1
#endif
#define	EXTRA_SOCKETS	1	/* For DHCP */

#elif defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
/*
 * WiFi
 */
#ifndef NETWORK_LWIP
#ifdef NETWORK_DEV_WILC
#error "NETWORK_DEV_WILC requires NETWORK_LWIP"
#endif
#define	NETWORK_OPT_EXTERNAL_DHCP_CLIENT
#define	NETWORK_OPT_DHCP_ONLY
#else
#define	NETWORK_OPT_EXTERNAL_DHCP_CLIENT
#endif
#define	EXTRA_SOCKETS	0

#else	/* Native networking */
#define	NETWORK_OPT_EXTERNAL_DHCP_CLIENT
#define	EXTRA_SOCKETS	1
#endif	/* NETWORK_DEV_x */

#ifndef CONFIG_NETWORK_NSOCKETS
#define	CONFIG_NETWORK_NSOCKETS	2
#endif

#define	NETWORK_OPT_COMMAND
#define	NETWORK_NSOCKETS	(CONFIG_NETWORK_NSOCKETS + EXTRA_SOCKETS)

#ifndef	CONFIG_USE_CONFIGDB
#define	NETWORK_GLOBAL_VARS			\
	uint32_t        glob_network_ip;	\
	uint32_t        glob_network_gate;	\
	uint32_t        glob_network_mask;	\
	uint32_t        glob_network_idle;
#endif /* CONFIG_USE_CONFIGDB */

typedef struct {
	uint32_t ip_addr;
	uint32_t ip_mask;
	uint32_t ip_gate;
} network_ip_params_t;

typedef enum {
	NETWORK_SIG_NONE = 0,
	NETWORK_SIG_POOR,
	NETWORK_SIG_FAIR,
	NETWORK_SIG_GOOD,
	NETWORK_SIG_EXCELLENT
} network_signal_strength_t;

typedef struct {
	uint8_t ls_state;
#define	NETWORK_LINK_DOWN		0
#define	NETWORK_LINK_UP			1
#define	NETWORK_LINK_CONNECTING		2
#define	NETWORK_LINK_PROVISIONING	3
/* The following are only relevant for WiFi */
	int8_t ls_rssi;
	uint8_t ls_chan;
	network_signal_strength_t ls_sig;
	char ls_bssid[33];
	char ls_key[65];
} network_link_status_t;

typedef void (*network_link_status_callback_t)(
		const network_link_status_t *, void *);

typedef void (*network_activity_callback_t)(void *);

typedef void	*network_sock_t;
#define	NETWORK_SOCKET_INVALID	((network_sock_t)NULL)

typedef enum {
	NETWORK_SOCK_TYPE_TCP,
	NETWORK_SOCK_TYPE_UDP
} network_sock_type_t;

typedef enum {
	NETWORK_STATUS_IDLE,
	NETWORK_STATUS_CONNECT,
	NETWORK_STATUS_TELNET_READY,
	NETWORK_STATUS_TELNET_BREAK,
	NETWORK_STATUS_DISCONNECT,
	NETWORK_STATUS_LINK_UP,
	NETWORK_STATUS_LINK_DOWN,
	NETWORK_STATUS_TIMEOUT,
	NETWORK_STATUS_RX_READY,
	NETWORK_STATUS_TX_DONE,
	NETWORK_STATUS_UNREACHABLE,
} network_sock_status_t;

typedef enum {
	NETWORK_SOCKS_NONE,
	NETWORK_SOCKS_UDP,
	NETWORK_SOCKS_LISTEN,
	NETWORK_SOCKS_ESTABLISHED,
	NETWORK_SOCKS_CLOSE_WAIT,
	NETWORK_SOCKS_SYNSENT,
	NETWORK_SOCKS_SYNRECV,
	NETWORK_SOCKS_FIN_WAIT,
	NETWORK_SOCKS_CLOSING,
	NETWORK_SOCKS_TIME_WAIT,
	NETWORK_SOCKS_LAST_ACK,
} network_sock_state_t;

typedef void (*network_sock_status_callback_t)(
		network_sock_status_t, void *);

#ifndef NETWORK_OPT_NO_TELNET
typedef enum {
	NETWORK_TELNET_NULL,
	NETWORK_TELNET_SEND_BREAK,
} network_telnet_msg_t;

typedef void (*network_telnet_callback_t)(network_telnet_msg_t, void *);
#endif /* NETWORK_OPT_NO_TELNET */

typedef struct {
	union {
		struct {
			uint32_t usp_dest_ip;
			uint32_t usp_local_ip;
			uint16_t usp_sport;
			uint16_t usp_dport;
		} u_tcpudp;
		struct {
			uint32_t usp_dest_ip;
			uint8_t usp_proto;
		} u_ipraw;
	} sp_u;
} network_sock_params_t;
#define	sp_dest_ip	sp_u.u_tcpudp.usp_dest_ip
#define	sp_local_ip	sp_u.u_tcpudp.usp_local_ip
#define	sp_sport	sp_u.u_tcpudp.usp_sport
#define	sp_dport	sp_u.u_tcpudp.usp_dport
#define	sp_proto	sp_u.u_ipraw.usp_proto

#define	NETWORK_SOCK_ERR_BAD_CALL	0xfffbu
#define	NETWORK_SOCK_ERR_NO_LINK	0xfffcu
#define	NETWORK_SOCK_ERR_NOT_OPEN	0xfffdu
#define	NETWORK_SOCK_ERR_BUSY		0xfffeu
#define	NETWORK_SOCK_ERR_EOF		0xffffu
#define	NETWORK_SOCK_ERR(x)		((x) >= 0xfff0u)

/*
 * On no account must ANY calls to the driver entrypoints, below, be
 * made by code outside of network.c. Driver code is generally not
 * re-entrant and network.c goes to some lengths to ensure locking
 * around driver calls. There is usually some other way to achieve
 * what you are trying to do via the proper API.
 *
 * Failure to follow this process *WILL* cause weird problems and
 * crashes.
 */
struct network_driver {
	void	*nd_cookie;
	void	(*nd_worker)(void *);
	void	(*nd_link_check)(void *);
	void	(*nd_link_status)(void *, network_link_status_callback_t,
			void *);
	void	(*nd_get_mac)(void *, uint8_t *);
	int8_t	(*nd_ioctl)(void *, uint8_t, void *);
	void	(*nd_set_ip_params)(void *, const network_ip_params_t *);
	network_sock_t (*nd_open)(void *, network_sock_type_t,
			network_sock_status_callback_t, void *);
	void	(*nd_close)(void *, network_sock_t);
	uint8_t	(*nd_connect)(void *, network_sock_t,
			const network_sock_params_t *);
	uint8_t	(*nd_listen)(void *, network_sock_t, uint16_t);
	uint16_t (*nd_rx_avail)(void *, network_sock_t);
	uint16_t (*nd_read)(void *, network_sock_t, void *, uint16_t);
	uint16_t (*nd_recv)(void *, network_sock_t, void *, uint16_t,
			network_sock_params_t *);
	uint16_t (*nd_tx_free)(void *, network_sock_t);
	network_sock_state_t (*nd_sock_status)(void *, network_sock_t,
			network_sock_params_t *);
	uint16_t (*nd_write)(void *, network_sock_t, const void *, uint16_t,
			const uint8_t *, bool);
};

#define	NETWORK_IP_STR_LEN	16

struct network_state;
extern struct network_state *network_register(const struct network_driver *);
extern void	network_get_mac(uint8_t *);
extern int8_t	network_ioctl(struct network_state *, uint8_t, void *);
extern void	network_driver_hw_interrupt(struct network_state *);
extern void	network_driver_soft_timeout(struct network_state *, uint32_t);
extern void	network_set_ip_params(const network_ip_params_t *);
extern void	network_set_ip_params_drv(const network_ip_params_t *);
extern void	network_set_ip_params_drv_persist(const network_ip_params_t *);
extern void	*network_socket_listen(uint16_t port,
			network_sock_status_callback_t, void *);
extern void	*network_socket_udp(const network_sock_params_t *,
			network_sock_status_callback_t, void *);
extern int8_t	network_establish(void *, ringbuff_t, ringbuff_t, bool,
			network_sock_params_t *);
extern void	network_tcp_nagle(void *, bool);
extern uint16_t	network_sock_rx_avail(void *);
extern uint16_t	network_sock_recv(void *, void *, uint16_t,
			network_sock_params_t *);
extern uint16_t	network_sock_sendto(void *, const void *, uint16_t,
			const uint8_t *);
extern network_sock_state_t network_sock_status(void *,network_sock_params_t *);
extern void	network_close(void *s);
extern int8_t	network_hook_link_status(network_link_status_callback_t,
			void *);
extern void	network_unhook_link_status(int8_t);
extern int8_t	network_hook_activity(network_activity_callback_t, void *);
extern void	network_unhook_activity(int8_t);
extern const char *network_status_string(network_sock_status_t);
extern char	*network_ip2str(uint32_t ip, char *);
extern const char *network_str2ip(const char *, uint32_t *);
extern const network_ip_params_t *network_get_ip_params(void);
extern const char *network_parse_ip_params(const char *, const char *,
			const char *, network_ip_params_t *);
extern void	network_load_defaults(void);
extern void	network_get_link_status(network_link_status_t *);
extern void	network_assert_driver_locked(void);

/* Network ioctl codes */
#define	NETWORK_IOCTL_NO_OP		0
#define	NETWORK_IOCTL_SUSPEND		1
struct network_suspend_args {
	void (*sa_cb)(void *);
	void *sa_cookie;
};
#define	NETWORK_IOCTL_RESUME		2
#define	NETWORK_IOCTL_DHCP_CLIENT_START	3
#define	NETWORK_IOCTL_DHCP_CLIENT_STOP	4
#define	NETWORK_IOCTL_DHCP_SERVER_START	5
#define	NETWORK_IOCTL_DHCP_SERVER_STOP	6
#define	NETWORK_IOCTL_WIFI_PROVISION	7
#define	NETWORK_IOCTL_TCP_NAGLE_OFF	8	/* 'arg' is socket cookie */
#define	NETWORK_IOCTL_TCP_NAGLE_ON	9	/* 'arg' is socket cookie */
#define	NETWORK_IOCTL_STATS		10

/* ioctl numbers below this are reserved for the network layer */
#define	NETWORK_IOCTL_DRIVER_BASE	16

static __always_inline uint16_t
_BSWAP16(uint16_t x)
{

	return ((x >> 8) & 0xffu) | ((x << 8) & 0xff00u);
}

static __always_inline uint32_t
_BSWAP32(uint32_t x)
{
	uint16_t t1, t2;

	t1 = _BSWAP16((uint16_t)((x & 0x0000fffful) >>  0));
	t2 = _BSWAP16((uint16_t)((x & 0xffff0000ul) >> 16));

	return (((uint32_t)t1) << 16) | ((uint32_t)t2);
}

#define	hton16(x)	_BSWAP16((uint16_t)(x))
#define	ntoh16(x)	_BSWAP16((uint16_t)(x))
#define	hton32(x)	_BSWAP32((uint32_t)(x))
#define	ntoh32(x)	_BSWAP32((uint32_t)(x))

#endif /* NETWORK_H */
