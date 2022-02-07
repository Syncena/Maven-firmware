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

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#ifdef NETWORK_LWIP
#include "network.h"
#endif

#define	NO_SYS				1
#define	SYS_LIGHTWEIGHT_PROT		0

#define	MEM_LIBC_MALLOC			0
#define	 mem_clib_free			zone_free
#define	 mem_clib_malloc		zone_malloc
#define	 mem_clib_calloc(c,s)		zone_calloc((c) * (s))
#define	MEM_ALIGNMENT			4

#define	MEMP_NUM_PBUF			2
#define	MEMP_NUM_NETDB			0
#define	MEMP_NUM_TCP_SEG		32

#define	LWIP_ARP			1
#define	ARP_TABLE_SIZE			4
#define	ETH_PAD_SIZE			2

#ifdef NETWORK_DEV_WILC
#define	PBUF_LINK_ENCAPSULATION_HLEN	36
#endif

#define	LWIP_ASSERT_CORE_LOCKED()	network_assert_driver_locked()

#define	LWIP_IPV4			1
#define	LWIP_ICMP			1
#define	LWIP_DHCP			1

#define	LWIP_ETHERNET			1
#define	LWIP_SINGLE_NETIF		1
#define	LWIP_NETIF_STATUS_CALLBACK	0
#define	LWIP_NETIF_EXT_STATUS_CALLBACK	1
#define	LWIP_NETIF_LINK_CALLBACK	0
#define	LWIP_NETIF_TX_SINGLE_PBUF	0
#define	LWIP_NETIF_HOSTNAME		1

#define	LWIP_NETCONN			0
#define	LWIP_SOCKET			0
#define	LWIP_SOCKET_SELECT		0
#define	LWIP_SOCKET_POLL		0
#define	LWIP_COMPAT_SOCKETS		0
#define	LWIP_POSIX_SOCKETS_IO_NAMES	0

#define	LWIP_TCP_KEEPALIVE		1

#define	LWIP_TCPIP_CORE_LOCKING		0

#define	TCP_MSS				1460
#define	TCP_SND_BUF			(8 * TCP_MSS)

#define SO_REUSE			1

#ifdef NETWORK_OPT_HTTPD
#define	HTTPD_ENABLE_HTTP		1
#define	HTTPD_SERVER_AGENT		\
	"Maven (https://www.maverick-embedded.co.uk/)"
#define	HTTPD_FSDATA_FILE		"fsdata_custom_data.c"
#define	LWIP_HTTPD_FILE_EXTENSION	1
#define	HTTPD_POLL_INTERVAL		1
#define	LWIP_HTTPD_CUSTOM_FILES		1
#define	LWIP_HTTPD_DYNAMIC_HEADERS	1
#define	LWIP_HTTPD_FILE_STATE		1
#define	LWIP_HTTPD_SUPPORT_POST		1
#endif	/* NETWORK_OPT_HTTPD */


//#define	LWIP_STATS			1
#if (LWIP_STATS == 1)
#define	LWIP_STATS_LARGE		1
#define	LWIP_STATS_DISPLAY		LWIP_STATS
#define	ETHARP_STATS			LWIP_STATS
#define	IP_STATS			LWIP_STATS
#define	IPFRAG_STATS			LWIP_STATS
#define	ICMP_STATS			LWIP_STATS
#define	UDP_STATS			LWIP_STATS
#define	TCP_STATS			LWIP_STATS
#define	MEM_STATS			LWIP_STATS
#endif


#undef	LWIP_DEBUG
#define	LWIP_NOASSERT
#define LWIP_DBG_MIN_LEVEL              LWIP_DBG_LEVEL_SERIOUS
#define LWIP_DBG_TYPES_ON               LWIP_DBG_ON
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define IGMP_DEBUG                      LWIP_DBG_OFF
#define INET_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define IP_REASS_DEBUG                  LWIP_DBG_OFF
#define RAW_DEBUG                       LWIP_DBG_OFF
#define MEM_DEBUG                       LWIP_DBG_OFF
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define SYS_DEBUG                       LWIP_DBG_OFF
#define TIMERS_DEBUG                    LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_OFF
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#define TCP_FR_DEBUG                    LWIP_DBG_OFF
#define TCP_RTO_DEBUG                   LWIP_DBG_OFF
#define TCP_CWND_DEBUG                  LWIP_DBG_OFF
#define TCP_WND_DEBUG                   LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF
#define TCP_RST_DEBUG                   LWIP_DBG_OFF
#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define SLIP_DEBUG                      LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF
#define AUTOIP_DEBUG                    LWIP_DBG_OFF
#define ACD_DEBUG                       LWIP_DBG_OFF
#define DNS_DEBUG                       LWIP_DBG_OFF
#define IP6_DEBUG                       LWIP_DBG_OFF
#define DHCP6_DEBUG                     LWIP_DBG_OFF

#endif /* LWIPOPTS_H */
