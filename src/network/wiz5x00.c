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

/*
 * Driver for the Wiznet WIZ5200 and/or WIZ5500 Ethernet Controllers
 *
 * Compile with one or both of -DWIZ_5200=1 -DWIZ_5500=1.
 * If both are defined, driver will auto-detect the chip flavour.
 *
 * Works with on-chip TCP/IP stack, or MACRAW mode for LwIP support.
 * Add -DNETWORK_LWIP=1 for the latter.
 */

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "wiz5x00.h"
#include "wiz5x00reg.h"
#include "rtos.h"
#include "linked-lists.h"
#include "timer.h"
#include "zone_alloc.h"

#ifdef NETWORK_LWIP
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"
#endif

#undef	WIZ_DUMP_REGS
//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

struct wiz_udp_packet_header {
	uint32_t wup_peer_ip;
	uint16_t wup_peer_port;
	uint16_t wup_plen;
} __attribute__((__packed__));

struct wiz_macraw_packet_header {
	uint16_t wmph_plen;
} __attribute__((__packed__));

struct wiz_socket {
	network_sock_status_callback_t s_status_callback;
	void *s_arg;
	uint8_t s_active;

	uint8_t s_sn;
	uint8_t s_mask;
	uint8_t s_type;

	uint16_t s_rx_size;
	uint16_t s_rxptr;

	uint8_t s_tx_pend;
	uint16_t s_tx_rd;
	uint16_t s_tx_size;
	uint16_t s_txptr;
};

#define	WIZ_SOCK_TYPE(s)	((s)->s_type & WIZ5x00_SREG_MR_P_MASK)

struct wiz_state {
	struct network_state *ws_network_state;
	void (*ws_read_write)(void *, uint8_t, const struct spi_iovec *);
	void (*ws_control)(void *, uint8_t, void *);
	void *ws_cookie;
	struct spi_iovec ws_iovecs[2];
	uint8_t ws_chip;
	volatile uint8_t ws_irq_pending;
	uint8_t ws_open_mask;
	uint8_t ws_udp_open_pending;
#define	ws_tx_busy	ws_udp_open_pending
	uint8_t ws_link_status;
	uint8_t ws_nsockets;
	struct wiz_socket *ws_socks;
	network_link_status_callback_t ws_link_cb;
	void *ws_link_cb_arg;
	network_ip_params_t ws_ip;
	uint8_t ws_mac[6];
};

#ifdef NETWORK_LWIP
struct wiz_pbuf {
	struct pbuf *wp_pbuf;
	STAILQ_ENTRY(wiz_pbuf) wp_qent;
};

STAILQ_HEAD(wiz_pbuf_qhead, wiz_pbuf);
struct wiz_if_state {
	struct wiz_state wis_ws;
	struct netif wis_netif;
	network_link_status_callback_t wis_link_cb;
	void *wis_link_cb_arg;
	struct wiz_pbuf_qhead wis_tx_queue;
	struct wiz_pbuf_qhead wis_free_queue;
};
#endif /* NETWORK_LWIP */

union wiz_spi_header {
#ifdef WIZ_5200
	struct wiz5200_spi_header h5200;
#endif
#ifdef WIZ_5500
	struct wiz5500_spi_header h5500;
#endif
};

#if defined(WIZ_5200) && defined(WIZ_5500)
/* Both 5200 and 5500 supported */
#define	WIZ_SOCKET_IRQ_REG(ws)	(((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_CREG_IR2 : WIZ5500_CREG_SIR)
#define	WIZ_SOCKET_IMASK_REG(ws) (((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_CREG_IMR2 : WIZ5500_CREG_SIMR)
#define	WIZ_COMMON_IMASK_REG(ws) (((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_CREG_IMR : WIZ5500_CREG_IMR)
#define	WIZ_COMMON_IMASK_BITS(ws) (((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5x00_CREG_IR_CONFLICT : \
				 (WIZ5x00_CREG_IR_CONFLICT | \
				 WIZ5500_CREG_IR_UNREACH))
#define	WIZ_COMMON_PHY_REG(ws)	(((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_CREG_PSTATUS : WIZ5500_CREG_PHYCFGR)
#define	WIZ_COMMON_PHY_LINK(ws)	(((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_CREG_PHY_LINK : WIZ5500_CREG_PHY_LINK)
#define	WIZ_CREG_OFF(ws)	(((ws)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_MAP_COMMON_REGS : WIZ5500_BLK_COMMON)
#define	WIZ_SREG_OFF(w,s)	(((w)->ws_chip == WIZ_CHIP_5200) ? \
				 WIZ5200_MAP_SOCKET_REGS(s):WIZ5500_BLK_REG(s))
#define	WIZ_INIT_SPI_HEADER_READ_REG(l)	\
do {									       \
	if (ws->ws_chip == WIZ_CHIP_5200) {				       \
		(void) wiz5200_spi_header_read(&h.h5200, blk + reg, (l));      \
		hlen = sizeof(h.h5200);					       \
	} else {							       \
		wiz5500_spi_header_read(&h.h5500, blk, reg, WIZ5500_OM_F(l));  \
		hlen = sizeof(h.h5500) - 1;				       \
	}								       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_REG(l)	\
do {									       \
	if (ws->ws_chip == WIZ_CHIP_5200) {				       \
		(void) wiz5200_spi_header_write(&h.h5200, blk + reg, (l));     \
		hlen = sizeof(h.h5200);					       \
	} else {							       \
		wiz5500_spi_header_write(&h.h5500, blk,reg,WIZ5500_OM_F(l));   \
		hlen = sizeof(h.h5500) - 1;				       \
	}								       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_READ_MEM(l)	\
do {									       \
	if (ws->ws_chip == WIZ_CHIP_5200) {				       \
		l = wiz5200_spi_header_read(&h.h5200, blk + addr, (l));        \
		hlen = sizeof(h.h5200);					       \
	} else {							       \
		wiz5500_spi_header_read(&h.h5500, blk, addr, WIZ5500_OM_VDM);  \
		hlen = sizeof(h.h5500) - 1;				       \
	}								       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_MEM(l)	\
do {									       \
	if (ws->ws_chip == WIZ_CHIP_5200) {				       \
		l = wiz5200_spi_header_write(&h.h5200, blk + addr, (l));       \
		hlen = sizeof(h.h5200);					       \
	} else {							       \
		wiz5500_spi_header_write(&h.h5500, blk, addr, WIZ5500_OM_VDM); \
		hlen = sizeof(h.h5500) - 1;				       \
	}								       \
} while (0)
#elif defined(WIZ_5200)
/* Only 5200 supported */
#define	WIZ_SOCKET_IRQ_REG(ws)		WIZ5200_CREG_IR2
#define	WIZ_SOCKET_IMASK_REG(ws)	WIZ5200_CREG_IMR2
#define	WIZ_COMMON_IMASK_REG(ws)	WIZ5200_CREG_IMR
#define	WIZ_COMMON_IMASK_BITS(ws)	WIZ5x00_CREG_IR_CONFLICT
#define	WIZ_COMMON_PHY_REG(ws)		WIZ5200_CREG_PSTATUS
#define	WIZ_COMMON_PHY_LINK(ws)		WIZ5200_CREG_PHY_LINK
#define	WIZ_CREG_OFF(ws)		WIZ5200_MAP_COMMON_REGS
#define	WIZ_SREG_OFF(w,s)		WIZ5200_MAP_SOCKET_REGS(s)
#define	WIZ_INIT_SPI_HEADER_READ_REG(l)	\
do {								       \
	(void) wiz5200_spi_header_read(&h.h5200, blk + reg, (l));      \
	hlen = sizeof(h.h5200);					       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_REG(l)	\
do {								       \
	(void) wiz5200_spi_header_write(&h.h5200, blk + reg, (l));     \
	hlen = sizeof(h.h5200);					       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_READ_MEM(l)	\
do {								       \
	l = wiz5200_spi_header_read(&h.h5200, blk + addr, (l));        \
	hlen = sizeof(h.h5200);					       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_MEM(l)	\
do {								       \
	l = wiz5200_spi_header_write(&h.h5200, blk + addr, (l));       \
	hlen = sizeof(h.h5200);					       \
} while (0)
#else
/* Only 5500 supported */
#define	WIZ_SOCKET_IRQ_REG(ws)		WIZ5500_CREG_SIR
#define	WIZ_SOCKET_IMASK_REG(ws)	WIZ5500_CREG_SIMR
#define	WIZ_COMMON_IMASK_REG(ws)	WIZ5500_CREG_IMR
#define	WIZ_COMMON_IMASK_BITS(ws)	(WIZ5x00_CREG_IR_CONFLICT | \
					 WIZ5500_CREG_IR_UNREACH)
#define	WIZ_COMMON_PHY_REG(ws)		WIZ5500_CREG_PHYCFGR
#define	WIZ_COMMON_PHY_LINK(ws)		WIZ5500_CREG_PHY_LINK
#define	WIZ_CREG_OFF(ws)		WIZ5500_BLK_COMMON
#define	WIZ_SREG_OFF(w,s)		WIZ5500_BLK_REG(s)
#define	WIZ_INIT_SPI_HEADER_READ_REG(l)	\
do {								       \
	wiz5500_spi_header_read(&h.h5500, blk, reg, WIZ5500_OM_F(l));  \
	hlen = sizeof(h.h5500) - 1;				       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_REG(l)	\
do {								       \
	wiz5500_spi_header_write(&h.h5500, blk, reg, WIZ5500_OM_F(l)); \
	hlen = sizeof(h.h5500) - 1;				       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_READ_MEM(l)	\
do {								       \
	wiz5500_spi_header_read(&h.h5500, blk, addr, WIZ5500_OM_VDM);  \
	hlen = sizeof(h.h5500) - 1;				       \
} while (0)
#define	WIZ_INIT_SPI_HEADER_WRITE_MEM(l)	\
do {								       \
	wiz5500_spi_header_write(&h.h5500, blk, addr, WIZ5500_OM_VDM); \
	hlen = sizeof(h.h5500) - 1;				       \
} while (0)
#endif

_NOINLINE_STATIC uint8_t
_wiz_reg_read8(struct wiz_state *ws, uint16_t blk, uint16_t reg)
{
	union wiz_spi_header h;
	uint8_t hlen, v;

	WIZ_INIT_SPI_HEADER_READ_REG(sizeof(v));

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = NULL;
	ws->ws_iovecs[1].io_rd = &v;
	ws->ws_iovecs[1].io_len = sizeof(v);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);

	return v;
}

_NOINLINE_STATIC void
_wiz_reg_write8(struct wiz_state *ws, uint16_t blk, uint16_t reg,
    uint8_t data)
{
	union wiz_spi_header h;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_WRITE_REG(sizeof(data));

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = &data;
	ws->ws_iovecs[1].io_rd = NULL;
	ws->ws_iovecs[1].io_len = sizeof(data);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);
}

_NOINLINE_STATIC uint16_t
_wiz_reg_read16(struct wiz_state *ws, uint16_t blk, uint16_t reg)
{
	union wiz_spi_header h;
	uint16_t v;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_READ_REG(sizeof(v));

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = NULL;
	ws->ws_iovecs[1].io_rd = &v;
	ws->ws_iovecs[1].io_len = sizeof(v);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);

	return ntoh16(v);
}

_NOINLINE_STATIC void
_wiz_reg_write16(struct wiz_state *ws, uint16_t blk, uint16_t reg,
    uint16_t data)
{
	union wiz_spi_header h;
	uint16_t v;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_WRITE_REG(sizeof(v));

	v = hton16(data);

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = &v;
	ws->ws_iovecs[1].io_rd = NULL;
	ws->ws_iovecs[1].io_len = sizeof(v);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);
}

_NOINLINE_STATIC uint32_t
_wiz_reg_read32(struct wiz_state *ws, uint16_t blk, uint16_t reg)
{
	union wiz_spi_header h;
	uint32_t v;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_READ_REG(sizeof(v));

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = NULL;
	ws->ws_iovecs[1].io_rd = &v;
	ws->ws_iovecs[1].io_len = sizeof(v);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);

	return ntoh32(v);
}

_NOINLINE_STATIC void
_wiz_reg_write32(struct wiz_state *ws, uint16_t blk, uint16_t reg,
    uint32_t data)
{
	union wiz_spi_header h;
	uint32_t v;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_WRITE_REG(sizeof(v));

	v = hton32(data);

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = &v;
	ws->ws_iovecs[1].io_rd = NULL;
	ws->ws_iovecs[1].io_len = sizeof(v);

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);
}

_NOINLINE_STATIC void
wiz_memory_read(struct wiz_state *ws, uint16_t blk, uint16_t addr,
    void *dest, uint16_t len)
{
	union wiz_spi_header h;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_READ_MEM(len);

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = NULL;
	ws->ws_iovecs[1].io_rd = dest;
	ws->ws_iovecs[1].io_len = len;

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);
}

_NOINLINE_STATIC void
wiz_memory_write(struct wiz_state *ws, uint16_t blk, uint16_t addr,
    const void *src, uint16_t len)
{
	union wiz_spi_header h;
	uint8_t hlen;

	WIZ_INIT_SPI_HEADER_WRITE_MEM(len);

	ws->ws_iovecs[0].io_wr = &h;
	ws->ws_iovecs[0].io_rd = NULL;
	ws->ws_iovecs[0].io_len = hlen;
	ws->ws_iovecs[1].io_wr = src;
	ws->ws_iovecs[1].io_rd = NULL;
	ws->ws_iovecs[1].io_len = len;

	(ws->ws_read_write)(ws->ws_cookie, 2, ws->ws_iovecs);
}

static uint8_t
wiz_creg_read8(struct wiz_state *ws, uint16_t reg)
{

	return _wiz_reg_read8(ws, WIZ_CREG_OFF(ws), reg);
}

#ifdef	WIZ_DUMP_REGS
static uint16_t
wiz_creg_read16(struct wiz_state *ws, uint16_t reg)
{

	return _wiz_reg_read16(ws, WIZ_CREG_OFF(ws), reg);
}
#endif /* WIZ_DUMP_REGS */

#if defined(WIZ_5500) || defined(WIZ_DUMP_REGS)
static uint32_t
wiz_creg_read32(struct wiz_state *ws, uint16_t reg)
{

	return _wiz_reg_read32(ws, WIZ_CREG_OFF(ws), reg);
}
#endif /* WIZ_5500 || WIZ_DUMP_REGS */

static void
wiz_creg_write8(struct wiz_state *ws, uint16_t reg, uint8_t v)
{

	_wiz_reg_write8(ws, WIZ_CREG_OFF(ws), reg, v);
}

static void
wiz_creg_write32(struct wiz_state *ws, uint16_t reg, uint32_t v)
{

	_wiz_reg_write32(ws, WIZ_CREG_OFF(ws), reg, v);
}

#ifdef	WIZ_DUMP_REGS
static void
wiz_cmemory_read(struct wiz_state *ws, uint16_t reg,
    void *dest, uint16_t len)
{

	wiz_memory_read(ws, WIZ_CREG_OFF(ws), reg, dest, len);
}
#endif

static void
wiz_cmemory_write(struct wiz_state *ws, uint16_t reg,
    const void *src, uint16_t len)
{

	wiz_memory_write(ws, WIZ_CREG_OFF(ws), reg, src, len);
}

static uint8_t
wiz_sreg_read8(struct wiz_state *ws, uint8_t sn, uint16_t reg)
{

	return _wiz_reg_read8(ws, WIZ_SREG_OFF(ws, sn), reg);
}

static uint16_t
wiz_sreg_read16(struct wiz_state *ws, uint8_t sn, uint16_t reg)
{

	return _wiz_reg_read16(ws, WIZ_SREG_OFF(ws, sn), reg);
}

static uint32_t
wiz_sreg_read32(struct wiz_state *ws, uint8_t sn, uint16_t reg)
{

	return _wiz_reg_read32(ws, WIZ_SREG_OFF(ws, sn), reg);
}

static void
wiz_sreg_write8(struct wiz_state *ws, uint8_t sn,
    uint16_t reg, uint8_t v)
{

	_wiz_reg_write8(ws, WIZ_SREG_OFF(ws, sn), reg, v);
}

static void
wiz_sreg_write16(struct wiz_state *ws, uint8_t sn,
    uint16_t reg, uint16_t v)
{

	_wiz_reg_write16(ws, WIZ_SREG_OFF(ws, sn), reg, v);
}

static void
wiz_sreg_write32(struct wiz_state *ws, uint8_t sn,
    uint16_t reg, uint32_t v)
{

	_wiz_reg_write32(ws, WIZ_SREG_OFF(ws, sn), reg, v);
}

#ifdef	WIZ_DUMP_REGS
static void
wiz_smemory_read(struct wiz_state *ws, uint8_t sn, uint16_t addr,
    void *dest, uint16_t len)
{

	wiz_memory_read(ws, WIZ_SREG_OFF(ws, sn), addr, dest, len);
}
#endif

static void
wiz_smemory_write(struct wiz_state *ws, uint8_t sn, uint16_t addr,
    const void *src, uint16_t len)
{

	wiz_memory_write(ws, WIZ_SREG_OFF(ws, sn), addr, src, len);
}

#ifdef	WIZ_DUMP_REGS
extern void
wiz_dump_cregs(struct wiz_state *ws);
void
wiz_dump_cregs(struct wiz_state *ws)
{
	uint8_t mac[6];

	printf("\nCommon registers\n");
	printf("MR\t%02x\n", (unsigned int)wiz_creg_read8(ws, WIZ5x00_CREG_MR));
	printf("GAR\t%08" PRIx32 "\n", wiz_creg_read32(ws, WIZ5x00_CREG_GAR));
	printf("SUBR\t%08" PRIx32 "\n", wiz_creg_read32(ws, WIZ5x00_CREG_SUBR));
	printf("SIPR\t%08" PRIx32 "\n", wiz_creg_read32(ws, WIZ5x00_CREG_SIPR));

	wiz_cmemory_read(ws, WIZ5x00_CREG_SHAR, mac, sizeof(mac));
	printf("SHAR\t%02x:%02x:%02x:%02x:%02x:%02x\n",
	    (unsigned int)mac[0], (unsigned int)mac[1], (unsigned int)mac[2],
	    (unsigned int)mac[3], (unsigned int)mac[4], (unsigned int)mac[5]);

	if (ws->ws_chip == WIZ_CHIP_5200) {
		printf("INTLEVEL\t%04" PRIx16 "\n",
		    wiz_creg_read16(ws, WIZ5200_CREG_INTLEVEL));
	} else {
		printf("INTLEVEL\t%04" PRIx16 "\n",
		    wiz_creg_read16(ws, WIZ5500_CREG_INTLEVEL));
	}
	printf("IR\t%02x\n", (unsigned int)wiz_creg_read8(ws, WIZ5x00_CREG_IR));
	if (ws->ws_chip == WIZ_CHIP_5200) {
		printf("IMR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_IMR));
		printf("IR2\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_IR2));
		printf("IMR2\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_IMR2));
		printf("RTR\t%04" PRIx16 "\n",
		    wiz_creg_read16(ws, WIZ5200_CREG_RTR));
		printf("RCR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_RCR));
		printf("PSTATUS\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_PSTATUS));
		printf("VERSION\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5200_CREG_VERSIONR));
	} else {
		printf("IMR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_IMR));
		printf("SIR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_SIR));
		printf("SIMR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_SIMR));
		printf("RTR\t%04" PRIx16 "\n",
		    wiz_creg_read16(ws, WIZ5500_CREG_RTR));
		printf("RCR\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_RCR));
		printf("PHYCFG\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_PHYCFGR));
		printf("VERSION\t%02x\n",
		    (unsigned int)wiz_creg_read8(ws, WIZ5500_CREG_VERSIONR));
	}
}

extern void
wiz_dump_sregs(struct wiz_state *ws, struct wiz_socket *s);
void
wiz_dump_sregs(struct wiz_state *ws, struct wiz_socket *s)
{
	uint8_t sn = s->s_sn;
	uint8_t mac[6];

	printf("Socket #%u\n", sn);
	printf("MR\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_MR));
	printf("CR\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_CR));
	printf("IR\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_IR));
	printf("SR\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_SR));
	printf("PORT\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_PORT));
	wiz_smemory_read(ws, sn, WIZ5x00_SREG_DHAR, mac, sizeof(mac));
	printf("DHAR\t%02x:%02x:%02x:%02x:%02x:%02x\n",
	    (unsigned int)mac[0], (unsigned int)mac[1], (unsigned int)mac[2],
	    (unsigned int)mac[3], (unsigned int)mac[4], (unsigned int)mac[5]);
	printf("DIPR\t%08" PRIx32 "\n",
	    wiz_sreg_read32(ws, sn, WIZ5x00_SREG_DIPR));
	printf("DPORT\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_DPORT));
	printf("MSSR\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_MSSR));
	if (ws->ws_chip == WIZ_CHIP_5200) {
		printf("PROTO\t%04" PRIx16 "\n",
		    wiz_sreg_read16(ws, sn, WIZ5200_SREG_PROTO));
	}
	printf("TOS\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_TOS));
	printf("TTL\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_TTL));
	printf("RXSIZE\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_RXMEM_SIZE));
	printf("TXSIZE\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_TXMEM_SIZE));
	printf("TXFSR\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_FSR));
	printf("TXRD\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_RD));
	printf("TXWR\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_WR));
	printf("RXRSR\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RSR));
	printf("RXRD\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RD));
	printf("RXWR\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_WR));
	printf("IMR\t%02x\n",
	    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5x00_SREG_IMR));
	printf("FRAG\t%04" PRIx16 "\n",
	    wiz_sreg_read16(ws, sn, WIZ5x00_SREG_FRAG));
	if (ws->ws_chip == WIZ_CHIP_5500) {
		printf("KPALVTR\t%02x\n",
		    (unsigned int)wiz_sreg_read8(ws, sn, WIZ5500_SREG_KPALVTR));
	}
}
#endif /* WIZ_DUMP_REGS */

static uint8_t
wiz_command(struct wiz_state *ws, uint8_t sn, uint8_t cmd,
    uint8_t expect, uint8_t timo)
{
	uint8_t reg;
	uint16_t x;

	wiz_sreg_write8(ws, sn, WIZ5x00_SREG_CR, cmd);

	x = ((uint16_t)timo) << 8;
	while (x) {
		reg = wiz_sreg_read8(ws, sn, WIZ5x00_SREG_CR);
		if (reg == 0)
			break;
		timer_buzz_delay_us(39);
		x--;
	}

	if (x == 0)
		return 0;

	if (expect == 0)
		return 1;

	x = ((uint16_t)timo) << 8;
	while (x) {
		reg = wiz_sreg_read8(ws, sn, WIZ5x00_SREG_SR);
		if (reg == expect)
			break;
		timer_buzz_delay_us(39);
		x--;
	}

	return x != 0;
}

static void
wiz_init_hw_socket(struct wiz_state *ws, uint8_t sn, struct wiz_socket *s)
{
	uint16_t rptr = 0, tptr = 0;
	uint16_t rx_size, tx_size;

	rx_size = WIZ5x00_BUFFER_SIZE_RX / ws->ws_nsockets;
	tx_size = WIZ5x00_BUFFER_SIZE_TX / ws->ws_nsockets;

#ifdef WIZ_5200
	if (ws->ws_chip == WIZ_CHIP_5200) {
		rptr = WIZ5200_MAP_RX_MEMORY + (rx_size * sn);
		tptr = WIZ5200_MAP_TX_MEMORY + (tx_size * sn);
		if (s != NULL) {
			s->s_rxptr = rptr;
			s->s_txptr = tptr;
		}
	}
#endif
#ifdef WIZ_5500
	if (ws->ws_chip == WIZ_CHIP_5500) {
		if (s != NULL) {
			s->s_rxptr = WIZ5500_BLK_RX(sn);
			s->s_txptr = WIZ5500_BLK_TX(sn);
		}
	}
#endif

	wiz_sreg_write8(ws, sn, WIZ5x00_SREG_RXMEM_SIZE, rx_size / 1024u);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_RX_RSR, 0);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_RX_RD, rptr);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_RX_WR, rptr);

	wiz_sreg_write8(ws, sn, WIZ5x00_SREG_TXMEM_SIZE, tx_size / 1024u);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_TX_FSR, tx_size);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_TX_RD, tptr);
	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_TX_WR, tptr);

	if (ws->ws_chip == WIZ_CHIP_5500) {
		/*
		 * Enable TCP keep-alive timer. Value is in units of 5 seconds,
		 * so '10' equates to 50 seconds.
		 */
		wiz_sreg_write8(ws, sn, WIZ5500_SREG_KPALVTR, 10);
	}
}

static void
wiz_reset(struct wiz_state *ws)
{
	struct wiz_socket *s;
	uint8_t i, r;

	(ws->ws_control)(ws->ws_cookie, WIZ5X00_CTL_RESET, NULL);

	/* Initiate a software reset */
	wiz_creg_write8(ws, WIZ5x00_CREG_MR, WIZ5x00_CREG_MR_RST);

	/* Wait for it to complete. We're feeling generous, so allow 1 sec */
	for (i = 100; i; i--) {
		r = wiz_creg_read8(ws, WIZ5x00_CREG_MR);
		if ((r & WIZ5x00_CREG_MR_RST) == 0)
			break;

		rtos_task_sleep(10);
	}

	if (i == 0) {
		DBFPRINTF("Chip failed to reset!\n");
		return;
	}

	/* Initialise socket state. */
	for (i = 0, s = ws->ws_socks; i < ws->ws_nsockets; i++, s++) {
		s->s_sn = i;
		s->s_mask = (1u << i);
		s->s_rx_size = WIZ5x00_BUFFER_SIZE_RX / ws->ws_nsockets;
		s->s_tx_size = WIZ5x00_BUFFER_SIZE_TX / ws->ws_nsockets;
		s->s_tx_pend = 0;
		wiz_init_hw_socket(ws, i, s);
	}

	/* Remaining h/w sockets still need to be initialised. */
	while (i < WIZ5x00_NSOCKETS)
		wiz_init_hw_socket(ws, i++, NULL);

	wiz_cmemory_write(ws, WIZ5x00_CREG_SHAR, ws->ws_mac,
	    sizeof(ws->ws_mac));

#ifdef WIZ_5500
	/*
	 * Set the PHY to fully auto-negotiate mode.
	 */
	r = WIZ5500_CREG_PHY_OPMD | WIZ5500_CREG_PHY_OPMDC_AUTO;
	wiz_creg_write8(ws, WIZ5500_CREG_PHYCFGR, r);
	rtos_task_sleep(50);
	r |= WIZ5500_CREG_PHY_RST;
	wiz_creg_write8(ws, WIZ5500_CREG_PHYCFGR, r);
#endif
}

static void
wiz5x00_sock_callback_common(struct wiz_state *ws, struct wiz_socket *s,
    network_sock_status_t st)
{
	network_sock_status_callback_t cb;

	(void) ws;

	if ((cb = s->s_status_callback) == NULL) {
		DBFPRINTF("sock %u: Spurious %s\n", (unsigned int)s->s_sn,
		    network_status_string(st));
	} else
		(cb)(st, s->s_arg);
}

static void
wiz5x00_sock_connect(struct wiz_state *ws, struct wiz_socket *s)
{

	wiz5x00_sock_callback_common(ws, s, NETWORK_STATUS_CONNECT);
}

static void
wiz5x00_sock_disconnect(struct wiz_state *ws, struct wiz_socket *s)
{

	wiz5x00_sock_callback_common(ws, s, NETWORK_STATUS_DISCONNECT);
}

static void
wiz5x00_sock_timeout(struct wiz_state *ws, struct wiz_socket *s)
{

	wiz5x00_sock_callback_common(ws, s, NETWORK_STATUS_TIMEOUT);
}

static void
wiz5x00_sock_recv(struct wiz_state *ws, struct wiz_socket *s)
{

	wiz5x00_sock_callback_common(ws, s, NETWORK_STATUS_RX_READY);
}

static void
wiz5x00_sock_send_ok(struct wiz_state *ws, struct wiz_socket *s)
{

	wiz5x00_sock_callback_common(ws, s, NETWORK_STATUS_TX_DONE);
}

static void
wiz5x00_sock_interrupt(struct wiz_state *ws, struct wiz_socket *s)
{
	uint8_t sir;

	sir = wiz_sreg_read8(ws, s->s_sn, WIZ5x00_SREG_IR);
	wiz_sreg_write8(ws, s->s_sn, WIZ5x00_SREG_IR, sir);
	sir &= (WIZ5x00_SREG_IR_CON | WIZ5x00_SREG_IR_DISCON |
	    WIZ5x00_SREG_IR_RECV | WIZ5x00_SREG_IR_TIMEOUT |
	    WIZ5x00_SREG_IR_SEND_OK);

	if (sir && s->s_status_callback != NULL) {
		if (sir & WIZ5x00_SREG_IR_CON)
			wiz5x00_sock_connect(ws, s);
		if (sir & WIZ5x00_SREG_IR_DISCON) {
			s->s_tx_pend = 0;
			wiz5x00_sock_disconnect(ws, s);
		}
		if (sir & WIZ5x00_SREG_IR_RECV)
			wiz5x00_sock_recv(ws, s);
		if (sir & WIZ5x00_SREG_IR_TIMEOUT) {
			s->s_tx_pend = 0;
			wiz5x00_sock_timeout(ws, s);
		}
		if (sir & WIZ5x00_SREG_IR_SEND_OK) {
			s->s_tx_pend = 0;
			wiz5x00_sock_send_ok(ws, s);
		}
	} else
	if (s->s_status_callback == NULL) {
		if ((sir & WIZ5x00_SREG_IR_DISCON) ||
		    (sir & WIZ5x00_SREG_IR_TIMEOUT) ||
		    (sir & WIZ5x00_SREG_IR_SEND_OK)) {
			s->s_tx_pend = 0;
		}
	}
}

static void
wiz5x00_worker(void *arg)
{
	struct wiz_state *ws = arg;
	struct wiz_socket *s;
	uint16_t trd, ireg;
	uint8_t reg, ir, sn;
	uint8_t enable = 1;

	ireg = WIZ_SOCKET_IRQ_REG(ws);

	while (ws->ws_irq_pending) {
		ws->ws_irq_pending = 0;

		ir = wiz_creg_read8(ws, WIZ5x00_CREG_IR);
		if ((ir & WIZ_COMMON_IMASK_BITS(ws)) != 0)
			wiz_creg_write8(ws, WIZ5x00_CREG_IR, ir);

		if (ir & WIZ5x00_CREG_IR_CONFLICT && ws->ws_ip.ip_addr != 0)
			DBFPRINTF("IP address conflict\n");

		reg = wiz_creg_read8(ws, ireg) & ws->ws_open_mask;

#ifdef WIZ_5500
		if (
#ifdef WIZ_5200
		    ws->ws_chip == WIZ_CHIP_5500 &&
#endif
		    (ir & WIZ5500_CREG_IR_UNREACH) != 0) {
			uint32_t uipr, ip;
			uint16_t uportr, port;

			uipr = wiz_creg_read32(ws, WIZ5500_CREG_UIPR);
			uportr = wiz_creg_read32(ws, WIZ5500_CREG_UPORTR);

			/*
			 * Find the offending socket(s). Yes, there could be
			 * more than one so check them all.
			 */
			s = ws->ws_socks;
			for (sn = 0; reg && sn < ws->ws_nsockets; sn++, s++) {
				if ((reg & s->s_mask) == 0 ||
				   (WIZ_SOCK_TYPE(s) != WIZ5x00_SREG_MR_P_TCP &&
				    WIZ_SOCK_TYPE(s) != WIZ5x00_SREG_MR_P_UDP)) {
					continue;
				}

				ip = wiz_sreg_read32(ws, s->s_sn,
				    WIZ5x00_SREG_DIPR);
				port = wiz_sreg_read16(ws, s->s_sn,
				    WIZ5x00_SREG_DPORT);

				if (ip == uipr && port == uportr) {
					wiz5x00_sock_callback_common(ws, s,
					    NETWORK_STATUS_UNREACHABLE);
				}
			}
		}
#endif

		while (reg) {
			s = ws->ws_socks;
			for (sn = 0; reg && sn < ws->ws_nsockets; sn++, s++) {
				if ((reg & s->s_mask) != 0) {
					reg &= ~(s->s_mask);
					wiz5x00_sock_interrupt(ws, s);
				}
			}

			reg = wiz_creg_read8(ws, ireg) & ws->ws_open_mask;
		}

		/* Check for pending UDP open */
		s = ws->ws_socks;
		for (sn = 0; ws->ws_udp_open_pending && sn < ws->ws_nsockets;
		    sn++, s++) {
			if ((ws->ws_udp_open_pending & s->s_mask) != 0) {
				ws->ws_udp_open_pending &= ~(s->s_mask);
				wiz5x00_sock_connect(ws, s);
			}
		}

		/* Flush pending Tx data, if necessary */
		s = ws->ws_socks;
		for (sn = 0; sn < ws->ws_nsockets; sn++, s++) {
			if (s->s_active == 0 || s->s_tx_pend == 0)
				continue;

			trd = wiz_sreg_read16(ws, s->s_sn, WIZ5x00_SREG_TX_RD);
			trd &= (s->s_tx_size - 1);
			if (trd == s->s_tx_rd)
				s->s_tx_pend = 0;
			else
			if (s->s_tx_pend)
				wiz_command(ws, s->s_sn, s->s_tx_pend, 0, 100);
		}

		(ws->ws_control)(ws->ws_cookie, WIZ5X00_CTL_IRQ_CTL, &enable);
	}
}

static void
wiz5x00_link_worker(void *arg)
{
	struct wiz_state *ws = arg;
	uint8_t phy, ls;

	phy = wiz_creg_read8(ws, WIZ_COMMON_PHY_REG(ws));
	ls = (phy & WIZ_COMMON_PHY_LINK(ws)) != 0;

	if ((ls ^ ws->ws_link_status) != 0) {
		/* Link change */
		network_link_status_t lst;
		uint16_t imr, imb;

		memset(&lst, 0, sizeof(lst));
		ws->ws_link_status = ls;
		imr = WIZ_COMMON_IMASK_REG(ws);
		imb = WIZ_COMMON_IMASK_BITS(ws);
		wiz_creg_write8(ws, WIZ5x00_CREG_IR, imb);

		if (ls) {
			lst.ls_state = NETWORK_LINK_UP;
#ifdef DEBUG_ENABLED
			DBFPRINTF("Link up");
#ifdef WIZ_5500
			if (ws->ws_chip == WIZ_CHIP_5500) {
				DBPRINTF(": %dMb/s %s duplex\n",
				    (phy & WIZ5500_CREG_PHY_SPD100) ? 100 : 10,
				    (phy & WIZ5500_CREG_PHY_DPX) ?
				    "full" : "half");
			} else
#endif
			DBPRINTF("\n");

#ifdef	WIZ_DUMP_REGS
			wiz_dump_cregs(ws);
			DBPRINTF("\n");
#endif
#endif /* DEBUG_ENABLED */
		} else {
			imb = 0;
			lst.ls_state = NETWORK_LINK_DOWN;
			DBFPRINTF("Link down\n");
#ifdef	WIZ_DUMP_REGS
			wiz_dump_cregs(ws);
			DBPRINTF("\n");
#endif
		}

		wiz_creg_write8(ws, imr, imb);

		if (ws->ws_link_cb)
			(ws->ws_link_cb)(&lst, ws->ws_link_cb_arg);
	}
}

static void
wiz5x00_get_mac(void *cookie, uint8_t *mac)
{
	struct wiz_state *ws = cookie;

	memcpy(mac, ws->ws_mac, sizeof(ws->ws_mac));
}

static void
wiz5x00_set_ip_params(void *cookie, const network_ip_params_t *ip)
{
	struct wiz_state *ws = cookie;

	if (ip->ip_addr != 0 && memcmp(ip, &ws->ws_ip, sizeof(*ip)) == 0)
		return;

	if (ip->ip_addr != ws->ws_ip.ip_addr ||
	    ip->ip_mask != ws->ws_ip.ip_mask) {
		struct wiz_socket *s;
		uint8_t i;

#ifdef DEBUG_ENABLED
		char ipstr[NETWORK_IP_STR_LEN];

		if (ws->ws_ip.ip_addr == 0) {
			ipstr[0] = '0';
			ipstr[1] = '\0';
		} else
			network_ip2str(ws->ws_ip.ip_addr, ipstr);
		printf("IP change %s/", ipstr);
		network_ip2str(ws->ws_ip.ip_mask, ipstr);
		printf("%s -> ", ipstr);
		network_ip2str(ip->ip_addr, ipstr);
		printf("%s/", ipstr);
		network_ip2str(ip->ip_mask, ipstr);
		printf("%s\n", ipstr);
#endif /* DEBUG_ENABLED */

		s = ws->ws_socks;
		for (i = 0; i < ws->ws_nsockets; i++, s++) {
			if ((ws->ws_open_mask & s->s_mask) != 0)
				wiz5x00_sock_disconnect(ws, s);
		}
	}

	ws->ws_ip = *ip;

	wiz_creg_write32(ws, WIZ5x00_CREG_SIPR, ip->ip_addr);
	wiz_creg_write32(ws, WIZ5x00_CREG_SUBR, ip->ip_mask);
	wiz_creg_write32(ws, WIZ5x00_CREG_GAR, ip->ip_gate);
}

static network_sock_t
wiz5x00_open(void *cookie, network_sock_type_t stype,
    network_sock_status_callback_t func, void *arg)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s;
	uint8_t sn, m, st;

	if (ws->ws_link_status == 0)
		return NETWORK_SOCKET_INVALID;

	switch (stype) {
	case NETWORK_SOCK_TYPE_TCP:
		st = WIZ5x00_SREG_MR_P_TCP;
		break;
	case NETWORK_SOCK_TYPE_UDP:
		st = WIZ5x00_SREG_MR_P_UDP;
		break;
	default:
		return NETWORK_SOCKET_INVALID;
	}

	if (ws->ws_chip == WIZ_CHIP_5200)
		st |= WIZ5200_SREG_MR_MF;
	m = ws->ws_open_mask >> 1;
	for (sn = 1; sn < ws->ws_nsockets; sn++, m >>= 1) {
		if ((m & 1) == 0)
			break;
	}

	if (sn == ws->ws_nsockets) {
		if ((ws->ws_open_mask & 1) == 0) {
			/* Use socket 0 as a last resort */
			sn = 0;
		} else {
			return NETWORK_SOCKET_INVALID;
		}
	}

	s = &ws->ws_socks[sn];
	s->s_type = st;
	s->s_status_callback = func;
	s->s_arg = arg;
	s->s_active = 0;
	ws->ws_open_mask |= s->s_mask;
	ws->ws_udp_open_pending &= ~(s->s_mask);

	return s;
}

static void
wiz5x00_close(void *cookie, network_sock_t sock)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	uint8_t rv;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0) {
		return;
	}

	if (s->s_active) {
		if (WIZ_SOCK_TYPE(s) == WIZ5x00_SREG_MR_P_TCP) {
			rv = wiz_command(ws, s->s_sn, WIZ5x00_SREG_CR_DISCON,
			    WIZ5x00_SREG_SR_CLOSED, 200);
			if (rv == 0) {
				/* Hmmm, what to do here? */
			}
		}

		(void) wiz_command(ws, s->s_sn, WIZ5x00_SREG_CR_CLOSE,
		    WIZ5x00_SREG_SR_CLOSED, 200);
	}

	ws->ws_open_mask &= ~(s->s_mask);
	ws->ws_udp_open_pending &= ~(s->s_mask);

	if (s->s_active) {
		wiz_creg_write8(ws, WIZ_SOCKET_IMASK_REG(ws),
		    ws->ws_open_mask);
		wiz_sreg_write8(ws, s->s_sn, WIZ5x00_SREG_IMR, 0);
	}

	s->s_active = 0;
	s->s_tx_pend = 0;
}

static uint8_t
wiz5x00_sock_init(struct wiz_state *ws, struct wiz_socket *s,
    const network_sock_params_t *sp)
{
	uint8_t m, expect;
	uint8_t sn, st;

	sn = s->s_sn;
	st = WIZ_SOCK_TYPE(s);

	wiz_sreg_write8(ws, sn, WIZ5x00_SREG_MR, s->s_type);

	if (st == WIZ5x00_SREG_MR_P_TCP || st == WIZ5x00_SREG_MR_P_UDP) {
		wiz_sreg_write16(ws, sn, WIZ5x00_SREG_MSSR, 1460u);
		wiz_sreg_write16(ws, sn, WIZ5x00_SREG_PORT, sp->sp_sport);
		wiz_sreg_write16(ws, sn, WIZ5x00_SREG_DPORT, sp->sp_dport);
	}
	wiz_sreg_write32(ws, sn, WIZ5x00_SREG_DIPR, sp->sp_dest_ip);

	m = WIZ5x00_SREG_IR_SEND_OK | WIZ5x00_SREG_IR_RECV;
	switch (st) {
	case WIZ5x00_SREG_MR_P_TCP:
		expect = WIZ5x00_SREG_SR_INIT;
		m |= WIZ5x00_SREG_IR_CON | WIZ5x00_SREG_IR_DISCON |
		    WIZ5x00_SREG_IR_TIMEOUT;
		break;
	case WIZ5x00_SREG_MR_P_UDP:
		expect = WIZ5x00_SREG_SR_UDP;
		break;
	default:
		expect = WIZ5x00_SREG_SR_MACRAW;
		break;
	}

	if (wiz_command(ws, sn, WIZ5x00_SREG_CR_OPEN, expect, 100) == 0) {
		ws->ws_open_mask &= ~(s->s_mask);
		wiz_sreg_write8(ws, sn, WIZ5x00_SREG_MR,
		    WIZ5x00_SREG_MR_P_CLOSED);
		return 0;
	}

	wiz_sreg_write8(ws, sn, WIZ5x00_SREG_IMR, m);
	wiz_creg_write8(ws, WIZ_SOCKET_IMASK_REG(ws), ws->ws_open_mask);

	return 1;
}

static uint8_t
wiz5x00_connect(void *cookie, network_sock_t sock,
    const network_sock_params_t *sp)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return 0;

	if (wiz5x00_sock_init(ws, s, sp) == 0)
		return 0;

	if (WIZ_SOCK_TYPE(s) == WIZ5x00_SREG_MR_P_TCP) {
		wiz_sreg_write8(ws, s->s_sn, WIZ5x00_SREG_CR,
		    WIZ5x00_SREG_CR_CONNECT);
	} else
	if (WIZ_SOCK_TYPE(s) == WIZ5x00_SREG_MR_P_UDP) {
		ws->ws_udp_open_pending |= s->s_mask;
		ws->ws_irq_pending = 1;
		network_driver_soft_timeout(ws->ws_network_state, 0);
	}

	s->s_active = 1;

	return 1;
}

static uint8_t
wiz5x00_listen(void *cookie, network_sock_t sock, uint16_t sport)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	network_sock_params_t sp;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return 0;

	if (WIZ_SOCK_TYPE(s) != WIZ5x00_SREG_MR_P_TCP)
		return 0;

	sp.sp_dest_ip = 0;
	sp.sp_sport = sport;
	sp.sp_dport = 0;

	if (wiz5x00_sock_init(ws, s, &sp) == 0)
		return 0;

	wiz_sreg_write8(ws, s->s_sn, WIZ5x00_SREG_CR, WIZ5x00_SREG_CR_LISTEN);
	s->s_active = 1;

	return 1;
}

static uint16_t
wiz5x00_read(void *cookie, network_sock_t sock, void *buff, uint16_t len)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	uint16_t tl, rsr, rd, src;
	uint16_t rv;
	uint8_t sn, sr;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	if (WIZ_SOCK_TYPE(s) != WIZ5x00_SREG_MR_P_TCP)
		return NETWORK_SOCK_ERR_BAD_CALL;

	sn = s->s_sn;

	if ((rsr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RSR)) == 0)
		return 0;

	rv = 0;

	while (len) {
		if (rsr == 0) {
			sr = wiz_sreg_read8(ws, sn, WIZ5x00_SREG_SR);
			switch (sr) {
			case WIZ5x00_SREG_SR_CLOSED:
			case WIZ5x00_SREG_SR_LISTEN:
			case WIZ5x00_SREG_SR_CLOSE_WAIT:
				if (rv == 0)
					rv = NETWORK_SOCK_ERR_EOF;
				break;
			}
			break;
		}
		if (rsr > len)
			rsr = len;

		rd = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RD);
		src = rd & (s->s_rx_size - 1);

		if ((src + rsr) > s->s_rx_size) {
			tl = s->s_rx_size - src;
			wiz_memory_read(ws, s->s_rxptr, src, buff, tl);
			buff = (void *)(((uint8_t *)buff) + tl);
			tl = rsr - tl;
			wiz_memory_read(ws, s->s_rxptr, 0, buff, tl);
			buff = (void *)(((uint8_t *)buff) + tl);
		} else {
			wiz_memory_read(ws, s->s_rxptr, src, buff, rsr);
			buff = (void *)(((uint8_t *)buff) + rsr);
		}

		len -= rsr;
		rv += rsr;
		wiz_sreg_write16(ws, sn, WIZ5x00_SREG_RX_RD, rd + rsr);
		wiz_command(ws, sn, WIZ5x00_SREG_CR_RECV, 0, 100);
		rsr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RSR);
	}

	return rv;
}

static void
wiz5x00_read_data(struct wiz_state *ws, struct wiz_socket *s, uint16_t rd,
    void *buff, uint16_t len)
{
	uint16_t tl, src;
	uint8_t *p = buff;

	src = rd & (s->s_rx_size - 1);

	if ((src + len) > s->s_rx_size) {
		tl = s->s_rx_size - src;
		wiz_memory_read(ws, s->s_rxptr, src, p, tl);
		p += tl;
		tl = len - tl;
		wiz_memory_read(ws, s->s_rxptr, 0, p, tl);
	} else {
		wiz_memory_read(ws, s->s_rxptr, src, p, len);
	}
}

static uint16_t
wiz5x00_recv(void *cookie, network_sock_t sock, void *buff, uint16_t len,
    network_sock_params_t *sp)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	struct wiz_udp_packet_header wup;
	uint16_t rsr, rd;
	uint8_t sn;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	if (WIZ_SOCK_TYPE(s) != WIZ5x00_SREG_MR_P_UDP)
		return NETWORK_SOCK_ERR_BAD_CALL;

	sn = s->s_sn;

	if ((rsr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RSR)) == 0)
		return 0;

	if (rsr < sizeof(struct wiz_udp_packet_header))
		return NETWORK_SOCK_ERR_EOF;

	rd = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RD);

	wiz5x00_read_data(ws, s, rd, &wup, sizeof(wup));
	if (sp) {
		sp->sp_dest_ip = ntoh32(wup.wup_peer_ip);
		sp->sp_dport = ntoh16(wup.wup_peer_port);
	}

	wup.wup_plen = ntoh16(wup.wup_plen);
	if (len > wup.wup_plen)
		len = wup.wup_plen;

	wiz5x00_read_data(ws, s, rd + sizeof(wup), buff, len);

	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_RX_RD, rd + rsr);
	wiz_command(ws, sn, WIZ5x00_SREG_CR_RECV, 0, 100);

	return len;
}

static uint16_t
wiz5x00_rx_available(void *cookie, network_sock_t sock)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	uint16_t rsr;
	uint8_t sr, sn;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	sn = s->s_sn;

	rsr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_RX_RSR);
	if (rsr == 0) {
		sr = wiz_sreg_read8(ws, sn, WIZ5x00_SREG_SR);
		switch (sr) {
		case WIZ5x00_SREG_SR_CLOSED:
		case WIZ5x00_SREG_SR_LISTEN:
		case WIZ5x00_SREG_SR_CLOSE_WAIT:
			rsr = NETWORK_SOCK_ERR_EOF;
			break;

		default:
			break;
		}
	} else
	if (WIZ_SOCK_TYPE(s) == WIZ5x00_SREG_MR_P_UDP) {
		if (rsr < sizeof(struct wiz_udp_packet_header))
			rsr = NETWORK_SOCK_ERR_EOF;
		else
			rsr -= sizeof(struct wiz_udp_packet_header);
	}

	return rsr;
}

static uint16_t
wiz5x00_write_data(struct wiz_state *ws, struct wiz_socket *s, uint16_t wr,
    const void *buff, uint16_t len)
{
	uint16_t dst = wr & (s->s_tx_size - 1u);

	if ((dst + len) > s->s_tx_size) {
		uint16_t tl = s->s_tx_size - dst;
		wiz_memory_write(ws, s->s_txptr, dst, buff, tl);
		buff = (const void *)(((const uint8_t *)buff) + tl);
		tl = len - tl;
		if (tl)
			wiz_memory_write(ws, s->s_txptr, 0, buff, tl);
	} else {
		wiz_memory_write(ws, s->s_txptr, dst, buff, len);
	}

	return wr + len;
}

static uint16_t
wiz5x00_write(void *cookie, network_sock_t sock, const void *buff, uint16_t len,
    const uint8_t *dstmac)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	uint16_t rd, fsr, wr;
	uint8_t cmd, sn;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	if (len == 0)
		return 0;

	if (s->s_tx_pend)
		return NETWORK_SOCK_ERR_BUSY;

	sn = s->s_sn;

/* XXX: Check socket status */

	fsr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_FSR);
	if (fsr < len)
		len = fsr;

	wr = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_WR);
	wr = wiz5x00_write_data(ws, s, wr, buff, len);

	s->s_tx_rd = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_RD);
	s->s_tx_rd += len;
	s->s_tx_rd &= (s->s_tx_size - 1);

	wiz_sreg_write16(ws, sn, WIZ5x00_SREG_TX_WR, wr);

	if (WIZ_SOCK_TYPE(s) == WIZ5x00_SREG_MR_P_UDP && dstmac != NULL) {
		wiz_smemory_write(ws, sn, WIZ5x00_SREG_DHAR, dstmac, 6);
		cmd = WIZ5x00_SREG_CR_SEND_MAC;
	} else {
		cmd = WIZ5x00_SREG_CR_SEND;
	}

	wiz_command(ws, sn, cmd, 0, 100);

	rd = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_TX_RD) & (s->s_tx_size - 1);
	if (rd != s->s_tx_rd)
		s->s_tx_pend = cmd;
	else
		s->s_tx_pend = 0;

	return len;
}

static uint16_t
wiz5x00_tx_free(void *cookie, network_sock_t sock)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	uint16_t fsr;

	if (sock == NETWORK_SOCKET_INVALID ||
	    (ws->ws_open_mask & s->s_mask) == 0)
		return NETWORK_SOCK_ERR_NOT_OPEN;

	if (s->s_tx_pend)
		return 0;

/* XXX: Check socket status */

	fsr = wiz_sreg_read16(ws, s->s_sn, WIZ5x00_SREG_TX_FSR);

	return fsr;
}

static void
wiz5x00_link_status(void *cookie, network_link_status_callback_t cb, void *arg)
{
	struct wiz_state *ws = cookie;

	ws->ws_link_cb = cb;
	ws->ws_link_cb_arg = arg;
}

static network_sock_state_t
wiz5x00_sock_status(void *cookie, network_sock_t sock,
    network_sock_params_t *sp)
{
	struct wiz_state *ws = cookie;
	struct wiz_socket *s = sock;
	network_sock_state_t st;
	uint8_t sr, stype, sn;

	memset(sp, 0, sizeof(*sp));
	sp->sp_local_ip = ws->ws_ip.ip_addr;

	sn = s->s_sn;
	sr = wiz_sreg_read8(ws, sn, WIZ5x00_SREG_SR);
	stype = WIZ_SOCK_TYPE(s);

	if (stype == WIZ5x00_SREG_MR_P_TCP || stype == WIZ5x00_SREG_MR_P_UDP)
		sp->sp_sport = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_PORT);

	if (stype == WIZ5x00_SREG_MR_P_TCP) {
		switch (sr) {
		case WIZ5x00_SREG_SR_ESTABLISHED:
		case WIZ5x00_SREG_SR_CLOSE_WAIT:
		case WIZ5x00_SREG_SR_SYNSENT:
		case WIZ5x00_SREG_SR_SYNRECV:
		case WIZ5x00_SREG_SR_FIN_WAIT:
		case WIZ5x00_SREG_SR_CLOSING:
		case WIZ5x00_SREG_SR_TIME_WAIT:
		case WIZ5x00_SREG_SR_LAST_ACK:
			sp->sp_dest_ip = wiz_sreg_read32(ws, sn,
			    WIZ5x00_SREG_DIPR);
			sp->sp_dport = wiz_sreg_read16(ws, sn,
			    WIZ5x00_SREG_DPORT);
			break;

		default:
			break;
		}
	} else
	if (stype == WIZ5x00_SREG_MR_P_UDP && sr == WIZ5x00_SREG_SR_UDP) {
		sp->sp_dest_ip = wiz_sreg_read32(ws, sn, WIZ5x00_SREG_DIPR);
		sp->sp_dport = wiz_sreg_read16(ws, sn, WIZ5x00_SREG_DPORT);
	}

	switch (sr) {
	case WIZ5x00_SREG_SR_LISTEN:
		st = NETWORK_SOCKS_LISTEN;
		break;
	case WIZ5x00_SREG_SR_ESTABLISHED:
		st = NETWORK_SOCKS_ESTABLISHED;
		break;
	case WIZ5x00_SREG_SR_CLOSE_WAIT:
		st = NETWORK_SOCKS_CLOSE_WAIT;
		break;
	case WIZ5x00_SREG_SR_SYNSENT:
		st = NETWORK_SOCKS_SYNSENT;
		break;
	case WIZ5x00_SREG_SR_SYNRECV:
		st = NETWORK_SOCKS_SYNRECV;
		break;
	case WIZ5x00_SREG_SR_FIN_WAIT:
		st = NETWORK_SOCKS_FIN_WAIT;
		break;
	case WIZ5x00_SREG_SR_CLOSING:
		st = NETWORK_SOCKS_CLOSING;
		break;
	case WIZ5x00_SREG_SR_TIME_WAIT:
		st = NETWORK_SOCKS_TIME_WAIT;
		break;
	case WIZ5x00_SREG_SR_LAST_ACK:
		st = NETWORK_SOCKS_LAST_ACK;
		break;
	case WIZ5x00_SREG_SR_UDP:
		st = NETWORK_SOCKS_UDP;
		break;
	default:
		st = NETWORK_SOCKS_NONE;
		break;
	}

	return st;
}

#ifdef NETWORK_LWIP
static void
wiz5x00_if_link_worker(struct netif *netif)
{

	wiz5x00_link_worker(netif->state);
}

static void
wiz5x00_if_link_status(struct netif *netif, network_link_status_callback_t cb,
    void *arg)
{
	struct wiz_if_state *wis = netif->state;

	wis->wis_link_cb = cb;
	wis->wis_link_cb_arg = arg;
}

static void
wiz5x00_if_get_mac(struct netif *netif, uint8_t *mac)
{

	memcpy(mac, netif->hwaddr, ETH_HWADDR_LEN);
}

static int8_t
wiz5x00_if_start_stop(struct netif *netif, int8_t do_start)
{
	struct wiz_if_state *wis = netif->state;
	struct wiz_state *ws = &wis->wis_ws;
	struct wiz_socket *s = ws->ws_socks;
	struct wiz_pbuf *wp;

	if (do_start && (ws->ws_open_mask & s->s_mask) == 0) {
		STAILQ_INIT(&wis->wis_tx_queue);
		STAILQ_INIT(&wis->wis_free_queue);

		s->s_type = WIZ5x00_SREG_MR_P_MACRAW;
		if (ws->ws_chip == WIZ_CHIP_5200)
			s->s_type |= WIZ5200_SREG_MR_MF;
		else
			s->s_type |= WIZ5500_SREG_MR_MFEN;
		wiz_sreg_write8(ws, 0, WIZ5x00_SREG_MR, s->s_type);

		if (wiz_command(ws, 0, WIZ5x00_SREG_CR_OPEN,
		    WIZ5x00_SREG_SR_MACRAW, 100) == 0) {
			wiz_sreg_write8(ws, 0, WIZ5x00_SREG_MR,
			    WIZ5x00_SREG_MR_P_CLOSED);
			return -1;
		}

		ws->ws_open_mask |= s->s_mask;
		wiz_creg_write8(ws, WIZ_SOCKET_IMASK_REG(ws), ws->ws_open_mask);
		wiz_sreg_write8(ws, 0, WIZ5x00_SREG_IMR,
		    WIZ5x00_SREG_IR_SEND_OK | WIZ5x00_SREG_IR_RECV);

		netif_set_up(netif);
	} else
	if (do_start == 0 && (ws->ws_open_mask & s->s_mask) != 0) {
		(void) wiz_command(ws, 0, WIZ5x00_SREG_CR_CLOSE,
		    WIZ5x00_SREG_SR_CLOSED, 200);
		wiz_creg_write8(ws, WIZ_SOCKET_IMASK_REG(ws),
		    ws->ws_open_mask);
		wiz_sreg_write8(ws, 0, WIZ5x00_SREG_IMR, 0);
		ws->ws_open_mask &= ~(s->s_mask);

		/* Discard any queued Tx pbufs */
		while ((wp = STAILQ_FIRST(&wis->wis_tx_queue)) != NULL) {
			STAILQ_REMOVE_HEAD(&wis->wis_tx_queue, wp_qent);
			assert(wp->wp_pbuf != NULL);
			pbuf_free(wp->wp_pbuf);
			wp->wp_pbuf = NULL;
			STAILQ_INSERT_TAIL(&wis->wis_free_queue, wp, wp_qent);
		}

		/* Now free the free list */
		while ((wp = STAILQ_FIRST(&wis->wis_free_queue)) != NULL) {
			STAILQ_REMOVE_HEAD(&wis->wis_free_queue, wp_qent);
			zone_free(wp);
		}

		netif_set_down(netif);
	}

	return 0;
}

static uint16_t
wiz5x00_if_rx(struct wiz_if_state *wis)
{
	struct wiz_macraw_packet_header wmph;
	struct wiz_state *ws = &wis->wis_ws;
	struct wiz_socket *s = ws->ws_socks;
	struct pbuf *p, *q;
	uint16_t tot_len, rsr, rd;

	if ((rsr = wiz_sreg_read16(ws, 0, WIZ5x00_SREG_RX_RSR)) == 0)
		return 0;

	rd = wiz_sreg_read16(ws, 0, WIZ5x00_SREG_RX_RD);
	wiz5x00_read_data(ws, s, rd, &wmph, sizeof(wmph));
	rd += sizeof(wmph);
	rsr -= sizeof(wmph);
	wmph.wmph_plen = ntoh16(wmph.wmph_plen);
	tot_len = wmph.wmph_plen - sizeof(wmph);

	p = pbuf_alloc(PBUF_RAW, tot_len + ETH_PAD_SIZE, PBUF_POOL);
	if (p) {
#if ETH_PAD_SIZE
		pbuf_remove_header(p, ETH_PAD_SIZE);
#endif
		for (q = p; q != NULL; q = q->next) {
			wiz5x00_read_data(ws, s, rd, q->payload, q->len);
			rd += q->len;	/* Wrap-around is ok */
			rsr -= q->len;
		}

#if ETH_PAD_SIZE
		pbuf_add_header(p, ETH_PAD_SIZE);
#endif

		if (wis->wis_netif.input(p, &wis->wis_netif) != ERR_OK)
			pbuf_free(p);
	} else {
		rd += tot_len;
		rsr -= tot_len;
	}

	wiz_sreg_write16(ws, 0, WIZ5x00_SREG_RX_RD, rd);
	wiz_command(ws, 0, WIZ5x00_SREG_CR_RECV, 0, 100);

	return rsr;
}

static void
wiz5x00_if_tx(struct wiz_if_state *wis, struct pbuf *p)
{
	struct wiz_state *ws = &wis->wis_ws;
	struct wiz_socket *s = ws->ws_socks;
	struct pbuf *q;
	uint16_t wr;

#if ETH_PAD_SIZE
	pbuf_remove_header(p, ETH_PAD_SIZE);
#endif

	if (p->tot_len < wiz_sreg_read16(ws, 0, WIZ5x00_SREG_TX_FSR)) {
		wr = wiz_sreg_read16(ws, 0, WIZ5x00_SREG_TX_WR);

		for (q = p; q != NULL; q = q->next)
			wr = wiz5x00_write_data(ws, s, wr, q->payload, q->len);

		wiz_sreg_write16(ws, 0, WIZ5x00_SREG_TX_WR, wr);
		wiz_command(ws, 0, WIZ5x00_SREG_CR_SEND, 0, 100);
		ws->ws_tx_busy = 1;
	}

#if ETH_PAD_SIZE
	pbuf_add_header(p, ETH_PAD_SIZE);
#endif

	pbuf_free(p);
}

static void
wiz5x00_if_link_callback(const network_link_status_t *ls, void *arg)
{
	struct wiz_if_state *wis = arg;

	if (wis->wis_link_cb != NULL)
		wis->wis_link_cb(ls, wis->wis_link_cb_arg);

	if (ls->ls_state == NETWORK_LINK_DOWN)
		netif_set_link_down(&wis->wis_netif);
	else
	if (ls->ls_state == NETWORK_LINK_UP)
		netif_set_link_up(&wis->wis_netif);
}

static uint32_t
wiz5x00_if_worker(struct netif *netif)
{
	struct wiz_if_state *wis = netif->state;
	struct wiz_state *ws = &wis->wis_ws;
	uint8_t reg, irq;
	struct wiz_pbuf *wp;

	for (;;) {
		(ws->ws_control)(ws->ws_cookie, WIZ5X00_CTL_IRQ_STATUS, &irq);

		if (ws->ws_irq_pending == 0 && irq == 0)
			break;

		ws->ws_irq_pending = 0;

		/*
		 * In i/f mode, there's nothing of interest in the IR
		 * register, so just clear any bits we find set.
		 */
		reg = wiz_creg_read8(ws, WIZ5x00_CREG_IR);
		if ((reg & WIZ_COMMON_IMASK_BITS(ws)) != 0)
			wiz_creg_write8(ws, WIZ5x00_CREG_IR, reg);

		reg = wiz_sreg_read8(ws, 0, WIZ5x00_SREG_IR);
		wiz_sreg_write8(ws, 0, WIZ5x00_SREG_IR, reg);

		if (reg & WIZ5x00_SREG_IR_RECV) {
			/* Packet received. */
			while (wiz5x00_if_rx(wis) != 0)
				;
		}

		if (reg & WIZ5x00_SREG_IR_SEND_OK) {
			/* Packet transmitted. */
			ws->ws_tx_busy = 0;
		}

		if (ws->ws_tx_busy == 0 &&
		    (wp = STAILQ_FIRST(&wis->wis_tx_queue)) != NULL) {
			STAILQ_REMOVE_HEAD(&wis->wis_tx_queue, wp_qent);
			wiz5x00_if_tx(wis, wp->wp_pbuf);
			wp->wp_pbuf = NULL;
			STAILQ_INSERT_TAIL(&wis->wis_free_queue, wp, wp_qent);
		}
	}

	irq = 1;
	(ws->ws_control)(ws->ws_cookie, WIZ5X00_CTL_IRQ_CTL, &irq);

	return RTOS_MAX_SLEEP_MS;
}

static err_t
wiz5x00_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct wiz_if_state *wis = netif->state;
	struct wiz_state *ws = &wis->wis_ws;
	struct wiz_pbuf *wp;

	pbuf_ref(p);

	if (ws->ws_tx_busy == 0 && STAILQ_EMPTY(&wis->wis_tx_queue)) {
		wiz5x00_if_tx(wis, p);
		return ERR_OK;
	}

	if ((wp = STAILQ_FIRST(&wis->wis_free_queue)) != NULL) {
		STAILQ_REMOVE_HEAD(&wis->wis_free_queue, wp_qent);
	} else {
		wp = zone_malloc(sizeof(*wp));
		if (wp == NULL) {
			pbuf_free(p);
			return ERR_MEM;
		}
	}

	wp->wp_pbuf = p;
	STAILQ_INSERT_TAIL(&wis->wis_tx_queue, wp, wp_qent);

	return ERR_OK;
}

static err_t
wiz5x00_netif_init(struct netif *netif)
{
	struct wiz_if_state *wis = netif->state;

#if LWIP_NETIF_HOSTNAME
	netif->hostname = "maven";
#endif

	netif->name[0] = 'm';
	netif->name[1] = 'e';
	netif->output = etharp_output;
	netif->linkoutput = wiz5x00_low_level_output;
	memcpy(netif->hwaddr, wis->wis_ws.ws_mac, ETH_HWADDR_LEN);
	netif->hwaddr_len = ETH_HWADDR_LEN;
	netif->mtu = 1500;
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
	    NETIF_FLAG_ETHERNET;

	wiz_reset(&wis->wis_ws);
	wis->wis_ws.ws_link_status = 0;

	return ERR_OK;
}
#endif /* NETWORK_LWIP */

static int
wiz5x00_attach_common(struct wiz_state *ws,
    const struct wiz5x00_attach_args *waa, u_int nsocks)
{

	switch (waa->waa_chip) {
#ifdef WIZ_5200
	case WIZ_CHIP_5200:
#endif
#ifdef WIZ_5500
	case WIZ_CHIP_5500:
#endif
		break;
	
	default:
		DBFPRINTF("unsupported chip\n");
		return -1;
	}

	ws->ws_chip = waa->waa_chip;
	ws->ws_read_write = waa->waa_read_write;
	ws->ws_control = waa->waa_control;
	ws->ws_cookie = waa->waa_cookie;
	memcpy(ws->ws_mac, waa->waa_mac, sizeof(ws->ws_mac));
	ws->ws_nsockets = nsocks;

	ws->ws_socks = zone_calloc(nsocks, sizeof(*ws->ws_socks));
	if (ws->ws_socks == NULL) {
		DBFPRINTF("out of memory for socks!\n");
		return -1;
	}

	ws->ws_open_mask = 0;
	ws->ws_udp_open_pending = 0;

	return 0;
}

void
wiz5x00_hw_interrupt(void *cookie)
{
	struct wiz_state *ws = cookie;
	uint8_t enable = 0;

	ws->ws_irq_pending = 1;
	(ws->ws_control)(ws->ws_cookie, WIZ5X00_CTL_IRQ_CTL, &enable);
	network_driver_hw_interrupt(ws->ws_network_state);
}

void *
wiz5x00_attach(const struct wiz5x00_attach_args *waa)
{
	struct network_driver *nd;
	struct wiz_state *ws;

	if ((ws = zone_calloc(1, sizeof(*ws))) == NULL)
		return NULL;

	if (wiz5x00_attach_common(ws, waa, WIZ5x00_NSOCKETS) < 0) {
		zone_free(ws);
		return NULL;
	}

	if ((nd = zone_calloc(1, sizeof(*nd))) == NULL) {
		zone_free(ws);
		DBFPRINTF("out of memory!\n");
		return NULL;
	}

	nd->nd_cookie = ws;
	nd->nd_worker = wiz5x00_worker;
	nd->nd_link_check = wiz5x00_link_worker;
	nd->nd_link_status = wiz5x00_link_status;
	nd->nd_get_mac = wiz5x00_get_mac;
	nd->nd_ioctl = NULL;

	nd->nd_open = wiz5x00_open;
	nd->nd_close = wiz5x00_close;
	nd->nd_connect = wiz5x00_connect;
	nd->nd_listen = wiz5x00_listen;
	nd->nd_read = wiz5x00_read;
	nd->nd_recv = wiz5x00_recv;
	nd->nd_rx_avail = wiz5x00_rx_available;
	nd->nd_write = wiz5x00_write;
	nd->nd_tx_free = wiz5x00_tx_free;
	nd->nd_sock_status = wiz5x00_sock_status;
	nd->nd_set_ip_params = wiz5x00_set_ip_params;

	wiz_reset(ws);
	ws->ws_link_status = 0;

#ifdef	WIZ_DUMP_REGS
	wiz_dump_cregs(ws);
	putchar('\n');
	wiz_dump_sregs(ws, ws->ws_socks);
#endif

	ws->ws_network_state = network_register(nd);
	if (ws->ws_network_state == NULL) {
		zone_free(ws->ws_socks);
		zone_free(nd);
		zone_free(ws);
	}

	return ws;
}

#ifdef NETWORK_LWIP
void *
wiz5x00_lwip_attach(const struct wiz5x00_attach_args *waa)
{
	struct network_lwip_driver *nld;
	struct wiz_if_state *wis;

	if ((wis = zone_calloc(1, sizeof(*wis))) == NULL)
		return NULL;

	if (wiz5x00_attach_common(&wis->wis_ws, waa, 1) < 0) {
		zone_free(wis);
		return NULL;
	}

	if ((nld = zone_calloc(1, sizeof(*nld))) == NULL) {
		zone_free(wis);
		DBFPRINTF("out of memory!\n");
		return NULL;
	}

	nld->nld_netif = &wis->wis_netif;
	nld->nld_handle_interrupt = wiz5x00_if_worker;
	nld->nld_link_check = wiz5x00_if_link_worker;
	nld->nld_link_status = wiz5x00_if_link_status;
	nld->nld_get_mac = wiz5x00_if_get_mac;
	nld->nld_ioctl = NULL;

	wis->wis_ws.ws_link_cb = wiz5x00_if_link_callback;
	wis->wis_ws.ws_link_cb_arg = wis;

	if (netif_add(&wis->wis_netif, NULL, NULL, NULL, wis,
	    wiz5x00_netif_init, netif_input) == NULL) {
#ifndef NDEBUG
		printf("wiz5x00_lwip_attach: netif_add() failed\n");
		return NULL;
#endif
	}
	netif_set_default(&wis->wis_netif);

#ifdef	WIZ_DUMP_REGS
	wiz_dump_cregs(&wis->wis_ws);
	putchar('\n');
	wiz_dump_sregs(&wis->wis_ws, wis->wis_ws.ws_socks);
#endif

	if (wiz5x00_if_start_stop(&wis->wis_netif, 1) < 0 ||
	    (wis->wis_ws.ws_network_state = network_lwip_attach(nld)) == NULL) {
		DBFPRINTF("bogons at 12 o'clock!\n");
		(void) wiz5x00_if_start_stop(&wis->wis_netif, 0);
		netif_remove(&wis->wis_netif);
		zone_free(wis->wis_ws.ws_socks);
		zone_free(nld);
		zone_free(wis);
		wis = NULL;
	}

	return wis;
}
#endif /* NETWORK_LWIP */
