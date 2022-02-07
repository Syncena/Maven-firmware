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

#include <sys/types.h>

#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "sam_gmac.h"
#include "rtos.h"
#include "timer.h"
#include "network.h"
#include "network_lwip.h"
#include "linked-lists.h"
#include "zone_alloc.h"

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "netif/etharp.h"

/*
 * LWIP driver for the Ethernet controller found on SAME5x and SAME7x
 * microcontrollers. Define GMAC_IS_SAME5X=1 for SAME5x support. Otherwise
 * SAME7x is assumed.
 *
 * TODO, mostly in priority order:
 *
 *  - PHY Interrupts.
 *  - Checksum offload.
 *  - Tx watchdog.
 */

_Static_assert(ETH_PAD_SIZE >= 0 && ETH_PAD_SIZE <= 3, "Bad ETH_PAD_SIZE");

#define	GMAC_DEBUG_TX		(1u << 0)
#define	GMAC_DEBUG_TX_REAP	(1u << 1)
#define	GMAC_DEBUG_TX_START	(1u << 2)
#define	GMAC_DEBUG_RX		(1u << 3)
#define	GMAC_DEBUG_ERROR	(1u << 4)
#define	GMAC_DEBUG_LINK		(1u << 5)
#define	GMAC_DEBUG_WORKER	(1u << 6)
#define	GMAC_DEBUG_START	(1u << 7)
#define	GMAC_DEBUG_STOP		(1u << 8)
#define	GMAC_DEBUG_RESET	(1u << 9)
#define	GMAC_DEBUG_HW_CHECKSUM	(1u << 10)
#define	GMAC_DEBUG_ATTACH	(1u << 11)

#if 0
#define GMAC_DEBUG	(GMAC_DEBUG_TX | GMAC_DEBUG_TX_REAP | GMAC_DEBUG_TX_START |\
			 GMAC_DEBUG_RX | GMAC_DEBUG_ERROR | GMAC_DEBUG_WORKER)
#endif
#ifdef GMAC_DEBUG
static uint32_t gmac_debug = GMAC_DEBUG;
#define DBPRINTF(lvl, fmt, ...)				\
	do {						\
		if (gmac_debug & (lvl)) {		\
			printf(fmt, ##__VA_ARGS__);	\
		}					\
	} while (0)
#else
#define DBPRINTF(x, ...)	do { } while (0)
#endif

#ifdef GMAC_IS_SAME5X
/*
 * The GMAC on SAM[DE]5x is a subset of that found on SAME7x. The primary
 * difference, from a functional point of view, is the absence of priority
 * queues on the former.
 *
 * However, Atmel/Microchip have made a major faux-pas in the header files
 * for the two devices - the register definitions are significantly
 * different. Fortunately, most of the damage can be fixed by defining
 * a few things here...
 */
#define	GMAC_NCR			NCR.reg
#define	 GMAC_NCR_TSTART_Msk		GMAC_NCR_TSTART
#define	 GMAC_NCR_CLRSTAT_Msk		GMAC_NCR_CLRSTAT
#define	 GMAC_NCR_MPE_Msk		GMAC_NCR_MPE
#define	 GMAC_NCR_TXEN_Msk		GMAC_NCR_TXEN
#define	 GMAC_NCR_RXEN_Msk		GMAC_NCR_RXEN
#define	 GMAC_NCR_FNP_Msk		GMAC_NCR_FNP
#define	 GMAC_NCR_THALT_Msk		GMAC_NCR_THALT
#define	GMAC_NCFGR			NCFGR.reg
#define	 GMAC_NCFGR_LFERD_Msk		GMAC_NCFGR_LFERD
#define	 GMAC_NCFGR_CLK_MCK_8		GMAC_NCFGR_CLK(0)
#define	 GMAC_NCFGR_CLK_MCK_16		GMAC_NCFGR_CLK(1)
#define	 GMAC_NCFGR_CLK_MCK_32		GMAC_NCFGR_CLK(2)
#define	 GMAC_NCFGR_CLK_MCK_48		GMAC_NCFGR_CLK(3)
#define	 GMAC_NCFGR_CLK_MCK_64		GMAC_NCFGR_CLK(4)
#define	 GMAC_NCFGR_CLK_MCK_96		GMAC_NCFGR_CLK(5)
#define	 GMAC_NCFGR_DCPF_Msk		GMAC_NCFGR_DCPF
#define	 GMAC_NCFGR_RFCS_Msk		GMAC_NCFGR_RFCS
#define	 GMAC_NCFGR_PEN_Msk		GMAC_NCFGR_PEN
#define	 GMAC_NCFGR_SPD_Msk		GMAC_NCFGR_SPD
#define	 GMAC_NCFGR_FD_Msk		GMAC_NCFGR_FD
#define	GMAC_NSR			NSR.reg
#define	 GMAC_NSR_IDLE_Msk		GMAC_NSR_IDLE
#define	GMAC_UR				UR.reg
#define	 GMAC_UR_RMII			(0u << GMAC_UR_MII_Pos)
#define	GMAC_DCFGR			DCFGR.reg
#define	 GMAC_DCFGR_FBLDO_INCR4		GMAC_DCFGR_FBLDO(4)
#define	 GMAC_DCFGR_RXBMS_FULL		GMAC_DCFGR_RXBMS(3)
#define	 GMAC_DCFGR_TXPBMS_Msk		GMAC_DCFGR_TXPBMS
#define	GMAC_TSR			TSR.reg
#define	 GMAC_TSR_Msk			GMAC_TSR_MASK
#define	 GMAC_TSR_TXGO_Msk		GMAC_TSR_TXGO
#define	 GMAC_TSR_TFC_Msk		GMAC_TSR_TFC
#define	 GMAC_TSR_HRESP_Msk		GMAC_TSR_HRESP
#define	GMAC_RBQB			RBQB.reg
#define	GMAC_TBQB			TBQB.reg
#define	GMAC_RSR			RSR.reg
#define	 GMAC_RSR_Msk			GMAC_RSR_MASK
#define	GMAC_ISR			ISR.reg
#define	GMAC_IER			IER.reg
#define	 GMAC_IER_RCOMP_Msk		GMAC_IER_RCOMP
#define	 GMAC_IER_TXUBR_Msk		GMAC_IER_TXUBR
#define	 GMAC_IER_TUR_Msk		GMAC_IER_TUR
#define	 GMAC_IER_RLEX_Msk		GMAC_IER_RLEX
#define	 GMAC_IER_TFC_Msk		GMAC_IER_TFC
#define	 GMAC_IER_ROVR_Msk		GMAC_IER_ROVR
#define	 GMAC_IER_HRESP_Msk		GMAC_IER_HRESP
#define	GMAC_IDR			IDR.reg
#define	 GMAC_IDR_Msk			GMAC_IDR_MASK
#define	GMAC_MAN			MAN.reg
#define	 GMAC_MAN_CLTTO_Msk		GMAC_MAN_CLTTO
#define	GMAC_SAB			SAB.reg
#define	GMAC_SAT			SAT.reg
#define	GMAC_TPSF			TPSF.reg
#define	 GMAC_TPSF_ENTXP_Msk		GMAC_TPSF_ENTXP
#define	GMAC_RPSF			TPSF.reg
#define	 GMAC_RPSF_ENRXP_Msk		GMAC_RPSF_ENRXP
#define	GMAC_WOL			WOL.reg

/* TIDM.ENID is completely missing in the SAM[DE]5x header. */
#define	 GMAC_TIDM1_ENID1_Msk		(1u << 31)
#define	 GMAC_TIDM2_ENID2_Msk		(1u << 31)
#define	 GMAC_TIDM3_ENID3_Msk		(1u << 31)
#define	 GMAC_TIDM4_ENID4_Msk		(1u << 31)

#define GMAC_DCACHE_CLEAN_INVALIDATE	CMCC_CleanInvalidateDCache_by_Addr
#define GMAC_DCACHE_INVALIDATE		CMCC_InvalidateDCache_by_Addr
#define GMAC_DCACHE_CLEAN		CMCC_CleanDCache_by_Addr
#define	LINESIZE			16

#include "sam_cmcc.h"

#define	GMAC_PAYLOAD_IN_SRAM(x)		(((uintptr_t)(x) >> 28) == 2u)
#else	/* SAME7x? */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT != 0)
#define	GMAC_DCACHE_CLEAN_INVALIDATE	SCB_CleanInvalidateDCache_by_Addr
#define	GMAC_DCACHE_INVALIDATE		SCB_InvalidateDCache_by_Addr
#define	GMAC_DCACHE_CLEAN		SCB_CleanDCache_by_Addr
#define	LINESIZE			__SCB_DCACHE_LINE_SIZE
#endif
#endif	/* GMAC_IS_SAME5X */

/*
 * LWIP must be compiled with custom PBUF support.
 */
_Static_assert(LWIP_SUPPORT_CUSTOM_PBUF, "Need LWIP_SUPPORT_CUSTOM_PBUF=1");

/*
 * If LWIP supports per-interface checksum controls, crash out.
 */
//#if LWIP_CHECKSUM_CTRL_PER_NETIF
//#error "Hardware checksum not supported."
//#endif

/* Define a memory barrier for use when writing descriptors. */
#define	MEMORY_STORE_BARRIER()	do { __DMB(); __DSB(); __ISB(); } while (0)

/*
 * Enable cache sync ops if the target has a data cache.
 */
#ifndef GMAC_DCACHE_CLEAN_INVALIDATE
#define	GMAC_DCACHE_CLEAN_INVALIDATE(a,s)	\
	do {					\
		(void) a;			\
		(void) s;			\
		MEMORY_STORE_BARRIER();		\
	} while (0)
#define	GMAC_DCACHE_INVALIDATE		GMAC_DCACHE_CLEAN_INVALIDATE
#define	GMAC_DCACHE_CLEAN		GMAC_DCACHE_CLEAN_INVALIDATE
#define	LINESIZE			(1u)
#endif	/* GMAC_DCACHE_CLEAN_INVALIDATE */

#define	GMAC_CACHE_ADDR_ROUND(a)	((uintptr_t)(a) & ~(LINESIZE-1))
#define	GMAC_CACHE_LEN_ROUND(a,l)	(((((uintptr_t)(a) & (LINESIZE-1)) + \
					 (l)) + (LINESIZE-1)) & ~(LINESIZE-1))

/*
 * Given an array, return the number of members.
 */
#define	ARRAY_ELEMENTS(x)		(sizeof(x) / sizeof((x)[0]))

/*
 * Tx descriptors.
 */
struct gmac_txdesc {
	volatile void *gt_payload;
	volatile uint32_t gt_csr;
};
_Static_assert(sizeof(struct gmac_txdesc) == 8, "Tx descriptor size");
#define	GMAC_TXD_CSR_USED	(1u << 31)
#define	GMAC_TXD_CSR_WRAP	(1u << 30)
#define	GMAC_TXD_CSR_RETRY_LIM	(1u << 29)
#define	GMAC_TXD_CSR_AHB_ERROR	(1u << 27)
#define	GMAC_TXD_CSR_LATE_COL	(1u << 26)
#define	GMAC_TXD_CSR_CS(x)	(((x) >> 20) & 0x7u)
#define	GMAC_TXD_CSR_CS_GOOD	0
#define	GMAC_TXD_CSR_CS_VLANE	1
#define	GMAC_TXD_CSR_CS_SNAPE	2
#define	GMAC_TXD_CSR_CS_NOT_IP	3
#define	GMAC_TXD_CSR_CS_BAD	4
#define	GMAC_TXD_CSR_CS_FRAG	5
#define	GMAC_TXD_CSR_CS_PROTO	6
#define	GMAC_TXD_CSR_CS_PEOP	7
#define	GMAC_TXD_CSR_NO_CRC	(1u << 16)
#define	GMAC_TXD_CSR_LAST	(1u << 15)
#define	GMAC_TXD_CSR_LEN(x)	((x) & 0x1fffu)

/*
 * Tx pbuf tracking.
 */
struct gmac_tx_pbuf {
	uint16_t tp_desc_cnt;
	uint16_t tp_static;
	struct pbuf *tp_pbuf;
	STAILQ_ENTRY(gmac_tx_pbuf) tp_qent;
};
STAILQ_HEAD(gmac_tx_pbuf_queue, gmac_tx_pbuf);

/*
 * Rx descriptors.
 */
struct gmac_rxdesc {
	volatile uint32_t gr_payload;
	volatile uint32_t gr_sr;
};
_Static_assert(sizeof(struct gmac_rxdesc) == 8, "Rx descriptor size");
#define	GMAC_RXD_PAYLOAD_WRAP	(1u << 1)
#define	GMAC_RXD_PAYLOAD_USED	(1u << 0)

#define	GMAC_RXD_SR_BCAST	(1u << 31)
#define	GMAC_RXD_SR_MULTI_MATCH	(1u << 30)
#define	GMAC_RXD_SR_UNI_MATCH	(1u << 29)
#define	GMAC_RXD_SR_SPEC_MATCH	(1u << 27)
#define	GMAC_RXD_SR_SPEC(x)	(((x) >> 25) & 0x3u)
#define	GMAC_RXD_SR_TYPE_MATCH	(1u << 24)
#define	GMAC_RXD_SR_TYPE(x)	(((x) >> 22) & 0x3u)
#define	GMAC_RXD_SR_CS_SNAP	(1u << 24)
#define	GMAC_RXD_SR_CS_STAT(x)	(((x) >> 22) & 0x3u)
#define	GMAC_RXD_SR_CS_NONE	0
#define	GMAC_RXD_SR_CS_IP	1
#define	GMAC_RXD_SR_CS_IP_TCP	2
#define	GMAC_RXD_SR_CS_IP_UDP	3
#define	GMAC_RXD_SR_VLAN	(1u << 21)
#define	GMAC_RXD_SR_PRI_TAG	(1u << 20)
#define	GMAC_RXD_SR_VLAN_PRI(x)	(((x) >> 17) & 0x3u)
#define	GMAC_RXD_SR_EOF		(1u << 15)
#define	GMAC_RXD_SR_SOF		(1u << 14)
#define	GMAC_RXD_SR_FCS		(1u << 13)
#define	GMAC_RXD_SR_LEN(x)	((x) & 0xfffu)

/*
 * Rx buffer size. This must be a multiple of 32 bytes.
 */
#define	GMAC_RXBUFF_SIZE	0x600u

/* Rx packets <= this size will be copied into a regular pbuf. */
#define	GMAC_RX_COPY_THRESHOLD	128

/* Global interrupt mask. */
#define	GMAC_INTERRUPT_MASK	(GMAC_IER_RCOMP_Msk | \
				 GMAC_IER_TXUBR_Msk | \
				 GMAC_IER_TUR_Msk | \
				 GMAC_IER_RLEX_Msk | \
				 GMAC_IER_TFC_Msk | \
				 GMAC_IER_ROVR_Msk | \
				 GMAC_IER_HRESP_Msk)

struct gmac_state;
struct gmac_rx_pbuf {
	struct pbuf_custom rp_pbuf;
	struct gmac_state *rp_state;
	union {
		TAILQ_ENTRY(gmac_rx_pbuf) rpu_qent;
		unsigned int rpu_len;
	} rp_u;
	uint8_t rp_payload[GMAC_RXBUFF_SIZE];
};
#define	rp_qent	rp_u.rpu_qent
#define	rp_len	rp_u.rpu_len
_Static_assert(offsetof(struct gmac_rx_pbuf, rp_payload) == 32,
    "gmac_rx_pbuf.rp_payload");
_Static_assert((sizeof(struct gmac_rx_pbuf) & 0x1f) == 0, "gmac_rx_pbuf size");
TAILQ_HEAD(gmac_rx_pbuf_queue, gmac_rx_pbuf);
#define	ETH_STATE_RX_PBUF_FREE(rp)	(rp)->rp_pbuf.pbuf.payload = NULL
#define	ETH_STATE_RX_PBUF_IS_FREE(rp)	((rp)->rp_pbuf.pbuf.payload == NULL)

enum gmac_link_status {
	GMAC_PHY_LINK_UNKNOWN,
	GMAC_PHY_LINK_DOWN,
	GMAC_PHY_LINK_100FDX,
	GMAC_PHY_LINK_100HDX,
	GMAC_PHY_LINK_10FDX,
	GMAC_PHY_LINK_10HDX,
};

struct gmac_state {
	Gmac *gs_regs;

	/* Tx state. */
	uint16_t gs_tx_desc_cnt;
	uint16_t gs_tx_desc_head;
	uint16_t gs_tx_desc_tail;
	uint16_t gs_tx_desc_free;
	uint16_t gs_tx_pbuf_static_cnt;
	struct gmac_txdesc *gs_tx_descs;
	struct gmac_tx_pbuf **gs_tx_desc2pbuf;
	struct gmac_tx_pbuf *gs_tx_pbuf_static;
	struct gmac_tx_pbuf_queue gs_tx_pbuf_freelist;
	struct gmac_tx_pbuf_queue gs_tx_pbuf_tx_queue;

	/* Rx state. */
	uint16_t gs_rx_pbuf_cnt;
	uint16_t gs_rx_desc_cnt;
	uint16_t gs_rx_curr_desc;
	struct gmac_rxdesc *gs_rx_descs;
	struct gmac_rx_pbuf **gs_rx_desc2pbuf;
	struct gmac_rx_pbuf_queue gs_rx_pbuf_freelist;
	struct gmac_rx_pbuf *gs_rx_pbufs;
	void *gs_rx_pbufs_real;

	/* PHY/link state. */
	uint32_t gs_phy_man;
	sam_gmac_hw_mode_t gs_hw_mode;
	enum gmac_link_status gs_link_status;

	/* Everything else - at the end because less frequently accessed. */
	uint32_t gs_clock;
	struct network_state *gs_ns;
	uint8_t gs_mac[ETH_HWADDR_LEN];
	const char *gs_hostname;
	network_link_status_callback_t gs_link_cb;
	void *gs_link_cb_arg;
	struct network_lwip_driver gs_nld;
	struct netif gs_netif;
};

#define	PHY_REG_BCR			0x00
#define	 PHY_BCR_FDX			(1u << 8)
#define	 PHY_BCR_AUTONEG_RESTART	(1u << 9)
#define	 PHY_BCR_ISOLATE		(1u << 10)
#define	 PHY_BCR_POWER_DOWN		(1u << 11)
#define	 PHY_BCR_AUTONEG_ENABLE		(1u << 12)
#define	 PHY_BCR_100M			(1u << 13)
#define	 PHY_BCR_LOOPBACK		(1u << 14)
#define	 PHY_BCR_RESET			(1u << 15)

#define	PHY_REG_BSR			0x01
#define	 PHY_BSR_EXTENDED		(1u << 0)
#define	 PHY_BSR_JABBER			(1u << 1)
#define	 PHY_BSR_LINK_UP		(1u << 2)
#define	 PHY_BSR_AUTONEG_CAPABLE	(1u << 3)
#define	 PHY_BSR_REMOTE_FAULT		(1u << 4)
#define	 PHY_BSR_AUTONEG_COMPLETE	(1u << 5)

#define	PHY_REG_ANAR			0x04
#define	PHY_REG_ANLPAR			0x05
/* Default: SYM PAUSE, 100FDX, 100HDX, 10FDX, 10HDX, CSMA */
#define	 PHY_ANAR_OUR_DEFAULT		0x05e1u
#define	 PHY_ANAR_FLOW_CTRL		(1u << 10)
#define	 PHY_ANAR_PAUSE_ASYMMETRIC	(1u << 11)
#define	 PHY_ANAR_100TX_FDX		(1u << 8)
#define	 PHY_ANAR_100TX_HDX		(1u << 7)
#define	 PHY_ANAR_10TX_FDX		(1u << 6)
#define	 PHY_ANAR_10TX_HDX		(1u << 5)

#ifndef	GMAC_IS_SAME5X
/*
 * Work-around for silly priority queue behaviour.
 */
#define	GMAC_DUMMY_DESC_CNT	2
struct gmac_dummy_desc {
	struct gmac_txdesc dd_tx[GMAC_DUMMY_DESC_CNT];
	struct gmac_rxdesc dd_rx[GMAC_DUMMY_DESC_CNT];
};
static __attribute__ ((aligned(32))) struct gmac_dummy_desc gmac_dummy_desc;
#endif

static void
gmac_phy_write(struct gmac_state *gs, uint32_t phyreg, uint16_t v)
{
	Gmac *r = gs->gs_regs;
	uint32_t reg;

	reg = gs->gs_phy_man | GMAC_MAN_OP(1) | GMAC_MAN_REGA(phyreg) |
	    GMAC_MAN_DATA(v);
	r->GMAC_MAN = reg;
	MEMORY_STORE_BARRIER();

	while ((r->GMAC_NSR & GMAC_NSR_IDLE_Msk) == 0)
		rtos_yield();
}

static uint16_t
gmac_phy_read(struct gmac_state *gs, uint32_t phyreg)
{
	Gmac *r = gs->gs_regs;
	uint32_t reg;

	reg = gs->gs_phy_man | GMAC_MAN_OP(2) | GMAC_MAN_REGA(phyreg);
	r->GMAC_MAN = reg;
	MEMORY_STORE_BARRIER();

	while ((r->GMAC_NSR & GMAC_NSR_IDLE_Msk) == 0)
		rtos_yield();

	reg = (r->GMAC_MAN & GMAC_MAN_DATA_Msk) >> GMAC_MAN_DATA_Pos;

	return (uint16_t)reg;
}

static void
gmac_cache_inv(const void *addr, unsigned int len)
{

	uint32_t *a;

	len = GMAC_CACHE_LEN_ROUND(addr, len);
	a = (uint32_t *)GMAC_CACHE_ADDR_ROUND(addr);

	GMAC_DCACHE_INVALIDATE(a, len);
}

static void
gmac_cache_clean(const void *addr, unsigned int len)
{
	uint32_t *a;

	len = GMAC_CACHE_LEN_ROUND(addr, len);
	a = (uint32_t *)GMAC_CACHE_ADDR_ROUND(addr);

	GMAC_DCACHE_CLEAN(a, len);
}

#ifndef	GMAC_IS_SAME5X
static void
gmac_cache_clean_inv(const void *addr, unsigned int len)
{
	uint32_t *a;

	len = GMAC_CACHE_LEN_ROUND(addr, len);
	a = (uint32_t *)GMAC_CACHE_ADDR_ROUND(addr);

	GMAC_DCACHE_CLEAN_INVALIDATE(a, len);
}
#endif

static struct gmac_rx_pbuf *
gmac_rx_pbuf_alloc(struct gmac_state *gs)
{
	struct gmac_rx_pbuf *rp;

	if ((rp = TAILQ_FIRST(&gs->gs_rx_pbuf_freelist)) != NULL) {
		assert(ETH_STATE_RX_PBUF_IS_FREE(rp));

		/*
		 * Note: We don't advance rp_payload by ETH_PAD_SIZE
		 * because the controller deals with the required
		 * offset via GMAC_NCFGR.RXBUFO.
		 */
		if (pbuf_alloced_custom(PBUF_RAW,
		    GMAC_RXBUFF_SIZE - ETH_PAD_SIZE, PBUF_REF,
		    &rp->rp_pbuf, rp->rp_payload,
		    GMAC_RXBUFF_SIZE - ETH_PAD_SIZE) == NULL) {
			return NULL;
		}

		TAILQ_REMOVE(&gs->gs_rx_pbuf_freelist, rp, rp_qent);
		rp->rp_len = 0;
	}

	return rp;
}

static void
gmac_pbuf_custom_free(struct pbuf *p)
{
	struct gmac_rx_pbuf *rp = (struct gmac_rx_pbuf *)p;
	struct gmac_state *gs = rp->rp_state;

	DBPRINTF(GMAC_DEBUG_RX, "gmac_pbuf_custom_free: rp %p, pbuf %p, "
	    "payload %p (%p), len %u\n", (void *)rp, (void *)p,
	    (void *)p->payload, (void *)rp->rp_payload, rp->rp_len);

	assert(!ETH_STATE_RX_PBUF_IS_FREE(rp));
	assert(rp->rp_len > 0);
	assert(rp->rp_len <= GMAC_RXBUFF_SIZE);

	/* Ensure the payload buffer is no longer cached. */
	gmac_cache_inv(rp->rp_payload, rp->rp_len);

	ETH_STATE_RX_PBUF_FREE(rp);
	TAILQ_INSERT_TAIL(&gs->gs_rx_pbuf_freelist, rp, rp_qent);
}

static void
gmac_rxdesc_init(struct gmac_state *gs, struct gmac_rxdesc *gr,
    unsigned int idx, struct gmac_rx_pbuf *rp)
{

	DBPRINTF(GMAC_DEBUG_RX, "gmac_rxdesc_init: rp %p, payload %p\n",
	    (void *)rp, (void *)rp->rp_payload);

	gs->gs_rx_desc2pbuf[idx] = rp;
	MEMORY_STORE_BARRIER();
	gr->gr_sr = 0;
	gr->gr_payload = ((uint32_t)(uintptr_t)rp->rp_payload) |
	    ((idx == (gs->gs_rx_desc_cnt - 1u)) ? GMAC_RXD_PAYLOAD_WRAP : 0);
	MEMORY_STORE_BARRIER();

	/* Descriptor is owned by h/w. */
}

static int
gmac_rx_reap(struct gmac_state *gs)
{
	uint16_t idx = gs->gs_rx_curr_desc;
	struct gmac_rxdesc *gr = &gs->gs_rx_descs[idx];
	struct gmac_rx_pbuf *rp, *new_rp;
	struct pbuf *p;
	uint32_t sr;
	unsigned int plen;
	int rv = 0;

	assert(idx < gs->gs_rx_desc_cnt);

	for (;;) {
		/* If this descriptor is owned by the h/w, we're done. */
		if ((gr->gr_payload & GMAC_RXD_PAYLOAD_USED) == 0)
			break;

		sr = gr->gr_sr;
		rv++;

		DBPRINTF(GMAC_DEBUG_RX, "gmac_rx_reap: idx %" PRIu16 ", sr "
		    "%08" PRIx32 "\n", idx, sr);

		/* Fetch the pbuf associated to this descriptor. */
		rp = gs->gs_rx_desc2pbuf[idx];
		assert(rp != NULL);

		if ((gs->gs_netif.flags & NETIF_FLAG_UP) == 0) {
			/* Drop packet if interface is down. */
			plen = 0;
			DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
			    "gmac_rx_reap: Interface down. Dropping\n");
		} else
		if ((sr & (GMAC_RXD_SR_SOF | GMAC_RXD_SR_EOF)) !=
		    (GMAC_RXD_SR_SOF | GMAC_RXD_SR_EOF)) {
			/* Drop packet if this is part of a jumbo frame. */
			plen = 0;
			DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
			    "gmac_rx_reap: Part of jumbo frame. Dropping\n");
		} else {
			plen = (unsigned int)GMAC_RXD_SR_LEN(sr);
			if (plen == 0) {
				DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
				    "gmac_rx_reap: Zero-length packet. "
				    "Dropping.\n");
			}
		}

		if (plen == 0) {
			/* Error, or zero length packet. Reuse this buffer. */
			p = NULL;
			new_rp = rp;
		} else
		if (plen <= GMAC_RX_COPY_THRESHOLD ||
		    (new_rp = gmac_rx_pbuf_alloc(gs)) == NULL) {
			/*
			 * We're out of Rx buffers. Copy the payload into
			 * a new pbuf and reuse the original buffer.
			 */
			if (plen > GMAC_RX_COPY_THRESHOLD) {
				DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
				    "gmac_rx_reap: Out of buffers. "
				    "Copying...\n");
			} else {
				DBPRINTF(GMAC_DEBUG_RX, "gmac_rx_reap: Small "
				    "buffer (%u). Copying...\n", plen);
			}
			new_rp = rp;
			plen += ETH_PAD_SIZE;
			p = pbuf_alloc(PBUF_RAW, plen, PBUF_POOL);
			if (p != NULL) {
				/* Copy payload into pbuf. */
				if (pbuf_take(p,rp->rp_payload,plen) != ERR_OK){
					pbuf_free(p);
					p = NULL;
					DBPRINTF(GMAC_DEBUG_ERROR |
					    GMAC_DEBUG_RX, "gmac_rx_reap: "
					    "pbuf_take() failed\n");
				}
				/* Done with the original payload buffer. */
				gmac_cache_inv(rp->rp_payload, plen);
			} else {
				DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
				    "gmac_rx_reap: pbuf_alloc() failed\n");
			}
		} else {
			p = &rp->rp_pbuf.pbuf;
			rp->rp_len = p->len = p->tot_len = plen + ETH_PAD_SIZE;
		}

		/* Prepare the descriptor for reuse. */
		gmac_rxdesc_init(gs, gr, idx, new_rp);

		if (p != NULL) {
			DBPRINTF(GMAC_DEBUG_RX, "gmac_rx_reap: Input %u "
			    "bytes.\n", p->tot_len);
			if (gs->gs_netif.input(p, &gs->gs_netif) != ERR_OK) {
				DBPRINTF(GMAC_DEBUG_ERROR | GMAC_DEBUG_RX,
				    "gmac_rx_reap: netif.input() failed.\n");
				pbuf_free(p);
			}
		}

		if (++idx == gs->gs_rx_desc_cnt) {
			idx = 0;
			gr = &gs->gs_rx_descs[0];
		} else {
			gr++;
		}
	}

	if (gs->gs_rx_curr_desc != idx) {
		gs->gs_rx_curr_desc = idx;
		DBPRINTF(GMAC_DEBUG_RX, "gmac_rx_reap: Finished\n");
	}

	return rv;
}

static uint16_t
gmac_tx_advance(struct gmac_state *gs, uint16_t curr, uint16_t count)
{

	curr += count;
	if (curr >= gs->gs_tx_desc_cnt)
		curr -= gs->gs_tx_desc_cnt;
	return curr;
}

static void
gmac_tx_desc_dump(struct gmac_txdesc *gt)
{

#ifdef GMAC_DEBUG
	DBPRINTF(GMAC_DEBUG_TX, "\tCSR %08" PRIx32 ", payload %p\n", gt->gt_csr,
	    gt->gt_payload);
#else
	(void) gt;
#endif
}

static int
gmac_tx_reap(struct gmac_state *gs, uint32_t tsr)
{
	struct gmac_txdesc *gt;
	struct gmac_tx_pbuf *tp;
	uint16_t desc, last_desc;
	int rv = 0;

	if (tsr & (GMAC_TSR_TFC_Msk | GMAC_TSR_HRESP_Msk)) {
		DBPRINTF(GMAC_DEBUG_TX_REAP | GMAC_DEBUG_ERROR, "gmac_tx_reap: "
		    "Error: 0x%08" PRIx32 "\n", tsr);
	}

	desc = gs->gs_tx_desc_tail;

	DBPRINTF(GMAC_DEBUG_TX_REAP, "gmac_tx_reap: Start: tail %" PRIu16
	    ", desc_free %" PRIu16 ", desc_cnt %" PRIu16 "\n", desc,
	    gs->gs_tx_desc_free, gs->gs_tx_desc_cnt);

	while (gs->gs_tx_desc_free < gs->gs_tx_desc_cnt) {
		tp = gs->gs_tx_desc2pbuf[desc];
		assert(tp != NULL);
		assert(tp->tp_pbuf != NULL);
		assert(tp->tp_desc_cnt > 0);
		assert(tp->tp_desc_cnt <= gs->gs_tx_desc_cnt);
		assert((tp->tp_desc_cnt + gs->gs_tx_desc_free) <=
		    gs->gs_tx_desc_cnt);

		gt = &gs->gs_tx_descs[desc];

		/* We're done if the hardware still owns it. */
		if ((gt->gt_csr & GMAC_TXD_CSR_USED) == 0)
			break;

		rv++;

		/* Locate the last descriptor for this pbuf */
		last_desc = gmac_tx_advance(gs, desc, tp->tp_desc_cnt - 1);

		DBPRINTF(GMAC_DEBUG_TX_REAP, "gmac_tx_reap: p %p, "
		    "desc %" PRIu16 "-%" PRIu16 "\n", (void *)tp->tp_pbuf, desc,
		    last_desc);
		gmac_tx_desc_dump(gt);

		gs->gs_tx_desc2pbuf[desc] = NULL;
		gs->gs_tx_desc_free += tp->tp_desc_cnt;

		pbuf_free(tp->tp_pbuf);

		if (tp->tp_static) {
			tp->tp_pbuf = NULL;
			/* Add to head to keep the cache hot */
			STAILQ_INSERT_HEAD(&gs->gs_tx_pbuf_freelist, tp,
			    tp_qent);
		} else {
			zone_free(tp);
		}

		/*
		 * For multi-segment packets, we must set the USED bit in
		 * the remaining descriptors.
		 */
		if (desc != last_desc) {
			do {
				desc = gmac_tx_advance(gs, desc, 1);
				gt = &gs->gs_tx_descs[desc];
				gt->gt_csr = GMAC_TXD_CSR_USED;
			} while (desc != last_desc);
			MEMORY_STORE_BARRIER();
		}

		/* Advance to the start of the next packet. */
		desc = gmac_tx_advance(gs, desc, 1);
	}

	gs->gs_tx_desc_tail = desc;

	DBPRINTF(GMAC_DEBUG_TX_REAP, "gmac_tx_reap: Done: tail %" PRIu16
	    ", desc_free %" PRIu16 ", desc_cnt %" PRIu16 "\n", desc,
	    gs->gs_tx_desc_free, gs->gs_tx_desc_cnt);

	return rv;
}

static void
gmac_tx_start(struct gmac_state *gs, struct gmac_tx_pbuf *tp)
{
	Gmac *r = gs->gs_regs;
	struct gmac_txdesc *gt, *first_gt;
	uint16_t desc, last_desc;
	struct pbuf *p;
	uint32_t csr;

	assert(tp->tp_desc_cnt > 0);
	assert(tp->tp_desc_cnt <= gs->gs_tx_desc_free);

	desc = gs->gs_tx_desc_head;
	first_gt = gt = &gs->gs_tx_descs[desc];
	last_desc = gmac_tx_advance(gs, desc, tp->tp_desc_cnt - 1);

	DBPRINTF(GMAC_DEBUG_TX, "gmac_tx_start: p %p, first %" PRIu16 " (%p), "
	    "last %" PRIu16 ", TBQB 0x%08" PRIx32 "\n", (void *)tp->tp_pbuf,
	    desc, (void *)gt, last_desc, r->GMAC_TBQB);

	for (p = tp->tp_pbuf; p != NULL; p = p->next) {
		if (p->len == 0)
			continue;

		DBPRINTF(GMAC_DEBUG_TX, "gmac_tx_start: desc %" PRIu16
		    ", payload %p, len %u\n", desc, p->payload,
		    (unsigned int)p->len);

		csr = GMAC_TXD_CSR_LEN(p->len);
		if (gt == first_gt)
			csr |= GMAC_TXD_CSR_USED;
		if (desc == (gs->gs_tx_desc_cnt - 1))
			csr |= GMAC_TXD_CSR_WRAP;
		if (desc == last_desc)
			csr |= GMAC_TXD_CSR_LAST;
		gt->gt_payload = p->payload;
		gt->gt_csr = csr;
		MEMORY_STORE_BARRIER();
		gmac_tx_desc_dump(gt);
		desc = gmac_tx_advance(gs, desc, 1);
		gt = &gs->gs_tx_descs[desc];
	}

	/*
	 * At this point, we still have to clear the USED bit on the first
	 * descriptor.
	 */
	MEMORY_STORE_BARRIER();
	first_gt->gt_csr &= ~GMAC_TXD_CSR_USED;
	MEMORY_STORE_BARRIER();

	/* Kick off the transmitter. */
	r->GMAC_NCR |= GMAC_NCR_TSTART_Msk;

	/* Update Tx state tracking. */
	gs->gs_tx_desc_free -= tp->tp_desc_cnt;
	gs->gs_tx_desc2pbuf[gs->gs_tx_desc_head] = tp;
	gs->gs_tx_desc_head = desc;

	DBPRINTF(GMAC_DEBUG_TX, "gmac_tx_start: TSR 0x%03" PRIx32 "\n",
	    r->GMAC_TSR);
}

static err_t
gmac_output(struct netif *netif, struct pbuf *p)
{
	struct gmac_state *gs = netif->state;
	struct gmac_tx_pbuf *tp;
	uint16_t descs;
	struct pbuf *q;

	/* Don't transmit if we have no link. */
	if ((gs->gs_netif.flags & NETIF_FLAG_LINK_UP) == 0)
		return ERR_IF;

#if ETH_PAD_SIZE
	pbuf_remove_header(p, ETH_PAD_SIZE);
#endif

	/*
	 * Count pbuf segments here while the chain is likely to still
	 * be hot in the data cache.
	 * While we're crawling the chain, clean the D-Cache for each
	 * payload segment.
	 */
#ifdef	GMAC_IS_SAME5X
 again:
#endif

	for (descs = 0, q = p; q != NULL; q = q->next) {
#ifdef	GMAC_IS_SAME5X
		/*
		 * The SAMD5x/E5x GMAC can perform DMA to/from SRAM only.
		 * Flash payloads must be bounced via an SRAM pbuf.
		 */
		if (!GMAC_PAYLOAD_IN_SRAM(q->payload)) {
			DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: non-SRAM "
			    "segment. Cloning...\n");

			q = pbuf_clone(PBUF_RAW, PBUF_POOL, p);
			if (q == NULL) {
				DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: "
				    "pbuf_clone() failed\n");
				return ERR_MEM;
			}

			/*
			 * Clone is now active.
			 * Note: Caller will free the original pbuf since
			 * we never invoked pbuf_ref() on it.
			 */
			p = q;
			goto again;
		}
#endif
		if (q->len != 0) {
			descs++;
			gmac_cache_clean(q->payload, q->len);
		}
	}

	DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: p %p, descs %" PRIu16 "\n",
	    (void *)p, descs);

	/*
	 * While highly unlikely, catch the case where there are more
	 * segments than we can deal with.
	 */
	if (descs > gs->gs_tx_desc_cnt) {
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: too many "
		    "segments\n");
		return ERR_MEM;
	}

	if (descs == 0) {
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: zero segments!\n");
		return ERR_OK;
	}

	if ((tp = STAILQ_FIRST(&gs->gs_tx_pbuf_freelist)) != NULL) {
		STAILQ_REMOVE_HEAD(&gs->gs_tx_pbuf_freelist, tp_qent);
		assert(tp->tp_pbuf == NULL);
		assert(tp->tp_static);
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: tp from q\n");
	} else {
		tp = zone_malloc(sizeof(*tp));
		if (tp == NULL) {
			DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: out of "
			    "memory\n");
			return ERR_MEM;
		}
		tp->tp_static = 0;
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: new tp\n");
	}

	pbuf_ref(p);

	tp->tp_pbuf = p;
	tp->tp_desc_cnt = descs;

	if (STAILQ_EMPTY(&gs->gs_tx_pbuf_tx_queue) &&
	    descs <= gs->gs_tx_desc_free) {
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: start Tx\n");
		gmac_tx_start(gs, tp);
	} else {
		DBPRINTF(GMAC_DEBUG_TX_START, "gmac_output: queued Tx\n");
		STAILQ_INSERT_TAIL(&gs->gs_tx_pbuf_tx_queue, tp, tp_qent);
	}

	return ERR_OK;
}

static void
gmac_reset(struct gmac_state *gs)
{
	Gmac *r = gs->gs_regs;
	unsigned int i;
	uint32_t reg;

	/* Set key registers to default values. */
	r->GMAC_NCR = GMAC_NCR_CLRSTAT_Msk;
	r->GMAC_NCFGR = GMAC_NCFGR_LFERD_Msk;
	r->GMAC_DCFGR = GMAC_DCFGR_FBLDO_INCR4 | GMAC_DCFGR_DRBS(2);
	r->GMAC_TSR = GMAC_TSR_Msk;
	r->GMAC_RSR = GMAC_RSR_Msk;
	r->GMAC_IDR = GMAC_IDR_Msk;

	/* Select MII or RMII. */
	r->GMAC_UR = (gs->gs_hw_mode == SAM_GMAC_MODE_RMII) ? GMAC_UR_RMII : 0;

	/* Figure out the MDIO clock divider */
	if (gs->gs_clock > 160000000)
		reg = GMAC_NCFGR_CLK_MCK_96;
	else
	if (gs->gs_clock > 120000000)
		reg = GMAC_NCFGR_CLK_MCK_64;
	else
	if (gs->gs_clock > 80000000)
		reg = GMAC_NCFGR_CLK_MCK_48;
	else
	if (gs->gs_clock > 40000000)
		reg = GMAC_NCFGR_CLK_MCK_32;
	else
	if (gs->gs_clock > 20000000)
		reg = GMAC_NCFGR_CLK_MCK_16;
	else
		reg = GMAC_NCFGR_CLK_MCK_8;

#if 0
	/* XXX: Here's where we'd enable Rx checksum offload. */
	reg |= GMAC_NCFGR_RXCOEN_Msk;
#endif

	/* We don't need a copy of pause frames. */
	reg |= GMAC_NCFGR_DCPF_Msk;

	/* Data bus width is always 32 (according to the datasheet) */
	reg |= GMAC_NCFGR_DBW(0);

	/* Don't copy the FCS field into received frames. */
	reg |= GMAC_NCFGR_RFCS_Msk;

	/* Configure Rx buffer offset using ETH_PAD_SIZE. */
	reg |= GMAC_NCFGR_RXBUFO(ETH_PAD_SIZE);

	/* Discard frames with length errors. */
	reg |= GMAC_NCFGR_LFERD_Msk;

	/* Enable Pause support. */
	reg |= GMAC_NCFGR_PEN_Msk;

	r->GMAC_NCFGR = reg;
	MEMORY_STORE_BARRIER();

	/* Safe to enable the management port. */
	r->GMAC_NCR |= GMAC_NCR_MPE_Msk;
	MEMORY_STORE_BARRIER();

	/* Configure GMAC_DCFGR */
	/* Set the DMA receive buffer size. */
	reg = GMAC_DCFGR_DRBS(GMAC_RXBUFF_SIZE / 64);

#if 0
	/* XXX: Here's where we'd enable Tx checksum offload. */
	reg |= GMAC_DCFGR_TXCOEN_Msk;
#endif

	/* Rx/Tx packet buffer memory sizes. */
	reg |= GMAC_DCFGR_RXBMS_FULL;
#if 1	//ndef	GMAC_IS_SAME5X
	reg |= GMAC_DCFGR_TXPBMS_Msk;
#endif

	/* Use INCR4 bursts */
	reg |= GMAC_DCFGR_FBLDO_INCR4;

	r->GMAC_DCFGR = reg;
	MEMORY_STORE_BARRIER();

	/* Disable partial store and forward. */
	r->GMAC_TPSF &= ~GMAC_TPSF_ENTXP_Msk;
	r->GMAC_RPSF &= ~GMAC_RPSF_ENRXP_Msk;
	MEMORY_STORE_BARRIER();

	/* Disable Type ID matching. */
#ifdef	GMAC_IS_SAME5X
	r->TIDM[0].reg &= ~GMAC_TIDM1_ENID1_Msk;
	r->TIDM[1].reg &= ~GMAC_TIDM2_ENID2_Msk;
	r->TIDM[2].reg &= ~GMAC_TIDM3_ENID3_Msk;
	r->TIDM[3].reg &= ~GMAC_TIDM4_ENID4_Msk;
#else
	r->GMAC_TIDM1 &= ~GMAC_TIDM1_ENID1_Msk;
	r->GMAC_TIDM2 &= ~GMAC_TIDM2_ENID2_Msk;
	r->GMAC_TIDM3 &= ~GMAC_TIDM3_ENID3_Msk;
	r->GMAC_TIDM4 &= ~GMAC_TIDM4_ENID4_Msk;
#endif
	MEMORY_STORE_BARRIER();

	/* Disable Wake On Lan */
	r->GMAC_WOL = 0;

#ifndef	GMAC_IS_SAME5X
	/* Disable screening. */
	for (i = 0; i < ARRAY_ELEMENTS(r->GMAC_ST1RPQ); i++)
		r->GMAC_ST1RPQ[i] = 0;
	for (i = 0; i < ARRAY_ELEMENTS(r->GMAC_ST2RPQ); i++)
		r->GMAC_ST2RPQ[i] = 0;
#endif

	MEMORY_STORE_BARRIER();

	/* Configure MAC address */
	reg  = (uint32_t)gs->gs_mac[0] << 0;
	reg |= (uint32_t)gs->gs_mac[1] << 8;
	reg |= (uint32_t)gs->gs_mac[2] << 16;
	reg |= (uint32_t)gs->gs_mac[3] << 24;
#ifdef	GMAC_IS_SAME5X
	r->Sa[0].GMAC_SAB = reg;
#else
	r->GmacSa[0].GMAC_SAB = reg;
#endif
	MEMORY_STORE_BARRIER();
	reg  = (uint32_t)gs->gs_mac[4] << 0;
	reg |= (uint32_t)gs->gs_mac[5] << 8;
#ifdef	GMAC_IS_SAME5X
	r->Sa[0].GMAC_SAT = reg;
#else
	r->GmacSa[0].GMAC_SAT = reg;
#endif
	MEMORY_STORE_BARRIER();

	/* Disable remaining specific address match register pairs. */
#ifdef	GMAC_IS_SAME5X
	for (i = 1; i < ARRAY_ELEMENTS(r->Sa); i++)
		r->Sa[i].GMAC_SAB = 0;
#else
	for (i = 1; i < GMACSA_NUMBER; i++)
		r->GmacSa[i].GMAC_SAB = 0;
#endif

	/* Configure the Rx/Tx queue base addresses. */
	r->GMAC_RBQB = (uint32_t)(uintptr_t)gs->gs_rx_descs;
	r->GMAC_TBQB = (uint32_t)(uintptr_t)gs->gs_tx_descs;

#ifndef	GMAC_IS_SAME5X
	/*
	 * The device has an annoying defect whereby all Tx queues are
	 * checked when TSTART is set; even those which are unconfigured.
	 * To fix this, we must configure dummy descriptors for the
	 * remaining priority queues.
	 */
	for (i = 0; i < (GMAC_DUMMY_DESC_CNT - 1); i++) {
		gmac_dummy_desc.dd_tx[i].gt_payload = NULL;
		gmac_dummy_desc.dd_tx[i].gt_csr = GMAC_TXD_CSR_USED;
		gmac_dummy_desc.dd_rx[i].gr_payload = GMAC_RXD_PAYLOAD_USED;
		gmac_dummy_desc.dd_rx[i].gr_sr = 0;
	}
	gmac_dummy_desc.dd_tx[i].gt_payload = NULL;
	gmac_dummy_desc.dd_tx[i].gt_csr = GMAC_TXD_CSR_USED | GMAC_TXD_CSR_WRAP;
	gmac_dummy_desc.dd_rx[i].gr_payload = GMAC_RXD_PAYLOAD_USED |
	    GMAC_RXD_PAYLOAD_WRAP;
	gmac_dummy_desc.dd_rx[i].gr_sr = 0;

	/* Point the priority queue base registers at the dummy descsriptors. */
	for (i = 0; i < ARRAY_ELEMENTS(r->GMAC_TBQBAPQ); i++) {
		r->GMAC_TBQBAPQ[i] = (uint32_t)gmac_dummy_desc.dd_tx;
		r->GMAC_RBQBAPQ[i] = (uint32_t)gmac_dummy_desc.dd_rx;
	}
	gmac_cache_clean_inv(&gmac_dummy_desc, sizeof(gmac_dummy_desc));
#endif

	/* Reset our queue indices. */
	gs->gs_rx_curr_desc = 0;
	gs->gs_tx_desc_head = gs->gs_tx_desc_tail = 0;

	/* Reset the PHY and start auto negotiation. */
	gmac_phy_write(gs, PHY_REG_BCR, PHY_BCR_RESET);
	rtos_task_sleep(300);
	gmac_phy_write(gs, PHY_REG_ANAR, PHY_ANAR_OUR_DEFAULT);
	gmac_phy_write(gs, PHY_REG_BCR, PHY_BCR_AUTONEG_ENABLE |
	    PHY_BCR_AUTONEG_RESTART);
}

static void
gmac_tx_rx_start(struct gmac_state *gs)
{
	Gmac *r = gs->gs_regs;;
	uint32_t reg;

	/* Configure duplex and speed. */
	reg = r->GMAC_NCFGR & ~(GMAC_NCFGR_SPD_Msk | GMAC_NCFGR_FD_Msk);
	switch (gs->gs_link_status) {
	case GMAC_PHY_LINK_100FDX:
		reg |= GMAC_NCFGR_SPD_Msk | GMAC_NCFGR_FD_Msk;
		break;
	case GMAC_PHY_LINK_100HDX:
		reg |= GMAC_NCFGR_SPD_Msk;
		break;
	case GMAC_PHY_LINK_10FDX:
		reg |= GMAC_NCFGR_FD_Msk;
		break;
	default:
		break;
	}
	r->GMAC_NCFGR = reg;
	MEMORY_STORE_BARRIER();

	/* Enable Tx/Rx. */
	r->GMAC_NCR |= GMAC_NCR_TXEN_Msk | GMAC_NCR_RXEN_Msk;
	MEMORY_STORE_BARRIER();
	r->GMAC_NCR |= GMAC_NCR_FNP_Msk;
	MEMORY_STORE_BARRIER();

	/* Enable GMAC interrupts. */
	r->GMAC_IER = GMAC_INTERRUPT_MASK;
}

static void
gmac_start(struct gmac_state *gs)
{
	struct gmac_rxdesc *gr;
	struct gmac_rx_pbuf *rp;
	struct gmac_tx_pbuf *tp;
	unsigned int i;

	/* Initialise Tx queues. */
	memset(gs->gs_tx_desc2pbuf, 0, sizeof(struct gmac_tx_pbuf *) *
	    gs->gs_tx_desc_cnt);
	STAILQ_INIT(&gs->gs_tx_pbuf_freelist);
	STAILQ_INIT(&gs->gs_tx_pbuf_tx_queue);

	/* Initialise Tx static pbuf list, if any. */
	tp = gs->gs_tx_pbuf_static;
	for (i = 0; i < gs->gs_tx_pbuf_static_cnt; i++, tp++) {
		tp->tp_pbuf = NULL;
		tp->tp_static = 1;
		STAILQ_INSERT_HEAD(&gs->gs_tx_pbuf_freelist, tp, tp_qent);
	}

	/* Initialise Tx descriptor state. */
	gs->gs_tx_desc_free = gs->gs_tx_desc_cnt;
	for (i = 0; i < gs->gs_tx_desc_cnt; i++)
		gs->gs_tx_descs[i].gt_csr = GMAC_TXD_CSR_USED;
	gs->gs_tx_descs[gs->gs_tx_desc_cnt - 1].gt_csr |= GMAC_TXD_CSR_WRAP;

	/* Initialise Rx pbuf freelist. */
	TAILQ_INIT(&gs->gs_rx_pbuf_freelist);
	for (i = 0, rp = gs->gs_rx_pbufs; i < gs->gs_rx_pbuf_cnt; rp++, i++) {
		memset(&rp->rp_pbuf, 0, sizeof(rp->rp_pbuf));
		rp->rp_pbuf.custom_free_function = gmac_pbuf_custom_free;
		rp->rp_state = gs;
		gmac_cache_inv(rp->rp_payload, sizeof(rp->rp_payload));
		TAILQ_INSERT_TAIL(&gs->gs_rx_pbuf_freelist, rp, rp_qent);
	}

	/* Initialise Rx descriptor state. */
	for (i = 0, gr = gs->gs_rx_descs; i < gs->gs_rx_desc_cnt; i++,gr++){
		rp = gmac_rx_pbuf_alloc(gs);
		gmac_rxdesc_init(gs, gr, i, rp);
	}

	gmac_tx_rx_start(gs);

	netif_set_up(&gs->gs_netif);
}

static int
gmac_rx_stop(struct gmac_state *gs)
{
	Gmac *r = gs->gs_regs;

	/* Stop Rx */
	r->GMAC_NCR &= ~GMAC_NCR_RXEN_Msk;
	MEMORY_STORE_BARRIER();

	return 0;
}

static int
gmac_tx_stop(struct gmac_state *gs)
{
	Gmac *r = gs->gs_regs;
	timer_timeout_t to;
	int rv = 0;

	/* Stop Tx. */
	r->GMAC_NCR |= GMAC_NCR_THALT_Msk;
	MEMORY_STORE_BARRIER();

	timer_timeout_start(&to, 500);
	while ((r->GMAC_TSR & GMAC_TSR_TXGO_Msk) != 0 &&
	    !timer_timeout_expired(&to)) {
		rtos_task_sleep(5);
	}

	r->GMAC_NCR &= ~GMAC_NCR_TXEN_Msk;
	MEMORY_STORE_BARRIER();

	timer_timeout_start(&to, 500);
	while (r->GMAC_TBQB != (uint32_t)(uintptr_t)gs->gs_tx_descs &&
	    !timer_timeout_expired(&to)) {
		rtos_task_sleep(5);
	}

	if (r->GMAC_TBQB != (uint32_t)(uintptr_t)gs->gs_tx_descs) {
		DBPRINTF(GMAC_DEBUG_STOP, "gmac_tx_stop: Tx stop timed out.\n");
		rv = -1;
	}

	/*
	 * When Tx has stopped, the controller will reset its Tx pointer to
	 * the first descriptor.
	 */
	gs->gs_tx_desc_head = gs->gs_tx_desc_tail = 0;

	return rv;
}

static void
gmac_stop(struct gmac_state *gs)
{
	struct gmac_tx_pbuf *tp;
	int need_reset = 0;

	DBPRINTF(GMAC_DEBUG_STOP, "gmac_stop: Stopping.\n");

	netif_set_down(&gs->gs_netif);
	netif_set_link_down(&gs->gs_netif);
	gs->gs_link_status = GMAC_PHY_LINK_UNKNOWN;

	if (gmac_rx_stop(gs) < 0)
		need_reset = 1;

	/* Clear any received frames. */
	gmac_rx_reap(gs);

	if (gmac_tx_stop(gs) < 0)
		need_reset = 1;

	/* Reap outstanding Tx packets, if necessary */
	if (gs->gs_tx_desc_free < gs->gs_tx_desc_cnt) {
		DBPRINTF(GMAC_DEBUG_STOP, "gmac_stop: Tx reap.\n");

		gmac_tx_reap(gs, 0);

		if (gs->gs_tx_desc_free < gs->gs_tx_desc_cnt) {
			DBPRINTF(GMAC_DEBUG_STOP, "gmac_stop: Tx reap "
			    "failed.\n");
			need_reset = 1;
		}
	}

	/* Discard any queued Tx pbufs */
	while ((tp = STAILQ_FIRST(&gs->gs_tx_pbuf_tx_queue)) != NULL) {
		STAILQ_REMOVE_HEAD(&gs->gs_tx_pbuf_tx_queue, tp_qent);
		assert(tp->tp_pbuf != NULL);
		pbuf_free(tp->tp_pbuf);
		if (tp->tp_static == 0)
			zone_free(tp);
	}

	if (need_reset) {
		DBPRINTF(GMAC_DEBUG_STOP, "gmac_stop: Reset h/w\n");
		gmac_reset(gs);
	}
}

static err_t
gmac_init(struct netif *netif)
{
	struct gmac_state *gs = netif->state;

#if LWIP_NETIF_HOSTNAME
	netif->hostname = gs->gs_hostname;
#endif

	netif->name[0] = 'e';
	netif->name[1] = 'n';
	netif->output = etharp_output;
	netif->linkoutput = gmac_output;
	memcpy(netif->hwaddr, gs->gs_mac, ETH_HWADDR_LEN);
	netif->hwaddr_len = ETH_HWADDR_LEN;
	netif->mtu = 1500;
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP |
	    NETIF_FLAG_ETHERNET;

	gs->gs_link_status = GMAC_PHY_LINK_UNKNOWN;
	gmac_reset(gs);

	return ERR_OK;
}

static uint32_t
gmac_worker(struct netif *netif)
{
	struct gmac_state *gs = netif->state;
	Gmac *r = gs->gs_regs;
	struct gmac_tx_pbuf *tp;
	uint32_t tsr, rsr;

	for (;;) {
		tsr = r->GMAC_TSR;
		r->GMAC_TSR = tsr;
		rsr = r->GMAC_RSR;
		r->GMAC_RSR = rsr;

		if ((tsr & ~GMAC_TSR_TXGO_Msk) == 0 && rsr == 0)
			break;

		DBPRINTF(GMAC_DEBUG_WORKER, "gmac_worker: TSR 0x%03" PRIx32
		    ", RSR 0x%02" PRIx32 "\n", tsr, rsr);

		/* Rx processing */
		if (rsr) {
			if (gmac_rx_reap(gs) == 0) {
				DBPRINTF(GMAC_DEBUG_WORKER | GMAC_DEBUG_ERROR,
				    "gmac_worker: RSR non-zero but no reap!\n");
			}
		}

		/* Reap transmitted pbufs */
		if (tsr) {
			if (gmac_tx_reap(gs, tsr) == 0) {
				DBPRINTF(GMAC_DEBUG_WORKER | GMAC_DEBUG_ERROR,
				    "gmac_worker: RSR non-zero but no reap!\n");
			}
		}

		/* Try to move more Tx packets. */
		if ((gs->gs_netif.flags & NETIF_FLAG_UP) != 0) {
			tp = STAILQ_FIRST(&gs->gs_tx_pbuf_tx_queue);
			if (tp && tp->tp_desc_cnt <= gs->gs_tx_desc_free) {
				STAILQ_REMOVE_HEAD(&gs->gs_tx_pbuf_tx_queue,
				    tp_qent);
				gmac_tx_start(gs, tp);
			}
		}
	}

	r->GMAC_IER = GMAC_INTERRUPT_MASK;

	return RTOS_MAX_SLEEP_MS;
}

#ifdef GMAC_DEBUG
static const char *
gmac_link_string(enum gmac_link_status link_status)
{

	switch (link_status) {
	case GMAC_PHY_LINK_DOWN:
		return "Down";
	case GMAC_PHY_LINK_100FDX:
		return "100FDX";
	case GMAC_PHY_LINK_100HDX:
		return "100HDX";
	case GMAC_PHY_LINK_10FDX:
		return "10FDX";
	case GMAC_PHY_LINK_10HDX:
		return "10HDX";
	case GMAC_PHY_LINK_UNKNOWN:
		return "Unknown";
	default:
		return "Invalid";
	}
}
#endif /* GMAC_DEBUG */

static void
gmac_link_worker(struct netif *netif)
{
	struct gmac_state *gs = netif->state;
	enum gmac_link_status link_status;
	network_link_status_t lst;
	uint16_t bsr;

	if ((gs->gs_netif.flags & NETIF_FLAG_UP) == 0)
		return;

	bsr = gmac_phy_read(gs, PHY_REG_BSR);
	bsr |= gmac_phy_read(gs, PHY_REG_BSR);
	bsr &= PHY_BSR_LINK_UP | PHY_BSR_AUTONEG_COMPLETE;
	if (bsr != (PHY_BSR_LINK_UP | PHY_BSR_AUTONEG_COMPLETE)) {
		link_status = GMAC_PHY_LINK_DOWN;
	} else {
		uint16_t anar = gmac_phy_read(gs, PHY_REG_ANAR) &
		    gmac_phy_read(gs, PHY_REG_ANLPAR);
		if (anar & PHY_ANAR_100TX_FDX)
			link_status = GMAC_PHY_LINK_100FDX;
		else
		if (anar & PHY_ANAR_100TX_HDX)
			link_status = GMAC_PHY_LINK_100HDX;
		else
		if (anar & PHY_ANAR_10TX_FDX)
			link_status = GMAC_PHY_LINK_10FDX;
		else
		if (anar & PHY_ANAR_10TX_HDX)
			link_status = GMAC_PHY_LINK_10HDX;
		else
			link_status = GMAC_PHY_LINK_DOWN;
	}

	if (link_status == gs->gs_link_status) {
		/* Link status unchanged. */
		return;
	}

	DBPRINTF(GMAC_DEBUG_LINK,"gmac_link_worker: link %s\n",
	    gmac_link_string(link_status));

	gs->gs_link_status = link_status;
	memset(&lst, 0, sizeof(lst));

	if (link_status > GMAC_PHY_LINK_DOWN) {
		lst.ls_state = NETWORK_LINK_UP;
		if (gmac_rx_stop(gs) < 0 || gmac_tx_stop(gs) < 0) {
			DBPRINTF(GMAC_DEBUG_LINK,"gmac_link_worker: Failed to "
			      "stop controller.\n");
		}
		/* Reconfigure the duplex/speed settings. */
		gmac_tx_rx_start(gs);
	} else {
		lst.ls_state = NETWORK_LINK_DOWN;
	}

	if (lst.ls_state == NETWORK_LINK_UP)
		netif_set_link_up(&gs->gs_netif);

	if (gs->gs_link_cb)
		(gs->gs_link_cb)(&lst, gs->gs_link_cb_arg);

	if (lst.ls_state == NETWORK_LINK_DOWN)
		netif_set_link_down(&gs->gs_netif);
}

static void
gmac_link_status(struct netif *netif, network_link_status_callback_t cb,
    void *arg)
{
	struct gmac_state *gs = netif->state;

	gs->gs_link_cb = cb;
	gs->gs_link_cb_arg = arg;
}

static void
gmac_get_mac(struct netif *netif, uint8_t *mac)
{
	struct gmac_state *gs = netif->state;

	memcpy(mac, gs->gs_mac, sizeof(gs->gs_mac));
}

static int8_t
gmac_start_stop(struct netif *netif, int8_t do_start)
{
	struct gmac_state *gs = netif->state;

	if (do_start)
		gmac_start(gs);
	else
		gmac_stop(gs);

	return 0;
}

void
sam_gmac_hw_interrupt(void *cookie)
{
	struct gmac_state *gs = cookie;
	Gmac *r = gs->gs_regs;

	/* Clear interrupt mask. */
	r->GMAC_IDR = GMAC_INTERRUPT_MASK;
	MEMORY_STORE_BARRIER();
	(void) r->GMAC_ISR;

	/* Handle the interrupt at task level. */
	network_driver_hw_interrupt(gs->gs_ns);
}

void *
sam_gmac_attach(const sam_gmac_attach_args_t *gaa)
{
	struct network_lwip_driver *nld;
	struct gmac_state *gs;
	struct gmac_txdesc *tx_descs;
	struct gmac_rxdesc *rx_descs;
	struct gmac_tx_pbuf *tx_pbufs;
	struct gmac_rx_pbuf *rx_pbufs;
	struct gmac_tx_pbuf **txpp;
	struct gmac_rx_pbuf **rxpp;
	uint16_t rx_pbuf_cnt, tx_pbuf_static_cnt;
	uintptr_t p;
	size_t msz;

	assert(gaa->ga_tx_descs >= 2);
	assert(gaa->ga_rx_descs >= 2);
	assert(((uintptr_t)gaa->ga_desc_memory & 0x3u) == 0);
	assert(gaa->ga_phy_num <= 31u);

	/* Apportion supplied memory to Tx and Rx descriptors. */
	tx_descs = (struct gmac_txdesc *)gaa->ga_desc_memory;
	rx_descs = (struct gmac_rxdesc *)(tx_descs + gaa->ga_tx_descs);

	/*
	 * We over-allocate Rx pbufs in order to reduce the chance of
	 * running out under load.
	 */
	rx_pbuf_cnt = gaa->ga_rx_descs + 2;

	/*
	 * We also over-allocate Tx pbufs to reduce the chance of needing
	 * to alloc/free in the critical path.
	 */
	tx_pbuf_static_cnt = gaa->ga_tx_descs * 2;

	/*
	 * All memory is allocated in one parcel before being
	 * divided up. Work out how much is required.
	 */
	msz  = rx_pbuf_cnt * sizeof(struct gmac_rx_pbuf);
	msz += tx_pbuf_static_cnt * sizeof(struct gmac_tx_pbuf);

	/* Rx/Tx descriptor-2-pbuf lookup array. */
	msz += sizeof(struct gmac_rx_pbuf **) * gaa->ga_rx_descs;
	msz += sizeof(struct gmac_tx_pbuf **) * gaa->ga_tx_descs;

	/*
	 * Add some additional space to permit later cache alignment of
	 * the Rx pbufs.
	 */
	msz += 32;

	/*
	 * Allocate what we need from the heap, all in one chunk.
	 */
	msz += sizeof(*gs);
	if ((gs = zone_calloc(1, msz)) == NULL) {
		DBPRINTF(GMAC_DEBUG_ATTACH | GMAC_DEBUG_ERROR,
		    "sam_gmac_attach: Failed to allocate state (%u).\n",
		    (unsigned int)msz);
		return NULL;
	}

	/*
	 * Buffers follow on after the state structure, but we need to ensure
	 * the buffers are aligned to a 32-byte boundary.
	 */
	p = (uintptr_t)(gs + 1);
	p += 31u;
	p &= ~31u;

	/* Buffer is now aligned. */
	rx_pbufs = (struct gmac_rx_pbuf *)(void *)p;
	tx_pbufs = (struct gmac_tx_pbuf *)(rx_pbufs + rx_pbuf_cnt);
	rxpp = (struct gmac_rx_pbuf **)(tx_pbufs + tx_pbuf_static_cnt);
	txpp = (struct gmac_tx_pbuf **)(rxpp + gaa->ga_rx_descs);

	/* Rx pbufs and payload buffers. */
	gs->gs_rx_pbufs = rx_pbufs;
	gs->gs_rx_pbuf_cnt = rx_pbuf_cnt;

	/* Tx static pbufs */
	gs->gs_tx_pbuf_static = tx_pbufs;
	gs->gs_tx_pbuf_static_cnt = tx_pbuf_static_cnt;

	/* Descriptor-2-pbuf arrays. */
	gs->gs_rx_desc2pbuf = rxpp;
	gs->gs_tx_desc2pbuf = txpp;

	/* Tx descriptors. */
	gs->gs_tx_descs = tx_descs;
	gs->gs_tx_desc_cnt = gaa->ga_tx_descs;

	/* Rx descriptors. */
	gs->gs_rx_descs = rx_descs;
	gs->gs_rx_desc_cnt = gaa->ga_rx_descs;

	gs->gs_regs = gaa->ga_regs;
	gs->gs_clock = gaa->ga_clock;
	gs->gs_phy_man = GMAC_MAN_WTN(2) |
	    (gaa->ga_phy_clause45 ? 0 : GMAC_MAN_CLTTO_Msk) |
	    GMAC_MAN_PHYA((uint32_t)gaa->ga_phy_num);
	gs->gs_hostname = gaa->ga_hostname;
	memcpy(gs->gs_mac, gaa->ga_mac, sizeof(gs->gs_mac));

	nld = &gs->gs_nld;
	nld->nld_netif = &gs->gs_netif;
	nld->nld_handle_interrupt = gmac_worker;
	nld->nld_link_check = gmac_link_worker;
	nld->nld_link_status = gmac_link_status;
	nld->nld_get_mac = gmac_get_mac;
	nld->nld_ioctl = NULL;
	nld->nld_start_stop = gmac_start_stop;

	if (netif_add(&gs->gs_netif, NULL, NULL, NULL, gs, gmac_init,
	    netif_input) == NULL) {
		zone_free(gs);
		DBPRINTF(GMAC_DEBUG_ATTACH | GMAC_DEBUG_ERROR,
		    "sam_gmac_attach: netif_add() failed\n");
		return NULL;
	}

	netif_set_default(&gs->gs_netif);

	if (gmac_start_stop(&gs->gs_netif, 1) < 0 ||
	    (gs->gs_ns = network_lwip_attach(nld)) == NULL) {
		gmac_start_stop(&gs->gs_netif, 0);
		netif_remove(&gs->gs_netif);
		zone_free(gs);
		DBPRINTF(GMAC_DEBUG_ATTACH | GMAC_DEBUG_ERROR,
		    "sam_gmac_attach: gmac_start_stop() or "
		    "network_lwip_attach() failed\n");
		return NULL;
	}

	DBPRINTF(GMAC_DEBUG_ATTACH, "sam_gmac_attach:\n\tDesc memory: %p\n",
	    gaa->ga_desc_memory);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tState size: 0x%x\n", (unsigned int)msz);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tState base: %p\n", (void *)gs);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tSizeof rx_pbuf: 0x%x\n",
	    (unsigned int)sizeof(struct gmac_rx_pbuf));
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tSizeof tx_pbuf: 0x%x\n",
	    (unsigned int)sizeof(struct gmac_tx_pbuf));
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tTx descriptor count: %" PRIu16 "\n",
	    gaa->ga_tx_descs);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tTx descriptor base: %p\n",
	    (void *)gs->gs_tx_descs);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tTx desc2pbuf base: %p\n",
	    (void *)gs->gs_tx_desc2pbuf);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tTx static pbuf count: %" PRIu16 "\n",
	    tx_pbuf_static_cnt);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tTx static pbuf base: %p\n",
	    (void *)gs->gs_tx_pbuf_static);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tRx descriptor count: %" PRIu16 "\n",
	    gaa->ga_rx_descs);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tRx descriptor base: %p\n",
	    (void *)gs->gs_rx_descs);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tRx desc2pbuf base: %p\n",
	    (void *)gs->gs_rx_desc2pbuf);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tRx buffer count: %" PRIu16 "\n",
	    gs->gs_rx_pbuf_cnt);
	DBPRINTF(GMAC_DEBUG_ATTACH, "\tMAC address: "
	    "%02x:%02x:%02x:%02x:%02x:%02x\n",
	    (unsigned int)gaa->ga_mac[0], (unsigned int)gaa->ga_mac[1],
	    (unsigned int)gaa->ga_mac[2], (unsigned int)gaa->ga_mac[3],
	    (unsigned int)gaa->ga_mac[4], (unsigned int)gaa->ga_mac[5]);

	return gs;
}
