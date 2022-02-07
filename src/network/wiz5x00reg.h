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

#ifndef	WIZ5x00REG_H
#define	WIZ5x00REG_H

/*
 * W5200 Memory map
 */
#define	WIZ5200_MAP_COMMON_REGS		0x0000u
#define	WIZ5200_MAP_COMMON_REGS_SZ	0x37u
#define	WIZ5200_MAP_SOCKET_REGS(s)	(0x4000u + (0x100u * (s)))
#define	WIZ5200_MAP_SOCKET_REGS_SZ	0x800u
#define	WIZ5200_MAP_TX_MEMORY		0x8000u
#define	WIZ5200_MAP_TX_MEMORY_SZ	0x4000u
#define	WIZ5200_MAP_RX_MEMORY		0xc000u
#define	WIZ5200_MAP_RX_MEMORY_SZ	0x4000u

/************************
 * SPI Transaction Header
 */
struct wiz5200_spi_header {
	uint16_t sh_addr;
	uint16_t sh_len;
};

static __always_inline uint16_t
wiz5200_spi_header_write(struct wiz5200_spi_header *sh, uint16_t addr,
    uint16_t len)
{

	sh->sh_addr = hton16(addr);
	len &= 0x7fffu;
	sh->sh_len = hton16(len | 0x8000u);

	return len;
}

static __always_inline uint16_t
wiz5200_spi_header_read(struct wiz5200_spi_header *sh, uint16_t addr,
    uint16_t len)
{

	sh->sh_addr = hton16(addr);
	len &= 0x7fffu;
	sh->sh_len = hton16(len);

	return len;
}

/*
 * W5500 Memory blocks
 */
#define	WIZ5500_BLK_COMMON		(0x00u << 3)
#define	WIZ5500_BLK_REG(s)		((0x01u + ((s) * 4)) << 3)
#define	WIZ5500_BLK_TX(s)		((0x02u + ((s) * 4)) << 3)
#define	WIZ5500_BLK_RX(s)		((0x03u + ((s) * 4)) << 3)

#define	WIZ5500_AM_READ			(0u << 2)
#define	WIZ5500_AM_WRITE		(1u << 2)

#define	WIZ5500_OM_VDM			(0x00u << 0)
#define	WIZ5500_OM_FDM1			(0x01u << 0)
#define	WIZ5500_OM_FDM2			(0x02u << 0)
#define	WIZ5500_OM_FDM4			(0x03u << 0)
#define	WIZ5500_OM_F(v)			(((v) >> 1) + 1)

/************************
 * SPI Transaction Header
 */
struct wiz5500_spi_header {
	uint16_t sh_addr;
	uint8_t sh_ctrl;
	uint8_t _sh_dummy;
};

static __always_inline void
wiz5500_spi_header_write(struct wiz5500_spi_header *sh, uint8_t blk,
    uint16_t addr, uint8_t om)
{

	sh->sh_addr = hton16(addr);
	sh->sh_ctrl = blk | om | WIZ5500_AM_WRITE;
}

static __always_inline void
wiz5500_spi_header_read(struct wiz5500_spi_header *sh, uint8_t blk,
    uint16_t addr, uint8_t om)
{

	sh->sh_addr = hton16(addr);
	sh->sh_ctrl = blk | om | WIZ5500_AM_READ;
}


/******************
 * Common Registers
 */
/*  8: Mode Register */
#define	WIZ5x00_CREG_MR			0x00
#define	 WIZ5x00_CREG_MR_RST		(1u << 7) /* Software Reset */
#define	 WIZ5500_CREG_MR_WOL		(1u << 5) /* Enable WOL */
#define	 WIZ5x00_CREG_MR_PB		(1u << 4) /* Enable Ping Block */
#define	 WIZ5200_CREG_MR_PPPoE		(1u << 3) /* Enable PPPoE Mode */
#define	 WIZ5500_CREG_MR_FARP		(1u << 1) /* Enable Force ARP */

/* 32: Gateway IP Address */
#define	WIZ5x00_CREG_GAR		0x01

/* 32: Subnet Mask */
#define	WIZ5x00_CREG_SUBR		0x05

/* 48: Source MAC Address */
#define	WIZ5x00_CREG_SHAR		0x09

/* 32: Source IP Address */
#define	WIZ5x00_CREG_SIPR		0x0f

/* 16: Interrupt Low Level Timer */
#define	WIZ5500_CREG_INTLEVEL		0x13
#define	WIZ5200_CREG_INTLEVEL		0x30

/* 8: Interrupt Status and Interrupt Mask Registers */
#define	WIZ5x00_CREG_IR			0x15
#define	WIZ5200_CREG_IMR		0x36
#define	WIZ5500_CREG_IMR		0x16
#define	 WIZ5x00_CREG_IR_CONFLICT	(1u << 7) /* IP Conflict */
#define	 WIZ5500_CREG_IR_UNREACH	(1u << 6) /* Destination unreachable  */
#define	 WIZ5x00_CREG_IR_PPPoE		(1u << 5) /* PPPoE Connection Close */
#define	 WIZ5500_CREG_IR_MP		(1u << 4) /* WOL Magic Packet */

/* 8: Socket Interrupt Status and Interrupt Mask registers */
#define	WIZ5200_CREG_IR2		0x34
#define	WIZ5200_CREG_IMR2		0x16
#define	WIZ5500_CREG_SIR		0x17
#define	WIZ5500_CREG_SIMR		0x18
#define  WIZ5x00_CREG_SIR_SOCK(s)	(1u << (s))

/* 16: Retry Time */
#define	WIZ5200_CREG_RTR		0x17
#define	WIZ5500_CREG_RTR		0x19

/* 8: Retry Count */
#define	WIZ5200_CREG_RCR		0x19
#define	WIZ5500_CREG_RCR		0x1b

/* 32: Unreachable IP Address */
#define	WIZ5500_CREG_UIPR		0x28

/* 16: Unreachable Port */
#define	WIZ5500_CREG_UPORTR		0x2c

/* 8: PHY Configuration */
#define	WIZ5200_CREG_PSTATUS		0x35
#define	 WIZ5200_CREG_PHY_LINK		(1u << 5)
#define	 WIZ5200_CREG_PHY_PWRDN		(1u << 3)
#define	WIZ5500_CREG_PHYCFGR		0x2e
#define  WIZ5500_CREG_PHY_RST		(1u << 7)
#define  WIZ5500_CREG_PHY_OPMD		(1u << 6)
#define  WIZ5500_CREG_PHY_OPMDC_10H	(0u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_10F	(1u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_100H	(2u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_100F	(3u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_100FA	(4u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_AUTO	(7u << 3)
#define  WIZ5500_CREG_PHY_OPMDC_MASK	(7u << 3)
#define  WIZ5500_CREG_PHY_DPX		(1u << 2)
#define  WIZ5500_CREG_PHY_SPD100	(1u << 1)
#define  WIZ5500_CREG_PHY_LINK		(1u << 0)

/* 8: Chip Version */
#define	WIZ5200_CREG_VERSIONR		0x1f
#define	WIZ5500_CREG_VERSIONR		0x39


/******************
 * Socket Registers
 */
#define	WIZ5x00_NSOCKETS		8

/*  8: Socket Mode Register */
#define	WIZ5x00_SREG_MR			0x00
#define	 WIZ5x00_SREG_MR_MULTI		(1u << 7) /* Enable Multicast (UDP) */
#define	 WIZ5500_SREG_MR_MFEN		(1u << 7) /* Enable MAC filtering */
#define	 WIZ5200_SREG_MR_MF		(1u << 6) /* Disable Promiscuous */
#define	 WIZ5500_SREG_MR_BCASTB		(1u << 6) /* Broadcast Blocking */
#define	 WIZ5x00_SREG_MR_NDMC		(1u << 5) /* DelACK / MC version */
#define	 WIZ5500_SREG_MR_UCASTB		(1u << 4) /* Unicast Blocking */
#define	 WIZ5x00_SREG_MR_P_MASK		0x0fu
#define	 WIZ5x00_SREG_MR_P_CLOSED	0x00u
#define	 WIZ5x00_SREG_MR_P_TCP		0x01u
#define	 WIZ5x00_SREG_MR_P_UDP		0x02u
#define	 WIZ5200_SREG_MR_P_IPRAW	0x03u
#define	 WIZ5x00_SREG_MR_P_MACRAW	0x04u /* Socket 0 only */
#define	 WIZ5200_SREG_MR_P_PPPoE	0x05u /* Socket 0 only */

/*  8: Socket Command Register */
#define	WIZ5x00_SREG_CR			0x01
#define	 WIZ5x00_SREG_CR_OPEN		0x01u
#define	 WIZ5x00_SREG_CR_LISTEN		0x02u
#define	 WIZ5x00_SREG_CR_CONNECT	0x04u
#define	 WIZ5x00_SREG_CR_DISCON		0x08u
#define	 WIZ5x00_SREG_CR_CLOSE		0x10u
#define	 WIZ5x00_SREG_CR_SEND		0x20u
#define	 WIZ5x00_SREG_CR_SEND_MAC	0x21u
#define	 WIZ5x00_SREG_CR_SEND_KEEP	0x22u
#define	 WIZ5200_SREG_CR_PCON		0x23u	/* Socket 0, PPPoE Mode */
#define	 WIZ5200_SREG_CR_PDISCON	0x24u	/* Socket 0, PPPoE Mode */
#define	 WIZ5200_SREG_CR_PCR		0x25u	/* Socket 0, PPPoE Mode */
#define	 WIZ5200_SREG_CR_PCN		0x26u	/* Socket 0, PPPoE Mode */
#define	 WIZ5200_SREG_CR_PCJ		0x27u	/* Socket 0, PPPoE Mode */
#define	 WIZ5x00_SREG_CR_RECV		0x40u

/*  8: Socket Interrupt Status Register */
#define	WIZ5x00_SREG_IR			0x02
#define  WIZ5200_SREG_IR_PRECV		(1u << 7) /* Socket 0, PPPoE Mode */
#define  WIZ5200_SREG_IR_PFAIL		(1u << 6) /* Socket 0, PPPoE Mode */
#define  WIZ5200_SREG_IR_PNEXT	(1u << 5) /* Socket 0, PPPoE Mode */
#define	 WIZ5x00_SREG_IR_SEND_OK	(1u << 4)
#define	 WIZ5x00_SREG_IR_TIMEOUT	(1u << 3)
#define	 WIZ5x00_SREG_IR_RECV		(1u << 2)
#define	 WIZ5x00_SREG_IR_DISCON		(1u << 1)
#define	 WIZ5x00_SREG_IR_CON		(1u << 0)

/*  8: Socket Status Register */
#define	WIZ5x00_SREG_SR			0x03
#define	 WIZ5x00_SREG_SR_CLOSED		0x00u
#define	 WIZ5x00_SREG_SR_INIT		0x13u
#define	 WIZ5x00_SREG_SR_LISTEN		0x14u
#define	 WIZ5x00_SREG_SR_ESTABLISHED	0x17u
#define	 WIZ5x00_SREG_SR_CLOSE_WAIT	0x1cu
#define	 WIZ5x00_SREG_SR_UDP		0x22u
#define	 WIZ5200_SREG_SR_IPRAW		0x32u
#define	 WIZ5x00_SREG_SR_MACRAW		0x42u /* Socket 0 only */
#define	 WIZ5x00_SREG_SR_SYNSENT	0x15u /* Status */
#define	 WIZ5x00_SREG_SR_SYNRECV	0x16u /* Status */
#define	 WIZ5x00_SREG_SR_FIN_WAIT	0x18u /* Status */
#define	 WIZ5x00_SREG_SR_CLOSING	0x1au /* Status */
#define	 WIZ5x00_SREG_SR_TIME_WAIT	0x1bu /* Status */
#define	 WIZ5x00_SREG_SR_LAST_ACK	0x1du /* Status */
#define	 WIZ5200_SREG_SR_ARP		0x01u /* Status */

/* 16: Source Port */
#define	WIZ5x00_SREG_PORT		0x04

/* 48: Destination MAC Address */
#define	WIZ5x00_SREG_DHAR		0x06

/* 32: Destination IP Address */
#define	WIZ5x00_SREG_DIPR		0x0c

/* 16: Destination Port */
#define	WIZ5x00_SREG_DPORT		0x10

/* 16: Max Segment Size */
#define	WIZ5x00_SREG_MSSR		0x12

/*  8: Raw IP Protocol */
#define	WIZ5200_SREG_PROTO		0x14

/*  8: IP TOS */
#define	WIZ5x00_SREG_TOS		0x15

/*  8: IP TTL */
#define	WIZ5x00_SREG_TTL		0x16

/* 8: Rx Memory Size, in units of 1KB */
#define	WIZ5x00_SREG_RXMEM_SIZE		0x1e

/* 8: Tx Memory Size, in units of 1KB */
#define	WIZ5x00_SREG_TXMEM_SIZE		0x1f

/* 16: Tx Free Size, in bytes */
#define	WIZ5x00_SREG_TX_FSR		0x20

/* 16: Tx Read Pointer */
#define	WIZ5x00_SREG_TX_RD		0x22

/* 16: Tx Write Pointer */
#define	WIZ5x00_SREG_TX_WR		0x24

/* 16: Rx Received Size, in bytes */
#define	WIZ5x00_SREG_RX_RSR		0x26

/* 16: Rx Read Pointer */
#define	WIZ5x00_SREG_RX_RD		0x28

/* 16: Rx Write Pointer */
#define	WIZ5x00_SREG_RX_WR		0x2a

/*  8: Interrupt Mask. See WIZ5500_SREG_IR for bit definitions */
#define	WIZ5x00_SREG_IMR		0x2c

/* 16: Frag Offset in Header */
#define	WIZ5x00_SREG_FRAG		0x2d

/*  8: Keep Alive Timer */
#define	WIZ5500_SREG_KPALVTR		0x2f

/*
 * Sizes of on-chip Rx/Tx buffers
 */
#define	WIZ5x00_BUFFER_SIZE_TX		16384u
#define	WIZ5x00_BUFFER_SIZE_RX		16384u

#endif /* WIZ5x00REG_H */
