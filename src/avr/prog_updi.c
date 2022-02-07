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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "prog_pdi.h"
#include "avr_ll.h"
#include "hardware.h"
#include "shell.h"
#include "programmer.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	prog_updi_debug
#include "debug.h"

/*
 * Low-level UPDI commands
 */
#define UPDI_CMD_LDS(sza, szb)	(0x00u | ((sza) << 2) | (szb))
#define UPDI_CMD_STS(sza, szb)	(0x40u | ((sza) << 2) | (szb))
#define UPDI_CMD_LD(ptr, szab)	(0x20u | ((ptr) << 2) | (szab))
#define UPDI_CMD_ST(ptr, szab)	(0x60u | ((ptr) << 2) | (szab))
#define UPDI_CMD_LDCS(csa)	(0x80u | (csa))
#define UPDI_CMD_STCS(csa)	(0xc0u | (csa))
#define UPDI_CMD_REPEAT(szb)	(0xa0u | (szb))
#define UPDI_CMD_KEY(sib,szc)	(0xe0u | ((sib) << 2) | (szc))

/* Values for the various command fields */
#define UPDI_SZA_1BYTE		0	/* Address size: 1 byte */
#define UPDI_SZA_2BYTES		1	/* Address size: 2 bytes */
#define UPDI_SZB_1BYTE		0	/* Data size: 1 byte */
#define UPDI_SZB_2BYTES		1	/* Data size: 2 bytes */
#define UPDI_SZC_8BYTES		0	/* Key size: 8 bytes */
#define UPDI_SZC_16BYTES	1	/* Key size: 16 bytes */
#define UPDI_PTR_INDIRECT	0	/* Pointer: *(ptr) */
#define UPDI_PTR_INDIRECT_PI	1	/* Pointer: *(ptr++) */
#define UPDI_PTR_DIRECT		2	/* Pointer */
#define UPDI_SIB_RECEIVE_KEY	0	/* SIB: Receive Key */
#define UPDI_SIB_SEND_SIB	1	/* SIB: Send SIB */

/* UPDI interface registers (for LDCS/STCS commands) */
#define	UPDI_CSA_STATUSA	0	/* UPDI Status Reg A */
#define	UPDI_CSA_STATUSB	1	/* UPDI Status Reg B */
#define	UPDI_CSA_CTRLA		2	/* UPDI Control Reg A */
#define	UPDI_CSA_CTRLB		3	/* UPDI Control Reg B */
#define	UPDI_CSA_ASI_KEY_STATUS	7	/* UPDI ASI Key status */
#define	UPDI_CSA_ASI_RESET_REQ	8	/* UPDI ASI reset request */
#define	UPDI_CSA_ASI_CTRLA	9	/* UPDI ASI CTRL A */
#define	UPDI_CSA_ASI_SYS_CTRLA	10	/* UPDI ASI Sys CTRL A */
#define	UPDI_CSA_ASI_SYS_STATUS	11	/* UPDI ASI Sys Status */
#define	UPDI_CSA_ASI_CRC_STATUS	12	/* UPDI ASI CRC Status */

/* Fields within the above registers */
#define	UPDI_STATUSA_UPDIREV(x)		(((x) & 0xf0u) >> 4)

#define	UPDI_STATUSB_PESIG(x)		(((x) & 0x07u) >> 0)
#define	UPDI_STATUSB_PESIG_NO_ERROR	0
#define	UPDI_STATUSB_PESIG_PARITY	1
#define	UPDI_STATUSB_PESIG_FRAME	2
#define	UPDI_STATUSB_PESIG_AL_TIMEOUT	3
#define	UPDI_STATUSB_PESIG_CLK_ERROR	4
#define	UPDI_STATUSB_PESIG_CONTENTION	7

#define	UPDI_CTRLA_GTVAL(x)		((x) & 0x07u)
#define	UPDI_CTRLA_GTVAL_CYCLES_128	0	/* Default */
#define	UPDI_CTRLA_GTVAL_CYCLES_64	1
#define	UPDI_CTRLA_GTVAL_CYCLES_32	2
#define	UPDI_CTRLA_GTVAL_CYCLES_16	3
#define	UPDI_CTRLA_GTVAL_CYCLES_8	4
#define	UPDI_CTRLA_GTVAL_CYCLES_4	5
#define	UPDI_CTRLA_GTVAL_CYCLES_2	6
#define	UPDI_CTRLA_GTVAL_OFF		7
#define	UPDI_CTRLA_RSD			(1u << 3)
#define	UPDI_CTRLA_DTD			(1u << 4)
#define	UPDI_CTRLA_PARD			(1u << 5)
#define	UPDI_CTRLA_IBDLY		(1u << 7)

#define	UPDI_CTRLB_UPDIDIS		(1u << 2)
#define	UPDI_CTRLB_CCDETDIS		(1u << 3)
#define	UPDI_CTRLB_NACKDIS		(1u << 4)

#define	UPDI_ASI_KEY_STATUS_CHIPERASE	(1u << 3)
#define	UPDI_ASI_KEY_STATUS_NVMPROG	(1u << 4)
#define	UPDI_ASI_KEY_STATUS_UROWWRITE	(1u << 5)

#define	UPDI_ASI_RESET_REQ_RSTREQ	0x59u

#define	UPDI_ASI_CTRLA_UPDICLKSEL(x)	((x) & 0x03u)
#define	UPDI_ASI_CTRLA_CLK_16MHZ	1u
#define	UPDI_ASI_CTRLA_CLK_8MHZ		2u
#define	UPDI_ASI_CTRLA_CLK_4MHZ		3u	/* Default */
#define	UPDI_ASI_CTRLA_CLK_MASK		3u

#define	UPDI_ASI_SYS_CTRLA_CLKREQ	(1u << 0)
#define	UPDI_ASI_SYS_CTRLA_UROWWRITE_FINAL	(1u << 1)

#define	UPDI_ASI_SYS_STATUS_LOCKSTATUS	(1u << 0)
#define	UPDI_ASI_SYS_STATUS_UROWPROG	(1u << 2)
#define	UPDI_ASI_SYS_STATUS_NVMPROG	(1u << 3)
#define	UPDI_ASI_SYS_STATUS_INSLEEP	(1u << 4)
#define	UPDI_ASI_SYS_STATUS_RSTSYS	(1u << 5)

#define	UPDI_ASI_CRC_STATUS(x)		((x) & 0x7u)
#define	UPDI_ASI_CRC_STATUS_IDLE	0u
#define	UPDI_ASI_CRC_STATUS_BUSY	1u
#define	UPDI_ASI_CRC_STATUS_DONE_OK	2u
#define	UPDI_ASI_CRC_STATUS_DONE_FAILED	4u

/* Sync/ACK values during communication */
#define	UPDI_SYNCH			0x55u
#define	UPDI_ACK			0x40u

/* Keys */
static const uint8_t updi_key_chiperase[] = {
	0x65, 0x73, 0x61, 0x72, 0x45, 0x4d, 0x56, 0x4e
};
static const uint8_t updi_key_nvmprog[] = {
	0x20, 0x67, 0x6f, 0x72, 0x50, 0x4d, 0x56, 0x4e
};
#if 0	/* Currently unused */
static const uint8_t updi_key_userrow_write[] = {
	0x65, 0x74, 0x26, 0x73, 0x55, 0x4d, 0x56, 0x4e
};
#endif

/* System Information Block (SIB) */
struct updi_sib {
	uint8_t us_family[7];
	uint8_t us_resvd1;
	uint8_t us_nvm_version[3];
	uint8_t us_ocd_version[3];
	uint8_t us_resvd2;
	uint8_t us_dbg_osc_freq;
} __attribute__((__packed__));


/*
 * Initial UPDI baud rate upon sign-on. Guaranteed to work regardless of
 * the UPDI clock.
 */
#define	PROG_PDI_INIT_BAUD	115200

/*
 * NVM Controller Registers
 */
#define UPDI_NVM_REG_CTRLA			0x00
#define UPDI_NVM_REG_CTRLB			0x01
#define UPDI_NVM_REG_STATUS			0x02
#define UPDI_NVM_REG_INTCTRL			0x03
#define UPDI_NVM_REG_INTFLAGS			0x04
#define UPDI_NVM_REG_DATAL			0x06
#define UPDI_NVM_REG_DATAH			0x07
#define UPDI_NVM_REG_ADDRL			0x08
#define UPDI_NVM_REG_ADDRH			0x09

/* Fields within the above registers */
#define	UPDI_NVM_CTRLA_CMD(x)			((x) & 0x07u)
#define	UPDI_NVM_CMD_NOP			0
#define	UPDI_NVM_CMD_WRITE_PAGE			1
#define	UPDI_NVM_CMD_ERASE_PAGE			2
#define	UPDI_NVM_CMD_ERASE_WRITE_PAGE		3
#define	UPDI_NVM_CMD_PAGE_BUFFER_CLEAR		4
#define	UPDI_NVM_CMD_CHIP_ERASE			5
#define	UPDI_NVM_CMD_EEPROM_ERASE		6
#define	UPDI_NVM_CMD_WRITE_FUSE			7

#define	UPDI_NVM_CTRLB_APCWP			(1u << 0)
#define	UPDI_NVM_CTRLB_BOOTLOCK			(1u << 1)

#define	UPDI_NVM_STATUS_FBUSY			(1u << 0)
#define	UPDI_NVM_STATUS_EEBUSY			(1u << 1)
#define	UPDI_NVM_STATUS_WRERROR			(1u << 2)

#define	UPDI_NVM_INTCTRL_EEREADY		(1u << 0)
#define	UPDI_NVM_INTFLAGS_EEREADY		(1u << 0)


struct updi_state {
	bool ps_sync_active;
	bool ps_nvm_active;
	volatile bool ps_timer_expired;
	struct prog_updi_param_arg ps_params;
	uint16_t ps_timeout;
	uint16_t ps_rx_yield_len;
	rtos_timer_t ps_timer;
};
static struct updi_state updi_state;

SHELL_CMD_DECL(updi, updi_cmd, "Configure UPDI programming");

static void
prog_updi_timer_start(struct updi_state *ps, uint16_t ms)
{

	rtos_timer_stop(ps->ps_timer);
	rtos_timer_change_wait_period(ps->ps_timer, ms);
	ps->ps_timer_expired = false;
	rtos_timer_start(ps->ps_timer);
}

static void
prog_updi_timer_stop(struct updi_state *ps)
{

	rtos_timer_stop(ps->ps_timer);
	ps->ps_timer_expired = false;
}

static void
prog_updi_error_recovery(struct updi_state *ps)
{
	uint8_t tx[2], rv;
	volatile uint8_t rx;
	u_int retry;

	(void) ps;

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LDCS(UPDI_CSA_STATUSB);

	if (avr_ll_pdi_transfer(tx, 2, (uint8_t *)(uintptr_t)&rx, 1) == 0) {
		/*
		 * Busy-wait for Rx
		 */
		retry = 1000;

		do {
			rv = avr_ll_pdi_rx_status();
			if (rv == AVR_LL_PDI_RX_BUSY)
				rtos_task_sleep(2);
		} while (rv == AVR_LL_PDI_RX_BUSY && --retry);

		if (rv != AVR_LL_PDI_RX_DONE) {
			DBFPRINTF("Rx not done\n");
		} else {
			DBFPRINTF("Rx stat %u\n", rx);
			if (rx == 0)
				return;
		}
	}

	/*
	 * Three things can land us here:
	 *
	 *  - We failed to transmit the STATUSB read request (unlikely)
	 *  - Timeout/error receiving the response
	 *  - STATUSB.PESIG is non-zero.
	 *
	 * In all cases, send a BREAK.
	 */
	avr_ll_pdi_send_break(1, 1);
	DBFPRINTF("BREAK sent\n");
}

static uint8_t
prog_updi_readwrite(struct updi_state *ps,
    const uint8_t *txbuff, uint32_t txlen,
    uint8_t *rxbuff, uint32_t rxlen)
{
	uint8_t rv;

	rv = avr_ll_pdi_transfer(txbuff, txlen, rxbuff, rxlen);
	if (rv != 0) {
		DBFPRINTF("usart error\n");
		return rv;
	}

	if (rxlen != 0) {
		rxlen = (rxlen >= ps->ps_rx_yield_len);

		do {
			rv = avr_ll_pdi_rx_status();
			if (rv == AVR_LL_PDI_RX_BUSY && rxlen)
				rtos_yield();
		} while (rv == AVR_LL_PDI_RX_BUSY && !ps->ps_timer_expired);
	}

	switch (rv) {
	default:
		rv = 0;
		break;
	case AVR_LL_PDI_RX_BUSY:
		rv = 1;
		DBFPRINTF("timed out\n");
		break;
	case AVR_LL_PDI_RX_CORRUPT:
		rv = 1;
		DBFPRINTF("data corruption\n");
		break;
	}

	if (rv) {
		/*
		 * Let's try to recover from the error.
		 */
		prog_updi_error_recovery(ps);
	}

	return rv;
}

static uint8_t
prog_updi_load_updi_reg(struct updi_state *ps, uint8_t reg, uint8_t *vp)
{
	uint8_t tx[2], rv;

	DBFPRINTF("reg %02x\n", reg);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LDCS(reg);

	if ((rv = prog_updi_readwrite(ps, tx, 2, vp, 1)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u (%02x)\n", rv, *vp);

	return rv;
}

static uint8_t
prog_updi_store_updi_reg(struct updi_state *ps, uint8_t reg, uint8_t v)
{
	uint8_t tx[3], rv;

	DBFPRINTF("reg %02x\n", reg);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_STCS(reg);
	tx[2] = v;

	if ((rv = prog_updi_readwrite(ps, tx, 3, NULL, 0)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u (%02x)\n", rv, v);

	return rv;
}

static uint8_t
prog_updi_load_byte(struct updi_state *ps, uint16_t addr, uint8_t *vp)
{
	uint8_t tx[4], rv;

	DBFPRINTF("addr %04x\n", addr);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LDS(UPDI_SZA_2BYTES, UPDI_SZB_1BYTE);
	tx[2] = (uint8_t)(addr & 0xffu);
	tx[3] = (uint8_t)(addr >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 4, vp, 1)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u (%02x)\n", rv, *vp);

	return rv;
}

#if 0	/* Currently unused */
static uint8_t
prog_updi_load_word(struct updi_state *ps, uint16_t addr, uint16_t *vp)
{
	uint8_t tx[4], rx[2], rv;

	DBFPRINTF("addr %04x\n", addr);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LDS(UPDI_SZA_2BYTES, UPDI_SZB_2BYTES);
	tx[2] = (uint8_t)(addr & 0xffu);
	tx[3] = (uint8_t)(addr >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 4, rx, 2)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	*vp = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

	DBFPRINTF("result %u (%04x)\n", rv, *vp);

	return rv;
}
#endif

static uint8_t
prog_updi_store_byte(struct updi_state *ps, uint16_t addr, uint8_t v)
{
	uint8_t tx[4], rx, rv;

	DBFPRINTF("addr %04x, data %02x\n", addr, v);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_STS(UPDI_SZA_2BYTES, UPDI_SZB_1BYTE);
	tx[2] = (uint8_t)(addr & 0xffu);
	tx[3] = (uint8_t)(addr >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 4, &rx, 1)) != 0) {
		DBFPRINTF("cmd readwrite failed\n");
		return rv;
	}

	if (rx != UPDI_ACK) {
		DBFPRINTF("No cmd ACK (%02x)\n", rx);
		prog_updi_error_recovery(ps);
		return 1;
	}

	if ((rv = prog_updi_readwrite(ps, &v, 1, &rx, 1)) != 0) {
		DBFPRINTF("data readwrite failed\n");
		return rv;
	}

	if (rx != UPDI_ACK) {
		DBFPRINTF("No data ACK (%02x)\n", rx);
		prog_updi_error_recovery(ps);
		return 1;
	}

	return 0;
}

static uint8_t
prog_updi_store_word(struct updi_state *ps, uint16_t addr, uint16_t v)
{
	uint8_t tx[4], rx[2], rv;

	DBFPRINTF("addr %04x, data %04x\n", addr, v);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_STS(UPDI_SZA_2BYTES, UPDI_SZB_2BYTES);
	tx[2] = (uint8_t)(addr & 0xffu);
	tx[3] = (uint8_t)(addr >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 4, rx, 1)) != 0) {
		DBFPRINTF("cmd readwrite failed\n");
		return rv;
	}

	if (rx[0] != UPDI_ACK) {
		DBFPRINTF("No cmd ACK (%02x)\n", rx[0]);
		prog_updi_error_recovery(ps);
		return 1;
	}

	tx[0] = (uint8_t)(v & 0xffu);
	tx[1] = (uint8_t)(v >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 2, rx, 1)) != 0) {
		DBFPRINTF("data readwrite failed\n");
		return rv;
	}

	if (rx[0] != UPDI_ACK) {
		DBFPRINTF("No data ACK (%02x)\n", rx[0]);
		prog_updi_error_recovery(ps);
		return 1;
	}

	return 0;
}

static uint8_t
prog_updi_set_pointer(struct updi_state *ps, uint16_t addr)
{
	uint8_t tx[4], rx, rv;

	DBFPRINTF("addr %04x\n", addr);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_ST(UPDI_PTR_DIRECT, UPDI_SZB_2BYTES);
	tx[2] = (uint8_t)(addr & 0xffu);
	tx[3] = (uint8_t)(addr >> 8);

	if ((rv = prog_updi_readwrite(ps, tx, 4, &rx, 1)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	if (rx != UPDI_ACK) {
		DBFPRINTF("No ACK (%02x)\n", rx);
		prog_updi_error_recovery(ps);
		return 1;
	}

	DBFPRINTF("result %u\n", rv);

	return rv;
}

static uint8_t
prog_updi_load_bytes_ptr_inc(struct updi_state *ps, uint8_t *vp, uint16_t len)
{
	uint8_t tx[2], rv;

	DBFPRINTF("len %u\n", len);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LD(UPDI_PTR_INDIRECT_PI, UPDI_SZB_1BYTE);

	if ((rv = prog_updi_readwrite(ps, tx, 4, vp, len)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u\n", rv);

	return rv;
}

#if 0	/* Currently unused */
static uint8_t
prog_updi_load_words_ptr_inc(struct updi_state *ps, uint8_t *vp, uint16_t len)
{
	uint8_t tx[2], rv;

	/*
	 * XXX: This assumes host is little-endian! (which it is, but ...)
	 */

	DBFPRINTF("len %u\n", len);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_LD(UPDI_PTR_INDIRECT_PI, UPDI_SZB_2BYTES);

	if ((rv = prog_updi_readwrite(ps, tx, 4, vp, len * 2)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u\n", rv);

	return rv;
}
#endif

static uint8_t
prog_updi_store_bytes_ptr_inc(struct updi_state *ps, const uint8_t *vp,
    uint16_t len)
{
	uint8_t tx[3], rx, rv, off;

	DBFPRINTF("len %u\n", len);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_ST(UPDI_PTR_INDIRECT_PI, UPDI_SZB_1BYTE);
	off = 2;

	while (len--) {
		tx[off] = *vp++;

		if ((rv = prog_updi_readwrite(ps, tx, off + 1, &rx, 1)) != 0) {
			DBFPRINTF("readwrite failed\n");
			return rv;
		}

		if (rx != UPDI_ACK) {
			DBFPRINTF("No ACK (%02x)\n", rx);
			prog_updi_error_recovery(ps);
			return 1;
		}

		off = 0;
	}

	return 0;
}

static uint8_t
prog_updi_store_words_ptr_inc(struct updi_state *ps, const uint8_t *vp,
    uint16_t len)
{
	uint8_t tx[4], rx, rv, off;

	DBFPRINTF("len %u\n", len);

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_ST(UPDI_PTR_INDIRECT_PI, UPDI_SZB_1BYTE);
	off = 2;

	while (len--) {
		tx[off] = *vp++;
		tx[off+1] = *vp++;

		if ((rv = prog_updi_readwrite(ps, tx, off + 2, &rx, 1)) != 0) {
			DBFPRINTF("readwrite failed\n");
			return rv;
		}

		if (rx != UPDI_ACK) {
			DBFPRINTF("No ACK (%02x)\n", rx);
			prog_updi_error_recovery(ps);
			return 1;
		}

		off = 0;
	}

	return 0;
}

static uint8_t
prog_updi_set_repeat(struct updi_state *ps, uint16_t count)
{
	uint8_t tx[4], rv;

	DBFPRINTF("count %u\n", count);

	count -= 1;
	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_REPEAT(UPDI_SZB_2BYTES);
	tx[2] = (uint8_t)(count & 0xffu);
	tx[3] = (uint8_t)(count >> 8);

	rv = prog_updi_readwrite(ps, tx, 4, NULL, 0);

	DBFPRINTF("result %u\n", rv);

	return rv;
}

static uint8_t
prog_updi_load_sib(struct updi_state *ps, struct updi_sib *sib)
{
	uint8_t tx[2], rv;

	DBFPRINTF("\n");

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_KEY(UPDI_SIB_SEND_SIB, UPDI_SZC_16BYTES);

	rv = prog_updi_readwrite(ps, tx, 2, (void *)sib, sizeof(*sib));

	DBFPRINTF("result %u\n", rv);

	return rv;
}

static uint8_t
prog_updi_store_key(struct updi_state *ps, const uint8_t *key,
    uint16_t key_len)
{
	uint8_t tx[18], rv, sz;

	DBFPRINTF("len %u\n", key_len);

	if (key_len == 8)
		sz = UPDI_SZC_8BYTES;
	else
	if (key_len == 16)
		sz = UPDI_SZC_16BYTES;
	else
		return 1;

	tx[0] = UPDI_SYNCH;
	tx[1] = UPDI_CMD_KEY(UPDI_SIB_RECEIVE_KEY, sz);
	memcpy(&tx[2], key, key_len);

	rv = prog_updi_readwrite(ps, tx, key_len + 2, NULL, 0);

	DBFPRINTF("result %u\n", rv);

	return rv;
}

static uint8_t
prog_updi_hw_enable(struct updi_state *ps)
{
	uint8_t rv, st, clksel;
	uint32_t baud;

	if ((rv = avr_ll_updi_enable(PROG_PDI_INIT_BAUD, 0)) != 0) {
		DBFPRINTF("avr_ll_updi_enable(0) failed (%u)\n", rv);

		/*
		 * If the UPDI pin remains high after the enable sequence then
		 * it's possible the target is already in UPDI mode. Otherwise
		 * there's not much we can do.
		 */
		if (rv != AVR_LL_UPDI_RESET_STILL_HIGH)
			return 1;

		DBFPRINTF("trying again...\n");
		if ((rv = avr_ll_updi_enable(PROG_PDI_INIT_BAUD, 1)) != 0) {
			DBFPRINTF("avr_ll_updi_enable(1) failed (%u)\n", rv);
			return rv;
		}
	}

	/*
	 * Note: From this point we have about 13 mS to send the
	 * first command otherwise the target will exit UPDI mode.
	 *
	 * The very first operation will be a STCS to UPDI.CTRLB
	 * to disable collision detection. Otherwise the target
	 * will not drive the bus hard, resulting in "problems".
	 */
	if (prog_updi_store_updi_reg(ps, UPDI_CSA_CTRLB,
	    UPDI_CTRLB_CCDETDIS) != 0) {
		/*
		 * Try again after sending a double-break
		 */
		avr_ll_pdi_send_break(2, 1);

		if (prog_updi_store_updi_reg(ps, UPDI_CSA_CTRLB,
		    UPDI_CTRLB_CCDETDIS) != 0) {
			DBFPRINTF("failed to disable collision detection\n");
			avr_ll_pdi_disable();
			return 1;
		}
	}

	prog_updi_timer_start(ps, 200);
	rv = prog_updi_load_updi_reg(ps, UPDI_CSA_STATUSA, &st);
	prog_updi_timer_stop(ps);
	if (rv != 0 || UPDI_STATUSA_UPDIREV(st) == 0) {
		/* It's dead, Jim */
		avr_ll_pdi_disable();
		DBFPRINTF("failed to read STATUSA (rv %u, STATUSA %02x)\n", rv,
		    st);
		return 1;
	}

	/*
	 * So far so good. Adjust the inter-byte delay and guard time to
	 * improve performance.
	 */
	if (prog_updi_store_updi_reg(ps, UPDI_CSA_CTRLA, UPDI_CTRLA_IBDLY |
	    UPDI_CTRLA_GTVAL_CYCLES_32) != 0) {
		avr_ll_pdi_disable();
		DBFPRINTF("failed to set CTRLA\n");
		return 1;
	}

	/*
	 * Fetch ASI_CTRLA so we can check UPDICLKSEL
	 */
	prog_updi_timer_start(ps, 100);
	rv = prog_updi_load_updi_reg(ps, UPDI_CSA_ASI_CTRLA, &st);
	prog_updi_timer_stop(ps);
	if (rv != 0) {
		avr_ll_pdi_disable();
		DBFPRINTF("failed to read ASI_CTRLA\n");
		return 1;
	}

#ifndef CONFIG_USE_CONFIGDB
	baud = glob.glob_updi_baud;
#else
	if (cf_get_uint32(CF_KEY_UINT32_UPDI_BAUD, &baud) == 0)
		baud = 0;
#endif

	if (baud == 0)
		baud = 225000;	/* A safe default */

	if (baud > 450000)
		clksel = UPDI_ASI_CTRLA_CLK_16MHZ;
	else
	if (baud > 225000)
		clksel = UPDI_ASI_CTRLA_CLK_8MHZ;
	else
		clksel = UPDI_ASI_CTRLA_CLK_4MHZ;

	if (clksel != UPDI_ASI_CTRLA_UPDICLKSEL(st)) {
		/*
		 * Configure the required UPDICLKSEL value, and hope
		 * the user knows about the BOD limitations...
		 */
		DBFPRINTF("CLKSEL -> %u\n", clksel);
		st = (st & ~UPDI_ASI_CTRLA_CLK_MASK) | clksel;
		if (prog_updi_store_updi_reg(ps, UPDI_CSA_ASI_CTRLA, st) != 0) {
			avr_ll_pdi_disable();
			DBFPRINTF("failed to set CLKSEL\n");
			return 1;
		}
	}

	programmer_curr_clock_ticks = 300;	/* 30 seconds */
	programmer_curr_clock_rate = baud;
	programmer_curr_clock_name = "UPDIbaud";

	/*
	 * Set the global timeout based on the uPDI baud rate
	 */
	ps->ps_timeout = 100 +
	    (uint16_t)((1.0F / ((float)(baud + 59) / 60.0F)) * 5000.0F);

	DBFPRINTF("setting baud rate to %lu, timeout %u\n", baud,
	    ps->ps_timeout);
	rv = avr_ll_updi_change_baud(baud);

	/* Zero on success, else non-zero */
	return rv;
}

static void
prog_updi_hw_disable(struct updi_state *ps)
{

	(void) ps;

	DBFPRINTF("disable UPDI\n");
	avr_ll_pdi_disable();
}

static int
prog_updi_wait_unlocked(struct updi_state *ps, int just_check)
{
	uint8_t x, st;
	int rv = 1;

	DBFPRINTF("\n");

	prog_updi_timer_start(ps, 1000);

	while (ps->ps_timer_expired == false) {
		x = prog_updi_load_updi_reg(ps, UPDI_CSA_ASI_SYS_STATUS, &st);
		if (x != 0) {
			rv = -1;
			break;
		} else
		if ((st & UPDI_ASI_SYS_STATUS_LOCKSTATUS) == 0) {
			rv = 0;
			break;
		}

		if (just_check)
			break;

		/*
		 * Atmel-ICE waits 2mS between polls, so we will too.
		 */
		rtos_task_sleep(2);
	}

	prog_updi_timer_stop(ps);

	DBFPRINTF("rv = %d\n", rv);

	return rv;
}

static void
prog_updi_target_reset_toggle(struct updi_state *ps)
{

	(void) prog_updi_store_updi_reg(ps, UPDI_CSA_ASI_RESET_REQ,
	    UPDI_ASI_RESET_REQ_RSTREQ);

	/* Atmel-ICE checks ASI_SYS_STATUS.RSTSYS here. We don't bother */

	(void) prog_updi_store_updi_reg(ps, UPDI_CSA_ASI_RESET_REQ, 0);
}

static int
prog_updi_unlock(struct updi_state *ps)
{
	uint8_t x, st;
	int rv = 0;

	DBFPRINTF("\n");

	/*
	 * Send the chip-erase key
	 */
	if (prog_updi_store_key(ps, updi_key_chiperase,
	    sizeof(updi_key_chiperase)) != 0) {
		DBFPRINTF("store key failed\n");
		return -1;
	}

	/*
	 * Was the key was accepted?
	 */
	prog_updi_timer_start(ps, ps->ps_timeout);
	x = prog_updi_load_updi_reg(ps, UPDI_CSA_ASI_KEY_STATUS, &st);
	prog_updi_timer_stop(ps);

	if (x != 0) {
		DBFPRINTF("Key status failed\n");
		return -1;
	}

	if ((st & UPDI_ASI_KEY_STATUS_CHIPERASE) == 0) {
		DBFPRINTF("Key ignored\n");
		return -1;
	}

	prog_updi_target_reset_toggle(ps);

	if ((rv = prog_updi_wait_unlocked(ps, 0)) != 0)
		return rv;

	return 0;
}

static int
prog_updi_check_prog_mode(struct updi_state *ps)
{
	uint8_t st, i;

	/* Check if already in programming mode */
	prog_updi_timer_start(ps, ps->ps_timeout);
	i = prog_updi_load_updi_reg(ps, UPDI_CSA_ASI_SYS_STATUS, &st);
	prog_updi_timer_stop(ps);
	if (i != 0)
		return -1;

	return (st & UPDI_ASI_SYS_STATUS_NVMPROG) != 0;
}

int
prog_updi_get_sync(void *arg)
{
	struct updi_state *ps = arg;

	DBFPRINTF("\n");

	if (ps->ps_sync_active) {
		DBFPRINTF("already active\n");
		return PROG_UPDI_STATUS_OK;
	}

	if (prog_updi_hw_enable(ps) != 0) {
		DBFPRINTF("prog_updi_hw_enable() failed\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	ps->ps_sync_active = true;

	return PROG_UPDI_STATUS_OK;
}

int
prog_updi_enter_prog_mode(void *arg)
{
	struct updi_state *ps = arg;
	uint8_t st, i;
	int rv;

	DBFPRINTF("enable program mode\n");

	if (ps->ps_sync_active == false) {
		DBFPRINTF("target not connected\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	/* Check if already in programming mode */
	if ((rv = prog_updi_check_prog_mode(ps)) < 0) {
		DBFPRINTF("SYS_STATUS read failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}
	if (rv != 0) {
		DBFPRINTF("already enabled\n");
		ps->ps_nvm_active = true;
		return PROG_UPDI_STATUS_OK;
	}

	/* Check for unlocked */
	DBFPRINTF("checking unlock\n");
	if ((rv = prog_updi_wait_unlocked(ps, 1)) < 0) {
		DBFPRINTF("check unlock failed\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	/* If locked, then we need to erase */
	if (rv != 0) {
		DBFPRINTF("erase to unlock\n");
		if ((rv = prog_updi_unlock(ps)) != 0) {
			DBFPRINTF("unlock failed: %d\n", rv);
			return PROG_UPDI_STATUS_FAILURE;
		}
	}

	/*
	 * Send the NVM key
	 */
	DBFPRINTF("sending NVM key\n");
	if (prog_updi_store_key(ps, updi_key_nvmprog,
	    sizeof(updi_key_nvmprog)) != 0) {
		DBFPRINTF("store key failed\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	/*
	 * Was the key was accepted?
	 */
	DBFPRINTF("checking NVM key status\n");
	prog_updi_timer_start(ps, ps->ps_timeout);
	i = prog_updi_load_updi_reg(ps, UPDI_CSA_ASI_KEY_STATUS, &st);
	prog_updi_timer_stop(ps);

	if (i != 0) {
		DBFPRINTF("Key status failed\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	if ((st & UPDI_ASI_KEY_STATUS_NVMPROG) == 0) {
		DBFPRINTF("Key ignored\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	DBFPRINTF("toggling reset\n");
	prog_updi_target_reset_toggle(ps);

	/* Wait for unlocked */
	DBFPRINTF("checking unlock\n");
	if ((rv = prog_updi_wait_unlocked(ps, 0)) < 0) {
		DBFPRINTF("wait unlock failed\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	/* Check again if we're in programming mode */
	DBFPRINTF("check programming mode\n");
	if ((rv = prog_updi_check_prog_mode(ps)) < 0) {
		DBFPRINTF("SYS_STATUS read2 failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}
	if (rv == 0) {
		DBFPRINTF("failed to enable\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	ps->ps_nvm_active = true;

	return PROG_UPDI_STATUS_OK;
}

int
prog_updi_leave_prog_mode(void *arg)
{
	struct updi_state *ps = arg;
	int rv;

	/* Check if we're in programming mode */
	if ((rv = prog_updi_check_prog_mode(ps)) < 0) {
		DBFPRINTF("SYS_STATUS read failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}
	if (rv == 0) {
		DBFPRINTF("already disabled\n");
		ps->ps_nvm_active = false;
		return PROG_UPDI_STATUS_OK;
	}

	prog_updi_target_reset_toggle(ps);

	if (prog_updi_store_updi_reg(ps, UPDI_CSA_CTRLB, UPDI_CTRLB_UPDIDIS)) {
		DBFPRINTF("failed to disable\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	ps->ps_nvm_active = false;

	return PROG_UPDI_STATUS_OK;
}

static uint8_t
prog_updi_nvm_reg_write(struct updi_state *ps, uint8_t reg, uint8_t v)
{

	return prog_updi_store_byte(ps, ps->ps_params.upa_nvm_addr + reg, v);
}

static uint8_t
prog_updi_nvm_reg_write16(struct updi_state *ps, uint8_t reg, uint16_t v)
{
	uint8_t rv;

	rv = prog_updi_store_byte(ps, ps->ps_params.upa_nvm_addr + reg,
	    (uint8_t)(v & 0xffu));
	if (rv != 0)
		return rv;

	return prog_updi_store_byte(ps, ps->ps_params.upa_nvm_addr + reg + 1,
	    (uint8_t)(v >> 8));
}


static uint8_t
prog_updi_nvm_reg_read(struct updi_state *ps, uint8_t reg, uint8_t *vp)
{

	return prog_updi_load_byte(ps, ps->ps_params.upa_nvm_addr + reg, vp);
}

static int
prog_updi_wait_nvm_ready(struct updi_state *ps)
{
	uint8_t i, st;
	int rv = PROG_UPDI_STATUS_OK;

	prog_updi_timer_start(ps, 5000);

	do {
		i = prog_updi_nvm_reg_read(ps, UPDI_NVM_REG_STATUS, &st);
		if (i != 0) {
			DBFPRINTF("NVM status timeout\n");
			rv = PROG_UPDI_STATUS_TIMEDOUT;
			break;
		}

		if (st & UPDI_NVM_STATUS_WRERROR) {
			DBFPRINTF("NVM error\n");
			rv = PROG_UPDI_STATUS_FAILURE;
		}
	} while (st & (UPDI_NVM_STATUS_FBUSY | UPDI_NVM_STATUS_EEBUSY));

	prog_updi_timer_stop(ps);

	return rv;
}

static int
prog_updi_erase_cmd(struct updi_state *ps, uint8_t cmd)
{
	uint8_t i;
	int rv;

	DBFPRINTF("cmd %02x\n", cmd);

	rv = prog_updi_wait_nvm_ready(ps);
	if (rv != PROG_UPDI_STATUS_OK) {
		DBFPRINTF("NVM not ready\n");
		return rv;
	}

	DBFPRINTF("NVM is ready\n");

	prog_updi_timer_start(ps, ps->ps_timeout * 4);
	i = prog_updi_nvm_reg_write(ps, UPDI_NVM_REG_CTRLA,
	    UPDI_NVM_CTRLA_CMD(cmd));
	prog_updi_timer_stop(ps);
	if (i != 0) {
		DBFPRINTF("cmd %d start timeout\n", cmd);
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	DBFPRINTF("command in progress\n");

	rv = prog_updi_wait_nvm_ready(ps);
	if (rv != PROG_UPDI_STATUS_OK) {
		DBFPRINTF("cmd %d wait timed out\n", cmd);
	} else {
		DBFPRINTF("command complete\n");
	}

	return rv;
}

int
prog_updi_erase(void *arg, const struct prog_updi_erase_arg *uea)
{
	struct updi_state *ps = arg;
	uint8_t cmd;

	if (ps->ps_nvm_active == false) {
		DBFPRINTF("not in programming mode\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	switch (uea->uea_mem_type) {
	case PROG_UPDI_ERASE_DEVICE:
		DBFPRINTF("erase device\n");
		cmd = UPDI_NVM_CMD_CHIP_ERASE;
		break;
	case PROG_UPDI_ERASE_EEPROM:
		DBFPRINTF("erase eeprom\n");
		cmd = UPDI_NVM_CMD_EEPROM_ERASE;
		break;
	default:
		DBFPRINTF("unknown memtype %d\n", uea->uea_mem_type);
		return PROG_UPDI_STATUS_FAILURE;
	}

	return prog_updi_erase_cmd(ps, cmd);
}

static int
prog_updi_write_pb16(struct updi_state *ps, const uint8_t *src,
    uint16_t addr, uint16_t len)
{
	uint8_t i;

	prog_updi_timer_start(ps, ps->ps_timeout * 16);

	if (len == 2) {
		uint16_t w;
		w = (uint16_t)src[0];
		w |= ((uint16_t)src[1]) << 8;
		i = prog_updi_store_word(ps, addr, w);
		if (i != 0) {
			prog_updi_timer_stop(ps);
			DBFPRINTF("store word failed\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}
		return PROG_UPDI_STATUS_OK;
	}

	i = prog_updi_set_pointer(ps, addr);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set ptr failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	i = prog_updi_set_repeat(ps, len / 2);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set repeat failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	prog_updi_timer_stop(ps);

	prog_updi_timer_start(ps, ps->ps_timeout * (4 * len));
	i = prog_updi_store_words_ptr_inc(ps, src, len / 2);
	prog_updi_timer_stop(ps);

	if (i != 0) {
		DBFPRINTF("store failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	return PROG_UPDI_STATUS_OK;
}

static int
prog_updi_write_pb8(struct updi_state *ps, const uint8_t *src,
    uint16_t addr, uint16_t len)
{
	uint8_t i;

	prog_updi_timer_start(ps, ps->ps_timeout * 12);

	if (len == 1) {
		i = prog_updi_store_byte(ps, addr, *src);
		if (i != 0) {
			prog_updi_timer_stop(ps);
			DBFPRINTF("store byte failed\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}
		return PROG_UPDI_STATUS_OK;
	}

	i = prog_updi_set_pointer(ps, addr);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set ptr failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	i = prog_updi_set_repeat(ps, len);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set repeat failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	prog_updi_timer_stop(ps);

	prog_updi_timer_start(ps, ps->ps_timeout * (3 * len));
	i = prog_updi_store_bytes_ptr_inc(ps, src, len);
	prog_updi_timer_stop(ps);

	if (i != 0) {
		DBFPRINTF("store failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	return PROG_UPDI_STATUS_OK;
}

static int
prog_updi_program_cmd(struct updi_state *ps,
    const struct prog_updi_program_arg *upa,
    uint8_t cmd)
{
	const uint8_t *payload;
	uint16_t addr;
	uint16_t len;
	uint8_t i;
	int rv;

	payload = upa->upa_data;
	addr = upa->upa_address;
	len = upa->upa_length;

	rv = prog_updi_wait_nvm_ready(ps);
	if (rv != PROG_UPDI_STATUS_OK) {
		DBFPRINTF("NVM busy1\n");
		return rv;
	}

	if (cmd != UPDI_NVM_CMD_WRITE_FUSE) {
		/* Clear page buffer */
		prog_updi_timer_start(ps, ps->ps_timeout * 4);
		i = prog_updi_nvm_reg_write(ps, UPDI_NVM_REG_CTRLA,
		    UPDI_NVM_CTRLA_CMD(UPDI_NVM_CMD_PAGE_BUFFER_CLEAR));
		prog_updi_timer_stop(ps);
		if (i != 0) {
			DBFPRINTF("clear PB timed out\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}

		rv = prog_updi_wait_nvm_ready(ps);
		if (rv != PROG_UPDI_STATUS_OK) {
			DBFPRINTF("NVM busy2\n");
			return rv;
		}

		/* Write page buffer */
		if (upa->upa_mode & PROG_UPDI_MODE_WORDS)
			rv = prog_updi_write_pb16(ps, payload, addr, len);
		else
			rv = prog_updi_write_pb8(ps, payload, addr, len);
		if (rv != PROG_UPDI_STATUS_OK) {
			DBFPRINTF("write PB failed\n");
			return rv;
		}
	} else {
		uint16_t x;

		x = (uint16_t) upa->upa_address;
		prog_updi_timer_start(ps, ps->ps_timeout * 8);
		i = prog_updi_nvm_reg_write16(ps, UPDI_NVM_REG_ADDRL, x);
		prog_updi_timer_stop(ps);
		if (i != 0) {
			DBFPRINTF("FUSE addr timeout\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}

		prog_updi_timer_start(ps, ps->ps_timeout * 8);
		i = prog_updi_nvm_reg_write(ps, UPDI_NVM_REG_DATAL, *payload);
		prog_updi_timer_stop(ps);
		if (i != 0) {
			DBFPRINTF("FUSE data timeout\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}
	}

	/* Execute the command */
	prog_updi_timer_start(ps, ps->ps_timeout * 4);
	i = prog_updi_nvm_reg_write(ps, UPDI_NVM_REG_CTRLA,
	    UPDI_NVM_CTRLA_CMD(cmd));
	prog_updi_timer_stop(ps);
	if (i != 0) {
		DBFPRINTF("command timed out\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	rv = prog_updi_wait_nvm_ready(ps);
	if (rv != PROG_UPDI_STATUS_OK) {
		DBFPRINTF("NVM busy3\n");
	}

	return rv;
}

int
prog_updi_program(void *arg, const struct prog_updi_program_arg *upa)
{
	struct updi_state *ps = arg;
	uint8_t cmd;
	const char *mt;

	if (ps->ps_nvm_active == false) {
		DBFPRINTF("not in programming mode\n");
		return PROG_UPDI_STATUS_FAILURE;
	}

	switch (upa->upa_mem_type) {
	case PROG_UPDI_MEM_TYPE_FLASH:
		mt = "FLASH";
		if (upa->upa_mode & PROG_UPDI_MODE_ERASE_FIRST)
			cmd = UPDI_NVM_CMD_ERASE_WRITE_PAGE;
		else
			cmd = UPDI_NVM_CMD_WRITE_PAGE;
		break;

	case PROG_UPDI_MEM_TYPE_EEPROM:
		mt = "EEPROM";
		cmd = UPDI_NVM_CMD_WRITE_PAGE;
		break;

	case PROG_UPDI_MEM_TYPE_FUSE:
	case PROG_UPDI_MEM_TYPE_LOCKBITS:
		mt = "FUSE/LOCK";
		cmd = UPDI_NVM_CMD_WRITE_FUSE;
		break;

	default:
		DBFPRINTF("bad mem type %d\n", upa->upa_mem_type);
		return PROG_UPDI_STATUS_FAILURE;
	}

	DBFPRINTF("mem_type %02x (%s), mode %02x, addr %04x, length %u\n",
	    upa->upa_mem_type, mt, upa->upa_mode, upa->upa_address,
	    upa->upa_length);
	(void) mt;

	return prog_updi_program_cmd(ps, upa, cmd);
}

#if 0
static int
prog_updi_read16(struct updi_state *ps, uint8_t *dest,
    uint16_t addr, uint16_t len)
{
	uint8_t i;

	prog_updi_timer_start(ps, 300);

	if (len == 2) {
		uint16_t w;
		i = prog_updi_load_word(ps, addr, &w);
		if (i != 0) {
			prog_updi_timer_stop(ps);
			DBFPRINTF("load word failed\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}
		dest[0] = (uint8_t)w;
		dest[1] = (uint8_t)(w >> 8);
		return PROG_UPDI_STATUS_OK;
	}

	i = prog_updi_set_pointer(ps, addr);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set ptr failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	i = prog_updi_set_repeat(ps, len / 2);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set repeat failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	prog_updi_timer_stop(ps);

	prog_updi_timer_start(ps, 1000);
	i = prog_updi_load_words_ptr_inc(ps, dest, len / 2);
	prog_updi_timer_stop(ps);

	if (i != 0) {
		DBFPRINTF("load failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	return PROG_UPDI_STATUS_OK;
}
#endif

static int
prog_updi_read8(struct updi_state *ps, uint8_t *dest,
    uint16_t addr, uint16_t len)
{
	uint8_t i;

	prog_updi_timer_start(ps, ps->ps_timeout * 12);

	if (len == 1) {
		i = prog_updi_load_byte(ps, addr, dest);
		if (i != 0) {
			prog_updi_timer_stop(ps);
			DBFPRINTF("load byte failed\n");
			return PROG_UPDI_STATUS_TIMEDOUT;
		}
		return PROG_UPDI_STATUS_OK;
	}

	i = prog_updi_set_pointer(ps, addr);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set ptr failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	i = prog_updi_set_repeat(ps, len);
	if (i != 0) {
		prog_updi_timer_stop(ps);
		DBFPRINTF("set repeat failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	prog_updi_timer_stop(ps);

	prog_updi_timer_start(ps, ps->ps_timeout * (3 * len));
	i = prog_updi_load_bytes_ptr_inc(ps, dest, len);
	prog_updi_timer_stop(ps);

	if (i != 0) {
		DBFPRINTF("load failed\n");
		return PROG_UPDI_STATUS_TIMEDOUT;
	}

	return PROG_UPDI_STATUS_OK;
}

int
prog_updi_read(void *arg, const struct prog_updi_read_arg *ura)
{
	struct updi_state *ps = arg;
	int rv;

	DBFPRINTF("mem_type %d, address 0x%04x, length %u\n", ura->ura_mem_type,
	    ura->ura_address, ura->ura_length);

	switch (ura->ura_mem_type) {
	case PROG_UPDI_MEM_TYPE_FLASH:
	case PROG_UPDI_MEM_TYPE_EEPROM:
	case PROG_UPDI_MEM_TYPE_SIGROW:
	case PROG_UPDI_MEM_TYPE_FUSE:
	case PROG_UPDI_MEM_TYPE_LOCKBITS:
		if (ps->ps_nvm_active == false) {
			DBFPRINTF("not in programming mode\n");
			return PROG_UPDI_STATUS_FAILURE;
		}

		rv = prog_updi_read8(ps, ura->ura_buffer, ura->ura_address,
		    ura->ura_length);
		if (rv != PROG_UPDI_STATUS_OK) {
			DBFPRINTF("read 8 failed\n");
		}
		break;

	case PROG_UPDI_MEM_TYPE_SIB:
		if (ps->ps_sync_active == false) {
			DBFPRINTF("target not connected\n");
			return PROG_UPDI_STATUS_FAILURE;
		}

		rv = prog_updi_load_sib(ps, (struct updi_sib *)ura->ura_buffer);
		if (rv != PROG_UPDI_STATUS_OK) {
			DBFPRINTF("read SIB failed\n");
		}
		break;

	default:
		DBFPRINTF("unsupported mem_type: %d\n", ura->ura_mem_type);
		rv = PROG_UPDI_STATUS_FAILURE;
		break;
	}

	return rv;
}

void
prog_updi_reset_comms(void *arg)
{

	(void) arg;

	avr_ll_pdi_send_break(2, 1);
}

int
prog_updi_set_param(void *arg, const struct prog_updi_param_arg *upa)
{
	struct updi_state *ps = arg;

	ps->ps_params = *upa;

	return PROG_UPDI_STATUS_OK;
}

static void
prog_updi_timer_callback(rtos_timer_t handle)
{
	struct updi_state *ps;

	ps = rtos_timer_get_id(handle);

        ps->ps_timer_expired = true;
}

static const char updi_usage[] = {
	"usage:\n"
	"updi\n"
	"\tDisplays current uPDI baud rate\n"
	"updi baud <rate>\n"
	"\tSets the uPDI baud rate to <rate>. This is the communication\n"
	"\trate used when programming uPDI devices if it is not specified by\n"
	"\tthe programmer. The default is 0, which means use a safe minimum\n"
	"\tof 225Kbaud. Otherwise the target's UPDICLKSEL field will be set\n"
	"\tto a value appropriate for the specified rate. Note that this\n"
	"\tmight fail to work if Brown Out Detect is set to anything other\n"
	"\tthan its highest level. See the target's datasheet for details.\n"
	"\tThe lowest acceptable value for the uPDI baud rate is 1200. The\n"
	"\thighest is 900000.\n"
};

static void
updi_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	uint32_t updi_baud, new_baud;

#ifndef CONFIG_USE_CONFIGDB
	updi_baud = glob.glob_updi_baud;
#else
	if (cf_get_uint32(CF_KEY_UINT32_UPDI_BAUD, &updi_baud) == 0)
		updi_baud = 0;
#endif

	if (argc == 2 && strcasecmp(argv[0], "baud") == 0) {
		char *p;

		if (strcasecmp(argv[1], "auto") == 0)
			new_baud = 0;
		else {
			new_baud = strtoul(argv[1], &p, 10);

			if (p == NULL || *p != '\0' || (new_baud != 0 &&
			    (new_baud < 1200 || new_baud > 900000))) {
				fprintf(os, "Invalid baud rate. Baud must be "
				    "between 1200 and 900000\n");
				new_baud = updi_baud;
			}
		}

		if (new_baud != updi_baud) {
#ifndef CONFIG_USE_CONFIGDB
			glob.glob_updi_baud = new_baud;
			glob_save();
#else
			cf_set_uint32(CF_KEY_UINT32_UPDI_BAUD, new_baud);
			configdb_persist();
#endif
			updi_baud = new_baud;
		}
	} else
#ifdef DEBUG_ENABLED
	if (argc >= 1 && strcasecmp(argv[0], "debug") == 0) {
		if (argc >= 2) {
			if (strcasecmp(argv[1], "on") == 0)
				DEBUG_FLAG = 1;
			else
			if (strcasecmp(argv[1], "off") == 0)
				DEBUG_FLAG = 0;
		}
	} else
#endif /* DEBUG_ENABLED */
	if (argc != 0) {
		fprintf(os, "%s", updi_usage);
		return;
	}

	fprintf(os, "UPDI baud rate: ");
	if (updi_baud != 0)
		fprintf(os, "%lu\n", updi_baud);
	else
		fprintf(os, "AUTO\n");

#ifdef DEBUG_ENABLED
	fprintf(os, "UPDI debug state: %s\n", (DEBUG_FLAG ? "on" : "off"));
#endif
}

void *
prog_updi_init(void *cookie)
{
	char pname[16];

	(void) cookie;

	updi_state.ps_params.upa_flash_base = 0;
	updi_state.ps_params.upa_nvm_addr = 0;
	updi_state.ps_params.upa_ocd_addr = 0;
	updi_state.ps_params.upa_flash_page_size = 0;
	updi_state.ps_params.upa_ee_page_size = 0;

	updi_state.ps_rx_yield_len = 8;
	updi_state.ps_timeout = 100;

	strcpy(pname, "UPDITimer");
	updi_state.ps_timer = rtos_timer_create(pname, 100, 0,
	    prog_updi_timer_callback, &updi_state);
	assert(updi_state.ps_timer != NULL);

	SHELL_CMD_ADD(updi);

	return &updi_state;
}

void
prog_updi_quiesce(void *cookie)
{
	struct updi_state *ps = cookie;

	if (ps->ps_nvm_active) {
		DBFPRINTF("disable UPDI\n");
		(void) prog_updi_leave_prog_mode(ps);
		ps->ps_nvm_active = false;
	}

	if (ps->ps_sync_active) {
		prog_updi_hw_disable(ps);
		ps->ps_sync_active = false;
	}
}

void
prog_updi_load_defaults(void)
{

	/* Zero means automatic baud selection */
#ifndef CONFIG_USE_CONFIGDB
	glob.glob_updi_baud = 0;
#else
	cf_set_uint32(CF_KEY_UINT32_UPDI_BAUD, 0);
#endif
}
