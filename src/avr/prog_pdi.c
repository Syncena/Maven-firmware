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
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "programmer.h"

//#define	DEBUG_FLAG_INIT 0
//#define	DEBUG_FLAG	prog_pdi_debug
#include "debug.h"

/* This might be defined if I decide to host on an XMega */
#ifndef NVM_NVMBUSY_bm
#define	NVM_NVMBUSY_bm	(1u << 7)
#endif

/*
 * PDI Protocol
 */
#define PDI_CMD_LDS(as, ds)	(0x00u | ((as) << 2) | (ds))
#define PDI_CMD_LD(pa, da)	(0x20u | ((pa) << 2) | (da))
#define PDI_CMD_STS(as, ds)	(0x40u | ((as) << 2) | (ds))
#define PDI_CMD_ST(pa, ds)	(0x60u | ((pa) << 2) | (ds))
#define PDI_CMD_LDCS(pr)	(0x80u | (pr))
#define PDI_CMD_REPEAT(ds)	(0xa0u | (ds))
#define PDI_CMD_STCS(pr)	(0xc0u | (pr))
#define PDI_CMD_KEY		0xe0u

#define PDI_REG_STATUS		0
#define PDI_REG_RESET		1
#define PDI_REG_CTRL		2

#define PDI_STATUS_NVMEM	(1u << 1)

#define PDI_RESET_KEY		0x59u

#define PDI_DATASIZE_1BYTE	0
#define PDI_DATASIZE_2BYTES	1
#define PDI_DATASIZE_3BYTES	2
#define PDI_DATASIZE_4BYTES	3

#define PDI_POINTER_INDIRECT	0
#define PDI_POINTER_INDIRECT_PI	1
#define PDI_POINTER_DIRECT	2

/* I believe this is the same value used by Atmel's programmers */
#define	PDI_CLOCK_DEFAULT	1000000ul

static const uint8_t pdi_nvm_key[] = {
	0xff, 0x88, 0xd8, 0xcd, 0x45, 0xab, 0x89, 0x12
};

/*
 * XMega NVM Commands
 */
#define XMEGA_CRC_LENGTH_BYTES			3

#define XMEGA_NVM_REG_ADDR0			0x00
#define XMEGA_NVM_REG_ADDR1			0x01
#define XMEGA_NVM_REG_ADDR2			0x02
#define XMEGA_NVM_REG_DAT0			0x04
#define XMEGA_NVM_REG_DAT1			0x05
#define XMEGA_NVM_REG_DAT2			0x06
#define XMEGA_NVM_REG_CMD			0x0a
#define XMEGA_NVM_REG_CTRLA			0x0b
#define XMEGA_NVM_REG_CTRLB			0x0c
#define XMEGA_NVM_REG_INTCTRL			0x0d
#define XMEGA_NVM_REG_STATUS			0x0f
#define XMEGA_NVM_REG_LOCKBITS			0x10

#define XMEGA_NVM_BIT_CTRLA_CMDEX		(1u << 0)

#define XMEGA_NVM_CMD_NOOP			0x00
#define XMEGA_NVM_CMD_CHIPERASE			0x40
#define XMEGA_NVM_CMD_READNVM			0x43
#define XMEGA_NVM_CMD_LOADFLASHPAGEBUFF		0x23
#define XMEGA_NVM_CMD_ERASEFLASHPAGEBUFF	0x26
#define XMEGA_NVM_CMD_ERASEFLASHPAGE		0x2b
#define XMEGA_NVM_CMD_WRITEFLASHPAGE		0x2e
#define XMEGA_NVM_CMD_ERASEWRITEFLASH		0x2f
#define XMEGA_NVM_CMD_FLASHCRC			0x78
#define XMEGA_NVM_CMD_ERASEAPPSEC		0x20
#define XMEGA_NVM_CMD_ERASEAPPSECPAGE		0x22
#define XMEGA_NVM_CMD_WRITEAPPSECPAGE		0x24
#define XMEGA_NVM_CMD_ERASEWRITEAPPSECPAGE	0x25
#define XMEGA_NVM_CMD_APPCRC			0x38
#define XMEGA_NVM_CMD_ERASEBOOTSEC		0x68
#define XMEGA_NVM_CMD_ERASEBOOTSECPAGE		0x2a
#define XMEGA_NVM_CMD_WRITEBOOTSECPAGE		0x2c
#define XMEGA_NVM_CMD_ERASEWRITEBOOTSECPAGE	0x2d
#define XMEGA_NVM_CMD_BOOTCRC			0x39
#define XMEGA_NVM_CMD_READUSERSIG		0x03
#define XMEGA_NVM_CMD_ERASEUSERSIG		0x18
#define XMEGA_NVM_CMD_WRITEUSERSIG		0x1a
#define XMEGA_NVM_CMD_READCALIBRATION		0x02
#define XMEGA_NVM_CMD_READFUSE			0x07
#define XMEGA_NVM_CMD_WRITEFUSE			0x4c
#define XMEGA_NVM_CMD_WRITELOCK			0x08
#define XMEGA_NVM_CMD_LOADEEPROMPAGEBUFF	0x33
#define XMEGA_NVM_CMD_ERASEEEPROMPAGEBUFF	0x36
#define XMEGA_NVM_CMD_ERASEEEPROM		0x30
#define XMEGA_NVM_CMD_ERASEEEPROMPAGE		0x32
#define XMEGA_NVM_CMD_WRITEEEPROMPAGE		0x34
#define XMEGA_NVM_CMD_ERASEWRITEEEPROMPAGE	0x35
#define XMEGA_NVM_CMD_READEEPROM		0x06

struct pdi_state {
	bool ps_hw_active;
	bool ps_prog_active;
	volatile bool ps_timer_expired;
	uint8_t ps_param_nvm_sr_addr;
	uint8_t ps_param_nvm_cmd_addr;
	uint16_t ps_param_ee_page_size;
	uint16_t ps_rx_yield_len;
	uint32_t ps_param_nvm_base;
	rtos_timer_t ps_timer;
};
static struct pdi_state pdi_state;

SHELL_CMD_DECL(pdi, pdi_cmd, "Configure PDI programming");

static void
prog_pdi_hw_enable(struct pdi_state *ps)
{

	if (ps->ps_hw_active == false) {
		uint32_t pdi_clock;

#ifndef CONFIG_USE_CONFIGDB
		pdi_clock = glob.glob_pdi_clock;
#else
		if (cf_get_uint32(CF_KEY_UINT32_PDI_CLOCK, &pdi_clock) == 0)
			pdi_clock = PDI_CLOCK_DEFAULT;
#endif

		ps->ps_hw_active = true;

		programmer_curr_clock_ticks = 300;	/* 30 seconds */
		programmer_curr_clock_name = "PDIclk";
		programmer_curr_clock_rate = pdi_clock;

		avr_ll_pdi_enable(pdi_clock);
	}
}

static void
prog_pdi_hw_disable(struct pdi_state *ps)
{

	if (ps->ps_hw_active) {
		ps->ps_hw_active = false;
		avr_ll_pdi_disable();
	}
}

static uint8_t
prog_pdi_readwrite(struct pdi_state *ps,
    const uint8_t *txbuff, uint32_t txlen,
    uint8_t *rxbuff, uint32_t rxlen)
{
	uint8_t rv;

	rv = avr_ll_pdi_transfer(txbuff, txlen, rxbuff, rxlen);

	if (rv != 0) {
		DBFPRINTF("usart error\n");
		return rv;
	}

	if (rxlen == 0)
		return 0;

	rxlen = (rxlen >= ps->ps_rx_yield_len);

	do {
		rv = avr_ll_pdi_rx_status();
		if (rv == AVR_LL_PDI_RX_BUSY && rxlen)
			rtos_yield();
	} while (rv == AVR_LL_PDI_RX_BUSY && ps->ps_timer_expired == false);

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

	return rv;
}

static uint8_t
prog_pdi_read_pdi_reg(struct pdi_state *ps, uint8_t reg,
    uint8_t *vp)
{
	uint8_t cmd, rv;

	DBFPRINTF("reg %02x\n", reg);

	cmd = PDI_CMD_LDCS(reg);
	if ((rv = prog_pdi_readwrite(ps, &cmd, 1, vp, 1)) != 0) {
		DBFPRINTF("readwrite failed\n");
		return rv;
	}

	DBFPRINTF("result %u (%02x)\n", rv, *vp);

	return rv;
}

static void
prog_pdi_timer_start(struct pdi_state *ps, uint16_t ms)
{

	rtos_timer_stop(ps->ps_timer);
	rtos_timer_change_wait_period(ps->ps_timer, ms);
	ps->ps_timer_expired = false;
	rtos_timer_start(ps->ps_timer);
}

static void
prog_pdi_timer_stop(struct pdi_state *ps)
{

	rtos_timer_stop(ps->ps_timer);
	ps->ps_timer_expired = false;
}

static uint8_t
prog_pdi_wait_for_nvm_enabled(struct pdi_state *ps)
{
	uint8_t sr, rv;

	rv = PROG_PDI_STATUS_TIMEDOUT;

	do {
		if (prog_pdi_read_pdi_reg(ps, PDI_REG_STATUS, &sr) != 0)
			break;

		if ((sr & PDI_STATUS_NVMEM) != 0)
			rv = PROG_PDI_STATUS_OK;
	} while (rv == PROG_PDI_STATUS_TIMEDOUT && !ps->ps_timer_expired);

	return rv;
}

static void
prog_pdi_hw_prep_address(uint32_t addr, uint8_t *buff)
{

	buff[0] = (addr >> 0) & 0xffu;
	buff[1] = (addr >> 8) & 0xffu;
	buff[2] = (addr >> 16) & 0xffu;
	buff[3] = (addr >> 24) & 0xffu;
}

static uint8_t
prog_pdi_hw_repeat(struct pdi_state *ps, uint32_t len)
{
	uint8_t cmdlen;
	uint8_t buff[5];

	len -= 1;

	if (len < 256u) {
		buff[0] = PDI_CMD_REPEAT(PDI_DATASIZE_1BYTE);
		buff[1] = (uint8_t)len;
		cmdlen = 2;
	} else
	if (len < 65536ul) {
		buff[0] = PDI_CMD_REPEAT(PDI_DATASIZE_2BYTES);
		buff[1] = (uint8_t)(len & 0xffu);
		buff[2] = (uint8_t)((len >> 8) & 0xffu);
		cmdlen = 3;
	} else {
		buff[0] = PDI_CMD_REPEAT(PDI_DATASIZE_4BYTES);
		prog_pdi_hw_prep_address(len, &buff[1]);
		cmdlen = 5;
	}

	if (prog_pdi_readwrite(ps, buff, cmdlen, NULL, 0) != 0)
		return PROG_PDI_STATUS_FAILURE;

	return PROG_PDI_STATUS_OK;
}

static void
prog_pdi_hw_prep_nvm_reg_address(struct pdi_state *ps, uint8_t reg,
    uint8_t *buff)
{

	prog_pdi_hw_prep_address((uint32_t)reg | ps->ps_param_nvm_base, buff);
}

static uint8_t
prog_pdi_wait_for_idle_nvm_controller(struct pdi_state *ps)
{
	uint8_t buf[5];
	uint8_t sr = 0x55, rv;

	buf[0] = PDI_CMD_ST(PDI_POINTER_DIRECT, PDI_DATASIZE_4BYTES);
	prog_pdi_hw_prep_nvm_reg_address(ps, XMEGA_NVM_REG_STATUS, &buf[1]);
	if (prog_pdi_readwrite(ps, buf, 5, NULL, 0) != 0)
		return PROG_PDI_STATUS_FAILURE;

	rv = PROG_PDI_STATUS_TIMEDOUT;

	do {
		buf[0] = PDI_CMD_LD(PDI_POINTER_INDIRECT, PDI_DATASIZE_1BYTE);
		if (prog_pdi_readwrite(ps, buf, 1, &sr, 1) != 0) {
			rv = PROG_PDI_STATUS_FAILURE;
			break;
		}

		if ((sr & NVM_NVMBUSY_bm) == 0)
			rv = PROG_PDI_STATUS_OK;
	} while (rv == PROG_PDI_STATUS_TIMEDOUT && !ps->ps_timer_expired);

	return rv;
}

int
prog_pdi_enter_prog_mode(void *arg)
{
	struct pdi_state *ps = arg;
	uint8_t i, cmd[10];
	int rv;

	DBFPRINTF("enable program mode\n");

	prog_pdi_hw_enable(ps);

	DBFPRINTF("sending reset key\n");

	prog_pdi_timer_start(ps, 1000);

	cmd[0] = PDI_CMD_STCS(PDI_REG_RESET);
	cmd[1] = PDI_RESET_KEY;
	if (prog_pdi_readwrite(ps, cmd, 2, NULL, 0) != 0) {
		prog_pdi_timer_stop(ps);
		prog_pdi_hw_disable(ps);
		DBFPRINTF("RESET_KEY failed\n");
		return PROG_PDI_STATUS_FAILURE;
	}

#if defined(MAVEN_BOARD) && (MAVEN_BOARD >= 2)
	/* Configures guard time */
	cmd[0] = PDI_CMD_STCS(PDI_REG_CTRL);
	cmd[1] = 2;	/* 32 cycles */
	if (prog_pdi_readwrite(ps, cmd, 2, NULL, 0) != 0) {
		prog_pdi_hw_disable(ps);
		return PROG_PDI_STATUS_FAILURE;
	}
#endif

	DBFPRINTF("sending command key\n");
	cmd[0] = PDI_CMD_KEY;
	for (i = 0; i < sizeof(pdi_nvm_key); i++)
		cmd[i + 1] = pdi_nvm_key[i];
	if (prog_pdi_readwrite(ps, cmd, 9, NULL, 0) != 0) {
		prog_pdi_timer_stop(ps);
		prog_pdi_hw_disable(ps);
		DBFPRINTF("CMD_KEY failed\n");
		return PROG_PDI_STATUS_FAILURE;
	}

	DBFPRINTF("wait for NVM idle\n");
	rv = prog_pdi_wait_for_nvm_enabled(ps);
	DBFPRINTF("NVM enabled wait returned %u\n", rv);

	prog_pdi_timer_stop(ps);

	ps->ps_prog_active = (rv == PROG_PDI_STATUS_OK);

	if (ps->ps_prog_active == false)
		prog_pdi_hw_disable(ps);

	return rv;
}

int
prog_pdi_leave_prog_mode(void *arg)
{
	struct pdi_state *ps = arg;
	uint8_t v, cmd[2];

	if (ps->ps_prog_active) {
		ps->ps_prog_active = false;

		prog_pdi_timer_start(ps, 1000);

		(void) prog_pdi_wait_for_idle_nvm_controller(ps);

		v = PDI_STATUS_NVMEM;
		if (prog_pdi_read_pdi_reg(ps, PDI_REG_STATUS, &v) == 0 &&
		    (v & PDI_STATUS_NVMEM) == 0) {
			prog_pdi_timer_stop(ps);
			return PROG_PDI_STATUS_OK;
		}

		cmd[0] = PDI_CMD_STCS(PDI_REG_RESET);
		cmd[1] = 0;
		(void) prog_pdi_readwrite(ps, cmd, 2, NULL, 0);

		prog_pdi_timer_stop(ps);
		rtos_task_sleep(100);
	}

	return PROG_PDI_STATUS_OK;
}

static uint8_t
prog_pdi_send_cmd(struct pdi_state *ps, uint8_t cmd, uint8_t exec)
{
	uint8_t buf[6];

	buf[0] = PDI_CMD_STS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
	prog_pdi_hw_prep_nvm_reg_address(ps, XMEGA_NVM_REG_CMD, &buf[1]);
	buf[5] = cmd;
	if (prog_pdi_readwrite(ps, buf, 6, NULL, 0) != 0)
		return PROG_PDI_STATUS_FAILURE;

	if (exec) {
		buf[0] = PDI_CMD_STS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
		prog_pdi_hw_prep_nvm_reg_address(ps, XMEGA_NVM_REG_CTRLA,
		    &buf[1]);
		buf[5] = XMEGA_NVM_BIT_CTRLA_CMDEX;
		if (prog_pdi_readwrite(ps, buf, 6, NULL, 0) != 0)
			return PROG_PDI_STATUS_FAILURE;
	}

	return PROG_PDI_STATUS_OK;
}

static int
prog_pdi_send_erase(struct pdi_state *ps, uint8_t cmd, uint32_t addr)
{
	uint8_t buf[10];
	int rv;

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	switch (cmd) {
	case XMEGA_NVM_CMD_CHIPERASE:
		rv = prog_pdi_send_cmd(ps, cmd, 1);
		break;

	case XMEGA_NVM_CMD_ERASEEEPROM:
		rv = prog_pdi_send_cmd(ps,
		    XMEGA_NVM_CMD_ERASEEEPROMPAGEBUFF, 1);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		rv = prog_pdi_wait_for_idle_nvm_controller(ps);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		rv = prog_pdi_send_cmd(ps, XMEGA_NVM_CMD_ERASEEEPROM, 1);
		break;

	default:
		rv = prog_pdi_wait_for_idle_nvm_controller(ps);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		rv = prog_pdi_send_cmd(ps, cmd, 0);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		/* Dummy write to start the erase and write command */
		buf[0] = PDI_CMD_STS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
		prog_pdi_hw_prep_address(addr, &buf[1]);
		buf[5] = 0x00;
		if (prog_pdi_readwrite(ps, buf, 6, NULL, 0) != 0)
			return PROG_PDI_STATUS_FAILURE;
		break;
	}

	rv = prog_pdi_wait_for_nvm_enabled(ps);
	if (rv != PROG_PDI_STATUS_OK) {
		debug_print("prog_pdi_send_erase: bus remained busy\n");
		return rv;
	}

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK)
		debug_print("prog_pdi_send_erase: controller remained busy\n");

	return rv;
}

int
prog_pdi_erase(void *arg, const struct prog_pdi_erase_arg *pea)
{
	struct pdi_state *ps = arg;
	uint8_t cmd;
	int rv;

	prog_pdi_timer_start(ps, 1000);

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK) {
		prog_pdi_timer_stop(ps);
		return rv;
	}

	switch (pea->pea_mem_type) {
	case PROG_PDI_ERASE_DEVICE:
		cmd = XMEGA_NVM_CMD_CHIPERASE;
		break;
	case PROG_PDI_ERASE_APPLICATION:
		cmd = XMEGA_NVM_CMD_ERASEAPPSEC;
		break;
	case PROG_PDI_ERASE_BOOTLOADER:
		cmd = XMEGA_NVM_CMD_ERASEBOOTSEC;
		break;
	case PROG_PDI_ERASE_EEPROM:
		cmd = XMEGA_NVM_CMD_ERASEEEPROM;
		break;
	case PROG_PDI_ERASE_APPLICATION_PAGE:
		cmd = XMEGA_NVM_CMD_ERASEAPPSECPAGE;
		break;
	case PROG_PDI_ERASE_BOOTLOADER_PAGE:
		cmd = XMEGA_NVM_CMD_ERASEBOOTSECPAGE;
		break;
	case PROG_PDI_ERASE_EEPROM_PAGE:
		cmd = XMEGA_NVM_CMD_ERASEEEPROMPAGE;
		break;
	case PROG_PDI_ERASE_USER_SIGNATURE:
		cmd = XMEGA_NVM_CMD_ERASEUSERSIG;
		break;
	default:
		cmd = XMEGA_NVM_CMD_NOOP;
		break;
	}

	rv = prog_pdi_send_erase(ps, cmd, pea->pea_address);

	prog_pdi_timer_stop(ps);

	return rv;
}

static int
pdi_prog_program_page(struct pdi_state *ps,
    const struct prog_pdi_program_arg *ppa,
    uint8_t cmd, uint8_t wbuf, uint8_t ebuf)
{
	const uint8_t *payload;
	uint32_t addr;
	uint16_t len;
	uint8_t buf[12];
	int rv;

	payload = ppa->ppa_data;
	addr = ppa->ppa_address;
	len = ppa->ppa_length;

	rv = PROG_PDI_STATUS_OK;

	if (ppa->ppa_page_mode & PROG_PDI_PAGEMODE_ERASE) {
		rv = prog_pdi_wait_for_idle_nvm_controller(ps);
		if (rv != PROG_PDI_STATUS_OK) {
			debug_print("pdi_prog_program_page: erase idle "
			    "timed out\n");
			return rv;
		}

		rv = prog_pdi_send_cmd(ps, ebuf, 1);
	}

	if (rv == PROG_PDI_STATUS_OK && len != 0) {
		rv = prog_pdi_wait_for_idle_nvm_controller(ps);
		if (rv != PROG_PDI_STATUS_OK) {
			debug_print("pdi_prog_program_page: program idle "
			    "timed out (rv = %d)\n", rv);
			return rv;
		}

		rv = prog_pdi_send_cmd(ps, wbuf, 0);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		buf[0] = PDI_CMD_ST(PDI_POINTER_DIRECT, PDI_DATASIZE_4BYTES);
		prog_pdi_hw_prep_address(addr, &buf[1]);
		if (prog_pdi_readwrite(ps, buf, 5, NULL, 0) != 0)
			return PROG_PDI_STATUS_FAILURE;

		rv = prog_pdi_hw_repeat(ps, len);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		buf[0] = PDI_CMD_ST(PDI_POINTER_INDIRECT_PI,
		    PDI_DATASIZE_1BYTE);
		if (prog_pdi_readwrite(ps, buf, 1, NULL, 0) != 0)
			return PROG_PDI_STATUS_FAILURE;

		if (prog_pdi_readwrite(ps, payload, len, NULL, 0) != 0) {
			debug_print("prog_pdi_program_page: failed\n");
			rv = PROG_PDI_STATUS_FAILURE;
		}
	}

	if (rv == PROG_PDI_STATUS_OK &&
	    (ppa->ppa_page_mode & PROG_PDI_PAGEMODE_WRITE)) {
		rv = prog_pdi_wait_for_idle_nvm_controller(ps);
		if (rv != PROG_PDI_STATUS_OK) {
			debug_print("pdi_prog_program_page: write idle "
			    "timed out.\n");
			return rv;
		}

		rv = prog_pdi_send_cmd(ps, cmd, 0);
		if (rv != PROG_PDI_STATUS_OK)
			return rv;

		buf[0] = PDI_CMD_STS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
		prog_pdi_hw_prep_address(addr, &buf[1]);
		buf[5] = 0;
		if (prog_pdi_readwrite(ps, buf, 6, NULL, 0) != 0)
			rv = PROG_PDI_STATUS_FAILURE;
	}

	if (rv == PROG_PDI_STATUS_OK)
		rv = prog_pdi_wait_for_idle_nvm_controller(ps);

	return rv;
}

static int
pdi_prog_program_byte(struct pdi_state *ps,
    const struct prog_pdi_program_arg *ppa, uint8_t cmd)
{
	uint8_t buf[6];
	int rv;

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	rv = prog_pdi_send_cmd(ps, cmd, 0);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	buf[0] = PDI_CMD_STS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
	prog_pdi_hw_prep_address(ppa->ppa_address, &buf[1]);
	buf[5] = *((const uint8_t *)ppa->ppa_data);
	if (prog_pdi_readwrite(ps, buf, 6, NULL, 0) != 0)
		return PROG_PDI_STATUS_FAILURE;

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);

	return rv;
}

int
prog_pdi_program(void *arg, const struct prog_pdi_program_arg *ppa)
{
	struct pdi_state *ps = arg;
	uint8_t cmd, wbuf, ebuf, pg;
	int rv;

	cmd = XMEGA_NVM_CMD_WRITEFLASHPAGE;
	wbuf = XMEGA_NVM_CMD_LOADFLASHPAGEBUFF;
	ebuf = XMEGA_NVM_CMD_ERASEFLASHPAGEBUFF;
	pg = 1;

	switch (ppa->ppa_mem_type) {
	case PROG_PDI_MEM_TYPE_APPLICATION:
		cmd = XMEGA_NVM_CMD_WRITEAPPSECPAGE;
		break;
	case PROG_PDI_MEM_TYPE_BOOTLOADER:
		cmd = XMEGA_NVM_CMD_WRITEBOOTSECPAGE;
		break;
	case PROG_PDI_MEM_TYPE_EEPROM:
		cmd = XMEGA_NVM_CMD_ERASEWRITEEEPROMPAGE;
		wbuf = XMEGA_NVM_CMD_LOADEEPROMPAGEBUFF;
		ebuf = XMEGA_NVM_CMD_ERASEEEPROMPAGEBUFF;
		break;
	case PROG_PDI_MEM_TYPE_USER_SIGNATURE:
		cmd = XMEGA_NVM_CMD_WRITEUSERSIG;
		break;
	case PROG_PDI_MEM_TYPE_FUSE:
		cmd = XMEGA_NVM_CMD_WRITEFUSE;
		pg = 0;
		break;
	case PROG_PDI_MEM_TYPE_LOCKBITS:
		cmd = XMEGA_NVM_CMD_WRITELOCK;
		pg = 0;
		break;
	default:
		return PROG_PDI_STATUS_FAILURE;
	}

	prog_pdi_timer_start(ps, 1000);

	if (pg)
		rv = pdi_prog_program_page(ps, ppa, cmd, wbuf, ebuf);
	else
		rv = pdi_prog_program_byte(ps, ppa, cmd);

	prog_pdi_timer_stop(ps);

	return rv;
}

int
prog_pdi_read(void *arg, const struct prog_pdi_read_arg *pra)
{
	struct pdi_state *ps = arg;
	uint32_t addr;
	uint16_t len;
	uint8_t cmd[8], *buff;
	int rv;

	addr = pra->pra_address;
	len = pra->pra_length;
	buff = pra->pra_buffer;

	prog_pdi_timer_start(ps, 1000);

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK) {
		prog_pdi_timer_stop(ps);
		debug_print("prog_pdi_read: controller not idle: %d\n", rv);
		return rv;
	}

	rv = prog_pdi_send_cmd(ps, XMEGA_NVM_CMD_READNVM, 0);

	if (rv == PROG_PDI_STATUS_OK && len > 1) {
		cmd[0] = PDI_CMD_ST(PDI_POINTER_DIRECT, PDI_DATASIZE_4BYTES);
		prog_pdi_hw_prep_address(addr, &cmd[1]);
		if (prog_pdi_readwrite(ps, cmd, 5, NULL, 0) != 0) {
			prog_pdi_timer_stop(ps);
			return PROG_PDI_STATUS_FAILURE;
		}

		rv = prog_pdi_hw_repeat(ps, len);
		if (rv != PROG_PDI_STATUS_OK) {
			prog_pdi_timer_stop(ps);
			return rv;
		}

		cmd[0] = PDI_CMD_LD(PDI_POINTER_INDIRECT_PI,
		    PDI_DATASIZE_1BYTE);
		if (prog_pdi_readwrite(ps, cmd, 1, buff, len) != 0)
			rv = PROG_PDI_STATUS_FAILURE;
	} else
	if (rv == PROG_PDI_STATUS_OK) {
		cmd[0] = PDI_CMD_LDS(PDI_DATASIZE_4BYTES, PDI_DATASIZE_1BYTE);
		prog_pdi_hw_prep_address(addr, &cmd[1]);
		if (prog_pdi_readwrite(ps, cmd, 5, buff, 1) != 0)
			rv = PROG_PDI_STATUS_FAILURE;
	}

	prog_pdi_timer_stop(ps);

	return rv;
}

#if 0
int
prog_pdi_crc(void *arg, const struct prog_pdi_crc_arg *pca)
{
	struct pdi_state *ps = arg;
	uint8_t buf[8], cmd, *p;
	int rv;

	p = buff;
	p += 1;

	switch (xc->xc_crc_type) {
	case PROG_PDI_CRC_APPLICATION:
		cmd = XMEGA_NVM_CMD_APPCRC;
		break;
	case PDI_CRC_BOOTLOADER:
		cmd = XMEGA_NVM_CMD_BOOTCRC;
		break;
	default:
		cmd = XMEGA_NVM_CMD_FLASHCRC;
		break;
	}

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	rv = prog_pdi_send_cmd(ps, cmd, 1);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	rv = prog_pdi_wait_for_idle_nvm_controller(ps);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	buf[0] = PDI_CMD_ST(PDI_POINTER_DIRECT, PDI_DATASIZE_4BYTES);
	prog_pdi_hw_prep_nvm_reg_address(ps, XMEGA_NVM_REG_DAT0, &buf[1]);
	if (prog_pdi_readwrite(ps, buf, 5, NULL, 0) != 0)
		return PROG_PDI_STATUS_FAILURE;

	rv = prog_pdi_hw_repeat(ps, XMEGA_CRC_LENGTH_BYTES);
	if (rv != PROG_PDI_STATUS_OK)
		return rv;

	buf[0] = PDI_CMD_LD(PDI_POINTER_INDIRECT_PI, PDI_DATASIZE_1BYTE);
	if (prog_pdi_readwrite(ps, buf, 1, NULL, 0) != 0 ||
	    prog_pdi_readwrite(ps, p, XMEGA_CRC_LENGTH_BYTES, NULL, 0) != 0) {
		rv = PROG_PDI_STATUS_FAILURE;
	} else {
		rv = PROG_PDI_STATUS_OK;
		*plen = XMEGA_CRC_LENGTH_BYTES;
	}

	return rv;
}
#endif

void
prog_pdi_reset_comms(void *arg)
{

	(void) arg;

	avr_ll_pdi_send_break(2, 0);
}

int
prog_pdi_set_param(void *arg, int which, const union prog_pdi_param_arg *ppa)
{
	struct pdi_state *ps = arg;
	int rv;

	rv = PROG_PDI_STATUS_OK;

	switch (which) {
	case PROG_PDI_PARAM_NVMBASE:
		ps->ps_param_nvm_base = ppa->ppa_nvm_base;
		break;
	case PROG_PDI_PARAM_EEPAGESIZE:
		ps->ps_param_ee_page_size = ppa->ppa_ee_page_size;
		break;
	case PROG_PDI_PARAM_NVMCMD_REG:
		ps->ps_param_nvm_cmd_addr = ppa->ppa_nvm_cmd_addr;
		break;
	case PROG_PDI_PARAM_NVMCSR_REG:
		ps->ps_param_nvm_sr_addr = ppa->ppa_nvm_sr_addr;
		break;
	default:
		rv = PROG_PDI_STATUS_FAILURE;
		break;
	}

	return rv;
}

static void
prog_pdi_timer_callback(rtos_timer_t handle)
{
	struct pdi_state *ps;

	ps = rtos_timer_get_id(handle);

        ps->ps_timer_expired = true;
}

static const char pdi_usage[] = {
	"usage:\n"
	"pdi\n"
	"\tDisplays current PDI clock frequency\n"
	"pdi clock <frequency>\n"
	"\tSets the PDI clock rate to <frequency>. This is the communication\n"
	"\trate used when programming XMega devices if it is not specified by\n"
	"\tthe programmer. The default is 1000000. The minimum supported\n"
	"\tclock is 100000 and the maximum is 10000000. You may need to\n"
	"\texperiment to determine the maximum rate at which your target\n"
	"\tdevice can be reliably programmed. For reference, it's worth\n"
	"\tnoting that Atmel-ICE's max PDI clock rate is 7.5 MHz...\n"
};

static void
pdi_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	uint32_t pdi_clock, new_clock;

#ifndef CONFIG_USE_CONFIGDB
	pdi_clock = glob.glob_pdi_clock;
#else
	if (cf_get_uint32(CF_KEY_UINT32_PDI_CLOCK, &pdi_clock) == 0)
		pdi_clock = PDI_CLOCK_DEFAULT;
#endif

	if (argc == 2 && strcasecmp(argv[0], "clock") == 0) {
		char *p;

		new_clock = strtoul(argv[1], &p, 10);
		if (p != NULL && *p == '\0' && new_clock >= 100000 &&
		    new_clock <= 10000000 && pdi_clock != new_clock) {
#ifndef CONFIG_USE_CONFIGDB
			glob.glob_pdi_clock = new_clock;
			glob_save();
#else
			cf_set_uint32(CF_KEY_UINT32_PDI_CLOCK, new_clock);
			configdb_persist();
#endif
			pdi_clock = new_clock;
		} else {
			fprintf(os, "Malformed clock rate. Clock must be "
			    "between 100000 and 10000000\n");
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
		fprintf(os, "%s", pdi_usage);
		return;
	}

	fprintf(os, "PDI Clock: %lu\n", pdi_clock);

#ifdef DEBUG_ENABLED
	fprintf(os, "PDI debug state: %s\n", (DEBUG_FLAG ? "on" : "off"));
#endif
}

void *
prog_pdi_init(void *cookie)
{
	char pname[16];

	(void) cookie;

	pdi_state.ps_param_nvm_base = 0x010001c0;
	pdi_state.ps_param_ee_page_size = 32;
	pdi_state.ps_param_nvm_cmd_addr = 0x33;
	pdi_state.ps_param_nvm_sr_addr = 0x32;
	pdi_state.ps_rx_yield_len = 8;

	strcpy(pname, "PDITimer");
	pdi_state.ps_timer = rtos_timer_create(pname, 100, 0,
	    prog_pdi_timer_callback, &pdi_state);
	assert(pdi_state.ps_timer != NULL);

	SHELL_CMD_ADD(pdi);

	return &pdi_state;
}

void
prog_pdi_quiesce(void *cookie)
{
	struct pdi_state *ps = cookie;

	DBFPRINTF("hw_active %u, prog_active %u\n", ps->ps_hw_active,
	    ps->ps_prog_active);

	(void) prog_pdi_leave_prog_mode(ps);
	prog_pdi_hw_disable(ps);
}

void
prog_pdi_load_defaults(void)
{

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_pdi_clock = PDI_CLOCK_DEFAULT;
#else
        cf_set_uint32(CF_KEY_UINT32_PDI_CLOCK, PDI_CLOCK_DEFAULT);
#endif
}
