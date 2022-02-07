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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "programmer.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#endif
#include "platform.h"
#include "proto_stk500v2.h"
#include "prog_isp.h"
#include "prog_pdi.h"
#include "timer.h"
#include "network.h"
#include "shell.h"

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	proto_stk500v2_debug_flag
#include "debug.h"

#define	STK500_HARDWARE_VERSION			1
#define	STK500_FIRMWARE_VERSION			0x700

/* Supported STK500v2 commands */
#define STK_CMD_SIGN_ON				0x01u
#define STK_CMD_SET_PARAMETER			0x02u
#define STK_CMD_GET_PARAMETER			0x03u
#define STK_CMD_SET_DEVICE_PARAMETERS		0x04u
#define STK_CMD_OSCCAL				0x05u
#define STK_CMD_LOAD_ADDRESS			0x06u
#define STK_CMD_FIRMWARE_UPGRADE		0x07u
#define STK_CMD_CHECK_TARGET_CONNECTION		0x0du
#define STK_CMD_LOAD_RC_ID_TABLE		0x0eu
#define STK_CMD_LOAD_EC_ID_TABLE		0x0fu
#define STK_CMD_ENTER_PROGMODE_ISP		0x10u
#define STK_CMD_LEAVE_PROGMODE_ISP		0x11u
#define STK_CMD_CHIP_ERASE_ISP			0x12u
#define STK_CMD_PROGRAM_FLASH_ISP		0x13u
#define STK_CMD_READ_FLASH_ISP			0x14u
#define STK_CMD_PROGRAM_EEPROM_ISP		0x15u
#define STK_CMD_READ_EEPROM_ISP			0x16u
#define STK_CMD_PROGRAM_FUSE_ISP		0x17u
#define STK_CMD_READ_FUSE_ISP			0x18u
#define STK_CMD_PROGRAM_LOCK_ISP		0x19u
#define STK_CMD_READ_LOCK_ISP			0x1au
#define STK_CMD_READ_SIGNATURE_ISP		0x1bu
#define STK_CMD_READ_OSCCAL_ISP			0x1cu
#define STK_CMD_SPI_MULTI			0x1du
#define STK_CMD_SET_CONTROL_STACK		0x2du
#define	STK_CMD_LEAVE_PROGMODE_HVSP		0x31u
#define STK_CMD_XPROG				0x50u
#define STK_CMD_XPROG_SETMODE			0x51u
#define STK_CMD_UNKNOWN				0x58u

/* Response status codes */
#define STK_STATUS_CMD_OK			0x00u
#define STK_STATUS_CMD_TOUT			0x80u
#define STK_STATUS_RDY_BSY_TOUT			0x81u
#define STK_STATUS_SET_PARAM_MISSING		0x82u
#define STK_STATUS_CMD_FAILED			0xc0u
#define STK_STATUS_CKSUM_ERROR			0xc1u
#define STK_STATUS_CMD_UNKNOWN			0xc9u
#define	STK_ANSWER_CKSUM_ERROR			0xb0u

/* STK500 parameters */
#define	STK_PARAMS_BUILD_VERSION_LOW		0x00u
#define	STK_PARAMS_BUILD_VERSION_HIGH		0x01u
#define	STK_PARAMS_HARDWARE_VERSION		0x10u
#define	STK_PARAMS_FIRMWARE_VERSION_MAJOR	0x11u
#define	STK_PARAMS_FIRMWARE_VERSION_MINOR	0x12u
#define	STK_PARAMS_VTARGET			0x14u
#define	STK_PARAMS_VREF				0x15u
#define	STK_PARAMS_OSC_PRESCALE			0x16u
#define	STK_PARAMS_OSC_CMATCH			0x17u
#define	STK_PARAMS_SCK_DURATION			0x18u
#define	STK_PARAMS_TOP_CARD_DETECT		0x1au
#define	STK_PARAMS_STATUS			0x1cu
#define	STK_PARAMS_DATA				0x1du
#define	STK_PARAMS_RESET_POLARITY		0x1eu
#define	STK_PARAMS_CONTROLLER_INIT		0x1fu

/* Parameters specific to STK600 */
#define	STK6_PARAMS_STATUS_TGT_CONN		0x21u
#define	STK6_PARAMS_DISCHARGEDELAY		0x24u
#define	STK6_PARAMS_SOCKETCARD_ID		0x25u
#define	STK6_PARAMS_ROUTINGCARD_ID		0x26u
#define	STK6_PARAMS_EXPCARD_ID			0x27u
#define	STK6_PARAMS_SW_MAJOR_SLAVE1		0x28u
#define	STK6_PARAMS_SW_MINOR_SLAVE1		0x29u
#define	STK6_PARAMS_SW_MAJOR_SLAVE2		0x2au
#define	STK6_PARAMS_SW_MINOR_SLAVE2		0x2bu
#define	STK6_PARAMS_BOARD_ID_STATUS		0x2du
#define	STK6_PARAMS_RESET			0x34u
#define	STK_NPARAMS_1BYTE			0x40u

#define	STK6_PARAMS2_SCK_DURATION		0x00u
#define	STK6_PARAMS2_CLOCK_CONF			0x01u
#define	STK6_PARAMS2_AREF0			0x02u
#define	STK6_PARAMS2_AREF1			0x03u
#define	STK6_PARAMS2_JTAG_FLASH_SIZE_H		0x05u
#define	STK6_PARAMS2_JTAG_FLASH_SIZE_L		0x06u
#define	STK6_PARAMS2_RC_ID_TABLE_REV		0x08u
#define	STK6_PARAMS2_EC_ID_TABLE_REV		0x09u
#define	STK_NPARAMS_2BYTES			0x10u

#define	STK_NPARAMS				0x50u

/* Constants specific to XPROG mode */
#define	STK_XPROG_MODE_PDI			0x00u /* TPI/JTAG unsupported */
#define	STK_XPROG_CMD_ENTER_PROGMODE		0x01u
#define	STK_XPROG_CMD_LEAVE_PROGMODE		0x02u
#define	STK_XPROG_CMD_ERASE			0x03u
#define	STK_XPROG_CMD_WRITE_MEM			0x04u
#define	STK_XPROG_CMD_READ_MEM			0x05u
#define	STK_XPROG_CMD_CRC			0x06u
#define	STK_XPROG_CMD_SET_PARAM			0x07u

#define STK_XPROG_ERROR_OK			0x00u
#define STK_XPROG_ERROR_FAILURE			0x01u
#define STK_XPROG_ERROR_COLLISION		0x02u
#define STK_XPROG_ERROR_TIMEDOUT		0x03u

#define	STK_XPROG_ERASE_DEVICE			0x01u
#define	STK_XPROG_ERASE_APPLICATION		0x02u
#define	STK_XPROG_ERASE_BOOTLOADER		0x03u
#define	STK_XPROG_ERASE_EEPROM			0x04u
#define	STK_XPROG_ERASE_APPLICATION_PAGE	0x05u
#define	STK_XPROG_ERASE_BOOTLOADER_PAGE		0x06u
#define	STK_XPROG_ERASE_EEPROM_PAGE		0x07u
#define	STK_XPROG_ERASE_USER_SIGNATURE		0x08u
#define	STK_XPROG_ERASE_CONFIG			0x09u

#define	STK_XPROG_MEM_TYPE_APPLICATION		0x01u
#define	STK_XPROG_MEM_TYPE_BOOTLOADER		0x02u
#define	STK_XPROG_MEM_TYPE_EEPROM		0x03u
#define	STK_XPROG_MEM_TYPE_FUSE			0x04u
#define	STK_XPROG_MEM_TYPE_LOCKBITS		0x05u
#define	STK_XPROG_MEM_TYPE_USER_SIGNATURE	0x06u
#define	STK_XPROG_MEM_TYPE_FACTORY_CALIB	0x07u

#define	STK_XPROG_PAGEMODE_ERASE		(1u << 0)
#define	STK_XPROG_PAGEMODE_WRITE		(1u << 1)

#define STK_XPROG_PARAM_NVMBASE			0x01u
#define STK_XPROG_PARAM_EEPPAGESIZE		0x02u
#define STK_XPROG_PARAM_NVMCMD_REG		0x03u
#define STK_XPROG_PARAM_NVMCSR_REG		0x04u


struct stk500_header {
	uint8_t sh_start;
	uint8_t sh_sequence;
	uint16_t sh_len;	/* Note: big-endian */
	uint8_t sh_token;
} __attribute__((__packed__));
#define STK500_MAGIC_START			0x1bu
#define STK500_MAGIC_TOKEN			0x0eu

struct stk500_state {
	uint8_t ss_prog_mode;
	uint8_t ss_address_is_extended;
	uint32_t ss_address;
	uint32_t ss_isp_clock;
	uint8_t ss_params[STK_NPARAMS_1BYTE];
	uint16_t ss_params2[STK_NPARAMS_2BYTES];
};
#define	STK500_PROG_MODE_NONE	0u
#define	STK500_PROG_MODE_ISP	1u
#define	STK500_PROG_MODE_XPROG	2u

static struct stk500_state stk500_state;

static uint32_t stk500v2_process_header(struct programmer_state *);
static void stk500v2_process_body(struct programmer_state *);
static void stk500v2_connection_closed(struct programmer_state *);

const struct programmer_protocol proto_stk500v2_protocol = {
        sizeof(struct stk500_header),
	stk500v2_process_header,
	stk500v2_process_body,
	stk500_dispatch_request_from_jtag2,
	stk500v2_connection_closed,
};

#ifdef DEBUG_ENABLED
SHELL_CMD_DECL(stk, stk_shell_cmd, "Configure STK500 debug mode");
#endif

typedef void (*stk500_command_t)(struct programmer_state *);
struct stk500_command {
	uint8_t sc_cmd;
	stk500_command_t sc_handler;
};

/* STK500 top-level and ISP commands */
static void stk500_cmd_sign_on(struct programmer_state *);
static void stk500_cmd_set_parameter(struct programmer_state *);
static void stk500_cmd_get_parameter(struct programmer_state *);
static void stk500_cmd_osccal(struct programmer_state *);
static void stk500_cmd_load_address(struct programmer_state *);
static void stk500_cmd_check_target_connection(struct programmer_state *);
static void stk500_cmd_load_rc_id_table(struct programmer_state *);
static void stk500_cmd_load_ec_id_table(struct programmer_state *);
static void stk500_cmd_enter_progmode_isp(struct programmer_state *);
static void stk500_cmd_leave_progmode_isp(struct programmer_state *);
static void stk500_cmd_chip_erase_isp(struct programmer_state *);
static void stk500_cmd_program_flash_isp(struct programmer_state *);
static void stk500_cmd_read_flash_isp(struct programmer_state *);
static void stk500_cmd_program_eeprom_isp(struct programmer_state *);
static void stk500_cmd_read_eeprom_isp(struct programmer_state *);
static void stk500_cmd_program_fuse_isp(struct programmer_state *);
static void stk500_cmd_read_fuse_isp(struct programmer_state *);
static void stk500_cmd_program_lock_isp(struct programmer_state *);
static void stk500_cmd_read_lock_isp(struct programmer_state *);
static void stk500_cmd_read_signature_isp(struct programmer_state *);
static void stk500_cmd_read_osccal_isp(struct programmer_state *);
static void stk500_cmd_spi_multi(struct programmer_state *);
static void stk500_cmd_xprog(struct programmer_state *);
static void stk500_cmd_xprog_setmode(struct programmer_state *);
static void stk500_cmd_ignore(struct programmer_state *);

static const struct stk500_command stk500_commands[] = {
	{STK_CMD_SIGN_ON,		stk500_cmd_sign_on},
	{STK_CMD_SET_PARAMETER,		stk500_cmd_set_parameter},
	{STK_CMD_GET_PARAMETER,		stk500_cmd_get_parameter},
	{STK_CMD_OSCCAL,		stk500_cmd_osccal},
	{STK_CMD_LOAD_ADDRESS,		stk500_cmd_load_address},
	{STK_CMD_CHECK_TARGET_CONNECTION, stk500_cmd_check_target_connection},
	{STK_CMD_LOAD_RC_ID_TABLE,	stk500_cmd_load_rc_id_table},
	{STK_CMD_LOAD_EC_ID_TABLE,	stk500_cmd_load_ec_id_table},
	{STK_CMD_SET_CONTROL_STACK,	stk500_cmd_ignore},
	{STK_CMD_ENTER_PROGMODE_ISP,	stk500_cmd_enter_progmode_isp},
	{STK_CMD_LEAVE_PROGMODE_ISP,	stk500_cmd_leave_progmode_isp},
	{STK_CMD_CHIP_ERASE_ISP,	stk500_cmd_chip_erase_isp},
	{STK_CMD_PROGRAM_FLASH_ISP,	stk500_cmd_program_flash_isp},
	{STK_CMD_READ_FLASH_ISP,	stk500_cmd_read_flash_isp},
	{STK_CMD_PROGRAM_EEPROM_ISP,	stk500_cmd_program_eeprom_isp},
	{STK_CMD_READ_EEPROM_ISP,	stk500_cmd_read_eeprom_isp},
	{STK_CMD_PROGRAM_FUSE_ISP,	stk500_cmd_program_fuse_isp},
	{STK_CMD_READ_FUSE_ISP,		stk500_cmd_read_fuse_isp},
	{STK_CMD_PROGRAM_LOCK_ISP,	stk500_cmd_program_lock_isp},
	{STK_CMD_READ_LOCK_ISP,		stk500_cmd_read_lock_isp},
	{STK_CMD_READ_SIGNATURE_ISP,	stk500_cmd_read_signature_isp},
	{STK_CMD_READ_OSCCAL_ISP,	stk500_cmd_read_osccal_isp},
	{STK_CMD_SPI_MULTI,		stk500_cmd_spi_multi},
	{STK_CMD_XPROG,			stk500_cmd_xprog},
	{STK_CMD_XPROG_SETMODE,		stk500_cmd_xprog_setmode},
	{STK_CMD_UNKNOWN,		stk500_cmd_ignore},
	{STK_CMD_LEAVE_PROGMODE_HVSP,	stk500_cmd_ignore},
	{0,				NULL}
};

/* STK500 Xprog commands */
static void stk500_xcmd_enter_progmode(struct programmer_state *);
static void stk500_xcmd_leave_progmode(struct programmer_state *);
static void stk500_xcmd_erase(struct programmer_state *);
static void stk500_xcmd_write_mem(struct programmer_state *);
static void stk500_xcmd_read_mem(struct programmer_state *);
static void stk500_xcmd_crc(struct programmer_state *);
static void stk500_xcmd_set_param(struct programmer_state *);

static const stk500_command_t stk500_xcommands[] = {
	stk500_xcmd_enter_progmode,	/* STK_XPROG_CMD_ENTER_PROGMODE */
	stk500_xcmd_leave_progmode,	/* STK_XPROG_CMD_LEAVE_PROGMODE */
	stk500_xcmd_erase,		/* STK_XPROG_CMD_ERASE */
	stk500_xcmd_write_mem,		/* STK_XPROG_CMD_WRITE_MEM */
	stk500_xcmd_read_mem,		/* STK_XPROG_CMD_READ_MEM */
	stk500_xcmd_crc,		/* STK_XPROG_CMD_CRC */
	stk500_xcmd_set_param,		/* STK_XPROG_CMD_SET_PARAM */
};
#define	STK_NXCOMMAND	(sizeof(stk500_xcommands) / sizeof(stk500_xcommands[0]))

static stk500_command_t
stk500_lookup_command(const struct stk500_command *sc, uint8_t cmd)
{

	while (sc->sc_handler != NULL && sc->sc_cmd != cmd)
		sc++;

	return sc->sc_handler;
}

static uint32_t
stk500_compute_clock(uint8_t prescaler, uint8_t cmatch)
{
	uint32_t rv;
	uint16_t div;

#define	STK500_FSYS	7370000	/* As per Atmel's docs */

	prescaler &= 0x07;

	switch (prescaler) {
	case 1:
		div = 1;
		break;
	case 2:
		div = 8;
		break;
	case 3:
		div = 32;
		break;
	case 4:
		div = 64;
		break;
	case 5:
		div = 128;
		break;
	case 6:
		div = 256;
		break;
	case 7:
		div = 1024;
		break;
	default:
		return 0;
	}

	rv = STK500_FSYS / div;

	rv /= (cmatch + 1);

	return rv / 2;
}

static void
stk500_default_params(struct stk500_state *ss)
{
	fix16_t tvcc;

	/* Reset all parameters to default */
	memset(ss->ss_params, 0, sizeof(ss->ss_params));
	ss->ss_params[STK_PARAMS_HARDWARE_VERSION] = STK500_HARDWARE_VERSION;
	ss->ss_params[STK_PARAMS_FIRMWARE_VERSION_MAJOR] =
	    STK500_FIRMWARE_VERSION >> 8;
	ss->ss_params[STK_PARAMS_FIRMWARE_VERSION_MINOR] =
	    STK500_FIRMWARE_VERSION & 0xffu;
	tvcc = fix16_mul(power_target_vcc, F16(10.0));
	ss->ss_params[STK_PARAMS_VTARGET] = (uint8_t) fix16_to_int(tvcc);
	ss->ss_params[STK_PARAMS_OSC_PRESCALE] = 1;
	ss->ss_params[STK_PARAMS_OSC_CMATCH] = 2;
	ss->ss_params[STK_PARAMS_SCK_DURATION] = 2;
	ss->ss_params[STK_PARAMS_TOP_CARD_DETECT] = 0xaa;

	/* Ditto for the STK600 params */
	ss->ss_params[STK6_PARAMS_STATUS_TGT_CONN] = 1;	/* XXX: dynamic! */
	ss->ss_params[STK6_PARAMS_DISCHARGEDELAY] = 0;
	ss->ss_params[STK6_PARAMS_SOCKETCARD_ID] = 0xff;
	ss->ss_params[STK6_PARAMS_ROUTINGCARD_ID] = 0xff;
	ss->ss_params[STK6_PARAMS_EXPCARD_ID] = 0;
	ss->ss_params[STK6_PARAMS_SW_MAJOR_SLAVE1] = 2;
	ss->ss_params[STK6_PARAMS_SW_MINOR_SLAVE1] = 4;
	ss->ss_params[STK6_PARAMS_SW_MAJOR_SLAVE2] = 2;
	ss->ss_params[STK6_PARAMS_SW_MINOR_SLAVE2] = 4;
	ss->ss_params[STK6_PARAMS_BOARD_ID_STATUS] = 0;
	ss->ss_params[STK6_PARAMS_RESET] = 0;

	ss->ss_params2[STK6_PARAMS2_SCK_DURATION] = 4;
	ss->ss_params2[STK6_PARAMS2_CLOCK_CONF] = 0;
	ss->ss_params2[STK6_PARAMS2_AREF0] = 0;
	ss->ss_params2[STK6_PARAMS2_AREF1] = 0;
	ss->ss_params2[STK6_PARAMS2_JTAG_FLASH_SIZE_H] = 0x10;
	ss->ss_params2[STK6_PARAMS2_JTAG_FLASH_SIZE_L] = 0x0;
	ss->ss_params2[STK6_PARAMS2_RC_ID_TABLE_REV] = 0x0182;
	ss->ss_params2[STK6_PARAMS2_EC_ID_TABLE_REV] = 0x0002;

	ss->ss_isp_clock = stk500_compute_clock(
	    ss->ss_params[STK_PARAMS_OSC_PRESCALE],
	    ss->ss_params[STK_PARAMS_OSC_CMATCH]);
}

static void
stk500_cmd_sign_on(struct programmer_state *ps)
{
	struct stk500_state *ss;
	uint32_t devid_size;
	const char *devid;

	ss = &stk500_state;

	if (ps->ps_proto_state == ss) {
		DBFPRINTF("already signed on\n");
		/*
		 * Not really an error - this may be a retransmit if
		 * our previous reply was dropped.
		 */
	} else {
		ps->ps_proto_state = ss;
	}

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);

	switch (programmer_get_proto()) {
	default:
		devid = PROGRAMMER_DEV_ID;
		break;

	case PROG_PROTO_DRAGON:
		devid = "DRAGON";
		break;

	case PROG_PROTO_WAVR:
		devid = "WAVR";
		break;
	}

	devid_size = strlen(devid);
	programmer_reply_add_byte(ps, (uint8_t)devid_size);
	programmer_reply_add_buff(ps, devid, devid_size);
}

static float
stk500_compute_sck_duration(struct programmer_state *ps, uint8_t value)
{
	float sck_duration;

	(void) ps;

#define	CLOCK_TO_US(m)	((1.0 / (m)) * 1000000.0)
	switch (value) {
	case 0:
		/* 1.8 MHz clock */
		sck_duration = CLOCK_TO_US(1800000.0);
		break;
	case 1:
		sck_duration = CLOCK_TO_US(460000.0);
		break;
	case 2:
		sck_duration = CLOCK_TO_US(115200.0);
		break;
	case 3:
		sck_duration = CLOCK_TO_US(58000.0);
		break;
	default:
		sck_duration = 3.25 * value;
		break;
	}

	return sck_duration;
}

static void
stk500_cmd_set_parameter(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t param;

	if (programmer_request_get_byte(ps, &param) != 1) {
		DBFPRINTF("failed to get param\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (param < 0x80u || param > (0x80u + STK_NPARAMS)) {
		DBFPRINTF("bad param %02x\n", param);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	param -= 0x80u;

	if (param < STK_NPARAMS_1BYTE) {
		if (programmer_request_get_byte(ps,
		    &ss->ss_params[param]) != 1) {
			DBFPRINTF("failed to get value\n");
			programmer_reply_add_byte(ps,
			    STK_STATUS_SET_PARAM_MISSING);
			return;
		}

		DBFPRINTF("param %02x -> %02x\n", param, ss->ss_params[param]);

		if (param == STK_PARAMS_SCK_DURATION) {
			prog_isp_update_sck(ps->ps_isp,
			    stk500_compute_sck_duration(ps,
			    ss->ss_params[param]));
		}

		if (param == STK_PARAMS_OSC_PRESCALE ||
		    param == STK_PARAMS_OSC_CMATCH) {
			ss->ss_isp_clock = stk500_compute_clock(
			    ss->ss_params[STK_PARAMS_OSC_PRESCALE],
			    ss->ss_params[STK_PARAMS_OSC_CMATCH]);
		}

		DBFPRINTF("param[%02x] <- %02x\n", param, ss->ss_params[param]);
	} else {
		uint8_t p[2];

		param -= STK_NPARAMS_1BYTE;

		if (programmer_request_get_buff(ps, p, 2) != 2) {
			DBFPRINTF("failed to get value\n");
			programmer_reply_add_byte(ps,
			    STK_STATUS_SET_PARAM_MISSING);
			return;
		}

		ss->ss_params2[param] = (uint16_t)p[0] << 8;
		ss->ss_params2[param] |= (uint16_t)p[1];

		if (param == STK6_PARAMS2_SCK_DURATION) {
			float v;
			v = 1000000.0 /
			    (8000000.0 / (ss->ss_params2[param] + 1));
			prog_isp_update_sck(ps->ps_isp, v);
		}

		DBFPRINTF("param2[%02x] <- %04x\n", param,
		    ss->ss_params2[param]);
	}

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
}

static void
stk500_cmd_get_parameter(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t param;

	if (programmer_request_get_byte(ps, &param) != 1) {
		DBFPRINTF("failed to get param\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (param < 0x80u || param > (0x80u + STK_NPARAMS)) {
		DBFPRINTF("bad param %02x\n", param);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	param -= 0x80u;

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);

	if (param < STK_NPARAMS_1BYTE) {
		if (param == STK_PARAMS_VTARGET) {
			fix16_t tvcc;
			tvcc = fix16_mul(power_target_vcc, F16(10.0));
			ss->ss_params[param] = (uint8_t) fix16_to_int(tvcc);
		}
		programmer_reply_add_byte(ps, ss->ss_params[param]);
		DBFPRINTF("param[%02x] -> %02x\n", param, ss->ss_params[param]);
	} else {
		param -= STK_NPARAMS_1BYTE;
		programmer_reply_add_byte(ps, ss->ss_params2[param] >> 8);
		programmer_reply_add_byte(ps, ss->ss_params2[param] & 0xffu);
		DBFPRINTF("param2[%02x] -> %04x\n", param,
		    ss->ss_params2[param]);
	}
}

static void
stk500_cmd_osccal(struct programmer_state *ps)
{

	/* XXX: Implement me? */
	programmer_reply_add_byte(ps, STK_STATUS_CMD_UNKNOWN);
}

static void
stk500_cmd_load_address(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint32_t address;

	if (programmer_request_get_buff(ps, &address, 4) != 4) {
		DBFPRINTF("failed to get address\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	address = ntoh32(address);
	ss->ss_address = address & 0x00fffffful;
	ss->ss_address_is_extended = (address & 0x80000000ul) != 0;

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
}

static void
stk500_cmd_check_target_connection(struct programmer_state *ps)
{

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	programmer_reply_add_byte(ps, 0);
}

static void
stk500_cmd_load_rc_id_table(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t p[2];

	if (programmer_request_get_buff(ps, p, 2) != 2) {
		DBFPRINTF("failed to get table revision\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);

	ss->ss_params2[STK6_PARAMS2_RC_ID_TABLE_REV] = (uint16_t)p[0] << 8;
	ss->ss_params2[STK6_PARAMS2_RC_ID_TABLE_REV] |= (uint16_t)p[1] & 0xffu;

	DBFPRINTF("rev %04x\n", ss->ss_params2[STK6_PARAMS2_RC_ID_TABLE_REV]);
}

static void
stk500_cmd_load_ec_id_table(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t p[2];

	if (programmer_request_get_buff(ps, p, 2) != 2) {
		DBFPRINTF("failed to get table revision\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);

	ss->ss_params2[STK6_PARAMS2_EC_ID_TABLE_REV] = (uint16_t)p[0] << 8;
	ss->ss_params2[STK6_PARAMS2_EC_ID_TABLE_REV] |= (uint16_t)p[1] & 0xffu;

	DBFPRINTF("rev %04x\n", ss->ss_params2[STK6_PARAMS2_EC_ID_TABLE_REV]);
}

static void
stk500_cmd_ignore(struct programmer_state *ps)
{

	programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
}

static void
stk500_cmd_enter_progmode_isp(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_start_params isp;
	uint8_t enter_args[11];

	DBFPRINTF("\n");

	if (programmer_request_get_buff(ps, enter_args, sizeof(enter_args)) !=
	    sizeof(enter_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (ss->ss_prog_mode != STK500_PROG_MODE_NONE) {
		DBFPRINTF("already in prog mode\n");
		programmer_reply_add_byte(ps,
		    (ss->ss_prog_mode == STK500_PROG_MODE_ISP) ?
		    STK_XPROG_ERROR_OK : STK_STATUS_CMD_FAILED);
		return;
	}

	/*
	 * The layout of most of struct prog_isp_start_params should match
	 * enter_args[] in theory, but I'm not going to trust the compiler
	 * to keep it that way.
	 */
	isp.isp_sck_duration = stk500_compute_sck_duration(ps,
	    ss->ss_params[STK_PARAMS_SCK_DURATION]);
	isp.isp_clock = ss->ss_isp_clock;
	isp.isp_timeout = enter_args[0];
	isp.isp_stab_delay = enter_args[1];
	isp.isp_cmd_delay = enter_args[2];
	isp.isp_sync_loops = enter_args[3];
	isp.isp_byte_delay = enter_args[4];
	isp.isp_poll_val = enter_args[5];
	isp.isp_poll_idx = enter_args[6];
	isp.isp_command[0] = enter_args[7];
	isp.isp_command[1] = enter_args[8];
	isp.isp_command[2] = enter_args[9];
	isp.isp_command[3] = enter_args[10];

	if (prog_isp_start(ps->ps_isp, &isp) == PROG_ISP_STATUS_OK) {
		ss->ss_prog_mode = STK500_PROG_MODE_ISP;
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	} else {
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		DBFPRINTF("prog_isp_start failed\n");
	}
}

static void
stk500_cmd_leave_progmode_isp(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_finish_params ifp;
	uint8_t leave_args[2];

	if (programmer_request_get_buff(ps, leave_args, sizeof(leave_args)) !=
	    sizeof(leave_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("not in ISP prog mode\n");
		/* Pretend we're good. */
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
		return;
	}

	/*
	 * The layout of struct prog_isp_finish_params should match
	 * leave_args[] in theory, but I'm not going to trust the
	 * compiler to keep it that way.
	 */
	ifp.ifp_delay_pre = leave_args[0];
	ifp.ifp_delay_post = leave_args[1];

	if (prog_isp_finish(ps->ps_isp, &ifp) == PROG_ISP_STATUS_OK) {
		ss->ss_prog_mode = STK500_PROG_MODE_NONE;
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	} else
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
}

static void
stk500_cmd_chip_erase_isp(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_chip_erase_params ice;
	uint8_t erase_args[6];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("not in prog mode\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, erase_args, sizeof(erase_args)) !=
	    sizeof(erase_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/*
	 * The layout of struct prog_isp_chip_erase_params should match
	 * erase_args[] in theory, but I'm not going to trust the
	 * compiler to keep it that way.
	 */
	ice.ice_delay = erase_args[0];
	ice.ice_poll = erase_args[1];
	ice.ice_command[0] = erase_args[2];
	ice.ice_command[1] = erase_args[3];
	ice.ice_command[2] = erase_args[4];
	ice.ice_command[3] = erase_args[5];

	switch (prog_isp_chip_erase(ps->ps_isp, &ice)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
}

static void
stk500_program_flash_eeprom_isp(struct programmer_state *ps, uint8_t is_eeprom)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_memory_program_params imp;
	uint8_t prog_args[9];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("is_eeprom %u: not in prog mode\n", is_eeprom);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, prog_args, sizeof(prog_args)) !=
	    sizeof(prog_args)) {
		DBFPRINTF("is_eeprom %u: failed to get args\n", is_eeprom);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Grab the length and verify it */
	imp.imp_len = (((uint16_t)prog_args[0]) << 8) | prog_args[1];

	if (imp.imp_len == 0 || imp.imp_len > (ps->ps_request_len -
	    ps->ps_request_cursor)) {
		DBFPRINTF("is_eeprom %u: bad request size (%u)\n", is_eeprom,
		    imp.imp_len);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Copy the remaining parameters */
	imp.imp_mode = prog_args[2];
	imp.imp_delay = prog_args[3];
	imp.imp_command[0] = prog_args[4];
	imp.imp_command[1] = prog_args[5];
	imp.imp_command[2] = prog_args[6];
	imp.imp_poll[0] = prog_args[7];
	imp.imp_poll[1] = prog_args[8];

	imp.imp_is_eeprom = is_eeprom;
	imp.imp_address = ss->ss_address;
	imp.imp_extended_address = ss->ss_address_is_extended;
	imp.imp_data = &ps->ps_request[ps->ps_request_cursor];

	switch (prog_isp_memory_program(ps->ps_isp, &imp)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		if (is_eeprom)
			ss->ss_address += imp.imp_len;
		else
			ss->ss_address += imp.imp_len / 2;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
}

static void
stk500_cmd_program_flash_isp(struct programmer_state *ps)
{

	stk500_program_flash_eeprom_isp(ps, 0);
}

static void
stk500_cmd_program_eeprom_isp(struct programmer_state *ps)
{

	stk500_program_flash_eeprom_isp(ps, 1);
}

static void
stk500_program_fuse_lock_isp(struct programmer_state *ps, uint8_t is_lock)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_fuse_program_params ifp;
	uint8_t prog_args[4];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("is_lock %u: not in prog mode\n", is_lock);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, prog_args, sizeof(prog_args)) !=
	    sizeof(prog_args)) {
		DBFPRINTF("is_lock %u: failed to get args\n", is_lock);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Copy the parameters */
	ifp.ifp_is_lock = is_lock;
	ifp.ifp_command[0] = prog_args[0];
	ifp.ifp_command[1] = prog_args[1];
	ifp.ifp_command[2] = prog_args[2];
	ifp.ifp_command[3] = prog_args[3];

	switch (prog_isp_fuse_program(ps->ps_isp, &ifp)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
	if (status == STK_STATUS_CMD_OK)
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
}

static void
stk500_cmd_program_fuse_isp(struct programmer_state *ps)
{

	stk500_program_fuse_lock_isp(ps, 0);
}

static void
stk500_cmd_program_lock_isp(struct programmer_state *ps)
{

	stk500_program_fuse_lock_isp(ps, 1);
}

static void
stk500_read_flash_eeprom_isp(struct programmer_state *ps, bool is_eeprom)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_memory_read_params imr;
	uint8_t read_args[3];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("is_eeprom %u: not in prog mode\n", is_eeprom);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, read_args, sizeof(read_args)) !=
	    sizeof(read_args)) {
		DBFPRINTF("is_eeprom %u: failed to get args\n", is_eeprom);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Grab the length and verify it */
	imr.imr_len = (((uint16_t)read_args[0]) << 8) | read_args[1];

	if (imr.imr_len == 0 || imr.imr_len > (sizeof(ps->ps_reply) -
	    (sizeof(struct stk500_header) + 1))) {
		DBFPRINTF("is_eeprom %u: bad request size (%u)\n", is_eeprom,
		    imr.imr_len);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Copy the remaining parameter */
	imr.imr_command = read_args[2];
	imr.imr_is_eeprom = (uint8_t)is_eeprom;
	imr.imr_address = ss->ss_address;
	imr.imr_extended_address = ss->ss_address_is_extended;
	imr.imr_data = &ps->ps_reply[ps->ps_reply_len + 1];

	switch (prog_isp_memory_read(ps->ps_isp, &imr)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		if (is_eeprom)
			ss->ss_address += imr.imr_len;
		else
			ss->ss_address += imr.imr_len / 2;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
	if (status == STK_STATUS_CMD_OK) {
		ps->ps_reply_len += imr.imr_len;
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	}
}

static void
stk500_cmd_read_flash_isp(struct programmer_state *ps)
{

	stk500_read_flash_eeprom_isp(ps, false);
}

static void
stk500_cmd_read_eeprom_isp(struct programmer_state *ps)
{

	stk500_read_flash_eeprom_isp(ps, true);
}

static void
stk500_read_special_isp(struct programmer_state *ps, const char *which)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_fuse_read_params ifr;
	uint8_t read_args[5];
	uint8_t value, status;

	(void) which;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("which '%s': not in prog mode\n", which);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, read_args, sizeof(read_args)) !=
	    sizeof(read_args)) {
		DBFPRINTF("which '%s': failed to get args\n", which);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Copy the parameters */
	ifr.ifr_addr = read_args[0];
	ifr.ifr_command[0] = read_args[1];
	ifr.ifr_command[1] = read_args[2];
	ifr.ifr_command[2] = read_args[3];
	ifr.ifr_command[3] = read_args[4];

	switch (prog_isp_fuse_read(ps->ps_isp, &ifr, &value)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
	if (status == STK_STATUS_CMD_OK) {
		programmer_reply_add_byte(ps, value);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	}
}

static void
stk500_cmd_read_fuse_isp(struct programmer_state *ps)
{

	stk500_read_special_isp(ps, "fuse");
}

static void
stk500_cmd_read_lock_isp(struct programmer_state *ps)
{

	stk500_read_special_isp(ps, "lock");
}

static void
stk500_cmd_read_signature_isp(struct programmer_state *ps)
{

	stk500_read_special_isp(ps, "signature");
}

static void
stk500_cmd_read_osccal_isp(struct programmer_state *ps)
{

	stk500_read_special_isp(ps, "osccal");
}

static void
stk500_cmd_spi_multi(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_isp_multi_spi_params ims;
	uint8_t multi_args[3];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_ISP) {
		DBFPRINTF("not in prog mode\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	if (programmer_request_get_buff(ps, multi_args, sizeof(multi_args)) !=
	    sizeof(multi_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	/* Validate the lengths */
	ims.ims_num_tx = multi_args[0];
	if (ims.ims_num_tx > (ps->ps_request_len - ps->ps_request_cursor)) {
		DBFPRINTF("bad Tx size (%u)\n", ims.ims_num_tx);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	ims.ims_num_rx = multi_args[1];
#if 0
	if (ims.ims_num_tx > (sizeof(ps->ps_reply) -
	    (sizeof(struct stk500_header) + 1))) {
		DBFPRINTF("bad Rx size (%u)\n", ims.ims_num_rx);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}
#endif

	/* Setup the remaining parameters */
	ims.ims_start_addr = multi_args[2];
	ims.ims_txbuff = &ps->ps_request[ps->ps_request_cursor];
	ims.ims_rxbuff = &ps->ps_reply[ps->ps_reply_len + 1];

	switch (prog_isp_spi_multi(ps->ps_isp, &ims)) {
	case PROG_ISP_STATUS_OK:
		status = STK_STATUS_CMD_OK;
		break;
	case PROG_ISP_STATUS_TIMEOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
	if (status == STK_STATUS_CMD_OK) {
		ps->ps_reply_len += ims.ims_num_rx;
		programmer_reply_add_byte(ps, STK_STATUS_CMD_OK);
	}
}

static void
stk500_cmd_xprog(struct programmer_state *ps)
{
	uint8_t cmd;

	if (programmer_request_get_byte(ps, &cmd) != 1) {
		DBFPRINTF("failed to get sub-command\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	/* Echo the command in the reply */
	programmer_reply_add_byte(ps, cmd);

	if (cmd < 1 || cmd > STK_NXCOMMAND) {
		DBFPRINTF("invalid sub-command %02x\n", cmd);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	(stk500_xcommands[cmd - 1])(ps);
}

static void
stk500_cmd_xprog_setmode(struct programmer_state *ps)
{
	uint8_t mode;

	if (programmer_request_get_byte(ps, &mode) != 1) {
		DBFPRINTF("failed to get mode\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (mode != STK_XPROG_MODE_PDI) {
		DBFPRINTF("bad mode %u\n", mode);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	programmer_reply_add_byte(ps, STK_XPROG_ERROR_OK);
}

static void
stk500_xcmd_enter_progmode(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_NONE) {
		DBFPRINTF("already in prog mode\n");
		programmer_reply_add_byte(ps,
		    (ss->ss_prog_mode == STK500_PROG_MODE_XPROG) ?
		    STK_XPROG_ERROR_OK : STK_XPROG_ERROR_FAILURE);
		return;
	}

	switch (prog_pdi_enter_prog_mode(ps->ps_pdi)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		ss->ss_prog_mode = STK500_PROG_MODE_XPROG;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	if (status != STK_XPROG_ERROR_OK)
		prog_pdi_reset_comms(ps->ps_pdi);

	programmer_reply_add_byte(ps, status);
}

static void
stk500_xcmd_leave_progmode(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_XPROG) {
		DBFPRINTF("not in xprog mode\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	switch (prog_pdi_leave_prog_mode(ps->ps_pdi)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	if (status != STK_XPROG_ERROR_OK)
		prog_pdi_reset_comms(ps->ps_pdi);

	programmer_reply_add_byte(ps, status);
}

static void
stk500_xcmd_erase(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_pdi_erase_arg pea;
	uint8_t erase_args[5];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_XPROG) {
		DBFPRINTF("not in xprog mode\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (programmer_request_get_buff(ps, erase_args, sizeof(erase_args)) !=
	    sizeof(erase_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	switch (erase_args[0]) {
	case STK_XPROG_ERASE_DEVICE:
		pea.pea_mem_type = PROG_PDI_ERASE_DEVICE;
		break;
	case STK_XPROG_ERASE_APPLICATION:
		pea.pea_mem_type = PROG_PDI_ERASE_APPLICATION;
		break;
	case STK_XPROG_ERASE_BOOTLOADER:
		pea.pea_mem_type = PROG_PDI_ERASE_BOOTLOADER;
		break;
	case STK_XPROG_ERASE_EEPROM:
		pea.pea_mem_type = PROG_PDI_ERASE_EEPROM;
		break;
	case STK_XPROG_ERASE_APPLICATION_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_APPLICATION_PAGE;
		break;
	case STK_XPROG_ERASE_BOOTLOADER_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_BOOTLOADER_PAGE;
		break;
	case STK_XPROG_ERASE_EEPROM_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_EEPROM_PAGE;
		break;
	case STK_XPROG_ERASE_USER_SIGNATURE:
		pea.pea_mem_type = PROG_PDI_ERASE_USER_SIGNATURE;
		break;
	default:
		DBFPRINTF("unknown memory type %u\n", erase_args[0]);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	pea.pea_address = ((uint32_t)erase_args[1]) << 24;
	pea.pea_address |= ((uint32_t)erase_args[2]) << 16;
	pea.pea_address |= ((uint32_t)erase_args[3]) << 8;
	pea.pea_address |= ((uint32_t)erase_args[4]) << 0;

	switch (prog_pdi_erase(ps->ps_pdi, &pea)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	if (status != STK_XPROG_ERROR_OK)
		prog_pdi_reset_comms(ps->ps_pdi);

	programmer_reply_add_byte(ps, status);
}

static int
stk500_xcmd_decode_mem_type(uint8_t mt, uint8_t *prv)
{

	switch (mt) {
	case STK_XPROG_MEM_TYPE_APPLICATION:
		*prv = PROG_PDI_MEM_TYPE_APPLICATION;
		break;
	case STK_XPROG_MEM_TYPE_BOOTLOADER:
		*prv = PROG_PDI_MEM_TYPE_BOOTLOADER;
		break;
	case STK_XPROG_MEM_TYPE_EEPROM:
		*prv = PROG_PDI_MEM_TYPE_EEPROM;
		break;
	case STK_XPROG_MEM_TYPE_FUSE:
		*prv = PROG_PDI_MEM_TYPE_FUSE;
		break;
	case STK_XPROG_MEM_TYPE_LOCKBITS:
		*prv = PROG_PDI_MEM_TYPE_LOCKBITS;
		break;
	case STK_XPROG_MEM_TYPE_USER_SIGNATURE:
		*prv = PROG_PDI_MEM_TYPE_USER_SIGNATURE;
		break;
	case STK_XPROG_MEM_TYPE_FACTORY_CALIB:
		*prv = PROG_PDI_MEM_TYPE_FACTORY_CALIB;
		break;
	default:
		return -1;
	}

	return 0;
}

static void
stk500_xcmd_write_mem(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_pdi_program_arg ppa;
	uint8_t write_args[8];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_XPROG) {
		DBFPRINTF("not in xprog mode\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (programmer_request_get_buff(ps, write_args, sizeof(write_args)) !=
	    sizeof(write_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (stk500_xcmd_decode_mem_type(write_args[0], &ppa.ppa_mem_type) < 0) {
		DBFPRINTF("bad mem-type %02x\n", write_args[0]);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	ppa.ppa_length = (((uint16_t)write_args[6]) << 8) | write_args[7];
	if (ppa.ppa_length == 0 || ppa.ppa_length > (ps->ps_request_len -
	    ps->ps_request_cursor)) {
		DBFPRINTF("bad request size (%u)\n", ppa.ppa_length);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	ppa.ppa_page_mode = 0;
	if (write_args[1] & STK_XPROG_PAGEMODE_ERASE)
		ppa.ppa_page_mode |= PROG_PDI_PAGEMODE_ERASE;
	if (write_args[1] & STK_XPROG_PAGEMODE_WRITE)
		ppa.ppa_page_mode |= PROG_PDI_PAGEMODE_WRITE;

	ppa.ppa_address = ((uint32_t)write_args[2]) << 24;
	ppa.ppa_address |= ((uint32_t)write_args[3]) << 16;
	ppa.ppa_address |= ((uint32_t)write_args[4]) << 8;
	ppa.ppa_address |= ((uint32_t)write_args[5]) << 0;
	ppa.ppa_data = &ps->ps_request[ps->ps_request_cursor];

	switch (prog_pdi_program(ps->ps_pdi, &ppa)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	if (status != STK_XPROG_ERROR_OK)
		prog_pdi_reset_comms(ps->ps_pdi);

	programmer_reply_add_byte(ps, status);
}

static void
stk500_xcmd_read_mem(struct programmer_state *ps)
{
	struct stk500_state *ss = ps->ps_proto_state;
	struct prog_pdi_read_arg pra;
	uint8_t read_args[7];
	uint8_t status;

	if (ss->ss_prog_mode != STK500_PROG_MODE_XPROG) {
		DBFPRINTF("not in xprog mode\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (programmer_request_get_buff(ps, read_args, sizeof(read_args)) !=
	    sizeof(read_args)) {
		DBFPRINTF("failed to get args\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (stk500_xcmd_decode_mem_type(read_args[0], &pra.pra_mem_type) < 0) {
		DBFPRINTF("bad mem-type %02x\n", read_args[0]);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	pra.pra_length = (((uint16_t)read_args[5]) << 8) | read_args[6];
	if (pra.pra_length == 0 || pra.pra_length > (sizeof(ps->ps_reply) -
	    sizeof(struct stk500_header) + 2)) {
		DBFPRINTF("bad request size (%u)\n", pra.pra_length);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	pra.pra_address = ((uint32_t)read_args[1]) << 24;
	pra.pra_address |= ((uint32_t)read_args[2]) << 16;
	pra.pra_address |= ((uint32_t)read_args[3]) << 8;
	pra.pra_address |= ((uint32_t)read_args[4]) << 0;
	pra.pra_buffer = &ps->ps_reply[ps->ps_reply_len + 1];

	switch (prog_pdi_read(ps->ps_pdi, &pra)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		pra.pra_length = 0;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		pra.pra_length = 0;
		break;
	}

	if (status != STK_XPROG_ERROR_OK)
		prog_pdi_reset_comms(ps->ps_pdi);

	programmer_reply_add_byte(ps, status);
	ps->ps_reply_len += pra.pra_length;
}

static void
stk500_xcmd_crc(struct programmer_state *ps)
{

	/* XXX: Support this? */
	programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
}

static void
stk500_xcmd_set_param(struct programmer_state *ps)
{
	union prog_pdi_param_arg ppa;
	uint8_t param, status, len;

	if (programmer_request_get_byte(ps, &param) != 1) {
		DBFPRINTF("failed to get type\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	switch (param) {
	case STK_XPROG_PARAM_NVMBASE:
		param = PROG_PDI_PARAM_NVMBASE;
		len = 4;
		break;
	case STK_XPROG_PARAM_EEPPAGESIZE:
		param = PROG_PDI_PARAM_EEPAGESIZE;
		len = 2;
		break;
	case STK_XPROG_PARAM_NVMCMD_REG:
		param = PROG_PDI_PARAM_NVMCMD_REG;
		len = 1;
		break;
	case STK_XPROG_PARAM_NVMCSR_REG:
		param = PROG_PDI_PARAM_NVMCSR_REG;
		len = 1;
		break;
	default:
		/* Pretend this succeeded */
		DBFPRINTF("unknown param %02x\n", param);
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_OK);
		return;
	}

	if (len < sizeof(ppa))
		memset(&ppa, 0, sizeof(ppa));
	if (programmer_request_get_buff(ps, &ppa, len) != len) {
		DBFPRINTF("failed to read param\n");
		programmer_reply_add_byte(ps, STK_XPROG_ERROR_FAILURE);
		return;
	}

	if (len == 4)
		ppa.ppa_nvm_base = ntoh32(ppa.ppa_nvm_base);
	else
	if (len == 2)
		ppa.ppa_ee_page_size = ntoh16(ppa.ppa_ee_page_size);

	switch (prog_pdi_set_param(ps->ps_pdi, param, &ppa)) {
	case PROG_PDI_STATUS_OK:
		status = STK_XPROG_ERROR_OK;
		break;
	case PROG_PDI_STATUS_TIMEDOUT:
		status = STK_STATUS_CMD_TOUT;
		break;
	default:
		status = STK_STATUS_CMD_FAILED;
		break;
	}

	programmer_reply_add_byte(ps, status);
}

static void
stk500_dispatch_request(struct programmer_state *ps)
{
	stk500_command_t handler;
	uint8_t cmd;

	/* Read the command byte. */
	if (programmer_request_get_byte(ps, &cmd) != 1) {
		DBFPRINTF("failed to get cmd byte\n");
		return;
	}

	/* STK500v2 replies always echo the command as the first byte */
	programmer_reply_add_byte(ps, cmd);

	/* The first command *must* be sign-on. */
	if (ps->ps_proto_state == NULL && cmd != STK_CMD_SIGN_ON) {
		DBFPRINTF("cmd %02x but not signed on\n", cmd);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_FAILED);
		return;
	}

	DBFPRINTF("cmd %02x\n", cmd);

	if ((handler = stk500_lookup_command(stk500_commands, cmd)) == NULL) {
		DBFPRINTF("cmd %02x not supported\n", cmd);
		programmer_reply_add_byte(ps, STK_STATUS_CMD_UNKNOWN);
		return;
	}

	(handler)(ps);
}

void
stk500_dispatch_request_from_jtag2(struct programmer_state *ps)
{
	struct stk500_state *ss;

	/*
	 * We don't normally get a sign-on command using this API so fix
	 * things up as if a sign-on happened.
	 */
	if ((ss = ps->ps_proto_state) == NULL) {
		ss = &stk500_state;
		ps->ps_proto_state = ss;
		stk500_default_params(ss);
	}

	/* Dispatch in the normal way */
	stk500_dispatch_request(ps);
}

static uint8_t
stk500_checksum(const void *p, uint16_t len, uint8_t sum)
{
	const uint8_t *buff = p;

	while (len--)
		sum ^= *buff++;

	return sum;
}

static uint32_t
stk500v2_process_header(struct programmer_state *ps)
{
	struct stk500_header *sh;
	uint16_t len;

	sh = (struct stk500_header *) ps->ps_request;

	/* Verify the magic values */
	if (sh->sh_start != STK500_MAGIC_START ||
	    sh->sh_token != STK500_MAGIC_TOKEN) {
		DBFPRINTF("start/token mismatch\n");
		return 0;
	}

	/* Copy the header into the reply buffer for later */
	if (programmer_reply_add_buff(ps, sh, sizeof(*sh)) != sizeof(*sh)) {
		DBFPRINTF("failed to back-copy header\n");
		return 0;
	}

	/*
	 * Let the caller know the body size, including checksum. Note that
	 * a bogus length (zero or > sizeof(ps->ps_request)) will be
	 * handled by the caller.
	 */
	len = ntoh16(sh->sh_len) + 1;

	return (uint32_t) len;
}

static void
stk500v2_process_body(struct programmer_state *ps)
{
	struct stk500_header *sh;
	uint16_t len;
	uint8_t sum, msgsum;

	/* Header was already copied into the reply buffer */
	sh = (struct stk500_header *) ps->ps_reply;
	len = ntoh16(sh->sh_len);

	if (len != (ps->ps_request_len - 1)) {
		DBFPRINTF("len %u != reply %lu\n", len, ps->ps_request_len - 1);
		return;
	}

	/* Compute local sum over header and body */
	msgsum = stk500_checksum(sh, sizeof(*sh), 0x00u);
	msgsum = stk500_checksum(ps->ps_request, len, msgsum);

	/* Get the CRC sent by the host */
	sum = ps->ps_request[len];

	/* Proceed only if sums match */
	if (msgsum != sum) {
		DBFPRINTF("bad sum (%02x != %02x)\n", msgsum, sum);
		programmer_reply_add_byte(ps, STK_ANSWER_CKSUM_ERROR);
	} else {
		/* Strip the trailing checksum */
		ps->ps_request_len -= 1;

		/* Decode the request */
		stk500_dispatch_request(ps);
	}

	/* Send the response */
	if (ps->ps_reply_len <= sizeof(*sh)) {
		DBFPRINTF("bogus reply length!\n");
		ps->ps_reply_len = 0;
		return;
	}

	/* Tweak the header and compute the new checksum */
	len = ps->ps_reply_len;
	sh->sh_len = hton16(len - sizeof(*sh));
	sum = stk500_checksum(ps->ps_reply, len, 0);
	programmer_reply_add_byte(ps, sum);
}

static void
stk500v2_connection_closed(struct programmer_state *ps)
{

	if (ps->ps_proto_state == &stk500_state)
		ps->ps_proto_state = NULL;

	stk500_state.ss_prog_mode = STK500_PROG_MODE_NONE;
}

#ifdef DEBUG_ENABLED
static void
stk_shell_cmd(FILE *os, uint8_t argc, const char * const *argv)
{

	if (argc >= 1 && strcasecmp(argv[0], "debug") == 0) {
		if (argc >= 2) {
			if (strcasecmp(argv[1], "on") == 0)
				DEBUG_FLAG = 1;
			else
			if (strcasecmp(argv[1], "off") == 0)
				DEBUG_FLAG = 0;
		}
	}

	fprintf(os, "STK500v2 debug state: %s\n", (DEBUG_FLAG ? "on" : "off"));
}
#endif /* DEBUG_ENABLED */

void
proto_stk500v2_init(void)
{

#ifdef	DEBUG_ENABLED
	SHELL_CMD_ADD(stk);
#endif
}
