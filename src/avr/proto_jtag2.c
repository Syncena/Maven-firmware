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
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#endif
#include "programmer.h"
#include "platform.h"
#include "proto_jtag2.h"
#include "prog_isp.h"
#include "prog_pdi.h"
#include "prog_updi.h"
#include "network.h"
#include "rtos.h"
#include "ringbuff.h"
#include "shell.h"

#define	PROTO_JTAG2_DEBUG_OFF	0u
#define	PROTO_JTAG2_DEBUG_ON	1u
#define	PROTO_JTAG2_DEBUG_DUMP	2u

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	proto_jtag2_debug
#include "debug.h"

typedef void (*jtag2_command_t)(struct programmer_state *);
static void jtag2_cmd_sign_off(struct programmer_state *);
static void jtag2_cmd_get_sign_on(struct programmer_state *);
static void jtag2_cmd_set_parameter(struct programmer_state *);
static void jtag2_cmd_get_parameter(struct programmer_state *);
static void jtag2_cmd_write_memory(struct programmer_state *);
static void jtag2_cmd_read_memory(struct programmer_state *);
static void jtag2_cmd_go(struct programmer_state *);
static void jtag2_cmd_reset(struct programmer_state *);
static void jtag2_cmd_set_device_descriptor(struct programmer_state *);
static void jtag2_cmd_get_sync(struct programmer_state *);
static void jtag2_cmd_chip_erase(struct programmer_state *);
static void jtag2_cmd_enter_progmode(struct programmer_state *);
static void jtag2_cmd_leave_progmode(struct programmer_state *);
static void jtag2_cmd_isp_packet(struct programmer_state *);
static void jtag2_cmd_xmega_erase(struct programmer_state *);
static void jtag2_cmd_set_xmega_params(struct programmer_state *);
static void jtag2_cmd_ignore(struct programmer_state *);

static const jtag2_command_t jtag2_commands[] = {
	jtag2_cmd_sign_off,		/* 00 */
	jtag2_cmd_get_sign_on,		/* 01 */
	jtag2_cmd_set_parameter,	/* 02 */
	jtag2_cmd_get_parameter,	/* 03 */
	jtag2_cmd_write_memory,		/* 04 */
	jtag2_cmd_read_memory,		/* 05 */
	jtag2_cmd_ignore,		/* 06 */
	jtag2_cmd_ignore,		/* 07 */
	jtag2_cmd_go,			/* 08 */
	jtag2_cmd_ignore,		/* 09 */
	jtag2_cmd_ignore,		/* 0a */
	jtag2_cmd_reset,		/* 0b */
	jtag2_cmd_set_device_descriptor,/* 0c */
	jtag2_cmd_ignore,		/* 0d */
	NULL,				/* 0e */
	jtag2_cmd_get_sync,		/* 0f */
	jtag2_cmd_ignore,		/* 10 */
	jtag2_cmd_ignore,		/* 11 */
	jtag2_cmd_ignore,		/* 12 */
	jtag2_cmd_chip_erase,		/* 13 */
	jtag2_cmd_enter_progmode,	/* 14 */
	jtag2_cmd_leave_progmode,	/* 15 */
	jtag2_cmd_ignore,		/* 16 */
	NULL,				/* 17 */
	NULL,				/* 18 */
	NULL,				/* 19 */
	jtag2_cmd_ignore,		/* 1a */
	NULL,				/* 1b */
	jtag2_cmd_ignore,		/* 1c */
	jtag2_cmd_ignore,		/* 1d */
	NULL,				/* 1e */
	NULL,				/* 1f */
	NULL,				/* 20 */
	NULL,				/* 21 */
	jtag2_cmd_ignore,		/* 22 */
	jtag2_cmd_ignore,		/* 23 */
	jtag2_cmd_ignore,		/* 24 */
	jtag2_cmd_ignore,		/* 25 */
	NULL,				/* 26 */
	NULL,				/* 27 */
	jtag2_cmd_ignore,		/* 28 */
	jtag2_cmd_ignore,		/* 29 */
	NULL,				/* 2a */
	jtag2_cmd_ignore,		/* 2b */
	jtag2_cmd_ignore,		/* 2c */
	jtag2_cmd_ignore,		/* 2d */
	NULL,				/* 2e */
	jtag2_cmd_isp_packet,		/* 2f */
	NULL,				/* 30 */
	NULL,				/* 31 */
	NULL,				/* 32 */
	NULL,				/* 33 */
	jtag2_cmd_xmega_erase,		/* 34 */
	NULL,				/* 35 */
	jtag2_cmd_set_xmega_params,	/* 36 */
};
#define	JTAG2_NCOMMANDS	((sizeof(jtag2_commands) / sizeof(jtag2_commands[0])))

struct jtag2_header {
	uint8_t jh_start;
	uint16_t jh_sequence;	/* Note: Little-endian */
	uint32_t jh_len;	/* Note: Little-endian */
	uint8_t jh_token;
} __attribute__((__packed__));
#define JTAG2_MAGIC_START		0x1bu
#define JTAG2_MAGIC_TOKEN		0x0eu

/*
 * Based on Atmel App Note AVR0607, with tweaks to use explicitly-sized
 * types.
 *
 * We only use a couple of the fields of this descriptor.
 */
struct jtag2_device_descriptor {
	uint8_t ucReadIO[8];
	uint8_t ucReadIOShadow[8];
	uint8_t ucWriteIO[8];
	uint8_t ucWriteIOShadow[8];
	uint8_t ucReadExtIO[52];
	uint8_t ucReadIOExtShadow[52];
	uint8_t ucWriteExtIO[52];
	uint8_t ucWriteIOExtShadow[52];
	uint8_t ucIDRAddress;
	uint8_t ucSPMCRAddress;
	uint8_t ucRAMPZAddress;
	uint16_t uiFlashPageSize;
	uint8_t ucEepromPageSize;
	uint32_t ulBootAddress;	
	uint16_t uiUpperExtIOLoc;
	uint32_t ulFlashSize;
	uint8_t ucEepromInst[20];
	uint8_t ucFlashInst[3];
	uint8_t ucSPHaddr;
	uint8_t ucSPLaddr;
	uint16_t uiFlashpages;
	uint8_t ucDWDRAddress;
	uint8_t ucDWBasePC;
	uint8_t ucAllowFullPageBitstream;
	uint16_t uiStartSmallestBootLoaderSection;
	uint8_t EnablePageProgramming;
	uint8_t ucCacheType;
	uint16_t uiSramStartAddr;
	uint8_t ucResetType;
	uint8_t ucPCMaskExtended;
	uint8_t ucPCMaskHigh;
	uint8_t ucEindAddress;
	uint16_t EECRAddress;
} __attribute__((__packed__));

/*
 * Descriptor for XMega devices.
 */
struct jtag2_xmega_descriptor {
	uint32_t xd_offset_app;
	uint32_t xd_offset_boot;
	uint32_t xd_offset_eeprom;
	uint32_t xd_offset_fuse;
	uint32_t xd_offset_lock;
	uint32_t xd_offset_usig;
	uint32_t xd_offset_psig;
	uint32_t xd_offset_data;
	uint32_t xd_size_app_flash;
	uint16_t xd_size_boot_flash;
	uint16_t xd_size_flash_page;
	uint16_t xd_size_eeprom;
	uint8_t xd_size_eeprom_page;
	uint16_t xd_addr_nvm_ctrl;
	uint16_t xd_addr_mcu_ctrl;
} __attribute__((__packed__));

/*
 * Descriptor for UPID devices.
 */
struct jtag2_updi_descriptor {
	uint16_t ud_base_flash;
	uint8_t ud_size_flash_page;
	uint8_t ud_size_eeprom_page;
	uint16_t ud_addr_nvm_ctrl;
	uint16_t ud_addr_ocd_ctrl;
} __attribute__((__packed__));

/*
 * Command response codes.
 */
#define JTAG2_RESPONSE_OK			0x80u
#define JTAG2_RESPONSE_PARAMETER		0x81u
#define JTAG2_RESPONSE_MEMORY			0x82u
#define JTAG2_RESPONSE_SIGN_ON			0x86u
#define JTAG2_RESPONSE_SPI_DATA			0x88u
#define JTAG2_RESPONSE_FAILED			0xa0u
#define JTAG2_RESPONSE_ILLEGAL_PARAMETER	0xa1u
#define JTAG2_RESPONSE_ILLEGAL_EMULATOR_MODE	0xa4u
#define JTAG2_RESPONSE_ILLEGAL_MCU_STATE	0xa5u
#define JTAG2_RESPONSE_ILLEGAL_VALUE		0xa6u
#define JTAG2_RESPONSE_ILLEGAL_COMMAND		0xaau

/*
 * Supported parameters.
 */
#define JTAG2_PARAM_HW_VERSION			0x01u
#define JTAG2_PARAM_FW_VERSION			0x02u
#define JTAG2_PARAM_EMULATOR_MODE		0x03u
#define  JTAG2_EMULATOR_MODE_SPI		0x03u
#define  JTAG2_EMULATOR_MODE_PDI		0x06u
#define  JTAG2_EMULATOR_MODE_UPDI		0x07u
#define JTAG2_PARAM_BAUD_RATE			0x05u
#define  JTAG2_BAUD_DEFAULT			0x04u	/* 19200 baud */
#define JTAG2_PARAM_OCD_VTARGET			0x06u
#define JTAG2_PARAM_DAISY_CHAIN_INFO		0x1bu
#define	JTAG2_PARAM_UPDI_CHIP_ERASE_TO_ENTER	0x66u
#define	JTAG2_PARAM_UPDI_SIB			0x50u

struct jtag2_signon_reply {
	uint8_t so_proto_vers;
	uint8_t so_m_mcu_bootloader_version;
	uint16_t so_m_mcu_firmware_version;
	uint8_t so_m_mcu_hardware_version;
	uint8_t so_s_mcu_bootloader_version;
	uint16_t so_s_mcu_firmware_version;
	uint8_t so_s_mcu_hardware_version;
	uint8_t so_serial[6];
	char so_devid[8];
} __attribute__((__packed__));

static const struct jtag2_signon_reply jtag2_signon_reply_prototype = {
	.so_proto_vers = 1,
	.so_m_mcu_bootloader_version = 1,
	.so_m_mcu_firmware_version = 0x100,
	.so_m_mcu_hardware_version = 1,
	.so_s_mcu_bootloader_version = 1,
	.so_s_mcu_firmware_version = 0x718,
	.so_s_mcu_hardware_version = 1
};

struct jtag2_memory_cmd {
	uint8_t mc_type;
	uint32_t mc_length;
	uint32_t mc_address;
} __attribute__((__packed__));
#define JTAG2_MEM_TYPE_EEPROM		0x22u
#define JTAG2_MEM_TYPE_FLASH_PAGE	0xb0u
#define JTAG2_MEM_TYPE_EEPROM_PAGE	0xb1u
#define JTAG2_MEM_TYPE_FUSE_BITS	0xb2u
#define JTAG2_MEM_TYPE_LOCK_BITS	0xb3u
#define JTAG2_MEM_TYPE_SIGN_JTAG	0xb4u
#define JTAG2_MEM_TYPE_OSCCAL_BYTE	0xb5u
#define JTAG2_MEM_TYPE_FLASH		0xc0u
#define JTAG2_MEM_TYPE_BOOT_FLASH	0xc1u
#define JTAG2_MEM_TYPE_EEPROM_XMEGA	0xc4u
#define JTAG2_MEM_TYPE_USERSIG		0xc5u
#define JTAG2_MEM_TYPE_SIB		0xd3u

struct jtag2_xmega_erase_cmd {
	uint8_t xe_what;
	uint32_t xe_address;
} __attribute__((__packed__));
#define JTAG2_XMEGA_ERASE_CHIP		0x00u
#define JTAG2_XMEGA_ERASE_APP		0x01u
#define JTAG2_XMEGA_ERASE_BOOT		0x02u
#define JTAG2_XMEGA_ERASE_EEPROM	0x03u
#define JTAG2_XMEGA_ERASE_APP_PAGE	0x04u
#define JTAG2_XMEGA_ERASE_BOOT_PAGE	0x05u
#define JTAG2_XMEGA_ERASE_EEPROM_PAGE	0x06u
#define JTAG2_XMEGA_ERASE_USERSIG	0x07u

typedef enum {
	JTAG2_DESC_TYPE_NONE = 0,
	JTAG2_DESC_TYPE_XMEGA,
	JTAG2_DESC_TYPE_UPDI
} jtag2_desc_type_t;

struct jtag2_state {
	bool js_prog_mode_enabled;
	uint8_t js_emulator_mode;
	uint8_t js_baud_rate;
	uint8_t js_erase_to_enter;
	void *js_stk500v2_state;
	jtag2_desc_type_t js_desc_type;
	union {
		struct jtag2_device_descriptor js_dd;
		struct jtag2_xmega_descriptor js_xd;
		struct jtag2_updi_descriptor js_ud;
	};
};
static struct jtag2_state jtag2_state;

static uint32_t jtag2_process_header(struct programmer_state *);
static void jtag2_process_body(struct programmer_state *);
static void jtag2_dispatch_request(struct programmer_state *);
static void jtag2_connection_closed(struct programmer_state *);

const struct programmer_protocol proto_jtag2_protocol = {
	sizeof(struct jtag2_header),
	jtag2_process_header,
	jtag2_process_body,
	jtag2_dispatch_request,
	jtag2_connection_closed,
};

#ifdef DEBUG_ENABLED
SHELL_CMD_DECL(jtag2, jtag2_shell_cmd, "Configure debug state");
#endif

#if (RELEASE_BUILD == 0) || defined(DEBUG_ENABLED)
static const char *
jtag2_mode_string(struct jtag2_state *js)
{
	const char *rv;

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_SPI:
		rv = "SPI";
		break;
	case JTAG2_EMULATOR_MODE_PDI:
		rv = "PDI";
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		rv = "UPDI";
		break;
	default:
		rv = "???";
		break;
	}

	return rv;
}
#endif /* (RELEASE_BUILD == 0) || defined(DEBUG_ENABLED) */

static void
jtag2_cmd_sign_off(struct programmer_state *ps)
{

	DBFPRINTF("\n");
	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
}

static void
jtag2_cmd_get_sign_on(struct programmer_state *ps)
{
	struct jtag2_signon_reply so;
	struct jtag2_state *js;

	if (ps->ps_proto_state != NULL) {
		DBFPRINTF("already signed on!\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_ILLEGAL_COMMAND);
		return;
	}

	ps->ps_proto_state = js = &jtag2_state;
	memset(js, 0, sizeof(*js));

	js->js_erase_to_enter = 0;
	js->js_emulator_mode = JTAG2_EMULATOR_MODE_PDI;
	js->js_baud_rate = JTAG2_BAUD_DEFAULT;

	so = jtag2_signon_reply_prototype;
	network_get_mac(so.so_serial);

	switch (programmer_get_proto()) {
	default:
		strlcpy(so.so_devid, PROGRAMMER_DEV_ID, sizeof(so.so_devid));
		break;

	case PROG_PROTO_DRAGON:
		strlcpy(so.so_devid, "DRAGON", sizeof(so.so_devid));
		break;

	case PROG_PROTO_WAVR:
		strlcpy(so.so_devid, "WAVR", sizeof(so.so_devid));
		break;
	}

	programmer_reply_add_byte(ps, JTAG2_RESPONSE_SIGN_ON);
	programmer_reply_add_buff(ps, &so, sizeof(so));

	DBFPRINTF("signed on\n");
}

static uint8_t
jtag2_par_set_emulator_mode(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	uint8_t mode, rv;

	if (programmer_request_get_byte(ps, &mode) != 1) {
		DBFPRINTF("no mode!\n");
		return JTAG2_RESPONSE_FAILED;
	}

	DBFPRINTF("%u\n", mode);

	switch (mode) {
	case JTAG2_EMULATOR_MODE_PDI:
	case JTAG2_EMULATOR_MODE_UPDI:
	case JTAG2_EMULATOR_MODE_SPI:
		js->js_emulator_mode = mode;
		rv = JTAG2_RESPONSE_OK;
		break;

	default:
		rv = JTAG2_RESPONSE_ILLEGAL_EMULATOR_MODE;
		break;
	}

	return rv;
}

static void
jtag2_cmd_set_parameter(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	uint8_t par, rv;

	if (programmer_request_get_byte(ps, &par) != 1) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		DBFPRINTF("no par!\n");
		return;
	}

	DBFPRINTF("par %u\n", par);

	switch (par) {
	case JTAG2_PARAM_EMULATOR_MODE:
		rv = jtag2_par_set_emulator_mode(ps);
		break;

	case JTAG2_PARAM_BAUD_RATE:
		if (programmer_request_get_byte(ps, &js->js_baud_rate) != 1)
			rv = JTAG2_RESPONSE_ILLEGAL_PARAMETER;
		else
			rv = JTAG2_RESPONSE_OK;
		break;

	case JTAG2_PARAM_UPDI_CHIP_ERASE_TO_ENTER:
		DBFPRINTF("erase_to_enter\n");
		if (programmer_request_get_byte(ps, &js->js_erase_to_enter)
		    != 1) {
			DBFPRINTF("failed\n");
			rv = JTAG2_RESPONSE_ILLEGAL_PARAMETER;
		} else {
			DBFPRINTF("erase_to_enter %u\n", js->js_erase_to_enter);
			rv = JTAG2_RESPONSE_OK;
		}
		break;

	case JTAG2_PARAM_DAISY_CHAIN_INFO:
		/* Just ignore this. */
		rv = JTAG2_RESPONSE_OK;
		break;

	default:
		/* No other parameters permitted. */
		rv = JTAG2_RESPONSE_ILLEGAL_PARAMETER;
		break;
	}

	programmer_reply_add_byte(ps, rv);
}

static void
jtag2_cmd_get_parameter(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_updi_read_arg ura;
	union {
		uint8_t sib[16];
		uint8_t u8[4];
		uint16_t u16[2];
	} u;
	uint32_t len;
	uint8_t par, rv;
	fix16_t tvcc;

	if (programmer_request_get_byte(ps, &par) != 1) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		DBFPRINTF("no par!\n");
		return;
	}

	DBFPRINTF("par %u\n", par);

	rv = JTAG2_RESPONSE_PARAMETER;
	len = 0;

	switch (par) {
	case JTAG2_PARAM_EMULATOR_MODE:
		u.u8[0] = js->js_emulator_mode;
		len = 1;
		break;

	case JTAG2_PARAM_HW_VERSION:
		u.u8[0] = jtag2_signon_reply_prototype.so_m_mcu_hardware_version;
		u.u8[1] = jtag2_signon_reply_prototype.so_s_mcu_hardware_version;
		len = 2;
		break;

	case JTAG2_PARAM_FW_VERSION:
		u.u16[0] = jtag2_signon_reply_prototype.so_m_mcu_firmware_version;
		u.u16[1] = jtag2_signon_reply_prototype.so_s_mcu_firmware_version;
		len = 4;
		break;

	case JTAG2_PARAM_BAUD_RATE:
		u.u8[0] = js->js_baud_rate;
		len = 1;
		break;

	case JTAG2_PARAM_OCD_VTARGET:
		tvcc = fix16_mul(power_target_vcc, F16(10.0));
		u.u16[0] = (uint16_t) fix16_to_int(tvcc);
		len = 2;
		break;

	case JTAG2_PARAM_UPDI_SIB:
		ura.ura_mem_type = PROG_UPDI_MEM_TYPE_SIB;
		ura.ura_length = 16;
		ura.ura_address = 0;
		ura.ura_buffer = u.sib;
		if (prog_updi_read(ps->ps_updi, &ura) != PROG_UPDI_STATUS_OK)
			rv = JTAG2_RESPONSE_FAILED;
		break;

	default:
		rv = JTAG2_RESPONSE_ILLEGAL_PARAMETER;
		break;
	}

	programmer_reply_add_byte(ps, rv);
	if (len)
		programmer_reply_add_buff(ps, &u, len);
}

static int
jtag2_mtype_to_pdi_mem(struct jtag2_state *js, uint8_t mt, uint8_t *pmt,
    bool *pmode, uint16_t *page_size, uint32_t *address)
{
	uint32_t addr_off;
	uint16_t psize;
	uint8_t mem_type;
	bool page_mode;
	int rv;

	addr_off = 0;
	psize = 1;
	page_mode = false;
	rv = 0;

	/*
	 * XXX: This is gnarly. Please fix.
	 */

	if (js->js_desc_type != JTAG2_DESC_TYPE_NONE &&
	    js->js_desc_type != JTAG2_DESC_TYPE_XMEGA) {
		return -1;
	}

	switch (mt) {
	case JTAG2_MEM_TYPE_EEPROM_PAGE:
	case JTAG2_MEM_TYPE_EEPROM:
	case JTAG2_MEM_TYPE_EEPROM_XMEGA:
		page_mode = true;
		mem_type = PROG_PDI_MEM_TYPE_EEPROM;
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA) {
			psize = js->js_xd.xd_size_eeprom_page;
			addr_off = js->js_xd.xd_offset_eeprom;
		} else {
			psize = js->js_dd.ucEepromPageSize;
		}
		break;

	case JTAG2_MEM_TYPE_FLASH_PAGE:
	case JTAG2_MEM_TYPE_FLASH:
		page_mode = true;
		mem_type = PROG_PDI_MEM_TYPE_APPLICATION;
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA) {
			psize = js->js_xd.xd_size_flash_page;
			addr_off = js->js_xd.xd_offset_app;
		} else {
			psize = js->js_dd.uiFlashPageSize;
		}
		break;

	case JTAG2_MEM_TYPE_BOOT_FLASH:
		mem_type = PROG_PDI_MEM_TYPE_BOOTLOADER;
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA) {
			psize = js->js_xd.xd_size_flash_page;
			addr_off = js->js_xd.xd_offset_boot;
		} else {
			psize = js->js_dd.uiFlashPageSize;
		}
		break;

	case JTAG2_MEM_TYPE_FUSE_BITS:
		mem_type = PROG_PDI_MEM_TYPE_FUSE;
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA)
			addr_off = js->js_xd.xd_offset_fuse;
		break;

	case JTAG2_MEM_TYPE_LOCK_BITS:
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA)
			addr_off = js->js_xd.xd_offset_lock;
		mem_type = PROG_PDI_MEM_TYPE_LOCKBITS;
		break;

	case JTAG2_MEM_TYPE_USERSIG:
	case JTAG2_MEM_TYPE_SIGN_JTAG:
		if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA)
			addr_off = js->js_xd.xd_offset_usig;
		mem_type = PROG_PDI_MEM_TYPE_USER_SIGNATURE;
		break;

	case JTAG2_MEM_TYPE_OSCCAL_BYTE:
		mem_type = PROG_PDI_MEM_TYPE_FACTORY_CALIB;
		break;

	default:
		mem_type = 0;
		rv = -1;
		break;
	}

	*pmt = mem_type;
	if (pmode != NULL)
		*pmode = page_mode;
	if (page_size != NULL)
		*page_size = psize;
	if (address != NULL && *address < addr_off)
		*address += addr_off;

	return rv;
}

static int
jtag2_mtype_to_updi_mem(struct jtag2_state *js, uint8_t mt, uint8_t *pmt,
    bool *pmode, uint8_t *page_size, uint32_t *address)
{
	uint32_t addr_off;
	uint16_t psize;
	uint8_t mem_type;
	bool page_mode;
	int rv;

	addr_off = 0;
	psize = 1;
	page_mode = false;
	rv = 0;

	/*
	 * XXX: This is also gnarly. Please fix.
	 */

	if (js->js_desc_type != JTAG2_DESC_TYPE_UPDI)
		return -1;

	switch (mt) {
	case JTAG2_MEM_TYPE_EEPROM_PAGE:
	case JTAG2_MEM_TYPE_EEPROM:
		page_mode = true;
		mem_type = PROG_UPDI_MEM_TYPE_EEPROM;
		psize = js->js_ud.ud_size_eeprom_page;
		break;

	case JTAG2_MEM_TYPE_FLASH_PAGE:
	case JTAG2_MEM_TYPE_FLASH:
	case JTAG2_MEM_TYPE_BOOT_FLASH:
		page_mode = true;
		mem_type = PROG_UPDI_MEM_TYPE_FLASH;
		psize = js->js_ud.ud_size_flash_page;
		addr_off = js->js_ud.ud_base_flash;
		break;

	case JTAG2_MEM_TYPE_FUSE_BITS:
		mem_type = PROG_UPDI_MEM_TYPE_FUSE;
		break;

	case JTAG2_MEM_TYPE_LOCK_BITS:
		mem_type = PROG_UPDI_MEM_TYPE_LOCKBITS;
		break;

	case JTAG2_MEM_TYPE_USERSIG:
		mem_type = PROG_UPDI_MEM_TYPE_USERROW;
		break;

	case JTAG2_MEM_TYPE_SIGN_JTAG:
		mem_type = PROG_UPDI_MEM_TYPE_SIGROW;
		addr_off = 0;
		break;

	case JTAG2_MEM_TYPE_SIB:
		mem_type = PROG_UPDI_MEM_TYPE_SIB;
		addr_off = 0;
		break;

	default:
		mem_type = 0;
		rv = -1;
		break;
	}

	*pmt = mem_type;
	if (pmode != NULL)
		*pmode = page_mode;
	if (page_size != NULL)
		*page_size = psize;
	if (address != NULL && *address < addr_off)
		*address += addr_off;

	return rv;
}

static void
jtag2_dump_buff(const char *title, const uint8_t *data, uint16_t length)
{
#ifdef DEBUG_ENABLED
	uint16_t i;

	if ((DEBUG_FLAG & PROTO_JTAG2_DEBUG_DUMP) == 0)
		return;

	if (length > 8)
		length = 8;

	DBPRINTF("%s:", title);

	for (i = 0; i < length; i++) {
		if ((i % 24) == 0)
			DBPRINTF("\n%03x:", i);
		DBPRINTF(" %02x", data[i]);
	}

	DBPRINTF("\n");
#else
	(void) title;
	(void) data;
	(void) length;
#endif
}

static void
jtag2_write_memory_pdi(struct programmer_state *ps,
    struct jtag2_memory_cmd *mc)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_pdi_program_arg ppa;
	uint16_t page_size;
	bool pmode;

	ppa.ppa_address = mc->mc_address;
	if (jtag2_mtype_to_pdi_mem(js, mc->mc_type, &ppa.ppa_mem_type, &pmode,
	    &page_size, &ppa.ppa_address) < 0) {
		DBFPRINTF("bad memory type\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	ppa.ppa_page_mode = pmode ? PROG_PDI_PAGEMODE_WRITE : 0;
	ppa.ppa_length = (uint16_t)mc->mc_length;
	ppa.ppa_data = &ps->ps_request[ps->ps_request_cursor];

	jtag2_dump_buff("Program", ppa.ppa_data, ppa.ppa_length);

	if (prog_pdi_program(ps->ps_pdi, &ppa) == PROG_PDI_STATUS_OK)
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	else {
		prog_pdi_reset_comms(ps->ps_pdi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_write_memory_updi(struct programmer_state *ps,
    struct jtag2_memory_cmd *mc)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_updi_program_arg upa;
	uint32_t address;
	uint8_t page_size;

	address = mc->mc_address;
	if (jtag2_mtype_to_updi_mem(js, mc->mc_type, &upa.upa_mem_type, NULL,
	    &page_size, &address) < 0) {
		DBFPRINTF("bad memory type\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	upa.upa_mode = 0;
	upa.upa_length = (uint16_t)mc->mc_length;
	upa.upa_address = (uint16_t)address;
	upa.upa_data = &ps->ps_request[ps->ps_request_cursor];

	jtag2_dump_buff("Program", upa.upa_data, upa.upa_length);

	if (prog_updi_program(ps->ps_updi, &upa) == PROG_UPDI_STATUS_OK)
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	else {
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_cmd_write_memory(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct jtag2_memory_cmd mc;

	if (programmer_request_get_buff(ps, &mc, sizeof(mc)) != sizeof(mc)) {
		DBFPRINTF("read failed\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	DBFPRINTF("type %02x, length %lu, addr %08lx\n", mc.mc_type,
	    mc.mc_length, mc.mc_address);

	if (mc.mc_length == 0 || mc.mc_length > (ps->ps_request_len -
	    ps->ps_request_cursor)) {
		DBFPRINTF("bad request size (%lu)\n", mc.mc_length);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		jtag2_write_memory_pdi(ps, &mc);
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		jtag2_write_memory_updi(ps, &mc);
		break;
	default:
		DBFPRINTF("not in PDI mode\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_read_memory_pdi(struct programmer_state *ps,
    struct jtag2_memory_cmd *mc)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_pdi_read_arg pra;

	pra.pra_address = mc->mc_address;
	if (jtag2_mtype_to_pdi_mem(js, mc->mc_type, &pra.pra_mem_type,
	    NULL, NULL, &pra.pra_address) < 0) {
		DBFPRINTF("unsupported memory type\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	pra.pra_length = mc->mc_length;
	pra.pra_buffer = &ps->ps_reply[ps->ps_reply_len + 1];

	if (prog_pdi_read(ps->ps_pdi, &pra) == PROG_PDI_STATUS_OK) {
		jtag2_dump_buff("Read", pra.pra_buffer, pra.pra_length);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_MEMORY);
		ps->ps_reply_len += mc->mc_length;
	} else {
		prog_pdi_reset_comms(ps->ps_pdi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_read_memory_updi(struct programmer_state *ps,
    struct jtag2_memory_cmd *mc)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_updi_read_arg ura;
	uint32_t address;

	address = mc->mc_address;
	if (jtag2_mtype_to_updi_mem(js, mc->mc_type, &ura.ura_mem_type,
	    NULL, NULL, &address) < 0) {
		DBFPRINTF("unsupported memory type\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	ura.ura_length = (uint16_t)mc->mc_length;
	ura.ura_address = (uint16_t)address;
	ura.ura_buffer = &ps->ps_reply[ps->ps_reply_len + 1];

	if (prog_updi_read(ps->ps_updi, &ura) == PROG_UPDI_STATUS_OK) {
		jtag2_dump_buff("Read", ura.ura_buffer, ura.ura_length);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_MEMORY);
		ps->ps_reply_len += mc->mc_length;
	} else {
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_cmd_read_memory(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct jtag2_memory_cmd mc;

	if (programmer_request_get_buff(ps, &mc, sizeof(mc)) != sizeof(mc)) {
		DBFPRINTF("read failed\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	DBFPRINTF("type %02x, length %lu, addr %08lx\n", mc.mc_type,
	    mc.mc_length, mc.mc_address);

	if (mc.mc_length > (sizeof(ps->ps_reply) -
	    (sizeof(struct jtag2_header) + sizeof(uint16_t)))) {
		DBFPRINTF("request too big (%lu)\n", mc.mc_length);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		jtag2_read_memory_pdi(ps, &mc);
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		jtag2_read_memory_updi(ps, &mc);
		break;
	default:
		DBFPRINTF("not in PDI mode\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_cmd_ignore(struct programmer_state *ps)
{

	(void) ps;

	DBFPRINTF("\n");
}

static void
jtag2_cmd_go(struct programmer_state *ps)
{

	DBFPRINTF("\n");
	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
}

static void
jtag2_cmd_reset(struct programmer_state *ps)
{
	uint8_t flags;

	if (programmer_request_get_byte(ps, &flags) != 1) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		DBFPRINTF("no flags!\n");
		return;
	}

	DBFPRINTF("flags 0x%02x\n", flags);

	/* Ignore reset for now */

	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
}

static void
jtag2_cmd_set_device_descriptor(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	uint32_t len;

	DBFPRINTF("len %u\n", sizeof(js->js_dd));

	len = programmer_request_get_buff(ps, &js->js_dd, sizeof(js->js_dd));
	if (len != sizeof(js->js_dd)) {
		DBFPRINTF("read error.%lu != %u\n", len, sizeof(js->js_dd));
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	js->js_desc_type = JTAG2_DESC_TYPE_NONE;

	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
}

static void
jtag2_cmd_get_sync(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_UPDI:
		if (prog_updi_get_sync(ps->ps_updi) == 0)
			programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		else
			programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;

	case JTAG2_EMULATOR_MODE_SPI:
	case JTAG2_EMULATOR_MODE_PDI:
		/*
		 * I don't think we need actually do anything here.
		 * At the very least, might have to put the target
		 * into reset. But not for now.
		 */
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		break;

	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_cmd_chip_erase(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct prog_updi_erase_arg uea;

	DBFPRINTF("mode %s\n", jtag2_mode_string(js));

	if (js->js_emulator_mode != JTAG2_EMULATOR_MODE_UPDI) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	uea.uea_address = 0;
	uea.uea_mem_type = PROG_UPDI_ERASE_DEVICE;

	if (prog_updi_erase(ps->ps_updi, &uea) == PROG_UPDI_STATUS_OK) {
		DBFPRINTF("erase succeeded\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	} else {
		DBFPRINTF("erase failed\n");
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_enter_progmode_pdi(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (js->js_desc_type == JTAG2_DESC_TYPE_XMEGA) {
		union prog_pdi_param_arg ppa;

		ppa.ppa_nvm_base = js->js_xd.xd_addr_nvm_ctrl +
		    js->js_xd.xd_offset_data;
		prog_pdi_set_param(ps->ps_pdi, PROG_PDI_PARAM_NVMBASE, &ppa);

		ppa.ppa_ee_page_size = js->js_xd.xd_size_eeprom_page;
		prog_pdi_set_param(ps->ps_pdi, PROG_PDI_PARAM_EEPAGESIZE, &ppa);
	} else {
		/*
		 * Note sure what to do here. In theory we should never be in
		 * PDI mode without an XMega descriptor...
		 */
		DBFPRINTF("no XMega descriptor\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	if (prog_pdi_enter_prog_mode(ps->ps_pdi) == PROG_PDI_STATUS_OK){
		js->js_prog_mode_enabled = true;
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	} else {
		prog_pdi_reset_comms(ps->ps_pdi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_enter_progmode_updi(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (js->js_desc_type == JTAG2_DESC_TYPE_UPDI) {
		struct prog_updi_param_arg upa;

		upa.upa_flash_base = js->js_ud.ud_base_flash;
		upa.upa_nvm_addr = js->js_ud.ud_addr_nvm_ctrl;
		upa.upa_ocd_addr = js->js_ud.ud_addr_ocd_ctrl;
		upa.upa_flash_page_size = js->js_ud.ud_size_flash_page;
		upa.upa_ee_page_size = js->js_ud.ud_size_eeprom_page;

		prog_updi_set_param(ps->ps_updi, &upa);
	} else {
		/*
		 * Note sure what to do here. In theory we should never be in
		 * PDI mode without an XMega descriptor...
		 */
		DBFPRINTF("no XMega descriptor\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	if (prog_updi_enter_prog_mode(ps->ps_updi) == PROG_UPDI_STATUS_OK) {
		js->js_prog_mode_enabled = true;
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	} else {
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_cmd_enter_progmode(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	DBFPRINTF("mode %s\n", jtag2_mode_string(js));

	/* Ignore duplicates */
	if (js->js_prog_mode_enabled) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		return;
	}

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		jtag2_enter_progmode_pdi(ps);
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		jtag2_enter_progmode_updi(ps);
		break;
	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_leave_progmode_pdi(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (prog_pdi_leave_prog_mode(ps->ps_pdi) == PROG_PDI_STATUS_OK) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		js->js_prog_mode_enabled = false;
	} else {
		prog_pdi_reset_comms(ps->ps_pdi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_leave_progmode_updi(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (prog_updi_leave_prog_mode(ps->ps_updi) == PROG_PDI_STATUS_OK) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		js->js_prog_mode_enabled = false;
	} else {
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_cmd_leave_progmode(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	DBFPRINTF("mode %s\n", jtag2_mode_string(js));

	/* Ignore duplicates */
	if (js->js_prog_mode_enabled == false) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
		return;
	}

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		jtag2_leave_progmode_pdi(ps);
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		jtag2_leave_progmode_updi(ps);
		break;
	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_cmd_isp_packet(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	uint16_t reply_len, post_len;
	uint32_t pre_len;

	/*
	 * These are simply STK500v2 commands with a JTAGMKII wrapper.
	 * Two bytes following the wrapper command byte are the length
	 * of the STK500v2 command's reply, in little-endian format.
	 */
	if (programmer_request_get_buff(ps, &reply_len, sizeof(reply_len)) !=
	    sizeof(reply_len)) {
		DBFPRINTF("read reply_len failed\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	DBFPRINTF("expected reply length %u\n", reply_len);

	if (js->js_emulator_mode != JTAG2_EMULATOR_MODE_SPI) {
		DBFPRINTF("Not in SPI mode\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_ILLEGAL_MCU_STATE);
		return;
	}

	/*
	 * Responses are prefixed with JTAG2_RESPONSE_SPI_DATA.
	 */
	programmer_reply_add_byte(ps, JTAG2_RESPONSE_SPI_DATA);

	pre_len = ps->ps_reply_len;

	/*
	 * Let the stk500v2 protocol do the rest, but must save and
	 * restore stk500v2's state.
	 */
	/* Restore stk500v2's state */
	ps->ps_proto_state = js->js_stk500v2_state;
	stk500_dispatch_request_from_jtag2(ps);

	/* Save stk500v2 state and restore ours */
	js->js_stk500v2_state = ps->ps_proto_state;
	ps->ps_proto_state = js;

	post_len = (uint16_t)(ps->ps_reply_len - pre_len);

	DBFPRINTF("actual reply length %u\n", post_len);

	if (post_len != reply_len) {
		/*
		 * Should we fail this?
		 */
		DBFPRINTF("WARNING! LENGTH MISMATCH!\n");
	}
}

static void
jtag2_xmega_erase(struct programmer_state *ps,
    const struct jtag2_xmega_erase_cmd *xe)
{
	struct prog_pdi_erase_arg pea;

	pea.pea_address = xe->xe_address;

	switch (xe->xe_what) {
	case JTAG2_XMEGA_ERASE_CHIP:
		pea.pea_mem_type = PROG_PDI_ERASE_DEVICE;
		break;
	case JTAG2_XMEGA_ERASE_APP:
		pea.pea_mem_type = PROG_PDI_ERASE_APPLICATION;
		break;
	case JTAG2_XMEGA_ERASE_BOOT:
		pea.pea_mem_type = PROG_PDI_ERASE_BOOTLOADER;
		break;
	case JTAG2_XMEGA_ERASE_EEPROM:
		pea.pea_mem_type = PROG_PDI_ERASE_EEPROM;
		break;
	case JTAG2_XMEGA_ERASE_APP_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_APPLICATION_PAGE;
		break;
	case JTAG2_XMEGA_ERASE_BOOT_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_BOOTLOADER_PAGE;
		break;
	case JTAG2_XMEGA_ERASE_EEPROM_PAGE:
		pea.pea_mem_type = PROG_PDI_ERASE_EEPROM_PAGE;
		break;
	case JTAG2_XMEGA_ERASE_USERSIG:
		pea.pea_mem_type = PROG_PDI_ERASE_USER_SIGNATURE;
		break;
	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	if (prog_pdi_erase(ps->ps_pdi, &pea) == PROG_PDI_STATUS_OK)
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	else {
		prog_pdi_reset_comms(ps->ps_pdi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_updi_erase(struct programmer_state *ps,
    const struct jtag2_xmega_erase_cmd *xe)
{
	struct prog_updi_erase_arg uea;

	uea.uea_address = (uint16_t)xe->xe_address;

	DBFPRINTF("type %u, address %04x\n", xe->xe_what, uea.uea_address);

	switch (xe->xe_what) {
	case JTAG2_XMEGA_ERASE_CHIP:
	case JTAG2_XMEGA_ERASE_APP:
	case JTAG2_XMEGA_ERASE_BOOT:
		uea.uea_mem_type = PROG_UPDI_ERASE_DEVICE;
		break;
	case JTAG2_XMEGA_ERASE_EEPROM:
		uea.uea_mem_type = PROG_UPDI_ERASE_EEPROM;
		break;
	case JTAG2_XMEGA_ERASE_USERSIG:
		uea.uea_mem_type = PROG_UPDI_ERASE_USERSIG;
		break;
	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	if (prog_updi_erase(ps->ps_updi, &uea) == PROG_UPDI_STATUS_OK) {
		DBFPRINTF("erase succeeded\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);
	} else {
		DBFPRINTF("erase failed\n");
		prog_updi_reset_comms(ps->ps_updi);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
	}
}

static void
jtag2_cmd_xmega_erase(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	struct jtag2_xmega_erase_cmd xe;

	DBFPRINTF("mode %s\n", jtag2_mode_string(js));

	if (programmer_request_get_buff(ps, &xe, sizeof(xe)) != sizeof(xe)) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		DBFPRINTF("read failed\n");
		return;
	}

	DBFPRINTF("type %02x, addr %08lx\n", xe.xe_what, xe.xe_address);

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		jtag2_xmega_erase(ps, &xe);
		break;
	case JTAG2_EMULATOR_MODE_UPDI:
		jtag2_updi_erase(ps, &xe);
		break;
	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		break;
	}
}

static void
jtag2_set_xmega_params(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (programmer_request_get_buff(ps, &js->js_xd, sizeof(js->js_xd)) !=
	    sizeof(js->js_xd)) {
		DBFPRINTF("read error\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	js->js_desc_type = JTAG2_DESC_TYPE_XMEGA;
	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);

	DBFPRINTF("\n");
	DBPRINTF("\txmega:      xd_offset_app %08lx\n",
	    js->js_xd.xd_offset_app);
	DBPRINTF("\txmega:      xd_offset_boot %08lx\n",
	    js->js_xd.xd_offset_boot);
	DBPRINTF("\txmega:    xd_offset_eeprom %08lx\n",
	    js->js_xd.xd_offset_eeprom);
	DBPRINTF("\txmega:      xd_offset_fuse %08lx\n",
	    js->js_xd.xd_offset_fuse);
	DBPRINTF("\txmega:      xd_offset_lock %08lx\n",
	    js->js_xd.xd_offset_lock);
	DBPRINTF("\txmega:      xd_offset_usig %08lx\n",
	    js->js_xd.xd_offset_usig);
	DBPRINTF("\txmega:      xd_offset_psig %08lx\n",
	    js->js_xd.xd_offset_psig);
	DBPRINTF("\txmega:      xd_offset_data %08lx\n",
	    js->js_xd.xd_offset_data);
	DBPRINTF("\txmega:   xd_size_app_flash %08lx\n",
	    js->js_xd.xd_size_app_flash);
	DBPRINTF("\txmega:  xd_size_boot_flash %04x\n",
	    js->js_xd.xd_size_boot_flash);
	DBPRINTF("\txmega:  xd_size_flash_page %04x\n",
	    js->js_xd.xd_size_flash_page);
	DBPRINTF("\txmega:      xd_size_eeprom %04x\n",
	    js->js_xd.xd_size_eeprom);
	DBPRINTF("\txmega: xd_size_eeprom_page %02x\n",
	    js->js_xd.xd_size_eeprom_page);
	DBPRINTF("\txmega:    xd_addr_nvm_ctrl %04x\n",
	    js->js_xd.xd_addr_nvm_ctrl);
	DBPRINTF("\txmega:    xd_addr_mcu_ctrl %04x\n",
	    js->js_xd.xd_addr_mcu_ctrl);
}

static void
jtag2_set_updi_params(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;

	if (programmer_request_get_buff(ps, &js->js_ud, sizeof(js->js_ud)) !=
	    sizeof(js->js_ud)) {
		DBFPRINTF("read error\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	js->js_desc_type = JTAG2_DESC_TYPE_UPDI;
	programmer_reply_add_byte(ps, JTAG2_RESPONSE_OK);

	DBFPRINTF("\n");
	DBPRINTF("\tupdi:       ud_base_flash %04x\n",
	    js->js_ud.ud_base_flash);
	DBPRINTF("\tupdi:    ud_addr_nvm_ctrl %04x\n",
	    js->js_ud.ud_addr_nvm_ctrl);
	DBPRINTF("\tupdi:    ud_addr_ocd_ctrl %04x\n",
	    js->js_ud.ud_addr_ocd_ctrl);
	DBPRINTF("\tupdi:  ud_size_flash_page %02x\n",
	    js->js_ud.ud_size_flash_page);
	DBPRINTF("\tupdi: ud_size_eeprom_page %02x\n",
	    js->js_ud.ud_size_eeprom_page);
}

static void
jtag2_cmd_set_xmega_params(struct programmer_state *ps)
{
	struct jtag2_state *js = ps->ps_proto_state;
	uint16_t magic;
	uint8_t len;

	if (programmer_request_get_buff(ps, &magic, 2) != 2 ||
	    programmer_request_get_byte(ps, &len) != 1) {
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	DBFPRINTF("magic %04x, len %u\n", magic,len);

	switch (js->js_emulator_mode) {
	case JTAG2_EMULATOR_MODE_PDI:
		if (len != sizeof(js->js_xd)) {
			DBFPRINTF("PDI len != %u\n", sizeof(js->js_xd));
			programmer_reply_add_byte(ps,
			    JTAG2_RESPONSE_ILLEGAL_VALUE);
			return;
		}
		jtag2_set_xmega_params(ps);
		break;

	case JTAG2_EMULATOR_MODE_UPDI:
		if (len != sizeof(js->js_ud)) {
			DBFPRINTF("UPDI len != %u\n", sizeof(js->js_ud));
			programmer_reply_add_byte(ps,
			    JTAG2_RESPONSE_ILLEGAL_VALUE);
			return;
		}
		jtag2_set_updi_params(ps);
		break;

	default:
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		DBFPRINTF("wrong emulator mode\n");
		break;
	}
}

static void
jtag2_dispatch_request(struct programmer_state *ps)
{
	jtag2_command_t pc;
	uint8_t cmd;

	if (programmer_request_get_byte(ps, &cmd) != 1) {
		DBFPRINTF("failed to get cmd byte\n");
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_FAILED);
		return;
	}

	if (ps->ps_proto_state == NULL && cmd != 0x01u) {
		DBFPRINTF("cmd %02x but not signed on\n", cmd);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_ILLEGAL_COMMAND);
		return;
	}

	if (cmd < JTAG2_NCOMMANDS) {
		pc = jtag2_commands[cmd];
		if (pc == NULL) {
			DBFPRINTF("bad cmd %02x\n", cmd);
			programmer_reply_add_byte(ps,
			    JTAG2_RESPONSE_ILLEGAL_COMMAND);
		} else {
			(pc)(ps);
		}
	} else {
		DBFPRINTF("cmd %02x out of range\n",cmd);
		programmer_reply_add_byte(ps, JTAG2_RESPONSE_ILLEGAL_COMMAND);
	}
}

static uint16_t
jtag2_crc16(const void *p, uint32_t len, uint16_t crc)
{
	static const uint16_t _crc16[256] = {
		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
		0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
		0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
		0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
		0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
		0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
		0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
		0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
		0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
		0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
		0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
		0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
		0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
		0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
		0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
		0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
		0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
		0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
		0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
	};
	const uint8_t *buff = p;

	while (len--)
		crc = (crc >> 8) ^ _crc16[(crc ^ *buff++) & 0xffu];

	return crc;
}

static uint32_t
jtag2_process_header(struct programmer_state *ps)
{
	struct jtag2_header *jh;
	uint32_t len;

	jh = (struct jtag2_header *) ps->ps_request;

	/* Verify the magic values */
	if (jh->jh_start != JTAG2_MAGIC_START ||
	    jh->jh_token != JTAG2_MAGIC_TOKEN) {
		DBFPRINTF("start/token mismatch\n");
		return 0;
	}

	/*
	 * Copy the header into the reply buffer for later
	 */
	if (programmer_reply_add_buff(ps, jh, sizeof(*jh)) != sizeof(*jh)) {
		DBFPRINTF("failed to back-copy header\n");
		return 0;
	}

	/*
	 * Let the caller know the body size, including CRC16. Note that
	 * a bogus length (zero or > sizeof(ps->ps_request)) will be
	 * handled by the caller.
	 */
	len = jh->jh_len + 2;

	return len;
}

static void
jtag2_process_body(struct programmer_state *ps)
{
	struct jtag2_header *jh;
	uint32_t len;
	uint16_t msgcrc, crc;

	jh = (struct jtag2_header *) ps->ps_reply;
	len = jh->jh_len;

	if (len != (ps->ps_request_len - 2)) {
		DBFPRINTF("len %lu != reply %lu\n", len, ps->ps_request_len-2);
		ps->ps_reply_len = 0;
		return;
	}

	/* Compute local CRC16 over header and body */
	msgcrc = jtag2_crc16(jh, sizeof(*jh), 0xffffu);
	msgcrc = jtag2_crc16(ps->ps_request, len, msgcrc);

	/* Get the CRC sent by the host */
	crc = ps->ps_request[len + 1];
	crc <<= 8;
	crc |= ps->ps_request[len];

	/* Proceed only if CRCs match */
	if (msgcrc != crc) {
		DBFPRINTF("bad crc (%04x != %04x)\n", msgcrc, crc);
		/* Don't reply if the CRC is bad */
		ps->ps_reply_len = 0;
		return;
	}

	/* Strip the trailing CRC16 */
	ps->ps_request_len -= 2;

	/* Decode the request */
	jtag2_dispatch_request(ps);

	/*
	 * Send the response, if any
	 */
	if (ps->ps_reply_len <= sizeof(*jh)) {
		DBFPRINTF("bogus reply length!\n");
		ps->ps_reply_len = 0;
		return;
	}

	/* Tweak the header and compute the new CRC */
	len = ps->ps_reply_len;
	jh->jh_len = len - sizeof(*jh);
	crc = jtag2_crc16(ps->ps_reply, len, 0xffffu);
	programmer_reply_add_buff(ps, &crc, sizeof(crc));
}

static void
jtag2_connection_closed(struct programmer_state *ps)
{
	struct jtag2_state *js = &jtag2_state;

	if (ps->ps_proto_state == js) {
		if (js->js_stk500v2_state) {
			ps->ps_proto_state = js->js_stk500v2_state;
			js->js_stk500v2_state = NULL;
			proto_stk500v2_protocol.pp_closed(ps);
		}

		ps->ps_proto_state = NULL;
	}
}

#ifdef DEBUG_ENABLED
static void
jtag2_shell_cmd(FILE *os, uint8_t argc, const char * const *argv)
{

	if (argc >= 1 && strcasecmp(argv[0], "debug") == 0) {
		if (argc >= 2) {
			if (strcasecmp(argv[1], "on") == 0)
				DEBUG_FLAG = PROTO_JTAG2_DEBUG_ON;
			else
			if (strcasecmp(argv[1], "off") == 0)
				DEBUG_FLAG = PROTO_JTAG2_DEBUG_OFF;
			else
			if (strcasecmp(argv[1], "dump") == 0)
				DEBUG_FLAG = PROTO_JTAG2_DEBUG_DUMP;
		}
	}

	fprintf(os, "JTAG2 PDI debug state: %s\n", (DEBUG_FLAG ? "on" : "off"));
}
#endif /* DEBUG_ENABLED */

void
proto_jtag2_init(void)
{

#ifdef DEBUG_ENABLED
	SHELL_CMD_ADD(jtag2);
#endif
}
