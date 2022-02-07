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

#ifndef PLATFORM_SOC_H
#define PLATFORM_SOC_H

#include <stdint.h>
#include <stdlib.h>

#include "zone_alloc.h"

#if (MAVEN_BOARD == 2)
#include "samd51g19a.h"
#elif (MAVEN_BOARD == 3)
#include "samd51j20a.h"
#else
#error "Unsupported MAVEN_BOARD"
#endif

#include "maven_button.h"
#include "maven_power.h"

extern void maven_display_attach(unsigned int gen);
extern void maven_wifi_attach(unsigned int gen);
extern void maven_wifi_provision_start(void);
extern uint16_t maven_get_adc_calib(const uint32_t);
extern int maven_check_button(void);
extern int maven_nvm_program(uint32_t addr, const void *src, size_t len);

#define	SOC_IRQ_NSOURCES		137

#define	PLATFORM_FLASH_BASE		FLASH_ADDR
#define	PLATFORM_SRAM_BASE		HSRAM_ADDR

#if (MAVEN_BOARD == 2)
#define	PLATFORM_FLASH_SIZE		((512u * 1024u) - 0x4000u)
#define	PLATFORM_SRAM_SIZE		(192u * 1024u)
#else
#define	PLATFORM_FLASH_SIZE		((1024u * 1024u) - 0x4000u)
#define	PLATFORM_SRAM_SIZE		(256u * 1024u)
#endif /* (MAVEN_BOARD == 2) */

#define	PLATFORM_MAX_RESCUE_CLK		12000000u

extern zone_t platform_rtos_zone;
#define	PLATFORM_RTOS_ZONE		platform_rtos_zone

#define	PLATFORM_SERIAL_NUMBER		((const serial_number_t *) \
					 (uintptr_t)(NVMCTRL_USER + 0x20u))

#define	MAVEN_MAX_BAUDRATE		2500000u

/*
 * Target comms default values. These match the requirements of
 * the target_comms.c:tc_get_config() function.
 */
#define	PLATFORM_TC_DEFAULT_CFG_TARGET_CONSOLE			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_NAGLE_RESTART |	\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_UART |			\
		      TARGET_COMMS_FLAGS_MUTABLE_CFG |		\
		      TARGET_COMMS_FLAGS_MUTABLE_BAUD |		\
		      TARGET_COMMS_FLAGS_TELNET |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 1,					\
	 .tcc_tcp_port = 2000,					\
	 .tcc_rb_size_to_host = 2048,				\
	 .tcc_rb_size_from_host = 256,				\
	 .tcc_uart_cfg = UART_DEFAULT_CFG,			\
	 .tcc_uart_baud = TARGET_COMMS_UART_BAUD_STANDARD,	\
	 .tcc_uart_max_baud = MAVEN_MAX_BAUDRATE}

#define	PLATFORM_TC_DEFAULT_CFG_SWO				\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_NAGLE_RESTART |	\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_UART |			\
		      TARGET_COMMS_FLAGS_MUTABLE_BAUD |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 2,					\
	 .tcc_tcp_port = 2001,					\
	 .tcc_rb_size_to_host = 32768,				\
	 .tcc_rb_size_from_host = 16,				\
	 .tcc_uart_cfg = UART_BAUDRATE_CUSTOM |			\
	 		 UART_CHARSIZE_8 |			\
			 UART_PARITY_NONE |			\
			 UART_STOPBITS_1,			\
	 .tcc_uart_baud = 1000000u,				\
	 .tcc_uart_max_baud = MAVEN_MAX_BAUDRATE}

#define	PLATFORM_TC_DEFAULT_CFG_SEMIHOST0			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_NAGLE_RESTART |	\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_TELNET |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = 2002,					\
	 .tcc_rb_size_to_host = 512,				\
	 .tcc_rb_size_from_host = 64}

#define	PLATFORM_TC_DEFAULT_CFG_SEMIHOST1			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_NAGLE_RESTART |	\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_TELNET |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = 2003,					\
	 .tcc_rb_size_to_host = 512,				\
	 .tcc_rb_size_from_host = 64}

#define	PLATFORM_TC_DEFAULT_CFG_GDB_SERVER0			\
	{.tcc_flags = TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_DO_NAGLE |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 0,					\
	 .tcc_tcp_port = 3333,					\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_AVR_PROGRAMMER			\
	{.tcc_flags = TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_DO_NAGLE |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 0,					\
	 .tcc_tcp_port = 3000,					\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_GDB_SERVER1			\
	{.tcc_flags = TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_DO_NAGLE |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = 3334,					\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_SHELL				\
	{.tcc_flags = TARGET_COMMS_FLAGS_TELNET |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = 23,					\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_WIFI_UART			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_NAGLE |	\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_UART |			\
		      TARGET_COMMS_FLAGS_TELNET |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = 2100,					\
	 .tcc_rb_size_to_host = 2048,				\
	 .tcc_rb_size_from_host = 16,				\
	 .tcc_uart_cfg = UART_BAUDRATE_115200 |			\
	 		 UART_CHARSIZE_8 |			\
			 UART_PARITY_NONE |			\
			 UART_STOPBITS_1,			\
	 .tcc_uart_baud = TARGET_COMMS_UART_BAUD_STANDARD}

/* We support the Target Comms auto-baud feature. */
//#define	PLATFORM_AUTOBAUD	1

#endif /* PLATFORM_SOC_H */
