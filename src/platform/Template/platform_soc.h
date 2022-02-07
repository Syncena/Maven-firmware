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

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "zone_alloc.h"

#include "fake-soc.h"
#include "template_power.h"

/*
 * Number of SoC-specific interrupt vector table entries.
 */
#define	SOC_IRQ_NSOURCES		60

/*
 * Location of Flash and SRAM.
 */
#define	PLATFORM_FLASH_BASE		0x00000000u
#define	PLATFORM_FLASH_SIZE		(256u * 1024u)
#define	PLATFORM_SRAM_BASE		0x20000000u
#define	PLATFORM_SRAM_SIZE		(256u * 1024u)

/* Force this to fail; a serial number will be fabricated. */
#define	PLATFORM_SERIAL_NUMBER		((const serial_number_t *) \
					 (uintptr_t)(0x20000000u))

/*
 * If the device has close-coupled SRAM, define this to enable its use.
 */
//#define	FAST_RAMFUNC			RAMFUNC

extern zone_t platform_rtos_zone;
#define	PLATFORM_RTOS_ZONE		platform_rtos_zone

#define	TEMPLATE_MAX_BAUDRATE		2250000u

/*
 * Target comms default values. These match the requirements of
 * the target_comms.c:tc_get_config() function.
 */
#define	PLATFORM_TC_DEFAULT_CFG_TARGET_CONSOLE			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_UART |			\
		      TARGET_COMMS_FLAGS_MUTABLE_CFG |		\
		      TARGET_COMMS_FLAGS_MUTABLE_BAUD |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 1,					\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 2048,				\
	 .tcc_rb_size_from_host = 256,				\
	 .tcc_uart_cfg = UART_DEFAULT_CFG,			\
	 .tcc_uart_baud = TARGET_COMMS_UART_BAUD_STANDARD,	\
	 .tcc_uart_max_baud = TEMPLATE_MAX_BAUDRATE}

#define	PLATFORM_TC_DEFAULT_CFG_SWO				\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_UART |			\
		      TARGET_COMMS_FLAGS_MUTABLE_BAUD |		\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 32768,				\
	 .tcc_rb_size_from_host = 16,				\
	 .tcc_uart_cfg = UART_BAUDRATE_CUSTOM |			\
	 		 UART_CHARSIZE_8 |			\
			 UART_PARITY_NONE |			\
			 UART_STOPBITS_1,			\
	 .tcc_uart_baud = 1000000u,				\
	 .tcc_uart_max_baud = TEMPLATE_MAX_BAUDRATE}

#define	PLATFORM_TC_DEFAULT_CFG_SEMIHOST0			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 512,				\
	 .tcc_rb_size_from_host = 64}

#define	PLATFORM_TC_DEFAULT_CFG_SEMIHOST1			\
	{.tcc_flags = TARGET_COMMS_FLAGS_CAN_SHARE |		\
		      TARGET_COMMS_FLAGS_MUTABLE_MODE |		\
		      TARGET_COMMS_FLAGS_CRLF |			\
		      TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 512,				\
	 .tcc_rb_size_from_host = 64}

#define	PLATFORM_TC_DEFAULT_CFG_GDB_SERVER0			\
	{.tcc_flags = TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 0,					\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_AVR_PROGRAMMER			\
	{.tcc_flags = TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = 0,					\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_GDB_SERVER1			\
	{.tcc_flags = TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#define	PLATFORM_TC_DEFAULT_CFG_SHELL				\
	{.tcc_flags = TARGET_COMMS_FLAGS_VALID,			\
	 .tcc_usb_port = TARGET_COMMS_USB_PORT_UNUSED,		\
	 .tcc_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED,		\
	 .tcc_rb_size_to_host = 0,				\
	 .tcc_rb_size_from_host = 0}

#endif /* PLATFORM_SOC_H */
