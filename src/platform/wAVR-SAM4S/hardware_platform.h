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

#ifndef HARDWARE_PLATFORM_H
#define HARDWARE_PLATFORM_H

#include <stdbool.h>
#include "hal_gpio.h"

/*
 * Our seven external interfaces. These are numbered according to their
 * pin number on the 10-pin header.
 */
HAL_GPIO_PIN(EXT1, A, 5)
HAL_GPIO_PIN(EXT1_DIR, A, 18)
HAL_GPIO_PIN(EXT3, A, 7)
HAL_GPIO_PIN(EXT3_DIR, A, 4)
HAL_GPIO_PIN(EXT4, A, 6)
HAL_GPIO_PIN(EXT4_DIR, A, 19)
HAL_GPIO_PIN(EXT5, A, 2)
HAL_GPIO_PIN(EXT5_DIR, A, 8)
HAL_GPIO_PIN(EXT7, A, 10)
HAL_GPIO_PIN(EXT7_DIR, A, 15)
HAL_GPIO_PIN(EXT8, A, 0)
HAL_GPIO_PIN(EXT8_DIR, A, 16)
HAL_GPIO_PIN(EXT9, A, 9)
HAL_GPIO_PIN(EXT9_DIR, A, 20)

/*
 * The bridge between TxD/RxD for the external interface
 */
HAL_GPIO_PIN(BRIDGE, B, 4)

/*
 * Target Vcc
 */
#define TARGET_VCC_AD_CHANNEL   ADC_CHER_CH0
HAL_GPIO_PIN(TARGET_VCC, A, 17)

/*
 * Declare pins for display
 */
HAL_GPIO_PIN(DISPLAY_DCRS, A, 1)
HAL_GPIO_PIN(DISPLAY_CS, A, 3)
#define	GPIO_DISPLAY_CS_FUNCTION	1

/*
 * Declare pins for WINC1500 / Wiz5x00
 */
HAL_GPIO_PIN(NETWORK_EN, B, 12)
HAL_GPIO_PIN(NETWORK_CS, A, 11)
#define	GPIO_NETWORK_CS_FUNCTION	0
HAL_GPIO_PIN(NETWORK_IRQ, B, 0)
#define	GPIO_NETWORK_IRQ_PIO		PIOB
#define	GPIO_NETWORK_IRQ_BIT		(1u << 0)
#define	GPIO_NETWORK_IRQ_NUMBER	PIOB_IRQn
HAL_GPIO_PIN(NETWORK_RST, B, 1)		/* Note: Also resets display */

/*
 * Declare SPI pins (WiFi and Display interface)
 */
HAL_GPIO_PIN(SPI_MISO, A, 12)
#define	GPIO_SPI_MISO_FUNCTION	0
HAL_GPIO_PIN(SPI_MOSI, A, 13)
#define	GPIO_SPI_MOSI_FUNCTION	0
HAL_GPIO_PIN(SPI_SCK, A, 14)
#define	GPIO_SPI_SCK_FUNCTION	0

/*
 * Debug Console
 */
#define CONS_UART_REGS			UART1
#define CONS_UART_VECTOR		UART1_IRQn
#define	CONS_UART_DEVID			ID_UART1
HAL_GPIO_PIN(CONS_UART_RX, B, 2)
#define	GPIO_CONS_UART_RX_FUNCTION	0
HAL_GPIO_PIN(CONS_UART_TX, B, 3)
#define	GPIO_CONS_UART_TX_FUNCTION	0

/*
 * Target Serial Port
 */
#define TARGET_UART_REGS		UART0
#define TARGET_UART_VECTOR		UART0_IRQn
#define	TARGET_UART_DEVID		ID_UART0
#define	GPIO_TARGET_UART_RX_FUNCTION	0
#define	GPIO_TARGET_UART_TX_FUNCTION	0

/*
 * Target SWO/TDO Port
 */
#define SWO_UART_REGS			USART0
#define SWO_UART_VECTOR			USART0_IRQn
#define	SWO_UART_DEVID			ID_USART0
#define	SWO_UART_RX_FUNCTION		0

/*
 * Provision button (on WINC1500 module)
 */
#define	WINC_PROVISION_SW	WINC_GPIO_PIN_6

/*
 * Network/WiFi activity LEDs (on WINC1500 module)
 */
#define ACTIVITY_LED_WIFI	M2M_PERIPH_GPIO4
#define ACTIVITY_LED_NET	M2M_PERIPH_GPIO3

/*
 * Generic convenience macros for target-specific code
 */
#define	HW_PIN(name, ext)							\
	static __always_inline void HW_PIN_##name##_set(void)			\
		{ HAL_GPIO_##ext##_set(); }					\
	static __always_inline void HW_PIN_##name##_clr(void)			\
		{ HAL_GPIO_##ext##_clr(); }					\
	static __always_inline bool HW_PIN_##name##_state(void)			\
		{ return HAL_GPIO_##ext##_state(); }				\
	static __always_inline void HW_PIN_##name##_in(void)			\
		{ HAL_GPIO_##ext##_in(); }					\
	static __always_inline void HW_PIN_##name##_out(void)			\
		{ HAL_GPIO_##ext##_out(); }					\
	static __always_inline int HW_PIN_##name##_read(void)			\
		{ return HAL_GPIO_##ext##_read(); }				\
	static __always_inline void HW_PIN_##name##_function(int func)		\
		{ HAL_GPIO_##ext##_abcd(func); }				\
	static __always_inline void HW_PIN_##name##_gpio(void)			\
		{ HAL_GPIO_##ext##_in(); }					\
	static __always_inline void HW_PIN_##name##_pullup(int onoff)		\
		{ HAL_GPIO_##ext##_pullup(onoff); }				\
	static __always_inline void HW_PIN_##name##_pulldown(int onoff)		\
		{ HAL_GPIO_##ext##_pulldown(onoff); }				\
	static __always_inline void HW_PIN_##name##_DIR_set(void)		\
		{ HAL_GPIO_##ext##_DIR_set(); }					\
	static __always_inline void HW_PIN_##name##_DIR_clr(void)		\
		{ HAL_GPIO_##ext##_DIR_clr(); }					\
	static __always_inline void HW_PIN_##name##_DIR_in(void)		\
		{ HAL_GPIO_##ext##_DIR_in(); }					\
	static __always_inline void HW_PIN_##name##_DIR_out(void)		\
		{ HAL_GPIO_##ext##_DIR_out(); }					\
	static __always_inline void HW_PIN_##name##_DIR_pullup(int onoff)	\
		{ HAL_GPIO_##ext##_DIR_pullup(onoff); }				\
	static __always_inline void HW_PIN_##name##_DIR_pulldown(int onoff)	\
		{ HAL_GPIO_##ext##_DIR_pulldown(onoff); }

#endif /* HARDWARE_PLATFORM_H */
