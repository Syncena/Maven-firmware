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

#include "platform.h"
#include "lpc_gpio.h"

/*
 * Our seven external interfaces. These are named according to their
 * pin number on the 10-pin header of the original wAVR 1.1 board.
 */
HAL_GPIO_PIN(EXT1, 0, 3)	/* SWO/TDO */
HAL_GPIO_PIN(EXT1_DIR, -1, 0)	/* Unused */
HAL_GPIO_PIN(EXT3, 0, 2)	/* SWDIO/TMS */
HAL_GPIO_PIN(EXT3_DIR, 0, 28)	/* SWDIO/TMS direction */
HAL_GPIO_PIN(EXT4, 0, 0)	/* SWCLK/TCK */
HAL_GPIO_PIN(EXT4_DIR, -1, 0)	/* Unused */
HAL_GPIO_PIN(EXT5, 0, 19)	/* /RST */
HAL_GPIO_PIN(EXT5_DIR, 0, 13)	/* /RST direction */
HAL_GPIO_PIN(EXT7, 0, 25)	/* Debug TxD */
HAL_GPIO_PIN(EXT7_DIR, -1, 0)	/* Unused */
HAL_GPIO_PIN(EXT8, 0, 1)	/* TDI */
HAL_GPIO_PIN(EXT8_DIR, -1, 0)	/* Unused */
HAL_GPIO_PIN(EXT9, 0, 24)	/* Debug RxD */
HAL_GPIO_PIN(EXT9_DIR, -1, 0)	/* Unused */
HAL_GPIO_PIN(BRIDGE, -1, 0)	/* Unused */

#define	SWO_UART_RX_FUNCTION	1

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
	static __always_inline void HW_PIN_##name##_function(char func)		\
		{ HAL_GPIO_##ext##_func(func); }				\
	static __always_inline void HW_PIN_##name##_gpio(void)			\
		{ HAL_GPIO_##ext##_func(HAL_GPIO_PIN_DISABLE_FUNCTION); }	\
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
		{ HAL_GPIO_##ext##_DIR_pullup(onoff); }

#endif /* HARDWARE_PLATFORM_H */
