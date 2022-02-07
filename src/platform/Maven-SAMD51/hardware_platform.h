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
#include "platform.h"
#include "sam_gpio.h"

/*
 * Our seven external interfaces. These are numbered according to their
 * pin number on the 10-pin header of the original wAVR 1.1 board.
 */
HAL_GPIO_PIN(EXT1, A, 6)
HAL_GPIO_PIN(EXT1_DIR, A, 10)
HAL_GPIO_PIN(EXT3, A, 20)
HAL_GPIO_PIN(EXT3_DIR, A, 8)
HAL_GPIO_PIN(EXT4, A, 4)
HAL_GPIO_PIN(EXT4_DIR, A, 9)
HAL_GPIO_PIN(EXT5, A, 5)
HAL_GPIO_PIN(EXT5_DIR, A, 21)
HAL_GPIO_PIN(EXT7, A, 0)
HAL_GPIO_PIN(EXT7_DIR, A, 27)
#if (MAVEN_BOARD == 2)
HAL_GPIO_PIN(EXT8, A, 16)
#else
HAL_GPIO_PIN(EXT8, B, 31)
#endif
HAL_GPIO_PIN(EXT8_DIR, A, 22)
HAL_GPIO_PIN(EXT9, A, 1)
HAL_GPIO_PIN(EXT9_DIR, A, 23)

/*
 * The bridge between EXT4/EXT1
 */
HAL_GPIO_PIN(BRIDGE, A, 2)

#define	SWO_UART_RX_FUNCTION	'D'

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
		{ HAL_GPIO_##ext##_pmux(func); }				\
	static __always_inline void HW_PIN_##name##_gpio(void)			\
		{ HAL_GPIO_##ext##_pmux(HAL_GPIO_PIN_DISABLE_FUNCTION); }	\
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

#ifdef CONFIG_TARGET_AVR
extern void platform_hw_init_target_avr(void);
#endif

#endif /* HARDWARE_PLATFORM_H */
