/*
 * Copyright (c) 2014-2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* From https://github.com/ataradov/mcu-starter-projects.git */

/*
 * Changes copyright (c) 2022, Steve C. Woodford.
 * All Rights Reserved.
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include "platform_soc.h"

#define HAL_GPIO_PIN(name, port, pin)						\
  static __always_inline void HAL_GPIO_##name##_set(void)			\
  {										\
    PIO##port->PIO_SODR = (1 << pin);						\
    (void)HAL_GPIO_##name##_set;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_clr(void)			\
  {										\
    PIO##port->PIO_CODR = (1 << pin);						\
    (void)HAL_GPIO_##name##_clr;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_toggle(void)			\
  {										\
    if (PIO##port->PIO_ODSR & (1 << pin))					\
      PIO##port->PIO_CODR = (1 << pin);						\
    else									\
      PIO##port->PIO_SODR = (1 << pin);						\
    (void)HAL_GPIO_##name##_toggle;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_write(int value)		\
  {										\
    if (value)									\
      PIO##port->PIO_SODR = (1 << pin);						\
    else									\
      PIO##port->PIO_CODR = (1 << pin);						\
    (void)HAL_GPIO_##name##_write;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_in(void)			\
  {										\
    PIO##port->PIO_PER = (1 << pin);						\
    PIO##port->PIO_ODR = (1 << pin);						\
    (void)HAL_GPIO_##name##_in;							\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_out(void)			\
  {										\
    PIO##port->PIO_PER = (1 << pin);						\
    PIO##port->PIO_OER = (1 << pin);						\
    (void)HAL_GPIO_##name##_out;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_is_output(void)			\
  {										\
    return (PIO##port->PIO_OSR & (1 << pin)) != 0;				\
    (void)HAL_GPIO_##name##_out;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_pullup(int onoff)		\
  {										\
    if (onoff)									\
    	PIO##port->PIO_PUER = (1 << pin);					\
    else									\
    	PIO##port->PIO_PUDR = (1 << pin);					\
    (void)HAL_GPIO_##name##_pullup;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_pulldown(int onoff)		\
  {										\
    if (onoff)									\
    	PIO##port->PIO_PPDER = (1 << pin);					\
    else									\
    	PIO##port->PIO_PPDDR = (1 << pin);					\
    (void)HAL_GPIO_##name##_pullup;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_read(void)			\
  {										\
    return (PIO##port->PIO_PDSR & (1 << pin)) != 0;				\
    (void)HAL_GPIO_##name##_read;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_state(void)			\
  {										\
    return (PIO##port->PIO_ODSR & (1 << pin)) != 0;				\
    (void)HAL_GPIO_##name##_state;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_abcd(int abcd)			\
  {										\
    if (abcd & 1)								\
      PIO##port->PIO_ABCDSR[0] |= (1 << pin);					\
    else									\
      PIO##port->PIO_ABCDSR[0] &= ~(1 << pin);					\
    if (abcd & 2)								\
      PIO##port->PIO_ABCDSR[1] |= (1 << pin);					\
    else									\
      PIO##port->PIO_ABCDSR[1] &= ~(1 << pin);					\
    PIO##port->PIO_PDR = (1 << pin);						\
    (void)HAL_GPIO_##name##_abcd;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_mden(void)			\
  {										\
    PIO##port->PIO_MDER = (1 << pin);						\
    (void)HAL_GPIO_##name##_mden;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_mddis(void)			\
  {										\
    PIO##port->PIO_MDDR = (1 << pin);						\
    (void)HAL_GPIO_##name##_mddis;						\
  }										\
  										\
  static __always_inline void HAL_GPIO_##name##_analog(void)			\
  {										\
    (void)HAL_GPIO_##name##_analog;						\
  }

#if 0
/* The following kept for posterity... */

#include "stm32f4xx.h"

#define HAL_GPIO_PIN(name, port, pin)						\
  static __always_inline void HAL_GPIO_##name##_set(void)			\
  {										\
    GPIO##port->BSRR = (1u << pin);						\
    (void)HAL_GPIO_##name##_set;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_clr(void)			\
  {										\
    GPIO##port->BSRR = (0x10000u << pin);					\
    (void)HAL_GPIO_##name##_clr;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_toggle(void)			\
  {										\
    if (GPIO##port->ODR & (1u << pin))						\
      GPIO##port->BSRR = (0x10000u << pin);					\
    else									\
      GPIO##port->BSRR = (1u << pin);						\
    (void)HAL_GPIO_##name##_toggle;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_write(int value)		\
  {										\
    if (value)									\
      GPIO##port->BSRR = (1u << pin);						\
    else									\
      GPIO##port->BSRR = (0x10000u << pin);					\
    (void)HAL_GPIO_##name##_write;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_in(void)			\
  {										\
    uint32_t __m = 3u << ((pin) * 2u);						\
    GPIO##port->MODER &= ~__m;							\
    (void)HAL_GPIO_##name##_in;							\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_out(void)			\
  {										\
    uint32_t __x, __m = 3u << ((pin) * 2u);					\
    __x = GPIO##port->MODER & ~__m;						\
    GPIO##port->MODER = __x | (1u << ((pin) * 2u));				\
    /* XXX: Deal with OSPEEDR setting. */					\
    (void)HAL_GPIO_##name##_out;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_is_output(void)			\
  {										\
    uint32_t __m = 3u << ((pin) * 2u);						\
    return (GPIO##port->MODER & __m) != 0u;					\
    (void)HAL_GPIO_##name##_out;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_pullup(int onoff)		\
  {										\
    uint32_t __x, __m = 1u << ((pin) * 2u);					\
    __x = GPIO##port->PUPDR & ~__m;						\
    if (onoff)									\
    	__x |= __m;								\
    GPIO##port->PUPDR = __x;							\
    (void)HAL_GPIO_##name##_pullup;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_pulldown(int onoff)		\
  {										\
    uint32_t __x, __m = 2u << ((pin) * 2u);					\
    __x = GPIO##port->PUPDR & ~__m;						\
    if (onoff)									\
    	__x |= __m;								\
    GPIO##port->PUPDR = __x;							\
    (void)HAL_GPIO_##name##_pullup;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_read(void)			\
  {										\
    return (GPIO##port->IDR & (1u << pin)) != 0;				\
    (void)HAL_GPIO_##name##_read;						\
  }										\
										\
  static __always_inline int HAL_GPIO_##name##_state(void)			\
  {										\
    return (GPIO##port->ODR & (1u << pin)) != 0;				\
    (void)HAL_GPIO_##name##_state;						\
  }										\
										\
  static __always_inline void HAL_GPIO_##name##_function(unsigned int af)	\
  {										\
    uint32_t __x, __m = 0xfu << (((pin) & 7u) * 4u);				\
    unsigned int __idx = ((pin) & 8u) ? 1u : 0u;				\
    __x = GPIO##port->AFR[__idx] & ~__m;					\
    GPIO##port->AFR[__idx] = __x | ((af << (((pin) & 7u) * 4u)) & __m);		\
    __x = GPIO##port->MODER & (3u << ((pin) * 2u));				\
    GPIO##port->MODER = __x | (2u << ((pin) * 2u));				\
    /* XXX: Deal with OSPEEDR setting. */					\
    (void)HAL_GPIO_##name##_function;						\
  }										\
  										\
  static __always_inline void HAL_GPIO_##name##_analog(void)			\
  {										\
    uint32_t __x, __m = 3u << ((pin) * 2u);					\
    __x = GPIO##port->MODER & ~__m;						\
    GPIO##port->MODER = __x | (3u << ((pin) * 2u));				\
    (void)HAL_GPIO_##name##_function;						\
  }
#endif

#endif /* HAL_GPIO_H */
