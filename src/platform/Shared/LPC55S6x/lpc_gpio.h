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

/*
 * This was based on the hal_gpio.h for SAM4S, before I realised that the
 * author had added SAME54 support to his git repository. So while this 'port'
 * is all mine, the original author deserves some credit. Hence the copyright
 * below.
 */

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

#ifndef LPC_GPIO_H
#define LPC_GPIO_H

/*
 * Low-level GPIO pin support for LPC55S69.
 */
#define	HAL_GPIO_PIN_DISABLE_FUNCTION	0

#define HAL_GPIO_PIN(name, port, pin)					\
static __always_inline void HAL_GPIO_##name##_set(void)			\
{									\
	if ((port) >= 0)						\
		GPIO->SET[(port)] = 1u << (pin);			\
}									\
									\
static __always_inline void HAL_GPIO_##name##_clr(void)			\
{									\
	if ((port) >= 0)						\
		GPIO->CLR[(port)] = 1u << (pin);			\
}									\
									\
static __always_inline void HAL_GPIO_##name##_toggle(void)		\
{									\
	if ((port) >= 0)						\
		GPIO->NOT[(port)] = 1u << (pin);			\
}									\
									\
static __always_inline void HAL_GPIO_##name##_write(int value)		\
{									\
	if (value)							\
		HAL_GPIO_##name##_set();				\
	else								\
		HAL_GPIO_##name##_clr();				\
}									\
									\
static __always_inline void HAL_GPIO_##name##_in(void)			\
{									\
	if ((port) >= 0)						\
		GPIO->DIRCLR[(port)] = 1u << (pin);			\
}									\
									\
static __always_inline void HAL_GPIO_##name##_digimode(int value)	\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~(IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_ASW_MASK);\
		if (value) 						\
			_ioc |= IOCON_PIO_DIGIMODE_MASK;		\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_anamode(int value)	\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~(IOCON_PIO_DIGIMODE_MASK | IOCON_PIO_ASW_MASK);\
		if (value) 						\
			_ioc |= IOCON_PIO_ASW_MASK;			\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_out(void)			\
{									\
	if ((port) >= 0)						\
		GPIO->DIRSET[(port)] = 1u << (pin);			\
}									\
									\
static __always_inline int HAL_GPIO_##name##_is_output(void)		\
{									\
	if ((port) >= 0)						\
		return !!(GPIO->DIR[(port)] & (1u << (pin)));		\
	else								\
		return 0;						\
}									\
									\
static __always_inline void HAL_GPIO_##name##_pullup(int onoff)		\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~IOCON_PIO_MODE_MASK;				\
		if (onoff) 						\
			_ioc |= IOCON_PIO_MODE(2);			\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_pulldown(int onoff)	\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~IOCON_PIO_MODE_MASK;				\
		if (onoff) 						\
			_ioc |= IOCON_PIO_MODE_MASK;			\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}									\
									\
static __always_inline int HAL_GPIO_##name##_read(void)			\
{									\
	if ((port) >= 0) 						\
		return !!(GPIO->W[(port)][(pin)]);			\
	else								\
		return 0;						\
}									\
									\
static __always_inline int HAL_GPIO_##name##_state(void)		\
{									\
	if ((port) >= 0) 						\
		return !!(GPIO->SET[(port)] & (1 << (pin)));		\
	else								\
		return 0;						\
}									\
									\
static __always_inline void HAL_GPIO_##name##_func(uint32_t func)	\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~IOCON_PIO_FUNC_MASK;				\
		_ioc |= IOCON_PIO_FUNC(func);				\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}									\
									\
static __always_inline uint32_t HAL_GPIO_##name##_current_func(void)	\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= IOCON_PIO_FUNC_MASK;				\
		return _ioc >> IOCON_PIO_FUNC_SHIFT;			\
	} else {							\
		return 0;						\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_slew(int hi)		\
{									\
	if ((port) >= 0) {						\
		uint32_t _ioc = IOCON->PIO[(port)][(pin)];		\
		_ioc &= ~IOCON_PIO_SLEW_MASK;				\
		if (hi)							\
			_ioc |= IOCON_PIO_SLEW_MASK;			\
		IOCON->PIO[(port)][(pin)] = _ioc;			\
	}								\
}

#endif /* LPC_GPIO_H */
