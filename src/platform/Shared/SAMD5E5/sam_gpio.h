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

#ifndef SAM_GPIO_H
#define SAM_GPIO_H

/*
 * Low-level GPIO pin support for SAMD5x/SAME5x.
 *
 * Some caveats:
 *
 *  - Pull-up and pull-down are mutually exclusive, and must only be applied
 *    to pins configured as inputs. Invoking the *_pullup() or *_pulldown()
 *    functions on an output pin will change the driven state of the pin!
 *  - In addition to invoking *_in() to configure an input pin, you must
 *    also invoke *_inen() and, optionally, *_sampling() in order to
 *    configure the pin to behave correctly. The latter two are only required
 *    for regular I/O pins. Pins with PMUXEN set are controlled by the
 *    appropriate peripheral.
 *  - Use *_drvstr() to select I/O pin drive strength. *_drvstr(0) permits
 *    a pin to source/sink 2 mA with Vcc >= 3v. *_drvstr(0) permits a pin
 *    to source/sink 8 mA. Drive strength reduces significantly at lower
 *    supply voltages.
 */

#define	_GPPORT(p)	0x ## p - 0xa

#define	HAL_GPIO_PIN_DISABLE_FUNCTION	'\0'

#define HAL_GPIO_PIN(name, port, pin)					\
static __always_inline void HAL_GPIO_##name##_set(void)			\
{									\
	PORT->Group[_GPPORT(port)].OUTSET.reg = 1u << (pin);		\
}									\
									\
static __always_inline void HAL_GPIO_##name##_clr(void)			\
{									\
	PORT->Group[_GPPORT(port)].OUTCLR.reg = 1u << (pin);		\
}									\
									\
static __always_inline void HAL_GPIO_##name##_toggle(void)		\
{									\
	PORT->Group[_GPPORT(port)].OUTTGL.reg = 1u << (pin);		\
}									\
									\
static __always_inline void HAL_GPIO_##name##_write(int value)		\
{									\
	if (value)							\
		PORT->Group[_GPPORT(port)].OUTSET.reg = 1u << (pin);	\
	else								\
		PORT->Group[_GPPORT(port)].OUTCLR.reg = 1u << (pin);	\
}									\
									\
static __always_inline void HAL_GPIO_##name##_in(void)			\
{									\
	PORT->Group[_GPPORT(port)].DIRCLR.reg = 1u << (pin);		\
}									\
									\
static __always_inline void HAL_GPIO_##name##_inen(int value)		\
{									\
	if (value) {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg |=		\
		    PORT_PINCFG_INEN;					\
	} else {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg &=		\
		    ~PORT_PINCFG_INEN;					\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_sampling(int cont)	\
{									\
	if (cont)							\
		PORT->Group[_GPPORT(port)].CTRL.reg |= 1u << (pin);	\
	else								\
		PORT->Group[_GPPORT(port)].CTRL.reg &= ~(1u << (pin));	\
}									\
  									\
static __always_inline void HAL_GPIO_##name##_out(void)			\
{									\
	PORT->Group[_GPPORT(port)].DIRSET.reg = 1u << (pin);		\
}									\
									\
static __always_inline int HAL_GPIO_##name##_is_output(void)		\
{									\
	return !!(PORT->Group[_GPPORT(port)].DIR.reg & (1u << (pin)));	\
}									\
									\
static __always_inline void HAL_GPIO_##name##_pullup(int onoff)		\
{									\
	if (onoff) {							\
		PORT->Group[_GPPORT(port)].OUTSET.reg = 1u << (pin);	\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg |=		\
		    PORT_PINCFG_PULLEN;					\
	} else {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg &=		\
		    ~PORT_PINCFG_PULLEN;				\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_pulldown(int onoff)	\
{									\
	if (onoff) {							\
		PORT->Group[_GPPORT(port)].OUTCLR.reg = 1u << (pin);	\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg |=		\
		    PORT_PINCFG_PULLEN;					\
	} else {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg &=		\
		    ~PORT_PINCFG_PULLEN;				\
	}								\
}									\
									\
static __always_inline int HAL_GPIO_##name##_read(void)			\
{									\
	return !!(PORT->Group[_GPPORT(port)].IN.reg & (1 << pin));	\
}									\
									\
static __always_inline int HAL_GPIO_##name##_state(void)		\
{									\
	return !!(PORT->Group[_GPPORT(port)].OUT.reg & (1 << (pin)));	\
}									\
									\
static __always_inline void HAL_GPIO_##name##_pmux(char func)		\
{									\
	if (func != HAL_GPIO_PIN_DISABLE_FUNCTION) {			\
		uint8_t mask, pmux;					\
		mask = ((pin) & 1u) ? 0x0fu : 0xf0u;			\
		pmux = PORT->Group[_GPPORT(port)].PMUX[(pin)/2u].reg;	\
		pmux &= mask;						\
		pmux |= (uint8_t)((func-'A') & 0x0fu) << (((pin)&1)*4);	\
		PORT->Group[_GPPORT(port)].PMUX[(pin)/2u].reg = pmux;	\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg |=		\
		    PORT_PINCFG_PMUXEN;					\
	} else {							\
		PORT->Group[_GPPORT(port)].DIRCLR.reg = 1u << (pin);	\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg &=		\
		    ~PORT_PINCFG_PMUXEN;				\
	}								\
}									\
									\
static __always_inline char HAL_GPIO_##name##_current_pmux(void)	\
{									\
	if (PORT->Group[_GPPORT(port)].PINCFG[pin].reg &		\
	    PORT_PINCFG_PMUXEN){					\
		const uint8_t pmux =					\
		    PORT->Group[_GPPORT(port)].PMUX[(pin) / 2u].reg;	\
		return (char)(pmux >> (((pin) & 1u) * 4u) & 0x0fu)+'A';	\
	} else {							\
		return HAL_GPIO_PIN_DISABLE_FUNCTION;			\
	}								\
}									\
									\
static __always_inline void HAL_GPIO_##name##_drvstr(int value)		\
{									\
	if (value) {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg |=		\
		    PORT_PINCFG_DRVSTR;					\
	} else {							\
		PORT->Group[_GPPORT(port)].PINCFG[pin].reg &=		\
		    ~PORT_PINCFG_DRVSTR;				\
	}								\
}

extern void sam_gpio_cmd_init(void);
extern const char *sam_gpio_pin_name(char groupname, unsigned int pin);

#endif /* SAM_GPIO_H */
