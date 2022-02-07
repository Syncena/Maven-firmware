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

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "platform.h"
#include "hardware.h"
#include "sam_clocks.h"
#include "sam_sercom_usart.h"
#include "timer.h"
#include "target_comms.h"

HAL_GPIO_PIN(CONSOLE_TX, B, 2)
HAL_GPIO_PIN(CONSOLE_RX, B, 3)
#define	CONSOLE_UART_ID			ID_SERCOM5
#define	CONSOLE_UART_DEV		SERCOM5
#define	CONSOLE_UART_IRQ		SERCOM5_0_IRQn
#define	CONSOLE_UART_GCLK_CORE		SERCOM5_GCLK_ID_CORE
#define	CONSOLE_UART_GCLK_SLOW		SERCOM5_GCLK_ID_SLOW
#define	CONSOLE_UART_TX_PAD		0
#define	CONSOLE_UART_RX_PAD		1
#define	CONSOLE_PINMUX			'D'

HAL_GPIO_PIN(TARGET_TX, A, 0)
HAL_GPIO_PIN(TARGET_RX, A, 1)
#define	TARGET_UART_ID			ID_SERCOM1
#define	TARGET_UART_DEV			SERCOM1
#define	TARGET_UART_IRQ			SERCOM1_0_IRQn
#define	TARGET_UART_GCLK_CORE		SERCOM1_GCLK_ID_CORE
#define	TARGET_UART_TX_PAD		0
#define	TARGET_UART_RX_PAD		1
#define	TARGET_PINMUX			'D'

#ifdef CONFIG_TARGET_CORTEXM
HAL_GPIO_PIN(SWO_RX, A, 6)
#define	SWO_UART_ID			ID_SERCOM0
#define	SWO_UART_DEV			SERCOM0
#define	SWO_UART_IRQ			SERCOM0_0_IRQn
#define	SWO_UART_GCLK_CORE		SERCOM0_GCLK_ID_CORE
#define	SWO_UART_TX_PAD			0	/* Unused */
#define	SWO_UART_RX_PAD			2	/* GPIOA.6: EXT1 */
#endif /* CONFIG_TARGET_CORTEXM */

#if (MAVEN_BOARD == 3)
HAL_GPIO_PIN(WILC_TX, B, 12)
HAL_GPIO_PIN(WILC_RX, B, 14)
#endif
#ifdef PLATFORM_WILC_UART
#define	WILC_UART_ID			ID_SERCOM4
#define	WILC_UART_DEV			SERCOM4
#define	WILC_UART_IRQ			SERCOM4_0_IRQn
#define	WILC_UART_GCLK_CORE		SERCOM4_GCLK_ID_CORE
#define	WILC_UART_TX_PAD		0
#define	WILC_UART_RX_PAD		2
#define	WILC_PINMUX			'C'
#endif /* PLATFORM_WILC_UART */

static int
attach_console(unsigned int slow_gen, unsigned int fast_gen)
{
	static const sam_sercom_usart_attach_args_t aa = {
		.aa_name = "Console",
		.aa_regs = &CONSOLE_UART_DEV->USART,
		.aa_gclk_id = CONSOLE_UART_GCLK_CORE,
		.aa_txpad = CONSOLE_UART_TX_PAD,
		.aa_rxpad = CONSOLE_UART_RX_PAD,
		.aa_cfg = UART_DEFAULT_CFG,
		.aa_baud = 0,
		.aa_vector = CONSOLE_UART_IRQ,
		.aa_ipl = HW_IPL_USART_CONSOLE,
		.aa_use_softirq = HW_IPL_USART_CONSOLE < HW_IPL_SCHEDULER
	};

	/* Enable the APB clock for the selected SERCOM. */
	sam_periph_clock_enable(CONSOLE_UART_ID);

	/* Switch the GPIO ports over to SERCOM mode. */
	HAL_GPIO_CONSOLE_TX_pmux(CONSOLE_PINMUX);
	HAL_GPIO_CONSOLE_RX_pmux(CONSOLE_PINMUX);

	return sam_sercom_usart_attach(&aa, slow_gen, fast_gen);
}

static int
attach_target(unsigned int slow_gen, unsigned int fast_gen)
{
	static const sam_sercom_usart_attach_args_t aa = {
		.aa_name = "Target Serial",
		.aa_regs = &TARGET_UART_DEV->USART,
		.aa_gclk_id = TARGET_UART_GCLK_CORE,
		.aa_txpad = TARGET_UART_TX_PAD,
		.aa_rxpad = TARGET_UART_RX_PAD,
		.aa_cfg = UART_DEFAULT_CFG,
		.aa_baud = 0,
		.aa_vector = TARGET_UART_IRQ,
		.aa_ipl = HW_IPL_USART_TARGET,
		.aa_use_softirq = HW_IPL_USART_TARGET < HW_IPL_SCHEDULER
	};

	/* Enable the APB clock for the selected SERCOM. */
	sam_periph_clock_enable(TARGET_UART_ID);

	HAL_GPIO_TARGET_RX_inen(1);

	/* Ensure the TxD/RxD buffers are set to input mode */
	HW_PIN_TARGET_UART_TX_DIR_clr();
	HW_PIN_TARGET_UART_TX_DIR_out();
	HW_PIN_TARGET_UART_RX_DIR_clr();
	HW_PIN_TARGET_UART_RX_DIR_out();

	return sam_sercom_usart_attach(&aa, slow_gen, fast_gen);
}

#ifdef CONFIG_TARGET_CORTEXM
static int
attach_swo(unsigned int slow_gen, unsigned int fast_gen)
{
	static const sam_sercom_usart_attach_args_t aa = {
		.aa_name = "SWO",
		.aa_regs = &SWO_UART_DEV->USART,
		.aa_gclk_id = SWO_UART_GCLK_CORE,
		.aa_txpad = SWO_UART_TX_PAD,
		.aa_rxpad = SWO_UART_RX_PAD,
		.aa_cfg = UART_DEFAULT_CFG,
		.aa_baud = 0,
		.aa_vector = SWO_UART_IRQ,
		.aa_ipl = HW_IPL_USART_TARGET,
		.aa_use_softirq = HW_IPL_USART_TARGET < HW_IPL_SCHEDULER
	};

	/* Enable the APB clock for the selected SERCOM. */
	sam_periph_clock_enable(SWO_UART_ID);

	/*
	 * The pin mux for the SWO pin will be switched when/if
	 * someone connects to the SWO port over network/USB.
	 * We might, eventually, implement JTAG, where the SWO
	 * pin is repurposed as TDO.
	 */

	return sam_sercom_usart_attach(&aa, slow_gen, fast_gen);
}
#endif /* CONFIG_TARGET_CORTEXM */

#ifdef PLATFORM_WILC_UART
static int
attach_wilc(unsigned int slow_gen, unsigned int fast_gen)
{
	static const sam_sercom_usart_attach_args_t aa = {
		.aa_name = "WILC",
		.aa_regs = &WILC_UART_DEV->USART,
		.aa_gclk_id = WILC_UART_GCLK_CORE,
		.aa_txpad = WILC_UART_TX_PAD,
		.aa_rxpad = WILC_UART_RX_PAD,
		.aa_cfg = (UART_BAUDRATE_115200 | UART_STOPBITS_1 | \
			   UART_CHARSIZE_8 | UART_PARITY_NONE),
		.aa_baud = 0,
		.aa_vector = WILC_UART_IRQ,
		.aa_ipl = HW_IPL_USART_TARGET,
		.aa_use_softirq = HW_IPL_USART_TARGET < HW_IPL_SCHEDULER
	};

	/* Enable the APB clock for the selected SERCOM. */
	sam_periph_clock_enable(WILC_UART_ID);

	/* Switch the GPIO ports over to SERCOM mode. */
	HAL_GPIO_WILC_TX_pmux(WILC_PINMUX);
	HAL_GPIO_WILC_RX_pmux(WILC_PINMUX);

	return sam_sercom_usart_attach(&aa, slow_gen, fast_gen);
}
#endif /* PLATFORM_WILC_UART */

void
platform_uart_target_enable(void)
{

	/*
	 * Enable TxD external output buffer, and connect the SERCOM
	 * to the TxD pin.
	 */
	HAL_GPIO_TARGET_TX_pmux(TARGET_PINMUX);
	HW_PIN_TARGET_UART_TX_DIR_set();

	/*
	 * Connect RxD pin to SERCOM.
	 */
	HAL_GPIO_TARGET_RX_pmux(TARGET_PINMUX);
}

void
platform_uart_target_disable(void)
{

	/* Disconnect the TxD pin and disable the output buffer. */
	HW_PIN_TARGET_UART_TX_DIR_clr();
	HAL_GPIO_TARGET_TX_pmux('\0');
	HAL_GPIO_TARGET_TX_in();

	/* Ditto the RxD pin. */
	HAL_GPIO_TARGET_RX_pmux('\0');
}

int
platform_uart_attach(platform_uart_t which)
{
	static unsigned int slow_gen, fast_gen;

	if (slow_gen == 0) {
		unsigned int clk;

		/*
		 * The Sercom USART driver needs two clock sources; fast and
		 * slow. The latter is for baudrates < 57600. The former is
		 * used for higher baudrates.
		 *
		 * We will use two GCLKs to divide DPLL#0 to provide suitable
		 * input clocks for the USART.
		 */
		/* Grab a clock generator for the slow clock. */
		slow_gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		assert(slow_gen > 0);

		/* Derive the required clock frequency from DPLL#0. */
		clk = sam_gclk_gen_config(slow_gen, SAM_GCLK_GEN_SRC_DPLL0,
		    SAM_GCLK_GEN_DIVSEL_DIRECT,
		    sam_dpll_frequencies[0] / SAM_SERCOM_USART_SLOW_CLOCK, 0);
		assert(clk > 0);
		(void)clk;

		/* Grab a clock generator for the fast clock. */
		fast_gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		assert(fast_gen > 0);

		/* Derive the required clock frequency from DPLL#0. */
		clk = sam_gclk_gen_config(fast_gen, SAM_GCLK_GEN_SRC_DPLL0,
		    SAM_GCLK_GEN_DIVSEL_DIRECT,
		    sam_dpll_frequencies[0] / SAM_SERCOM_USART_FAST_CLOCK,
		    0);
		assert(clk > 0);
		(void)clk;

#ifdef CONFIG_TARGET_AVR
		{
			extern unsigned int platform_hw_pdi_clk_gen;
			platform_hw_pdi_clk_gen = fast_gen;
		}
#endif

		/* Slow clock is shared with all Sercoms */
		sam_gclk_periph_chan_enable(CONSOLE_UART_GCLK_SLOW, slow_gen);

		/*
		 * Configure some additional clocks for the benefit of
		 * sam_usart_baud_me_harder().
		 */
		unsigned int gen;

		/* Start with a 48 MHz clock derived from DFLL48M. */
		gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		clk = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DFLL,
		    SAM_GCLK_GEN_DIVSEL_DIRECT, 1, 0);
		assert(clk == 48000000u);
		(void)clk;

		/* A 50 MHz clock derived from DPLL#1. */
		gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		clk = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DPLL1,
		    SAM_GCLK_GEN_DIVSEL_DIRECT, 4, 0);
		assert(clk == 50000000u);
		(void)clk;

		/* A 40 MHz clock derived from DPLL#0. */
		gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
		clk = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DPLL0,
		    SAM_GCLK_GEN_DIVSEL_DIRECT, 3, 0);
		assert(clk == 40000000u);
		(void)clk;

#if !defined(PLATFORM_WILC_UART) && (MAVEN_BOARD == 3)
		HAL_GPIO_WILC_TX_pullup(1);
		HAL_GPIO_WILC_RX_pullup(1);
#endif
	}

	if (which == PLATFORM_UART_CONSOLE)
		return attach_console(slow_gen, fast_gen);

	if (which == PLATFORM_UART_TARGET)
		return attach_target(slow_gen, fast_gen);

#ifdef CONFIG_TARGET_CORTEXM
	if (which == PLATFORM_UART_SWO)
		return attach_swo(slow_gen, fast_gen);
#endif

#ifdef PLATFORM_WILC_UART
	if (which == PLATFORM_UART_WILC)
		return attach_wilc(slow_gen, fast_gen);
#endif

	return -1;
}
