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
#include <string.h>

#include "timer.h"
#include "sam-efc.h"
#include "platform.h"
#include "hardware.h"
#include "mpu_common.h"
#include "sam-usart.h"
#include "spi-devs.h"
#include "timer.h"
#include "zone_alloc.h"
#include "rtos.h"
#ifdef CONFIG_TARGET_AVR
#include "avr_ll.h"
#endif

/*
 * PDI USART declarations
 */
#define	PDI_USART		USART0          /* PDI interface is on USART0 */
#define	PDI_USART_DEVID		ID_USART0
#define	PDI_USART_PDC		PDC_USART0
#define	PDI_USART_VECTOR	USART0_IRQn
#define	USART_RX_ERR_MASK	(US_IMR_RXBRK | US_IMR_OVRE | \
				 US_IMR_FRAME | US_IMR_PARE)

#define	GPIO_PDI_SCK_FUNCTION	1
#define	GPIO_PDI_RXD_FUNCTION	0
#define	GPIO_PDI_TXD_FUNCTION	0

#define	HW_PDI_MODE_INIT	(				\
	US_MR_USART_MODE_NORMAL	| /* Normal operating mode */	\
	US_MR_SYNC		| /* Synchronous mode */	\
	US_MR_CLKO		| /* Drive clock to SCK pin */	\
	US_MR_USCLKS_MCK	| /* Peripheral clock */	\
	US_MR_CHRL_8_BIT	| /* 8-bit character size */	\
	US_MR_PAR_EVEN		| /* Even parity */		\
	US_MR_NBSTOP_2_BIT	) /* 2 stop bits */

#define	HW_UPDI_MODE_INIT	(				\
	US_MR_USART_MODE_NORMAL	| /* Normal operating mode */	\
	US_MR_USCLKS_MCK	| /* Peripheral clock */	\
	US_MR_CHRL_8_BIT	| /* 8-bit character size */	\
	US_MR_PAR_EVEN		| /* Even parity */		\
	US_MR_NBSTOP_2_BIT	) /* 2 stop bits */

/*
 * Declare pins for ISP
 */
#define	GPIO_ISP_CLK_FUNCTION	1	/* Maybe use Function C PWMH0 */

uint8_t sysconfig_xtal;
static uint16_t sysconf_mainf;

static const struct mpu_entry mpu_table[] = {
	{
		/*
		 * The first region covers internal Flash.
		 * 0x400000 -> 0x41ffff (256 KB)
		 */
		IFLASH0_ADDR,
		IFLASH0_SIZE,
		(7u << MPU_RASR_AP_Pos) |	/* Read-only */
		(0u << MPU_RASR_XN_Pos) |	/* Execute OK */
		(1u << MPU_RASR_TEX_Pos) |	/* TEX: 001 */
		(1u << MPU_RASR_C_Pos) |	/* C: 1 */
		(1u << MPU_RASR_B_Pos) |	/* B: 1 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(1u << MPU_RASR_ENABLE_Pos)
	},
	{
		/*
		 * Second region covers all of internal SRAM
		 * 0x20000000 -> 0x2000ffff (64 KB)
		 */
		IRAM_ADDR,
		IRAM_SIZE,
		(3u << MPU_RASR_AP_Pos) |	/* Read/Write */
		(1u << MPU_RASR_XN_Pos) |	/* No execute */
		(1u << MPU_RASR_TEX_Pos) |	/* TEX: 001 */
		(1u << MPU_RASR_C_Pos) |	/* C: 1 */
		(1u << MPU_RASR_B_Pos) |	/* B: 1 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(1u << MPU_RASR_ENABLE_Pos)
	},
	{
		/*
		 * Third region covers internal peripherals
		 * 0x40000000 -> 0x400fffff (4 MB)
		 */
		0x40000000u,
		(4 * 1024 * 1024),
		(3u << MPU_RASR_AP_Pos) |	/* Read/Write */
		(1u << MPU_RASR_XN_Pos) |	/* No execute */
		(2u << MPU_RASR_TEX_Pos) |	/* TEX: 010 */
		(0u << MPU_RASR_C_Pos) |	/* C: 0 */
		(0u << MPU_RASR_B_Pos) |	/* B: 0 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(1u << MPU_RASR_ENABLE_Pos)
	},
	{
		/*
		 * Fourth region covers system peripherals
		 * 0xe0000000 -> 0xefffffff (256 MB)
		 */
		0xe0000000u,
		(256 * 1024 * 1024),
		(3u << MPU_RASR_AP_Pos) |	/* Read/Write */
		(1u << MPU_RASR_XN_Pos) |	/* No execute */
		(2u << MPU_RASR_TEX_Pos) |	/* TEX: 010 */
		(0u << MPU_RASR_C_Pos) |	/* C: 0 */
		(0u << MPU_RASR_B_Pos) |	/* B: 0 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(1u << MPU_RASR_ENABLE_Pos)
	},
	{
		/*
		 * Fifth for writing Flash. Disabled by default; to be enabled when
		 * performaing Flash operations.
		 */
		IFLASH0_ADDR,
		IFLASH0_SIZE,
		(3u << MPU_RASR_AP_Pos) |	/* Read-Write */
		(0u << MPU_RASR_XN_Pos) |	/* Execute OK */
		(1u << MPU_RASR_TEX_Pos) |	/* TEX: 001 */
		(1u << MPU_RASR_C_Pos) |	/* C: 1 */
		(1u << MPU_RASR_B_Pos) |	/* B: 1 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(0u << MPU_RASR_ENABLE_Pos)	/* Off until needed */
	},
	{
		/*
		 * Sixth covers the SRAM stubs, currently 512 bytes in size.
		 * This entry overrides the no-execute entry #1.
		 */
		IRAM_ADDR,
		512,
		(7u << MPU_RASR_AP_Pos) |	/* Read-only */
		(0u << MPU_RASR_XN_Pos) |	/* Execute OK */
		(1u << MPU_RASR_TEX_Pos) |	/* TEX: 001 */
		(1u << MPU_RASR_C_Pos) |	/* C: 1 */
		(1u << MPU_RASR_B_Pos) |	/* B: 1 */
		(0u << MPU_RASR_S_Pos) |	/* S: 0 */
		(1u << MPU_RASR_ENABLE_Pos)	/* Enabled */
	}
};
#define MPU_TABLE_COUNT	(sizeof(mpu_table) / sizeof(mpu_table[0]))

void
platform_reboot(void)
{

	RSTC->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST;

	for(;;);
}

void
mpu_efc_begin(void)
{
	rtos_saved_ipl_t ipl;
	MPU_Type *mpu = MPU;

	if ((MPU->CTRL & MPU_CTRL_ENABLE_Msk) == 0)
		return;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	__DSB();
	__ISB();

	/* Flash is now read/write */
	mpu->RNR = 4;
	mpu->RASR |= (1u << MPU_RASR_ENABLE_Pos);

	__DSB();
	__ISB();
	rtos_ipl_restore(ipl);
}

void
mpu_efc_end(void)
{
	rtos_saved_ipl_t ipl;
	MPU_Type *mpu = MPU;

	if ((MPU->CTRL & MPU_CTRL_ENABLE_Msk) == 0)
		return;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	__DSB();
	__ISB();

	/* Flash back to read-only */
	mpu->RNR = 4;
	mpu->RASR &= ~(1u << MPU_RASR_ENABLE_Pos);

	__DSB();
	__ISB();
	rtos_ipl_restore(ipl);
}

zone_t platform_rtos_zone;

void
platform_early_init(void)
{
	extern unsigned int _heap_end;	/* Initialised by the linker. */
	uintptr_t hstart = (uintptr_t)*((uint32_t *)SCB->VTOR);
	uintptr_t hend = (uintptr_t)(void *)&_heap_end;

	(void) zone_create(NULL, (void *)hstart, (size_t)(hend - hstart), 0);

#if defined(PLATFORM_RTOS_ZONE_LEN) && (PLATFORM_RTOS_ZONE_LEN != 0)
	/* Carve off a separate zone for RTOS allocations. */
	void *p = zone_malloc(PLATFORM_RTOS_ZONE_LEN);
	platform_rtos_zone = zone_create_named("RTOS", NULL, p,
	    PLATFORM_RTOS_ZONE_LEN, ZONE_FLAGS_ASSERT_ALLOC_FAILURE);
#endif
}

void
platform_watchdog_disable(void)
{

	/* Disable watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
}

void
platform_init(void)
{
	uint32_t mainf;

#if 0 /* Disable this. Too prone to trips when target-powered. */
	/*
	 * Enable brown-out reset.
	 *
	 * The WiFi controller is the critical device for supply voltage.
	 * It is spec'd down to 2.7 volts. The display controller is
	 * spec'd down to 1.65 volts but its DC-DC converter only goes
	 * down to 3.0 volts. I'm not too concerned about that.
	 *
	 * The SAM4S datasheet (page 1147 of the June 9, 2015 version)
	 * shows two potential thresholds: 2.68 and 2.8 volts. Looking
	 * at the spread between their min/max I reckon 2.68 is a fairly
	 * safe bet. We can always change it to 2.8 volts with a software
	 * update.
	 *
	 * So, 0x9 is the magic value for 2.68 volts.
	 */
	SUPC->SUPC_SMMR = SUPC_SMMR_SMSMPL_CSM | SUPC_SMMR_SMRSTEN | 0x9;
#endif

	/*
	 * Configure Flash wait states.
	 */
	(void) sam_efc_init(5);

	/*
	 * Initialise memory protection
	 */
	mpu_init(mpu_table, MPU_TABLE_COUNT);
	mpu_enable();

	/*
	 * Initialise clocks, power and cache
	 */

	/*
	 * We'd like to switch to the fast external crystal, but first
	 * we need to ensure we have one. The Atmel docs spell out the
	 * procedure for this in section 28.5.7 of the SAM4S datasheet.
	 *
	 * 1. MCK must select the slow clock (CSS = 0 in the Master Clock
	 *    Register (PMC_MCKR))
	 *    Ensure the prescalar is 1.
	 */
	PMC->PMC_MCKR =
	    (PMC->PMC_MCKR & ~(PMC_MCKR_CSS_Msk | PMC_MCKR_PRES_Msk)) |
	    PMC_MCKR_CSS_SLOW_CLK | PMC_MCKR_PRES_CLK_1;

	/*
	 * 2. Wait for the MCKRDY flag in PMC_SR to be 1
	 */
	while ((PMC->PMC_SR & PMC_SR_MCKRDY) == 0)
		;

	/*
	 * 3. The fast crystal must be enabled by programming 1 in the MOSCXTEN
	 *    field in the CKGR_MOR with the MOSCXTST field being programmed to
	 *    the appropriate value.
	 */
	PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN |
	    CKGR_MOR_MOSCXTST(1);

	/*
	 * 4. Wait for the MOSCXTS flag to be 1 in PMC_SR to get the end of a
	 *    start-up period of the fast crystal oscillator.
	 */
	while ((PMC->PMC_SR & PMC_SR_MOSCXTS) == 0)
		;

	/*
	 * 5. MOSCSEL must be programmed to 1 in CKGR_MOR to select fast main
	 *    crystal oscillator for the main clock.
	 */
	PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL;

	/*
	 * 6. MOSCSEL must be read until its value equals 1
	 */
	while ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == 0)
		;

	/*
	 * 7. Then the MOSCSELS status flag must be checked in PMC_SR.
	 */
	if ((PMC->PMC_SR & PMC_SR_MOSCSELS) == 0) {
		/*
		 * We have no fast crystal clock. Fall-back to the fast
		 * RC oscillator. It's running at 4 MHz by default but
		 * we're going to amp it up to 12 MHz.
		 *
		 * But first disable the external crystal.
		 */

	}

	/*
	 * We're up and running on the external crystal.
	 * Now work out its frequency. The options are 12 or 16 MHz
	 * depending on which one was cheaper for any given batch.
	 */
	PMC->CKGR_MCFR = CKGR_MCFR_RCMEAS;
	while ((PMC->CKGR_MCFR & CKGR_MCFR_MAINFRDY) == 0)
		;
	sysconf_mainf = (PMC->CKGR_MCFR & CKGR_MCFR_MAINF_Msk);
	mainf = sysconf_mainf * 2048;

	if (mainf < 14000000) {
		/* PLLA == 120 MHz == (12 MHz * 10) */
		sysconfig_xtal = 12;
		PMC->CKGR_PLLAR = CKGR_PLLAR_MULA(10-1) |
		    CKGR_PLLAR_PLLACOUNT(0x3f) |
		    CKGR_PLLAR_DIVA(1) | CKGR_PLLAR_ONE;
	} else {
		/* PLLA == 120 MHz == (16 MHz * 15) / 2 */
		sysconfig_xtal = 16;
		PMC->CKGR_PLLAR = CKGR_PLLAR_MULA(15-1) |
		    CKGR_PLLAR_PLLACOUNT(0x3f) |
		    CKGR_PLLAR_DIVA(2) | CKGR_PLLAR_ONE;
	}

	while ((PMC->PMC_SR & PMC_SR_LOCKA) == 0)
		;

	/* Switch main system clock to PLLA */
	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) |
	    PMC_MCKR_CSS_PLLA_CLK;
	while ((PMC->PMC_SR & PMC_SR_MCKRDY) == 0)
		;

	/*
	 * Get the system tick running
	 * This would normally be the job of FreeRTOS but we need the
	 * ticker running during initialisation for various delay loops.
	 */
	SysTick->LOAD = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	/* Enable PIOA and PIOB */
	PMC->PMC_PCER0 = (1u << ID_PIOA) | (1u << ID_PIOB);

	/* Enable SPI */
	PMC->PMC_PCER0 = 1u << ID_SPI;

	/* Enable UART0 and UART1 */
	PMC->PMC_PCER0 = (1u << ID_UART0) | (1u << ID_UART1);

	/* Enable the ADC */
	PMC->PMC_PCER0 = 1u << ID_ADC;

	/* Enable Timer0 for ISP CLK */
	PMC->PMC_PCER0 = 1u << ID_TC0;

#ifndef __SAM4S4A__
	/*
	 * Note that the SAM4SA has no cache (according to the datasheet).
	 * So on the XPlained Pro board (with SAM4SD32C) we intentionally
	 * disable the cache to more closely mimic performance of the
	 * production hardware.
	 */
	if ((CMCC->CMCC_SR & CMCC_SR_CSTS) != 0)
		CMCC->CMCC_CTRL = 0;
#endif

	/*
	 * We need a couple of additional I/O pins:
	 *  - PB4: ISP PDI Output Enabled
	 *    Defaults to JTAG.TDI but we're only using SWD.
	 *  - PB12: WINC1500 Enable
	 *    Defaults to ERASE.
	 *
	 * Note that we need to delay long enough for a high-level on ERASE
	 * to be recognised. Otherwise we will prevent the "emergency"
	 * Flash Erase from happening.
	 */
	REG_CCFG_SYSIO = 0;
	timer_buzz_delay_ms(1000);
	REG_CCFG_SYSIO = CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO12;
}

void
platform_fabricate_serial(serial_number_t *sn)
{
	sam_efc_unique_id_t *uid = (sam_efc_unique_id_t *)sn;

	static_assert(sizeof(*uid) == sizeof(*sn), "Serial number bogons");

	if (sam_efc_get_unique_id(uid) < 0) {
		/*
		 * This is how the legacy code fabricates a serial number
		 * if the unique ID is unreadable.
		 */
		sn->u32[0] = 0x32313000u;
		sn->u32[1] = 0x36353433u;
		sn->u32[2] = 0x41493837u;
		sn->u32[3] = 0x45444342u;
	}
}

int
platform_uart_attach(platform_uart_t which)
{
	sam_usart_attach_args_t aa;

	aa.aa_cfg = UART_DEFAULT_CFG;
	aa.aa_baud = 38400u;

	if (which == PLATFORM_UART_CONSOLE) {
		HAL_GPIO_CONS_UART_TX_abcd(GPIO_CONS_UART_TX_FUNCTION);
		HAL_GPIO_CONS_UART_RX_pullup(1);
		HAL_GPIO_CONS_UART_RX_abcd(GPIO_CONS_UART_RX_FUNCTION);
		aa.aa_name = NULL;	/* Hide it from view. */
		aa.aa_type = SAM_USART_TYPE_UART;
		aa.aa_regs.uart = CONS_UART_REGS;
		aa.aa_use_dma = 0;
		aa.aa_vector = CONS_UART_VECTOR;
		aa.aa_ipl = HW_IPL_USART_CONSOLE;
		aa.aa_use_softirq = HW_IPL_USART_CONSOLE < HW_IPL_SCHEDULER;
	} else
	if (which == PLATFORM_UART_TARGET) {
		/* Ensure the TxD/RxD buffers are set to input mode */
		HW_PIN_TARGET_UART_TX_DIR_clr();
		HW_PIN_TARGET_UART_TX_DIR_out();
		HW_PIN_TARGET_UART_RX_DIR_clr();
		HW_PIN_TARGET_UART_RX_DIR_out();

		/* Enable pull-ups on TxD/RxD pins */
		HW_PIN_TARGET_UART_TX_pullup(1);
		HW_PIN_TARGET_UART_RX_pullup(1);

		aa.aa_name = "Target Serial";
		aa.aa_type = SAM_USART_TYPE_UART;
		aa.aa_regs.uart = TARGET_UART_REGS;
		aa.aa_use_dma = 0;
		aa.aa_vector = TARGET_UART_VECTOR;
		aa.aa_ipl = HW_IPL_USART_TARGET;
		aa.aa_use_softirq = HW_IPL_USART_TARGET < HW_IPL_SCHEDULER;
	} else {
		/* SWO UART */

		/* Power-up USART0 */
		PMC->PMC_PCER0 = 1u << ID_USART0;

		/* PinMux is handled in arm/hardware.c */
		aa.aa_name = "SWO";
		aa.aa_type = SAM_USART_TYPE_USART;
		aa.aa_regs.usart = USART0;
		aa.aa_use_dma = 1;
		aa.aa_vector = USART0_IRQn;
		aa.aa_ipl = HW_IPL_USART_TARGET;
		aa.aa_use_softirq = HW_IPL_USART_TARGET < HW_IPL_SCHEDULER;
	}

	return sam_usart_init(&aa);
}

void
platform_uart_target_enable(void)
{

	/*
	 * Enable TxD external output buffer, and connect the UART
	 * to the TxD pin.
	 */
	HW_PIN_TARGET_UART_TX_function(GPIO_TARGET_UART_TX_FUNCTION);
	HW_PIN_TARGET_UART_TX_DIR_set();

	/*
	 * Connect RxD pin to the UART.
	 */
	HW_PIN_TARGET_UART_RX_function(GPIO_TARGET_UART_RX_FUNCTION);
}

void
platform_uart_target_disable(void)
{

	/* Disconnect the TxD pin and disable the output buffer. */
	HW_PIN_TARGET_UART_TX_DIR_clr();
	HW_PIN_TARGET_UART_TX_in();

	/* Ditto the RxD pin. */
	HW_PIN_TARGET_UART_RX_in();
}

void
platform_wipe_globals(void)
{

	sam_efc_wipe_user_area();
}

void
platform_attach_devices(void)
{

	spi_devs_init();
}

int
platform_flash_program(uint32_t page, const void *src, uint32_t size)
{

	(void) size;

	return sam_efc_program_page(page, src, 0);
}

void
platform_hw_init_target(void)
{

	HAL_GPIO_EXT8_pulldown(0);
	HAL_GPIO_EXT8_pullup(1);
	HAL_GPIO_EXT8_in();
	HAL_GPIO_EXT8_clr();

	/* Power-up the PDI USART */
	PMC->PMC_PCER0 = 1u << PDI_USART_DEVID;
}

#ifdef CONFIG_TARGET_AVR
void
platform_hw_isp_clk_start(uint32_t freq)
{
	rtos_saved_ipl_t ipl;
	uint32_t rc;

	/*
	 * We want WAVSEL=11, MCLK/2, TIOA0 toggle on RC compare
	 */

	rc = (120000000 / 4) / freq;

	if (rc == 0 || rc >= 65536ul)
		return;

	TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |
	    TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_ACPC_TOGGLE;
	TC0->TC_CHANNEL[0].TC_RC = rc;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HAL_GPIO_EXT8_abcd(GPIO_ISP_CLK_FUNCTION);
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	HAL_GPIO_EXT8_DIR_set();
	rtos_ipl_restore(ipl);
}

void
platform_hw_isp_clk_stop(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HAL_GPIO_EXT8_DIR_clr();
	HAL_GPIO_EXT8_in();
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKDIS;
	rtos_ipl_restore(ipl);
}

void
platform_hw_pdi_enable(uint32_t pdi_clk)
{
	Usart *u = PDI_USART;

	/* Enable USART, but keep ASYNC for now */
	u->US_BRGR = F_CPU / pdi_clk;
	u->US_MR = HW_PDI_MODE_INIT & ~US_MR_SYNC;

	/* Enable the USART's SCK and RxD functions */
	HW_PIN_PDI_RXD_function(GPIO_PDI_RXD_FUNCTION);
	HW_PIN_PDI_TXD_function(GPIO_PDI_TXD_FUNCTION);
	HW_PIN_PDI_SCK_function(GPIO_PDI_SCK_FUNCTION);

	/* Drive SCK out to the target */
	HW_PIN_PDI_SCK_BUFFER_OUT();

	/* Enable tx and rx */
	u->US_CR = US_CR_RXEN | US_CR_TXEN;

	/* Switch USART to synchronous mode to start the clock */
	u->US_MR = HW_PDI_MODE_INIT;
}

void
platform_hw_pdi_disable(void)
{
	Usart *u = PDI_USART;
	Pdc *p = PDI_USART_PDC;

	(void) u->US_CSR;

	u->US_CR = US_CR_RXDIS | US_CR_TXDIS;
	u->US_MR = 0;
	p->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
	p->PERIPH_RCR = 0;
}

void
platform_hw_pdi_rx_purge(void)
{
	Usart *u = PDI_USART;
	Pdc *p = PDI_USART_PDC;

	if (u->US_MR == 0) {
		/* PDI/UPDI mode is not enabled. Abort! */
		return;
	}

	/* Disable Rx DMA */
	p->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
	p->PERIPH_RCR = 0;

	/* Clear the FIFO */
	u->US_CR = US_CR_RSTSTA;
	while ((u->US_CSR & (US_CSR_RXRDY | USART_RX_ERR_MASK)) != 0)
		(void) u->US_RHR;
	u->US_CR = US_CR_RSTSTA;
}

static uint16_t
platform_calc_brgr(uint32_t baud)
{
	uint32_t fp, cd;

	/*
	 * Verify the specified baudrate can be achieved.
	 */
	fp = (8 * F_CPU + (16 * baud) / 2) / (16 * baud);
	cd = fp >> 3;
	fp &= 0x07;
	if (cd < 1 || cd > 0xffffu)
		return 0;

	return (uint16_t)((cd << US_BRGR_CD_Pos) | (fp << US_BRGR_FP_Pos));
}

void
platform_hw_pdi_send_break(int howmany, int change_baud)
{
	Usart *u = PDI_USART;
	uint16_t old_brgr, brgr;
	rtos_saved_ipl_t ipl;

	if (u->US_MR == 0) {
		/* PDI/UPDI mode is not enabled. Abort! */
		return;
	}

	brgr = old_brgr = u->US_BRGR;

	if (change_baud != 0) {
		if ((brgr = platform_calc_brgr(400)) == 0)
			brgr = old_brgr;
	}

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	platform_hw_pdi_rx_purge();

	/* Enable Tx */
	avr_ll_pdi_tx_enable();

	/* Ensure the transmitter is idle */
	while ((u->US_CSR & US_CSR_TXEMPTY) == 0)
		;

	u->US_BRGR = brgr;

	while (howmany--) {
		/*
		 * Start the break, wait a few cycles and signal "stop"
		 * The controller will clear US_CSR.US_CSR_TXEMPTY
		 * until the break condition is finished.
		 */
		u->US_CR = US_CR_STTBRK;
		timer_delay_cycles(4);
		u->US_CR = US_CR_STPBRK;

		/* Wait for TXEMPTY */
		while ((u->US_CSR & US_CSR_TXEMPTY) == 0)
			;
	}

	/* Disable Tx */
	avr_ll_pdi_tx_disable();

	/* Clear Rx "break received" status and Rx holding register */
	platform_hw_pdi_rx_purge();

	u->US_BRGR = old_brgr;

	rtos_ipl_restore(ipl);
}

void
platform_hw_updi_function_enable(void)
{

	/* Enable the USART's RxD/TxD functions */
	HW_PIN_PDI_RXD_function(GPIO_PDI_RXD_FUNCTION);
	HW_PIN_PDI_TXD_function(GPIO_PDI_TXD_FUNCTION);
}

uint8_t
platform_hw_updi_enable(uint32_t baud)
{
	Usart *u = PDI_USART;
	uint16_t brgr;

	brgr = platform_calc_brgr(baud);
	if (brgr == 0)
		return 1;

	/* Enable USART */
	u->US_BRGR = brgr;
	u->US_MR = HW_UPDI_MODE_INIT;

	/* Enable tx and rx */
	u->US_CR = US_CR_RXEN | US_CR_TXEN;

	/* Ensure Rx is quiescent */
	platform_hw_pdi_rx_purge();

	return AVR_LL_UPDI_RESET_SUCCESS;
}

uint8_t
platform_hw_updi_change_baud(uint32_t baud)
{
	uint16_t brgr;

	if ((brgr = platform_calc_brgr(baud)) == 0)
		return 1;

	PDI_USART->US_BRGR = brgr;

	return AVR_LL_UPDI_RESET_SUCCESS;
}

uint8_t
platform_hw_pdi_transfer(const uint8_t *txp, uint32_t txlen,
    uint8_t *rxp, uint32_t rxlen)
{
	Usart *u = PDI_USART;
	Pdc *p = PDI_USART_PDC;
	rtos_saved_ipl_t ipl;

	if (u->US_MR == 0) {
		/* PDI/UPDI mode is not enabled. Abort! */
		return 1;
	}

	/* Purge the Rx FIFO */
	platform_hw_pdi_rx_purge();

	/* Prepare for data phase DMA Rx */
	p->PERIPH_TPR = (uintptr_t) txp; 
	p->PERIPH_TCR = txlen;
	p->PERIPH_RPR = (uintptr_t) rxp; 

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	/* Enable the Tx buffer */
	avr_ll_pdi_tx_enable();

	/* Ensure all stores to RAM have completed */
	__DSB();
	__ISB();

	/* Send the Tx buffer */
	p->PERIPH_PTCR = PERIPH_PTCR_TXTEN;

	/* Wait for Tx to complete */
	while (p->PERIPH_TCR != 0)
		;
	while ((u->US_CSR & US_CSR_TXEMPTY) == 0)
		;

	/*
	 * At this point we need to turn the interface around.
	 * According to the spec, the default turnaround period
	 * is 128 XCK clock cycles (minimum is 2, but we don't
	 * configure anything that low).
	 *
	 * With the default 128 cycles, we have the following
	 * times to effect the transition:
	 *
	 *   - 128 uS with XCK = 1 MHz.
	 *   - 12.8 uS with XCK = 10 MHz.
	 *
	 * The latter is around 1500 CPU cycles, which should
	 * allow plenty time.
	 */

	/*
	 * Flush the receiver.
	 * Note: We don't expect any Rx errors here since Tx is
	 * connected directly to our Rx pin via the bridge.
	 * Since there's a buffer between us and the target we
	 * will never see contention. Thus no errors.
	 */
	(void) u->US_RHR;
	(void) u->US_RHR;
	u->US_CR = US_CR_RSTSTA;

	/* Start Rx DMA if required */
	if (rxlen != 0) {
		p->PERIPH_RCR = rxlen;
		p->PERIPH_PTCR = PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTDIS;
	} else
		p->PERIPH_PTCR = PERIPH_PTCR_TXTDIS;

	/* Switch to receive mode */
	avr_ll_pdi_tx_disable();

	/* The receive, if any, will continue in the background */
	rtos_ipl_restore(ipl);

	return 0;
}

uint8_t
platform_hw_pdi_rx_status(void)
{
	Usart *u = PDI_USART;
	Pdc *p = PDI_USART_PDC;
	uint32_t st;

	if (u->US_MR == 0) {
		/* PDI/UPDI mode is not enabled. Abort! */
		return AVR_LL_PDI_RX_IDLE;
	}

	/*
	 * If the receiver is disabled then we were not expecting
	 * to receive anything, so let the caller know we're idle.
	 */
	if ((p->PERIPH_PTSR & PERIPH_PTSR_RXTEN) == 0) {
		return AVR_LL_PDI_RX_IDLE;
	}

	/*
	 * Check for Rx errors.
	 */
	st = u->US_CSR;
	if (st & USART_RX_ERR_MASK) {
		/*
		 * Abort any in-progress receive and purge.
		 */
		platform_hw_pdi_rx_purge();
#if (RELEASE_BUILD == 0)
		printf("ERR_MASK: st = %08" PRIx32 "\n", st);
#endif
		return AVR_LL_PDI_RX_CORRUPT;
	}

	/* Check if all expected data has been received */
	if ((st & (US_CSR_ENDRX | US_CSR_RXBUFF)) != 0) {
		/* We're done */
		p->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
		return AVR_LL_PDI_RX_DONE;
	}

	/* Data still being received */
	return AVR_LL_PDI_RX_BUSY;
}
#endif /* CONFIG_TARGET_AVR */
