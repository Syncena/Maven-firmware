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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "platform.h"
#include "platform_usb.h"
#include "hardware.h"
#include "serial_number.h"
#include "timer.h"
#include "rtos.h"
#include "mpu_common.h"
#include "network.h"
#include "sam_cmcc.h"
#include "sam_clocks.h"
#include "sam_eic.h"
#include "configdb.h"
#include "zone_alloc.h"

#include "sam_dmac.h"

#if (CONFIG_USE_ATMEL_USB != 0)
#include "sam_usb.h"
#endif

#define	SMARTEEPROM_SIZE	1024u

/* Unused GPIO pins. These will have pull-ups enabled at startup. */
#if (MAVEN_BOARD == 2)
#define	MV_UNUSED_PORTA_OUTSET	(1u << 11)
#define	MV_UNUSED_PORTB_OUTSET	0u
#else
#define	MV_UNUSED_PORTA_OUTSET	((1u << 11) | (1u << 16))
#define	MV_UNUSED_PORTB_OUTSET	((1u << 0) | (1u << 4) | (1u << 5) |	\
				 (1u << 6) | (1u << 7) | (1u << 12) |	\
				 (1u << 13) | (1u << 15) | (1u << 16) |	\
				 (1u << 17) | (1u << 22) | (1u << 30))
#endif /* (MAVEN_BOARD == 2) */

/*
 * We have no external clock source on this board (discounting USB SOF).
 *
 * Ultimately, we expect DPLL#0 to generate a 120 MHz clock using
 * any working clock as its reference. Since the only clock we have is
 * DFLL48M, use that.
 */
#define	MAVEN_DPLL_SOURCE	SAM_DPLL_REFCLK_GCLK	/* DFLL48M */

#define	FREQ_MHZ(f)		((f) * 1000u * 1000u)

/* F_CPU would normally be defined in the Makefile, but just in case... */
#ifndef F_CPU
#define	F_CPU	FREQ_MHZ(120)
#endif

#define	MPU_FLASH_ATTRS(srd, en) \
	(\
		(ARM_MPU_AP_RO << MPU_RASR_AP_Pos) |	/* Read-only */	 \
		(0u << MPU_RASR_XN_Pos) |		/* Execute OK */ \
		(0u << MPU_RASR_TEX_Pos) |		/* TEX: 000 */   \
		(1u << MPU_RASR_C_Pos) |		/* C: 1 */       \
		(0u << MPU_RASR_B_Pos) |		/* B: 0 */       \
		(0u << MPU_RASR_S_Pos) | 		/* S: 0 */	 \
		((srd) << MPU_RASR_SRD_Pos) |		/* SRD mask */	 \
		((en) << MPU_RASR_ENABLE_Pos)		/* Enable bit */ \
	)

#define	MPU_NVM_ATTRS(srd, ap, en) \
	(\
		((ap) << MPU_RASR_AP_Pos) |		/* Access perm */\
		(1u << MPU_RASR_XN_Pos) |		/* No Execute */ \
		(0u << MPU_RASR_TEX_Pos) |		/* TEX: 000 */   \
		(0u << MPU_RASR_C_Pos) |		/* C: 0 */       \
		(0u << MPU_RASR_B_Pos) |		/* B: 0 */       \
		(0u << MPU_RASR_S_Pos) | 		/* S: 0 */	 \
		((srd) << MPU_RASR_SRD_Pos) |		/* SRD mask */	 \
		((en) << MPU_RASR_ENABLE_Pos)		/* Enable bit */ \
	)

#define	MPU_SRAM_ATTRS(srd) \
	(\
		(ARM_MPU_AP_FULL << MPU_RASR_AP_Pos) |	/* Read-write */ \
		(1u << MPU_RASR_XN_Pos) |		/* No Execute */ \
		(0u << MPU_RASR_TEX_Pos) |		/* TEX: 000 */	 \
		(1u << MPU_RASR_C_Pos) |		/* C: 1 */	 \
		(0u << MPU_RASR_B_Pos) |		/* B: 0 */	 \
		(0u << MPU_RASR_S_Pos) |		/* S: 0 */	 \
		((srd) << MPU_RASR_SRD_Pos) |		/* SRD mask */	 \
		(1u << MPU_RASR_ENABLE_Pos)				 \
	)

#define	MPU_DEVICE_ATTRS \
	(\
		(ARM_MPU_AP_FULL << MPU_RASR_AP_Pos) |	/* Read/Write */ \
		(1u << MPU_RASR_XN_Pos) |		/* No execute */ \
		(2u << MPU_RASR_TEX_Pos) |		/* TEX: 010 */	 \
		(0u << MPU_RASR_C_Pos) |		/* C: 0 */	 \
		(0u << MPU_RASR_B_Pos) |		/* B: 0 */	 \
		(0u << MPU_RASR_S_Pos) |		/* S: 0 */	 \
		(1u << MPU_RASR_ENABLE_Pos)				 \
	)

static const struct mpu_entry mpu_table[] = {
#if (MAVEN_BOARD == 2)
	/* 512 KB Internal Flash. Write-through, read-allocate */
	{FLASH_ADDR, 512u * 1024u, MPU_FLASH_ATTRS(0x0u, 1)},

	/* 192 KB of SRAM. No execute, write-through. */
	{HSRAM_ADDR, 256u * 1024u, MPU_SRAM_ATTRS(0xc0u)},
#else /* MAVEN_BOARD == 3 */
	/* 1 MB Internal Flash. Write-through, read-allocate */
	{FLASH_ADDR, 1024u * 1024u, MPU_FLASH_ATTRS(0x0u, 1)},

	/* 256 KB of SRAM. No execute, write-through. */
	{HSRAM_ADDR, 256u * 1024u, MPU_SRAM_ATTRS(0x00u)},
#endif	/* (MAVEN_BOARD == 2) */

	/*
	 * Block out the lowest 4 KB of internal Flash. This permits
	 * reliable detection of NULL pointer dereferencing.
	 */
	{FLASH_ADDR, 4u * 1024, MPU_FLASH_ATTRS(0x0u, 1)},

	/* Internal peripherals. */
	{HPB0_ADDR, 128u * 1024u * 1024u, MPU_DEVICE_ATTRS},

	/*
	 * NVM User Page. Read only, non-cacheable, no access to
	 * first 64 bytes (config and serial number).
	 */
	{NVMCTRL_USER, FLASH_USER_PAGE_SIZE,
	    MPU_NVM_ATTRS(0x01u, ARM_MPU_AP_RO, 1)},

	/* NVM Software Calibration region. */
	{NVMCTRL_SW0, 32u, MPU_NVM_ATTRS(0x01u, ARM_MPU_AP_RO, 1)},
};
#define	MPU_TABLE_COUNT	(sizeof(mpu_table) / sizeof(mpu_table[0]))

/* Copy of NVM sw calib. area. */
struct maven_nvm_sw_cal {
	uint32_t sc_ana_cal;
	uint16_t sc_usb_cal;
};
static struct maven_nvm_sw_cal maven_nvm_sw_cal;

#if (PLATFORM_HAS_CPU_DELAY != 0)
static void
tcc_wait_sync(Tcc *tcc, uint32_t mask)
{

	while ((tcc->SYNCBUSY.reg & mask) != 0)
		;
}
#endif

static void
maven_init_clocks(void)
{
	unsigned int gen, clock;

	/*
	 * Initialise DPLL#0 and DPLL#1 to generate 120 MHz and 200 MHz
	 * clocks respectively. We'll derive all other clocks from them.
	 *
	 * The primary clock source will be DFLL48M. Without USB, this
	 * will provide a nominal 48 MHz clock. If/when USB is connected
	 * then DFLL48M will phase-lock to USB SOF to provide a more
	 * accurate frequency. Not that it matters too much - we're
	 * tolerant of minor variations in the primary clock frequency.
	 */
	clock = sam_dfll48_lock(SAM_DFLL48_LOCK_USB_SOF, false);
	assert(clock = 48000000);

	/* Allocate a clock generator for DFLL48. */
	gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
	assert(gen > 0);

	/*
	 * GCLK is used undivided by DPLL#n, so we must configure the
	 * generator to divide it down to a legal value. Anything under
	 * 3.2 MHz is acceptable. Dividing by 24 gives a nice even
	 * multiple, and a 2 MHz output clock.
	 */
	clock = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DFLL,
	    SAM_GCLK_GEN_DIVSEL_DIRECT,
	    sam_dfll48m_frequency / FREQ_MHZ(2), 0);
	assert(clock >= 32768u);
	assert(clock <= 3200000u);
	(void) clock;

	/*
	 * Enable the DPLL#0 and DPLL#1 peripheral channel clocks, sourced
	 * from the generator allocated above.
	 */
	clock = sam_gclk_periph_chan_enable(OSCCTRL_GCLK_ID_FDPLL0, gen);
	assert(clock > 0);
	(void) clock;
	clock = sam_gclk_periph_chan_enable(OSCCTRL_GCLK_ID_FDPLL1, gen);
	assert(clock > 0);
	(void) clock;

	/*
	 * Start up DPLL#0, slaved to the selected clock source, at 120 MHz.
	 * This will be used to drive GCLK0 and, hence, MAINCK.
	 */
	clock = sam_dpll_config(SAM_DPLL_ID_0, SAM_DPLL_REFCLK_GCLK, F_CPU);
	assert(clock > 0);
	(void) clock;

	/* Start up DPLL#1, configured to generate a 200 MHz clock. */
	clock = sam_dpll_config(SAM_DPLL_ID_1, SAM_DPLL_REFCLK_GCLK,
	    FREQ_MHZ(200));
	assert(clock > 0);
	(void) clock;

	/*
	 * Switch GCLK_MAIN over to DPLL#0's undivided output, CPUDIV=1.
	 * This configures Flash wait states and SystemCoreClock.
	 */
	clock = sam_gclk0_mainck_config(SAM_GCLK_GEN_SRC_DPLL0, 1,
	    SAM_GCLK_GEN_DIVSEL_DIRECT, 0, 0);
	assert(clock > 0);
	(void) clock;

	/*
	 * timer_cpu_delay_us() uses 24-bit TCC1, clocked at 16 MHz.
	 */
	sam_periph_clock_enable(ID_TCC1);
	gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
	assert(gen > 0);

	/* The generator will use DFLL to generate a 16 MHz clock. */
	clock = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DFLL,
	    SAM_GCLK_GEN_DIVSEL_DIRECT,
	    sam_dfll48m_frequency / FREQ_MHZ(16), 0);
	assert(clock > 0);
	(void) clock;

        sam_gclk_periph_chan_enable(TCC1_GCLK_ID, gen);

	Tcc *tcc = TCC1;

	/* If the timer is currently enabled, stop it. */
	if (tcc->CTRLA.bit.ENABLE) {
		tcc->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
		tcc_wait_sync(tcc, TCC_SYNCBUSY_ENABLE);
	}

	/* Reset timer. */
	tcc->CTRLA.reg = TCC_CTRLA_SWRST;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_SWRST);

	/* Counter is clocked directly from the GCLK source. */
	tcc->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TCC_CTRLA_RUNSTDBY;

	/* "Normal Frequency Generation" */
	tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_WAVE);

	/* Start timer. */
	tcc->CTRLA.reg |= TCC_CTRLA_ENABLE;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_ENABLE);
}

zone_t platform_rtos_zone;

void
platform_early_init(void)
{

	/* Always ensure the cache is invalidated and disabled. */
	CMCC_Disable();

	uintptr_t hstart = (uintptr_t)*((uint32_t *)SCB->VTOR);
	uintptr_t hend = HSRAM_ADDR + PLATFORM_SRAM_SIZE;

        (void) zone_create(NULL, (void *)hstart, (size_t)(hend - hstart), 0);

#ifdef PLATFORM_RTOS_ZONE_LEN
	/* Carve off a separate zone for RTOS allocations. */
	void *p = zone_malloc(PLATFORM_RTOS_ZONE_LEN);
	platform_rtos_zone = zone_create_named("RTOS", NULL, p,
	    PLATFORM_RTOS_ZONE_LEN, ZONE_FLAGS_ASSERT_ALLOC_FAILURE);
#endif
}

void
platform_fabricate_serial(serial_number_t *sn)
{
	uint32_t u;

	/*
	 * Serial number failed validation. Fabricate something which
	 * has a half decent chance of being unique, but add a '-' to
	 * indicate this is a fabrication.
	 */

	u  = *((uint32_t *)(uintptr_t)0x008061fcu);
	u ^= *((uint32_t *)(uintptr_t)0x00806010u);
	u ^= *((uint32_t *)(uintptr_t)0x00806014u);
	u ^= *((uint32_t *)(uintptr_t)0x00806018u);

	sn->zero = 0x00u;
	sprintf(sn->cstr, "MAV%uZZ%07" PRIX32, MAVEN_BOARD, u >> 4);
	sn->checksum = 0x00u;
}

void
platform_init(void)
{

	maven_init_clocks();

	/* Read the NVM Calibration Areas. */
	memcpy(&maven_nvm_sw_cal, (void *)NVMCTRL_SW0, 6);

	/* Configure the MPU. */
	mpu_init(mpu_table, MPU_TABLE_COUNT);

	mpu_enable();

	/* It's now safe to bring up the cache. */
	CMCC_Enable();
}

void
platform_reboot(void)
{

	NVIC_SystemReset();
}

void
platform_watchdog_disable(void)
{
}

static size_t
maven_cf_read(void *cookie, void *buff)
{

	if (buff != NULL) {
		while (NVMCTRL->SEESTAT.bit.BUSY)
			rtos_yield();
		memcpy(buff, cookie, SMARTEEPROM_SIZE);
	}

	return SMARTEEPROM_SIZE;
}

#ifndef CONFIGDB_READ_ONLY
static int
maven_cf_write(void *cookie, const void *buff, size_t len)
{

	assert(len < SMARTEEPROM_SIZE);

	while (NVMCTRL->SEESTAT.bit.BUSY)
		rtos_yield();

	memcpy(cookie, buff, len);

	while (NVMCTRL->SEESTAT.bit.BUSY)
		rtos_yield();

	if (NVMCTRL->SEESTAT.bit.LOAD) {
		NVMCTRL->CTRLB.reg = 0xa533u;	/* SEEFLUSH */
		(void) NVMCTRL->CTRLB.reg;
	}

	while (NVMCTRL->SEESTAT.bit.BUSY)
		rtos_yield();

	return len;
}
#endif /* CONFIGDB_READ_ONLY */

void
platform_attach_configdb(void)
{
	static struct configdb_driver cd;

	cd.cd_read = maven_cf_read;
#ifndef CONFIGDB_READ_ONLY
	cd.cd_write = maven_cf_write;
#endif
	cd.cd_cookie = (void *)SEEPROM_ADDR;

	(void) configdb_attach_driver(&cd, SMARTEEPROM_SIZE);
}

static void
maven_unused_pin(unsigned int group, uint32_t mask)
{
	int pin;

	if (mask == 0)
		return;

	PORT->Group[group].OUTSET.reg = mask;

	while ((pin = ffs(mask)) != 0) {
		pin -= 1;
		mask &= ~(1u << pin);
		PORT->Group[group].PINCFG[pin].reg |= PORT_PINCFG_PULLEN;
	}
}

void
platform_attach_devices(void)
{
	unsigned int gen, clock;
	static const uint8_t unused_peripheral_clocks[] = {
		ID_PUKCC, ID_ICM, ID_SDHC0, ID_RTC, ID_RAMECC, ID_QSPI,
#ifdef ID_CAN0
		ID_CAN0,
#endif
#ifdef ID_CAN1
		ID_CAN1,
#endif
#ifdef ID_SDHC1
		ID_SDHC1,
#endif
#ifdef ID_GMAC
		ID_GMAC,
#endif
#ifdef ID_TC4
		ID_TC4,
#endif
	};
#define	NCLOCKS	(sizeof(unused_peripheral_clocks) / \
		 sizeof(unused_peripheral_clocks[0]))

	/*
	 * Before doing anything else, disable clocks to all unused
	 * devices. A whole bunch are enabled by default at reset.
	 */
	for (unsigned int i = 0; i < NCLOCKS; i++)
		sam_periph_clock_enable(unused_peripheral_clocks[i]);

	/*
	 * Enable pullups on unused pins
	 */
	maven_unused_pin(0, MV_UNUSED_PORTA_OUTSET);
	maven_unused_pin(1, MV_UNUSED_PORTB_OUTSET);

	/*
	 * SPI and I2C devices share a common 100 MHz clock. We can derive
	 * that rate by dividing DPLL#1's 200 MHz output.
	 */
	gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
	assert(gen > 0);

	/* The generator will use DPLL#1 divided by 2. */
	clock = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DPLL1,
	    SAM_GCLK_GEN_DIVSEL_DIRECT, 2, 0);
	assert(clock > 0);
	(void) clock;

	/*
	 * For best performance, the SPI controller uses the DMAC.
	 */
	sam_dmac_attach();

	/*
	 * WiFi and user button need the external interrupt controller's
	 * services.
	 */
	sam_eic_init(gen, SAM_EIC_FILTER_CLOCK_GCLK,
	    SAM_EIC_DEBOUNCE_SAMPLES_7, SAM_EIC_DEBOUNCE_PRESCALER_DIV256);

	/*
	 * WiFi and OLED display share a common /RST pin.
	 * Since the WiFi's reset timing is pretty strict with respect
	 * to its ChipEN signal, we must attach WiFi first.
	 * The display will be reset as a side effect of this, so will
	 * not need to perform its own reset.
	 */
	maven_wifi_attach(gen);

	maven_display_attach(gen);

	maven_button_attach();

#if (CONFIG_USE_TINYUSB != 0)
	sam_usb_platform_init();
#endif

#if !defined(NDEBUG)
	sam_gpio_cmd_init();
#endif
}

void
platform_hw_init_target(void)
{

	/*
	 * Enable the input bufffers
	 */
	HAL_GPIO_EXT1_inen(1);
	HAL_GPIO_EXT3_inen(1);
	HAL_GPIO_EXT4_inen(1);
	HAL_GPIO_EXT5_inen(1);
	HAL_GPIO_EXT7_inen(1);
	HAL_GPIO_EXT8_inen(1);

	/*
	 * Enable continuous sampling mode.
	 */
	HAL_GPIO_EXT1_sampling(1);
	HAL_GPIO_EXT3_sampling(1);
	HAL_GPIO_EXT4_sampling(1);
	HAL_GPIO_EXT5_sampling(1);
	HAL_GPIO_EXT7_sampling(1);
	HAL_GPIO_EXT8_sampling(1);

#ifdef CONFIG_TARGET_AVR
	platform_hw_init_target_avr();
#endif
}

uint16_t
maven_get_adc_calib(const uint32_t id)
{
	uint16_t comp, refbuf, r2r;

	if (id == ID_ADC0) {
		comp = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 2) & 7u);
		refbuf = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 5) & 7u);
		r2r = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 8) & 7u);
	} else {
		comp = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 16) & 7u);
		refbuf = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 19) & 7u);
		r2r = (uint16_t)((maven_nvm_sw_cal.sc_ana_cal >> 22) & 7u);
	}

	return ADC_CALIB_BIASREFBUF(refbuf) | ADC_CALIB_BIASR2R(r2r) |
	    ADC_CALIB_BIASCOMP(comp);
}

#if (CONFIG_USE_ATMEL_USB != 0)
uint16_t
sam_usb_get_padcal(void)
{
	uint16_t transn, transp, trim;

	transn = (uint16_t)((maven_nvm_sw_cal.sc_usb_cal >> 0) & 0x1fu);
	if (transn == 0x1f)
		transn = 5;
	transp = (uint16_t)((maven_nvm_sw_cal.sc_usb_cal >> 5) & 0x1fu);
	if (transp == 0x1f)
		transp = 29;
	trim = (uint16_t)((maven_nvm_sw_cal.sc_usb_cal >> 10) & 0x7u);
	if (trim == 0x7)
		transp = 3;

	return USB_PADCAL_TRIM(trim) | USB_PADCAL_TRANSN(transn) |
	    USB_PADCAL_TRANSP(transp);
}
#endif

#if !defined(NDEBUG)
static const char *groupa_names[] = {
	/* GPIOA.0 */	"Target EXT7 (TxD)",
	/* GPIOA.1 */	"Target EXT9 (RxD)",
	/* GPIOA.2 */	"EXT4->EXT1 Bridge Enable",
	/* GPIOA.3 */	"USB VBUS",
#ifdef CONFIG_TARGET_CORTEXM
	/* GPIOA.4 */	"Target EXT4 (SWCLK)",
	/* GPIOA.5 */	"Target EXT5 (nRST)",
	/* GPIOA.6 */	"Target EXT1 (SWO)",
#else
	/* GPIOA.4 */	"Target EXT4 (ISP MOSI)",
	/* GPIOA.5 */	"Target EXT5 (ISP RST/PDI CLK)",
	/* GPIOA.6 */	"Target EXT1 (ISP MISO/PDI DAT)",
#endif
	/* GPIOA.7 */	"Target Vcc",
	/* GPIOA.8 */	"Target EXT3 Direction",
	/* GPIOA.9 */	"Target EXT4 Direction",
	/* GPIOA.10 */	"Target EXT1 Direction",
	/* GPIOA.11 */	NULL,
	/* GPIOA.12 */	"Network MISO",
	/* GPIOA.13 */	"Network SCK",
	/* GPIOA.14 */	"Network /CS",
	/* GPIOA.15 */	"Network MOSI",
	/* GPIOA.16 */	NULL,		/* Maven2: EXT8 Target Clock, TDI */
	/* GPIOA.17 */	"Network Enable",
	/* GPIOA.18 */	"Network /IRQ",
	/* GPIOA.19 */	"Network/Display /RST",
#ifdef CONFIG_TARGET_CORTEXM
	/* GPIOA.20 */	"Target EXT3 (SWDIO)",
#else
	/* GPIOA.20 */	"Target EXT3 (ISP SCK)",
#endif
	/* GPIOA.21 */	"Target EXT5 Direction",
	/* GPIOA.22 */	"Target EXT8 Direction",
	/* GPIOA.23 */	"Target EXT9 Direction",
	/* GPIOA.24 */	"USB D-",
	/* GPIOA.25 */	"USB D+",
	/* GPIOA.26 */	NULL,
	/* GPIOA.27 */	"Target EXT7 Direction",
	/* GPIOA.28 */	NULL,
	/* GPIOA.29 */	NULL,
	/* GPIOA.30 */	"SWCLK",
	/* GPIOA.31 */	"SWDIO"
};

static const char *groupb_names[] = {
	/* GPIOB.0 */	NULL,
	/* GPIOB.1 */	NULL,		/* Maven3: Power/Status LED */
	/* GPIOB.2 */	"Console TxD",
	/* GPIOB.3 */	"Console RxD",
	/* GPIOB.4 */	NULL,
	/* GPIOB.5 */	NULL,
	/* GPIOB.6 */	NULL,
	/* GPIOB.7 */	NULL,
	/* GPIOB.8 */	"Display MOSI",
	/* GPIOB.9 */	"Display SCK",
	/* GPIOB.10 */	"Display DCRS",
	/* GPIOB.11 */	"Display /CS",
	/* GPIOB.12 */	"WILC Debug TxD",/* Maven3 only */
	/* GPIOB.13 */	NULL,
	/* GPIOB.14 */	"WILC Debug RxD",/* Maven3 only */
	/* GPIOB.15 */	NULL,
	/* GPIOB.16 */	NULL,
	/* GPIOB.17 */	NULL,
	/* GPIOB.18 */	NULL,
	/* GPIOB.19 */	NULL,
	/* GPIOB.20 */	NULL,
	/* GPIOB.21 */	NULL,
	/* GPIOB.22 */	NULL,		/* Maven2: Power/Status LED */
	/* GPIOB.23 */	"Button",
	/* GPIOB.24 */	NULL,
	/* GPIOB.25 */	NULL,
	/* GPIOB.26 */	NULL,
	/* GPIOB.27 */	NULL,
	/* GPIOB.28 */	NULL,
	/* GPIOB.29 */	NULL,
	/* GPIOB.30 */	"SWO",		/* Maven3 only */
	/* GPIOB.31 */	NULL,		/* Maven3: EXT8 Target Clock, TDI */
};

const char *
sam_gpio_pin_name(char groupname, unsigned int pin)
{
	static const char ext8_name[] =
#ifdef CONFIG_TARGET_CORTEXM
	    "Target EXT8 (TDI)";
#else
	    "Target EXT8 (Rescue Clock)";
#endif

	if (groupname == 'A') {
		assert(pin <= 31);
#if (MAVEN_BOARD == 2)
		if (pin == 16)
			return ext8_name;
#endif
		return groupa_names[pin];
	} else
	if (groupname == 'B') {
		assert(pin <= 31);
#if (MAVEN_BOARD == 2)
		if (pin == 22)
			return "Power/Status LED";
#else
		if (pin == 1)
			return "Power/Status LED";
		else
		if (pin == 31)
			return ext8_name;
#endif
		return groupb_names[pin];
	}

	return NULL;
}
#endif /* !defined(NDEBUG) */

#if (PLATFORM_HAS_CPU_DELAY != 0)
static uint32_t
tcc_get_count(Tcc *tcc)
{

	tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
	tcc_wait_sync(tcc, TC_SYNCBUSY_CTRLB);
	tcc_wait_sync(tcc, TC_SYNCBUSY_COUNT);

	return tcc->COUNT.reg;
}

void
timer_cpu_delay_us(uint32_t us)
{
	Tcc *tcc = TCC1;
	uint32_t count;

	if (us == 0)
		return;

	us *= 16u;	/* Reference clock is 16 MHz */

	count = tcc_get_count(tcc);

	/* 'us' is target counter value */
	us += count;

	if ((us & 0xff000000u) != 0) {
		us &= 0x00ffffffu;
		/*
		 * Wait for count to wrap around.
		 */
		while ((count = tcc_get_count(tcc)) > us)
			;
	}

	/* Wait for count to catch up to target. */
	while (count < us)
		count = tcc_get_count(tcc);
}
#endif /* (PLATFORM_HAS_CPU_DELAY != 0) */

void platform_benchmark(void);
void
platform_benchmark(void)
{
	timer_timeout_t to;
	unsigned int loops;
	volatile uint32_t *m;
	void *b1, *b2;

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to))
		loops++;
	printf("Benchmark: buzzloops/sec %u\n", loops);

#define	BENCH_CHUNK	32768u
	b1 = malloc(BENCH_CHUNK);
	b2 = malloc(BENCH_CHUNK);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memset(b1, 0, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: RAM clear loops/sec %u\n", loops);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memcpy(b2, b1, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: RAM->RAM loops/sec %u\n", loops);

	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		memcpy(b1, (const void *)(uintptr_t)0x30000u, BENCH_CHUNK);
		loops++;
	}
	printf("Benchmark: ROM->RAM loops/sec %u\n", loops);

	m = (volatile uint32_t *)(uintptr_t)0x30000u;
	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		loops++;
		(void) *m++;
		if (m >= (volatile uint32_t *)(uintptr_t)(0x30000u + BENCH_CHUNK))
			m = (volatile uint32_t *)(uintptr_t)0x30000u;
	}
	printf("Benchmark: ROM read loops/sec %u\n", loops);

	m = (volatile uint32_t *)(uintptr_t)0x20000000u;
	timer_timeout_start(&to, 1000);
	loops = 0;
	while (!timer_timeout_expired(&to)) {
		loops++;
		(void) *m++;
		if (m >= (volatile uint32_t *)(uintptr_t)(0x20000000u + BENCH_CHUNK))
			m = (volatile uint32_t *)(uintptr_t)0x20000000u;
	}
	printf("Benchmark: RAM read loops/sec %u\n", loops);

	free(b1);
	free(b2);
}
