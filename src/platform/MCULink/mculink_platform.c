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

#include "rtos.h"
#include "platform.h"
#include "platform_usb.h"
#include "hardware.h"
#include "lpc_flexcomm_usart.h"	/* For LPC_FLEXCOMM_MAX_CLOCK */
#include "mpu_common.h"
#include "serial_number.h"
#include "sys_event.h"
#include "timer.h"
#include "zone_alloc.h"
#include "fsl_power.h"

#define	FREQ_MHZ(f)		((f) * 1000u * 1000u)

#ifndef F_CPU
#error "F_CPU is not defined."
#endif
static_assert(F_CPU == FREQ_MHZ(100) || F_CPU == FREQ_MHZ(150),
    "F_CPU must either be 100 MHz or 150 MHz");

/* Unused I/O pins */
#define	MCL_UNUSED_PORT0	((1u << 4) | (1u << 6) | (1u << 7) | \
				 (1u << 8) | (1u << 15) | (1u << 16) | \
				 (1u << 17) | (1u << 20) | (1u << 21) | \
				 (1u << 23) | (1u << 26) | (1u << 29) | \
				 (1u << 30))
#define	MCL_UNUSED_PORT1	((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3))

#define	MPU_CODE_ENTRY(base, end)			 		\
	/* Execute OK, Read-only. */					\
	MPU_ENTRY(base, end, MPU_PERM_EXEC_OK, MPU_PERM_ANY_READ_ONLY,	\
		  MPU_ATTR_WRITE_THRU_RA, MPU_ATTR_WRITE_THRU_RA)

#define	MPU_EEPROM_ENTRY(base, end)			 		\
	/* No execute, Read-only. */					\
	MPU_ENTRY(base, end, MPU_PERM_NO_EXEC, MPU_PERM_ANY_READ_ONLY,	\
		  MPU_ATTR_WRITE_THRU_RA, MPU_ATTR_WRITE_THRU_RA)

#define	MPU_DATA_ENTRY(base, end)					\
	/* No execute, Read/Write. */					\
	MPU_ENTRY(base, end, MPU_PERM_NO_EXEC, MPU_PERM_ANY_READ_WRITE,	\
		  MPU_ATTR_WRITE_THRU_RA, MPU_ATTR_WRITE_THRU_RA)

#define	MPU_USB_SRAM_ENTRY(base, end)					\
	/* No execute, Read/Write, Unbuffered. */			\
	MPU_ENTRY(base, end, MPU_PERM_NO_EXEC, MPU_PERM_ANY_READ_WRITE,	\
		  MPU_ATTR_NON_CACHEABLE, MPU_ATTR_NON_CACHEABLE)

#define	MPU_DEVICE_ENTRY(base, end)					\
	/* No execute, Read/Write, Device. */				\
	MPU_ENTRY(base, end, MPU_PERM_NO_EXEC, MPU_PERM_ANY_READ_WRITE,	\
		  MPU_ATTR_DEVICE, MPU_DEVICE_nGnRnE)

static const struct mpu_entry mpu_table[] = {
#if (RELEASE_BUILD == 0)
	/*
	 * Code region, NS and S. Read-only, execute ok.
	 *
	 * Note: This does not include the first 4KB of Flash, so as to
	 * catch access via NULL pointers. The linker script will pad text
	 * to ensure no valid code is located here.
	 */
	MPU_CODE_ENTRY(0x00001000u, 0x1fffffffu),

	/* 272 KB for SRAM0-4. No execute, write-through. */
	MPU_DATA_ENTRY(0x20000000u, 0x20043fffu),
#endif

	/* Device memory. */
	MPU_DEVICE_ENTRY(0x40000000u, 0x400affffu),

	/* 16 KB for USB SRAM. No execute, write-through. */
	MPU_USB_SRAM_ENTRY(0x40100000u, 0x40103fffu)
};
#define	MPU_TABLE_COUNT	(sizeof(mpu_table) / sizeof(mpu_table[0]))

/*
 * Status LED
 */
HAL_GPIO_PIN(LED, 0, 5)
static rtos_timer_t mculink_led_timer;

zone_t platform_rtos_zone;

void
mculink_reset_ctrl(unsigned int which, uint32_t bit)
{

	SYSCON->PRESETCTRLSET[which] = bit;
	__DSB();
	while ((SYSCON->PRESETCTRLX[which] & bit) == 0)
		;

	timer_buzz_delay_us(100);

	SYSCON->PRESETCTRLCLR[which] = bit;
	__DSB();
	while ((SYSCON->PRESETCTRLX[which] & bit) != 0)
		;

	timer_buzz_delay_us(100);
}

void
mculink_clock_ctrl(unsigned int which, uint32_t bits, bool clk_on)
{

	if (clk_on)
		SYSCON->AHBCLKCTRLSET[which] = bits;
	else
		SYSCON->AHBCLKCTRLCLR[which] = bits;

	timer_buzz_delay_us(100);
}

static void
mculink_init_core_clock(void)
{

	/*
	 * The MCU-Link has a 16 MHz crystal connected to the XTAL32M pins.
	 * We want to clock the core at the max permissable rate of 150 MHz.
	 * Note: This is only attainable on 1B silicon; 0A is restricted to
	 * a max clock of 100 MHz.
	 *
	 * Note: On entry, this code assumes we're running on the FRO12 clock,
	 * so we're free to reconfigure XTAL32M and the PLL.
	 */

	/* Power up XTAL32M. */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_XTAL32M_MASK;
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_LDOXO32M_MASK;
	timer_buzz_delay_us(10);

	/* 'clkin' is provided from XTAL32M. */
	SYSCON->CLOCK_CTRL |= SYSCON_CLOCK_CTRL_CLKIN_ENA_MASK;

	/* Connect 'clkin' to system. */
	ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_SYSTEM_CLK_OUT_MASK;
	timer_buzz_delay_us(10);

	/* PLL0 clock source select is 'clkin'. */
	SYSCON->PLL0CLKSEL = SYSCON_PLL0CLKSEL_SEL(1);

#if (F_CPU == FREQ_MHZ(100))
	const uint32_t pllctrl = SYSCON_PLL0CTRL_SELI(53u) |
	    SYSCON_PLL0CTRL_SELP(26u) | SYSCON_PLL0CTRL_CLKEN_MASK;
	const uint32_t pllndec = SYSCON_PLL0NDEC_NDIV(4u);
	const uint32_t pllpdec = SYSCON_PLL0PDEC_PDIV(2u);
	const uint32_t pllsscg = SYSCON_PLL0SSCG1_MDIV_EXT(100u) |
	    SYSCON_PLL0SSCG1_SEL_EXT_MASK;
#else
	const uint32_t pllctrl = SYSCON_PLL0CTRL_SELI(53u) |
	    SYSCON_PLL0CTRL_SELP(31u) | SYSCON_PLL0CTRL_CLKEN_MASK;
	const uint32_t pllndec = SYSCON_PLL0NDEC_NDIV(8u);
	const uint32_t pllpdec = SYSCON_PLL0PDEC_PDIV(1u);
	const uint32_t pllsscg = SYSCON_PLL0SSCG1_MDIV_EXT(150u) |
	    SYSCON_PLL0SSCG1_SEL_EXT_MASK;
#endif

	/* Configure PLL0 for the required F_CPU frequency. */
	SYSCON->PLL0CTRL = pllctrl;
	SYSCON->PLL0NDEC = pllndec;
	SYSCON->PLL0NDEC = pllndec | SYSCON_PLL0NDEC_NREQ_MASK;
	SYSCON->PLL0PDEC = pllpdec;
	SYSCON->PLL0PDEC = pllpdec | SYSCON_PLL0PDEC_PREQ_MASK;
	SYSCON->PLL0SSCG0 = 0x0u;
	SYSCON->PLL0SSCG1 = pllsscg;
	SYSCON->PLL0SSCG1 = pllsscg | SYSCON_PLL0SSCG1_MD_REQ_MASK |
	    SYSCON_PLL0SSCG1_MREQ_MASK;

	/* Power up PLL0, and enable spread-spectrum module. */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_PLL0_MASK;
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_PLL0_SSCG_MASK;
	timer_buzz_delay_us(10);

	/* Wait for PLL0 to lock. */
	uint32_t r;
	do {
		r = SYSCON->PLL0STAT & SYSCON_PLL0STAT_LOCK_MASK;
	} while (r == 0u);

	/* NXP library call: set core LDO voltage to suit required clock. */
	POWER_SetVoltageForFreq(F_CPU);

	/* Set Flash wait-states. */
#if (F_CPU == FREQ_MHZ(100))
	const uint32_t wait_states = 0x8u;	/* 100 MHz core clock. */
#else
	const uint32_t wait_states = 0xbu;	/* 150 MHz core clock. */
#endif
	SYSCON->FMCCR = (SYSCON->FMCCR & ~SYSCON_FMCCR_FMCTIM_MASK) |
	    (wait_states << SYSCON_FMCCR_FMCTIM_SHIFT);
	__DSB();
	__ISB();

	/* Switch main clock source over to PLL0. */
	SYSCON->MAINCLKSELA = SYSCON_MAINCLKSELA_SEL(0); /* FRO12->A */
	__DSB();
	__ISB();
	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(1); /* PLL0->B */
	__DSB();
	__ISB();

	/* AHB clock is MAINCLK / 1. */
	SYSCON->AHBCLKDIV = SYSCON_AHBCLKDIV_DIV(0u);
	__DSB();
	__ISB();

	/* Configure PLL0_CLKDIV to provide a clock for Flexcomms. */
	SYSCON->PLL0CLKDIV = SYSCON_PLL0CLKDIV_RESET_MASK;
	__DSB();
	__ISB();
	SYSCON->PLL0CLKDIV = SYSCON_PLL0CLKDIV_DIV((F_CPU /
	    PLATFORM_FLEXCOMM_CLOCK) - 1u);
	__DSB();
	__ISB();
	timer_buzz_delay_us(10);

#if (F_CPU > FREQ_MHZ(100))
	/*
	 * Programming Flash (for configdb) needs to run at no more than
	 * 100 MHz. We don't want to divide PLL0 because FLEXCOMM relies
	 * on its output. Instead, we enable FRO96M to permit a quick
	 * switch-over while Flash is being programmed.
	 */
	mculink_reset_ctrl(2, SYSCON_PRESETCTRL2_ANALOG_CTRL_RST_SHIFT);
	mculink_clock_ctrl(2, SYSCON_AHBCLKCTRL2_ANALOG_CTRL_MASK, true);
	ANACTRL->FRO192M_CTRL |= ANACTRL_FRO192M_CTRL_ENA_96MHZCLK(1);
	__DSB();
	__ISB();

	/* Switch MAINCLKSELA over to FRO 96M */
	SYSCON->MAINCLKSELA = SYSCON_MAINCLKSELA_SEL(3);
	__DSB();
	__ISB();

	/* Flash code now just has to switch MAINCLKSELB. */
#endif
}

void
platform_early_init(void)
{
	extern int _siramxfunc, _sramxfunc, _eramxfunc;
	extern int _stack_top, _heap_end;
	volatile int *src, *dst;
	size_t hsize;

	/* Copy RAM-X code. */
	src = &_siramxfunc;
	dst = &_sramxfunc;
	while (dst < &_eramxfunc)
		*dst++ = *src++;
	__DSB();
	__ISB();

	hsize = (size_t)(((uintptr_t)&_heap_end) - ((uintptr_t)&_stack_top));

	(void) zone_create(NULL, (void *)&_stack_top, hsize, 0);

	/* RTOS zone is allocated in SRAM4. */
	platform_rtos_zone = zone_create_named("RTOS", NULL,
	    (void *)(uintptr_t)0x20040000u, 16u * 1024u,
	    ZONE_FLAGS_ASSERT_ALLOC_FAILURE);
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

	u  = *((uint32_t *)(uintptr_t)0x0009fc70);
	u ^= *((uint32_t *)(uintptr_t)0x0009fc74);
	u ^= *((uint32_t *)(uintptr_t)0x0009fc78);
	u ^= *((uint32_t *)(uintptr_t)0x0009fc7c);

	sn->zero = 0x00u;
	sprintf(sn->cstr, "MCL1ZZ%07" PRIX32, u >> 4);
	sn->checksum = 0x00u;
}

void
platform_init(void)
{
#if (RELEASE_BUILD != 0)
	extern int _sramfunc, _eramfunc;
	extern int _sramxfunc, _eramxfunc;
	extern int _data;
	extern int _eeprom_flash_start, _eeprom_flash_end;

	/* Code regions. Read-only, execute ok. */
	struct mpu_entry mpu_entry_ramx = 
	    MPU_CODE_ENTRY(&_sramxfunc, ((uintptr_t)&_eramxfunc) - 1);
	struct mpu_entry mpu_entry_code = 
	    MPU_CODE_ENTRY(&_sramfunc, ((uintptr_t)&_eramfunc) - 1);

	/* Data. No execute, write-through. */
	struct mpu_entry mpu_entry_data = MPU_DATA_ENTRY(&_data, 0x20043fffu);

	/* Mark Flash as read-only, no execute (except for first 4KB). */
	struct mpu_entry mpu_entry_flash =
	    MPU_EEPROM_ENTRY(0x00001000u,
	    ((uintptr_t)&_eeprom_flash_start) - 1);

	/* EEPROM emulation area. Read-only, no execute. */
	struct mpu_entry mpu_entry_eeprom = 
	    MPU_EEPROM_ENTRY(&_eeprom_flash_start,
	    ((uintptr_t)&_eeprom_flash_end) - 1);
#endif

	mculink_clock_ctrl(0, SYSCON_AHBCLKCTRL0_MUX0_MASK |
	    SYSCON_AHBCLKCTRL0_IOCON_MASK | SYSCON_AHBCLKCTRL0_GPIO0_MASK |
	    SYSCON_AHBCLKCTRL0_GPIO1_MASK, true);

	mculink_init_core_clock();

	/* Configure the MPU. */
	mpu_init(mpu_table, MPU_TABLE_COUNT);

#if (RELEASE_BUILD != 0)
	mpu_add_region(&mpu_entry_ramx);
	mpu_add_region(&mpu_entry_code);
	mpu_add_region(&mpu_entry_data);
	mpu_add_region(&mpu_entry_flash);
	mpu_add_region(&mpu_entry_eeprom);
#endif

	mpu_enable();
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

static void
mculink_unused_pin(unsigned int group, uint32_t mask)
{
	int pin;

	if (mask == 0)
		return;

	/* Configure pins as inputs. */
	GPIO->DIRCLR[group] = mask;

	while ((pin = ffs(mask)) != 0) {
		pin -= 1;
		mask &= ~(1u << pin);
		IOCON->PIO[group][pin] = IOCON_PIO_MODE(2);	/* Pullup */
	}
}

static void
mculink_led_timer_cb(rtos_timer_t t)
{

	(void) t;

	HAL_GPIO_LED_set();
}

static void
mculink_sys_event(void *arg, sys_event_t ev, const void *ev_arg)
{

	(void)arg;
	(void)ev_arg;

	switch (ev) {
	case SYS_EVENT_TMON_SCANNING:
		HAL_GPIO_LED_clr();
		rtos_timer_start(mculink_led_timer);
		break;

	case SYS_EVENT_TMON_ATTACHED:
		rtos_timer_stop(mculink_led_timer);
		HAL_GPIO_LED_clr();
		break;

	case SYS_EVENT_TMON_DISCONNECT:
		rtos_timer_stop(mculink_led_timer);
		HAL_GPIO_LED_set();
		break;

	default:
		break;
	}
}

void
platform_attach_devices(void)
{
	sys_event_handle_t eh;

	/*
	 * Enable pullups on unused pins
	 */
	mculink_unused_pin(0, MCL_UNUSED_PORT0);
	mculink_unused_pin(1, MCL_UNUSED_PORT1);

	HAL_GPIO_LED_digimode(1);
	HAL_GPIO_LED_out();
	HAL_GPIO_LED_set();     /* LED off */

	mculink_led_timer = rtos_timer_create("led", 250, 0,
	    mculink_led_timer_cb, NULL);
	assert(mculink_led_timer != NULL);

	/* Register a system event callback to control LED. */
	eh = sys_event_register(mculink_sys_event, NULL);
	assert(eh != NULL);
	(void) eh;

	mculink_usb_init();
}

void
platform_hw_init_target(void)
{

	/*
	 * Enable the digital input bufffers
	 */
	HAL_GPIO_EXT1_digimode(1);
	HAL_GPIO_EXT3_digimode(1);
	HAL_GPIO_EXT4_digimode(1);
	HAL_GPIO_EXT5_digimode(1);
	HAL_GPIO_EXT7_digimode(1);
	HAL_GPIO_EXT8_digimode(1);
	HAL_GPIO_EXT9_digimode(1);

	HAL_GPIO_EXT3_DIR_digimode(1);
	HAL_GPIO_EXT5_DIR_digimode(1);
}

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
