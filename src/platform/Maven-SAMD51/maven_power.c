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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "sam_clocks.h"
#include "display.h"
#include "timer.h"
#include "fix16.h"

/*
 * ToDo:
 *  - Periodically save ps->ps_gain_corr in CONFIG-DB, but without
 *    forcing a persist. It'll be saved the next time the user
 *    saves configuration (probably when provisioning WiFi). This
 *    will reduce the time needed for convergence.
 *  - Figure out some way to do much the same for the zero offset.
 */

#define	PWR_ADC_REGS			ADC0
#define	PWR_DEFAULT_GAIN_CORRECTION	0.99824
#define	PWR_DEFAULT_ZERO_OFFSET		265.0
#define	PWR_REDUCE_FAC			16.0

/*
 * Target Vcc support.
 *
 * We have a voltage divider consisting of two resistors between TargetVcc
 * and Gnd. Our ADC input measures the voltage across the lower 'Sense'
 * resistor.
 */
#define	PWR_BRIDGE_R		51700.0	/* Total resistance */
#define	PWR_BRIDGE_R_SENSE	4700.0	/* Sense resistance */

/*
 * We use the internal 1.0 volt reference.
 */
#define	PWR_ADC_REFERENCE	SUPC_VREF_SEL_1V0
#define	PWR_ADC_VREF		1.0

/*
 * Minimum acceptable VDDio value.
 */
#define	PWR_VDDIO_MIN		2.9

/*
 * Min/Max values for Target Vcc
 */
#define	PWR_TVCC_MIN		1.7
#define	PWR_TVCC_MAX		5.6

/*
 * Target Vcc
 */
HAL_GPIO_PIN(TARGET_VCC, A, 7)

/*
 * USB Vbus
 */
HAL_GPIO_PIN(USB_VBUS, A, 3)

/*
 * Power/Status LED
 */
#if (MAVEN_BOARD == 2)
HAL_GPIO_PIN(LED, B, 22)
#else
HAL_GPIO_PIN(LED, B, 1)
#endif

/* This variable is updated regularly with TVcc */
fix16_t power_target_vcc;
int power_target_vcc_state = -1;

/*
 * This variable is updated regularly with VDDIO (our Vcc)
 */
fix16_t power_vddio;
int power_vddio_state;

struct power_state {
	Adc *ps_regs;
	int16_t ps_result;
	int16_t ps_toss;
	uint8_t ps_led_saved_state;
	uint8_t ps_power_bad;
#define	POWER_BAD_MAVEN		(1u << 0)
#define	POWER_BAD_TARGET	(1u << 1)

	fix16_t ps_vcore_acc;
	fix16_t ps_vbus;

	fix16_t ps_zero_offset;
	fix16_t ps_gain_corr;

	rtos_task_t ps_task;
};
static struct power_state power_state;

struct power_mux {
	uint16_t pm_mux;
	uint16_t pm_flags;
	void (*pm_handler)(struct power_state *, fix16_t);
};
#define	POWER_MUX_FLAG_VOLTS_CONVERSION		(1u << 0)
#define	POWER_MUX_FLAG_OFFSET_CORRECTION	(1u << 1)
#define	POWER_MUX_FLAG_GAIN_CORRECTION		(1u << 2)
#define	POWER_MUX_FLAG_RESISTOR_BRIDGE		(1u << 3)
#define	POWER_MUX_FLAG_QUARTER_VOLTS		(1u << 4)

static void power_handle_vddio(struct power_state *, fix16_t);
static void power_handle_vddcore(struct power_state *, fix16_t);
static void power_handle_tvcc(struct power_state *, fix16_t);
static void power_handle_vbus(struct power_state *, fix16_t);

static const struct power_mux power_mux[] = {
	{ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC | ADC_INPUTCTRL_MUXNEG_GND,
	 POWER_MUX_FLAG_OFFSET_CORRECTION | POWER_MUX_FLAG_VOLTS_CONVERSION |
	 POWER_MUX_FLAG_QUARTER_VOLTS | POWER_MUX_FLAG_GAIN_CORRECTION,
	 power_handle_vddio},

	{ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC | ADC_INPUTCTRL_MUXNEG_GND,
	 POWER_MUX_FLAG_OFFSET_CORRECTION | POWER_MUX_FLAG_VOLTS_CONVERSION |
	 POWER_MUX_FLAG_QUARTER_VOLTS | POWER_MUX_FLAG_GAIN_CORRECTION,
	 power_handle_vddcore},

	{ADC_INPUTCTRL_MUXPOS_AIN7 | ADC_INPUTCTRL_MUXNEG_GND,
	 POWER_MUX_FLAG_OFFSET_CORRECTION | POWER_MUX_FLAG_VOLTS_CONVERSION |
	 POWER_MUX_FLAG_GAIN_CORRECTION | POWER_MUX_FLAG_RESISTOR_BRIDGE,
	 power_handle_tvcc},

	{ADC_INPUTCTRL_MUXPOS_AIN1 | ADC_INPUTCTRL_MUXNEG_GND,
	 POWER_MUX_FLAG_OFFSET_CORRECTION | POWER_MUX_FLAG_VOLTS_CONVERSION |
	 POWER_MUX_FLAG_GAIN_CORRECTION | POWER_MUX_FLAG_RESISTOR_BRIDGE,
	power_handle_vbus}
};
#define	POWER_MUX_COUNT	(sizeof(power_mux) / sizeof(power_mux[0]))

static void
power_adc_waitsync(Adc *r, uint32_t mask)
{

	while ((r->SYNCBUSY.reg & mask) != 0)
		;
}

static void
power_adc_interrupt(void *arg)
{
	struct power_state *ps = arg;
	uint16_t result;

	result = ps->ps_regs->RESULT.reg;

	if (ps->ps_toss-- > 0) {
		ps->ps_regs->SWTRIG.reg = ADC_SWTRIG_START;
		power_adc_waitsync(ps->ps_regs, ADC_SYNCBUSY_SWTRIG);
		return;
	}

	ps->ps_result = (int16_t)result;

	rtos_notify_give_isr(ps->ps_task);
}

static fix16_t
power_calc_bridge_volts(fix16_t vsense)
{

	return fix16_mul(vsense, F16(PWR_BRIDGE_R / PWR_BRIDGE_R_SENSE));
}

static fix16_t
power_vcore(struct power_state *ps)
{

	return fix16_div(ps->ps_vcore_acc, F16(PWR_REDUCE_FAC));
}

static fix16_t
power_adc_volts(fix16_t val_x8, bool div4)
{
	fix16_t val;

	val_x8 = fix16_div(val_x8, F16(4096.0));

	if (div4)
		val = fix16_mul(val_x8, F16(PWR_ADC_VREF / 2.0));
	else
		val = fix16_mul(val_x8, F16(PWR_ADC_VREF / 8.0));

	return val;
}

static void
power_dump_values(struct power_state *ps)
{
#if 0
	char str[16];

#define	FIX_PREC	2

//	fix16_to_str(ps->ps_zero_offset, str, FIX_PREC);
//	printf("Zero: %s, ", str);

	fix16_to_str(power_vddio, str, FIX_PREC);
	printf("VDDio: %s, ", str);

	fix16_to_str(power_vcore(ps), str, FIX_PREC);
	printf("VCore: %s, ", str);

	fix16_to_str(power_target_vcc, str, FIX_PREC);
	printf("TVcc: %s, ", str);

	fix16_to_str(ps->ps_vbus, str, FIX_PREC);
	printf("VBus: %s\n", str);
#else
	(void) ps;
#endif
}

static void
power_led_state(uint8_t state)
{

	if (state)
		HAL_GPIO_LED_clr();
	else
		HAL_GPIO_LED_set();
}

static void
power_led_toggle(void)
{

	HAL_GPIO_LED_toggle();
}

static void
power_thread(void *cookie)
{
	struct power_state *ps = cookie;
	const struct power_mux *pm;
	uint8_t last_power_bad;
	fix16_t result;
	int count;

	pm = power_mux;

	HAL_GPIO_LED_out();

	power_led_state(1);
	last_power_bad = ps->ps_power_bad = 0;
	ps->ps_led_saved_state = 1;
	count = 0;

	for (;;) {
		/* Wait a bit between conversions. */
		rtos_task_sleep(150);

		/* Start the next conversion. */
		ps->ps_toss = 6;
		ps->ps_regs->INPUTCTRL.reg = pm->pm_mux;
		power_adc_waitsync(ps->ps_regs, ADC_SYNCBUSY_INPUTCTRL);
		ps->ps_regs->SWTRIG.reg = ADC_SWTRIG_START;
		power_adc_waitsync(ps->ps_regs, ADC_SYNCBUSY_SWTRIG);

		/* Wait for the result to be ready. */
		rtos_notify_take(1);
		assert(ps->ps_toss < 0);

		result = fix16_from_int((int)ps->ps_result);

		if ((pm->pm_mux & ADC_INPUTCTRL_DIFFMODE) == 0) {
			/* Assert result is positive. */
			assert(result >= F16(0.0));
		}

		if (pm->pm_flags & POWER_MUX_FLAG_OFFSET_CORRECTION) {
			/*
			 * Apply the ADC zero offset correction.
			 */
			result -= ps->ps_zero_offset;

			if ((pm->pm_mux & ADC_INPUTCTRL_DIFFMODE) == 0 &&
			    result < F16(0.0)) {
				result = F16(0.0);
			}
		}

		if (pm->pm_flags & POWER_MUX_FLAG_VOLTS_CONVERSION) {
			/*
			 * Convert to voltage as measured by the ADC, after
			 * accounting for the x8 average.
			 */
			bool div4 =
			    (pm->pm_flags & POWER_MUX_FLAG_QUARTER_VOLTS) != 0;

			result = power_adc_volts(result, div4);

			if (pm->pm_flags & POWER_MUX_FLAG_RESISTOR_BRIDGE)
				result = power_calc_bridge_volts(result);
		}

		if (pm->pm_flags & POWER_MUX_FLAG_GAIN_CORRECTION)
			result = fix16_mul(result, ps->ps_gain_corr);

		/* Invoke the mux-specific handler to deal with it. */
		(pm->pm_handler)(ps, result);

		/* Advance to the next mux. */
		if (++pm == &power_mux[POWER_MUX_COUNT]) {
			pm = power_mux;
			power_dump_values(ps);
		}

		if (last_power_bad != ps->ps_power_bad) {
			if (ps->ps_power_bad == 0) {
				count = 0;
				power_led_state(ps->ps_led_saved_state);
			} else
			if (last_power_bad == 0)
				power_led_state(1);	/* LED on */

			last_power_bad = ps->ps_power_bad;
		}

		if (ps->ps_power_bad && ++count == 2)
			power_led_toggle();
	}
}

static void
power_handle_vddio(struct power_state *ps, fix16_t volts)
{

	power_vddio = volts;
	power_vddio_state = power_vddio >= F16(PWR_VDDIO_MIN);

	if (power_vddio_state == 0)
		ps->ps_power_bad |= POWER_BAD_MAVEN;
	else
		ps->ps_power_bad &= ~POWER_BAD_MAVEN;
}

static void
power_handle_vddcore(struct power_state *ps, fix16_t volts)
{

	if (ps->ps_vcore_acc == F16(0.0)) {
		ps->ps_vcore_acc = fix16_mul(volts, F16(PWR_REDUCE_FAC));
	} else {
		ps->ps_vcore_acc -= power_vcore(ps);
		ps->ps_vcore_acc += volts;
	}

	ps->ps_gain_corr = fix16_div(F16(1.2), power_vcore(ps));
}

static void
power_handle_tvcc(struct power_state *ps, fix16_t volts)
{

	(void) ps;

	power_target_vcc = volts;

	if (volts < F16(PWR_TVCC_MIN))
		power_target_vcc_state = -1;
	else
	if (volts > F16(PWR_TVCC_MAX))
		power_target_vcc_state = 1;
	else
		power_target_vcc_state = 0;

	if (power_target_vcc_state > 0)
		ps->ps_power_bad |= POWER_BAD_TARGET;
	else
		ps->ps_power_bad &= ~POWER_BAD_TARGET;
}

static void
power_handle_vbus(struct power_state *ps, fix16_t volts)
{

	ps->ps_vbus = volts;
}

fix16_t
power_vbus(void)
{
	struct power_state *ps = &power_state;

	return ps->ps_vbus;
}

void
power_init(void)
{
	struct power_state *ps = &power_state;
	Adc *r = PWR_ADC_REGS;
	unsigned int gen, clock;
	uint32_t id;
	void *ih;

	/* Enable the ADC */
	id = (r == ADC0) ? ID_ADC0 : ID_ADC1;
	sam_periph_clock_enable(id);

	/*
	 * The ADC needs a sample clock. Min is 160 KHz, max is 16 MHz. The
	 * minimum prescaler is /2, so the input clock range is 320 KHz to
	 * 32 MHz. The actual sample rate, for 12-bit resolution, is
	 * 'sample clock / 16'.
	 *
	 * Practicaly, we'll feed the ADC with a 60 MHz input clock. This
	 * will be divided by 128 to provide a 468.75 KHz sample clock,
	 * which is good for around 30,000 samples/second.
	 */

	/* Allocate a clock generator. */
	gen = sam_gclk_gen_alloc(SAM_GCLK_GEN_TYPE_ANY);
	assert(gen > 0);

	/* 60 MHz clock, please. */
	clock = sam_gclk_gen_config(gen, SAM_GCLK_GEN_SRC_DPLL0,
	    SAM_GCLK_GEN_DIVSEL_DIRECT,
	    sam_dpll_frequencies[0] / 60000000u, 0);
	assert(clock >= 320000u && clock <= 100000000u);
	(void)clock;

	/* Route the clock to the appropriate ADC. */
	(void) sam_gclk_periph_chan_enable((r == ADC0) ? ADC0_GCLK_ID :
	    ADC1_GCLK_ID, gen);

	/* Ensure the ADC is disabled. */
	if ((r->CTRLA.reg & ADC_CTRLA_ENABLE) != 0) {
		r->CTRLA.reg = 0;
		power_adc_waitsync(r, ADC_SYNCBUSY_ENABLE);
	}

	/* ADC software reset. */
	r->CTRLA.reg = ADC_CTRLA_SWRST;
	power_adc_waitsync(r, ADC_SYNCBUSY_SWRST);
	while ((r->CTRLA.reg & ADC_CTRLA_SWRST) != 0)
		;

	/* Go ahead and configure the ADC */

	/* Configure sampling clock */
	r->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV128 | ADC_CTRLA_RUNSTDBY;

	/* Use the internal reference from SUPV */
	r->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTREF | ADC_REFCTRL_REFCOMP;
	power_adc_waitsync(r, ADC_SYNCBUSY_REFCTRL);

	/* Take 1 cycle per sample. */
	r->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0) | ADC_SAMPCTRL_OFFCOMP;
	power_adc_waitsync(r, ADC_SYNCBUSY_SAMPCTRL);

	/* Average 8 samples. */
	r->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8;
	power_adc_waitsync(r, ADC_SYNCBUSY_AVGCTRL);

	/* Result is 16 bits. */
	r->CTRLB.reg = ADC_CTRLB_RESSEL_16BIT;
	power_adc_waitsync(r, ADC_SYNCBUSY_CTRLB);

	/* Fetch and configure calibration values. */
	r->CALIB.reg = maven_get_adc_calib(id);
	power_adc_waitsync(r, ADC_SYNCBUSY_MASK);

	/* XXX: Configure GAINCORR and OFFSETCORR rergisters. */

	/* Enable the ADC. */
	r->CTRLA.reg |= ADC_CTRLA_ENABLE;
	power_adc_waitsync(r, ADC_SYNCBUSY_ENABLE);

	/* Enable the internal analogue reference. */
	SUPC->VREF.reg = SUPC_VREF_VREFOE | SUPC_VREF_RUNSTDBY |
	    PWR_ADC_REFERENCE;

	ps->ps_regs = r;
	ps->ps_vbus = F16(0.0);
	ps->ps_vcore_acc = F16(0.0);
	ps->ps_zero_offset = F16(PWR_DEFAULT_ZERO_OFFSET);
	ps->ps_gain_corr = F16(PWR_DEFAULT_GAIN_CORRECTION);

	HAL_GPIO_TARGET_VCC_pmux('B');
	HAL_GPIO_USB_VBUS_pmux('B');

	/* Hook the ADC 'result ready' interrupt. */
	ih = rtos_irq_register_named("ADC", (r == ADC0) ? ADC0_1_IRQn :
	    ADC1_1_IRQn, HW_IPL_ADC, power_adc_interrupt, ps);
	assert(ih != NULL);
	(void)ih;

	r->INTENSET.reg = ADC_INTENSET_RESRDY;

	/*
	 * XXX: Configure the Analogue Comparator for continuous monitoring of
	 * TVcc. Saves us polling it at high frequency.
	 */

	power_target_vcc = F16(0.0);
	power_vddio = F16(0.0);
	power_vddio_state = -1;

	ps->ps_task = rtos_task_create("Power", power_thread, ps,
	    RTOS_TASK_PRIORITY_LOW, RTOS_STACK_SIZE(640));
	assert(ps->ps_task != NULL);
}

void
platform_wifi_led_init(void)
{
}

void
platform_wifi_led_set(uint8_t state)
{

	power_state.ps_led_saved_state = state;

	if (power_state.ps_power_bad != 0)
		return;

	power_led_state(state);
}
