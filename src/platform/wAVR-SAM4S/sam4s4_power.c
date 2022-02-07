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

#include <stdio.h>
#include <stdlib.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "timer.h"

/*
 * Target Vcc support.
 *
 * We have a voltage divider consisting of two identical resistors
 * between TargetVcc and Gnd. Our AD0 pin measures the voltage
 * across the lower resistor. This is basically TargetVcc / 2.
 *
 * The target's Vcc is connected via a diode with a forward voltage
 * drop of approximately 150 mV. This rises to around 300 mV if
 * Maven is parisitising the target supply, but we don't currently
 * have a method to determine when we're drawing power in this way.
 */
/* We use a simple rolling-average filter for Target Vcc */
#define	SUPV_TARGET_VCC_REDUCE_FACTOR	4

/* Run the ADC at 6.4MHz (it's what the examples do!) */
#define	SUPV_ADC_CLOCK		6400000u

/* Our reference is 2.5 volts. */
#define	SUPV_ADC_VREF		2.5

/* Sample Target Vcc every 200 mS */
#define	SUPV_TVCC_SAMPLE_PERIOD	200

/* Forward voltage drop of Target Vcc diode */
#define	SUPV_TVCC_VF_LOW	0.150

/* Min/Max limits of Target Vcc */
#define TARGET_VCC_MIN		F16(1.7)
#define TARGET_VCC_MAX		F16(5.5)
#define TARGET_VCC_COUNT	4

/*
 * Supply Monitor Parameters
 *
 * The SAM4S' ADC is not capable of measuring VDDIO directly, but the
 * Supply Monitor can be persuaded to generate an interrupt if VDDIO
 * drops below a configurable threshold. So with the combination of a
 * threshold interrupt and a regular timer, we can deduce and average
 * the current VDDIO.
 */
/* Interrupt priority for the Supply Monitor interrupt */
#define	SUPV_SUPMON_IRQ_IPL		HW_IPL_ADC

/* We use a simple rolling-average filter */
#define	SUPV_SUPMON_VCC_REDUCE_FACTOR	4

/* How often to sample the VDDIO threshold */
#define	SUPV_VDDIO_SAMPLE_PERIOD	251

/* Measured VDDIO is around 100 mV lower than actual. */
#define	SUPV_VDDIO_COMPENSATE		0.1

struct power_state {
	uint16_t ss_adc_count;
	uint16_t ss_adc_raw;
	fix16_t ss_adc_target_vcc;
	rtos_timer_t ss_adc_timer;

	int ss_supmon_vcc_first;
	fix16_t ss_supmon_vcc;
	rtos_timer_t ss_supmon_timer;
};
static struct power_state power_state;

/* This variable is updated once per second with TVcc */
fix16_t power_target_vcc;
int power_target_vcc_state = -1;

/*
 * This variable is updated once per second with VDDIO (our Vcc)
 */
fix16_t power_vddio;
int power_vddio_state = -1;

/*
 * ADC sample-complete interrupt handler
 */
static void
power_adc_interrupt(void *arg)
{
	struct power_state *ss = arg;
	Adc *adc = ADC;

	/* Clear the interrupt status register */
	(void) adc->ADC_ISR;

	/* Grab the raw ADC value for this sample */
	ss->ss_adc_raw = (uint16_t)adc->ADC_LCDR;

	/* Disable any further ADC interrupts */
	adc->ADC_IDR = ADC_IDR_DRDY;

	/* Start the timer */
	rtos_timer_start_isr(ss->ss_adc_timer);
}

/*
 * Invoked when the ADC timer expires
 */
static void
power_target_vcc_scheduler(rtos_timer_t handle)
{
	struct power_state *ss = rtos_timer_get_id(handle);
	Adc *adc = ADC;
	fix16_t v, avg;

	/*
	 * Compute the instantaneous target Vcc, accounting for the voltage
	 * divider (/2).
	 */
	v = fix16_mul(F16(SUPV_ADC_VREF),
	    fix16_from_int((int)(ss->ss_adc_raw & ADC_LCDR_LDATA_Msk)));
	v = fix16_div(v, F16(2048.0));

	/* Fold the value into the rolling average */
	if (ss->ss_adc_count == 0) {
		/* Special case at startup */
		ss->ss_adc_target_vcc = fix16_mul(v,
		    F16(SUPV_TARGET_VCC_REDUCE_FACTOR));
		ss->ss_adc_count = 1;
	} else {
		/* The rolling-average filter */
		avg = fix16_div(ss->ss_adc_target_vcc,
		    F16(SUPV_TARGET_VCC_REDUCE_FACTOR));
		ss->ss_adc_target_vcc = fix16_add(v,
		    fix16_sub(ss->ss_adc_target_vcc, avg));
		v = fix16_div(ss->ss_adc_target_vcc,
		    F16(SUPV_TARGET_VCC_REDUCE_FACTOR));
	}

	/* Update the global Target Vcc variables with the new value */
	power_target_vcc = fix16_add(v, F16(SUPV_TVCC_VF_LOW));

	if (power_target_vcc >= TARGET_VCC_MIN &&
	    power_target_vcc <= TARGET_VCC_MAX) {
		power_target_vcc_state = 0;
	} else {
		power_target_vcc_state = (power_target_vcc < TARGET_VCC_MIN) ?
		    (-1) : 1;
	}

	/* Start the ADC again for the next sample */
	(void) adc->ADC_LCDR;
	adc->ADC_IER = ADC_IER_DRDY;
	adc->ADC_CR = ADC_CR_START;
}

static void
power_supmon_timer(rtos_timer_t handle)
{
	struct power_state *ss = rtos_timer_get_id(handle);
	uint32_t smth;
	fix16_t v, avg;

	smth = SUPC_SMMR_SMTH(SUPC->SUPC_SMMR);

	/* Compute the current threshold */
	v = fix16_from_int((int)(smth * 128));
	v = fix16_mul(v, F16(0.12));
	v = fix16_add(v, F16(1.6 * 128.0));

	/* Fold the value into the rolling average */
	if (ss->ss_supmon_vcc_first == 0) {
		/* Special case at startup */
		ss->ss_supmon_vcc = fix16_mul(v,
		    F16(SUPV_SUPMON_VCC_REDUCE_FACTOR));
		ss->ss_supmon_vcc_first = 1;
	} else {
		/* The rolling-average filter */
		avg = fix16_div(ss->ss_supmon_vcc,
		    F16(SUPV_SUPMON_VCC_REDUCE_FACTOR));
		ss->ss_supmon_vcc = fix16_add(v,
		    fix16_sub(ss->ss_supmon_vcc, avg));
		v = fix16_div(ss->ss_supmon_vcc,
		    F16(SUPV_SUPMON_VCC_REDUCE_FACTOR));
		if (ss->ss_supmon_vcc_first < 5)
			ss->ss_supmon_vcc_first += 1;
	}

	/* Update the global Vcc variable with the new value */
	v = fix16_div(v, F16(128.0));
	v = fix16_add(v, F16(SUPV_VDDIO_COMPENSATE));
	power_vddio = v;

	if (ss->ss_supmon_vcc_first >= 5)
		power_vddio_state = (v >= F16(2.7));
}

void
power_init(void)
{
	struct power_state *ss = &power_state;
	Adc *adc = ADC;
	int i;

	/*
	 * Ensure pull-up is disabled on the Tvcc analogue pin.
	 * It seems clear from the docs that the peripheral does not
	 * do this when it assumes control of the pin.
	 */
	HAL_GPIO_TARGET_VCC_pullup(0);

	/*
	 * Initialise the ADC. This is based on a combination of datasheet
	 * and the ASF example code.
	 */

	/* ADC Software reset */
	adc->ADC_CR = ADC_CR_SWRST;

	/* Get the mode register to a known value */
	adc->ADC_MR = 0;

	/* Clear the PDC, dunno why - examples do it though */
	adc->ADC_PTCR = (ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS);
	adc->ADC_RCR = 0;
	adc->ADC_RNCR = 0;

	/*
	 * Configure the ADC mode.
	 *  - Manual (software) trigger.
	 *  - Always on
	 *  - ADC clock set to SUPV_ADC_CLOCK
	 *  - Startup time 64 cycles
	 *  - Settling time 17uS
	 *  - Single diff/gain/off
	 *  - Track time of 1
	 *  - Transfer time of 1 (datasheet recommends 2, example uses 1)
	 *  - Don't use sequence table.
	 */
	adc->ADC_MR |= ADC_MR_TRGEN_DIS | ADC_MR_SLEEP_NORMAL |
	    ADC_MR_FREERUN_OFF |
	    (((F_CPU / (2 * SUPV_ADC_CLOCK)) - 1) << ADC_MR_PRESCAL_Pos) |
	    ADC_MR_STARTUP_SUT64 | ADC_MR_SETTLING_AST17 | ADC_MR_ANACH_NONE |
	    (1u << ADC_MR_TRACKTIM_Pos) | (1u << ADC_MR_TRANSFER_Pos) |
	    ADC_MR_USEQ_NUM_ORDER;

	/* Enable bias, as per the datasheet - ADCclk > 1MHz */
	adc->ADC_ACR = 1 << ADC_ACR_IBCTL_Pos;

	/* Enable ADC channel */
	adc->ADC_CHER = TARGET_VCC_AD_CHANNEL;

	/* Initiate an auto-calibration */
	adc->ADC_CR = ADC_CR_AUTOCAL;

	/* Wait up to one second for the calibration to succeed */
	for (i = 0; i < 100; i++) {
		if ((adc->ADC_ISR & ADC_ISR_EOCAL) != 0)
			break;
		timer_buzz_delay_us(10);
	}

	/* If this fails, is the ADC toast? */
	if (i == 100)
		printf("ADC autocal timed out\n");

	/*
	 * Monitor Target Vcc using a 1-second timer
	 */
	ss->ss_adc_timer = rtos_timer_create("TVcc", SUPV_TVCC_SAMPLE_PERIOD, 0,
	    power_target_vcc_scheduler, ss);
	assert(ss->ss_adc_timer != NULL);

	/* Hook the ADC interrupt */
	rtos_irq_register_named("TVcc", ADC_IRQn, HW_IPL_ADC,
	    power_adc_interrupt, ss);

	/* Enable ADC data ready interrupt */
	adc->ADC_IER = ADC_IER_DRDY;

	/* Start the first conversion */
	adc->ADC_CR = ADC_CR_START;

	/*
	 * Monitor our own Vcc using a 1-second timer and
	 * Supply Monitor interrupt
	 */
	ss->ss_supmon_timer = rtos_timer_create("VDDIO",
	    SUPV_VDDIO_SAMPLE_PERIOD, 1, power_supmon_timer, ss);
	assert(ss->ss_supmon_timer != NULL);
	rtos_timer_start(ss->ss_supmon_timer);

	/*
	 * Start the supply monitor.
	 */
	SUPC->SUPC_SMMR = SUPC_SMMR_SMSMPL_CSM;
}
