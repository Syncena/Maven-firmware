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
#include "fix16.h"
#include "timer.h"

/*
 * The target detect pin is connected to pin 9 of the target debug header.
 * This is normally grounded at the target end, so by enabling our internal
 * pull-up on the pin we can detect when it goes low, implying a target has
 * been connected.
 */
HAL_GPIO_PIN(TARGET_DETECT, 0, 22)

/*
 * ISP Ctrl signal, output, Cortex header pin 7.
 * Looks like this is a non-standard extension, using the unused "blank"
 * pin on the header. We don't do anything with it, other than enable
 * the built-in pulldown resistor so that the buffer between the LPC55
 * and the header is in a known good state. Many targets will tie the
 * pin to Gnd, so we *really* don't want the buffer to drive it high.
 */
HAL_GPIO_PIN(ISP_CTRL, 0, 14)

/*
 * Target Vcc
 */
HAL_GPIO_PIN(TARGET_VCC, 0, 31)
#define	MCULINK_TVCC_ADC_INPUT	3u
#define	MCULINK_TVCC_R_BRIDGE	4000.0	/* 4 KOhm resistor bridge. */
#define	MCULINK_TVCC_R_SENSE	2000.0	/* 2 KOhm sense resistor. */
#define	MCULINK_TVCC_CHANNEL	0u

/*
 * Min/Max values for Target Vcc
 */
#define	PWR_TVCC_MIN		1.7
#define	PWR_TVCC_MAX		5.6

/*
 * ADC reference voltage. Same as our Vcc...
 */
#define	PWR_VREF		3.3

/* This variable is updated regularly with TVcc */
fix16_t power_target_vcc;
int power_target_vcc_state = -1;

/*
 * Since MCULink is powered by USB, we assume our Vcc is always a nice
 * steady 3.30 volts. This is also the voltage on the VREFP pin, so
 * Target Vcc is measured with respect to it.
 */
fix16_t power_vddio = F16(PWR_VREF);
int power_vddio_state = 1;

struct power_state {
	ADC_Type *ps_adc;
	uint16_t ps_result;
	rtos_task_t ps_task;
};
static struct power_state power_state;

static void
power_adc_interrupt(void *arg)
{
	struct power_state *ps = arg;
	uint32_t result;

	result = ps->ps_adc->RESFIFO[0];

	/* Mask the result, and account for right-shift by 3 bits. */
	ps->ps_result = (uint16_t)(result & ADC_RESFIFO_D_MASK) >> 3;
	rtos_notify_give_isr(ps->ps_task);
}

/*
 * Convert a 12-bit ADC result into a voltage, reference to PWR_VREF.
 */
static fix16_t
mculink_adc_volts(fix16_t result)
{

	return fix16_mul(result, F16(PWR_VREF / 4096.0));
}

static void
mculink_handle_tvcc(struct power_state *ps)
{
	fix16_t v;

//	This is what we'd like to do, but it turns out there are targets
//	where debug header pin-9 is open...
//
//	if (HAL_GPIO_TARGET_DETECT_read() != 0) {
//		/*
//		 * No target connected. Clamp TVcc at 0.0.
//		 */
//		v = F16(0.0);
//	} else
	{
		/*
		 * Looks like a target is connected. The TVcc measurement
		 * should be valid.
		 */
		v = mculink_adc_volts(fix16_from_int(ps->ps_result));

		/* Correct for the voltage dividor... */
		v = fix16_mul(v, F16(MCULINK_TVCC_R_BRIDGE /
		    MCULINK_TVCC_R_SENSE));
	}

	power_target_vcc = v;

	if (v < F16(PWR_TVCC_MIN))
		power_target_vcc_state = -1;
	else
	if (v > F16(PWR_TVCC_MAX))
		power_target_vcc_state = 1;
	else
		power_target_vcc_state = 0;
}

static void
power_task(void *arg)
{
	struct power_state *ps = arg;

	ps->ps_adc->IE = ADC_IE_FWMIE0_MASK;

	for (;;) {
		/* Wait a bit between conversions. */
		rtos_task_sleep(150);

		/* Start the next conversion. */
		ps->ps_adc->SWTRIG = 1u << MCULINK_TVCC_CHANNEL;

		/* Wait for the result. */
		rtos_notify_take(1);

		/* Calculate TVcc. */
		mculink_handle_tvcc(ps);
	}
}

void
power_init(void)
{
	struct power_state *ps = &power_state;
	uint32_t gcca, gccb;
	timer_timeout_t to;
	ADC_Type *adc;
	void *ih;

	ps->ps_adc = adc = ADC0;

	/* Configure Target Detect pin and digital input, with pull-up. */
	HAL_GPIO_TARGET_DETECT_digimode(1);
	HAL_GPIO_TARGET_DETECT_in();
	HAL_GPIO_TARGET_DETECT_pullup(1);

	/* The ISP-CTRL pin is not used, but we enable the pull-down. */
	HAL_GPIO_ISP_CTRL_digimode(1);
	HAL_GPIO_ISP_CTRL_in();
	HAL_GPIO_ISP_CTRL_pulldown(1);

	/* Configure Target Vcc pin for analogue input. */
	HAL_GPIO_TARGET_VCC_pullup(0);
	HAL_GPIO_TARGET_VCC_pulldown(0);
	HAL_GPIO_TARGET_VCC_func(0);
	HAL_GPIO_TARGET_VCC_anamode(1);

	/* Final clock is MainCK / 8. */
	SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_RESET_MASK;
	SYSCON->ADCCLKDIV = SYSCON_ADCCLKDIV_DIV(7);

	/* ADC will be clocked from Main Clock. */
	SYSCON->ADCCLKSEL = SYSCON_ADCCLKSEL_SEL(0);

	/* Power-up Aux Bias (example calls this LDOGPADC ... */
	PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_AUXBIAS_MASK;

	/* Enable its clock. */
	mculink_clock_ctrl(0, SYSCON_AHBCLKCTRL0_ADC_MASK, true);

	/* Reset the ADC controller. */
	adc->CTRL |= ADC_CTRL_RST_MASK;
	adc->CTRL &= ~ADC_CTRL_RST_MASK;

	/* Reset FIFOs. */
	adc->CTRL |= ADC_CTRL_RSTFIFO0_MASK;
	adc->CTRL |= ADC_CTRL_RSTFIFO1_MASK;

	/* Follow documented power-up procedure. (Data sheet sec. 39.7.1.1) */
	adc->CTRL &= ~ADC_CTRL_ADCEN_MASK;
	adc->CTRL |= ADC_CTRL_DOZEN_MASK;	/* Disable Doze. */
	adc->CTRL |= ADC_CTRL_CAL_AVGS(7);	/* Average over 128 samples. */
	adc->CFG = ADC_CFG_PWREN_MASK |
	    ADC_CFG_PUDLY(128) |		/* Default power-up delay */
	    ADC_CFG_REFSEL(2) |			/* Reference on VREFP pin. */
	    ADC_CFG_PWRSEL(0) |			/* Default power setting. */
	    ADC_CFG_TPRICTRL(2);		/* Sensible trigger policy. */

	/* Pause between conversions. XXX: Experiment with this. */
	adc->PAUSE = ADC_PAUSE_PAUSEEN_MASK | ADC_PAUSE_PAUSEDLY(64);

	adc->FCTRL[0] = ADC_FCTRL_FWMARK(0);
	adc->FCTRL[1] = ADC_FCTRL_FWMARK(0);

	/* Now safe to enable the ADC. */
	adc->CTRL |= ADC_CTRL_ADCEN_MASK;

	/* Use the same offset trim values as the example code. */
	adc->OFSTRIM = ADC_OFSTRIM_OFSTRIM_A(0x10u) |
	    ADC_OFSTRIM_OFSTRIM_B(0x10u);

	/* Perform an auto-calibration before anything else. */
	adc->CTRL |= ADC_CTRL_CAL_REQ_MASK;

	timer_timeout_start(&to, 2000);

	do {
		rtos_task_sleep(10);
		gcca = adc->GCC[0];
		gccb = adc->GCC[1];
	} while (((gcca | gccb) & ADC_GCC_RDY_MASK) == 0 &&
	    !timer_timeout_expired(&to));

	if (!timer_timeout_expired(&to)) {
		gcca &= ADC_GCC_GAIN_CAL_MASK;
		gcca = ADC_GCR_GCALR((gcca << 16) / (0xffffu - gcca));

		gccb &= ADC_GCC_GAIN_CAL_MASK;
		gccb = ADC_GCR_GCALR((gccb << 16) / (0xffffu - gccb));

		adc->GCR[0] = gcca;
		adc->GCR[1] = gccb;
		adc->GCR[0] |= ADC_GCR_RDY_MASK;
		adc->GCR[1] |= ADC_GCR_RDY_MASK;

		timer_timeout_start(&to, 2000);
		while ((adc->STAT & ADC_STAT_CAL_RDY_MASK) == 0 &&
		    !timer_timeout_expired(&to)) {
			rtos_task_sleep(10);
		}
	}

	if (timer_timeout_expired(&to))
		printf("Warning: ADC auto-calibration timed out.\n");

	/*
	 * Configure channel for measuring TVcc.
	 */
	adc->CMD[MCULINK_TVCC_CHANNEL].CMDL =
	    ADC_CMDL_ADCH(MCULINK_TVCC_ADC_INPUT) |
	    ADC_CMDL_CTYPE(0) |
	    ADC_CMDL_MODE(0);
	adc->CMD[MCULINK_TVCC_CHANNEL].CMDH =
	    ADC_CMDH_NEXT(0) |
	    ADC_CMDH_LOOP(0) |
	    ADC_CMDH_AVGS(7) |
	    ADC_CMDH_STS(5) |
	    ADC_CMDH_CMPEN(0) |
	    ADC_CMDH_WAIT_TRIG_MASK;
	adc->TCTRL[MCULINK_TVCC_CHANNEL] =
	    ADC_TCTRL_TCMD(1u << MCULINK_TVCC_CHANNEL) |
	    ADC_TCTRL_TDLY(0) |
	    ADC_TCTRL_TPRI(0) |
	    ADC_TCTRL_FIFO_SEL_A(0) |
	    ADC_TCTRL_FIFO_SEL_B(0);

	power_target_vcc = F16(0.0);

	ih = rtos_irq_register_named("ADC", ADC0_IRQn, HW_IPL_ADC,
	    power_adc_interrupt, ps);
        assert(ih != NULL);
        (void)ih;

	ps->ps_task = rtos_task_create("Power", power_task, ps,
	    RTOS_TASK_PRIORITY_LOW, RTOS_STACK_SIZE(640));
        assert(ps->ps_task != NULL);
}
