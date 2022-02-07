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

#include <stdint.h>
#include <stdlib.h>

#include "avr_ll.h"
#include "hardware.h"
#include "platform.h"
#include "timer.h"
#include "rtos.h"

void
avr_ll_init_target(void)
{

	/*
	 * Configure the external buffers
	 */
	hw_init();

	/* ISP RST Signal */
	HW_PIN_ISP_RST_pulldown(1);	/* ISP RST GPIO pin pull-down on */
	HW_PIN_ISP_RST_pullup(0);	/* ISP RST GPIO pin pull-up off */
	HW_PIN_ISP_RST_in();		/* ISP RST GPIO pin set to input */
	HW_PIN_ISP_RST_clr();		/* Keep it low */

	/* Rescue clock on EXT8 is handled in platform_avr_ll_init_target() */

	/* ISP SCK Signal */
	HW_PIN_ISP_SCK_pulldown(1);
	HW_PIN_ISP_SCK_pullup(0);
	HW_PIN_ISP_SCK_in();
	HW_PIN_ISP_SCK_clr();

	/* ISP MISO Signal */
	HW_PIN_ISP_MISO_pulldown(1);
	HW_PIN_ISP_MISO_pullup(0);
	HW_PIN_ISP_MISO_in();
	HW_PIN_ISP_MISO_clr();

	/* ISP MOSI Signal */
	HW_PIN_ISP_MOSI_pulldown(1);
	HW_PIN_ISP_MOSI_pullup(0);
	HW_PIN_ISP_MOSI_in();
	HW_PIN_ISP_MOSI_clr();

	platform_hw_init_target();
}

void
avr_ll_isp_start(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	HW_PIN_ISP_SCK_clr();
	HW_PIN_ISP_SCK_out();
	HW_PIN_ISP_SCK_BUFFER_OUT();

	HW_PIN_ISP_MOSI_clr();
	HW_PIN_ISP_MOSI_out();
	HW_PIN_ISP_MOSI_BUFFER_OUT();

	HW_PIN_ISP_MISO_in();
	HW_PIN_ISP_MISO_BUFFER_IN();

	avr_ll_isp_rst_low();

	rtos_ipl_restore(ipl);
}

void
avr_ll_isp_stop(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	HW_PIN_ISP_SCK_BUFFER_IN();
	HW_PIN_ISP_SCK_in();
	HW_PIN_ISP_MOSI_BUFFER_IN();
	HW_PIN_ISP_MOSI_in();
	HW_PIN_ISP_MISO_in();

	avr_ll_isp_rst_high();

	rtos_ipl_restore(ipl);
}

void
avr_ll_isp_clk_start(uint32_t freq)
{

	platform_hw_isp_clk_start(freq);
}

void
avr_ll_isp_clk_stop(void)
{

	platform_hw_isp_clk_stop();
}

void
avr_ll_isp_rst_high(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	/* Basically tri-state the RST output driver */
	HW_PIN_ISP_RST_BUFFER_IN();
	HW_PIN_ISP_RST_in();
	rtos_ipl_restore(ipl);
}

void
avr_ll_isp_rst_low(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HW_PIN_ISP_RST_out();
	HW_PIN_ISP_RST_BUFFER_OUT();
	rtos_ipl_restore(ipl);
}

void
avr_ll_isp_sck_high(void)
{

	HW_PIN_ISP_SCK_set();
}

void
avr_ll_isp_sck_low(void)
{

	HW_PIN_ISP_SCK_clr();
}

void
avr_ll_isp_mosi_high(void)
{

	HW_PIN_ISP_MOSI_set();
}

void
avr_ll_isp_mosi_low(void)
{

	HW_PIN_ISP_MOSI_clr();
}

uint8_t
avr_ll_isp_xfer(uint8_t data, uint32_t clk_delay)
{
	uint32_t i;

	if (clk_delay == 0)
		clk_delay = 1;

	/*
	 * We have to bit-bang SPI because the h/w SCK pin is unavailable
	 *
	 * Note: This loop and 'clk_delay' passed in are closely
	 * coupled. Based on cycle counting, SCK clock phases should
	 * be almost identical even for relatively small values of
	 * 'clk_delay'.
	 *
	 * When compiled with -O2, this loop is roughly 26 cycles in
	 * length, with each 'clk_delay' increment adding another 2 x 3.
	 *
	 * So appropriate 'clk_delay' values can be derived fairly easily.
	 *
	 * After some noodling, the equation to convert clock period,
	 * in micro-seconds, to clk_delay is:
	 *
	 *	((F_CPU / 6000) * us) / 722
	 *
	 * This is a first approximation. I'll need some time with the
	 * scope to verify.
	 */
	for (i = 8; i; i--) {
		if (data & 0x80u)
			HW_PIN_ISP_MOSI_set();
		else
			HW_PIN_ISP_MOSI_clr();

		data <<= 1;

		timer_delay_cycles(clk_delay);

		HW_PIN_ISP_SCK_set();

		if (HW_PIN_ISP_MISO_read())
			data |= 0x1u;

		timer_delay_cycles(clk_delay);

		HW_PIN_ISP_SCK_clr();
	}

	return data;
}

void
avr_ll_pdi_enable(uint32_t pdi_clk)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	/*
	 * Set TxD to output, even though we will communicate over
	 * the Rx buffer output.
	 *
	 * We keep the actual external pin in input mode, which means
	 * there will be contention internally with our TxD pin whenever
	 * it is driven low. However, there's 330R resistor between the
	 * two drivers which will limit contention current to 10mA. I've
	 * verified on the scope that the logic low level is about 680mV
	 * so well within spec to be registered as a zero.
	 *
	 * The point of this is to avoid driving pins we're not actively
	 * using - no knowing what else users may put on that 6-pin
	 * header.
	 */
	HW_PIN_PDI_TXD_set();
	HW_PIN_PDI_TXD_out();

	/*
	 * Ensure RxD is in input mode, and activate the bridge.
	 * Then switch RxD buffer to output. The resister between
	 * the bridge and the buffer will prevent contention damage.
	 */
	HW_PIN_PDI_RXD_in();
	HW_PIN_PDI_BRIDGE_ACTIVATE();
	HW_PIN_PDI_RXD_BUFFER_OUT();

	/* Hold the high level on the PDI pin for 50 uS */
	timer_buzz_delay_us(50);

	platform_hw_pdi_enable(pdi_clk);

	/*
	 * Keep things as they are for a further 20 uS
	 * to allow at least 16 CLK cycles (at 1 MHz).
	 */
	timer_buzz_delay_us(20);

	/* The target should now be in PDI mode */

	/* Ensure Rx is quiescent */
	platform_hw_pdi_rx_purge();

	rtos_ipl_restore(ipl);
}

static uint8_t
avr_ll_updi_enable_impl(void)
{
	u_int i;

	/*
	 * Before doing anything else, ensure the target's UPDI/RESET
	 * pin is high. Note that we assume TVcc is already good and
	 * this the external pull-up is doing something.
	 */
	if (HW_PIN_PDI_RXD_read() == 0)
		return AVR_LL_UPDI_RESET_NOT_HIGH;

	/*
	 * We'll use the RxD pin in output mode to send the UPDI Enable
	 * sequence.
	 */
	HW_PIN_PDI_RXD_set();
	HW_PIN_PDI_RXD_out();
	HW_PIN_PDI_RXD_BUFFER_OUT();
	timer_delay_cycles(40);

	/*
	 * Start the UPDI enable sequence...
	 *
	 * 1. Pull UPDI/RESET low for between 200nS and 1uS.
	 *
	 * With our 120MHz CPU clock we have a cycle time of around
	 * 25nS. So let's buzz for 20 cycles - that should guarantee
	 * we're in the window.
	 */
	HW_PIN_PDI_RXD_clr();
	timer_delay_cycles(20);
	HW_PIN_PDI_RXD_BUFFER_IN();
	HW_PIN_PDI_RXD_in();

	/*
	 * If the target is behaving, it should now be pulling the
	 * UPDI/RESET line low until its programming clock has started.
	 * We wait about 3.5uS before checking because if the target is
	 * not responding then the we're relying on the pullup to take
	 * the line back to TVcc.
	 */
	timer_delay_cycles(150);
	if (HW_PIN_PDI_RXD_read() != 0)
		return AVR_LL_UPDI_RESET_STILL_HIGH;

	/*
	 * 2. The target should now hold the line low for between
	 *    10uS and 200uS. We'll give it 500uS just to be safe.
	 */
	i = 500;
	while (i-- && HW_PIN_PDI_RXD_read() == 0)
		timer_delay_cycles(40);

	/*
	 * Something's wrong if the line is still low after 500uS.
	 */
	if (HW_PIN_PDI_RXD_read() == 0)
		return AVR_LL_UPDI_RESET_LOW_TOO_LONG;

	/*
	 * Target is now in UPDI mode. Configure the hardware ready
	 * for programming.
	 */

	/*
	 * Set TxD to output, even though we will communicate over
	 * the Rx buffer output.
	 *
	 * We keep the actual external pin in input mode, which means
	 * there will be contention internally with our TxD pin whenever
	 * it is driven low. However, there's 330R resistor between the
	 * two drivers which will limit contention current to 10mA. I've
	 * verified on the scope that the logic low level is about 680mV
	 * so well within spec to be registered as a zero.
	 *
	 * The point of this is to avoid driving pins we're not actively
	 * using - no knowing what else users may put on that 6-pin
	 * header.
	 */
	HW_PIN_PDI_TXD_set();
	HW_PIN_PDI_TXD_out();

	return 0;
}

uint8_t
avr_ll_updi_enable(uint32_t updi_baud, int skip_reset_seq)
{
	uint8_t rv;

	/*
	 * Switch the hardware into UPDI mode.
	 */
	rv = platform_hw_updi_enable(updi_baud);
	if (rv != AVR_LL_UPDI_RESET_SUCCESS)
		return rv;

	if (!skip_reset_seq) {
		/*
		 * Send the reset sequence
		 */
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_MAX);
		rv = avr_ll_updi_enable_impl();
		rtos_ipl_restore(ipl);

		/*
		 * If it worked, enable UART functions.
		 */
		if (rv == AVR_LL_UPDI_RESET_SUCCESS)
			platform_hw_updi_function_enable();
		else
			platform_hw_pdi_disable();
	} else {
		/*
		 * No reset sequence needed. Just enable UART functions
		 * and send a break.
		 */
		platform_hw_updi_function_enable();
		platform_hw_pdi_send_break(2, 1);
	}

	return rv;
}

uint8_t
avr_ll_updi_change_baud(uint32_t updi_baud)
{

	return platform_hw_updi_change_baud(updi_baud);
}

void
avr_ll_pdi_disable(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_MAX);

	platform_hw_pdi_disable();

	HW_PIN_PDI_SCK_clr();
	HW_PIN_PDI_SCK_out();
	timer_buzz_delay_us(10);
	HW_PIN_PDI_RXD_BUFFER_IN();
	HW_PIN_PDI_BRIDGE_DEACTIVATE();
	HW_PIN_PDI_RXD_gpio();
	HW_PIN_PDI_TXD_BUFFER_IN();
	HW_PIN_PDI_TXD_gpio();
	timer_buzz_delay_us(90);
	HW_PIN_PDI_SCK_BUFFER_IN();
	HW_PIN_PDI_SCK_gpio();

	/* Give the target time to reset */
	timer_buzz_delay_us(300);

	rtos_ipl_restore(ipl);
}

uint8_t
avr_ll_pdi_transfer(const uint8_t *txp, uint32_t txlen,
    uint8_t *rxp, uint32_t rxlen)
{

	return platform_hw_pdi_transfer(txp, txlen, rxp, rxlen);
}

uint8_t
avr_ll_pdi_rx_status(void)
{
	uint8_t rv;

	rv = platform_hw_pdi_rx_status();

	if (rv == AVR_LL_PDI_RX_DONE) {
		/* Add some guard time */
		timer_delay_cycles(2000);
	}

	return rv;
}

void
avr_ll_pdi_send_break(uint8_t howmany, int change_baud)
{

	platform_hw_pdi_send_break(howmany, change_baud);
}
