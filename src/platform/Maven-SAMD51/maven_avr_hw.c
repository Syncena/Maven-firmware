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
#include <stdint.h>
#include <unistd.h>

#include "rtos.h"
#include "platform.h"
#include "avr_ll.h"
#include "hardware.h"
#include "sam_clocks.h"
#include "sam_dmac.h"
#include "sam_cmcc.h"
#include "sam_sercom_usart.h"
#include "timer.h"

#define	MAVEN_RESCUE_CLOCK_TCC_REGS	TCC0
#define	MAVEN_RESCUE_CLOCK_TCC_ID	ID_TCC0
#define	MAVEN_RESCUE_CLOCK_TCC_GCLK_ID	TCC0_GCLK_ID
#define	MAVEN_RESCUE_CLOCK_TCC_OTMX	2u	/* CC0 out on WO[4] & WO[7] */
#define	MAVEN_RESCUE_CLOCK_TCC_PMUX	'G'

#define	MAVEN_PDI_SERCOM_INSTANCE	0
#define	MAVEN_PDI_SERCOM_REGS		(&SERCOM0->USART)
#define	MAVEN_PDI_SERCOM_ID		ID_SERCOM0
#define	MAVEN_PDI_SERCOM_GCLK_ID	SERCOM0_GCLK_ID_CORE
#define	MAVEN_PDI_SERCOM_TX_PAD		0
#define	MAVEN_PDI_SERCOM_RX_PAD		2
#define	MAVEN_PDI_SERCOM_FUNCTION	'D'

/* Initialised at startup in maven_uart.c:platform_uart_attach() */
unsigned int platform_hw_pdi_clk_gen;
static uint32_t maven_pdi_actual_baud;
static void *maven_pdi_dma_chan;
static sam_dmac_desc_t maven_pdi_dma_desc;
static volatile sam_dmac_status_t maven_pdi_dma_status;
static uint8_t *maven_pdi_rx_ptr;
static uint32_t maven_pdi_rx_len;

void
platform_hw_init_target_avr(void)
{

	/*
	 * Ensure the timer used to generate ISP CLK on EXT8 has a
	 * clock source.
	 */
	sam_periph_clock_enable(MAVEN_RESCUE_CLOCK_TCC_ID);
	sam_gclk_periph_chan_enable(MAVEN_RESCUE_CLOCK_TCC_GCLK_ID, 0);
}

static void
tcc_wait_sync(Tcc *tcc, uint32_t mask)
{

	while ((tcc->SYNCBUSY.reg & mask) != 0)
		;
}

/*
 * Output a clock, of the specified frequency, on the ISP Clock pin
 */
void
platform_hw_isp_clk_start(uint32_t freq)
{
	rtos_saved_ipl_t ipl;
	Tcc *tcc = MAVEN_RESCUE_CLOCK_TCC_REGS;
	uint32_t cc;

	if (freq == 0 || freq >= (sam_gclk_frequencies[0] / 2))
		return;

	cc = (sam_gclk_frequencies[0] / (freq * 2)) - 1;
	while (cc && (sam_gclk_frequencies[0] / (cc + 1)) > (freq * 2))
		cc++;

	if (cc >= (24u * 1024u * 1024u))
		cc = (24u * 1024u * 1024u) - 1u;

	/* If the timer is currently enabled, stop it. */
	if (tcc->CTRLA.bit.ENABLE) {
		tcc->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
		tcc_wait_sync(tcc, TCC_SYNCBUSY_ENABLE);
	}

	/* Reset timer. */
	tcc->CTRLA.reg = TCC_CTRLA_SWRST;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_SWRST);

	/* Counter is clocked directly from the GCLK source. */
	tcc->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_RUNSTDBY;

	/* "Match Frequency Generation" */
	tcc->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_WAVE);

	/* Waveform extension: Set OTMX to route CC0 to appropriate pins. */
	tcc->WEXCTRL.reg = TCC_WEXCTRL_OTMX(MAVEN_RESCUE_CLOCK_TCC_OTMX);

	/* The compare value is written to CC0 */
	tcc->CC[0].reg = cc;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_CC(0));

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HAL_GPIO_EXT8_pmux(MAVEN_RESCUE_CLOCK_TCC_PMUX);
	HAL_GPIO_EXT8_DIR_set();
	rtos_ipl_restore(ipl);

	tcc->CTRLA.reg |= TCC_CTRLA_ENABLE;
	tcc_wait_sync(tcc, TCC_SYNCBUSY_ENABLE);
}

/*
 * Stop the above clock
 */
void
platform_hw_isp_clk_stop(void)
{
	rtos_saved_ipl_t ipl;
	Tcc *tcc = MAVEN_RESCUE_CLOCK_TCC_REGS;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HAL_GPIO_EXT8_DIR_clr();
	HAL_GPIO_EXT8_pmux(HAL_GPIO_PIN_DISABLE_FUNCTION);
	rtos_ipl_restore(ipl);

	if (tcc->CTRLA.bit.ENABLE) {
		tcc->CTRLA.reg = 0;
		tcc_wait_sync(tcc, TCC_SYNCBUSY_ENABLE);
	}
}

/*
 * Configure a SERCOM for synchronous serial mode suitable for PDI.
 * Serial clock given by pdi_clk.
 */
static void
sam_usart_syncbusy(SercomUsart *u, uint32_t mask)
{

	while ((u->SYNCBUSY.reg & mask) != 0)
		;
}

static void
maven_pdi_init_dma(void)
{
	sam_dmac_config_t cfg;

	/*
	 * First time through, allocate a DMA channel for PDI Rx.
	 */
	if (maven_pdi_dma_chan != NULL)
		return;

	/* DMA channel for reads *from* the SPI device. */
	cfg.cfg_threshold = SAM_DMAC_THRESHOLD_1BEAT;
	cfg.cfg_burstlen = SAM_DMAC_BURSTLEN(1);
	cfg.cfg_trigact = SAM_DMAC_TRIGACT_BURST;
	cfg.cfg_trigsrc = SAM_DMAC_TRIGSRC_SERCOM_RX(MAVEN_PDI_SERCOM_INSTANCE);
	cfg.cfg_priority = SAM_DMAC_PRIORITY_HIGHEST;

	maven_pdi_dma_desc = NULL;
	maven_pdi_dma_chan = sam_dmac_channel_alloc(&cfg, &maven_pdi_dma_desc);
	assert(maven_pdi_dma_chan != NULL);
	assert(maven_pdi_dma_desc != NULL);

	maven_pdi_dma_desc->dd_src = &MAVEN_PDI_SERCOM_REGS->DATA.reg;
	maven_pdi_dma_desc->dd_next = NULL;
}

void
platform_hw_pdi_enable(uint32_t pdi_xck)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	unsigned int clock;
	uint32_t baud;

	/* Validate the range of supported clock rates. */
	assert(pdi_xck >= 100000u && pdi_xck <= 10000000u);

	/* Ensure our clock source is configured. */
	assert(platform_hw_pdi_clk_gen > 0);

	/* Enable clocks to the device. */
	sam_periph_clock_enable(MAVEN_PDI_SERCOM_ID);
	clock = sam_gclk_periph_chan_enable(MAVEN_PDI_SERCOM_GCLK_ID,
	    platform_hw_pdi_clk_gen);
	assert(clock > 0);

	/* Compute value for u->BAUD. */
	baud = ((clock / 2u) / pdi_xck) - 1u;

	/* u->BAUD is 8-bits wide in synchronous mode. */
	assert(baud <= 255);

	if (u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		u->CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	u->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_SWRST);

	/* Now initialise the device in sync mode */
	u->CTRLA.reg =
	    SERCOM_USART_CTRLA_RUNSTDBY |
	    SERCOM_USART_CTRLA_DORD |		   /* Bit order: LSB first */
	    SERCOM_USART_CTRLA_CMODE |		   /* Sync mode */
	    SERCOM_USART_CTRLA_CPOL |		   /* Sample on rising XCK */
	    SERCOM_USART_CTRLA_FORM(1) |	   /* USART frame with parity */
	    SERCOM_USART_CTRLA_SAMPA(0) |	   /* Sample adjust 7-8-9 */
	    SERCOM_USART_CTRLA_SAMPR(0) |
	    SERCOM_USART_CTRLA_MODE(1) |	   /* USART mode, int clk */
	    SERCOM_USART_CTRLA_TXPO(MAVEN_PDI_SERCOM_TX_PAD) |/* TXD pinout */
	    SERCOM_USART_CTRLA_RXPO(MAVEN_PDI_SERCOM_RX_PAD); /* RXD pinout */
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

	/* 8 bits, even parity, 2 stop bits. */
	u->CTRLB.reg = SERCOM_USART_CTRLB_SBMODE | SERCOM_USART_CTRLB_TXEN |
	    SERCOM_USART_CTRLB_RXEN;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

#ifdef SERCOM_USART_CTRLC_OFFSET
	u->CTRLC.reg = 0;
#endif

	u->BAUD.reg = (uint16_t)baud;

	HW_PIN_PDI_RXD_function(MAVEN_PDI_SERCOM_FUNCTION);
	HW_PIN_PDI_TXD_function(MAVEN_PDI_SERCOM_FUNCTION);
	HW_PIN_PDI_SCK_function(MAVEN_PDI_SERCOM_FUNCTION);
	HW_PIN_PDI_SCK_BUFFER_OUT();

	u->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);

	maven_pdi_actual_baud = clock / ((baud + 1) * 2u);

	maven_pdi_init_dma();
}

/*
 * Disable PDI/UPDI.
 */
void
platform_hw_pdi_disable(void)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;

	if (maven_pdi_dma_chan != NULL)
		sam_dmac_channel_stop(maven_pdi_dma_chan);

	if (u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		u->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	u->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_SWRST);

	sam_gclk_periph_chan_disable(MAVEN_PDI_SERCOM_GCLK_ID);
	sam_periph_clock_disable(MAVEN_PDI_SERCOM_ID);
}

/*
 * Ensure the SERCOM Rx holding register is empty.
 */
void
platform_hw_pdi_rx_purge(void)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	uint8_t sr;

	if ((u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) == 0)
		return;

	if (maven_pdi_dma_chan != NULL)
		sam_dmac_channel_stop(maven_pdi_dma_chan);

	/* Clear the FIFO */
	sr = u->INTFLAG.reg;

	while (sr & (SERCOM_USART_INTFLAG_RXC | SERCOM_USART_INTFLAG_RXBRK |
	    SERCOM_USART_INTFLAG_ERROR)) {
		uint16_t st;

		st = u->STATUS.reg;
		(void) u->DATA.reg;
		u->STATUS.reg = st;
		u->INTFLAG.reg = sr;
		sr = u->INTFLAG.reg;
	}
}

/*
 * Send a PDI/UPDI break sequence.
 *
 * SERCOM cannot generate a break sequence in hardware, at least not in
 * async/sync modes. We have to do things manually.
 */
void
platform_hw_pdi_send_break(int howmany, int change_baud)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	uint32_t baud, cycles;
	rtos_saved_ipl_t ipl;

	if ((u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) == 0)
		return;

	baud = (change_baud == 0) ? maven_pdi_actual_baud : 400u;
	cycles = (2u * (F_CPU / (baud / 16u))) / 3u;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	HW_PIN_PDI_TXD_gpio();
	HW_PIN_PDI_TXD_set();
	HW_PIN_PDI_TXD_out();

	/* Enable Tx */
	avr_ll_pdi_tx_enable();

	while (howmany--) {
		HW_PIN_PDI_TXD_clr();

		timer_delay_cycles(cycles);

		HW_PIN_PDI_TXD_set();

		timer_delay_cycles(cycles / 8u);
	}

	/* Disable Tx */
	avr_ll_pdi_tx_disable();

	HW_PIN_PDI_TXD_in();
	HW_PIN_PDI_TXD_function(MAVEN_PDI_SERCOM_FUNCTION);
	rtos_ipl_restore(ipl);

	platform_hw_pdi_rx_purge();
}

/*
 * Perform PDI/UPDI data transfer.
 */
uint8_t
platform_hw_pdi_transfer(const uint8_t *txp, uint32_t txlen,
    uint8_t *rxp, uint32_t rxlen)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	sam_dmac_desc_t dd = maven_pdi_dma_desc;
	rtos_saved_ipl_t ipl;

	assert(dd != NULL);

	if ((u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) == 0)
		return 1;

	/* Purge the Rx FIFO */
	platform_hw_pdi_rx_purge();
	u->INTFLAG.reg = SERCOM_USART_INTFLAG_MASK;

	if (rxlen) {
		dd->dd_cnt = maven_pdi_rx_len = rxlen;
		maven_pdi_rx_ptr = (void *)rxp;
		dd->dd_dst = (void *)(rxp + rxlen);
	}

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	/* Enable the Tx buffer */
	avr_ll_pdi_tx_enable();

	/*
	 * Use polled Tx, for the sake of simplicity.
	 */
	while (txlen--) {
		while ((u->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == 0)
				;
		u->DATA.reg = (uint32_t) *txp++;
	}

	/* Wait for Tx to complete. */
	while ((u->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == 0)
		;
	while ((u->INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0)
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

	/* Purge the receiver. */
	while (u->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)
		(void) u->DATA.reg;

	/* Clear any latched status */
	u->STATUS.reg = SERCOM_USART_STATUS_MASK;
	u->INTFLAG.reg = SERCOM_USART_INTFLAG_MASK;

	if (rxlen) {
		dd->dd_ctrl =
		    SAM_DMAC_CTRL_VALID |
		    /* 1 byte steps */
		    SAM_DMAC_CTRL_STEPSIZE(0) |
		    SAM_DMAC_CTRL_STEPSEL_DST |
		    /* 1 byte beats */
		    SAM_DMAC_CTRL_BEATSIZE(SAM_DMAC_BEAT_8BITS) |
		    /* Increment destination address. */
		    SAM_DMAC_CTRL_DSTINC |
		    /* Configure completion action. */
		    SAM_DMAC_CTRL_BLOCKACT(SAM_DMAC_BLOCKACT_NOACT);

		sam_dmac_channel_start(maven_pdi_dma_chan, false);
	}

	/* Switch to receive mode */
	avr_ll_pdi_tx_disable();

	/* The receive, if any, will continue in the background */
	rtos_ipl_restore(ipl);

	return 0;
}

/*
 * Return current PDI status
 */
uint8_t
platform_hw_pdi_rx_status(void)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	uint8_t rv;

	assert(maven_pdi_dma_desc != NULL);

	if (maven_pdi_rx_len != 0) {
		switch (sam_dmac_channel_status(maven_pdi_dma_chan)) {
		case SAM_DMAC_STATUS_BUSY:
			rv = AVR_LL_PDI_RX_BUSY;
			break;

		case SAM_DMAC_STATUS_COMPLETE:
			if (u->INTFLAG.reg & SERCOM_USART_INTFLAG_ERROR)
				rv = AVR_LL_PDI_RX_CORRUPT;
			else
				rv = AVR_LL_PDI_RX_DONE;

			CMCC_InvalidateDCache_by_Addr(
			    (const void *)maven_pdi_rx_ptr, maven_pdi_rx_len);

			u->STATUS.reg = SERCOM_USART_STATUS_MASK;
			u->INTFLAG.reg = SERCOM_USART_INTFLAG_MASK;
			maven_pdi_rx_len = 0;
			break;

		default:
			u->STATUS.reg = SERCOM_USART_STATUS_MASK;
			u->INTFLAG.reg = SERCOM_USART_INTFLAG_MASK;
			rv = AVR_LL_PDI_RX_CORRUPT;
			maven_pdi_rx_len = 0;
			break;
		}
	} else {
		rv = AVR_LL_PDI_RX_IDLE;
	}

	return rv;
}

void
platform_hw_updi_function_enable(void)
{

	HW_PIN_PDI_RXD_function(MAVEN_PDI_SERCOM_FUNCTION);
	HW_PIN_PDI_TXD_function(MAVEN_PDI_SERCOM_FUNCTION);
}

/*
 * Enable UPDI
 */
uint8_t
platform_hw_updi_enable(uint32_t baud)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	unsigned int clock;
	uint32_t sampr;
	uint16_t baudreg;

	/* Ensure our clock source is configured. */
	assert(platform_hw_pdi_clk_gen > 0);

	/* Enable clocks to the device. */
	sam_periph_clock_enable(MAVEN_PDI_SERCOM_ID);
	clock = sam_gclk_periph_chan_enable(MAVEN_PDI_SERCOM_GCLK_ID,
	    platform_hw_pdi_clk_gen);
	assert(clock > 0);

	if (u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		u->CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	u->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_SWRST);

	/* Now initialise the device in sync mode */
	u->CTRLA.reg =
	    SERCOM_USART_CTRLA_RUNSTDBY |
	    SERCOM_USART_CTRLA_DORD |		   /* Bit order: LSB first */
	    SERCOM_USART_CTRLA_FORM(1) |	   /* USART frame with parity */
	    SERCOM_USART_CTRLA_SAMPA(0) |	   /* Sample adjust 7-8-9 */
	    SERCOM_USART_CTRLA_SAMPR(sampr) |
	    SERCOM_USART_CTRLA_MODE(1) |	   /* USART mode, int clk */
	    SERCOM_USART_CTRLA_TXPO(MAVEN_PDI_SERCOM_TX_PAD) |/* TXD pinout */
	    SERCOM_USART_CTRLA_RXPO(MAVEN_PDI_SERCOM_RX_PAD); /* RXD pinout */
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

	/* 8 bits, even parity, 2 stop bits. */
	u->CTRLB.reg = SERCOM_USART_CTRLB_SBMODE | SERCOM_USART_CTRLB_TXEN |
	    SERCOM_USART_CTRLB_RXEN;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

#ifdef SERCOM_USART_CTRLC_OFFSET
	u->CTRLC.reg = 0;
#endif

	/* Compute value for u->BAUD. */
	baudreg = sam_sercom_usart_calc_baudreg(clock, baud, &sampr,
	    &maven_pdi_actual_baud);
	assert(baudreg != 0);

	u->BAUD.reg = baudreg;

	u->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

	maven_pdi_init_dma();

	return AVR_LL_UPDI_RESET_SUCCESS;
}

/*
 * Change UPDI baudrate
 */
uint8_t
platform_hw_updi_change_baud(uint32_t baud)
{
	SercomUsart *u = MAVEN_PDI_SERCOM_REGS;
	uint32_t sampr, actual_baud;
	unsigned int clock;
	uint16_t baudreg;

	if ((u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) == 0)
		return AVR_LL_UPDI_RESET_NOT_HIGH;

	assert(platform_hw_pdi_clk_gen > 0);
	clock = sam_gclk_frequencies[platform_hw_pdi_clk_gen];

	baudreg = sam_sercom_usart_calc_baudreg(clock, baud, &sampr,
	    &actual_baud);

	if (actual_baud != maven_pdi_actual_baud) {
		uint32_t ctrla;

		ctrla = u->CTRLA.reg;
		u->CTRLA.reg = ctrla & ~SERCOM_USART_CTRLA_ENABLE;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);

		u->BAUD.reg = baudreg;

		ctrla &= ~SERCOM_USART_CTRLA_SAMPR_Msk;
		ctrla |= SERCOM_USART_CTRLA_SAMPR(sampr);

		u->CTRLA.reg = ctrla;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	return AVR_LL_UPDI_RESET_SUCCESS;
}
