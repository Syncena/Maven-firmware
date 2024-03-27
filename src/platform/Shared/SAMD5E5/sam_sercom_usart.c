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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "ringbuff.h"
#include "platform.h"
#include "sam_sercom_usart.h"
#include "sam_clocks.h"
#include "fix16.h"
#include "zone_alloc.h"

#define	SAM_USART_SAMPR_ARITH	0
#define	SAM_USART_SAMPR_FB16	1
#define	SAM_USART_SAMPR_FB8	3

struct sam_usart_state {
	SercomUsart *us_regs;
	unsigned int us_gclk_id;
	unsigned int us_slow_gen;
	unsigned int us_fast_gen;
	unsigned int us_curr_gen;
	unsigned int us_clock;
	uint32_t us_ipl;
	uint16_t us_num;
#ifdef RTOS_OPTION_SOFT_IRQ
	void *us_si_handle;
	uint32_t us_si_flag;
#define	SAM_USART_SI_FLAG_RX	(1u << 0)
#define	SAM_USART_SI_FLAG_TX	(1u << 1)
#endif
	ringbuff_t us_uart2app;
	ringbuff_t us_app2uart;
	struct uart_stats us_stats;
};

static struct sam_usart_state *
sam_usart_alloc(void)
{
	struct sam_usart_state *us;
	static uint16_t unum;

	us = zone_calloc(1, sizeof(struct sam_usart_state));
	if (us != NULL) {
		us->us_num = unum++;
	}

	return us;
}

static __always_inline void
sam_usart_tx_irq_disable(SercomUsart *u)
{

	u->INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

static __always_inline void
sam_usart_tx_irq_enable(SercomUsart *u)
{

	u->INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

static __always_inline void
sam_usart_rx_irq_disable(SercomUsart *u)
{

	u->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC|SERCOM_USART_INTENCLR_ERROR;
}

static __always_inline void
sam_usart_rx_irq_enable(struct sam_usart_state *us)
{
	SercomUsart *u = us->us_regs;

	u->INTENSET.reg = SERCOM_USART_INTENSET_RXC|SERCOM_USART_INTENSET_ERROR;
}

static __always_inline uint8_t
sam_usart_tx_busy(SercomUsart *u)
{

	return (u->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) == 0;
}

static int
sam_usart_irq_rx_ready(struct sam_usart_state *us)
{
	uint32_t data;

	data =  us->us_regs->DATA.reg;

	us->us_stats.us_rx_bytes++;

	if (us->us_uart2app == NULL)
		return 0;

	if (!ringbuff_is_full(us->us_uart2app)) {
		ringbuff_produce(us->us_uart2app, (uint8_t)(data & 0xffu));
	} else {
		us->us_stats.us_rx_ring_overruns++;
	}

	return 1;
}

static void
sam_usart_irq_tx_done(struct sam_usart_state *us)
{
	uint8_t ch;

	if (us->us_app2uart == NULL || ringbuff_is_empty(us->us_app2uart)) {
		/* No peer, or FIFO empty. Disable Tx Empty irq */
		sam_usart_tx_irq_disable(us->us_regs);
		return;
	}

	ch = ringbuff_consume(us->us_app2uart);
	us->us_regs->DATA.reg = (uint32_t) ch;
	us->us_stats.us_tx_bytes++;
}

static void
sam_usart_interrupt(void *arg)
{
	struct sam_usart_state *us = arg;
	SercomUsart *u = us->us_regs;
	int rx_work, tx_work;
	uint8_t st;

	rx_work = tx_work = 0;

	st = u->INTFLAG.reg;
	u->INTFLAG.reg = st;
	st &= u->INTENSET.reg;

	do {
		if ((st & SERCOM_USART_INTFLAG_ERROR) != 0) {
			uint8_t err = u->STATUS.reg;
			u->STATUS.reg = err;
			if (err & SERCOM_USART_STATUS_FERR)
				us->us_stats.us_rx_fe++;
			if (err & SERCOM_USART_STATUS_PERR)
				us->us_stats.us_rx_pe++;
			if (err & SERCOM_USART_STATUS_BUFOVF)
				us->us_stats.us_rx_fifo_overruns++;
		}

		if ((st & SERCOM_USART_INTFLAG_RXC) != 0)
			rx_work += sam_usart_irq_rx_ready(us);

		if ((st & SERCOM_USART_INTFLAG_DRE) != 0) {
			sam_usart_irq_tx_done(us);
			tx_work = 1;
		}

		st = u->INTFLAG.reg;
		u->INTFLAG.reg = st;
		st &= u->INTENSET.reg;
	} while (st != 0);

#ifdef RTOS_OPTION_SOFT_IRQ
	uint32_t si_flag = us->us_si_flag;
#endif

	if (rx_work && us->us_uart2app) {
#ifdef RTOS_OPTION_SOFT_IRQ
		if (us->us_si_handle != NULL)
			si_flag |= SAM_USART_SI_FLAG_RX;
		else
#endif
		ringbuff_produce_done(us->us_uart2app);
	}

	if (tx_work && us->us_app2uart) {
#ifdef RTOS_OPTION_SOFT_IRQ
		if (us->us_si_handle != NULL)
			si_flag |= SAM_USART_SI_FLAG_TX;
		else
#endif
		ringbuff_consume_done(us->us_app2uart);
	}

#ifdef RTOS_OPTION_SOFT_IRQ
	if (si_flag != us->us_si_flag) {
		us->us_si_flag = si_flag;
		rtos_soft_irq_schedule(us->us_si_handle);
	}
#endif
}

#ifdef RTOS_OPTION_SOFT_IRQ
static void
sam_usart_soft_interrupt(void *arg)
{
	struct sam_usart_state *us = arg;
	rtos_saved_ipl_t ipl;
	uint32_t si_flag;

	ipl = rtos_ipl_raise(us->us_ipl);
	si_flag = us->us_si_flag;
	us->us_si_flag = 0;
	rtos_ipl_restore(ipl);

	if (si_flag & SAM_USART_SI_FLAG_RX)
		ringbuff_produce_done(us->us_uart2app);

	if (si_flag & SAM_USART_SI_FLAG_TX)
		ringbuff_consume_done(us->us_app2uart);
}
#endif /* RTOS_OPTION_SOFT_IRQ */

/*
 * Invoked whenever data is received from the application and has
 * been loaded into the ring buffer.
 */
static void
sam_usart_tx_avail_cb(ringbuff_t rb, void *arg)
{
	struct sam_usart_state *us = arg;

	(void) rb;
	(void) arg;

	sam_usart_tx_irq_enable(us->us_regs);
}

static void
sam_usart_polled_tx(void *arg, uint8_t ch)
{
	struct sam_usart_state *us = arg;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(us->us_ipl);

	while (sam_usart_tx_busy(us->us_regs))
		;

	us->us_regs->DATA.reg = (uint32_t) ch;
	us->us_stats.us_tx_bytes++;
	us->us_stats.us_tx_bytes_polled++;

	rtos_ipl_restore(ipl);
}

static void
sam_usart_syncbusy(SercomUsart *u, uint32_t mask)
{

	while ((u->SYNCBUSY.reg & mask) != 0)
		;
}

static uint8_t
sam_usart_open(void *arg, ringbuff_t uart2app, ringbuff_t app2uart)
{
	struct sam_usart_state *us = arg;
	SercomUsart *u = us->us_regs;
	uint32_t ctrlb = 0;

	if ((us->us_uart2app = uart2app) != NULL) {
		ctrlb |= SERCOM_USART_CTRLB_RXEN;
	}

	if ((us->us_app2uart = app2uart) != NULL) {
		ctrlb |= SERCOM_USART_CTRLB_TXEN;
		ringbuff_consumer_init(app2uart, sam_usart_tx_avail_cb, us);
	}

	u->CTRLB.reg |= ctrlb;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_CTRLB);

	if (ctrlb & SERCOM_USART_CTRLB_RXEN)
		sam_usart_rx_irq_enable(us);

	return 1;
}

static void
sam_usart_close(void *arg)
{
	struct sam_usart_state *us = arg;
	SercomUsart *u = us->us_regs;

	sam_usart_tx_irq_disable(us->us_regs);
	sam_usart_rx_irq_disable(us->us_regs);

	u->CTRLB.reg &= ~(SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN);
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_CTRLB);

	if (us->us_uart2app != NULL) {
		ringbuff_producer_init(us->us_uart2app, NULL, NULL);
		us->us_uart2app = NULL;
	}
	if (us->us_app2uart != NULL) {
		ringbuff_consumer_init(us->us_app2uart, NULL, NULL);
		us->us_app2uart = NULL;
	}
}

static uint32_t
sam_usart_actual_speed(uint32_t clock, uint16_t baudreg, uint32_t sampr)
{
	uint32_t baud;

	if (baudreg == 0)
		return 0;

	if (sampr == SAM_USART_SAMPR_ARITH) {
		float x;

		/*
		 * This provides much better accuracy compared to the
		 * fixed-point version.
		 */
		x = 1.0F - ((float)baudreg / 65536.0F);
		x *= (float)clock / 16.0F;

		baud = (uint32_t)x;
	} else {
		uint32_t cd, fp;

		/* Extract the two fields from baudreg. */
		cd = (uint32_t)(baudreg & SERCOM_USART_BAUD_FRACFP_BAUD_Msk);
		cd >>= SERCOM_USART_BAUD_FRACFP_BAUD_Pos;
		fp = (uint32_t)(baudreg & SERCOM_USART_BAUD_FRACFP_FP_Msk);
		fp >>= SERCOM_USART_BAUD_FRACFP_FP_Pos;

		/* Scale and combine the two. */
		cd = (cd * 8u) + fp;

		/* Additional scaling for x16 mode. */
		if (sampr == SAM_USART_SAMPR_FB16)
			cd *= 2u;

		/* Divide clock, with rounding. */
		baud = (clock + (cd / 2u)) / cd;
	}

	return baud;
}

static uint16_t
sam_usart_calc_baudreg_arith(uint32_t clock, uint32_t baud)
{
	uint16_t baudreg;

	/*
	 * Formula is:
	 *
	 *  65536.0 * (1.0 - (16 * (baud / clock)))
	 */
	float x;

	x = 1.0F - (16.0F * ((float)baud / (float)clock));
	x *= 65536.0F;
	baudreg = (uint32_t)(x + 0.5F);

	return baudreg;
}

#define	SAM_USART_BAUD_CD_MASK	(SERCOM_USART_BAUD_FRACFP_BAUD_Msk << \
				 SERCOM_USART_BAUD_FRACFP_BAUD_Pos)

static uint16_t
sam_usart_calc_baudreg_frac(uint32_t clock, uint32_t baud, uint32_t *sampr)
{
	uint32_t fp16, cd16;
	uint32_t baud16;
	uint16_t x16;

	baud16 = baud * 16u;
	fp16 = ((8 * clock) + (baud16 / 2)) / baud16;

	/* Extract the fields. */
	cd16 = fp16 >> 3;
	fp16 &= 0x07;
	if ((cd16 & ~SAM_USART_BAUD_CD_MASK) != 0)
		cd16 = 0;

	/* Generate the BAUD register. */
	x16 = (uint16_t)SERCOM_USART_BAUD_FRACFP_BAUD(cd16) |
	    (uint16_t)SERCOM_USART_BAUD_FRACFP_FP(fp16);

	uint32_t fp8, cd8;
	uint32_t baud8;
	uint16_t x8;

	baud8 = baud * 8u;
	fp8 = ((8 * clock) + (baud8 / 2)) / baud8;

	/* Extract the fields. */
	cd8 = fp8 >> 3;
	fp8 &= 0x07;
	if ((cd8 & ~SAM_USART_BAUD_CD_MASK) != 0)
		cd8 = 0;

	/* Bail now if the baudrate is unachievable. */
	if (cd16 == 0 && cd8 == 0)
		return 0;

	/* At least x8 must be valid. */
	assert(cd8 != 0);

	/* Generate the two contenders for the BAUD register. */
	x8 = (uint16_t)SERCOM_USART_BAUD_FRACFP_BAUD(cd8) |
	    (uint16_t)SERCOM_USART_BAUD_FRACFP_FP(fp8);

	/* Compute actual x8 rate. */
	baud8 = sam_usart_actual_speed(clock, x8, SAM_USART_SAMPR_FB8);

	/* If x16 is zero then we have to go with x8 */
	if (x16 == 0) {
		*sampr = SAM_USART_SAMPR_FB8;
		return x8;
	}

	/* Compute actual x16 rate. */
	baud16 = sam_usart_actual_speed(clock, x16, SAM_USART_SAMPR_FB16);

	/*
	 * Go with whichever configuration has the least error, but
	 * prefer x16 if both are the same.
	 */
	if (abs((int)(baud - baud16)) <= abs((int)(baud - baud8))) {
		*sampr = SAM_USART_SAMPR_FB16;
		return x16;
	}

	*sampr = SAM_USART_SAMPR_FB8;
	return x8;
}

uint16_t
sam_sercom_usart_calc_baudreg(uint32_t clock, uint32_t baud, uint32_t *sampr,
    uint32_t *actual_baud)
{
	uint16_t baudreg_arith, baudreg_frac, rv;
	uint32_t actual_arith, actual_frac;
	unsigned int err_arith, err_frac, err_limit;

	err_limit = baud / 50u;

	baudreg_arith = sam_usart_calc_baudreg_arith(clock, baud);
	actual_arith = sam_usart_actual_speed(clock, baudreg_arith,
	    SAM_USART_SAMPR_ARITH);
	err_arith = (unsigned int)abs((int)actual_arith - (int)baud);

	baudreg_frac = sam_usart_calc_baudreg_frac(clock, baud, sampr);
	actual_frac = sam_usart_actual_speed(clock, baudreg_frac, *sampr);
	err_frac = (unsigned int)abs((int)actual_frac - (int)baud);

	rv = 0;

	if (err_arith <= err_frac) {
		if (err_arith <= err_limit) {
			*sampr = SAM_USART_SAMPR_ARITH;
			if (actual_baud != NULL)
				*actual_baud = actual_arith;
			rv = baudreg_arith;
		}
	} else {
		if (err_frac <= err_limit) {
			rv = baudreg_frac;
			if (actual_baud != NULL)
				*actual_baud = actual_frac;
		}
	}

	return rv;
}

static void
sam_usart_write_config(SercomUsart *u, uint32_t ctrla, uint32_t ctrlb,
    uint32_t baudreg)
{

	if ((u->CTRLB.reg & (SERCOM_USART_CTRLB_TXEN |
	    SERCOM_USART_CTRLB_RXEN)) != 0) {
		/* Disable Rx/Tx. */
		u->CTRLB.reg &= ~(SERCOM_USART_CTRLB_TXEN |
		    SERCOM_USART_CTRLB_RXEN);
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_CTRLB);
	}

	if ((u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) != 0) {
		/* Disable the controller. */
		u->CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	/* Update baudrate. */
	u->BAUD.reg = baudreg;

	/* Enable Tx/Rx. */
	u->CTRLB.reg = ctrlb;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_CTRLB);

	/* Write the new config and enable. */
	u->CTRLA.reg = ctrla | SERCOM_USART_CTRLA_ENABLE;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
}

/*
 * Sometimes, particularly when using high speed on the SWO port, we can't
 * achieve the desired baudrate with our regular clock generators.
 * In this situation, we retry the baud rate calculations using all
 * configured generators in an effort to find one which brings us closest
 * to the target baud rate.
 *
 * platform_uart_attach() configured some additional clocks for just this
 * purpose, so we should be pretty good at achieving most rates the user
 * throws at us.
 */
static uint32_t
sam_usart_baud_me_harder(struct sam_usart_state *us, uint32_t want_baud,
    uint16_t *pbaudreg, uint32_t *psampr)
{
	unsigned int gen, clock, minclock, best_gen;
	uint32_t sampr, actual_baud, err, best_err, best_sampr;
	uint16_t baudreg, best_baudreg;

	best_err = want_baud;
	best_gen = GCLK_GEN_NUM;
	best_sampr = 0;
	best_baudreg = 0;
	minclock = want_baud * 16;

	for (gen = 0; gen < GCLK_GEN_NUM; gen++) {
		clock = sam_gclk_frequencies[gen];
		if (clock == 0 || clock > 100000000u || clock < minclock)
			continue;

		sampr = 0;
		actual_baud = 0;
		baudreg = sam_sercom_usart_calc_baudreg(clock, want_baud,
		    &sampr, &actual_baud);
		if (actual_baud == 0)
			continue;

		if (actual_baud > want_baud)
			err = actual_baud - want_baud;
		else
			err = want_baud - actual_baud;

		if (err < (want_baud / 75u) && err < best_err) {
			best_err = err;
			best_gen = gen;
			best_sampr = sampr;
			best_baudreg = baudreg;
			if (err == 0)
				break;
		}
	}

	if (best_gen == GCLK_GEN_NUM)
		return 0;

	*pbaudreg = best_baudreg;
	*psampr = best_sampr;
	us->us_curr_gen = best_gen;
	us->us_clock = sam_gclk_periph_chan_enable(us->us_gclk_id, best_gen);
#if (RELEASE_BUILD == 0)
	if (us->us_regs == &SERCOM0->USART) {
		printf("SWO harder baudreg %04" PRIx16 ", sampr %" PRIu32
		    ", clock %u (%u), baud %" PRIu32 "\n",
		    *pbaudreg, *psampr, us->us_clock, us->us_curr_gen,
		    actual_baud);
	}
#endif

	return actual_baud;
}

static uint32_t
sam_usart_configure(void *arg, uint16_t cfg, uint32_t custom_baud)
{
	struct sam_usart_state *us = arg;
	SercomUsart *u = us->us_regs;
	uint32_t ctrla, ctrlb, sampr, baud, actual_baud;
	rtos_saved_ipl_t ipl;
	uint16_t baudreg;

	if (custom_baud && us->us_clock == 0)
		return 0;

	baud = uart_cfg_to_baud(cfg, custom_baud);

	unsigned int old_gen = us->us_curr_gen;

	us->us_curr_gen = (baud >= 57600u) ? us->us_fast_gen : us->us_slow_gen;
	if (us->us_curr_gen != old_gen) {
		us->us_clock = sam_gclk_periph_chan_enable(us->us_gclk_id,
		    us->us_curr_gen);
	}

	switch (cfg & UART_PARITY_MASK) {
	default:
		ctrla = SERCOM_USART_CTRLA_FORM(0);
		ctrlb = 0;
		break;
	case UART_PARITY_ODD:
		ctrla = SERCOM_USART_CTRLA_FORM(1);
		ctrlb = SERCOM_USART_CTRLB_PMODE;
		break;
	case UART_PARITY_EVEN:
		ctrla = SERCOM_USART_CTRLA_FORM(1);
		ctrlb = 0;
		break;
	}

	if ((cfg & UART_STOPBITS_MASK) == UART_STOPBITS_2)
		ctrlb |= SERCOM_USART_CTRLB_SBMODE;

	switch (cfg & UART_CHARSIZE_MASK) {
	case UART_CHARSIZE_5:
		ctrlb |= SERCOM_USART_CTRLB_CHSIZE(5);
		break;
	case UART_CHARSIZE_6:
		ctrlb |= SERCOM_USART_CTRLB_CHSIZE(6);
		break;
	case UART_CHARSIZE_7:
		ctrlb |= SERCOM_USART_CTRLB_CHSIZE(7);
		break;
	case UART_CHARSIZE_8:
		ctrlb |= SERCOM_USART_CTRLB_CHSIZE(0);
		break;
	}

	baudreg = 0;
	sampr = 0;
	actual_baud = 0;
	if (actual_baud == 0 &&
	    (actual_baud = sam_usart_baud_me_harder(us, baud, &baudreg,
	        &sampr)) == 0) {
		if (us->us_curr_gen != old_gen) {
			us->us_clock = sam_gclk_periph_chan_enable(
			    us->us_gclk_id, old_gen);
			us->us_curr_gen = old_gen;
		}
#if (RELEASE_BUILD == 0)
	if (us->us_regs == &SERCOM0->USART) {
		printf("SWO failed baudreg %04" PRIx16 ", sampr %" PRIu32
		    ", clock %u (%u), baud %" PRIu32 "\n",
		    baudreg, sampr, us->us_clock, us->us_curr_gen, actual_baud);
	}
#endif
		return 0;
	}

	ctrla |= SERCOM_USART_CTRLA_SAMPR(sampr);
	ctrla |= u->CTRLA.reg & ~(SERCOM_USART_CTRLA_FORM_Msk |
	    SERCOM_USART_CTRLA_ENABLE | SERCOM_USART_CTRLA_SAMPR_Msk);
	ctrlb |= u->CTRLB.reg & ~(SERCOM_USART_CTRLB_PMODE |
	    SERCOM_USART_CTRLB_SBMODE | SERCOM_USART_CTRLB_CHSIZE_Msk);

	ipl = rtos_ipl_raise(us->us_ipl);
	sam_usart_write_config(u, ctrla, ctrlb, baudreg);
	rtos_ipl_restore(ipl);

	return baud;
}

static void
sam_usart_get_stats(void *arg, struct uart_stats *s)
{
	struct sam_usart_state *us = arg;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(us->us_ipl);
	memcpy(s, &us->us_stats, sizeof(*s));
	rtos_ipl_restore(ipl);
}

int
sam_sercom_usart_attach(const sam_sercom_usart_attach_args_t *aa,
    unsigned int slow_gen, unsigned int fast_gen)
{
	static struct uart_funcs uf;
	struct sam_usart_state *us;
	SercomUsart *u;
	const char *name;
	char irq_name[20], *p;

	if ((us = sam_usart_alloc()) == NULL)
		return -1;

	u = us->us_regs = aa->aa_regs;
	us->us_slow_gen = slow_gen;
	us->us_fast_gen = fast_gen;
	us->us_gclk_id = aa->aa_gclk_id;
	us->us_ipl = aa->aa_ipl;

	/* Default to the fast clock on startup. */
	us->us_clock = sam_gclk_periph_chan_enable(us->us_gclk_id,
	    us->us_fast_gen);
	us->us_curr_gen = us->us_fast_gen;

	if (u->CTRLA.reg & SERCOM_USART_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		u->CTRLA.reg = 0;
		sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	u->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_SWRST);

	/* Now initialise the device in async UART mode */
	u->CTRLA.reg =
	    SERCOM_USART_CTRLA_RUNSTDBY |
	    SERCOM_USART_CTRLA_DORD |		   /* Bit order: LSB first */
	    (0u << SERCOM_USART_CTRLA_CMODE_Pos) | /* Async mode */
	    SERCOM_USART_CTRLA_FORM(0) |	   /* USART frame, no parity */
	    SERCOM_USART_CTRLA_SAMPA(0) |	   /* Sample adjust 7-8-9 */
	    SERCOM_USART_CTRLA_SAMPR(0) |
	    SERCOM_USART_CTRLA_MODE(1) |	   /* USART mode, int clk */
	    SERCOM_USART_CTRLA_TXPO(aa->aa_txpad) |/* TXD pinout */
	    SERCOM_USART_CTRLA_RXPO(aa->aa_rxpad); /* RXD pinout */
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

	u->CTRLB.reg = 0;	/* 8 bits, no parity, 1 stop bit. */
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_MASK);

#ifdef SERCOM_USART_CTRLC_OFFSET
	u->CTRLC.reg = 0;	/* 8 bits, no parity, 1 stop bit. */
#endif
	u->RXPL.reg = 0;

	u->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
	sam_usart_syncbusy(u, SERCOM_USART_SYNCBUSY_ENABLE);

	sam_usart_configure(us, aa->aa_cfg, aa->aa_baud);

	name = (aa->aa_name != NULL) ? aa->aa_name : "Console";
#ifdef RTOS_OPTION_SOFT_IRQ
	if (aa->aa_use_softirq) {
		us->us_si_handle = rtos_soft_irq_register_named(name,
		    sam_usart_soft_interrupt, us);
		assert(us->us_si_handle != NULL);
	} else {
		us->us_si_handle = NULL;
	}
#endif

	strlcpy(irq_name, name, sizeof(irq_name));
	if ((p = strchr(irq_name, ' ')) != NULL)
		*p = '\0';
#ifdef SERCOM_USART_CTRLC_OFFSET
	else
		p = &irq_name[strlen(irq_name)];
#endif

	/* Bit#0: Tx Data Register Empty interrupt. */
#ifdef SERCOM_USART_CTRLC_OFFSET
	strcpy(p, "-TX");
#endif
	rtos_irq_register_named(irq_name, aa->aa_vector + 0, aa->aa_ipl,
	    sam_usart_interrupt, us);

#ifdef SERCOM_USART_CTRLC_OFFSET
#if 0
	/* Bit#1: Tx Complete interrupt is of no interest to us. */
	strcpy(p, "-TXC");
	rtos_irq_register_named(irq_name, aa->aa_vector + 1, aa->aa_ipl,
	    sam_usart_interrupt, us);
#endif
	/* Bit#2: Rx Complete interrupt. */
	strcpy(p, "-RX");
	rtos_irq_register_named(irq_name, aa->aa_vector + 2, aa->aa_ipl,
	    sam_usart_interrupt, us);

	/* Bit#3 - Bit#7: Mainly for errors. */
	strcpy(p, "-ERR");
	rtos_irq_register_named(irq_name, aa->aa_vector + 3, aa->aa_ipl,
	    sam_usart_interrupt, us);
#endif

	uf.uf_open = sam_usart_open;
	uf.uf_close = sam_usart_close;
	uf.uf_configure = sam_usart_configure;
	uf.uf_polled_tx = sam_usart_polled_tx;
	uf.uf_get_stats = sam_usart_get_stats;

	return uart_register(aa->aa_name, &uf, us);
}
