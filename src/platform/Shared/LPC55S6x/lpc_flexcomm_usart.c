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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "ringbuff.h"
#include "platform.h"
#include "lpc_flexcomm_usart.h"
#include "uart.h"
#include "fix16.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

struct lpc_usart_state {
	USART_Type *us_regs;
	volatile uint32_t *us_frg_reg;
	uint32_t us_clock;	/* FLEXCOMM input clock, in Hz */
	fix16_t us_clock_mhz;	/* FLEXCOMM input clock, in MHz, as fix16_t */
	uint32_t us_ipl;
#ifdef RTOS_OPTION_SOFT_IRQ
	void *us_si_handle;
	uint32_t us_si_flag;
#define	LPC_USART_SI_FLAG_RX	(1u << 0)
#define	LPC_USART_SI_FLAG_TX	(1u << 1)
#endif
	ringbuff_t us_uart2app;
	ringbuff_t us_app2uart;
	struct uart_stats us_stats;
};

#define	USART_STAT_ERROR_MASK	(USART_STAT_FRAMERRINT_MASK | \
				 USART_STAT_PARITYERRINT_MASK | \
				 USART_STAT_RXNOISEINT_MASK)
#define	USART_FIFORD_ERROR_MASK	(USART_FIFORD_FRAMERR_MASK | \
				 USART_FIFORD_PARITYERR_MASK | \
				 USART_FIFORD_RXNOISE_MASK)

static __always_inline void
lpc_usart_tx_irq_disable(USART_Type *u)
{

	u->FIFOINTENCLR = USART_FIFOINTENCLR_TXLVL_MASK;
}

static __always_inline bool
lpc_usart_tx_full(USART_Type *u)
{

	return (u->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK) == 0;
}

static int lpc_usart_irq_rx_ready(struct lpc_usart_state *us) FAST_RAMFUNC;
static int
lpc_usart_irq_rx_ready(struct lpc_usart_state *us)
{
	USART_Type *u = us->us_regs;
	ringbuff_t rb = us->us_uart2app;
	uint32_t data, cnt;
	int rv = 0;

	cnt = 0;
	while ((u->FIFOSTAT & USART_FIFOSTAT_RXNOTEMPTY_MASK) != 0) {
		data = us->us_regs->FIFORD;
		if (rb == NULL)
			continue;

		if ((data & USART_FIFORD_ERROR_MASK) != 0) {
			if (data & USART_FIFORD_RXNOISE_MASK) {
				us->us_stats.us_rx_noise++;
			} else {
				if (data & USART_FIFORD_FRAMERR_MASK)
					us->us_stats.us_rx_fe++;
				if (data & USART_FIFORD_PARITYERR_MASK)
					us->us_stats.us_rx_pe++;
			}
			continue;
		}

		if (!ringbuff_is_full(rb)) {
			rv = 1;
			ringbuff_produce(rb, (uint8_t)(data & 0xffu));
		} else {
			us->us_stats.us_rx_ring_overruns++;
		}

		cnt++;
	}

	us->us_stats.us_rx_bytes += cnt;

	return rv;
}

static int lpc_usart_irq_tx_done(struct lpc_usart_state *us) FAST_RAMFUNC;
static int
lpc_usart_irq_tx_done(struct lpc_usart_state *us)
{
	USART_Type *u = us->us_regs;
	ringbuff_len_t l;
	ringbuff_t rb;
	uint32_t cnt;

	if ((rb = us->us_app2uart) == NULL ||
	    (l = ringbuff_get_count(rb)) == 0) {
		/* No peer, or FIFO empty. Disable Tx Empty irq */
		lpc_usart_tx_irq_disable(u);
		return 0;
	}

	cnt = 0;
	while (!lpc_usart_tx_full(u) && l--) {
		u->FIFOWR = (uint32_t) ringbuff_consume(rb);
		cnt++;
	}

	us->us_stats.us_tx_bytes += cnt;
	return cnt ? 1 : 0;
}

static void lpc_usart_interrupt(void *arg) FAST_RAMFUNC;
static void
lpc_usart_interrupt(void *arg)
{
	struct lpc_usart_state *us = arg;
	USART_Type *u = us->us_regs;
	int rx_work, tx_work;
	uint32_t fifost;

	rx_work = tx_work = 0;

	u->STAT = USART_STAT_DELTACTS_MASK | USART_STAT_DELTARXBRK_MASK |
	    USART_STAT_START_MASK | USART_STAT_FRAMERRINT_MASK |
	    USART_STAT_PARITYERRINT_MASK | USART_STAT_RXNOISEINT_MASK |
	    USART_STAT_ABERR_MASK;
	fifost = u->FIFOINTSTAT & u->FIFOINTENSET;

	do {
		if (fifost & USART_FIFOINTSTAT_RXERR_MASK) {
			u->FIFOSTAT = USART_FIFOSTAT_RXERR_MASK;
			u->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;
			us->us_stats.us_rx_fifo_overruns++;
			fifost = u->FIFOINTSTAT & u->FIFOINTENSET;
		}

		/* Process the Rx FIFO. */
		if ((fifost & USART_FIFOINTSTAT_RXLVL_MASK) != 0)
			rx_work += lpc_usart_irq_rx_ready(us);

		/* Process the Tx FIFO. */
		if ((fifost & USART_FIFOINTSTAT_TXLVL_MASK) != 0)
			tx_work += lpc_usart_irq_tx_done(us);

		u->STAT = USART_STAT_DELTACTS_MASK | USART_STAT_DELTARXBRK_MASK |
		    USART_STAT_START_MASK | USART_STAT_FRAMERRINT_MASK |
		    USART_STAT_PARITYERRINT_MASK | USART_STAT_RXNOISEINT_MASK |
		    USART_STAT_ABERR_MASK;
		fifost = u->FIFOINTSTAT & u->FIFOINTENSET;
	} while (fifost != 0);

#ifdef RTOS_OPTION_SOFT_IRQ
	uint32_t si_flag = us->us_si_flag;
#endif

	if (rx_work && us->us_uart2app) {
#ifdef RTOS_OPTION_SOFT_IRQ
		if (us->us_si_handle != NULL)
			si_flag |= LPC_USART_SI_FLAG_RX;
		else
#endif
		ringbuff_produce_done(us->us_uart2app);
	}

	if (tx_work && us->us_app2uart) {
#ifdef RTOS_OPTION_SOFT_IRQ
		if (us->us_si_handle != NULL)
			si_flag |= LPC_USART_SI_FLAG_TX;
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
static void lpc_usart_soft_interrupt(void *arg) FAST_RAMFUNC;
static void
lpc_usart_soft_interrupt(void *arg)
{
	struct lpc_usart_state *us = arg;
	rtos_saved_ipl_t ipl;
	uint32_t si_flag;

	ipl = rtos_ipl_raise(us->us_ipl);
	si_flag = us->us_si_flag;
	us->us_si_flag = 0;
	rtos_ipl_restore(ipl);

	if (si_flag & LPC_USART_SI_FLAG_RX)
		ringbuff_produce_done(us->us_uart2app);

	if (si_flag & LPC_USART_SI_FLAG_TX)
		ringbuff_consume_done(us->us_app2uart);
}
#endif /* RTOS_OPTION_SOFT_IRQ */

/*
 * Invoked whenever data is received from the application and has
 * been loaded into the ring buffer.
 */
static void lpc_usart_tx_avail_cb(ringbuff_t rb, void *arg) FAST_RAMFUNC;
static void
lpc_usart_tx_avail_cb(ringbuff_t rb, void *arg)
{
	struct lpc_usart_state *us = arg;

	(void) rb;

	us->us_regs->FIFOINTENSET = USART_FIFOINTENSET_TXLVL_MASK;
}

static void
lpc_usart_polled_tx(void *arg, uint8_t ch)
{
	struct lpc_usart_state *us = arg;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(us->us_ipl);

	while (lpc_usart_tx_full(us->us_regs))
		;

	us->us_regs->FIFOWR = (uint32_t) ch;
	us->us_stats.us_tx_bytes++;
	us->us_stats.us_tx_bytes_polled++;

	rtos_ipl_restore(ipl);
}

static uint8_t
lpc_usart_open(void *arg, ringbuff_t uart2app, ringbuff_t app2uart)
{
	struct lpc_usart_state *us = arg;
	USART_Type *u = us->us_regs;
	uint32_t ctl = 0;

	DBFPRINTF("%" PRIx32 ": u2a %p, a2u %p\n", us->us_regs,
	    uart2app, app2uart);

	if ((us->us_app2uart = app2uart) != NULL)
		ringbuff_consumer_init(app2uart, lpc_usart_tx_avail_cb, us);
	else
		ctl |= USART_CTL_TXDIS_MASK;

	u->CTL = ctl;

	if ((us->us_uart2app = uart2app) != NULL) {
		rtos_saved_ipl_t ipl = rtos_ipl_raise(us->us_ipl);
		u->INTENSET = USART_STAT_ERROR_MASK;
		u->FIFOINTENSET = USART_FIFOINTENSET_RXERR_MASK |
		    USART_FIFOINTENSET_RXLVL_MASK;
		rtos_ipl_restore(ipl);
	}

	return 1;
}

static void
lpc_usart_close(void *arg)
{
	struct lpc_usart_state *us = arg;
	USART_Type *u = us->us_regs;
	rtos_saved_ipl_t ipl;

	DBFPRINTF("%" PRIx32 "\n", us->us_regs);

	ipl = rtos_ipl_raise(us->us_ipl);
	lpc_usart_tx_irq_disable(us->us_regs);
	u->INTENCLR = USART_STAT_ERROR_MASK;
	u->FIFOINTENCLR = USART_FIFOINTENCLR_RXERR_MASK |
	    USART_FIFOINTENCLR_RXLVL_MASK;

	/* Clear any pending interrupts. */
	u->INTENCLR = u->INTENSET;
	u->STAT = u->STAT;
	u->FIFOSTAT = USART_FIFOSTAT_RXERR_MASK | USART_FIFOSTAT_TXERR_MASK;

	rtos_ipl_restore(ipl);

	/* Disable the USART. */
	u->CFG &= ~USART_CFG_ENABLE_MASK;

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
lpc_calculate_frg(struct lpc_usart_state *us, uint32_t desired_clock,
    uint32_t *actual_clock)
{
	fix16_t frg_mhz, v;
	uint32_t r;

	assert(desired_clock <= us->us_clock);
	assert(desired_clock >= (us->us_clock / 2u));

	/*
	 * Convert desired FRG clock to MHz.
	 */
	frg_mhz = fix16_from_int((int)(desired_clock / 1000u));
	if ((desired_clock % 1000u) >= 500u)
		frg_mhz += F16(1.0);
	frg_mhz = fix16_div(frg_mhz, F16(1000.0));

	/* Calculate the corresponding MULT value. */
	v = fix16_div(us->us_clock_mhz, frg_mhz) - F16(1.0);
	v = fix16_mul(v, F16(255.0));

	/* Combine to form the fractional generator value. */
	r = (uint32_t)fix16_to_int(v);
	r = SYSCON_FLEXFRG0CTRL_DIV(0xffu) | SYSCON_FLEXFRG0CTRL_MULT(r);

	if (actual_clock != NULL) {
		uint32_t clk;
		uint64_t x;

		/* Work backwards to calculate the true FRG output clock. */
		v = fix16_from_int((int)((r & SYSCON_FLEXFRG0CTRL_MULT_MASK) >>
		    SYSCON_FLEXFRG0CTRL_MULT_SHIFT));
		v = fix16_div(v, F16(255.0)) + F16(1.0);
		v = fix16_div(us->us_clock_mhz, v);

		/*
		 * 'v' now holds the actual FRG output clock frequency, in
		 * MHz, as a fix16_t value. We need to convert this to Hz
		 * in a uint32_t with as little loss of precision as possible.
		 *
		 * The easiest way is to multiply the raw fix16_t value by
		 * 1000000 then divide by 65536. Obviously, the intermediate
		 * result will need a 64-bit type, but it will avoid floating
		 * point arithmetic. In fact, gcc is smart enough to perform
		 * this calculation in-line using a handful of instructions.
		 */
		x = 1000000ull * (uint32_t)v;
		clk = (uint32_t)(x >> 16);
		if ((x & 0xffffull) >= 0x8000ull)
			clk++;
		*actual_clock = clk;
	}

	return r;
}

struct lpc_baud_cfg {
	uint32_t bc_osr;
	uint32_t bc_brg;
	uint32_t bc_frg;
};

static bool
lpc_baud_me_harder(struct lpc_usart_state *us, uint32_t baud,
    struct lpc_baud_cfg *bc)
{
	uint32_t best_err = 0xffffffffu;

	/* For all valid oversample selection values... */
	for (uint32_t osr = 16u; osr >= 5; osr--) {
		uint32_t target_clk, brg, fc = osr * baud;

		/*
		 * Figure out the best BRG value which results in an
		 * FRG output between (us->us_clock / 2) and us->us_clock.
		 */
		brg = us->us_clock / (2u * fc);
		if ((brg * fc) < (us->us_clock / 2u))
			brg++;

		/* For all sensible BRG values... */
		while (brg >= 1u && brg < 0x10000u &&
		    (target_clk = (brg * fc)) <= us->us_clock) {
			uint32_t err, frg, actual_clk;

			/*
			 * Compute the associated FRG and the actual clock
			 * it will provide...
			 */
			frg = lpc_calculate_frg(us, target_clk, &actual_clk);

			/*
			 * How close is the actual clock to the desired value?
			 */
			if (actual_clk > target_clk)
				err = actual_clk - target_clk;
			else
				err = target_clk - actual_clk;

			/*
			 * Record the configuration which achieves the
			 * lowest error, and is within 3% of the target.
			 */
			if (err < best_err &&
			    err <= ((target_clk / 100u) * 3u)) {
				bc->bc_osr = osr - 1u;
				bc->bc_brg = brg - 1u;
				bc->bc_frg = frg;
				best_err = err;
				DBFPRINTF("%" PRIx32 ": err %" PRIu32 ", osr 0x%"
				    PRIx32 ", brg 0x%" PRIx32 ", frg 0x%" PRIx32
				    "\n", us->us_regs, err, osr - 1u, brg - 1u,
				    frg);
			}

			brg++;
		}
	}

	/* Return 'true' if we found a suitable configuration. */
	return best_err != 0xffffffffu;
}

static uint32_t
lpc_usart_configure(void *arg, uint16_t ucfg, uint32_t custom_baud)
{
	struct lpc_usart_state *us = arg;
	struct lpc_baud_cfg bc;
	USART_Type *u = us->us_regs;
	uint32_t baud, cfg;
	rtos_saved_ipl_t ipl;

	/* Get the requested baud rate. */
	baud = uart_cfg_to_baud(ucfg, custom_baud);

	DBFPRINTF("%" PRIx32 ": ucfg %04x, baud %" PRIu32 "\n", us->us_regs,
	    ucfg, baud);

	if (baud == 0)
		return 0;

	/*
	 * Datasheet section 34.3 hints at 6.25 MHz being the maximum
	 * support baudrate in async mode.
	 */
	assert(baud <= 6250000u);

	/* Work out the best way to achieve the requested baud rate. */
	if (lpc_baud_me_harder(us, baud, &bc) == false) {
		/* Requested baud rate is unachievable. */
		DBFPRINTF("%" PRIx32 ": unachievable baudrate\n", us->us_regs);
		return 0;
	}

	switch (ucfg & UART_PARITY_MASK) {
	default:
		cfg = USART_CFG_PARITYSEL(0);
		break;
	case UART_PARITY_ODD:
		cfg = USART_CFG_PARITYSEL(3);
		break;
	case UART_PARITY_EVEN:
		cfg = USART_CFG_PARITYSEL(2);
		break;
	}

	if ((ucfg & UART_STOPBITS_MASK) == UART_STOPBITS_2)
		cfg |= USART_CFG_STOPLEN(1);

	switch (ucfg & UART_CHARSIZE_MASK) {
	default:
		break;
	case UART_CHARSIZE_7:
		cfg |= USART_CFG_DATALEN(0);
		break;
	case UART_CHARSIZE_8:
		cfg |= USART_CFG_DATALEN(1);
		break;
	}

	DBFPRINTF("%" PRIx32 ": Writing cfg 0x%" PRIx32 ", osr 0x%" PRIx32
	    ", brg 0x%" PRIx32 ", frg 0x%" PRIx32 "\n", us->us_regs,
	    cfg, bc.bc_osr, bc.bc_brg, bc.bc_frg);

	/*
	 * Make the configuration current.
	 */
	ipl = rtos_ipl_raise(us->us_ipl);

	u->CFG = cfg;
	__DSB();
	__ISB();
	*us->us_frg_reg = bc.bc_frg;
	__DSB();
	__ISB();
	u->CFG = cfg | USART_CFG_ENABLE_MASK;
	u->BRG = USART_BRG_BRGVAL(bc.bc_brg);
	u->OSR = USART_OSR_OSRVAL(bc.bc_osr);

	rtos_ipl_restore(ipl);

	DBFPRINTF("%" PRIx32 ": Registers cfg 0x%" PRIx32 ", osr 0x%" PRIx32
	    ", brg 0x%" PRIx32 ", frg 0x%" PRIx32 "\n", us->us_regs,
	    u->CFG, u->OSR, u->BRG, *us->us_frg_reg);

	return baud;
}

static void
lpc_usart_get_stats(void *arg, struct uart_stats *s)
{
	struct lpc_usart_state *us = arg;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(us->us_ipl);
	memcpy(s, &us->us_stats, sizeof(*s));
	rtos_ipl_restore(ipl);
}

int
lpc_flexcomm_usart_attach(const lpc_flexcomm_usart_attach_args_t *aa)
{
	static USART_Type * const regs[] = USART_BASE_PTRS;
	static struct uart_funcs uf;
	struct lpc_usart_state *us;
	USART_Type *u;
	fix16_t f;

	assert(aa->aa_name != NULL);
	assert(aa->aa_instance < (sizeof(regs) / sizeof(regs[0])));

	/*
	 * Data sheet is ambiguous on the max input clock for a Flexcomm
	 * in USART mode. Some places say 48 MHz, others 44 MHz, and even
	 * 32 MHz is mentioned.
	 *
	 * However, section 34.3 mentions 6.25 MHz as being the upper limit
	 * of the "function clock" when in async mode.
	 *
	 * I'm going to go with 44 MHz. The 6.25 MHz is much more likely to
	 * be the max async baudrate.
	 */
	assert(aa->aa_clock <= 44000000u);

	if ((us = zone_calloc(1, sizeof(*us))) == NULL)
		return -1;

	u = us->us_regs = regs[aa->aa_instance];
	us->us_frg_reg = &SYSCON->FLEXFRGXCTRL[aa->aa_instance];
	f = fix16_from_int((int)(aa->aa_clock / 1000u));
	if ((aa->aa_clock % 1000u) >= 500u)
		f += F16(1.0);
	us->us_clock_mhz = fix16_div(f, F16(1000.0));
	us->us_clock = aa->aa_clock;
	us->us_ipl = aa->aa_ipl;

	/* Ensure caller placed Flexcomm in USART mode. */
	assert((u->ID & USART_ID_ID_MASK) == USART_ID_ID(0xe010u));

	/* Configure word format and baud rate. */
	lpc_usart_configure(us, aa->aa_cfg, aa->aa_baud);

	/* Clear any pending interrupts. */
	u->INTENCLR = u->INTENSET;
	u->FIFOSTAT = USART_FIFOSTAT_RXERR_MASK | USART_FIFOSTAT_TXERR_MASK;

	/* Configure the Tx/Rx FIFOs. */
	u->FIFOCFG = USART_FIFOCFG_EMPTYTX_MASK | USART_FIFOCFG_EMPTYRX_MASK;
	u->FIFOCFG = USART_FIFOCFG_ENABLETX_MASK | USART_FIFOCFG_ENABLERX_MASK |
	    USART_FIFOCFG_WAKERX_MASK | USART_FIFOCFG_WAKETX_MASK;

	/* Trigger Rx/Tx FIFO level interrupt at 0 bytes. */
	u->FIFOTRIG = USART_FIFOTRIG_TXLVLENA_MASK | USART_FIFOTRIG_TXLVL(0) |
	    USART_FIFOTRIG_RXLVLENA_MASK | USART_FIFOTRIG_RXLVL(0);

#ifdef RTOS_OPTION_SOFT_IRQ
	if (aa->aa_use_softirq) {
		us->us_si_handle = rtos_soft_irq_register_named(aa->aa_name,
		    lpc_usart_soft_interrupt, us);
		assert(us->us_si_handle != NULL);
	} else {
		us->us_si_handle = NULL;
	}
#endif

	rtos_irq_register_named(aa->aa_name, aa->aa_instance + FLEXCOMM0_IRQn,
	    aa->aa_ipl, lpc_usart_interrupt, us);

	uf.uf_open = lpc_usart_open;
	uf.uf_close = lpc_usart_close;
	uf.uf_configure = lpc_usart_configure;
	uf.uf_polled_tx = lpc_usart_polled_tx;
	uf.uf_get_stats = lpc_usart_get_stats;

	return uart_register(aa->aa_name, &uf, us);
}
