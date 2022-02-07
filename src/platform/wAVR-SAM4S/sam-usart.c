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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "ringbuff.h"
#include "zone_alloc.h"

/*
 * The application-specific sam-uart.h is expected to pull in the relevant
 * CMSIS header file with Uart definitions.
 */
#include "sam-usart.h"

/*
 * Note that the registers of USART devices are a superset of those for
 * UART devices, so we use the CMSIS definition for the former and use
 * a flag to indicate the flavour of any given instance.
 *
 * ToDo:	Add support for BREAK
 *		Add support for flow control
 */

struct sam_usart_state {
	sam_usart_type_t us_type;
	Usart *us_regs;
	Pdc *us_pdc;
	uint32_t us_ipl;
	uint16_t us_pdc_rcr_prev[2];
	uint16_t us_pdc_max;
	uint16_t us_pdc_idx;
#ifndef SAM_USART_NDEVS
	uint16_t us_num;
#endif
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

#ifdef SAM_USART_NDEVS
static struct sam_usart_state sam_usart_states[SAM_USART_NDEVS];

static struct sam_usart_state *
sam_usart_alloc(void)
{
	struct sam_usart_state *us;

	for (us = sam_usart_states;
	    us < &sam_usart_states[SAM_USART_NDEVS]; us++) {
		if (us->us_regs == NULL)
			return us;
	}

	return NULL;
}
#define	SAM_USART_NUM(u)	((u_int)((u) - sam_usart_states))
#else
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
#define	SAM_USART_NUM(u)	((u_int)((u)->us_num))
#endif

static __always_inline void
sam_usart_tx_irq_disable(Usart *u)
{

	u->US_IDR = US_IDR_TXRDY;
}

static __always_inline void
sam_usart_tx_irq_enable(Usart *u)
{

	u->US_IER = US_IER_TXRDY;
}

static __always_inline void
sam_usart_rx_irq_disable(Usart *u)
{

	u->US_IDR = US_IDR_RXRDY | US_IER_ENDRX | US_IER_RXBUFF |
	    US_IER_TIMEOUT | US_IDR_OVRE | US_IDR_FRAME | US_IDR_PARE;
}

static __always_inline void
sam_usart_rx_irq_enable(struct sam_usart_state *us)
{
	Usart *u = us->us_regs;
	uint32_t ier;

	ier = US_IER_OVRE | US_IER_FRAME | US_IER_PARE;

	if (us->us_pdc == NULL)
		ier |= US_IER_RXRDY;
	else
		ier |= US_IER_ENDRX | US_IER_RXBUFF | US_IER_TIMEOUT;

	u->US_IER = ier;
}

static int
sam_usart_irq_rx_ready(struct sam_usart_state *us, uint32_t st)
{
	uint32_t data;
	int rv = 0;

#define	RX_ERROR	(US_CSR_FRAME | US_CSR_PARE | US_CSR_OVRE)

	data =  us->us_regs->US_RHR;

	us->us_stats.us_rx_bytes++;

	if (us->us_uart2app == NULL)
		return 0;

	if (st & RX_ERROR)
		return 0;

	if (!ringbuff_is_full(us->us_uart2app)) {
		rv = 1;
		ringbuff_produce(us->us_uart2app, (uint8_t)(data & 0xffu));
	} else {
		rv = 0;
		us->us_stats.us_rx_ring_overruns++;
	}

	return rv;
}

static int
sam_usart_pdc_load_next(struct sam_usart_state *us)
{
	uint16_t cnt, rcr, idx, rem;
	ringbuff_t rb;
	void *ptr;
	Pdc *p = us->us_pdc;

	idx = us->us_pdc_idx;

	/*
	 * Remaining contiguous space in the current buffer?
	 */
	rem = us->us_pdc_rcr_prev[idx];

	rb = us->us_uart2app;
	cnt = ringbuff_produce_contig_next_size(rb, rem);
	if (cnt == 0)
		return 0;
	rcr = (cnt > us->us_pdc_max) ? us->us_pdc_max : cnt;
	ptr = ringbuff_produce_next_buff_pointer(rb, rem);

	if (us->us_pdc_rcr_prev[idx] == 0) {
		/*
		 * DMA is all out of buffers. Kick it off anew.
		 */
		us->us_pdc_rcr_prev[idx] = rcr;
		__DSB();
		__ISB();
		p->PERIPH_RPR = (uint32_t)(uintptr_t)ptr;
		p->PERIPH_RCR = rcr;

		cnt = ringbuff_produce_contig_next_size(rb, rcr);
		rcr = (cnt > us->us_pdc_max) ? us->us_pdc_max : cnt;
		ptr = ringbuff_produce_next_buff_pointer(rb, rcr);
	}

	if (rcr != 0) {
		idx = 1 - idx;
		us->us_pdc_rcr_prev[idx] = rcr;
		__DSB();
		__ISB();
		p->PERIPH_RNPR = (uint32_t)(uintptr_t)ptr;
		p->PERIPH_RNCR = rcr;
	}

	return 1;
}

static void
sam_usart_pdc_consumed(ringbuff_t rb, void *arg)
{
	struct sam_usart_state *us = arg;
	rtos_saved_ipl_t ipl;
	Pdc *p = us->us_pdc;

	(void) rb;

	if (p == NULL)
		return;

	ipl = rtos_ipl_raise(us->us_ipl);
	if (us->us_pdc_rcr_prev[1 - us->us_pdc_idx] == 0) {
		if (sam_usart_pdc_load_next(us) &&
		    (us->us_regs->US_IMR & US_CSR_RXBUFF) == 0) {
			us->us_regs->US_IER = US_CSR_RXBUFF |
			    US_CSR_ENDRX;
		}
	}
	rtos_ipl_restore(ipl);
}

static int
sam_usart_irq_pdc_ready(struct sam_usart_state *us, uint32_t st)
{
	Pdc *p = us->us_pdc;
	uint16_t cnt, rcr;
	uint16_t idx;
	int done = 0;

	if (unlikely(st & US_CSR_RXRDY)) {
		/*
		 * This happens if we receive a character when both DMA
		 * counters are zero - as in, there's no free space in
		 * the ring buffer.
		 * We have to read the Rx holding register and discard
		 * the character in order to clear the interrupt.
		 */
		(void) us->us_regs->US_RHR;

		/* Might as well account for the lost byte */
		us->us_stats.us_rx_bytes++;

		/* Log the ring buffer over-run */
		us->us_stats.us_rx_ring_overruns++;
	}

	/*
	 * If there's no Rx ring buffer associated then there's
	 * nothing more to do.
	 */
	if (unlikely(us->us_uart2app == NULL)) {
		p->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
		return 0;
	}

	idx = us->us_pdc_idx;

	rcr = (uint16_t) p->PERIPH_RCR;

	if (unlikely(p->PERIPH_RNCR == 0)) {
		/*
		 * PDC has switched pointers/counts.
		 * Work out how many bytes it transferred since the last time.
		 */
		if (likely((cnt = us->us_pdc_rcr_prev[idx]) != 0)) {
			us->us_pdc_rcr_prev[idx] = 0;
			ringbuff_produce_contig(us->us_uart2app,
			    (ringbuff_len_t)cnt);
			done = 1;
			us->us_stats.us_rx_bytes += cnt;

			/* Update the current buffer index */
			idx = 1 - idx;
			us->us_pdc_idx = idx;
		}

		/*
		 * Re-fetch RCR in case it switched between the first read
		 * of RCR and the above read of RNCR.
		 */
		rcr = (uint16_t) p->PERIPH_RCR;
	}

	/*
	 * Work out how much of the current buffer is occupied
	 */
	cnt = us->us_pdc_rcr_prev[idx] ? (us->us_pdc_rcr_prev[idx] - rcr) : 0;

	if (cnt != 0) {
		ringbuff_produce_contig(us->us_uart2app, (ringbuff_len_t)cnt);
		us->us_pdc_rcr_prev[idx] = rcr;
		us->us_stats.us_rx_bytes += cnt;
		done = 1;
	}

	/* See if we can reload the next DMA segment */
	if (p->PERIPH_RNCR == 0) {
		if (sam_usart_pdc_load_next(us) == 0 && (st & US_CSR_RXBUFF)) {
			/*
			 * We're all out of buffers. Disable RXBUFF/ENDRX
			 * interrupts until we have space available.
			 */
			us->us_regs->US_IDR = US_CSR_RXBUFF | US_CSR_ENDRX;
		}
	}

	return done;
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
	us->us_regs->US_THR = (uint32_t) ch;
	us->us_stats.us_tx_bytes++;
}

static void
sam_usart_interrupt(void *arg)
{
	struct sam_usart_state *us = arg;
	Usart *u = us->us_regs;
	int rx_work, tx_work;
	uint32_t st;

	rx_work = tx_work = 0;

	st = u->US_CSR & u->US_IMR;

#define	RX_ATTENTION	(US_CSR_RXRDY | US_CSR_ENDRX | US_CSR_TIMEOUT)

	do {
		if (us->us_pdc == NULL) {
			if ((st & US_CSR_RXRDY) == US_CSR_RXRDY)
				rx_work += sam_usart_irq_rx_ready(us, st);
		} else
		if ((st & (US_CSR_ENDRX | US_CSR_TIMEOUT |
		    US_CSR_RXBUFF)) != 0) {
			if (st & US_CSR_TIMEOUT)
				u->US_CR = US_CR_STTTO;
			rx_work += sam_usart_irq_pdc_ready(us, st);
		}

		if ((st & (US_CSR_FRAME | US_CSR_PARE | US_CSR_OVRE)) != 0) {
			if (st & US_CSR_FRAME)
				us->us_stats.us_rx_fe++;
			if (st & US_CSR_PARE)
				us->us_stats.us_rx_pe++;
			if (st & US_CSR_OVRE)
				us->us_stats.us_rx_fifo_overruns++;
		}

		if ((st & US_CSR_TXRDY) != 0) {
			sam_usart_irq_tx_done(us);
			tx_work = 1;
		}

		st = u->US_CSR & u->US_IMR;
	} while ((st & (RX_ATTENTION | US_CSR_TXRDY)) != 0);

	if (st & RX_ERROR)
		u->US_CR = US_CR_RSTSTA;

#ifdef RTOS_OPTION_SOFT_IRQ
	uint32_t si_flag = us->us_si_flag;
#endif

	if (rx_work && us->us_uart2app) {
		if (us->us_pdc) {
			__DSB();
			__ISB();
		}
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
	while ((us->us_regs->US_CSR & US_CSR_TXRDY) == 0)
		;

	us->us_regs->US_THR = (uint32_t) ch;
	us->us_stats.us_tx_bytes++;
	us->us_stats.us_tx_bytes_polled++;
	rtos_ipl_restore(ipl);
}

static uint8_t
sam_usart_open(void *arg, ringbuff_t uart2app, ringbuff_t app2uart)
{
	struct sam_usart_state *us = arg;
	uint32_t cr = 0;

	us->us_regs->US_CR = US_CR_RSTRX | US_CR_RSTTX;
	if ((us->us_uart2app = uart2app) != NULL) {
		Pdc *p;
		if ((p = us->us_pdc) != NULL) {
			uint16_t cnt;

			cnt = ringbuff_produce_contig_size(uart2app);

			us->us_pdc_rcr_prev[0] = 0;
			us->us_pdc_rcr_prev[1] = 0;
			us->us_pdc_idx = 0;
			us->us_pdc_max = cnt / 2;

			sam_usart_pdc_load_next(us);

			ringbuff_producer_init(uart2app, sam_usart_pdc_consumed,
			    us);

			us->us_regs->US_RTOR = 20;	/* XXX */
			cr |= US_CR_STTTO;

			p->PERIPH_PTCR = PERIPH_PTCR_RXTEN;
		}
		cr |= US_CR_RXEN;
	}

	if ((us->us_app2uart = app2uart) != NULL) {
		cr |= US_CR_TXEN;
		ringbuff_consumer_init(app2uart, sam_usart_tx_avail_cb, us);
	}

	us->us_regs->US_CR = cr;

	if (cr & US_CR_RXEN)
		sam_usart_rx_irq_enable(us);

	return 1;
}

static void
sam_usart_close(void *arg)
{
	struct sam_usart_state *us = arg;

	sam_usart_tx_irq_disable(us->us_regs);
	sam_usart_rx_irq_disable(us->us_regs);
	if (us->us_pdc != NULL)
		us->us_pdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS;
	us->us_regs->US_CR = US_CR_RXDIS | US_CR_TXDIS;

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
sam_usart_actual_speed(struct sam_usart_state *us, uint32_t brgr, uint32_t mr)
{
	uint32_t cd, fp;

	if (brgr == 0)
		return 0;

	if (us->us_type == SAM_USART_TYPE_UART)
		return F_CPU / (16u * brgr);

	cd = ((brgr & US_BRGR_CD_Msk) >> US_BRGR_CD_Pos) * 8u;
	fp = (brgr & US_BRGR_FP_Msk) >> US_BRGR_FP_Pos;
	cd += fp;
	if ((mr & US_MR_OVER) == 0)
		cd *= 2u;

	return (F_CPU + (cd / 2)) / cd;
}

static uint32_t
sam_usart_calc_brgr(struct sam_usart_state *us, uint32_t baud, uint32_t *over)
{
	uint32_t fp8, cd8, fp16, cd16;
	uint32_t baud8, baud16;

	fp16 = ((8 * F_CPU) + ((16 * baud) / 2)) / (16 * baud);
	cd16 = fp16 >> 3;
	fp16 &= 0x07;

	fp8 = ((8 * F_CPU) + ((8 * baud) / 2)) / (8 * baud);
	cd8 = fp8 >> 3;
	fp8 &= 0x07;

	/*
	 * There's no way in hell we will ever exceed the limit for 'CD'
	 * for the supported baud rates, as we'd have to be using a
	 * ludicrously small F_CPU.
	 */
	if (cd16 > 0xffffu || cd8 > 0xffffu)
		return 0;

	/* Both will be zero if the rate is unattainably high */
	if (cd16 == 0 && cd8 == 0)
		return 0;

	cd8 = (cd8 << US_BRGR_CD_Pos) | (fp8 << US_BRGR_FP_Pos);
	baud8 = sam_usart_actual_speed(us, cd8, US_MR_OVER);

	/* If cd16 is zero then we have to go with cd8 */
	if (cd16 == 0) {
		*over |= US_MR_OVER;
		return cd8;
	}

	cd16 = (cd16 << US_BRGR_CD_Pos) | (fp16 << US_BRGR_FP_Pos);
	baud16 = sam_usart_actual_speed(us, cd16, 0);

	/*
	 * Go with whichever configuration has the least error, but
	 * prefer x16 if both are the same.
	 */
	if (abs((int)(baud - baud16)) <= abs((int)(baud - baud8))) {
		*over &= ~US_MR_OVER;
		return cd16;
	}

	*over |= US_MR_OVER;
	return cd8;
}

static uint32_t
sam_usart_configure(void *arg, uint16_t cfg, uint32_t custom_baud)
{
	struct sam_usart_state *us = arg;
	uint32_t mr, brgr, baud, actual_baud;

	baud = uart_cfg_to_baud(cfg, custom_baud);

	mr = US_MR_USART_MODE_NORMAL | US_MR_USCLKS_MCK;

	switch (cfg & UART_PARITY_MASK) {
	default:
		mr |= US_MR_PAR_NO;
		break;
	case UART_PARITY_ODD:
		mr |= US_MR_PAR_ODD;
		break;
	case UART_PARITY_EVEN:
		mr |= US_MR_PAR_EVEN;
		break;
	}

	if (us->us_type == SAM_USART_TYPE_USART) {
		if ((cfg & UART_STOPBITS_MASK) == UART_STOPBITS_2)
			mr |= US_MR_NBSTOP_2_BIT;

		switch (cfg & UART_CHARSIZE_MASK) {
		case UART_CHARSIZE_5:
			mr |= US_MR_CHRL_5_BIT;
			break;
		case UART_CHARSIZE_6:
			mr |= US_MR_CHRL_6_BIT;
			break;
		case UART_CHARSIZE_7:
			mr |= US_MR_CHRL_7_BIT;
			break;
		case UART_CHARSIZE_8:
			mr |= US_MR_CHRL_8_BIT;
			break;
		}

		brgr = sam_usart_calc_brgr(us, baud, &mr);
	} else {
		brgr = F_CPU / (16u * baud);
	}

	actual_baud = sam_usart_actual_speed(us, brgr, mr);

	/* We need to be within 1.5% of the desired baud rate. */
	if (actual_baud && (uint32_t)abs((int)baud - (int)actual_baud) >
	    (baud / 75u)) {
		actual_baud = 0;
	}

	/* Switch to the new rate if it is within spec. */
	if (actual_baud) {
		rtos_saved_ipl_t ipl = rtos_ipl_raise(us->us_ipl);
		us->us_regs->US_MR = mr;
		us->us_regs->US_BRGR = brgr;
		rtos_ipl_restore(ipl);
	}

	return actual_baud;
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
sam_usart_init(const sam_usart_attach_args_t *aa)
{
	static struct uart_funcs uf;
	struct sam_usart_state *us;

	if ((us = sam_usart_alloc()) == NULL)
		return -1;

	us->us_type = aa->aa_type;
	us->us_regs = aa->aa_regs.usart;
	us->us_ipl = aa->aa_ipl;

#ifndef NDEBUG
	const char *name;
	switch (aa->aa_vector) {
	case UART0_IRQn:
		name = "UART0";
		break;
	case UART1_IRQn:
		name = "UART1";
		break;
	case USART0_IRQn:
		name = "USART0";
		break;
	default:
		name = NULL;
		break;
	}
#endif

	/*
	 * DMA cannot be used on a plain UART because of the lack of
	 * receive timer. One day I'll see if it can be emulated using
	 * a regular system timer.
	 */
	if (us->us_type == SAM_USART_TYPE_USART && aa->aa_use_dma)
		us->us_pdc = (Pdc *)((uintptr_t)us->us_regs + 0x100u);
	else
		us->us_pdc = NULL;

	us->us_regs->US_CR = US_CR_RSTRX | US_CR_RSTTX;
	sam_usart_rx_irq_disable(us->us_regs);
	sam_usart_tx_irq_disable(us->us_regs);
	sam_usart_configure(us, aa->aa_cfg, aa->aa_baud);

#ifdef RTOS_OPTION_SOFT_IRQ
	if (aa->aa_use_softirq) {
		us->us_si_handle = rtos_soft_irq_register_named(name,
		    sam_usart_soft_interrupt, us);
		assert(us->us_si_handle != NULL);
	} else {
		us->us_si_handle = NULL;
	}
#endif

	rtos_irq_register_named(name, aa->aa_vector, aa->aa_ipl,
	    sam_usart_interrupt, us);

	uf.uf_open = sam_usart_open;
	uf.uf_close = sam_usart_close;
	uf.uf_configure = sam_usart_configure;
	uf.uf_polled_tx = sam_usart_polled_tx;
	uf.uf_get_stats = sam_usart_get_stats;

	return uart_register(aa->aa_name, &uf, us);
}
