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

#include "rtos.h"
#include "platform.h"
#include "timer.h"
#include "zone_alloc.h"
#include "sam_sercom_spi.h"

#include "timer.h"
#include "sam_cmcc.h"
#include "sam_dmac.h"
#define	SAM_SPI_USE_DMAC

struct sam_spi_state {
	SercomSpi *ss_regs;
	bool ss_tx_only;
	timer_timeout_t ss_timeout;
	struct spi_interface ss_spi;
	void (*ss_cs_func)(bool);
#ifdef SAM_SPI_USE_DMAC
	uint16_t ss_dma_threshold;
	uint16_t ss_dma_irq_threshold;
	void *ss_wr_dma_chan;
	sam_dmac_desc_t ss_wr_dma_desc;
	void *ss_rd_dma_chan;
	sam_dmac_desc_t ss_rd_dma_desc;
	volatile sam_dmac_status_t ss_dma_status;
	rtos_sema_t ss_sema;
#endif
};

static int
sam_spi_readwrite_pio(struct sam_spi_state *ss, const struct spi_iovec *io)
{
	SercomSpi *r = ss->ss_regs;
	uint32_t wrdata = 0x00u;
	uint16_t wrlen, rdlen;
	const uint8_t *wr;
	uint8_t *rd, mask;

	wr = io->io_wr;
	rd = io->io_rd;
	wrlen = io->io_len;

	if (ss->ss_tx_only) {
		rdlen = 0;
		mask = SERCOM_SPI_INTFLAG_DRE;
	} else {
		rdlen = wrlen;
		mask = SERCOM_SPI_INTFLAG_DRE | SERCOM_SPI_INTFLAG_RXC;
	}

	/*
	 * For each byte we need to read or write...
	 */
	while (wrlen || rdlen) {
		/*
		 * Verify we are in no danger of a buffer overflow.
		 */
		if (mask != SERCOM_SPI_INTFLAG_DRE) {
			assert(wrlen <= rdlen);
			assert((rdlen - wrlen) <= 2);
		}

		if (wrlen && (r->INTFLAG.reg & mask) == SERCOM_SPI_INTFLAG_DRE){
			/*
			 * We can write another byte. If the caller supplied
			 * a write buffer, use the next value. Otherwise
			 * write a dummy value.
			 */
			if (wr != NULL)
				wrdata = (uint32_t)*wr++;
			r->DATA.reg = wrdata;
			wrlen--;
		}

		if (rdlen && r->INTFLAG.bit.RXC) {
			/*
			 * We can read another byte.
			 */
			uint32_t rdata = r->DATA.reg;

			if (rd != NULL)
				*rd++ = (uint8_t)rdata;
			rdlen--;
		}

		if (timer_timeout_expired(&ss->ss_timeout))
			break;
	}

	if (mask != SERCOM_SPI_INTFLAG_DRE) {
		/* We should never see a buffer overflow. */
		assert(r->STATUS.bit.BUFOVF == 0);
	}

	return (wrlen | rdlen) ? -1 : 0;
}

#ifdef SAM_SPI_USE_DMAC
static uint32_t sam_spi_dma_dummy_tx;
static uint32_t sam_spi_dma_dummy_rx;

static void
sam_spi_dma_callback(void *cookie, sam_dmac_status_t status)
{
	struct sam_spi_state *ss = cookie;

	ss->ss_dma_status = status;
	rtos_sema_give_isr(ss->ss_sema);
}

static int
sam_spi_readwrite_dma(struct sam_spi_state *ss, const struct spi_iovec *io,
    bool irq_mode)
{
	sam_dmac_status_t status;
	sam_dmac_desc_t dd;
	uint32_t blockact;
	uint16_t ctrl;

	blockact = irq_mode ? SAM_DMAC_BLOCKACT_INT : SAM_DMAC_BLOCKACT_NOACT;

	if (ss->ss_tx_only == false) {
		/* Configure the read channel */
		assert(ss->ss_rd_dma_desc != NULL);

		dd = ss->ss_rd_dma_desc;
		dd->dd_cnt = io->io_len;

		ctrl = SAM_DMAC_CTRL_VALID |
		    /* 1 byte steps */
		    SAM_DMAC_CTRL_STEPSIZE(0) |
		    SAM_DMAC_CTRL_STEPSEL_DST |
		    /* 1 byte beats */
		    SAM_DMAC_CTRL_BEATSIZE(SAM_DMAC_BEAT_8BITS) |
		    /* Configure completion action. */
		    SAM_DMAC_CTRL_BLOCKACT(blockact);

		if (io->io_rd != NULL) {
			/*
			 * We're reading data from the SPI device.
			 */
			/* Increment destination address. */
			ctrl |= SAM_DMAC_CTRL_DSTINC;

			dd->dd_dst = (void *)((uintptr_t)io->io_rd +
			    io->io_len);
		} else {
			/*
			 * Data from the SPI device is to be ignored.
			 */
			dd->dd_dst = &sam_spi_dma_dummy_rx;
		}

		dd->dd_ctrl = ctrl;
		sam_dmac_channel_start(ss->ss_rd_dma_chan, irq_mode);
		blockact = SAM_DMAC_BLOCKACT_NOACT;
	} else {
		/*
		 * Data from the SPI device is ignored.
		 */
		assert(ss->ss_rd_dma_desc == NULL);
	}

	/*
	 * Configure the 'write' DMA descriptor.
	 */
	assert(ss->ss_wr_dma_desc != NULL);

	dd = ss->ss_wr_dma_desc;
	dd->dd_cnt = io->io_len;

	ctrl = SAM_DMAC_CTRL_VALID |
	    /* 1 byte steps */
	    SAM_DMAC_CTRL_STEPSIZE(0) |
	    SAM_DMAC_CTRL_STEPSEL_SRC |
	    /* 1 byte beats */
	    SAM_DMAC_CTRL_BEATSIZE(SAM_DMAC_BEAT_8BITS) |
	    SAM_DMAC_CTRL_BLOCKACT(blockact);

	if (io->io_wr == NULL) {
		/*
		 * Write dummy data to the SPI device.
		 */
		dd->dd_src = &sam_spi_dma_dummy_tx;
	} else {
		/*
		 * We're writing data from our buffer to SPI device.
		 */
		ctrl |= SAM_DMAC_CTRL_SRCINC;

		dd->dd_src = (const void *)((uintptr_t)io->io_wr + io->io_len);
	}
	dd->dd_ctrl = ctrl;

	sam_dmac_channel_start(ss->ss_wr_dma_chan,
	    blockact == SAM_DMAC_BLOCKACT_INT);

	if (irq_mode == false) {
		void *poll_chan;

		/* Poll for DMA completion. */

		poll_chan = ss->ss_tx_only ? ss->ss_wr_dma_chan :
		    ss->ss_rd_dma_chan;

		do {
			status = sam_dmac_channel_status(poll_chan);
			if (status != SAM_DMAC_STATUS_BUSY)
				break;
		} while (!timer_timeout_expired(&ss->ss_timeout));

		if (status == SAM_DMAC_STATUS_BUSY)
			status = SAM_DMAC_STATUS_TIMEOUT;
	} else {
		/* DMA will interrupt on completion. */

		if (rtos_sema_take_timed(ss->ss_sema, 1000) == 0)
			status = SAM_DMAC_STATUS_TIMEOUT;
		else
			status = ss->ss_dma_status;
	}

	if (status != SAM_DMAC_STATUS_COMPLETE) {
		if (ss->ss_tx_only == false)
			sam_dmac_channel_stop(ss->ss_rd_dma_chan);
		sam_dmac_channel_stop(ss->ss_wr_dma_chan);
	} else
	if (io->io_rd != NULL) {
		/*
		 * Read buffers must be cleaned/invalidated now in case the
		 * buffer was read elsewhere while the transfer was in progress.
		 * For example, if the buffer straddled cache lines shared
		 * with some other variables.
		 */
		CMCC_CleanInvalidateDCache_by_Addr(io->io_rd, io->io_len);
	}

#ifndef NDEBUG
	SercomSpi *r = ss->ss_regs;
	if (status != SAM_DMAC_STATUS_COMPLETE ||
	    (!ss->ss_tx_only && (r->STATUS.bit.BUFOVF || r->INTFLAG.bit.ERROR ||
	    r->INTFLAG.bit.RXC))) {
		printf("SPI DMA %u: ST %04" PRIx16 ", IF %02x. r %p, "
		    "len %" PRIu16 ", wr %p, rd %p\n", (unsigned int)status,
		    r->STATUS.reg, (unsigned int)r->INTFLAG.reg, (void *)r,
		    io->io_len, (const void *)io->io_wr, (void *)io->io_rd);

		(void) r->DATA.reg;
		r->STATUS.reg = SERCOM_SPI_STATUS_BUFOVF;
		r->INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;
	}
#endif

	return (status == SAM_DMAC_STATUS_COMPLETE) ? 0 : -1;
}
#endif /* SAM_SPI_USE_DMAC */

static int
sam_sercom_spi_readwrite(const struct spi_interface *spi, unsigned int nio,
    const struct spi_iovec *io)
{
	struct sam_spi_state *ss = spi->si_cookie;
	int rv = 0;

	/*
	 * Apply a 2 second timeout for the transfer.
	 */
	timer_timeout_start(&ss->ss_timeout, 2000);

	/* Assert /CS in software if necessary. */
	if (ss->ss_cs_func != NULL)
		(ss->ss_cs_func)(true);

	/*
	 * Transfer each of the iovecs
	 */
	while (nio--) {
		assert(io->io_len != 0);

		if (ss->ss_tx_only)
			assert(io->io_rd == NULL);

		if (io->io_rd != NULL) {
			assert((uintptr_t)io->io_rd >= HSRAM_ADDR);
			assert((uintptr_t)io->io_rd + io->io_len <=
			    (HSRAM_ADDR + HSRAM_SIZE));
		}

		if (io->io_wr != NULL) {
			assert((uintptr_t)io->io_wr >= HSRAM_ADDR);
			assert((uintptr_t)io->io_wr + io->io_len <=
			    (HSRAM_ADDR + HSRAM_SIZE));
		}

#ifdef SAM_SPI_USE_DMAC
		if (io->io_len >= ss->ss_dma_threshold) {
			bool irq_mode = ss->ss_dma_irq_threshold &&
			    (io->io_len >= ss->ss_dma_irq_threshold);
			rv = sam_spi_readwrite_dma(ss, io++, irq_mode);
		} else
#endif
			rv = sam_spi_readwrite_pio(ss, io++);

		if (rv < 0)
			break;
	}

	if (ss->ss_cs_func != NULL) {
		/*
		 * /CS is managed in software.
		 */
		if (ss->ss_tx_only && rv == 0) {
			/*
			 * We must wait until transmission is complete before
			 * deasserting /CS.
			 */
			SercomSpi *r = ss->ss_regs;
			uint32_t reg;

			do {
				reg = r->INTFLAG.reg & (SERCOM_SPI_INTFLAG_DRE |
				    SERCOM_SPI_INTFLAG_TXC);
			} while (reg != (SERCOM_SPI_INTFLAG_DRE |
			    SERCOM_SPI_INTFLAG_TXC));

			r->INTFLAG.reg = SERCOM_SPI_INTFLAG_TXC;
		}

		/* Deassert /CS in software. */
		(ss->ss_cs_func)(false);
	}

	return rv;
}

static void
sam_spi_syncbusy(SercomSpi *r, uint32_t mask)
{

	while ((r->SYNCBUSY.reg & mask) != 0)
		;
}

const struct spi_interface *
sam_sercom_spi_attach(const sam_sercom_spi_attach_args_t *aa,
    unsigned int clock)
{
	struct sam_spi_state *ss;
	uint32_t baud;
	SercomSpi *r;
#ifdef SAM_SPI_USE_DMAC
	sam_dmac_config_t cfg;
	void *ih;
#endif

	baud = (((clock / aa->aa_baud) + 1) / 2u) - 1u;
	assert(baud <= 255);
	assert((clock / (2u * (baud + 1u))) <= aa->aa_baud);

	if ((ss = zone_malloc(sizeof(*ss))) == NULL)
		return NULL;

	ss->ss_regs = r = aa->aa_regs;
	ss->ss_tx_only = aa->aa_tx_only;
	ss->ss_cs_func = aa->aa_cs_func;

	if (r->CTRLA.reg & SERCOM_SPI_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		r->CTRLA.reg = 0;
		sam_spi_syncbusy(r, SERCOM_SPI_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	r->CTRLA.reg = SERCOM_SPI_CTRLA_SWRST;
	sam_spi_syncbusy(r, SERCOM_SPI_SYNCBUSY_SWRST);

	/* Now initialise the device in SPI master mode */
	r->CTRLA.reg =
	    SERCOM_SPI_CTRLA_MODE(3) |		/* SPI master mode */
	    (aa->aa_lsb_first ? SERCOM_SPI_CTRLA_DORD : 0) |
	    (aa->aa_sample_trailing ? SERCOM_SPI_CTRLA_CPHA : 0) |
	    (aa->aa_sck_idle_high ? SERCOM_SPI_CTRLA_CPOL : 0) |
	    SERCOM_SPI_CTRLA_DIPO((uint32_t)aa->aa_dipo) |
	    SERCOM_SPI_CTRLA_DOPO((uint32_t)aa->aa_dopo) |
	    SERCOM_SPI_CTRLA_RUNSTDBY;
	sam_spi_syncbusy(r, SERCOM_SPI_SYNCBUSY_MASK);

	r->CTRLB.reg =
	    ((ss->ss_cs_func == NULL) ? SERCOM_SPI_CTRLB_MSSEN : 0) |
	    (aa->aa_tx_only ? 0 : SERCOM_SPI_CTRLB_RXEN);
	sam_spi_syncbusy(r, SERCOM_SPI_SYNCBUSY_MASK);

	r->CTRLC.reg = SERCOM_SPI_CTRLC_ICSPACE((uint32_t)aa->aa_sck_space);
	r->BAUD.reg = SERCOM_SPI_BAUD_BAUD(baud);

	r->CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
	sam_spi_syncbusy(r, SERCOM_SPI_SYNCBUSY_MASK);

	r->STATUS.reg = SERCOM_SPI_STATUS_BUFOVF;
	r->INTFLAG.reg = SERCOM_SPI_INTFLAG_TXC | SERCOM_USART_INTFLAG_ERROR;

#ifdef SAM_SPI_USE_DMAC
	ss->ss_sema = rtos_sema_create_binary();
	assert(ss->ss_sema != NULL);
	ss->ss_dma_irq_threshold = aa->aa_dma_irq_threshold;
	ss->ss_dma_threshold = aa->aa_dma_threshold;

	/*
	 * Initialise DMA
	 */
	if (aa->aa_tx_only == false) {
		/* DMA channel for reads *from* the SPI device. */
		cfg.cfg_threshold = SAM_DMAC_THRESHOLD_1BEAT;
		cfg.cfg_burstlen = SAM_DMAC_BURSTLEN(1);
		cfg.cfg_trigact = SAM_DMAC_TRIGACT_BURST;
		cfg.cfg_trigsrc = SAM_DMAC_TRIGSRC_SERCOM_RX(aa->aa_instance);
		cfg.cfg_priority = SAM_DMAC_PRIORITY_LOW; /* Must be > TX */
		cfg.cfg_evomode = SAM_DMAC_EVOMODE_NONE;
		cfg.cfg_evact = SAM_DMAC_EVACT_NOACT;

		ss->ss_rd_dma_chan = sam_dmac_channel_alloc(&cfg,
		    &ss->ss_rd_dma_desc);
		assert(ss->ss_rd_dma_chan != NULL);
		assert(ss->ss_rd_dma_desc != NULL);

		/* Source for this channel is always the SPI DATA register. */
		ss->ss_rd_dma_desc->dd_src = &r->DATA.reg;
		ss->ss_rd_dma_desc->dd_next = NULL;

		/* Hook the completion interrupt. */
		ih = sam_dmac_irq_register(ss->ss_rd_dma_chan,
		    aa->aa_dma_name_rx, aa->aa_dma_ipl,
		    sam_spi_dma_callback, ss);
		assert(ih != NULL);
		(void)ih;
	} else {
		ss->ss_rd_dma_chan = NULL;
		ss->ss_rd_dma_desc = NULL;
	}

	/* DMA channel for writes *to* the SPI device. */
	cfg.cfg_threshold = SAM_DMAC_THRESHOLD_1BEAT;
	cfg.cfg_burstlen = SAM_DMAC_BURSTLEN(1);
	cfg.cfg_trigact = SAM_DMAC_TRIGACT_BURST;
	cfg.cfg_trigsrc = SAM_DMAC_TRIGSRC_SERCOM_TX(aa->aa_instance);
	cfg.cfg_priority = SAM_DMAC_PRIORITY_LOWEST; /* Must be < RX */
	cfg.cfg_evomode = SAM_DMAC_EVOMODE_NONE;
	cfg.cfg_evact = SAM_DMAC_EVACT_NOACT;

	ss->ss_wr_dma_chan = sam_dmac_channel_alloc(&cfg,
	    &ss->ss_wr_dma_desc);
	assert(ss->ss_wr_dma_chan != NULL);
	assert(ss->ss_wr_dma_desc != NULL);

	/* Destination for this channel is always the SPI DATA register. */
	ss->ss_wr_dma_desc->dd_dst = &r->DATA.reg;
	ss->ss_wr_dma_desc->dd_next = NULL;

	if (ss->ss_tx_only) {
		/* Hook the completion interrupt. */
		ih = sam_dmac_irq_register(ss->ss_wr_dma_chan,
		    aa->aa_dma_name_tx, aa->aa_dma_ipl,
		    sam_spi_dma_callback, ss);
		assert(ih != NULL);
		(void)ih;
	}
#endif

	ss->ss_spi.si_read_write = sam_sercom_spi_readwrite;
	ss->ss_spi.si_cookie = ss;

	return &ss->ss_spi;
}
