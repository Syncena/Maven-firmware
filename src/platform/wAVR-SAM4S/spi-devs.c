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
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "rtos.h"
#include "spi-interface.h"
#include "spi-devs.h"
#ifdef NETWORK_DEV_WINC
#include "sam4s4_wifi.h"
#elif defined(NETWORK_DEV_WIZCHIP)
#include "sam4s4_wizchip.h"
#else
#error "Unsupported network driver"
#endif
#include "display.h"
#include "shell.h"
#include "hardware.h"

#ifndef	NDEBUG
#undef	SPI_STATS
#endif

/*
 * Transfers >= the following number of bytes will be handled using
 * interrupt-on-completion. Otherwise, polled mode is used.
 * Set to zero for always-on polled mode.
 */
#define	SPI_NETWORK_IRQ_THRESHOLD	64u
#define	SPI_DISPLAY_IRQ_THRESHOLD	16u

/*
 * For DMA transfers where no Tx buffer is suppled, this is used as the
 * Tx source address. This points to the last 2KB of Flash which, in pretty
 * much all cases, will be full of 0xff bytes. If this is not defined, a
 * scratch buffer in RAM will be used.
 */
#define	SPI_DEV_WR_SCRATCH	\
	((const void *)(uintptr_t)((IFLASH0_ADDR + IFLASH0_SIZE) - 2048u))

#define	SPI_NDEVICES		2
#define	SPI_NETWORK_CHIP_SELECT	0
#define	SPI_DISPLAY_CHIP_SELECT	3

/*
 * The WINC1500's max SPI clock frequency is, apparently, 48 MHz.
 * I've tested it at 40 MHz on the production board and it works
 * fine. Speed-wise, performance doesn't improve past 30 MHz as I
 * suspect the CPU becomes the limiting factor (PIO mode still).
 *
 * In the interests of data integrity, let's be conservative and
 * stick to 15 MHz for now.
 *
 * The WizChip can pretty much handle anything we can throw at
 * it, so run it at 20 MHz.
 */
#ifdef NETWORK_DEV_WINC
#define	SPI_NETWORK_CLOCK	((F_CPU / 20000000) << SPI_CSR_SCBR_Pos)
#else
#define	SPI_NETWORK_CLOCK	((F_CPU / 20000000) << SPI_CSR_SCBR_Pos)
#endif

/* Display datasheet says minimum clock cycle time is 500nS. So 2 MHz */
#define	SPI_DISPLAY_CLOCK	((F_CPU / 2000000) << SPI_CSR_SCBR_Pos)

#ifdef SPI_STATS
#define	SPI_STATS_BUCKETS	12u
struct spi_stats {
	unsigned int st_polled;
	unsigned int st_polled_bytes;
	unsigned int st_irq;
	unsigned int st_irq_bytes;
	unsigned int st_sizes[SPI_STATS_BUCKETS];
	unsigned int st_sizes_bytes[SPI_STATS_BUCKETS];
	unsigned int st_loops[SPI_STATS_BUCKETS];
};
#endif

struct spi_sam_interface {
	struct spi_interface ssi_spi;
	int ssi_cs;
	unsigned int ssi_irq_threshold;
#ifdef SPI_STATS
#define	SPI_NSTATS	2
#define	SPI_STATS_READ	0
#define	SPI_STATS_WRITE	1
	struct spi_stats ssi_stats[SPI_NSTATS];
#endif
};

struct spi_state {
	Spi *ss_spi_regs;
	Pdc *ss_spi_pdi_regs;
	rtos_sema_t ss_sem;
	rtos_mutex_t ss_mutex;
	struct spi_sam_interface ss_spi[SPI_NDEVICES];
#define	SPI_DEV_NETWORK	0
#define	SPI_DEV_DISPLAY	1
};
static struct spi_state spi_state;

#ifdef SPI_STATS
static unsigned int *
spi_devs_update_stats(struct spi_sam_interface *ssi, uint16_t cnt, bool wr,
    bool rd, bool irq_mode)
{
	struct spi_stats *st;
	unsigned int pt, *rv = NULL;

	if ((pt = (unsigned int)fls((int)cnt)) > 0)
		pt -= 1;
	if (pt >= SPI_STATS_BUCKETS)
		pt = SPI_STATS_BUCKETS - 1;

	if (wr) {
		st = &ssi->ssi_stats[SPI_STATS_WRITE];
		if (irq_mode) {
			st->st_irq++;
			st->st_irq_bytes += cnt;
		} else {
			st->st_polled++;
			st->st_polled_bytes += cnt;
		}
		st->st_sizes[pt]++;
		st->st_sizes_bytes[pt] += cnt;
		rv = &st->st_loops[pt];
	}

	if (rd) {
		st = &ssi->ssi_stats[SPI_STATS_READ];
		if (irq_mode) {
			st->st_irq++;
			st->st_irq_bytes += cnt;
		} else {
			st->st_polled++;
			st->st_polled_bytes += cnt;
		}
		st->st_sizes[pt]++;
		st->st_sizes_bytes[pt] += cnt;
		rv = &st->st_loops[pt];
	}

	return rv;
}
#endif

static void
spi_complete_irq(void *arg)
{
	struct spi_state *ss = arg;
	Spi *spi = ss->ss_spi_regs;

	(void) spi->SPI_SR;

	spi->SPI_IDR = SPI_IDR_ENDTX;
	rtos_sema_give_isr(ss->ss_sem);
}

static int
spi_devs_do_xfer(struct spi_state *ss, const void *wr, void *rd, uint16_t cnt,
    bool irq_mode
#ifdef SPI_STATS
    , unsigned int *poll_loops
#endif
    )
{
	Spi *spi = ss->ss_spi_regs;
	Pdc *pdc = ss->ss_spi_pdi_regs;
	uint32_t ptcr, sr;
	int rv;

	/* If we're only reading, set up the dummy write pointer. */
	if (wr == NULL)
		wr = SPI_DEV_WR_SCRATCH;

	/* Ensure the Rx Data register is completely empty. */
	(void) spi->SPI_RDR;
	(void) spi->SPI_RDR;

	/* Configure the PDC. */
	pdc->PERIPH_TPR = (uintptr_t)wr;
	pdc->PERIPH_TCR = cnt;

	if (rd != NULL) {
		pdc->PERIPH_RPR = (uintptr_t)rd;
		pdc->PERIPH_RCR = cnt;
		ptcr = PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN;
	} else {
		pdc->PERIPH_RCR = 0;
		ptcr = PERIPH_PTCR_TXTEN;
	}

	__DSB();
	__ISB();

	if (irq_mode) {
		/* Enable the interrupt. */
		spi->SPI_IER = SPI_IER_ENDTX;
	}

	/* Fire off the transfer. */
	pdc->PERIPH_PTCR = ptcr;
	(void) pdc->PERIPH_PTCR;
	__DSB();
	__ISB();

	/* For interrupt mode, wait for the notification from the ISR. */
	if (irq_mode) {
		/* It is an error if the transfer takes longer than 500 mS. */
		rv = rtos_sema_take_timed(ss->ss_sem, 500) ? 0 : -1;

		spi->SPI_IDR = SPI_IDR_ENDTX;
	} else {
		rv = 0;
	}

	/*
	 * Wait for the transfer to complete. This is primarily for polled
	 * mode, but we also double-check in interrupt mode.
	 */
#ifdef SPI_STATS
	unsigned int loops = 0;
	do {
		loops++;
		sr = spi->SPI_SR & (SPI_SR_ENDTX | SPI_SR_TXEMPTY);
	} while (sr != (SPI_SR_ENDTX | SPI_SR_TXEMPTY));

	if (poll_loops)
		*poll_loops += loops - 1;
#else
	do {
		sr = spi->SPI_SR & (SPI_SR_ENDTX | SPI_SR_TXEMPTY);
	} while (sr != (SPI_SR_ENDTX | SPI_SR_TXEMPTY));
#endif

	/* Disable the DMA controller. */
	pdc->PERIPH_PTCR = PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS;

	return rv;
}

static void
spi_devs_chip_select(Spi *spi, int cs)
{

	spi->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_MR_PCS(~(1u << cs));
}

static int
spi_devs_spi_readwrite(const struct spi_interface *si, unsigned int iocnt,
    const struct spi_iovec *io)
{
	struct spi_state *ss = si->si_cookie;
	struct spi_sam_interface *ssi;
	int rv;

	/* The chip select and stats are here. */
	ssi = (struct spi_sam_interface *)(uintptr_t)si;

	/* Grab the mutex and task ID. */
	rtos_mutex_acquire(ss->ss_mutex);

#ifdef NETWORK_DEV_WIZCHIP
	/* WIZchip *needs* to see /CS deassert between xfers. */
	ss->ss_spi_regs->SPI_CR = SPI_CR_SPIEN;
#endif

	/* Select the relevant /CS line. */
	spi_devs_chip_select(ss->ss_spi_regs, ssi->ssi_cs);

	do {
		/* Determine if this xfer is polled or IRQ. */
		bool irq_mode = io->io_len >= ssi->ssi_irq_threshold;

#ifdef SPI_STATS
		/* Keep the stats up to date. */
		unsigned int *plp;
		plp = spi_devs_update_stats(ssi, io->io_len, io->io_wr != NULL,
		    io->io_rd != NULL, irq_mode);
#endif

		/* Perform the transfer. */
		rv = spi_devs_do_xfer(ss, io->io_wr, io->io_rd, io->io_len,
		    irq_mode
#ifdef SPI_STATS
		    , plp
#endif
		    );

		io++;
	} while (rv == 0 && --iocnt);

#ifdef NETWORK_DEV_WIZCHIP
	/* WIZchip *needs* to see /CS deassert between xfers. */
	ss->ss_spi_regs->SPI_CR = SPI_CR_SPIEN | SPI_CR_LASTXFER;
#endif

	/* Done with the mutex. */
	rtos_mutex_release(ss->ss_mutex);

	return rv;
}

#ifdef SPI_STATS
SHELL_CMD_DECL(spi, spi_cmd, "Tweak SPI settings/Dump SPI stats");

static void
spi_dump_stats(FILE *os, struct spi_state *ss, const char *name,
    struct spi_stats *st)
{
	static struct spi_stats local_st[SPI_NSTATS];

	rtos_mutex_acquire(ss->ss_mutex);
	memcpy(local_st, st, sizeof(local_st));
	memset(st, 0, sizeof(*st) * SPI_NSTATS);
	rtos_mutex_release(ss->ss_mutex);

	st = &local_st[0];
	for (unsigned int i = 0; i < SPI_NSTATS; i++, st++) {
		if ((st->st_polled + st->st_irq) == 0)
			continue;

		fprintf(os, "%s%s\tTotal calls %u, Polled %u, IRQ %u\n"
		    "\tTotal bytes %u, Polled %u, IRQ %u\n", name,
		    ((i & 1u) == 0) ? "Rd" : "Wr",
		    st->st_polled + st->st_irq, st->st_polled, st->st_irq,
		    st->st_polled_bytes + st->st_irq_bytes,
		    st->st_polled_bytes, st->st_irq_bytes);

		for (unsigned int j = 0; j < SPI_STATS_BUCKETS; j++) {
			if (st->st_sizes[j] == 0)
				continue;

			fprintf(os, "\t%d-%d bytes: Calls %u, Bytes %u, "
			    "Loops/call %u\n", 1 << j, (1 << (j + 1)) - 1,
			    st->st_sizes[j], st->st_sizes_bytes[j],
			    st->st_loops[j] / st->st_sizes[j]);
		}
	}
}

static void
spi_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct spi_state *ss = &spi_state;

	if (argc >= 2) {
		unsigned long threshold;

		threshold = strtoul(argv[0], NULL, 0);
		ss->ss_spi[SPI_DEV_NETWORK].ssi_irq_threshold =
		    (unsigned int)threshold;

		threshold = strtoul(argv[1], NULL, 0);
		ss->ss_spi[SPI_DEV_DISPLAY].ssi_irq_threshold =
		    (unsigned int)threshold;
	}

	fprintf(os, "Network IRQ Threshold: %u\n",
		ss->ss_spi[SPI_DEV_NETWORK].ssi_irq_threshold);
	spi_dump_stats(os, ss, "Net", ss->ss_spi[SPI_DEV_NETWORK].ssi_stats);

	fprintf(os, "Display IRQ Threshold: %u\n",
		ss->ss_spi[SPI_DEV_DISPLAY].ssi_irq_threshold);
	spi_dump_stats(os, ss, "Disp", ss->ss_spi[SPI_DEV_DISPLAY].ssi_stats);
}
#endif /* SPI_STATS */

static void
sam4s_display_dcrs(void *arg, bool state)
{

	(void) arg;

	if (state)
		HAL_GPIO_DISPLAY_DCRS_set();
	else
		HAL_GPIO_DISPLAY_DCRS_clr();
}

void
spi_devs_init(void)
{
	struct spi_state *ss = &spi_state;
	Spi *spi = SPI;

	ss->ss_spi_regs = SPI;
	ss->ss_spi_pdi_regs = PDC_SPI;

	/* Configure display DCRS pin */
	HAL_GPIO_DISPLAY_DCRS_clr();
	HAL_GPIO_DISPLAY_DCRS_out();

	/* Configure special pin functions for SPI controller */
	HAL_GPIO_NETWORK_CS_abcd(GPIO_NETWORK_CS_FUNCTION);
	HAL_GPIO_DISPLAY_CS_abcd(GPIO_DISPLAY_CS_FUNCTION);
	HAL_GPIO_SPI_MISO_pullup(1);
	HAL_GPIO_SPI_MISO_abcd(GPIO_SPI_MISO_FUNCTION);
	HAL_GPIO_SPI_MOSI_abcd(GPIO_SPI_MOSI_FUNCTION);
	HAL_GPIO_SPI_SCK_abcd(GPIO_SPI_SCK_FUNCTION);

	/* Configure SPI controller */
	spi->SPI_WPMR = SPI_WPMR_WPKEY_PASSWD;
	spi->SPI_CR = SPI_CR_SPIDIS;
	spi->SPI_CR = SPI_CR_SWRST;
	spi->SPI_WPMR = SPI_WPMR_WPKEY_PASSWD;

	spi->SPI_CSR[SPI_NETWORK_CHIP_SELECT] = SPI_CSR_CSAAT |
	    SPI_CSR_BITS_8_BIT | SPI_CSR_NCPHA |
	    SPI_NETWORK_CLOCK | (2u << SPI_CSR_DLYBS_Pos);

	spi->SPI_CSR[SPI_DISPLAY_CHIP_SELECT] = SPI_CSR_CSAAT |
	    SPI_CSR_BITS_8_BIT | SPI_CSR_NCPHA |
	    SPI_DISPLAY_CLOCK | (2u << SPI_CSR_DLYBS_Pos);

	spi->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
	spi->SPI_CR = SPI_CR_SPIEN;

	ss->ss_sem = rtos_sema_create_binary();
	assert(ss->ss_sem != NULL);
	ss->ss_mutex = rtos_mutex_create();
	assert(ss->ss_mutex != NULL);

	rtos_irq_register_named("SPI", SPI_IRQn, HW_IPL_NETWORK,
	    spi_complete_irq, ss);

	ss->ss_spi[SPI_DEV_NETWORK].ssi_spi.si_cookie = ss;
	ss->ss_spi[SPI_DEV_NETWORK].ssi_spi.si_read_write =
	    spi_devs_spi_readwrite;
	ss->ss_spi[SPI_DEV_NETWORK].ssi_cs = SPI_NETWORK_CHIP_SELECT;
	ss->ss_spi[SPI_DEV_NETWORK].ssi_irq_threshold =
	    SPI_NETWORK_IRQ_THRESHOLD;

	ss->ss_spi[SPI_DEV_DISPLAY].ssi_spi.si_cookie = ss;
	ss->ss_spi[SPI_DEV_DISPLAY].ssi_spi.si_read_write =
	    spi_devs_spi_readwrite;
	ss->ss_spi[SPI_DEV_DISPLAY].ssi_cs = SPI_DISPLAY_CHIP_SELECT;
	ss->ss_spi[SPI_DEV_DISPLAY].ssi_irq_threshold =
	    SPI_DISPLAY_IRQ_THRESHOLD;

#ifdef NETWORK_DEV_WINC
	/*
	 * Attach the WiFi driver
	 */
	sam4s_wifi_attach(&ss->ss_spi[SPI_DEV_NETWORK].ssi_spi);
#else
	/*
	 * Attach the WizChip Ethernet driver
	 */
	sam4s_wizchip_attach(&ss->ss_spi[SPI_DEV_NETWORK].ssi_spi);
#endif

	/*
	 * Attach the display.
	 */
	display_attach(&ss->ss_spi[SPI_DEV_DISPLAY].ssi_spi, sam4s_display_dcrs,
	    NULL);

#ifdef SPI_STATS
	SHELL_CMD_ADD(spi);
#endif
}
