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
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "rtos.h"
#include "platform.h"
#include "timer.h"
#include "sam_gpio.h"
#include "sam_clocks.h"
#include "sam_sercom_spi.h"
#include "display.h"

HAL_GPIO_PIN(DISPLAY_DCRS, B, 10)
HAL_GPIO_PIN(DISPLAY_CS, B, 11)
HAL_GPIO_PIN(DISPLAY_SCK, B, 9)
HAL_GPIO_PIN(DISPLAY_MOSI, B, 8)
#define	DISPLAY_SPI_ID			ID_SERCOM4
#define	DISPLAY_SPI_DEV			(&SERCOM4->SPI)
#define	DISPLAY_SPI_DEV_INSTANCE	4
#define	DISPLAY_SPI_GCLK_CORE		SERCOM4_GCLK_ID_CORE
#define	DISPLAY_SPI_DOPO_PAD		0  /* DO PAD[0] SCK PAD[1] CS PAD[2] */
#define	DISPLAY_SPI_DIPO_PAD		3  /* Unused */
#define	DISPLAY_SPI_PMUX		'D'
#define	DISPLAY_CLOCK			2000000u

#ifdef PLATFORM_WILC_UART
/*
 * XXX: We'll need this if the WiFi module's UART is to be accessible.
 * If it's non-zero at startup, we'll bitbang SPI to the display.
 * This frees up SERCOM4 for WiFi module UART duties.
 */
#define	maven_display_use_bitbang	1
#else
#define	maven_display_use_bitbang	0
#endif

static void
maven_display_cs_control(bool assert_cs)
{

	if (assert_cs)
		HAL_GPIO_DISPLAY_CS_clr();
	else
		HAL_GPIO_DISPLAY_CS_set();
}

static int
maven_display_write_bitbang(const struct spi_interface *si, unsigned int iocnt,
    const struct spi_iovec *io)
{
	const uint8_t *wr;
	uint16_t len;
	uint8_t ch;

	(void) si;

	maven_display_cs_control(true);

	while (iocnt--) {
		wr = io->io_wr;
		len = io->io_len;
		io++;

		while (len--) {
			ch = *wr++;

			for (uint8_t mask = 0x80u; mask != 0; mask >>= 1) {
				/* Data changes while SCK is low. */
				if ((ch & mask) == 0)
					HAL_GPIO_DISPLAY_MOSI_clr();
				else
					HAL_GPIO_DISPLAY_MOSI_set();

				/* MOSI settling time. */
				timer_delay_cycles(25);

				/* Data is sampled on SCK rising edge. */
				HAL_GPIO_DISPLAY_SCK_set();

				/* MOSI hold time. */
				timer_delay_cycles(30);

				/* SCK low again. */
				HAL_GPIO_DISPLAY_SCK_clr();
			}
		}
	}

	timer_delay_cycles(20);
	maven_display_cs_control(false);

	return 0;
}

static void
maven_display_dcrs(void *arg, bool state)
{

	(void) arg;

	if (state)
		HAL_GPIO_DISPLAY_DCRS_set();
	else
		HAL_GPIO_DISPLAY_DCRS_clr();
}

void
maven_display_attach(unsigned int gen)
{
	const struct spi_interface *si;
	unsigned int clock;
	static const sam_sercom_spi_attach_args_t aa = {
		.aa_regs = DISPLAY_SPI_DEV,
		.aa_instance = DISPLAY_SPI_DEV_INSTANCE,
		.aa_dma_threshold = 0,      /* Always use DMA. */
		.aa_dma_irq_threshold = 16, /* DMA interrupts for xfers >= 16 */
		.aa_dma_name_tx = "Display-DMAC",
		.aa_dma_name_rx = NULL,
		.aa_dma_ipl = HW_IPL_DISPLAY,
		.aa_baud = DISPLAY_CLOCK,
		.aa_cs_func = maven_display_cs_control,
		.aa_lsb_first = false,
		.aa_sample_trailing = false,
		.aa_sck_idle_high = false,
		.aa_tx_only = true,
		.aa_sck_space = 0,
		.aa_dipo = DISPLAY_SPI_DIPO_PAD,
		.aa_dopo = DISPLAY_SPI_DOPO_PAD
	};

	/* Set up DCRS pin. */
	HAL_GPIO_DISPLAY_DCRS_clr();
	HAL_GPIO_DISPLAY_DCRS_out();

	/* We control CS manually. */
	HAL_GPIO_DISPLAY_CS_set();
	HAL_GPIO_DISPLAY_CS_out();

	if (maven_display_use_bitbang == 0) {
		/* Enable the APB clock for the selected SERCOM. */
		sam_periph_clock_enable(DISPLAY_SPI_ID);
		clock = sam_gclk_periph_chan_enable(DISPLAY_SPI_GCLK_CORE, gen);

		/* SPI signals are controlled by SERCOM. Note: MISO not needed. */
		HAL_GPIO_DISPLAY_MOSI_pmux(DISPLAY_SPI_PMUX);
		HAL_GPIO_DISPLAY_SCK_pmux(DISPLAY_SPI_PMUX);

		si = sam_sercom_spi_attach(&aa, clock);
	} else {
		/*
		 * We'll bitbang SPI to the display if SERCOM4 is being used
		 * to debug the WiFi module.
		 */
		static struct spi_interface spi;

		HAL_GPIO_DISPLAY_MOSI_clr();
		HAL_GPIO_DISPLAY_MOSI_out();
		HAL_GPIO_DISPLAY_SCK_clr();
		HAL_GPIO_DISPLAY_SCK_out();

		spi.si_cookie = NULL;
		spi.si_read_write = maven_display_write_bitbang;
		si = &spi;
	}

	assert(si != NULL);
	display_attach(si, maven_display_dcrs, NULL);
}
