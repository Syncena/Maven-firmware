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

#ifndef SAM_SERCOM_SPI_H
#define SAM_SERCOM_SPI_H

#include <stdbool.h>
#include "spi-interface.h"

typedef struct {
	/* Pointer to the Sercom registers. */
	SercomSpi *aa_regs;

	/* Sercom instance number (0-7) */
	unsigned int aa_instance;

	/* Transfer lengths >= this number of bytes will use DMA. */
	uint16_t aa_dma_threshold;

	/* If non-zero, DMA transfer lengths >= this will use IRQ completion. */
	uint16_t aa_dma_irq_threshold;

	/* Priority of DMA completion interrupts. */
	uint32_t aa_dma_ipl;

	/* Names for DMA completion interrupts. */
	const char *aa_dma_name_tx;
	const char *aa_dma_name_rx;

	/* Desired SPI 'SCK' speed. */
	unsigned int aa_baud;

	/* Function to control /SS in software. NULL for hardware control. */
	void (*aa_cs_func)(bool);

	/* Bit order. True for LSB first, else MSB first (normal). */
	bool aa_lsb_first;

	/* If true, SPI data is sampled on SCK trailing edge. Else leading. */
	bool aa_sample_trailing;

	/* If true, SCK pin will be high when idle. Otherwise, low. */
	bool aa_sck_idle_high;

	/* If true, this device ignores Rx data on MISO. */
	bool aa_tx_only;

	/* Number of clocks between characters. */
	uint8_t aa_sck_space;

	/* Input pad configuration. */
	uint8_t aa_dipo;

	/* Output pad configuration. */
	uint8_t aa_dopo;
} sam_sercom_spi_attach_args_t;

extern const struct spi_interface *sam_sercom_spi_attach(
		const sam_sercom_spi_attach_args_t *, unsigned int clock);

#endif /* SAM_SERCOM_SPI_H */
