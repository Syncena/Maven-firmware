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

#ifndef UART_IMPL_H
#define UART_IMPL_H

#include "ringbuff.h"

struct uart_stats {
	uint32_t us_tx_bytes;
	uint32_t us_tx_bytes_polled;
	uint32_t us_tx_xoff;
	uint32_t us_tx_xon;
	uint32_t us_tx_breaks;
	uint32_t us_rx_bytes;
	uint32_t us_rx_pe;
	uint32_t us_rx_fe;
	uint32_t us_rx_noise;
	uint32_t us_rx_fifo_overruns;
	uint32_t us_rx_ring_overruns;
	uint32_t us_rx_xoff;
	uint32_t us_rx_xon;
};

enum uart_hwflow_state {
	UART_HWFLOW_ASSERT,		/* Assert RTS, or CTS asserted */
	UART_HWFLOW_DEASSERT,
};

struct uart_funcs {
	uint8_t (*uf_open)(void *, ringbuff_t from_uart, ringbuff_t to_uart);
	void (*uf_close)(void *);
	uint32_t (*uf_configure)(void *, uint16_t, uint32_t);
	void (*uf_get_stats)(void *, struct uart_stats *);
	void (*uf_polled_tx)(void *, uint8_t);
};

/* UART's baudrate. (bits 0-3) */
#define	UART_BAUDRATE_150	0x00u
#define	UART_BAUDRATE_300	0x01u
#define	UART_BAUDRATE_600	0x02u
#define	UART_BAUDRATE_1200	0x03u
#define	UART_BAUDRATE_2400	0x04u
#define	UART_BAUDRATE_4800	0x05u
#define	UART_BAUDRATE_9600	0x06u
#define	UART_BAUDRATE_19200	0x07u
#define	UART_BAUDRATE_38400	0x08u
#define	UART_BAUDRATE_57600	0x09u
#define	UART_BAUDRATE_115200	0x0au
#define	UART_BAUDRATE_230400	0x0bu
#define	UART_BAUDRATE_460800	0x0cu
#define	UART_BAUDRATE_921600	0x0du
#define	UART_BAUDRATE_1843200	0x0eu
#define	UART_BAUDRATE_CUSTOM	0x0fu
#define	UART_BAUDRATE_MASK	0x0fu

/* NRZ Mode: Number of stop bits (bit 4) */
#define	UART_STOPBITS_1		(0u << 4)
#define	UART_STOPBITS_2		(1u << 4)
#define	UART_STOPBITS_MASK	(1u << 4)

/* NRZ Mode: Parity (bits 5-6) */
#define	UART_PARITY_NONE	(0u << 5)
#define	UART_PARITY_ODD		(1u << 5)
#define	UART_PARITY_EVEN	(2u << 5)
#define	UART_PARITY_MASK	(3u << 5)

/* NRZ Mode: Character size (bits 7-8) */
#define	UART_CHARSIZE_5		(0u << 7)
#define	UART_CHARSIZE_6		(1u << 7)
#define	UART_CHARSIZE_7		(2u << 7)
#define	UART_CHARSIZE_8		(3u << 7)
#define	UART_CHARSIZE_MASK	(3u << 7)

/* NRZ Mode: Flow control configuration (bits 9-10) */
#define	UART_FLOW_NONE		(0u << 9)
#define	UART_FLOW_SOFTWARE	(1u << 9)
#define	UART_FLOW_SOFTWARE_ANY	(2u << 9)
#define	UART_FLOW_HARDWARE	(3u << 9)
#define	UART_FLOW_MASK		(3u << 9)

/* UART operating mode (bits 11-12) */
#define	UART_OPMODE_NRZ		(0u << 11)	/* Standard UART */
#define	UART_OPMODE_MANCHESTER	(1u << 11)

/* Bits 13-15 reserved */
/* Note: bits 14-15 used internally by uart code, so only 13 available */
#define	UART_CFG_MASK		0x1fffu

#ifdef WAVR_BOARD
#define	UART_DEFAULT_CFG	(UART_BAUDRATE_38400 | UART_STOPBITS_1 | \
				 UART_CHARSIZE_8 | UART_PARITY_NONE)
#else
#define	UART_DEFAULT_CFG	(UART_BAUDRATE_115200 | UART_STOPBITS_1 | \
				 UART_CHARSIZE_8 | UART_PARITY_NONE)
#endif

/* For use by driver only */
extern int uart_register(const char *, const struct uart_funcs *, void *);

/* Non-driver API */
extern void *uart_claim(int);
extern void uart_unclaim(void *);
extern uint8_t uart_open(void *, ringbuff_t from_uart, ringbuff_t to_uart);
extern void uart_close(void *);
extern void uart_polled_tx(void *, uint8_t);
extern uint32_t uart_configure(void *, uint16_t, uint32_t);
extern uint16_t uart_current_config(void *, uint32_t *);
extern void uart_config_strings(uint16_t, uint32_t, char *, char *);
extern uint16_t uart_baud_to_cfg(uint32_t);
extern uint32_t uart_cfg_to_baud(uint16_t, uint32_t);

#endif /* UART_IMPL_H */
