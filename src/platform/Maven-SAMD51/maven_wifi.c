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
#include <stdbool.h>
#include <stdlib.h>

#include "rtos.h"
#include "platform.h"
#include "hardware.h"
#include "sam_clocks.h"
#include "sam_sercom_spi.h"
#include "sam_eic.h"
#include "timer.h"
#include "wifi_driver.h"
#ifdef PLATFORM_WILC_UART
#include "uart.h"
#include "ringbuff.h"
#include "target_comms.h"
#include "port_daemon.h"
#endif

#ifdef NETWORK_DEV_WILC
#include "puff.h"

/* Compressed firmware image for WILC1000 device. */
static const uint8_t wilc_compressed_fw[] = {
#include "wilc1000_fw.h"
};
#endif	/* NETWORK_DEV_WILC */

#undef	WIFI_BITBANG_SPI

HAL_GPIO_PIN(WIFI_MISO, A, 12)
HAL_GPIO_PIN(WIFI_SCK, A, 13)
HAL_GPIO_PIN(WIFI_CS, A, 14)
HAL_GPIO_PIN(WIFI_MOSI, A, 15)
HAL_GPIO_PIN(WIFI_ENABLE, A, 17)
HAL_GPIO_PIN(WIFI_IRQ, A, 18)
HAL_GPIO_PIN(WIFI_RESET, A, 19)		/* Note: Shared with display */

#define	WIFI_IRQ_EXTINT			2
#define	WIFI_IRQ_PMUX			'A'

#ifndef WIFI_BITBANG_SPI
#define	WIFI_SPI_ID			ID_SERCOM2
#define	WIFI_SPI_DEV			(&SERCOM2->SPI)
#define	WIFI_SPI_DEV_INSTANCE		2
#define	WIFI_SPI_GCLK_CORE		SERCOM2_GCLK_ID_CORE
#define	WIFI_SPI_DOPO_PAD		2  /* DO PAD[3] SCK PAD[1] CS PAD[2] */
#define	WIFI_SPI_DIPO_PAD		0  /* DI PAD[0] */
#define	WIFI_SPI_PMUX			'C'

/*
 * The WiFi module's SPI clock is stable at 30 MHz, but fails to work at
 * 40 MHz (despite the docs stating that 48 MHz is the limit). This is
 * may well be PCB routing issues.
 *
 * Things work reliably at 25 MHz with drvstr(1) on the SPI pins.
 */
#define	WIFI_CLOCK			25000000u
#endif	/* WIFI_BITBANG_SPI */

struct wifi_state {
	void *ws_eic;
	void *ws_wifi;
};
static struct wifi_state wifi_state;

static void
maven_wifi_interrupt(void *arg, bool state)
{
	struct wifi_state *ws = arg;

	(void) state;

	if (ws->ws_wifi != NULL)
		wifi_driver_interrupt(ws->ws_wifi);
}

#ifdef NETWORK_DEV_WILC
static void
maven_wilc_get_ts(void *arg)
{
	struct wilc_fw_timestamp *ts = arg;

	strlcpy(ts->ts_date, WILC_FW_BUILD_DATE, sizeof(ts->ts_date));
	strlcpy(ts->ts_time, WILC_FW_BUILD_TIME, sizeof(ts->ts_time));
}
#endif	/* NETWORK_DEV_WILC */

static void
maven_wifi_control(void *cookie, uint8_t cmd, void *argp)
{
	struct wifi_state *ws = cookie;
	uint8_t enable;

	switch (cmd) {
#ifdef NETWORK_DEV_WINC
	case WINC_WIFI_IRQ_CTL:
#else
	case WILC_WIFI_IRQ_CTL:
#endif
		enable = *(uint8_t *)argp;
		sam_eic_irq_control(ws->ws_eic, enable != 0);
		break;

#ifdef NETWORK_DEV_WINC
	case WINC_WIFI_RESET:
#else
	case WILC_WIFI_RESET:
#endif
		HAL_GPIO_WIFI_RESET_clr();
		rtos_task_sleep(100);
		HAL_GPIO_WIFI_RESET_set();
		rtos_task_sleep(100);
		break;

#ifdef NETWORK_DEV_WILC
	case WILC_WIFI_GET_FW_TIMESTAMP:
		maven_wilc_get_ts(argp);
		break;
#endif /* NETWORK_DEV_WILC */

	default:
		break;
	}
}

static void
maven_wifi_cs_control(bool assert_cs)
{

	if (assert_cs)
		HAL_GPIO_WIFI_CS_clr();
	else
		HAL_GPIO_WIFI_CS_set();
}

#ifdef WIFI_BITBANG_SPI
static int
maven_wifi_spi_bitbang(const struct spi_interface *si, unsigned int iocnt,
    const struct spi_iovec *io)
{
	const uint8_t *wr;
	uint8_t *rd;
	uint16_t len;
	uint8_t wch, rch;

	(void) si;

	rch = 0;
	maven_wifi_cs_control(true);

	while (iocnt--) {
		wr = io->io_wr;
		rd = io->io_rd;
		len = io->io_len;
		io++;

		while (len--) {
			wch = (wr != NULL) ? *wr++ : 0;

			for (uint8_t mask = 0x80u; mask != 0; mask >>= 1) {
				/* Data changes while SCK is low. */
				if ((wch & mask) == 0)
					HAL_GPIO_WIFI_MOSI_clr();
				else
					HAL_GPIO_WIFI_MOSI_set();

				/* MOSI settling time. */
				timer_delay_cycles(1);

				/* Data is sampled on SCK rising edge. */
				HAL_GPIO_WIFI_SCK_set();

				/* Sample MISO. */
				rch = (rch << 1) | HAL_GPIO_WIFI_MISO_read();

				/* MOSI hold time. */
				timer_delay_cycles(2);

				/* SCK low again. */
				HAL_GPIO_WIFI_SCK_clr();
			}

			if (rd != NULL)
				*rd++ = rch;
		}
	}

	timer_delay_cycles(2);
	maven_wifi_cs_control(false);

	return 0;
}
#endif /* WIFI_BITBANG_SPI */

void
maven_wifi_provision_start(void)
{
	struct wifi_state *ws = &wifi_state;

	wifi_driver_provision_start(ws->ws_wifi);
}

#ifdef PLATFORM_WILC_UART
static struct wilc_uart_state {
	void *us_uart;
	port_daemon_instance_t us_port_daemon;
	ringbuff_t us_to_host;
	ringbuff_t us_from_host;
	ringbuff_t us_rb_from_wilc;
	ringbuff_t us_rb_to_wilc;
	uint8_t us_from_buff[2000];
	uint8_t us_to_buff[64];
} wilc_uart_state;

static void
wilc_uart_rb_callback(ringbuff_t rb, void *arg)
{
	struct wilc_uart_state *us = arg;

	(void) rb;

	if (us->us_port_daemon != NULL)
		port_daemon_attention(us->us_port_daemon);
}

static bool
wilc_uart_connect(void *arg, port_daemon_instance_t si,
    ringbuff_t to_host, ringbuff_t from_host, unsigned int unused)
{
	target_comms_arg_t ca = arg;
	struct wilc_uart_state *us = ca->ca_ops_arg;

	(void)unused;

	if (us->us_port_daemon != NULL)
		return false;

	us->us_to_host = to_host;
	us->us_from_host = from_host;

	ringbuff_consumer_init(from_host, wilc_uart_rb_callback, us);
	ringbuff_producer_init(to_host, wilc_uart_rb_callback, us);

	us->us_port_daemon = si;

	return true;
}

static void
wilc_uart_disconnect(void *arg)
{
	target_comms_arg_t ca = arg;
	struct wilc_uart_state *us = ca->ca_ops_arg;

	us->us_port_daemon = NULL;
}

static void
wilc_uart_worker(void *arg)
{
	target_comms_arg_t ca = arg;
	struct wilc_uart_state *us = ca->ca_ops_arg;

	target_comms_generic_worker(us->us_rb_from_wilc, us->us_to_host,
	    ca->ca_flags);

	target_comms_generic_worker(us->us_from_host, us->us_rb_to_wilc,
	    ca->ca_flags);
}

void platform_wilc_uart_attach(void);
void
platform_wilc_uart_attach(void)
{
	static const struct port_daemon_ops wilc_uart_ops = {
		.op_connect = wilc_uart_connect,
		.op_disconnect = wilc_uart_disconnect,
		.op_worker = wilc_uart_worker,
		.op_ctrl = NULL,
		.op_hangup = NULL
	};
	static struct target_comms_source cs;
	struct wilc_uart_state *us = &wilc_uart_state;

	if (us->us_uart == NULL)
		return;

	cs.cs_port = TARGET_COMMS_PORT_WIFI_UART;
	cs.cs_ops = &wilc_uart_ops;
	cs.cs_arg.ca_ops_arg = us;

	target_comms_register(&cs);
}

static void
maven_wifi_attach_uart(void)
{
	struct wilc_uart_state *us = &wilc_uart_state;
	int uart_id;
	uint8_t x;

	if ((uart_id = platform_uart_attach(PLATFORM_UART_WILC)) < 0)
		return;

	us->us_uart = uart_claim(uart_id);
	if (us->us_uart == NULL)
		return;

	us->us_rb_from_wilc = ringbuff_alloc(us->us_from_buff,
	    sizeof(us->us_from_buff));
	assert(us->us_rb_from_wilc != NULL);

	us->us_rb_to_wilc = ringbuff_alloc(us->us_to_buff,
	    sizeof(us->us_to_buff));
	assert(us->us_rb_to_wilc != NULL);

	ringbuff_consumer_init(us->us_rb_from_wilc, wilc_uart_rb_callback, us);

	x = uart_open(us->us_uart, us->us_rb_from_wilc, us->us_rb_to_wilc);
	assert(x != 0);
	(void) x;

#define	WILC_CFG	(UART_BAUDRATE_115200 | UART_STOPBITS_1 | \
			 UART_CHARSIZE_8 | UART_PARITY_NONE)
	uart_configure(us->us_uart, WILC_CFG, 0);
}
#endif /* PLATFORM_WILC_UART */

void
maven_wifi_attach(unsigned int gen)
{
	struct wifi_state *ws = &wifi_state;
	const struct spi_interface *si;
#ifndef WIFI_BITBANG_SPI
	unsigned int clock;
	static const sam_sercom_spi_attach_args_t aa = {
		.aa_regs = WIFI_SPI_DEV,
		.aa_instance = WIFI_SPI_DEV_INSTANCE,
		.aa_dma_threshold = 128,/* Use DMA for transfers >= than this */
		.aa_dma_irq_threshold = 128,/* Always use interrupts with DMA */
		.aa_dma_ipl = HW_IPL_NETWORK,
		.aa_dma_name_tx = NULL,
		.aa_dma_name_rx = "WiFi-DMAC",
		.aa_baud = WIFI_CLOCK,
		.aa_cs_func = maven_wifi_cs_control,
		.aa_lsb_first = false,
		.aa_sample_trailing = false,
		.aa_sck_idle_high = false,
		.aa_tx_only = false,
		.aa_sck_space = 0,
		.aa_dipo = WIFI_SPI_DIPO_PAD,
		.aa_dopo = WIFI_SPI_DOPO_PAD
	};
#endif /* WIFI_BITBANG_SPI */
	static const sam_eic_irq_t ei = {
		.ei_name = "WiFi",
		.ei_ipl = HW_IPL_NETWORK,
		.ei_extint = WIFI_IRQ_EXTINT,
#ifdef NM_EDGE_INTERRUPT
		.ei_type = SAM_EIC_IRQ_TYPE_EDGE_FALLING,
#elif defined(NM_LEVEL_INTERRUPT)
		.ei_type = SAM_EIC_IRQ_TYPE_LEVEL_LOW,
#else
#error	"Neither NM_EDGE_INTERRUPT nor NM_LEVEL_INTERRUPT defined."
#endif
		.ei_async = false,
		.ei_filter = false,
		.ei_debounce = false
	};

#ifdef PLATFORM_WILC_UART
	maven_wifi_attach_uart();
#endif

#ifndef WIFI_BITBANG_SPI
	/* Enable the APB clock for the selected SERCOM. */
	sam_periph_clock_enable(WIFI_SPI_ID);
	clock = sam_gclk_periph_chan_enable(WIFI_SPI_GCLK_CORE, gen);
#else
	(void) gen;
#endif

	/*
	 * Configure GPIOs for WiFi reset and enable.
	 *
	 * Note that timing is important here. The documentation states
	 * that there must be at least 5mS between the rising edges of
	 * CHIP_EN and RESET, with the former rising first.
	 *
	 * We'll be a little more accomodating.
	 */
	/* Keep RESET asserted until WINC_WIFI_RESET is invoked. */
	HAL_GPIO_WIFI_RESET_clr();
	HAL_GPIO_WIFI_RESET_out();
	HAL_GPIO_WIFI_ENABLE_clr();
	HAL_GPIO_WIFI_ENABLE_out();
	rtos_task_sleep(50);

	/* Enable the device. */
	HAL_GPIO_WIFI_ENABLE_set();
	rtos_task_sleep(50);

	HAL_GPIO_WIFI_MISO_pullup(1);
#ifndef WIFI_BITBANG_SPI
	/* SPI signals are controlled by SERCOM. */
	HAL_GPIO_WIFI_MISO_pmux(WIFI_SPI_PMUX);
	HAL_GPIO_WIFI_MOSI_pmux(WIFI_SPI_PMUX);
	HAL_GPIO_WIFI_SCK_pmux(WIFI_SPI_PMUX);
#else
	/* SPI signals are controlled by software. */
	HAL_GPIO_WIFI_MISO_inen(1);
	HAL_GPIO_WIFI_MOSI_out();
	HAL_GPIO_WIFI_SCK_out();
#endif
	HAL_GPIO_WIFI_CS_set();
	HAL_GPIO_WIFI_CS_out();

	HAL_GPIO_WIFI_MOSI_drvstr(1);
	HAL_GPIO_WIFI_SCK_drvstr(1);
	HAL_GPIO_WIFI_CS_drvstr(1);

	/* Hook the external interrupt on EIC/EXTINT[2], external pullup. */
	HAL_GPIO_WIFI_IRQ_pmux(WIFI_IRQ_PMUX);
	ws->ws_eic = sam_eic_irq_register(&ei, maven_wifi_interrupt, ws);
	assert(ws->ws_eic != NULL);

#ifndef WIFI_BITBANG_SPI
	/* Attach the SERCOM SPI driver. */
	si = sam_sercom_spi_attach(&aa, clock);
	assert(si != NULL);
#else
	static struct spi_interface spi;
	spi.si_cookie = NULL;
	spi.si_read_write = maven_wifi_spi_bitbang;
	si = &spi;
#endif

	/* Attach the WiFi controller */
	ws->ws_wifi = wifi_driver_attach(si, maven_wifi_control, ws);
	assert(ws->ws_wifi != NULL);

	/* Enable WiFi interrupt. */
	sam_eic_irq_control(ws->ws_eic, true);
}

#ifdef NETWORK_DEV_WILC
int platform_wilc_fw_decompress(int (*cb)(void *, const unsigned char *,
			    unsigned long), void *arg, unsigned long chunklen);

static void *
calloc_wrapper(unsigned int items, unsigned int itemsize)
{

	return zone_calloc(items, itemsize);
}

static void
free_wrapper(void *p)
{

	zone_free(p);
}

int
platform_wilc_fw_decompress(int (*cb)(void *, const unsigned char *,
    unsigned long), void *arg, unsigned long chunklen)
{
	struct puff_args pa;

	pa.pa_calloc = calloc_wrapper;
	pa.pa_free = free_wrapper;
	pa.pa_chunklen = chunklen;
	pa.pa_cb = cb;
	pa.pa_cb_arg = arg;
	pa.pa_source = wilc_compressed_fw;
	pa.pa_sourcelen = sizeof(wilc_compressed_fw);

	return puff_cb(&pa);
}
#endif /* NETWORK_DEV_WILC */
