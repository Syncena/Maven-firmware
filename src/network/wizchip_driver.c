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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "spi-interface.h"
#include "wizchip_driver.h"
#include "wiz5x00.h"

struct wiz_state {
	void *ws_driver;
	void (*ws_hw_ctl)(void *, uint8_t, void *);
	void *ws_hw_cookie;
	const struct spi_interface *ws_spi;
};
static struct wiz_state wiz_state;

void
wizchip_interrupt(void *arg)
{
	struct wiz_state *ws = arg;

	wiz5x00_hw_interrupt(ws->ws_driver);
}

static void
wiz_read_write(void *cookie, uint8_t nios, const struct spi_iovec *iov)
{
	struct wiz_state *ws = cookie;

	ws->ws_spi->si_read_write(ws->ws_spi, (unsigned int)nios, iov);
}

static void
wiz_control(void *cookie, uint8_t cmd, void *argp)
{
	struct wiz_state *ws = cookie;

	(ws->ws_hw_ctl)(ws->ws_hw_cookie, cmd, argp);
}

void *
wizchip_driver_attach(const struct spi_interface *spi, const uint8_t *mac,
    void (*ctl)(void *, uint8_t, void *), void *arg)
{
	struct wiz_state *ws = &wiz_state;
	struct wiz5x00_attach_args waa;

	ws->ws_spi = spi;
	ws->ws_hw_ctl = ctl;
	ws->ws_hw_cookie = arg;

	memcpy(waa.waa_mac, mac, sizeof(waa.waa_mac));

#ifdef WIZ_5200
	waa.waa_chip = WIZ_CHIP_5200;
#elif defined(WIZ_5500)
	waa.waa_chip = WIZ_CHIP_5500;
#else
#error "Wiznet chip not defined"
#endif

	waa.waa_cookie = ws;
	waa.waa_read_write = wiz_read_write;
	waa.waa_control = wiz_control;

	/*
	 * Attach the WizChip controller
	 */
#ifndef NETWORK_LWIP
	ws->ws_driver = wiz5x00_attach(&waa);
#else
	ws->ws_driver = wiz5x00_lwip_attach(&waa);
#endif

	if (ws->ws_driver == NULL)
		return NULL;

	return ws;
}
