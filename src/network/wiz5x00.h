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

#ifndef WIZ5x00_H
#define WIZ5x00_H

#include "spi-interface.h"
#include "network.h"
#ifdef NETWORK_LWIP
#include "network_lwip.h"
#endif

#if !defined(WIZ_5200) && !defined(WIZ_5500)
#error "One or both of WIZ_5200 and WIZ_5500 must be defined"
#endif

#define WIZ_CHIP_5200	0
#define WIZ_CHIP_5500	1

struct wiz5x00_attach_args {
	uint8_t waa_chip;
	uint8_t waa_mac[6];
	void *waa_cookie;
	void (*waa_read_write)(void *, uint8_t, const struct spi_iovec *);
	void (*waa_control)(void *, uint8_t, void *);
};
#define	WIZ5X00_CTL_SCHEDULE	0
#define	WIZ5X00_CTL_IRQ_CTL	1
#define	WIZ5X00_CTL_IRQ_STATUS	2
#define	WIZ5X00_CTL_RESET	3

extern void *wiz5x00_attach(const struct wiz5x00_attach_args *);

#ifdef NETWORK_LWIP
extern void *wiz5x00_lwip_attach(const struct wiz5x00_attach_args *);
#endif

extern void wiz5x00_hw_interrupt(void *);

#endif /* WIZ5x00_H */
