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

#ifndef WIFI_DRIVER_H
#define WIFI_DRIVER_H

#ifdef NETWORK_DEV_WINC
#include "winc_wifi.h"
#else
#include "wilc_wifi.h"
#endif
#include "spi-interface.h"

extern void *wifi_driver_attach(const struct spi_interface *,
		void (*ctl)(void *, uint8_t, void *), void *arg);
extern int wifi_ioctl(void *, int, void *);
extern int wifi_driver_provision_start(void *);
extern void wifi_driver_interrupt(void *);
extern void wifi_driver_load_defaults(void);

#define	WIFI_CTL_PROVISIONING	0x10u

#ifndef CONFIG_USE_CONFIGDB
#ifdef NETWORK_DEV_WILC
#error "WILC uses configdb only."
#endif
/* WiFi global vars in eeprom */
#define WIFI_GLOBAL_VARS			\
	struct winc_wifi_glob_vars glob_wifi;
#endif

#endif /* WIFI_DRIVER_H */
