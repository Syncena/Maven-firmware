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

#ifndef WILC_WIFI_H
#define	WILC_WIFI_H

#include "wilc_bsp.h"

#if defined(MAVEN_BOARD) && (MAVEN_BOARD >= 2)
/* No special options */
#else
#error "Not supported on this board."
#endif

#undef	WILC_DEBUG_VERBOSE

#define	WILC_OPT_COMMAND
#if (RELEASE_BUILD == 0)
/*
 * Don't enable this - there's a deadlock with rtos printf mutex
 * when non-zero.
 */
#undef	WILC_DEBUG
#endif

struct wilc_wifi_attach_args {
	void *waa_cookie;
	void (*waa_read_write)(void *, const void *, void *, uint16_t);
	void (*waa_control)(void *, uint8_t, void *);
};

#define	WILC_WIFI_IRQ_CTL		2
#define	WILC_WIFI_RESET			3
#define	WILC_WIFI_PROVISIONING		4
#define	WILC_WIFI_GET_FW_TIMESTAMP	5
struct wilc_fw_timestamp {
	char ts_date[16];
	char ts_time[16];
};

/*
 * Entry point to attach WILC in 'LWIP' mode.
 */
extern void *wilc_wifi_lwip_attach(const struct wilc_wifi_attach_args *);

/*
 * Load default configuration.
 */
extern void wilc_wifi_load_defaults(void);

/*
 * Hardware interrupt handler
 */
extern void wilc_wifi_interrupt(void *);

extern void wilc_wifi_provision(void *);

#define	WILC_LED_ERR		0
#define	WILC_LED_WIFI		1
#define	WILC_LED_NETWORK	2
#define	WILC_LED_OFF		0
#define	WILC_LED_ON		1
extern void wilc_wifi_led_init(void);
extern void wilc_wifi_led_control(uint8_t, uint8_t);

extern int8_t wilc_wifi_ioctl(void *, uint8_t, void *);
/*
 * WILC-specific ioctl codes
 */
#define	WILC_IOCTL_PROVISION	(NETWORK_IOCTL_DRIVER_BASE)
struct wilc_prov_arg {
	void (*pa_cb)(void *);
	void *pa_cookie;
};

#define	WILC_IOCTL_INIT_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 1)
#define	WILC_IOCTL_WRITE_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 2)
#define	WILC_IOCTL_READ_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 3)

struct wilc_gpio {
	uint8_t wg_pin;
	uint8_t wg_value;
};

/* Possible values for 'wg_pin' */
#define	WILC_GPIO_PIN_3		0
#define	WILC_GPIO_PIN_4		1
#define	WILC_GPIO_PIN_5		2
#define	WILC_GPIO_PIN_6		3

/* Possible values for 'wg_value' for WILC_IOCTL_INIT_GPIO */
#define WILC_GPIO_DIRECTION_IN	(0u << 0)
#define WILC_GPIO_DIRECTION_OUT	(1u << 0)
#define WILC_GPIO_PULLUP_OFF	(0u << 1)
#define WILC_GPIO_PULLUP_ON	(1u << 1)

#endif	/* WILC_WIFI_H */
