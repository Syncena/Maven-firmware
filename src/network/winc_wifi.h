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

#ifndef WINC_WIFI_H
#define	WINC_WIFI_H

#include "common/include/nm_common.h"
#include "driver/include/m2m_periph.h"
#include "driver/include/m2m_wifi.h"

#include "winc_bsp.h"

#if defined(WAVR_BOARD) && (WAVR_BOARD == 1)
#define	WINC_OPT_GPIO	1
#elif defined(MAVEN_BOARD) && (MAVEN_BOARD >= 2)
/* No special options */
#else
#error "Unsupported board"
#endif

#undef	WINC_DEBUG_VERBOSE

#define	WINC_OPT_COMMAND
#define	WINC_OPT_OTA_UPDATE
#if (RELEASE_BUILD == 0)
/*
 * Don't enable this - there's a deadlock with rtos printf mutex
 * when non-zero.
 */
#undef	WINC_DEBUG
#endif

#ifndef CONFIG_USE_CONFIGDB
struct winc_wifi_glob_vars {
	uint8_t		gv_enc_type;
	char		gv_name[32];
	char		gv_ssid[64];
	char		gv_key[64];
	char		gv_prov_name[32];
	char		gv_prov_ssid[64];
};
#endif

struct winc_wifi_attach_args {
	void *waa_cookie;
	void (*waa_read_write)(void *, const void *, void *, uint16_t);
	void (*waa_control)(void *, uint8_t, void *);
#ifndef CONFIG_USE_CONFIGDB
	struct winc_wifi_glob_vars *waa_glob_vars;
#endif
};

#define	WINC_WIFI_SUSPEND	1
#define	WINC_WIFI_IRQ_CTL	2
#define	WINC_WIFI_RESET		3
#ifndef CONFIG_USE_CONFIGDB
#define	WINC_WIFI_GLOB_SAVE	4
#endif
#define	WINC_WIFI_PROVISIONING	5

/*
 * Entry point to attach WINC in native TCP/IP mode.
 */
extern void *winc_wifi_attach(const struct winc_wifi_attach_args *);

/*
 * Entry point to attach WINC in 'LWIP' mode.
 */
extern void *winc_wifi_lwip_attach(const struct winc_wifi_attach_args *);

/*
 * Hardware interrupt handler
 */
extern void winc_wifi_interrupt(void *);

extern void winc_wifi_provision(void *);

#define	WINC_WIFI_SEC_INVALID	0
#define	WINC_WIFI_SEC_OPEN	1
#define	WINC_WIFI_SEC_WPA_PSK	2
#define	WINC_WIFI_SEC_WEP	3
#define	WINC_WIFI_SEC_802_1X	4

#define	WINC_LED_ERR		0
#define	WINC_LED_WIFI		1
#define	WINC_LED_NETWORK	2

#define	WINC_LED_OFF		0
#define	WINC_LED_ON		1

extern void winc_wifi_led_init(void);

/*
 * These are invoked from within the WINC driver, so no worries
 * about re-entrancy.
 */
extern void winc_wifi_led_control(uint8_t, uint8_t);
#ifdef WINC_ADAFRUIT_BREAKOUT
extern void winc_wifi_ada_led_init(void);
extern void winc_wifi_ada_led_control(uint8_t, uint8_t);
#endif	/* WINC_ADAFRUIT_BREAKOUT */

extern int8_t winc_wifi_ioctl(void *, uint8_t, void *);
/*
 * WINC-specific ioctl codes
 */
#define	WINC_IOCTL_PROVISION	(NETWORK_IOCTL_DRIVER_BASE)
struct winc_prov_arg {
	void (*pa_cb)(void *);
	void *pa_cookie;
};

#define	WINC_IOCTL_INIT_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 1)
#define	WINC_IOCTL_WRITE_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 2)
#define	WINC_IOCTL_READ_GPIO	(NETWORK_IOCTL_DRIVER_BASE + 3)

struct winc_gpio {
	uint8_t wg_pin;
	uint8_t wg_value;
};

/* Possible values for 'wg_pin' */
#define	WINC_GPIO_PIN_3		0
#define	WINC_GPIO_PIN_4		1
#define	WINC_GPIO_PIN_5		2
#define	WINC_GPIO_PIN_6		3

/* Possible values for 'wg_value' for WINC_IOCTL_INIT_GPIO */
#define WINC_GPIO_DIRECTION_IN	(0u << 0)
#define WINC_GPIO_DIRECTION_OUT	(1u << 0)
#define WINC_GPIO_PULLUP_OFF	(0u << 1)
#define WINC_GPIO_PULLUP_ON	(1u << 1)

#ifdef WINC_OPT_OTA_UPDATE
#define	WINC_IOCTL_OTA_START	(NETWORK_IOCTL_DRIVER_BASE + 4)
#define	WINC_IOCTL_OTA_STATUS	(NETWORK_IOCTL_DRIVER_BASE + 5)
#define	WINC_IOCTL_OTA_ROLLBACK	(NETWORK_IOCTL_DRIVER_BASE + 6)

/*
 * Return values for the above ioctl calls.
 *
 * These status codes are overlaid with the tenuOtaUpdateStatus enum from
 * m2m_types.h so make sure they don't clash (except for OTA_STATUS_SUCSESS)
 * since we never pass that one back to the caller directly.
 */
#define	WINC_OTA_STATUS_IDLE			0
#define	WINC_OTA_STATUS_FAILED			1	/* Generic failure */
#define	WINC_OTA_STATUS_INVALID_ARG		2	/* Bad URL? */
#define	WINC_OTA_STATUS_INVALID_RB_IMAGE	3	/* Bad f/w image */
#define	WINC_OTA_STATUS_INVALID_FLASH_SIZE	4	/* Need >= 4Mb */
#define	WINC_OTA_STATUS_ALREADY_ENABLED		5	/* When switching? */
#define	WINC_OTA_STATUS_IN_PROGRESS		6	/* Update in progress */
#define	WINC_OTA_STATUS_VERIFY_FAILED		7	/* Bad Flash? */
#define	WINC_OTA_STATUS_CONNECTION_ERROR	8	/* Conectivity? */
#define	WINC_OTA_STATUS_SERVER_ERROR		9	/* ? */
#define	WINC_OTA_STATUS_SWITCHING		20	/* Switch in progress */
#define	WINC_OTA_STATUS_UPDATED			21	/* Update complete */
#define	WINC_OTA_STATUS_NOT_ASSOCIATED		22	/* Not associated */
#define	WINC_OTA_STATUS_BUSY			23	/* Already updating */

/* Likewise these need to match the OTA codes in m2m_types.h */
#define	WINC_OTA_ERR_WORKING_IMAGE_LOAD_FAIL	(-1)
#define	WINC_OTA_ERR_INVAILD_CONTROL_SEC	(-2)
#define	WINC_OTA_ERR_SWITCH_FAIL		(-3)
#define	WINC_OTA_ERR_START_UPDATE_FAIL		(-4)
#define	WINC_OTA_ERR_ROLLBACK_FAIL		(-5)
#define	WINC_OTA_ERR_INVAILD_FLASH_SIZE		(-6)
#define	WINC_OTA_ERR_INVAILD_ARG		(-7)
#define	WINC_OTA_ERR_BUSY			(-20)
#endif /* WINC_OPT_OTA_UPDATE */

#endif	/* WINC_WIFI_H */
