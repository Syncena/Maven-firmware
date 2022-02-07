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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "rtos.h"
#include "wifi_driver.h"
#include "spi-interface.h"
#include "timer.h"
#include "hardware.h"
#include "platform.h"
#ifdef NETWORK_DEV_WINC
#include "winc_wifi.h"
#else
#include "wilc_wifi.h"
#endif
#include "network.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "display.h"
#include "shell.h"
#include "serial_number.h"
#include "product.h"

struct wifi_state {
	void *ws_wifi_driver;
	const struct spi_interface *ws_spi;
	void (*ws_hw_control)(void *cookie, uint8_t cmd, void *argp);
	void *ws_hw_cookie;
};
static struct wifi_state wifi_state;

#if defined(NETWORK_DEV_WINC) && defined(WINC_OPT_OTA_UPDATE)
SHELL_CMD_DECL(wincota, winc_ota_cmd, "OTA update of WINC1500 firmare.");
#endif

void
wifi_driver_interrupt(void *arg)
{
	struct wifi_state *ws = arg;

#ifdef NETWORK_DEV_WINC
	winc_wifi_interrupt(ws->ws_wifi_driver);
#else
	wilc_wifi_interrupt(ws->ws_wifi_driver);
#endif
}

static void
wifi_control(void *cookie, uint8_t cmd, void *argp)
{
	struct wifi_state *ws = cookie;

	switch (cmd) {
#ifdef NETWORK_DEV_WINC
	case WINC_WIFI_SUSPEND:
	{
		uint32_t ms = *(uint32_t *)argp;
		rtos_task_sleep(ms);
		break;
	}

	case WINC_WIFI_PROVISIONING:
		display_provision_details();
		break;
#endif

#ifdef NETWORK_DEV_WILC
	case WILC_WIFI_PROVISIONING:
		display_provision_details();
		break;
#endif

#ifndef CONFIG_USE_CONFIGDB
	case WINC_WIFI_GLOB_SAVE:
		glob_save();
		break;
#endif

	default:
		ws->ws_hw_control(ws->ws_hw_cookie, cmd, argp);
		break;
	}
}

static void
wifi_read_write(void *cookie, const void *wrp, void *rdp, uint16_t len)
{
	struct wifi_state *ws = cookie;
	struct spi_iovec io;

	io.io_wr = wrp;
	io.io_rd = rdp;
	io.io_len = len;

	(void) ws->ws_spi->si_read_write(ws->ws_spi, 1, &io);
}

static void
wifi_driver_provisioned(void *arg)
{
	struct wifi_state *ws = arg;

#ifdef NETWORK_DEV_WINC
	winc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_RESUME, NULL);
#else
	wilc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_RESUME, NULL);
#endif
}

static void
wifi_driver_suspended(void *arg)
{
	struct wifi_state *ws = arg;
#ifdef NETWORK_DEV_WINC
	struct winc_prov_arg pa;
#else
	struct wilc_prov_arg pa;
#endif

	pa.pa_cb = wifi_driver_provisioned;
	pa.pa_cookie = ws;

#ifdef NETWORK_DEV_WINC
	winc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_WIFI_PROVISION, &pa);
#else
	wilc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_WIFI_PROVISION, &pa);
#endif
}

int
wifi_driver_provision_start(void *arg)
{
	struct wifi_state *ws = arg;
	struct network_suspend_args sa;
	int8_t rv;

	sa.sa_cb = wifi_driver_suspended;
	sa.sa_cookie = ws;

#ifdef NETWORK_DEV_WINC
	rv = winc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_SUSPEND, &sa);
#else
	rv = wilc_wifi_ioctl(ws->ws_wifi_driver, NETWORK_IOCTL_SUSPEND, &sa);
#endif

	return (int)rv;
}

#if defined(NETWORK_DEV_WINC) && defined(WINC_OPT_OTA_UPDATE)
static const char * const wincota_cmd_usage =
"Usage:\n"
"wincota <firmware-url>\n"
"\tInitiates an Over The Air update of firmware in the WINC1500 WiFi\n"
"\tcontroller. The specified URL must be in the form:\n"
"\t\thttp://server-name/firmware-file\n"
"\twhere \"server-name\" will usually be winc-fw.maverick-embedded.co.uk\n"
"\tand \"firmware-file\" specifies the location and name of the file\n"
"\ton the server. The purpose of this command is to enable in-the-field\n"
"\tpatching of the WiFi controller in cases where serious WiFi security-\n"
"\trelated updates are needed, such as the 2017 WPA2 \"Krack\" incident.\n"
"\t(Don't worry - the patch for that one has already been applied!)\n"
"\tWARNING: You are very strongly cautioned against uploading a firmware\n"
"\timage which has not been tested.\n"
"\tDoing so could render your device completely unusable if the " PRODUCT_NAME
"\n"
"\tdriver in your device is incompatible with the new firmware.\n"
"\tRecovering from this situation could entail desoldering and replacing\n"
"\tthe WiFi module!\n";

static void
winc_ota_cmd(FILE *fp, uint8_t argc, const char * const *argv)
{
	struct wifi_state *ws = &wifi_state;
	const char *msg;
	char tmpbuf[20];
	int8_t rv, i;

	if (argc == 1 && strcasecmp(argv[0], "rb") == 0) {
		fprintf(fp, "Initiating rollback\n");
		rv = winc_wifi_ioctl(ws->ws_wifi_driver,
		    WINC_IOCTL_OTA_ROLLBACK, NULL);
	} else
	if (argc == 1 && strncasecmp(argv[0], "http", 4) == 0) {
		fprintf(fp, "Initiating OTA update from %s\n", argv[0]);
		rv = winc_wifi_ioctl(ws->ws_wifi_driver,
		    WINC_IOCTL_OTA_START, (void *)(uintptr_t)argv[0]);
	} else
	if (argc == 0) {
		rv = winc_wifi_ioctl(ws->ws_wifi_driver,
		    WINC_IOCTL_OTA_STATUS, NULL);
	}else {
		fprintf(fp, "%s", wincota_cmd_usage);
		return;
	}

	if (rv == WINC_OTA_STATUS_IN_PROGRESS ||
	    rv == WINC_OTA_STATUS_SWITCHING) {
		fprintf(fp, "Please wait ...");
		fflush(fp);
	}

	i = -1;
	while (rv == WINC_OTA_STATUS_IN_PROGRESS ||
	    rv == WINC_OTA_STATUS_SWITCHING) {
		if (i++ == 10) {
			fputc('.', fp);
			fflush(fp);
			i = 0;
		}
		rtos_task_sleep(100);
		rv = winc_wifi_ioctl(ws->ws_wifi_driver,
		    WINC_IOCTL_OTA_STATUS, NULL);
	}

	if (i > 0)
		fputc('\n', fp);

	switch (rv) {
	case WINC_OTA_STATUS_IDLE:
		msg = "Idle.";
		break;
	case WINC_OTA_STATUS_FAILED:
		msg = "Failed.";
		break;
	case WINC_OTA_STATUS_INVALID_ARG:
	case WINC_OTA_ERR_INVAILD_ARG:
		msg = "Invalid arg. Bad URL?";
		break;
	case WINC_OTA_STATUS_INVALID_RB_IMAGE:
		msg = "Invalid firmware image.";
		break;
	case WINC_OTA_STATUS_INVALID_FLASH_SIZE:
	case WINC_OTA_ERR_INVAILD_FLASH_SIZE:
		msg = "H/W doesn't support OTA.";
		break;
	case WINC_OTA_STATUS_ALREADY_ENABLED:
		msg = "Duplicate firmware switch.";
		break;
	case WINC_OTA_STATUS_IN_PROGRESS:
		msg = "This can't happen.";
		break;
	case WINC_OTA_STATUS_VERIFY_FAILED:
		msg = "Flash verificvation failure.";
		break;
	case WINC_OTA_STATUS_CONNECTION_ERROR:
		msg = "Connection error.";
		break;
	case WINC_OTA_STATUS_SERVER_ERROR:
		msg = "Server error.";
		break;
	case WINC_OTA_STATUS_SWITCHING:
		msg = "This can't happen either.";
		break;
	case WINC_OTA_STATUS_UPDATED:
		msg = "OTA firmware update complete. Please reboot now.";
		break;
	case WINC_OTA_STATUS_NOT_ASSOCIATED:
		msg = "No WiFi connection.";
		break;
	case WINC_OTA_STATUS_BUSY:
		msg = "OTA update already in progress.";
		break;
	case WINC_OTA_ERR_WORKING_IMAGE_LOAD_FAIL:
		msg = "Working image load failed.";
		break;
	case WINC_OTA_ERR_INVAILD_CONTROL_SEC:
		msg = "Invalid control sec.";
		break;
	case WINC_OTA_ERR_SWITCH_FAIL:
		msg = "Image switch-over failure.";
		break;
	case WINC_OTA_ERR_START_UPDATE_FAIL:
		msg = "Start update failure.";
		break;
	case WINC_OTA_ERR_ROLLBACK_FAIL:
		msg = "Rollback failure.";
		break;
	case WINC_OTA_ERR_BUSY:
		msg = "OTA update already in progress.";
		break;
	default:
		snprintf(tmpbuf, sizeof(tmpbuf), "Unknown err: %d", rv);
		msg = tmpbuf;
		break;
	}

	fprintf(fp, "WINC1500 OTA Status: %s\n", msg);
}
#endif /* defined(NETWORK_DEV_WINC) && defined(WINC_OPT_OTA_UPDATE) */

int
wifi_ioctl(void *cookie, int cmd, void *cmdarg)
{
	struct wifi_state *ws = cookie;

#ifdef NETWORK_DEV_WINC
	return (int)winc_wifi_ioctl(ws->ws_wifi_driver, cmd, cmdarg);
#else
	return (int)wilc_wifi_ioctl(ws->ws_wifi_driver, cmd, cmdarg);
#endif
}

void *
wifi_driver_attach(const struct spi_interface *spi,
    void (*ctl)(void *, uint8_t, void *), void *arg)
{
	struct wifi_state *ws = &wifi_state;
#ifdef NETWORK_DEV_WINC
	struct winc_wifi_attach_args waa;
#else
	struct wilc_wifi_attach_args waa;
#endif

	ws->ws_spi = spi;
	ws->ws_hw_control = ctl;
	ws->ws_hw_cookie = arg;

	waa.waa_cookie = ws;
	waa.waa_read_write = wifi_read_write;
	waa.waa_control = wifi_control;
#ifndef CONFIG_USE_CONFIGDB
	waa.waa_glob_vars = &glob.glob_wifi;
#endif

	/*
	 * Attach the WiFi controller
	 */
#ifdef NETWORK_DEV_WINC
#ifndef NETWORK_LWIP
	ws->ws_wifi_driver = winc_wifi_attach(&waa);
#else
	ws->ws_wifi_driver = winc_wifi_lwip_attach(&waa);
#endif
#else /* NETWORK_DEV_WINC */
	ws->ws_wifi_driver = wilc_wifi_lwip_attach(&waa);
#endif /* NETWORK_DEV_WINC */

	if (ws->ws_wifi_driver == NULL)
		return NULL;

#if defined(NETWORK_DEV_WINC) && defined(WINC_OPT_OTA_UPDATE)
	SHELL_CMD_ADD(wincota);
#endif

	return ws;
}

void
wifi_driver_load_defaults(void)
{
	const char *prov_ssid = "Maverick Embedded";

#ifndef CONFIG_USE_CONFIGDB
	strcpy(glob.glob_wifi.gv_name, PRODUCT_NAME);
	strcpy(glob.glob_wifi.gv_prov_ssid, prov_ssid);
	strcpy(glob.glob_wifi.gv_prov_name, "192.168.0.1");	/* Not used */

	memset(glob.glob_wifi.gv_ssid, 0xff, sizeof(glob.glob_wifi.gv_ssid));
	memset(glob.glob_wifi.gv_key, 0xff, sizeof(glob.glob_wifi.gv_key));
	glob.glob_wifi.gv_enc_type = WINC_WIFI_SEC_INVALID;
#else
	const serial_number_t *sn;

	sn = serial_number_fetch();
	assert(sn != NULL);

	cf_set_string(CF_KEY_STRING_WIFI_PROV_SSID, prov_ssid);
	cf_set_string(CF_KEY_STRING_WIFI_PROV_PASS, sn->cstr);
	cf_del_uint32(CF_KEY_UINT32_WIFI_PROV_IP);
	cf_del_string(CF_KEY_STRING_WIFI_SSID);
	cf_del_string(CF_KEY_STRING_WIFI_KEY);
	cf_del_uint8(CF_KEY_UINT8_WIFI_ENC_TYPE);
#endif
#ifdef NETWORK_DEV_WILC
	wilc_wifi_load_defaults();
#endif
}

#ifdef NETWORK_DEV_WINC
/*
 * Invoked from within the WINC driver lock perimeter, so we can
 * call API functions directly.
 */
void
winc_wifi_led_init(void)
{
#if defined(ACTIVITY_LED_NET) || defined(ACTIVITY_LED_WIFI) || \
    defined(ACTIVITY_LED_ERR) || defined(WINC_PROVISION_SW)
	m2m_periph_init(NULL);

#ifdef ACTIVITY_LED_NET
	m2m_periph_gpio_set_dir(ACTIVITY_LED_NET, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_NET, 1);
#endif
#ifdef ACTIVITY_LED_WIFI
	m2m_periph_gpio_set_dir(ACTIVITY_LED_WIFI, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_WIFI, 1);
#endif
#ifdef ACTIVITY_LED_ERR
	m2m_periph_gpio_set_dir(ACTIVITY_LED_ERR, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_ERR, 1);
#endif
#ifdef WINC_PROVISION_SW
	m2m_periph_gpio_set_dir(WINC_PROVISION_SW, 0);
#endif
#endif
}

void
winc_wifi_led_control(uint8_t which, uint8_t state)
{

#if defined(ACTIVITY_LED_NET) || defined(ACTIVITY_LED_WIFI) || \
    defined(ACTIVITY_LED_ERR)
	switch (which) {
#ifdef ACTIVITY_LED_NET
	case WINC_LED_NETWORK:
		m2m_periph_gpio_set_val(ACTIVITY_LED_NET, !state);
		break;
#endif
#ifdef ACTIVITY_LED_WIFI
	case WINC_LED_WIFI:
		m2m_periph_gpio_set_val(ACTIVITY_LED_WIFI, !state);
		break;
#endif
#ifdef ACTIVITY_LED_ERR
	case WINC_LED_ERR:
		m2m_periph_gpio_set_val(ACTIVITY_LED_ERR, !state);
		break;
#endif
	default:
		break;
	}
#else
	(void) which;
	(void) state;
#endif
}
#endif /* NETWORK_DEV_WINC */

#ifdef NETWORK_DEV_WILC
/*
 * Invoked from within the WILC driver lock perimeter, so we can
 * call API functions directly.
 */
void
wilc_wifi_led_init(void)
{
#if defined(ACTIVITY_LED_NET) || defined(ACTIVITY_LED_WIFI) || \
    defined(ACTIVITY_LED_ERR) || defined(WILC_PROVISION_SW)
	m2m_periph_init(NULL);

#ifdef ACTIVITY_LED_NET
	m2m_periph_gpio_set_dir(ACTIVITY_LED_NET, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_NET, 1);
#endif
#ifdef ACTIVITY_LED_WIFI
	m2m_periph_gpio_set_dir(ACTIVITY_LED_WIFI, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_WIFI, 1);
#endif
#ifdef ACTIVITY_LED_ERR
	m2m_periph_gpio_set_dir(ACTIVITY_LED_ERR, 1);
	m2m_periph_gpio_set_val(ACTIVITY_LED_ERR, 1);
#endif
#ifdef WILC_PROVISION_SW
	m2m_periph_gpio_set_dir(WILC_PROVISION_SW, 0);
#endif
#else
	platform_wifi_led_init();
#endif
}

void
wilc_wifi_led_control(uint8_t which, uint8_t state)
{

#if defined(ACTIVITY_LED_NET) || defined(ACTIVITY_LED_WIFI) || \
    defined(ACTIVITY_LED_ERR)
	switch (which) {
#ifdef ACTIVITY_LED_NET
	case WILC_LED_NETWORK:
		m2m_periph_gpio_set_val(ACTIVITY_LED_NET, !state);
		break;
#endif
#ifdef ACTIVITY_LED_WIFI
	case WILC_LED_WIFI:
		m2m_periph_gpio_set_val(ACTIVITY_LED_WIFI, !state);
		break;
#endif
#ifdef ACTIVITY_LED_ERR
	case WILC_LED_ERR:
		m2m_periph_gpio_set_val(ACTIVITY_LED_ERR, !state);
		break;
#endif
	default:
		break;
	}
#else
	if (which == WILC_LED_WIFI)
		platform_wifi_led_set(state);
#endif
}
#endif /* NETWORK_DEV_WILC */
