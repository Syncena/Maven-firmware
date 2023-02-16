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
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "fix16.h"
#include "shell.h"
#include "display.h"
#include "display_bitmaps.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#endif
#include "spi-interface.h"
#include "timer.h"
#include "platform.h"
#include "hardware.h"
#include "ringbuff.h"
#include "version.h"
#include "u8g.h"
#include "product.h"
#include "zone_alloc.h"

#define	DISPLAY_WIDTH			128
#define	DISPLAY_HEIGHT			64
#define	DISPLAY_SPLASH_TIMEOUT		10	/* Seconds */

/* Font is 5 pixels wide by 10-2 pixels tall */
#define	DISPLAY_STATUS_FONT		u8g_font_profont10r
#define	DISPLAY_STATUS_FONT_WIDTH	5
#define	DISPLAY_STATUS_FONT_HEIGHT	8
#define	DISPLAY_STATUS_PIXELS_PER_ROW	DISPLAY_STATUS_FONT_HEIGHT
#define	DISPLAY_STATUS_IP_LEN		15
#define	DISPLAY_STATUS_RSSI_LEN		10
#define	DISPLAY_STATUS_COLUMNS		(DISPLAY_WIDTH / \
					 DISPLAY_STATUS_FONT_WIDTH)
#define	DISPLAY_STATUS_ROWS		2
#define	DISPLAY_STATUS_TEXT_START_Y	0
#define	DISPLAY_STATUS_TEXT_START_X	0

/* Font is 6 pixels wide by 12-1 pixels tall */
#define	DISPLAY_MAIN_FONT		u8g_font_profont12
#define	DISPLAY_MAIN_FONT_WIDTH		6
#define	DISPLAY_MAIN_FONT_HEIGHT	11
#define	DISPLAY_MAIN_PIXELS_PER_ROW	DISPLAY_MAIN_FONT_HEIGHT
#define	DISPLAY_MAIN_COLUMNS		(DISPLAY_WIDTH / \
					 DISPLAY_MAIN_FONT_WIDTH)
#define	DISPLAY_MAIN_TEXT_START_Y	(DISPLAY_STATUS_ROWS * \
					 DISPLAY_STATUS_PIXELS_PER_ROW)
#define	DISPLAY_MAIN_TEXT_START_X	0
#define	DISPLAY_MAIN_ROWS		((DISPLAY_HEIGHT - \
					  DISPLAY_MAIN_TEXT_START_Y) / \
					 DISPLAY_MAIN_PIXELS_PER_ROW)

#define	DISPLAY_FLAGS_FLIPPED		(1u << 0)
#define	DISPLAY_FLAGS_UART_TAP		(1u << 1)	/* XXX: Obsolete */
#define	DISPLAY_FLAGS_IGNORE_ACTIVITY	(1u << 2)
#define	DISPLAY_ORIENTATION_IS_NORMAL(ds)	\
					(((ds)->ds_flags & \
					 DISPLAY_FLAGS_FLIPPED) == 0)
#define	DISPLAY_WAKE_FOR_ACTIVITY(ds)	(((ds)->ds_flags & \
					 DISPLAY_FLAGS_IGNORE_ACTIVITY) == 0)

#define	DISPLAY_IPADDR_FONT		u8g_font_helvB12n

#define	SCREENSAVER_TICKS		1000

struct display_state {
	u8g_t ds_u8g;
	uint8_t ds_content;
#define	DISPLAY_CONTENT_NORMAL	0
#define	DISPLAY_CONTENT_SPLASH	1
#define	DISPLAY_CONTENT_IPADDR	2
	uint8_t ds_show_rssi;
	uint8_t ds_link_state;
	uint8_t ds_current_row;
	int8_t ds_current_col;
	int8_t ds_current_rssi;
#ifdef CONFIG_USE_NETWORK
	network_signal_strength_t ds_current_sig;
#endif
	uint8_t ds_net_ticks;
	uint8_t ds_usb_ticks;
	volatile uint8_t ds_stopped_status;
#define	DISPLAY_STOP_STATE_RUNNING	0
#define	DISPLAY_STOP_STATE_RESET	1
#define	DISPLAY_STOP_STATE_REINIT	2
	uint8_t ds_flags;
	uint32_t ds_current_ip;
	struct timeval ds_content_timer;
	const struct spi_interface *ds_spi;
	unsigned int ds_spi_idx;
	uint8_t ds_spi_buf[DISPLAY_WIDTH];
	void (*ds_dcrs_func)(void *, bool);
	void *ds_dcrs_arg;
	FILE *ds_ostream;
	uint16_t ds_ss_timer;
	int8_t ds_ss_x;
	int8_t ds_ss_y;
	int8_t ds_ss_dx;
	int8_t ds_ss_dy;
	rtos_task_t ds_task;
	rtos_mutex_t ds_mutex;
	rtos_timer_t ds_refresh_timer;
	rtos_timer_t ds_activity_timer;
	char ds_status[DISPLAY_STATUS_ROWS][DISPLAY_STATUS_COLUMNS + 16];
	char ds_buffer[DISPLAY_MAIN_ROWS][DISPLAY_MAIN_COLUMNS + 1];
};
static struct display_state display_state;
FILE *display_stream;

#define	DISPLAY_ACTIVITY_TICKS	5

SHELL_CMD_DECL(display, display_cmd, "Manipulate the onboard display");
#define	DISPLAY_CMD_STATUS_GLOB_SAVE	(1u << 0)
#define	DISPLAY_CMD_STATUS_SHOW		(1u << 1)
#define	DISPLAY_CMD_STATUS_USAGE	(1u << 2)

static const char display_cmd_usage[] = {
	"usage:\n"
	"display say [text] ...\n"
	"\tPrints some text onto the display\n"
#if (RELEASE_BUILD == 0)
	"display type <chipset-name>\n"
	"\tSelects the chipset used in the attached display.\n"
	"\tThe new selection will take effect after the next power-cycle.\n"
	"\tSee \"display type help\" for a list of supported chipsets.\n"
#endif
	"display flip\n"
	"\tFlips the display orientation.\n"
	"display wake <on|off>\n"
	"\tWake display on network/usb activity.\n"
};

struct display_driver {
	const char *dd_name;
	u8g_dev_t *dd_driver;
};

static const struct display_driver display_drivers[] = {
	{"sh1106",	&u8g_dev_sh1106_128x64_sw_spi},

// The following devices were early contenders...
//	{"ssd1306",	&u8g_dev_ssd1306_128x64_sw_spi},
//	{"ssd1309",	&u8g_dev_ssd1309_128x64_sw_spi},
//	{"st7920",	&u8g_dev_st7920_128x64_sw_spi}
};
#define	DISPLAY_NDRIVERS (sizeof(display_drivers) / sizeof(display_drivers[0]))

static const char *display_splash[4] = {
	PRODUCT_NAME " WiFi Programmer",
	"Vers " PRODUCT_VERSION_MAJOR "." PRODUCT_VERSION_MINOR "." PRODUCT_VERSION_TINY
	" (c) 2022",
	"Maverick Embedded",
	"Technology"
};
#define	DISPLAY_SPLASH_NROWS	(sizeof(display_splash) / \
				 sizeof(display_splash[0]))

static void
display_putchar(char ch, struct display_state *ds, uint8_t *pnotify,
    uint8_t *ptimer_reset)
{
	uint8_t row;
	int8_t col;

	row = ds->ds_current_row;
	col = ds->ds_current_col;

	switch (ch) {
	case '\b':
		/* Backspace */
		if (col > 0)
			col -= 1;
		break;

	case '\r':
		/* Carraige return */
		col = 0;
		break;

	case '\n':
		/* Newline */
		col = -1;
		if (pnotify)
			*pnotify = 1;
		break;

	case '\f':
		/* Formfeed - clears the display */
		for (row = 0; row < DISPLAY_MAIN_ROWS; row++)
			ds->ds_buffer[row][0] = '\0';
		row = 0;
		col = 0;
		if (pnotify)
			*pnotify = 1;
		break;

	default:
		/* Bail if the character isn't printable */
		if (!isprint((unsigned int)ch))
			return;

		/* col will be -1 if we need to advance to a new line */
		if (col < 0) {
			/*
			 * Move cursor to column zero of the next line, and
			 * scroll the display if necessary.
			 */
			col = 0;
			if (++row == DISPLAY_MAIN_ROWS) {
				/* Need to scroll */
				for (row = 1; row < DISPLAY_MAIN_ROWS; row++) {
					memcpy(ds->ds_buffer[row - 1],
					    ds->ds_buffer[row],
					    DISPLAY_MAIN_COLUMNS);
				}
				row = DISPLAY_MAIN_ROWS - 1;
			}
			ds->ds_buffer[row][0] = '\0';
		}

		/*
		 * Convert tabs to spaces. It's not worthwhile expanding
		 * them correctly on such a small display.
		 */
		if (ch == '\t')
			ch = ' ';

		/* Store character */
		ds->ds_buffer[row][col++] = ch;
		ds->ds_buffer[row][col] = '\0';

		/*
		 * We defer wrapping until the next character arrives. This
		 * avoids leaving a blank line until we have something to
		 * print on it.
		 */
		if (col == DISPLAY_MAIN_COLUMNS)
			col = -1;

		if (ptimer_reset)
			*ptimer_reset = 1;
		break;
	}

	ds->ds_current_row = row;
	ds->ds_current_col = col;
}

static int
display_write(void *arg, const char *buf, int buflen)
{
	struct display_state *ds = arg;
	int rv = buflen;
	uint8_t notify, timer_reset;

	rtos_mutex_acquire(ds->ds_mutex);

	notify = timer_reset = 0;

	while (buflen--)
		display_putchar(*buf++, ds, &notify, &timer_reset);

	if (notify) {
		ds->ds_ss_timer = SCREENSAVER_TICKS;
		rtos_timer_stop(ds->ds_refresh_timer);
		rtos_notify_give(ds->ds_task);
	} else
	if (timer_reset) {
		ds->ds_ss_timer = SCREENSAVER_TICKS;
		rtos_timer_reset(ds->ds_refresh_timer);
	}

	rtos_mutex_release(ds->ds_mutex);

	return rv;
}

static void
display_reset(struct display_state *ds)
{

	u8g_Begin(&ds->ds_u8g);
	u8g_SetColorIndex(&ds->ds_u8g, 1);
	u8g_SetDefaultForegroundColor(&ds->ds_u8g);
}

static void
display_refresh_timer_callback(rtos_timer_t handle)
{
	struct display_state *ds;

	ds = rtos_timer_get_id(handle);

	rtos_notify_give(ds->ds_task);
}

static bool
display_net_usb_wakeup(struct display_state *ds)
{

	if (ds->ds_usb_ticks != 0)
		ds->ds_usb_ticks -= 1;

	if (ds->ds_net_ticks != 0)
		ds->ds_net_ticks -= 1;

	if (DISPLAY_WAKE_FOR_ACTIVITY(ds))
		return ds->ds_net_ticks == 1 || ds->ds_usb_ticks == 1;

	return false;
}

static void
display_activity_timer_callback(rtos_timer_t handle)
{
	struct display_state *ds;
	rtos_saved_ipl_t ipl;
	bool do_notify;

	ds = rtos_timer_get_id(handle);

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	do_notify = display_net_usb_wakeup(ds);

	if (ds->ds_ss_timer != 0) {
		ds->ds_ss_timer -= 1;
		if (ds->ds_ss_timer == 0) {
			do_notify = true;
			rtos_timer_start(ds->ds_refresh_timer);
		}
	}

	rtos_ipl_restore(ipl);

	if (do_notify)
		rtos_notify_give(ds->ds_task);
}

void
display_activity(uint8_t activity)
{
	struct display_state *ds = &display_state;
	uint8_t *ptr, changed;

	switch (activity) {
	case DISPLAY_ACTIVITY_NETWORK:
		ptr = &ds->ds_net_ticks;
		break;
	case DISPLAY_ACTIVITY_USB:
		ptr = &ds->ds_usb_ticks;
		break;
	case DISPLAY_ACTIVITY_WAKE:
		changed = 1;
		ptr = NULL;
		break;
	default:
		return;
	}

	if (ptr != NULL) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		if (DISPLAY_WAKE_FOR_ACTIVITY(ds))
			changed = *ptr == 0;
		else
			changed = false;
		*ptr = DISPLAY_ACTIVITY_TICKS;
		rtos_ipl_restore(ipl);
	}

	if (changed) {
		ds->ds_ss_timer = SCREENSAVER_TICKS;
		rtos_timer_stop(ds->ds_refresh_timer);
		rtos_notify_give_any(ds->ds_task);
	}
}

static void
display_network_activity_callback(void *arg)
{
	struct display_state *ds = arg;

	if (ds->ds_net_ticks < (DISPLAY_ACTIVITY_TICKS / 2))
		display_activity(DISPLAY_ACTIVITY_NETWORK);
}

void
display_provision_details(void)
{
#if defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
	if (display_stream == NULL)
		return;

#ifdef CONFIG_USE_CONFIGDB
	char *v;

	if ((v = zone_malloc(256)) != NULL) {
		cf_get_string(CF_KEY_STRING_WIFI_PROV_SSID, 256, v);
		fprintf(display_stream, "\fNetwork name:\n%s\n", v);
		if (cf_get_string(CF_KEY_STRING_WIFI_PROV_PASS, 256, v) != 0)
			fprintf(display_stream, "WiFi Pass:\n%s\n", v);
		zone_free(v);
	}
#else
	fprintf(display_stream, "\fNetwork name:\n%s\n",
	    glob.glob_wifi.gv_prov_ssid);
#endif
#endif /* defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC) */
}

#ifdef CONFIG_USE_NETWORK
static void
display_network_link_status_callback(const network_link_status_t *ls, void *arg)
{
	struct display_state *ds = arg;
	const network_ip_params_t *ipp;
	char status_string[DISPLAY_STATUS_IP_LEN + 1];
	char rssi_string[DISPLAY_STATUS_RSSI_LEN + 1];
	uint8_t link_changed;

	rssi_string[0] = '\0';

	if (ls->ls_state != NETWORK_LINK_UP) {
		ds->ds_current_ip = 0;
		ds->ds_current_rssi = 0;
		link_changed = (ls->ls_state != ds->ds_link_state);

		if (link_changed) {
			switch (ls->ls_state) {
			case NETWORK_LINK_DOWN:
				strcpy(status_string, "No connection");
				fprintf(display_stream, "\fNetwork down\n");
				break;
			case NETWORK_LINK_CONNECTING:
				strcpy(status_string, "Connecting");
				if (ls->ls_bssid[0] != '\0') {
					fprintf(display_stream,
					    "\fNetwork:\n%s\n", ls->ls_bssid);
				}
				break;
			case NETWORK_LINK_PROVISIONING:
				strcpy(status_string, "Provisioning");
				display_provision_details();
				break;
			}
		}
	} else {
		if (ds->ds_link_state != ls->ls_state &&
		    ds->ds_link_state != NETWORK_LINK_CONNECTING) {
			fprintf(display_stream, "\f");
		}

		ipp = network_get_ip_params();

		if (ds->ds_link_state != ls->ls_state ||
		    ipp->ip_addr != ds->ds_current_ip ||
		    ds->ds_current_rssi != ls->ls_rssi) {
			ds->ds_current_ip = ipp->ip_addr;
			ds->ds_current_rssi = ls->ls_rssi;
			ds->ds_current_sig = ls->ls_sig;
			network_ip2str(ds->ds_current_ip, status_string);
			if (ds->ds_show_rssi) {
				snprintf(rssi_string, sizeof(rssi_string),
				    "%d dBm", ls->ls_rssi);
			} else
				rssi_string[0] = '\0';
			link_changed = 1;
		} else {
			link_changed = 0;
		}
	}

#define	_STR_HELPER(x)	#x
#define	_STR(x)		_STR_HELPER(x)

	if (link_changed) {
		ds->ds_link_state = ls->ls_state;
		snprintf(ds->ds_status[0], sizeof(ds->ds_status[0]),
		    "%-" _STR(DISPLAY_STATUS_RSSI_LEN) "s%"
		        _STR(DISPLAY_STATUS_IP_LEN) "s",
			rssi_string, status_string);

		rtos_notify_give(ds->ds_task);
	}
}
#endif /* CONFIG_USE_NETWORK */

static uint8_t
display_centre_string(struct display_state *ds, unsigned int start_x,
    const char *str)
{
	int sw;

	sw = DISPLAY_WIDTH - start_x;

	sw -= (int)u8g_GetStrPixelWidth(&ds->ds_u8g, str);

	if (sw < 0)
		sw = 0;

	return (uint8_t)sw / 2u;
}

static void
display_redraw(struct display_state *ds)
{
	struct display_bitmap signal_bm, activity_bm;
	uint8_t row, x, y, net_ticks, usb_ticks;
	char status_string[DISPLAY_STATUS_IP_LEN + 1];

	if (ds->ds_content != DISPLAY_CONTENT_NORMAL) {
		struct timeval tv;

		/* Show the alternate screen for 6 seconds */
		timer_get_mono_time(&tv);
		if (timer_cmp(&tv, &ds->ds_content_timer, >=))
			ds->ds_content = DISPLAY_CONTENT_NORMAL;
		else
		if (ds->ds_content == DISPLAY_CONTENT_IPADDR)
			network_ip2str(ds->ds_current_ip, status_string);
	}

	memset(&signal_bm, 0, sizeof(signal_bm));
	if (ds->ds_show_rssi == 0) {
		static const uint8_t bm[] = {
			NETWORK_SIG_NONE, NETWORK_SIG_POOR, NETWORK_SIG_FAIR,
			NETWORK_SIG_GOOD, NETWORK_SIG_EXCELLENT
		};

		assert(ds->ds_current_sig < sizeof(bm));

		display_bitmap_get(bm[ds->ds_current_sig], &signal_bm);
	}

	net_ticks = ds->ds_net_ticks;
	usb_ticks = ds->ds_usb_ticks;

	if (net_ticks || usb_ticks) {
		uint8_t bm = 0;

		if (net_ticks && usb_ticks)
			bm = DISPLAY_BITMAP_ACTIVITY_NET_AND_USB;
		else
		if (net_ticks)
			bm = DISPLAY_BITMAP_ACTIVITY_NET_ONLY;
		else
		if (usb_ticks)
			bm = DISPLAY_BITMAP_ACTIVITY_USB_ONLY;

		display_bitmap_get(bm, &activity_bm);
	}

	u8g_SetDisplayOrientation(&ds->ds_u8g,
	    DISPLAY_ORIENTATION_IS_NORMAL(ds));

	u8g_FirstPage(&ds->ds_u8g);
	do {
		u8g_SetFont(&ds->ds_u8g, DISPLAY_STATUS_FONT);
		u8g_SetFontPosTop(&ds->ds_u8g);
		u8g_SetFontRefHeightText(&ds->ds_u8g);

		for (row = 0; row < DISPLAY_STATUS_ROWS; row++) {
			y = row * DISPLAY_STATUS_PIXELS_PER_ROW;
			y += DISPLAY_STATUS_TEXT_START_Y;
			u8g_DrawStr(&ds->ds_u8g, DISPLAY_STATUS_TEXT_START_X,
			    y, ds->ds_status[row]);
		}

		if (ds->ds_show_rssi == 0) {
			u8g_DrawXBMP(&ds->ds_u8g, 0, 0, signal_bm.b_width,
			    signal_bm.b_height, signal_bm.b_bits);
		}

		if (net_ticks || usb_ticks) {
			u8g_DrawXBMP(&ds->ds_u8g, 24, 0, activity_bm.b_width,
			    activity_bm.b_height, activity_bm.b_bits);
		}

		if (ds->ds_content != DISPLAY_CONTENT_IPADDR)
			u8g_SetFont(&ds->ds_u8g, DISPLAY_MAIN_FONT);
		else
			u8g_SetFont(&ds->ds_u8g, DISPLAY_IPADDR_FONT);
		u8g_SetFontPosTop(&ds->ds_u8g);
		u8g_SetFontRefHeightText(&ds->ds_u8g);

		switch (ds->ds_content) {
		default:
			for (row = 0; row < DISPLAY_MAIN_ROWS; row++) {
				y = row * DISPLAY_MAIN_PIXELS_PER_ROW;
				y += DISPLAY_MAIN_TEXT_START_Y;
				u8g_DrawStr(&ds->ds_u8g,
				    DISPLAY_MAIN_TEXT_START_X, y,
				    ds->ds_buffer[row]);
			}
			break;

		case DISPLAY_CONTENT_SPLASH:
			/*
			 * Would be nice to have some graphics here.
			 * This also assumes the main display has 4 rows.
			 */
			y = DISPLAY_MAIN_TEXT_START_Y;

			for (row = 0; row < DISPLAY_SPLASH_NROWS; row++) {
				x = display_centre_string(ds,
				    DISPLAY_MAIN_TEXT_START_X,
				    display_splash[row]);
				u8g_DrawStr(&ds->ds_u8g, x,
				    y + (row * DISPLAY_MAIN_PIXELS_PER_ROW),
				    display_splash[row]);
			}
			break;

		case DISPLAY_CONTENT_IPADDR:
			x = display_centre_string(ds, DISPLAY_MAIN_TEXT_START_X,
			    status_string);
			u8g_DrawStr(&ds->ds_u8g, x, DISPLAY_HEIGHT / 2,
			    status_string);
			break;
		}
	} while (u8g_NextPage(&ds->ds_u8g));
}

static void
display_screensaver(struct display_state *ds)
{
	int x, y;

	x = (int) ds->ds_ss_x;
	y = (int) ds->ds_ss_y;

	u8g_FirstPage(&ds->ds_u8g);
	do {
		u8g_DrawPixel(&ds->ds_u8g, x + 0, y + 0);
		u8g_DrawPixel(&ds->ds_u8g, x + 1, y + 0);
		u8g_DrawPixel(&ds->ds_u8g, x + 0, y + 1);
		u8g_DrawPixel(&ds->ds_u8g, x + 1, y + 1);
	} while (u8g_NextPage(&ds->ds_u8g));

	x += (int) ds->ds_ss_dx;
	y += (int) ds->ds_ss_dy;

	if (x < 0) {
		ds->ds_ss_dx = ((SysTick->VAL & 3u) == 0) ? 2 : 1;
		x = 0;
	} else
	if (x > (DISPLAY_WIDTH - 5)) {
		ds->ds_ss_dx = ((SysTick->VAL & 3u) == 0) ? -2 : -1;
		x = DISPLAY_WIDTH - 5;
	}

	if (y < 0) {
		ds->ds_ss_dy = ((SysTick->VAL & 3u) == 0) ? 2 : 1;
		y = 0;
	} else
	if (y > (DISPLAY_HEIGHT - 2)) {
		ds->ds_ss_dy = ((SysTick->VAL & 3u) == 0) ? -2 : -1;
		y = DISPLAY_HEIGHT - 2;
	}

	ds->ds_ss_x = (int8_t)x;
	ds->ds_ss_y = (int8_t)y;
}

static void
display_coms_flush(struct display_state *ds)
{
	struct spi_iovec io;

	if (ds->ds_spi_idx == 0)
		return;

	io.io_wr = ds->ds_spi_buf;
	io.io_rd = NULL;
	io.io_len = ds->ds_spi_idx;

	(void) ds->ds_spi->si_read_write(ds->ds_spi, 1, &io);

	ds->ds_spi_idx = 0;
}

static void
display_coms_write(struct display_state *ds, uint8_t v)
{

	if (ds->ds_spi_idx == sizeof(ds->ds_spi_buf))
		display_coms_flush(ds);
	ds->ds_spi_buf[ds->ds_spi_idx++] = v;
}

static uint8_t
display_coms(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
	struct display_state *ds = (void *)u8g;
	const uint8_t *ptr = arg_ptr;

	switch (msg) {
	case U8G_COM_MSG_INIT:
		ds->ds_spi_idx = 0;
		break;
	case U8G_COM_MSG_ADDRESS:
		display_coms_flush(ds);
		(ds->ds_dcrs_func)(ds->ds_dcrs_arg, arg_val != 0);
		break;
	case U8G_COM_MSG_CHIP_SELECT:
		if (arg_val == 0)
			display_coms_flush(ds);
		break;
	case U8G_COM_MSG_WRITE_BYTE:
		display_coms_write(ds, arg_val);
		break;
	case U8G_COM_MSG_WRITE_SEQ:
	case U8G_COM_MSG_WRITE_SEQ_P:
		while (arg_val--)
			display_coms_write(ds, *ptr++);
		break;
	default:
		break;
	}

	return 1;
}

static u8g_dev_t *
display_find_driver(const char *name)
{
	const struct display_driver *dd;
	u8g_dev_t *driver;

	for (driver = NULL, dd = display_drivers;
	    dd < &display_drivers[DISPLAY_NDRIVERS]; dd++) {
		if (strcasecmp(name, dd->dd_name) == 0) {
			driver = dd->dd_driver;
			break;
		}
	}

	return driver;
}

static const char *
humanise_clock(uint32_t clk, fix16_t *phclk)
{
	const char *str;
	fix16_t tmp;

	if (clk >= 1000000u) {
		clk /= 1000u;
		tmp = fix16_from_int((int)clk);
		*phclk = fix16_div(tmp, F16(1000.0));
		str = "MHz";
	} else
	if (clk >= 100000u) {
		clk /= 100u;
		tmp = fix16_from_int((int)clk);
		*phclk = fix16_div(tmp, F16(10.0));
		str = "KHz";
	} else
	if (clk >= 1000u) {
		tmp = fix16_from_int((int)clk);
		*phclk = fix16_div(tmp, F16(1000.0));
		str = "KHz";
	} else {
		*phclk = fix16_from_int((int)clk);
		str = "Hz";
	}

	return str;
}

static void
display_task(void *arg)
{
	struct display_state *ds = arg;
	network_link_status_t ls;
	const char *driver_name;
	u8g_dev_t *driver;
	const char *pclk_name, *pclk_suff;
	uint32_t pclk;
	char fstr[16];
	fix16_t f;
	int sz;

#ifdef CONFIG_USE_CONFIGDB
	if (!cf_get_string(CF_KEY_STRING_DISPLAY_DRIVER, sizeof(fstr), fstr)) {
		fstr[0] = '\0';
		display_load_defaults();
		cf_get_string(CF_KEY_STRING_DISPLAY_DRIVER, sizeof(fstr), fstr);
		driver_name = fstr;
	}
#else
	driver_name = glob.glob_display_driver_name;
#endif

	driver = display_find_driver(driver_name);
	if (driver == NULL)
		driver = &u8g_dev_sh1106_128x64_sw_spi;

	u8g_InitComFn(&ds->ds_u8g, driver, display_coms);

	ds->ds_stopped_status = DISPLAY_STOP_STATE_REINIT;

	rtos_task_sleep(100);

#ifdef CONFIG_USE_NETWORK
	network_get_link_status(&ls);
	display_network_link_status_callback(&ls, ds);
	network_hook_link_status(display_network_link_status_callback, ds);
	network_hook_activity(display_network_activity_callback, ds);
#endif

	for (;;) {
		fflush(display_stream);

		rtos_mutex_acquire(ds->ds_mutex);

		pclk_name = product_status(&pclk);

		if (POWER_TARGET_VCC_LOW)
			strcpy(fstr, "LO ");
		else
		if (POWER_TARGET_VCC_HIGH)
			strcpy(fstr, "HI ");
		else
			fix16_to_str(power_target_vcc, fstr, 1);

		sz = snprintf(ds->ds_status[1], sizeof(ds->ds_status[1]),
		    "TVcc %s ", fstr);

		if (pclk_name == NULL || power_vddio_state == 0) {
			fix16_to_str(power_vddio, fstr, 1);
			snprintf(&ds->ds_status[1][sz],
			    sizeof(ds->ds_status[1]) - sz,
			    "%8s Vcc %s",
			    power_vddio_state ?  PRODUCT_NAME : "LOW",
			    fstr);
		} else
		if (pclk_name != NULL) {
			pclk_suff = humanise_clock(pclk, &f);
			fix16_to_str(f, fstr, 2);
			snprintf(&ds->ds_status[1][sz],
			    sizeof(ds->ds_status[1]) - sz, "%s %s %s",
			    pclk_name, fstr, pclk_suff);
		}

		/*
		 * On the original wAVR board, the display's reset pin is
		 * shared with the WiFi module. If the WiFi chip is reset,
		 * we will be notified and will need to re-initialise the
		 * display controller once reset has been de-asserted.
		 *
		 * Note that reset will not be asserted while we are holding
		 * the display mutex.
		 */
		if (ds->ds_stopped_status == DISPLAY_STOP_STATE_REINIT) {
			ds->ds_stopped_status = DISPLAY_STOP_STATE_RUNNING;
			display_reset(ds);
		}

		if (ds->ds_stopped_status == DISPLAY_STOP_STATE_RUNNING) {
			if (ds->ds_ss_timer == 0)
				display_screensaver(ds);
			else
				display_redraw(ds);
		}

		rtos_mutex_release(ds->ds_mutex);

		(void) rtos_notify_take_timed(1, 1000);
	}
}

static void
display_cmd_say(FILE *os, uint8_t argc, const char * const *argv)
{

	(void)os;

	while (argc--) {
		fputs(*argv++, display_stream);
		putc(' ', display_stream);
	}
	putc('\n', display_stream);
}

static uint8_t
display_cmd_type(FILE *os, uint8_t argc, const char * const *argv)
{
	const struct display_driver *dd;
	uint8_t rv = 0;
	const char *dname;

#ifndef CONFIG_USE_CONFIGDB
	dname = glob.glob_display_driver_name;
#else
	char dstr[16];

	dstr[0] = '\0';
	cf_get_string(CF_KEY_STRING_DISPLAY_DRIVER, sizeof(dstr), dstr);
	dname = dstr;
#endif

	(void) argc;

	if (display_find_driver(argv[0]) != NULL) {
		if (strcasecmp(argv[0], dname) != 0) {
			fprintf(os, "Driver name changed from '%s' to '%s'\n",
			    dname, argv[1]);
			fprintf(os, "This will take effect after the next"
			    " power-cycle\n");

#ifndef CONFIG_USE_CONFIGDB
			strlcpy(glob.glob_display_driver_name, argv[1],
			    sizeof(glob.glob_display_driver_name));
#else
			cf_set_string(CF_KEY_STRING_DISPLAY_DRIVER, argv[1]);
#endif
			rv = DISPLAY_CMD_STATUS_GLOB_SAVE;
		} else {
			rv = DISPLAY_CMD_STATUS_SHOW;
		}
	} else
	if (strcasecmp(argv[0], "help") == 0) {
		fprintf(os, "Supported display driver chipsets are:");
		for (dd = display_drivers;
		    dd < &display_drivers[DISPLAY_NDRIVERS]; dd++) {
			fprintf(os, " %s", dd->dd_name);
		}
		fprintf(os, "\n");
	} else {
		rv = DISPLAY_CMD_STATUS_USAGE;
	}

	return rv;
}

static uint8_t
display_cmd_wake(struct display_state *ds, const char * const *argv)
{
	uint8_t rv = 0, old;

	old = ds->ds_flags;

	if (strcasecmp(argv[0], "on") == 0)
		ds->ds_flags &= ~DISPLAY_FLAGS_IGNORE_ACTIVITY;
	else
	if (strcasecmp(argv[0], "off") == 0)
		ds->ds_flags |= DISPLAY_FLAGS_IGNORE_ACTIVITY;
	else
		return DISPLAY_CMD_STATUS_USAGE;

	if (ds->ds_flags != old)
		rv = DISPLAY_CMD_STATUS_GLOB_SAVE;

	return rv;
}

static uint8_t
display_cmd_rssi(struct display_state *ds, FILE *os, uint8_t argc,
    const char * const *argv)
{

	(void) os;
	(void) argc;

	if (strcasecmp(argv[0], "on") == 0) {
		ds->ds_show_rssi = 1;
	} else
	if (strcasecmp(argv[0], "off") == 0) {
		ds->ds_show_rssi = 0;
	} else
		return DISPLAY_CMD_STATUS_USAGE;

	return 0;
}

static void
display_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct display_state *ds = &display_state;
	uint8_t status = DISPLAY_CMD_STATUS_SHOW;

	/*
	 * Don't want to hold the display mutex for the 'say' command
	 * or else we risk a deadlock if the text causes the stream
	 * to be flushed.
	 */
	if (argc >= 1 && strcasecmp(argv[0], "say") == 0) {
		display_cmd_say(os, argc - 1, argv + 1);
		return;
	}

	rtos_mutex_acquire(ds->ds_mutex);

	if (argc == 1 && strcasecmp(argv[0], "flip") == 0) {
		ds->ds_flags ^= DISPLAY_FLAGS_FLIPPED;
		status = DISPLAY_CMD_STATUS_GLOB_SAVE;
	} else
	if (argc == 2 && (strcasecmp(argv[0], "type") == 0 ||
	    strcasecmp(argv[0], "driver") == 0)) {
		status = display_cmd_type(os, argc - 1, argv + 1);
	} else
	if (argc == 2 && (strcasecmp(argv[0], "wake") == 0)) {
		status = display_cmd_wake(ds, argv + 1);
	} else
	if (argc == 2 && strcasecmp(argv[0], "rssi") == 0) {
		status = display_cmd_rssi(ds, os, argc - 1, argv + 1);
	} else
	if (argc > 0)
		status = DISPLAY_CMD_STATUS_USAGE;

	rtos_mutex_release(ds->ds_mutex);

	if (status & DISPLAY_CMD_STATUS_USAGE)
		fprintf(os, "%s", display_cmd_usage);

	if (status & DISPLAY_CMD_STATUS_GLOB_SAVE) {
#ifndef CONFIG_USE_CONFIGDB
		glob.glob_display_flags = ds->ds_flags;
		glob_save();
#else
		cf_set_uint8(CF_KEY_UINT8_DISPLAY_FLAGS, ds->ds_flags);
		configdb_persist();
#endif
	}

	if (status & DISPLAY_CMD_STATUS_SHOW) {
		const char *dname;
#ifndef CONFIG_USE_CONFIGDB
		dname = glob.glob_display_driver_name;
#else
		char dstr[16];
		dstr[0] = '\0';
		cf_get_string(CF_KEY_STRING_DISPLAY_DRIVER, sizeof(dstr), dstr);
		dname = dstr;
#endif
		fprintf(os, "Display type: %s\n", dname);
		/* Note: reversal of meaning here is intentional */
		fprintf(os, "Display orientation: %s\n",
		    DISPLAY_ORIENTATION_IS_NORMAL(ds) ? "Normal" : "Flipped");
		fprintf(os, "Wake for activity: %s\n",
		    DISPLAY_WAKE_FOR_ACTIVITY(ds) ? "On" : "Off");
	}
}

void
u8g_Delay(uint16_t ms)
{

	if (rtos_scheduler_running())
		rtos_task_sleep(ms);
	else
		timer_buzz_delay_ms(ms);
}

void
display_stop(void)
{
	struct display_state *ds = &display_state;

	if (ds->ds_spi == NULL)
		return;

	rtos_mutex_acquire(ds->ds_mutex);
	ds->ds_stopped_status = DISPLAY_STOP_STATE_RESET;
	rtos_mutex_release(ds->ds_mutex);
}

void
display_restart(void)
{
	struct display_state *ds = &display_state;

	if (ds->ds_spi == NULL)
		return;

	rtos_mutex_acquire(ds->ds_mutex);
	ds->ds_stopped_status = DISPLAY_STOP_STATE_REINIT;
	rtos_notify_give(ds->ds_task);
	rtos_mutex_release(ds->ds_mutex);
}

void
display_ip_address(void)
{
	struct display_state *ds = &display_state;
	struct timeval tv1, tv2;
	bool wakeup;

	rtos_mutex_acquire(ds->ds_mutex);

	if ((wakeup = ds->ds_current_ip != 0)) {
		ds->ds_content = DISPLAY_CONTENT_IPADDR;
		timer_get_mono_time(&tv1);
		tv2.tv_sec = 5;
		tv2.tv_usec = 0;
		timer_add(&tv1, &tv2, &ds->ds_content_timer);
	}

	rtos_mutex_release(ds->ds_mutex);

	if (wakeup)
		display_activity(DISPLAY_ACTIVITY_WAKE);
}

void
display_attach(const struct spi_interface *spi, void (*dcrs_func)(void *, bool),
    void *dcrs_arg)
{
	struct display_state *ds = &display_state;

	ds->ds_spi = spi;
	ds->ds_dcrs_func = dcrs_func;
	ds->ds_dcrs_arg = dcrs_arg;

	ds->ds_current_row = 0;
	ds->ds_current_col = 0;

#ifndef CONFIG_USE_CONFIGDB
	ds->ds_flags = glob.glob_display_flags;
#else
	if (cf_get_uint8(CF_KEY_UINT8_DISPLAY_FLAGS, &ds->ds_flags) == 0) {
		display_load_defaults();
		cf_get_uint8(CF_KEY_UINT8_DISPLAY_FLAGS, &ds->ds_flags);
	}
#endif

	ds->ds_ostream = fwopen(ds, display_write);
	assert(ds->ds_ostream != NULL);

	/* Small I/O buffer only please */
	ds->ds_ostream->_flags |= __SMBF;
	display_stream = ds->ds_ostream;

	SHELL_CMD_ADD(display);

	ds->ds_link_state = (uint8_t)(~0u);
	ds->ds_current_rssi = 0;
	ds->ds_current_sig = NETWORK_SIG_NONE;
	ds->ds_content = DISPLAY_CONTENT_SPLASH;
	ds->ds_content_timer.tv_sec = 6;
	ds->ds_content_timer.tv_usec = 0;
	ds->ds_ss_x = 0;
	ds->ds_ss_y = 1;
	ds->ds_ss_dx = 1;
	ds->ds_ss_dy = 1;
	ds->ds_ss_timer = SCREENSAVER_TICKS;

	ds->ds_mutex = rtos_mutex_create();
	assert(ds->ds_mutex != NULL);

	ds->ds_task = rtos_task_create("Display", display_task, ds,
	    RTOS_TASK_PRIORITY_LOW, RTOS_STACK_SIZE(900));
	assert(ds->ds_task != NULL);

	ds->ds_refresh_timer = rtos_timer_create("DispRefresh", 100, 1,
	    display_refresh_timer_callback, ds);
	assert(ds->ds_refresh_timer != NULL);

	ds->ds_activity_timer = rtos_timer_create("DispActivity", 100, 1,
	    display_activity_timer_callback, ds);
	assert(ds->ds_activity_timer != NULL);
	rtos_timer_start(ds->ds_activity_timer);
}

void
display_load_defaults(void)
{

#ifndef CONFIG_USE_CONFIGDB
	strcpy(glob.glob_display_driver_name, "sh1106");
	glob.glob_display_flags = 0;
#else
	cf_set_string(CF_KEY_STRING_DISPLAY_DRIVER, "sh1106");
	cf_set_uint8(CF_KEY_UINT8_DISPLAY_FLAGS, 0);
#endif
}
