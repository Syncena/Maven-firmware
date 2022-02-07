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
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"
#include "platform_usb.h"
#include "product.h"
#include "shell.h"
#include "serial_number.h"
#include "usb_common.h"
#include "tusb.h"

#if (CFG_TUD_CDC > 0)
#include "usb_cdc.h"
#endif
#if (CFG_TUD_VENDOR > 0)
#include "usb_vendor.h"
#endif

#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif

#ifndef USB_FIXED_VENDOR_PRODUCT_IDS
#if defined(CONFIG_USE_CONFIGDB) || defined(CONFIG_USE_USB_VENDOR)
SHELL_CMD_DECL(usb, usb_cmd, "Configure USB");
#endif
#endif

char usb_serial_no[USB_DEVICE_GET_SERIAL_NAME_LENGTH];

struct usb_common_state {
	void *cs_dev_state;
	unsigned int cs_usb_mode;
	rtos_task_t cs_task;
};
static struct usb_common_state usb_common_state;

/*
 * Supported USB personalities.
 */
#define	USB_MODE_VENDOR(cs)	((cs)->cs_usb_mode == USB_PERSONALITY_VENDOR)
#define	USB_MODE_CDC(cs)	((cs)->cs_usb_mode == USB_PERSONALITY_CDC)

/* The active USB configuration is copied here. */
tusb_desc_device_t	device_descriptor;

/*
 * Invoked when received GET DEVICE DESCRIPTOR
 * Application return pointer to descriptor
 */
uint8_t const *
tud_descriptor_device_cb(void)
{

	return (uint8_t const *) &device_descriptor;
}

uint8_t
usb_get_config(uint16_t *pvid, uint16_t *ppid)
{
	uint16_t vid;
	uint8_t mode;

	/*
	 * Fetch the configuration from persistant storage.
	 */
#ifndef CONFIG_USE_CONFIGDB
	/* USB CDC is enabled by default on Maven1 */
	vid = USB_CDC_DEVICE_VENDOR_ID;
	if (ppid != NULL)
		*ppid = USB_CDC_DEVICE_PRODUCT_ID;

#ifdef CONFIG_USE_USB_VENDOR
	mode = glob.glob_usb_personality;
#else
	mode = USB_PERSONALITY_CDC;
#endif

#else /* CONFIG_USE_CONFIGDB */
	/* USB CDC is contingent on non-zero VID:PID on later product. */
	if (cf_get_uint16(CF_KEY_UINT16_USB_VID, &vid) != sizeof(vid))
		vid = USB_DEFAULT_DEVICE_VENDOR_ID;
	if (ppid != NULL &&
	    cf_get_uint16(CF_KEY_UINT16_USB_PID, ppid) != sizeof(*ppid)) {
		*ppid = USB_DEFAULT_DEVICE_PRODUCT_ID;
	}

	/*
	 * If Vendor support (for AVR) is enabled, then grab the mode
	 * from configdb. Otherwise, default to USB disabled.
	 */
#ifdef CONFIG_USE_USB_VENDOR
	if (cf_get_uint8(CF_KEY_UINT8_USB_PERSONALITY, &mode) == 0)
#endif
		mode = USB_PERSONALITY_DISABLED;

#endif	/* CONFIG_USE_CONFIGDB */

	/*
	 * Now sanity-check the configurations. This is required because
	 * things can become inconsistent if the customer swaps firmware
	 * images between AVR/ARM (for example).
	 */
	if (vid == 0 && mode == USB_PERSONALITY_CDC)
		mode = USB_PERSONALITY_DISABLED;
	else
	if (vid != 0 && mode == USB_PERSONALITY_DISABLED)
		mode = USB_PERSONALITY_CDC;

	if (pvid != NULL)
		*pvid = vid;

	return mode;
}

static void
usb_task_fn(void *arg)
{
	struct usb_common_state *cs = arg;
	const tusb_desc_device_t *cfg = NULL;

	if (USB_MODE_CDC(cs))
		cfg = usb_cdc_attach(cs, &cs->cs_dev_state);
#ifdef CONFIG_USE_USB_VENDOR
	else
	if (USB_MODE_VENDOR(cs))
		cfg = usb_vendor_attach(cs, &cs->cs_dev_state);
#endif

	assert(cfg != NULL);

	memcpy(&device_descriptor, cfg, sizeof(device_descriptor));

	rtos_task_sleep(500);

	tusb_init();

	for (;;) {
		tud_task();
	}
}

#ifndef USB_FIXED_VENDOR_PRODUCT_IDS
#if defined(CONFIG_USE_CONFIGDB) || defined(CONFIG_USE_USB_VENDOR)
#ifdef CONFIG_USE_CONFIGDB
#define	USB_MODE_CMD	"VCOM"
#else
#define	USB_MODE_CMD	"CDC"
#endif
static const char usb_cmd_usage_string[] = {
"Usage:\n"
"usb\n\tDisplay current USB settings.\n"
#ifdef CONFIG_USE_CONFIGDB
"usb id <VID:PID>\n"
"\tIn order to enable support for USB, " PRODUCT_NAME " requires valid "
 "USB Vendor\n"
"\tand Product IDs. Unfortunately, acquiring a Vendor ID costs a\n"
"\tsubstantial amount of money; too much for a small company to bear for\n"
"\ta product whose primary use-case does not involve USB. Thus "
 PRODUCT_NAME "'s\n"
"\tUSB virtual COM ports are disabled by default, but can be enabled by\n"
"\tspecifying a VID:PID pair here. You can choose any value between 0x0001\n"
"\tand 0xfffe for either of them, but you must ensure the value for VID\n"
"\tdoes not clash with an existing device. Higher values (above 0x4000)\n"
"\tseem to be less prevalent. Setting VID:PID to 0:0 will disable support\n"
"\tfor USB virtual COM ports. Note that VID and PID are interpreted as\n"
"\thexadecimal values. Also note than any change will take effect after a\n"
"\treboot.\n"
#endif
#ifdef CONFIG_USE_USB_VENDOR
"usb mode [ \"" USB_MODE_CMD "\" | \"STK600\" ]\n"
"\tChange USB mode to either " USB_MODE_CMD " (USB Serial Port) or STK600\n"
"\temulation (for Atmel Studio's benefit). "
#ifdef CONFIG_USE_CONFIGDB
"Note that the USB VID and PID\n"
"\tare chosen automatically in STK600 mode.\n"
#else
"\n"
#endif
"\tNote that a mode change will take effect after a reboot.\n"
#endif
};

static void
usb_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	size_t l = argc ? strlen(argv[0]) : 0;
#ifdef CONFIG_USE_CONFIGDB
	unsigned int new_vid, new_pid;
	uint16_t vid, pid;
#endif
	uint8_t mode, new_mode;
	bool changed;

#ifndef CONFIG_USE_CONFIGDB
	new_mode = mode = usb_get_config(NULL, NULL);
#else
	new_mode = mode = usb_get_config(&vid, &pid);
	new_vid = (unsigned int)vid;
	new_pid = (unsigned int)pid;

	if (argc == 2 && strncasecmp(argv[0], "id", l) == 0) {
		if (sscanf(argv[1], "%x:%x", &new_vid, &new_pid) != 2 ||
		    new_vid > 0xffffu || new_pid > 0xffffu) {
			fprintf(os, "Invalid VID:PID pair\n");
			return;
		}

		if (mode != USB_PERSONALITY_VENDOR) {
			if (new_vid == 0)
				new_mode = USB_PERSONALITY_DISABLED;
			else
				new_mode = USB_PERSONALITY_CDC;
		}
	} else
#endif
#ifdef CONFIG_USE_USB_VENDOR
	if (argc == 2 && strncasecmp(argv[0], "mode", l) == 0) {
		l = strlen(argv[1]);
		if (strncasecmp(argv[1], USB_MODE_CMD, l) == 0) {
#ifdef CONFIG_USE_CONFIGDB
			if (new_vid == 0) {
				new_mode = USB_PERSONALITY_DISABLED;
				fprintf(os, "USB VID:PID also needs to be"
				    " configured.\n");
			} else {
				new_mode = USB_PERSONALITY_CDC;
			}
#else
			new_mode = USB_PERSONALITY_CDC;
#endif
		} else
		if (strncasecmp(argv[1], "stk600", l) == 0)
			new_mode = USB_PERSONALITY_VENDOR;
		else {
			fprintf(os, "Mode can be VCOM or STK600\n");
			return;
		}
	} else
#endif	/* CONFIG_USE_USB_VENDOR */
	if (argc != 0) {
		fprintf(os, "%s", usb_cmd_usage_string);
		return;
	}

#ifdef CONFIG_USE_USB_VENDOR
	fprintf(os, "USB Mode: %s\n", (new_mode == USB_PERSONALITY_CDC) ?
	    USB_MODE_CMD : ((new_mode == USB_PERSONALITY_VENDOR) ? "STK600" :
	    "Disabled"));

	changed = new_mode != mode;
#else
	changed = false;
#endif

#ifdef CONFIG_USE_CONFIGDB
	fprintf(os, "USB Virtual COM support %s %sabled. VID:PID = %04X:%04x\n",
	    (usb_common_state.cs_usb_mode != new_mode) ? "will be" : "is",
	    (new_pid == 0) ? "dis" : "en", new_vid, new_pid);

	changed = changed || new_vid != vid || new_pid != pid;
#endif

	if (changed) {
#ifndef CONFIG_USE_CONFIGDB
#ifdef CONFIG_USE_USB_VENDOR
		glob.glob_usb_personality = new_mode;
#endif
		glob_save();

#else	/* CONFIG_USE_CONFIGDB */
		cf_set_uint16(CF_KEY_UINT16_USB_VID, (uint16_t)new_vid);
		cf_set_uint16(CF_KEY_UINT16_USB_PID, (uint16_t)new_pid);
#ifdef CONFIG_USE_USB_VENDOR
		cf_set_uint8(CF_KEY_UINT8_USB_PERSONALITY, new_mode);
#endif

		configdb_persist();
#endif	/* CONFIG_USE_CONFIGDB */
	}

#ifdef CONFIG_USE_CONFIGDB
	changed = changed || usb_common_state.cs_usb_mode != new_mode;
#endif

	if (changed) {
		fprintf(os, "You must reboot " PRODUCT_NAME " for the "
		    "change to take effect.\n");
	}
}
#endif /* defined(CONFIG_USE_CONFIGDB) || defined(CONFIG_USE_USB_VENDOR) */
#endif /* USB_FIXED_VENDOR_PRODUCT_IDS */

void
usb_interrupt(void *arg)
{

	(void) arg;
	tud_int_handler(BOARD_DEVICE_RHPORT_NUM);
}

void
usb_init(void)
{
	struct usb_common_state *cs = &usb_common_state;
	const serial_number_t *serial;

	serial = serial_number_fetch();
	assert(serial != NULL);
	strncpy(usb_serial_no, serial->cstr, sizeof(usb_serial_no));

	cs->cs_usb_mode = (unsigned int) usb_get_config(NULL, NULL);

	if (cs->cs_usb_mode != USB_PERSONALITY_DISABLED) {
		cs->cs_task = rtos_task_create("USB", usb_task_fn, cs,
		    RTOS_TASK_PRIORITY_MED, RTOS_STACK_SIZE(1300));
		assert(cs->cs_task != NULL);
	}

#ifndef USB_FIXED_VENDOR_PRODUCT_IDS
#if defined(CONFIG_USE_CONFIGDB) || defined(CONFIG_USE_USB_VENDOR)
	SHELL_CMD_ADD(usb);
#endif
#endif
}

bool
usb_cdc_enabled(void)
{

	return usb_common_state.cs_usb_mode == USB_PERSONALITY_CDC;
}

void
usb_load_defaults(void)
{

#ifndef CONFIG_USE_CONFIGDB
#ifdef CONFIG_USE_USB_VENDOR
	glob.glob_usb_personality = USB_PERSONALITY_CDC;
#endif
#else
	cf_del_uint8(CF_KEY_UINT8_USB_PERSONALITY);
	cf_del_uint16(CF_KEY_UINT16_USB_VID);
	cf_del_uint16(CF_KEY_UINT16_USB_PID);
#endif
}
