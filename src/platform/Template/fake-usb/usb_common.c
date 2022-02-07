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
#include "usb_cdc.h"
#include "configdb.h"

SHELL_CMD_DECL(usb, usb_cmd, "Configure USB");

char usb_serial_no[USB_DEVICE_GET_SERIAL_NAME_LENGTH];

static unsigned int usb_mode;

uint8_t
usb_get_config(uint16_t *pvid, uint16_t *ppid)
{
	uint16_t vid;
	uint8_t mode;

	/*
	 * Fetch the configuration from persistant storage.
	 */
	/* USB CDC is contingent on non-zero VID:PID on later product. */
	if (cf_get_uint16(CF_KEY_UINT16_USB_VID, &vid) != sizeof(vid))
		vid = USB_DEFAULT_DEVICE_VENDOR_ID;
	if (ppid != NULL &&
	    cf_get_uint16(CF_KEY_UINT16_USB_PID, ppid) != sizeof(*ppid)) {
		*ppid = USB_DEFAULT_DEVICE_PRODUCT_ID;
	}

	mode = USB_PERSONALITY_DISABLED;

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
	const void *cfg = NULL;
	void *foo;

	(void) arg;

	cfg = usb_cdc_attach(NULL, &foo);
	assert(cfg != NULL);
	(void) cfg;

	rtos_task_sleep(500);

	for (;;) {
		rtos_notify_take(1);
	}
}

static const char usb_cmd_usage_string[] = {
"Usage:\n"
"usb\n\tDisplay current USB settings.\n"
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
};

static void
usb_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	size_t l = argc ? strlen(argv[0]) : 0;
	unsigned int new_vid, new_pid;
	uint16_t vid, pid;
	uint8_t mode, new_mode;
	bool changed;

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
	if (argc != 0) {
		fprintf(os, "%s", usb_cmd_usage_string);
		return;
	}

	changed = false;

	fprintf(os, "USB Virtual COM support %s %sabled. VID:PID = %04X:%04x\n",
	    (usb_mode != new_mode) ? "will be" : "is",
	    (new_pid == 0) ? "dis" : "en", new_vid, new_pid);

	changed = changed || new_vid != vid || new_pid != pid;

	if (changed) {
		cf_set_uint16(CF_KEY_UINT16_USB_VID, (uint16_t)new_vid);
		cf_set_uint16(CF_KEY_UINT16_USB_PID, (uint16_t)new_pid);

		configdb_persist();
	}

	changed = changed || usb_mode != new_mode;

	if (changed) {
		fprintf(os, "You must reboot " PRODUCT_NAME " for the "
		    "change to take effect.\n");
	}
}

void
usb_init(void)
{
	const serial_number_t *serial;

	serial = serial_number_fetch();
	assert(serial != NULL);
	strncpy(usb_serial_no, serial->cstr, sizeof(usb_serial_no));

	usb_mode = (unsigned int) usb_get_config(NULL, NULL);

	if (usb_mode != USB_PERSONALITY_DISABLED) {
		rtos_task_t t;
		t = rtos_task_create("USB", usb_task_fn, NULL,
		    RTOS_TASK_PRIORITY_MED + 1, RTOS_STACK_SIZE(1300));
		assert(t != NULL);
		(void) t;
	}

	SHELL_CMD_ADD(usb);
}

bool
usb_cdc_enabled(void)
{

	return usb_mode == USB_PERSONALITY_CDC;
}

void
usb_load_defaults(void)
{

	cf_del_uint8(CF_KEY_UINT8_USB_PERSONALITY);
	cf_del_uint16(CF_KEY_UINT16_USB_VID);
	cf_del_uint16(CF_KEY_UINT16_USB_PID);
}
