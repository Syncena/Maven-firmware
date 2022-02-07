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
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#if defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
#include "wifi_driver.h"
#endif
#include "platform.h"
#include "serial_number.h"
#include "sys_event.h"
#include "shell.h"
#include "product.h"
#include "port_daemon.h"
#include "target_comms.h"
#ifdef CONFIG_USE_USB
#include "usb_common.h"
#endif
#ifdef CONFIG_USE_DISPLAY
#include "display.h"
#endif
#include "console.h"
#include "hardware.h"
#include "version.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_LWIP
#include "lwip/opt.h"
#include "lwip/init.h"
#ifdef NETWORK_OPT_HTTPD
#include "network_httpd.h"
#endif
#endif
#endif /* CONFIG_USE_NETWORK */

#ifdef CONFIG_USE_CONFIGDB
static void load_default_settings(void);
#endif
static rtos_task_t product_task_handle;

static void
do_reboot(int to_defaults, int to_bootloader)
{

	(void) to_bootloader;

	if (to_defaults) {
#ifndef CONFIG_USE_CONFIGDB
		platform_wipe_globals();
#else
		load_default_settings();
		configdb_persist();
#endif /* CONFIG_USE_CONFIGDB */
	}

	platform_reboot();
	for (;;);
}

static void
main_init(void *arg)
{
	ringbuff_t rb;

	(void) arg;

#ifndef CONFIG_USE_CONFIGDB
	/* Global vars */
	(void) glob_init();

#else /* CONFIG_USE_CONFIGDB */

	/* Config DB */
	platform_attach_configdb();
	if (configdb_init() == 0)
		load_default_settings();
#endif	/* CONFIG_USE_CONFIGDB */

	/*
	 * A console would be nice
	 */
	rb = console_init("\n" PRODUCT_NAME " Serial Console\n");

#if 0
{
	extern void platform_benchmark(void);
	platform_benchmark();
}
#endif

#ifdef CONFIG_USE_NETWORK
	/* Bring up networking */
#ifdef NETWORK_LWIP
	lwip_init();

#if defined(NETWORK_OPT_HTTPD)
	network_httpd_init();
#endif
#endif
#endif /* CONFIG_USE_NETWORK */

	/* Platform-specific start-up. */
	platform_attach_devices();

	/* Start the power supply task */
	power_init();

	/* Start the port daemon */
	port_daemon_init();

	/* Start the programmer/GDB server */
	product_init();

	/* Get the shell running */
	shell_init(rb, do_reboot);
	app_shell_init();

#ifdef CONFIG_USE_USB
	/* USB perchance? */
	usb_init();
#endif

	product_task(product_task_handle);

	/*NOTREACHED*/
}

int
main(void)
{

	platform_watchdog_disable();

	/*
	 * Fetch and validate our serial number.
	 */
	serial_number_init();

	/*
	 * Initialise clocks, power and cache
	 */
	platform_init();

	/*
	 * Get the rtos up and running
	 */
	rtos_init();

	/* Bring up the system event API. */
	sys_event_init();

	/*
	 * Create the main application task and continue initialisation there.
	 */
	product_task_handle = rtos_task_create(PRODUCT_TASK_NAME,
	    main_init, NULL, RTOS_TASK_PRIORITY_MAX,
	    RTOS_STACK_SIZE(PRODUCT_TASK_STACK_LEN));
	assert(product_task_handle != NULL);

	/*
	 * Enter the scheduler, never to return. Initialisation will
	 * continue in task mode.
	 */
	rtos_scheduler_start();

	/*
	 * Appease gcc, which assumes main() returns 'int' - pretty daft
	 * in this kind of environment...
	 */
	return 0;
}

#ifndef CONFIG_USE_CONFIGDB
void
glob_app_defaults(uint8_t which)
{

	if (which == 0) {
#ifdef CONFIG_USE_NETWORK
		network_load_defaults();
#if defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
		wifi_driver_load_defaults();
#endif
#endif
#ifdef CONFIG_USE_DISPLAY
		display_load_defaults();
#endif
	} else {
		product_load_defaults();
#ifdef CONFIG_USE_USB
		usb_load_defaults();
#endif
		target_comms_load_defaults();
	}
}
#else
static void
load_default_settings(void)
{

#ifdef CONFIG_USE_NETWORK
	network_load_defaults();
#if defined(NETWORK_DEV_WINC) || defined(NETWORK_DEV_WILC)
	wifi_driver_load_defaults();
#endif
#endif /* CONFIG_USE_NETWORK */

#ifdef CONFIG_USE_DISPLAY
	display_load_defaults();
#endif
	product_load_defaults();
#ifdef CONFIG_USE_USB
	usb_load_defaults();
#endif
	target_comms_load_defaults();
}
#endif /* CONFIG_USE_CONFIGDB */
