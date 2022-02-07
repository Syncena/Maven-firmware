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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#include "hardware.h"
#include "adiv5_ll.h"
#include "adiv5regs.h"
#include "tmon.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "ringbuff.h"
#include "platform.h"
#include "uart.h"
#include "target_comms.h"
#include "shell.h"
#include "timer.h"

uint32_t adiv5_ll_cfg_extra_set;
uint32_t adiv5_ll_cfg_extra_clr;
#ifdef CONFIG_USE_CONFIGDB
uint8_t adiv5_ll_config;
#endif

/*
 * Common Software Reset pin
 */
HW_PIN(nRST, EXT5)

SHELL_CMD_DECL(rstcfg, adiv5_ll_cmd_reset_config, "Configure nRST behaviour");

void
adiv5_ll_load_defaults(void)
{
	const uint8_t cfg = ADIV5_LL_NRST_DRIVER_OPEN_DRAIN;

	adiv5_ll_config = cfg;

#ifdef CONFIG_USE_CONFIGDB
	cf_set_uint8(CF_KEY_UINT8_HW_CONFIG, cfg);
#endif
}

bool
adiv5_ll_extended_reset_configured(void)
{

	return (adiv5_ll_config & ADIV5_LL_NRST_EXTENDED) != 0;
}

void
adiv5_ll_configure_reset(void)
{
	rtos_saved_ipl_t ipl;

	/* Atomic to minimise pin float time. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);

	if ((adiv5_ll_config & ADIV5_LL_NRST_DRIVER_MASK) ==
	    ADIV5_LL_NRST_DRIVER_PUSH_PULL ) {
		/* Assume deasserted */
		HW_PIN_nRST_set();

		/* Our GPIO pin to output */
		HW_PIN_nRST_out();
		/* External buffer to output */
		HW_PIN_nRST_DIR_set();
	} else {
		/* External buffer to input */
		HW_PIN_nRST_DIR_clr();
		HW_PIN_nRST_in();

		HW_PIN_nRST_pullup(1);
		HW_PIN_nRST_clr();
	}

	rtos_ipl_restore(ipl);
}

int
adiv5_ll_nRST_state(void)
{

	return HW_PIN_nRST_read() != 0;
}

void
adiv5_ll_nRST_assert(void)
{

	/* Assert RST */
	HW_PIN_nRST_clr();

	if (((adiv5_ll_config | adiv5_ll_cfg_extra_set) &
	    ADIV5_LL_NRST_DRIVER_MASK) == ADIV5_LL_NRST_DRIVER_OPEN_DRAIN ) {
		rtos_saved_ipl_t ipl;

		/* Atomic to minimise contention time. */
		ipl = rtos_ipl_raise(HW_IPL_MAX);
		/* Our GPIO pin to output */
		HW_PIN_nRST_out();
		/* External buffer to output */
		HW_PIN_nRST_DIR_set();
		rtos_ipl_restore(ipl);
	}
}

void
adiv5_ll_nRST_deassert(void)
{
	timer_timeout_t to;

	HW_PIN_nRST_set();

	if (((adiv5_ll_config | adiv5_ll_cfg_extra_set) &
	    ADIV5_LL_NRST_DRIVER_MASK) == ADIV5_LL_NRST_DRIVER_OPEN_DRAIN ) {
		rtos_saved_ipl_t ipl;

		/* Atomic to minimise pin float time. */
		ipl = rtos_ipl_raise(HW_IPL_MAX);
		/* External buffer to input */
		HW_PIN_nRST_DIR_clr();
		/* Our GPIO pin to input */
		HW_PIN_nRST_in();
		rtos_ipl_restore(ipl);

		/*
		 * Now wait for nRST to read back as 'high'.
		 * The timeout is simply to prevent spinning here
		 * forever if the target hardware is messed up.
		 */
		timer_timeout_start(&to, 500u);
		while (!timer_timeout_expired(&to) && HW_PIN_nRST_read() == 0)
			rtos_task_sleep(1);

		if (HW_PIN_nRST_read() == 0) {
			tmon_log("adiv5_ll_nRST_deassert: Open-drain nRST "
			    "failed to float high within 500mS");
		}
	}

	/*
	 * Some weird targets (Sony SPRSENSE, for example) tie nRST to a
	 * power-enable pin. We need to wait a bit longer to ensure power
	 * is stable before proceeding.
	 */
	rtos_task_sleep(200);
}

void
adiv5_ll_nRST_toggle(void)
{

	adiv5_ll_nRST_assert();

	/*
	 * Keep it asserted for a period of time.
	 * XXX: Make this configurable.
	 */
	timer_delay_cycles(1000);

	adiv5_ll_nRST_deassert();
}

void
adiv5_ll_init_target(void)
{

	/*
	 * The external buffers have already been set to input mode
	 * by our caller.
	 */

#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_uint8(CF_KEY_UINT8_HW_CONFIG, &adiv5_ll_config) == 0) {
		adiv5_ll_load_defaults();
		cf_get_uint8(CF_KEY_UINT8_HW_CONFIG, &adiv5_ll_config);
	}
#endif

	adiv5_ll_configure_reset();

	platform_hw_init_target();

	SHELL_CMD_ADD(rstcfg);
}

static const char adiv5_ll_cmd_reset_usage[] =
"usage:\n"
"rstcfg driver <option>\n"
"\tPossible values for 'option' are:\n"
"\t  opendrain   nRST driver is open-drain (default).\n"
"\t  pushpull    nRST driver is push-pull.\n"
"rstcfg extended <option>\n"
"\tPossible values for 'option' are:\n"
"\t  on          Perform Extended Reset sequence during attach.\n"
"\t  off         Normal reset sequence during attach (default).\n"
"\tThis option is useful for certain SoCs, most notably those from\n"
"\tMicrochip/Atmel. You will not normally need to enable this feature\n"
"\tas Maven performs an Extended Reset on its own where it is deemed\n"
"\tnecessary. However, if your SoC has been programmed with a bad\n"
"\tfirmware image such that Maven fails to attach in the normal way\n"
"\t(usually because the firmware has messed up clock configuration),\n"
"\tthen enabling this manually will usually allow Maven to gain access\n"
"\tto the DSU so that the device can be erased with an 'unlock' command.\n"
;

static void
adiv5_ll_cmd_reset_config(FILE *os, uint8_t argc, const char * const *argv)
{
	size_t arglen;
	uint8_t v;

	if (argc) {
		if (argc != 2) {
 usage:
			fputs(adiv5_ll_cmd_reset_usage, os);
			return;
		}

		arglen = strlen(argv[0]);
		if (strncasecmp("driver", argv[0], arglen) == 0) {
			arglen = strlen(argv[1]);
			if (strncasecmp("opendrain", argv[1], arglen) == 0)
				v = ADIV5_LL_NRST_DRIVER_OPEN_DRAIN;
			else
			if (strncasecmp("pushpull", argv[1], arglen) == 0)
				v = ADIV5_LL_NRST_DRIVER_PUSH_PULL;
			else
				goto usage;

			if ((adiv5_ll_config & ADIV5_LL_NRST_DRIVER_MASK) != v) {
				adiv5_ll_config &= ~ADIV5_LL_NRST_DRIVER_MASK;
				adiv5_ll_config |= v;
				adiv5_ll_configure_reset();
#ifndef CONFIG_USE_CONFIGDB
				glob_save();
#else
				cf_set_uint8(CF_KEY_UINT8_HW_CONFIG, adiv5_ll_config);
				configdb_persist();
#endif
			}
		} else
		if (strncasecmp("extended", argv[0], arglen) == 0) {
			if (strcasecmp("off",argv[1]) == 0 || argv[1][0] == '0')
				v = 0;
			else
			if (strcasecmp("on", argv[1]) == 0 || argv[1][0] == '1')
				v = ADIV5_LL_NRST_EXTENDED;
			else
				goto usage;

			if ((adiv5_ll_config & ADIV5_LL_NRST_EXTENDED) != v) {
				adiv5_ll_config &= ~ADIV5_LL_NRST_EXTENDED;
				adiv5_ll_config |= v;
#ifndef CONFIG_USE_CONFIGDB
				glob_save();
#else
				cf_set_uint8(CF_KEY_UINT8_HW_CONFIG, adiv5_ll_config);
				configdb_persist();
#endif
			}
		} else
			goto usage;
	}

	fprintf(os, "nRST pin driver is: ");
	switch (adiv5_ll_config & ADIV5_LL_NRST_DRIVER_MASK) {
	case ADIV5_LL_NRST_DRIVER_OPEN_DRAIN:
		fputs("Open-drain\n", os);
		break;
	case ADIV5_LL_NRST_DRIVER_PUSH_PULL:
		fputs("Push-pull\n", os);
		break;
	}

	fprintf(os, "Reset Extension is: %sabled\n",
	    (adiv5_ll_config & ADIV5_LL_NRST_EXTENDED) ? "En" : "Dis");
}
