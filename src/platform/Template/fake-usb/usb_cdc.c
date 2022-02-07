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

#include "platform.h"
#include "product.h"
#include "uart.h"
#include "usb_common.h"
#include "usb_cdc.h"

//#define DEBUG_FLAG_INIT	1
#include "debug.h"
#include "hexdump.h"

static uint32_t dummy_value;

void *
usb_cdc_claim(uint8_t instance, usb_cdc_status_callback_t cb, void *arg)
{

	(void) instance;
	(void) cb;
	(void) arg;
	return &dummy_value;
}

void
usb_cdc_unclaim(void *arg)
{

	(void) arg;
	assert(arg == &dummy_value);
}

int
usb_cdc_open(void *arg, uint16_t cfg, uint32_t custom_baud, ringbuff_t to_usb,
    ringbuff_t from_usb)
{

	(void) arg;
	(void) cfg;
	(void) custom_baud;
	(void) to_usb;
	(void) from_usb;

	return -1;
}

void
usb_cdc_close(void *arg)
{

	(void) arg;
}

uint16_t
usb_cdc_get_config(void *arg, uint32_t *custom_baud)
{

	(void) arg;
	(void) custom_baud;
	return UART_DEFAULT_CFG;
}

const void *
usb_cdc_attach(void *common_state, void **cdc_state)
{

	(void) common_state;
	*cdc_state = &dummy_value;
	return &dummy_value;
}
