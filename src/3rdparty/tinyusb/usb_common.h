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

#ifndef USB_COMMON_H
#define USB_COMMON_H

#include <stdbool.h>

#include "serial_number.h"

#define USB_DEVICE_GET_SERIAL_NAME_POINTER	((void *)(&usb_serial_no[0]))
#define USB_DEVICE_GET_SERIAL_NAME_LENGTH	sizeof(((serial_number_t *)0ul)->cstr)
extern char usb_serial_no[];

extern void usb_init(void);
extern void usb_interrupt(void *) FAST_RAMFUNC;
extern void usb_load_defaults(void);
extern void usb_common_wakeup(void *);
extern bool usb_cdc_enabled(void);
extern uint8_t usb_get_config(uint16_t *pvid, uint16_t *ppid);

#define	USB_PERSONALITY_VENDOR		0
#define	USB_PERSONALITY_CDC		1
#define	USB_PERSONALITY_DISABLED	2

#ifndef CONFIG_USE_CONFIGDB
#ifdef CONFIG_USE_USB_VENDOR
#define	USB_GLOBAL_VARS		uint8_t glob_usb_personality;
#else
#define	USB_GLOBAL_VARS		/* Nothing */
#endif
#endif	/* CONFIG_USE_CONFIGDB */

#endif /* USB_COMMON_H */
