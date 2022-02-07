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

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "platform.h"
#include "platform_usb.h"

#define	CFG_TUSB_OS			OPT_OS_CUSTOM

#define	CFG_TUSB_MCU			PLATFORM_USB_MCU
#define BOARD_DEVICE_RHPORT_SPEED	PLATFORM_USB_SPEED
#define BOARD_DEVICE_RHPORT_NUM		PLATFORM_USB_PORT
#define CFG_TUSB_MEM_SECTION		PLATFORM_USB_RAM_SECTION
#define	CFG_TUSB_MEM_ALIGN		PLATFORM_USB_RAM_ALIGN

#if (BOARD_DEVICE_RHPORT_NUM == 0)
#define CFG_TUSB_RHPORT0_MODE	(OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif (BOARD_DEVICE_RHPORT_NUM == 1)
#define CFG_TUSB_RHPORT1_MODE	(OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
#error "Incorrect RHPort configuration"
#endif

#ifndef CFG_TUSB_DEBUG
#define	CFG_TUSB_DEBUG	0
#endif
#define	CFG_TUSB_DEBUG_PRINTF	debug_print

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define	CFG_TUD_ENDPOINT0_SIZE	64
#endif

#define	CFG_TUD_CDC		PLATFORM_USB_CDC_INSTANCE_NB
#define	CFG_TUD_MSC		0
#define	CFG_TUD_HID		0
#define	CFG_TUD_MIDI		0
#define	CFG_TUD_VENDOR		0

#ifndef CFG_TUD_CDC_RX_BUFSIZE
#define	CFG_TUD_CDC_RX_BUFSIZE	CFG_TUD_CDC_EP_BUFSIZE
#endif
#ifndef CFG_TUD_CDC_TX_BUFSIZE
#define	CFG_TUD_CDC_TX_BUFSIZE	CFG_TUD_CDC_EP_BUFSIZE
#endif

#endif /* TUSB_CONFIG_H */
