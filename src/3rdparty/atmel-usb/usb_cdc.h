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

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdbool.h>
#include "ringbuff.h"

#include "platform_usb.h"

typedef enum {
	USB_CDC_STATUS_ATTACHED,
	USB_CDC_STATUS_DETACHED,
	USB_CDC_STATUS_CONNECTED,
	USB_CDC_STATUS_HANGUP,
	USB_CDC_STATUS_SEND_BREAK,
	USB_CDC_STATUS_CONFIG
} usb_cdc_status_t;

typedef void (*usb_cdc_status_callback_t)(usb_cdc_status_t, void *);

extern const void *usb_cdc_attach(void *, void **);
extern void usb_cdc_workloop(void *);

extern void *usb_cdc_claim(uint8_t, usb_cdc_status_callback_t, void *);
extern void usb_cdc_unclaim(void *);
extern int usb_cdc_open(void *, uint16_t, uint32_t, ringbuff_t, ringbuff_t);
extern void usb_cdc_close(void *);
extern void usb_cdc_set_config(void *, uint16_t, uint32_t);
extern uint16_t usb_cdc_get_config(void *, uint32_t *);

#ifdef _USB_PROTOCOL_CDC_H_
extern uint8_t usb_cdc_enable(uint8_t);
extern void usb_cdc_disable(uint8_t);
extern void usb_cdc_rx_notify(uint8_t);
extern void usb_cdc_tx_empty_notify(uint8_t);
extern void usb_cdc_set_dtr(uint8_t, uint8_t);
extern void usb_cdc_set_break(uint8_t, uint8_t);
extern void usb_cdc_set_coding(uint8_t, usb_cdc_line_coding_t *);
#endif

#endif /* USB_CDC_H */
