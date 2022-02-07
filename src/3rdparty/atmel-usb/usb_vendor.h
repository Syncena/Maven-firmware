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

#ifndef USB_VENDOR_H
#define USB_VENDOR_H

typedef enum {
	USB_VENDOR_STATUS_ENABLE,
	USB_VENDOR_STATUS_DISABLE,
	USB_VENDOR_STATUS_RX_READY,
	USB_VENDOR_STATUS_RX_ERROR,
	USB_VENDOR_STATUS_TX_DONE,
	USB_VENDOR_STATUS_TX_ERROR,
} usb_vendor_status_t;

typedef void (*usb_vendor_status_callback_t)(usb_vendor_status_t, void *);

extern const void *usb_vendor_attach(void *, void **);
extern void usb_vendor_workloop(void *);

extern void *usb_vendor_open(usb_vendor_status_callback_t, void *);
extern void usb_vendor_close(void *);
extern void usb_vendor_post_read(void *, void *, uint16_t *, uint16_t);
extern void usb_vendor_post_write(void *, void *, uint16_t);

extern bool usb_vendor_enable(void);
extern void usb_vendor_disable(void);
extern bool usb_vendor_setup_out_received(void);
extern bool usb_vendor_setup_in_received(void);

#endif /* USB_VENDOR_H */
