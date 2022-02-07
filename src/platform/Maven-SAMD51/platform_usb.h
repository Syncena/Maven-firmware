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

#ifndef PLATFORM_USB_H
#define PLATFORM_USB_H

/*
 * SAM D5x/E5x USB controller has bi-directional endpoints, so the Rx and
 * notification pipes can share an endpoint. This means we can have up to
 * three CDC ports.
 */
#define	PLATFORM_USB_CDC_INSTANCE_NB	3

#ifdef CONFIG_USE_ATMEL_USB
#define	PLATFORM_USB_DEVICE_MAX_EP	7
#define	UDI_CDC_DATA_EP_IN_0		(1 | USB_EP_DIR_IN)
#define	UDI_CDC_DATA_EP_OUT_0		(2 | USB_EP_DIR_OUT)
#define	UDI_CDC_COMM_EP_0		(2 | USB_EP_DIR_IN)
#define	UDI_CDC_DATA_EP_IN_1		(3 | USB_EP_DIR_IN)
#define	UDI_CDC_DATA_EP_OUT_1		(4 | USB_EP_DIR_OUT)
#define	UDI_CDC_COMM_EP_1		(4 | USB_EP_DIR_IN)
#define	UDI_CDC_DATA_EP_IN_2		(5 | USB_EP_DIR_IN)
#define	UDI_CDC_DATA_EP_OUT_2		(6 | USB_EP_DIR_OUT)
#define	UDI_CDC_COMM_EP_2		(6 | USB_EP_DIR_IN)

/*
 * The following numbers are chosen to match the STK600 endpoints.
 */
#define	UDI_VENDOR_EP_BULK_IN		(3 | USB_EP_DIR_IN)
#define	UDI_VENDOR_EP_BULK_OUT		(2 | USB_EP_DIR_OUT)

#elif defined(CONFIG_USE_TINYUSB)
#define	PLATFORM_USB_MCU		OPT_MCU_SAMD51
#define	PLATFORM_USB_SPEED		OPT_MODE_FULL_SPEED
#define	PLATFORM_USB_PORT		0
#define	PLATFORM_USB_RAM_SECTION	/* */
#define	PLATFORM_USB_RAM_ALIGN		__attribute__((aligned(8)))
#else
#error "USB driver undefined."
#endif

extern uint16_t sam_usb_get_padcal(void);
extern void sam_usb_platform_init(void);
extern void sam_usb_platform_deinit(void);

#endif /* PLATFORM_USB_H */
