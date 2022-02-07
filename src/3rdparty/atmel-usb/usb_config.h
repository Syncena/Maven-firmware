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

#ifndef USB_CONFIG_H
#define USB_CONFIG_H

#include "platform.h"
#include "platform_usb.h"
#include "serial_number.h"
#include "product.h"

#define	UDD_NO_SLEEP_MGR	1

#define	USB_CDC_DEVICE_VENDOR_ID	0x6666
#define	USB_CDC_DEVICE_PRODUCT_ID	0x0005
#define	USB_CDC_DEVICE_MAJOR_VERSION	2
#define	USB_CDC_DEVICE_MINOR_VERSION	0

#define	USB_AVRISP_DEVICE_VENDOR_ID	0x03EB
#define	USB_AVRISP_DEVICE_PRODUCT_ID	0x2106
#define	USB_AVRISP_DEVICE_MAJOR_VERSION	2
#define	USB_AVRISP_DEVICE_MINOR_VERSION	0

/* Appease ASF. Dumbasses */
#define	USB_DEVICE_VENDOR_ID	USB_CDC_DEVICE_VENDOR_ID
#define	USB_DEVICE_PRODUCT_ID	USB_CDC_DEVICE_PRODUCT_ID

#define	USB_DEVICE_POWER		500
#define	USB_DEVICE_ATTR			USB_CONFIG_ATTR_BUS_POWERED
/* Note: determine this at run-time */
//	(USB_CONFIG_ATTR_SELF_POWERED)

#define	USB_DEVICE_MANUFACTURE_NAME	"Maverick Embedded Technology"
#define	USB_DEVICE_PRODUCT_NAME		PRODUCT_NAME_USB
extern char usb_serial_no[];
#define	USB_DEVICE_SERIAL_NAME
#define	USB_DEVICE_GET_SERIAL_NAME_POINTER	((void *)(&usb_serial_no[0]))
#define	USB_DEVICE_GET_SERIAL_NAME_LENGTH	sizeof(((serial_number_t *)0ul)->cstr)

/*
 * USB Device Callback definitions
 */
#define	UDC_VBUS_EVENT(b_vbus_high)	/* Not interesting */
#define	UDC_SOF_EVENT()			/* Not interesting */
#define	UDC_SUSPEND_EVENT()		/* Not interesting */
#define	UDC_RESUME_EVENT()		/* Not interesting */

#define	USB_DEVICE_MAX_EP		PLATFORM_USB_DEVICE_MAX_EP

#ifdef CONFIG_USE_USB_VENDOR
/*
 * Vendor-specific definitions
 */
#define	UDI_VENDOR_EPS_SIZE_INT_FS	0
#define	UDI_VENDOR_EPS_SIZE_BULK_FS	64
#define	UDI_VENDOR_EPS_SIZE_ISO_FS	0

#define UDI_VENDOR_ENABLE_EXT()           usb_vendor_enable()
#define UDI_VENDOR_DISABLE_EXT()          usb_vendor_disable()
#define UDI_VENDOR_SETUP_OUT_RECEIVED()   usb_vendor_setup_out_received()
#define UDI_VENDOR_SETUP_IN_RECEIVED()    usb_vendor_setup_in_received()
#include "udi_vendor_conf.h"
#endif /* CONFIG_USE_USB_VENDOR */

/*
 * CDC Class Definitions.
 *
 * Define 2 USB communication ports:
 *  0 - Target
 *  1 - GDB #0
 *  2 - GDB #1
 */
#define	UDI_CDC_PORT_NB			PLATFORM_USB_CDC_INSTANCE_NB

/* CDC Interface callback definitions */
#define	UDI_CDC_ENABLE_EXT(port)	usb_cdc_enable(port)
#define	UDI_CDC_DISABLE_EXT(port)	usb_cdc_disable(port)
#define	UDI_CDC_RX_NOTIFY(port)		usb_cdc_rx_notify(port)
#define	UDI_CDC_TX_EMPTY_NOTIFY(port)	usb_cdc_tx_empty_notify(port)
#define	UDI_CDC_SET_CODING_EXT(port,cfg) usb_cdc_set_coding(port,cfg)
#define	UDI_CDC_SET_DTR_EXT(port,set)	usb_cdc_set_dtr(port,set)
#define	UDI_CDC_SET_RTS_EXT(port,set)	/* Not interesting */
#define	UDI_CDC_SET_BREAK_EXT(port,set)	usb_cdc_set_break(port,set)

#define	UDI_CDC_LOW_RATE		/* UARTs are low speed */

#define	UDI_CDC_DEFAULT_RATE		38400
#define	UDI_CDC_DEFAULT_STOPBITS	CDC_STOP_BITS_1
#define	UDI_CDC_DEFAULT_PARITY		CDC_PAR_NONE
#define	UDI_CDC_DEFAULT_DATABITS	8
#include "udi_cdc_conf.h"

#endif /* USB_CONFIG_H */
