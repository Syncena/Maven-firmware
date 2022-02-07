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
 * Define these to override the default values of 0x0000/0x0000, which
 * will force USB to be disabled by default.
 */
//#define	USB_DEFAULT_DEVICE_VENDOR_ID	0x6666
//#define	USB_DEFAULT_DEVICE_PRODUCT_ID	0x0005

/*
 * Number of USB CDC instances supported by this platform.
 */
#define	PLATFORM_USB_CDC_INSTANCE_NB	2

/*
 * Other platform-specific USB definitions.
 */
//#define	PLATFORM_USB_MCU		OPT_MCU_LPC55XX
//#define	PLATFORM_USB_SPEED		OPT_MODE_HIGH_SPEED
//#define	PLATFORM_USB_PORT		1
//#define	PLATFORM_USB_RAM_SECTION	/* */
//#define	PLATFORM_USB_RAM_ALIGN		/* */

extern void template_usb_init(void);

#endif /* PLATFORM_USB_H */
