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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rtos.h"
#include "display.h"
#include "usb_config.h"
#include "udc.h"
#include "udi_vendor.h"
#include "usb_protocol_vendor.h"
#include "usb_common.h"

#if (RELEASE_BUILD == 0)
//#define	USB_DEBUG	1
#endif

struct usb_vendor_state {
	usb_vendor_status_callback_t uvs_status_cb;
	void *uvs_status_cb_arg;
	uint32_t uvs_state;
#define	USB_VENDOR_STATE_OPEN		(1u << 0)
#define	USB_VENDOR_STATE_ENABLED	(1u << 1)
	uint16_t *uvs_rx_lenp;
	void *uvs_common_state;
};
static struct usb_vendor_state usb_vendor_state;

/*
 * ==========================================================================
 *                     USB Descriptors for AVRISP-MKII
 * ==========================================================================
 */

/* Don't mark this as const - it must be in SRAM for USB DMA to work. */
static ALIGNED(16) usb_dev_desc_t udc_device_desc_avrisp = {
	.bLength		= sizeof(usb_dev_desc_t),
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB			= LE16(USB_V2_0),
	.bDeviceClass		= CLASS_VENDOR_SPECIFIC,
	.bDeviceSubClass	= NO_SUBCLASS,
	.bDeviceProtocol	= NO_PROTOCOL,
	.bMaxPacketSize0	= USB_DEVICE_EP_CTRL_SIZE,
	.idVendor		= LE16(USB_AVRISP_DEVICE_VENDOR_ID),
	.idProduct		= LE16(USB_AVRISP_DEVICE_PRODUCT_ID),
	.bcdDevice		= LE16((USB_AVRISP_DEVICE_MAJOR_VERSION << 8) |
					USB_AVRISP_DEVICE_MINOR_VERSION),
	.iManufacturer		= 1,
	.iProduct		= 2,
	.iSerialNumber		= 3,
	.bNumConfigurations	= 1
};

typedef struct {
	usb_conf_desc_t		conf;
	udi_vendor_desc_t	udi_vendor;
} udc_desc_avrisp_t PACKED;

#define	USB_DEVICE_NB_INTERFACE_AVRISP	1

/* Don't mark this as const - it must be in SRAM for USB DMA to work. */
static ALIGNED(16) udc_desc_avrisp_t udc_desc_fs_avrisp = {
	.conf.bLength		= sizeof(usb_conf_desc_t),
	.conf.bDescriptorType	= USB_DT_CONFIGURATION,
	.conf.wTotalLength	= LE16(sizeof(udc_desc_avrisp_t)),
	.conf.bNumInterfaces	= USB_DEVICE_NB_INTERFACE_AVRISP,
	.conf.bConfigurationValue = 1,
	.conf.iConfiguration	= 0,
	.conf.bmAttributes	= USB_CONFIG_ATTR_MUST_SET | USB_DEVICE_ATTR,
	.conf.bMaxPower		= USB_CONFIG_MAX_POWER(USB_DEVICE_POWER),
	.udi_vendor		= UDI_VENDOR_DESC_FS,
};

static const udi_api_t * const udi_apis_avrisp[USB_DEVICE_NB_INTERFACE_AVRISP] = {
	&udi_api_vendor,
};

static const udc_config_speed_t udc_config_fs_avrisp = {
	.desc		= &udc_desc_fs_avrisp.conf,
	.udi_apis	= udi_apis_avrisp,
};

static const udc_config_t udc_config_avrisp = {
	.confdev_lsfs	= (usb_dev_desc_t *)(uintptr_t)&udc_device_desc_avrisp,
	.conf_lsfs	= (udc_config_speed_t *)(uintptr_t)&udc_config_fs_avrisp,
};

#ifdef USB_DEBUG
#define DBPRINTF(fmt, ...)      do {    \
	printf(fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define DBPRINTF(fmt, ...)      do { } while (0)
#endif /* USB_DEBUG */

void *
usb_vendor_open(usb_vendor_status_callback_t cb, void *arg)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	if (uvs->uvs_common_state == NULL) {
		DBPRINTF("usb_vendor_open: not in vendor mode\n");
		return NULL;
	}

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) != 0) {
		DBPRINTF("usb_vendor_open: already open\n");
		return NULL;
	}

	uvs->uvs_rx_lenp = NULL;
	uvs->uvs_status_cb_arg = arg;
	uvs->uvs_status_cb = cb;
	uvs->uvs_state |= USB_VENDOR_STATE_OPEN;

	DBPRINTF("usb_vendor_open: open.\n");

	return uvs;
}

void
usb_vendor_close(void *arg)
{
	struct usb_vendor_state *uvs = arg;

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) != 0) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_USB);
		uvs->uvs_state &= ~USB_VENDOR_STATE_OPEN;
		rtos_ipl_restore(ipl);

		uvs->uvs_rx_lenp = NULL;
		uvs->uvs_status_cb_arg = NULL;
		uvs->uvs_status_cb = NULL;
		DBPRINTF("usb_vendor_close: closed.\n");
	} else {
		DBPRINTF("usb_vendor_close: already closed.\n");
	}
}

static void
usb_vendor_out_received(udd_ep_status_t status,
    iram_size_t nb_transferred, udd_ep_id_t ep)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	(void) ep;

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) == 0 ||
	    uvs->uvs_rx_lenp == NULL || uvs->uvs_status_cb == NULL) {
		DBPRINTF("usb_vendor_out_received: ignoring.\n");
		return;
	}

	DBPRINTF("usb_vendor_out_received: nb_transferred %u, status %u.\n",
	    (unsigned int)nb_transferred, status);

	*(uvs->uvs_rx_lenp) = (uint16_t) nb_transferred;

	(uvs->uvs_status_cb)((status == UDD_EP_TRANSFER_OK) ?
	    USB_VENDOR_STATUS_RX_READY : USB_VENDOR_STATUS_RX_ERROR,
	    uvs->uvs_status_cb_arg);
}

void
usb_vendor_post_read(void *arg, void *buff, uint16_t *lenp, uint16_t len)
{
	struct usb_vendor_state *uvs = arg;

	display_activity(DISPLAY_ACTIVITY_USB);

	uvs->uvs_rx_lenp = lenp;

	udi_vendor_bulk_out_run(buff, len, usb_vendor_out_received);
}

static void
usb_vendor_in_done(udd_ep_status_t status,
    iram_size_t nb_transferred, udd_ep_id_t ep)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	(void) ep;
	(void) nb_transferred;

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) == 0 ||
	    uvs->uvs_status_cb == NULL) {
		DBPRINTF("usb_vendor_in_done: ignoring\n");
		return;
	}

	DBPRINTF("usb_vendor_in_done: status %u.\n", status);

	(uvs->uvs_status_cb)((status == UDD_EP_TRANSFER_OK) ?
	    USB_VENDOR_STATUS_TX_DONE : USB_VENDOR_STATUS_TX_ERROR,
	    uvs->uvs_status_cb_arg);
}

void
usb_vendor_post_write(void *arg, void *buff, uint16_t len)
{

	(void) arg;

	display_activity(DISPLAY_ACTIVITY_USB);

	DBPRINTF("usb_vendor_post_write: buff len %u.\n", len);

	udi_vendor_bulk_in_run(buff, len, usb_vendor_in_done);
}

bool
usb_vendor_enable(void)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	uvs->uvs_state |= USB_VENDOR_STATE_ENABLED;

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) != 0 &&
	    uvs->uvs_status_cb != NULL) {
		DBPRINTF("usb_vendor_enable: posting enable status\n");
		(uvs->uvs_status_cb)(USB_VENDOR_STATUS_ENABLE,
		    uvs->uvs_status_cb_arg);
	} else {
		DBPRINTF("usb_vendor_enable: ignoring\n");
	}

	return true;
}

void
usb_vendor_disable(void)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	uvs->uvs_state &= ~USB_VENDOR_STATE_ENABLED;

	if ((uvs->uvs_state & USB_VENDOR_STATE_OPEN) != 0 &&
	    uvs->uvs_status_cb != NULL) {
		DBPRINTF("usb_vendor_disable: posting disable status\n");
		(uvs->uvs_status_cb)(USB_VENDOR_STATUS_DISABLE,
		    uvs->uvs_status_cb_arg);
	} else {
		DBPRINTF("usb_vendor_disable: ignoring\n");
	}
}

bool
usb_vendor_setup_out_received(void)
{

	DBPRINTF("usb_vendor_setup_out_received:\n");

	return true;
}

bool
usb_vendor_setup_in_received(void)
{

	DBPRINTF("usb_vendor_setup_in_received:\n");

	return true;
}

void
usb_vendor_workloop(void *arg)
{
	/* Nothing to do */
	(void)arg;
}

const void *usb_vendor_attach(void *common_state, void **vendor_state)
{
	struct usb_vendor_state *uvs = &usb_vendor_state;

	memset(uvs, 0, sizeof(*uvs));

	uvs->uvs_common_state = common_state;
	*vendor_state = uvs;
	return &udc_config_avrisp;
}
