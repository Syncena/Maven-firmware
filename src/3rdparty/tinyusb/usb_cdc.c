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
#include "tusb.h"
#include "product.h"
#include "uart.h"
#include "usb_common.h"
#include "tusb.h"
#include "usb_cdc.h"
/* XXX: Needed for usbd_defer_func() prototype. */
#include "device/usbd_pvt.h"

#ifdef CONFIG_USE_DISPLAY
#include "display.h"
#endif

//#define DEBUG_FLAG_INIT	1
#include "debug.h"
#include "hexdump.h"

#ifndef USB_CDC_VENDOR_STRING
#define	USB_CDC_VENDOR_STRING	"Maverick Embedded Technology"
#endif
#ifndef USB_CDC_PRODUCT_STRING
#define	USB_CDC_PRODUCT_STRING	PRODUCT_NAME_USB
#endif

struct usb_cdc_state {
	uint8_t ucs_instance;

	volatile uint8_t ucs_state;
#define	USB_CDC_STATE_CLAIMED		(1u << 0)
#define	USB_CDC_STATE_ATTACHED		(1u << 1)
#define	USB_CDC_STATE_OPEN		(1u << 2)
#define	USB_CDC_STATE_DTR_SET		(1u << 3)
#define	USB_CDC_STATE_RX_READY		(1u << 4)
#define	USB_CDC_STATE_TX_READY		(1u << 5)
#define	USB_CDC_STATE_SEND_BREAK	(1u << 6)

#define	USB_CDC_INSTANCE_IS_CLAIMED(ucs)	\
		(((ucs)->ucs_state & USB_CDC_STATE_CLAIMED) != 0)
#define	USB_CDC_INSTANCE_IS_FREE(ucs)	(!USB_CDC_INSTANCE_IS_CLAIMED(ucs))

	uint16_t ucs_cfg;
	volatile bool ucs_tx_idle;
	uint32_t ucs_custom_baud;
	ringbuff_t ucs_rb_from_usb;
	ringbuff_t ucs_rb_to_usb;
	usb_cdc_status_callback_t ucs_status_cb;
	void *ucs_status_cb_arg;
	void *ucs_common_state;
	rtos_mutex_t ucs_mutex;
};
static struct usb_cdc_state usb_cdc_states[CFG_TUD_CDC];

/*
 * ==========================================================================
 *                     USB Descriptors for CDC Device
 *                   From TinyUSB cdc_fual_ports example
 * ==========================================================================
 */

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

/*
 * --------------------------------------------------------------------+
 * Device Descriptors
 * --------------------------------------------------------------------+
 */
static tusb_desc_device_t desc_device = {
	.bLength            = sizeof(tusb_desc_device_t),
	.bDescriptorType    = TUSB_DESC_DEVICE,
	.bcdUSB             = 0x0200,

	/*
	* Use Interface Association Descriptor (IAD) for CDC
	* As required by USB Specs IAD's subclass must be common class (2) and
	* protocol must be IAD (1)
	*/
	.bDeviceClass       = TUSB_CLASS_MISC,
	.bDeviceSubClass    = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol    = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor           = 0xCafe,	/* XXX: Dynamic. */
	.idProduct          = USB_PID,	/* XXX: Dynamic. */
	.bcdDevice          = 0x0100,
	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x03,
	.bNumConfigurations = 0x01
};

/*
 * --------------------------------------------------------------------+
 * Configuration Descriptor
 * --------------------------------------------------------------------+
 */
enum {
	ITF_NUM_CDC_0 = 0,
	ITF_NUM_CDC_0_DATA,
#if (CFG_TUD_CDC > 1)
	ITF_NUM_CDC_1,
	ITF_NUM_CDC_1_DATA,
#endif
#if (CFG_TUD_CDC > 2)
	ITF_NUM_CDC_2,
	ITF_NUM_CDC_2_DATA,
#endif
	ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + CFG_TUD_CDC * TUD_CDC_DESC_LEN)

#define EPNUM_CDC_0_NOTIF   0x81u
#define EPNUM_CDC_0_DATA    0x02u
#if (CFG_TUD_CDC > 1)
#define EPNUM_CDC_1_NOTIF   0x83u
#define EPNUM_CDC_1_DATA    0x04u
#endif
#if (CFG_TUD_CDC > 2)
#define EPNUM_CDC_2_NOTIF   0x85u
#define EPNUM_CDC_2_DATA    0x06u
#endif

#ifndef PLATFORM_USB_CURRENT
#define	PLATFORM_USB_CURRENT	500
#endif

static uint8_t const desc_fs_configuration[] = {
	/*
	 * Config number, interface count, string index, total length,
	 * attribute, power in mA.
	 */
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
	    TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
	    PLATFORM_USB_CURRENT),

	/*
	 * CDC: Interface number, string index, EP notification address
	 * and size, EP data address (out, in) and size.
	 */

	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8,
	    EPNUM_CDC_0_DATA, 0x80 | EPNUM_CDC_0_DATA, 64),

#if (CFG_TUD_CDC > 1)
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8,
	    EPNUM_CDC_1_DATA, 0x80 | EPNUM_CDC_1_DATA, 64),
#endif
#if (CFG_TUD_CDC > 2)
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, 4, EPNUM_CDC_2_NOTIF, 8,
	    EPNUM_CDC_2_DATA, 0x80 | EPNUM_CDC_2_DATA, 64),
#endif
};

#if TUD_OPT_HIGH_SPEED
static uint8_t const desc_hs_configuration[] = {
	/*
	 * Config number, interface count, string index, total length,
	 * attribute, power in mA.
	 */
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
	    TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

	/*
	 * CDC: Interface number, string index, EP notification address
	 * and size, EP data address (out, in) and size.
	 */

	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8,
	    EPNUM_CDC_0_DATA, 0x80 | EPNUM_CDC_0_DATA, 512),

#if (CFG_TUD_CDC > 1)
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, EPNUM_CDC_1_NOTIF, 8,
	    EPNUM_CDC_1_DATA, 0x80 | EPNUM_CDC_1_DATA, 512),
#endif
#if (CFG_TUD_CDC > 2)
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, 4, EPNUM_CDC_2_NOTIF, 8,
	    EPNUM_CDC_2_DATA, 0x80 | EPNUM_CDC_2_DATA, 512),
#endif
};
#endif	/* TUD_OPT_HIGH_SPEED */

/*
 * Invoked when received GET CONFIGURATION DESCRIPTOR
 * Application return pointer to descriptor
 * Descriptor contents must exist long enough for transfer to complete.
 */
uint8_t const *
tud_descriptor_configuration_cb(uint8_t index)
{

	(void) index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
	/* Although we are highspeed, host may be fullspeed. */
	return (tud_speed_get() == TUSB_SPEED_HIGH) ?  desc_hs_configuration :
	    desc_fs_configuration;
#else
	return desc_fs_configuration;
#endif
}

/* --------------------------------------------------------------------+
 * String Descriptors
 * --------------------------------------------------------------------+
 */
/* Array of pointer to string descriptors */
char const *string_desc_arr[] = {
	/* 0: Supported language is English */
	(const char[]) { 0x09, 0x04 },

	/* 1: Manufacturer */
	"Maverick Embedded Technology",

	/* 2: Product */
	PRODUCT_NAME_USB,

	/* 3: Serials, should use chip ID */
	NULL,

	/* 4: CDC Interface */
	"CDC",
};

/*
 * Invoked when received GET STRING DESCRIPTOR request
 * Application return pointer to descriptor, whose contents must exist
 * long enough for transfer to complete
 */
uint16_t const *
tud_descriptor_string_cb(uint8_t idx, uint16_t langid)
{
	static uint16_t desc_str[96];
#define	USB_DESC_SIZE	(sizeof(desc_str) / sizeof(desc_str[0]))
	size_t chr_cnt;

	(void) langid;

	if (idx == 0) {
		/* Supported language is English. */
		desc_str[1] = 0x0409u;
		chr_cnt = 1;
	} else {
		const char *str;

		switch (idx) {
		case 1:
			/* Manufacturer */
			str = USB_CDC_VENDOR_STRING;
			chr_cnt = sizeof(USB_CDC_VENDOR_STRING) - 1;
			break;

		case 2:
			/* Product name */
			str = USB_CDC_PRODUCT_STRING;
			chr_cnt = sizeof(USB_CDC_PRODUCT_STRING) - 1;
			break;

		case 3:
			/* Serial number */
			str = usb_serial_no;
			chr_cnt = strlen(str);
			break;

		case 4:
			/* CDC Interface */
			str = "CDC";
			chr_cnt = sizeof("CDC") - 1;
			break;

		default:
			return NULL;
		}

		if (chr_cnt > (USB_DESC_SIZE - 1u))
			chr_cnt = USB_DESC_SIZE - 1u;

		/* Convert ASCII string into UTF-16 */
		for (size_t i = 0; i < chr_cnt; i++)
			desc_str[i + 1] = (uint16_t)str[i] & 0xffu;
	}

	/* first byte is length (including header), second byte is type */
	desc_str[0] = (TUSB_DESC_STRING << 8 ) | ((2u * chr_cnt) + 2u);

	return desc_str;
}

static void
usb_cdc_notify_line_state(struct usb_cdc_state *ucs, bool dtr)
{
	uint8_t state;

	DBFPRINTF("Instance %u: DTR %u\n", ucs->ucs_instance, dtr);

	state = ucs->ucs_state;

	if (dtr)
		ucs->ucs_state = state | USB_CDC_STATE_DTR_SET;
	else
		ucs->ucs_state = state & ~USB_CDC_STATE_DTR_SET;

	if (USB_CDC_INSTANCE_IS_FREE(ucs)) {
		DBFPRINTF("Instance %u: already free.\n", ucs->ucs_instance);
		return;
	}

	if (ucs->ucs_status_cb != NULL && ((state & USB_CDC_STATE_DTR_SET) ||
	    dtr)) {
		DBFPRINTF("Instance %u: Invoking status callback.\n",
		    ucs->ucs_instance);

		(ucs->ucs_status_cb)(dtr ? USB_CDC_STATUS_CONNECTED :
		    USB_CDC_STATUS_HANGUP, ucs->ucs_status_cb_arg);
	} else {
		DBFPRINTF("Instance %u: No status callback.\n",
		    ucs->ucs_instance);
	}
}

static void
usb_cdc_do_rx(struct usb_cdc_state *ucs)
{
	ringbuff_len_t len;
	ringbuff_t rb;
	uint32_t cdclen, rxlen, total;
	void *p;

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("instance %u Rx handler\n", ucs->ucs_instance);

	/*
	 * If we receive data while not yet open, assume the host has failed
	 * to raise DTR. Fake it, so that netusb_mux invokes usb_cdc_open().
	 */
	if ((ucs->ucs_state & (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) !=
	    (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) {
		DBFPRINTF("not open/attached.\n");
		if ((ucs->ucs_state & USB_CDC_STATE_DTR_SET) == 0) {
			DBFPRINTF("Faking DTR=1.\n");
			usb_cdc_notify_line_state(ucs, true);
		}
		rtos_mutex_release(ucs->ucs_mutex);
		return;
	}

#ifdef CONFIG_USE_DISPLAY
	display_activity(DISPLAY_ACTIVITY_USB);
#endif

	rb = ucs->ucs_rb_from_usb;
	total = cdclen = 0;

	while ((len = ringbuff_produce_contig_size(rb)) != 0 &&
	    (cdclen = tud_cdc_n_available(ucs->ucs_instance)) != 0) {
		if (cdclen < len)
			len = cdclen;

		p = ringbuff_produce_current_buff_pointer(rb);

		DBFPRINTF("Reading %" PRIuRBLEN " bytes to %p\n", len, p);

		rxlen = tud_cdc_n_read(ucs->ucs_instance, p, len);

		DBFPRINTF("Received %" PRIu32 " bytes:\n", rxlen);
#if (DEBUG_FLAG_INIT != 0)
		hexdump(NULL, " ", 16, p, rxlen);
#endif

		ringbuff_produce_contig(rb, rxlen);
		total += rxlen;
	}

	if (total)
		ringbuff_produce_done(rb);

	DBFPRINTF("done. Consumed %" PRIu32 " bytes\n", total);

	rtos_mutex_release(ucs->ucs_mutex);
}

static uint8_t
usb_cdc_do_tx(struct usb_cdc_state *ucs)
{
	ringbuff_len_t len;
	ringbuff_t rb;
	uint32_t total, cdclen, txlen;
	rtos_saved_ipl_t ipl;
	void *p;

	if ((ucs->ucs_state & (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) !=
	    (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) {
		DBFPRINTF("not open/attached. Ignoring (%02x)\n",ucs->ucs_state);
		/* XXX: Flush ringbuff, if non-NULL? */
		return 0;
	}

	DBFPRINTF("instance %u Tx handler\n", ucs->ucs_instance);

#ifdef CONFIG_USE_DISPLAY
	display_activity(DISPLAY_ACTIVITY_USB);
#endif

	rb = ucs->ucs_rb_to_usb;
	total = txlen = 0;

	ipl = rtos_ipl_raise(HW_IPL_USB);

	while ((len = ringbuff_consume_contig_size(rb)) != 0 &&
	    (cdclen = tud_cdc_n_write_available(ucs->ucs_instance)) != 0) {
		rtos_ipl_restore(ipl);

		if (cdclen < len)
			len = cdclen;

		p = ringbuff_consume_current_buff_pointer(rb);

		DBFPRINTF("Sending %" PRIuRBLEN " bytes.\n", len);
#if (DEBUG_FLAG_INIT != 0)
		hexdump(NULL, " ", 16, p, len);
#endif

		txlen = tud_cdc_n_write(ucs->ucs_instance, p, len);

		ringbuff_consume_contig(rb, txlen);
		total += txlen;

		ipl = rtos_ipl_raise(HW_IPL_USB);
	}

	ucs->ucs_tx_idle = total == 0;
	rtos_ipl_restore(ipl);

	DBFPRINTF("done. Sent %" PRIu32 " bytes.\n", total);

	return total != 0;
}

static void
usb_cdc_tx_ready_callback(void *arg)
{
	struct usb_cdc_state *ucs = arg;

	/*
	 * We're running in the USB task's context, in response to
	 * new data being available to transmit.
	 */

	rtos_mutex_acquire(ucs->ucs_mutex);

	/*
	 * Try to Tx some data.
	 */
	if (usb_cdc_do_tx(ucs)) {
		/*
		 * We were able to send some data, so if the Tx ring is now
		 * empty, flush the CDC FiFo to send a short packet.
		 */
		if (ringbuff_is_empty(ucs->ucs_rb_to_usb))
			tud_cdc_n_write_flush(ucs->ucs_instance);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

static void
usb_cdc_tx_ready(ringbuff_t rb, void *arg)
{
	struct usb_cdc_state *ucs = arg;
	rtos_saved_ipl_t ipl;
	bool tx_idle;

	(void) rb;

	/*
	 * Danger, Will Robinson!
	 *
	 * This function may be invoked from the Target Serial/SWO USART's
	 * interrupt handler.
	 */

	/*
	 * Grab the current Tx idle status, then mark unconditionally as
	 * not-idle.
	 */
	ipl = rtos_ipl_raise(HW_IPL_USB);
	tx_idle = ucs->ucs_tx_idle;
	ucs->ucs_tx_idle = false;
	rtos_ipl_restore(ipl);

	/*
	 * We don't need to do anything if there's a transmission in
	 * progress.
	 */
	if (!tx_idle)
		return;

	/*
	 * We need to kick CDC Tx into action.
	 * Schedule a callback in the USB task's context if we're invoked
	 * from interrupt mode. Otherwise, we can kick it off immediately.
	 */
	if (_rtos_interrupt_context())
		usbd_defer_func(usb_cdc_tx_ready_callback, ucs, true);
	else
		usb_cdc_tx_ready_callback(ucs);
}

void *
usb_cdc_claim(uint8_t instance, usb_cdc_status_callback_t cb, void *arg)
{
	struct usb_cdc_state *ucs;
	bool fail = false;

	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return NULL;

	assert(ucs->ucs_instance == instance);

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("instance %u\n", instance);

	if (USB_CDC_INSTANCE_IS_FREE(ucs)) {
		ucs->ucs_state |= USB_CDC_STATE_CLAIMED;
		ucs->ucs_status_cb = cb;
		ucs->ucs_status_cb_arg = arg;

		DBFPRINTF("new owner\n");
	} else {
		DBFPRINTF("instance already claimed\n");
		fail = true;
	}

	rtos_mutex_release(ucs->ucs_mutex);

	return fail ? NULL : ucs;
}

void
usb_cdc_unclaim(void *arg)
{
	struct usb_cdc_state *ucs = arg;

	assert(ucs->ucs_common_state != NULL);

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("instance %u\n", ucs->ucs_instance);

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs)) {
		ucs->ucs_state &= ~USB_CDC_STATE_CLAIMED;
		ucs->ucs_status_cb = NULL;

		if ((ucs->ucs_state & USB_CDC_STATE_OPEN) != 0) {
			ucs->ucs_state &= ~USB_CDC_STATE_OPEN;
			ucs->ucs_rb_to_usb = NULL;
			ucs->ucs_rb_from_usb = NULL;
		}
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

int
usb_cdc_open(void *arg, uint16_t cfg, uint32_t custom_baud, ringbuff_t to_usb,
    ringbuff_t from_usb)
{
	struct usb_cdc_state *ucs = arg;
	int rv = -1;

	assert(ucs->ucs_common_state != NULL);

	rtos_mutex_acquire(ucs->ucs_mutex);

	if ((ucs->ucs_state & USB_CDC_STATE_ATTACHED) != 0) {
		ucs->ucs_state |= USB_CDC_STATE_OPEN;
		ucs->ucs_tx_idle = true;
		ucs->ucs_cfg = cfg;
		ucs->ucs_custom_baud = custom_baud;
		ucs->ucs_rb_to_usb = to_usb;
		ucs->ucs_rb_from_usb = from_usb;
		rv = 0;
	}

	if (rv == 0) {
		DBFPRINTF("instance %u opened. rbto %p, rbfrom %p\n",
		    ucs->ucs_instance, (void *)to_usb, (void *)from_usb);
		ringbuff_consumer_init(to_usb, usb_cdc_tx_ready, ucs);
	} else {
		DBFPRINTF("instance %u not attached\n", ucs->ucs_instance);
	}

	rtos_mutex_release(ucs->ucs_mutex);

	/* Try to get any pending Rx data moving. */
	usb_cdc_do_rx(ucs);

	return rv;
}

void
usb_cdc_close(void *arg)
{
	struct usb_cdc_state *ucs = arg;

	rtos_mutex_acquire(ucs->ucs_mutex);

	if ((ucs->ucs_state & USB_CDC_STATE_OPEN) != 0) {
		tud_cdc_n_read_flush(ucs->ucs_instance);
		tud_cdc_n_write_clear(ucs->ucs_instance);
		ucs->ucs_state &= ~USB_CDC_STATE_OPEN;
		ucs->ucs_rb_to_usb = NULL;
		ucs->ucs_rb_from_usb = NULL;

		DBFPRINTF("instance %u closed\n", ucs->ucs_instance);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

static void
usb_cdc_enable(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("enabling instance %u\n", instance);

	ucs->ucs_state |= USB_CDC_STATE_ATTACHED;

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs) && ucs->ucs_status_cb != NULL) {
		DBFPRINTF("invoking 'attached' callback\n");
		(ucs->ucs_status_cb)(USB_CDC_STATUS_ATTACHED,
		    ucs->ucs_status_cb_arg);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

static void
usb_cdc_disable(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("disabling instance %u\n", instance);

	ucs->ucs_state &= ~USB_CDC_STATE_ATTACHED;

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs) && ucs->ucs_status_cb != NULL) {
		(ucs->ucs_status_cb)(USB_CDC_STATUS_DETACHED,
		    ucs->ucs_status_cb_arg);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

/*
 * TinyUSB callback API
 */
void
tud_mount_cb(void)
{

	DBFPRINTF("Enabling CDC interfaces.\n");

	/*
	 * We've been plugged into a host, and configuration is complete.
	 */
	for (uint8_t i = 0; i < CFG_TUD_CDC; i++)
		usb_cdc_enable(i);
}

void
tud_umount_cb(void)
{

	DBFPRINTF("Disabling CDC interfaces.\n");

	/*
	 * We've been unpluggined from a host.
	 */
	for (uint8_t i = 0; i < CFG_TUD_CDC; i++)
		usb_cdc_disable(i);
}

void
tud_cdc_rx_cb(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	/*
	 * New Rx data is available on a CDC port.
	 */
	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	DBFPRINTF("instance %u\n", ucs->ucs_instance);
	usb_cdc_do_rx(ucs);
}

void
tud_cdc_tx_complete_cb(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	/*
	 * Tx space is available on a CDC port.
	 */
	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	DBFPRINTF("Instance %u: Tx complete.\n", instance);

	if (ucs->ucs_rb_to_usb != NULL)
		ringbuff_consume_done(ucs->ucs_rb_to_usb);

	usb_cdc_tx_ready_callback(ucs);
}

void
tud_cdc_line_state_cb(uint8_t instance, bool dtr, bool rts)
{
	struct usb_cdc_state *ucs;

	/*
	 * CDC_REQUEST_SET_CONTROL_LINE_STATE message received.
	 */
	(void) rts;
	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	DBFPRINTF("Instance %u: DTR %u, RTS %u\n", instance, dtr, rts);

	rtos_mutex_acquire(ucs->ucs_mutex);

	usb_cdc_notify_line_state(ucs, dtr);

	rtos_mutex_release(ucs->ucs_mutex);
}

void
tud_cdc_send_break_cb(uint8_t instance, uint16_t duration_ms)
{
	struct usb_cdc_state *ucs;
	uint8_t state;

	/*
	 * Host requests we send a line BREAK
	 */
	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	rtos_mutex_acquire(ucs->ucs_mutex);

	DBFPRINTF("Instance %u: duration %u ms\n", instance, duration_ms);

	state = ucs->ucs_state;
	if (duration_ms)
		ucs->ucs_state = state | USB_CDC_STATE_SEND_BREAK;
	else
		ucs->ucs_state = state & ~USB_CDC_STATE_SEND_BREAK;

	if (!USB_CDC_INSTANCE_IS_FREE(ucs) && duration_ms &&
	    (state & USB_CDC_STATE_SEND_BREAK) == 0 &&
	    ucs->ucs_status_cb != NULL) {
		(ucs->ucs_status_cb)(USB_CDC_STATUS_SEND_BREAK,
		    ucs->ucs_status_cb_arg);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

void
tud_cdc_line_coding_cb(uint8_t instance, const cdc_line_coding_t *clc)
{
	struct usb_cdc_state *ucs;
	uint32_t baud;
	uint16_t cfg;

	/*
	 * Change the line coding (baudrate, stop-bits, parity ...)
	 */
	assert(instance < CFG_TUD_CDC);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	if (USB_CDC_INSTANCE_IS_FREE(ucs))
		return;

	rtos_mutex_acquire(ucs->ucs_mutex);

	if (clc->bit_rate == 0) {
		baud = ucs->ucs_custom_baud;
		cfg = ucs->ucs_cfg;
	} else {
		cfg = uart_baud_to_cfg(clc->bit_rate);
		if (cfg == UART_BAUDRATE_CUSTOM)
			baud = clc->bit_rate;
		else
			baud = clc->bit_rate;
	}

	switch (clc->stop_bits) {
	case 1:
	default:
		cfg |= UART_STOPBITS_1;
		break;
	case 2:
		cfg |= UART_STOPBITS_2;
		break;
	}

	switch (clc->parity) {
	default:
	case 0:
		cfg |= UART_PARITY_NONE;
		break;
	case 1:
		cfg |= UART_PARITY_ODD;
		break;
	case 2:
		cfg |= UART_PARITY_EVEN;
		break;
	}

	switch (clc->data_bits) {
	case 5:
		cfg |= UART_CHARSIZE_5;
		break;
	case 6:
		cfg |= UART_CHARSIZE_6;
		break;
	case 7:
		cfg |= UART_CHARSIZE_7;
		break;
	case 8:
	default:
		cfg |= UART_CHARSIZE_8;
		break;
	}

	ucs->ucs_cfg = cfg;
	ucs->ucs_custom_baud = baud;
	if (ucs->ucs_status_cb != NULL) {
		(ucs->ucs_status_cb)(USB_CDC_STATUS_CONFIG,
		    ucs->ucs_status_cb_arg);
	}

	rtos_mutex_release(ucs->ucs_mutex);
}

uint16_t
usb_cdc_get_config(void *arg, uint32_t *custom_baud)
{
	struct usb_cdc_state *ucs = arg;

	*custom_baud = ucs->ucs_custom_baud;
	return ucs->ucs_cfg;
}

#if 0
void
usb_cdc_set_config(void *arg, uint16_t cfg, uint32_t custom_baud)
{
	struct usb_cdc_state *ucs = arg;
	usb_cdc_line_coding_t clc;

	ucs->ucs_cfg = cfg;
	ucs->ucs_custom_baud = custom_baud;
	usb_cdc_cfg_to_line_coding(cfg, custom_baud, &clc);
	udi_cdc_update_line_coding(ucs->ucs_instance, &clc);
}
#endif

const void *
usb_cdc_attach(void *common_state, void **cdc_state)
{
	struct usb_cdc_state *ucs;
	uint16_t vid, pid;
	unsigned int p;
	uint8_t mode;

	mode = usb_get_config(&vid, &pid);
	assert(mode == USB_PERSONALITY_CDC);
	assert(vid != 0);
	(void) mode;

	desc_device.idVendor = vid;	/* XXX: Assumes little-endian */
	desc_device.idProduct = pid;

	memset(usb_cdc_states, 0, sizeof(usb_cdc_states));

	for (p = 0, ucs = usb_cdc_states; p < CFG_TUD_CDC; p++, ucs++) {
		ucs->ucs_instance = p;
		ucs->ucs_state = 0;
		ucs->ucs_tx_idle = true;
		ucs->ucs_cfg = UART_DEFAULT_CFG;
		ucs->ucs_common_state = common_state;
		ucs->ucs_mutex = rtos_mutex_create();
		assert(ucs->ucs_mutex != NULL);
	}

	*cdc_state = usb_cdc_states;
	return &desc_device;
}
