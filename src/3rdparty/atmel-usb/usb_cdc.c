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
#include "display.h"
#include "uart.h"
#ifdef CONFIG_USE_CONFIGDB
#include "configdb.h"
#endif
#include "usb_config.h"
#include "udc.h"
#include "udi_cdc.h"
#include "usb_protocol_cdc.h"
#include "usb_common.h"

#if 0
#define DBPRINTF(fmt, ...)				\
	do {						\
		fprintf(stderr, fmt, ##__VA_ARGS__);	\
		fflush(stderr);				\
	} while (0)
#else
#define DBPRINTF(fmt, ...)      do { } while (0)
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
#define	USB_CDC_STATE_TX_BUSY		(1u << 7)

#define	USB_CDC_INSTANCE_IS_CLAIMED(ucs)	\
		(((ucs)->ucs_state & USB_CDC_STATE_CLAIMED) != 0)
#define	USB_CDC_INSTANCE_IS_FREE(ucs)	(!USB_CDC_INSTANCE_IS_CLAIMED(ucs))

	uint16_t ucs_cfg;
	uint32_t ucs_custom_baud;
	ringbuff_t ucs_rb_from_usb;
	ringbuff_t ucs_rb_to_usb;
	usb_cdc_status_callback_t ucs_status_cb;
	void *ucs_status_cb_arg;
	void *ucs_common_state;
	rtos_mutex_t ucs_mutex;
};
static struct usb_cdc_state usb_cdc_states[UDI_CDC_PORT_NB];

/*
 * ==========================================================================
 *                     USB Descriptors for CDC Device
 * ==========================================================================
 */
#ifdef USB_DEVICE_LPM_SUPPORT
#define USB_VERSION   USB_V2_1
#else
#define USB_VERSION   USB_V2_0
#endif

/* Don't mark this as const - it must be in SRAM for USB DMA to work. */
static ALIGNED(16) usb_dev_desc_t udc_device_desc_cdc = {
	.bLength		= sizeof(usb_dev_desc_t),
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB			= LE16(USB_VERSION),
	.bDeviceClass		= NO_CLASS,
	.bDeviceSubClass	= NO_SUBCLASS,
	.bDeviceProtocol	= NO_PROTOCOL,
	.bMaxPacketSize0	= USB_DEVICE_EP_CTRL_SIZE,
	.idVendor		= LE16(USB_CDC_DEVICE_VENDOR_ID),
	.idProduct		= LE16(USB_CDC_DEVICE_PRODUCT_ID),
	.bcdDevice		= LE16((USB_CDC_DEVICE_MAJOR_VERSION << 8) |
					USB_CDC_DEVICE_MINOR_VERSION),
	.iManufacturer		= 1,
	.iProduct		= 2,
	.iSerialNumber		= 3,
	.bNumConfigurations	= 1
};

#ifdef USB_DEVICE_LPM_SUPPORT
/* Don't mark this as const - it must be in SRAM for USB DMA to work. */
static ALIGNED(16) usb_dev_lpm_desc_t udc_device_lpm_cdc = {
	.bos.bLength		= sizeof(usb_dev_bos_desc_t),
	.bos.bDescriptorType	= USB_DT_BOS,
	.bos.wTotalLength	= LE16(sizeof(usb_dev_bos_desc_t) +
				       sizeof(usb_dev_capa_ext_desc_t)),
	.bos.bNumDeviceCaps	= 1,
	.capa_ext.bLength	= sizeof(usb_dev_capa_ext_desc_t),
	.capa_ext.bDescriptorType = USB_DT_DEVICE_CAPABILITY,
	.capa_ext.bDevCapabilityType = USB_DC_USB20_EXTENSION,
	.capa_ext.bmAttributes	= USB_DC_EXT_LPM,
};
#endif

typedef struct {
	usb_conf_desc_t		conf;
	usb_iad_desc_t		udi_cdc_iad_0;
	udi_cdc_comm_desc_t	udi_cdc_comm_0;
	udi_cdc_data_desc_t	udi_cdc_data_0;
#if (PLATFORM_USB_CDC_INSTANCE_NB > 1)
	usb_iad_desc_t		udi_cdc_iad_1;
	udi_cdc_comm_desc_t	udi_cdc_comm_1;
	udi_cdc_data_desc_t	udi_cdc_data_1;
#if (PLATFORM_USB_CDC_INSTANCE_NB > 2)
	usb_iad_desc_t		udi_cdc_iad_2;
	udi_cdc_comm_desc_t	udi_cdc_comm_2;
	udi_cdc_data_desc_t	udi_cdc_data_2;
#endif
#endif
} udc_desc_cdc_t PACKED;

#define	USB_DEVICE_NB_INTERFACE_CDC	(PLATFORM_USB_CDC_INSTANCE_NB * 2)

/* Don't mark this as const - it must be in SRAM for USB DMA to work. */
static ALIGNED(16) udc_desc_cdc_t udc_desc_fs_cdc = {
	.conf.bLength		= sizeof(usb_conf_desc_t),
	.conf.bDescriptorType	= USB_DT_CONFIGURATION,
	.conf.wTotalLength	= LE16(sizeof(udc_desc_cdc_t)),
	.conf.bNumInterfaces	= USB_DEVICE_NB_INTERFACE_CDC,
	.conf.bConfigurationValue = 1,
	.conf.iConfiguration	= 0,
	.conf.bmAttributes	= USB_CONFIG_ATTR_MUST_SET | USB_DEVICE_ATTR,
	.conf.bMaxPower		= USB_CONFIG_MAX_POWER(USB_DEVICE_POWER),
	.udi_cdc_iad_0		= UDI_CDC_IAD_DESC_0,
	.udi_cdc_comm_0		= UDI_CDC_COMM_DESC_0,
	.udi_cdc_data_0		= UDI_CDC_DATA_DESC_0_FS,
#if (PLATFORM_USB_CDC_INSTANCE_NB > 1)
	.udi_cdc_iad_1		= UDI_CDC_IAD_DESC_1,
	.udi_cdc_comm_1		= UDI_CDC_COMM_DESC_1,
	.udi_cdc_data_1		= UDI_CDC_DATA_DESC_1_FS,
#if (PLATFORM_USB_CDC_INSTANCE_NB > 2)
	.udi_cdc_iad_2		= UDI_CDC_IAD_DESC_2,
	.udi_cdc_comm_2		= UDI_CDC_COMM_DESC_2,
	.udi_cdc_data_2		= UDI_CDC_DATA_DESC_2_FS,
#endif /* (PLATFORM_USB_CDC_INSTANCE_NB > 2) */
#endif /* (PLATFORM_USB_CDC_INSTANCE_NB > 1) */
};

static const udi_api_t * const udi_apis_cdc[USB_DEVICE_NB_INTERFACE_CDC] = {
	&udi_api_cdc_comm, &udi_api_cdc_data,
#if (PLATFORM_USB_CDC_INSTANCE_NB > 1)
	&udi_api_cdc_comm, &udi_api_cdc_data,
#if (PLATFORM_USB_CDC_INSTANCE_NB > 2)
	&udi_api_cdc_comm, &udi_api_cdc_data,
#endif
#endif
};

static const udc_config_speed_t udc_config_fs_cdc = {
	.desc		= &udc_desc_fs_cdc.conf,
	.udi_apis	= udi_apis_cdc,
};

static const udc_config_t udc_config_cdc = {
	.confdev_lsfs	= (usb_dev_desc_t *)(uintptr_t)&udc_device_desc_cdc,
	.conf_lsfs	= (udc_config_speed_t *)(uintptr_t)&udc_config_fs_cdc,
#ifdef USB_DEVICE_LPM_SUPPORT
	.conf_bos	= &udc_device_lpm_cdc.bos;
#else
	.conf_bos	= NULL,
#endif
};

static void
usb_cdc_mutex_acquire(struct usb_cdc_state *ucs)
{

	rtos_mutex_acquire(ucs->ucs_mutex);
}

static void
usb_cdc_mutex_release(struct usb_cdc_state *ucs)
{

	rtos_mutex_release(ucs->ucs_mutex);
}

static void
usb_cdc_do_rx(struct usb_cdc_state *ucs)
{
	uint8_t produced;
	ringbuff_len_t len;
	ringbuff_t rb;
	iram_size_t rxlen;

	DBPRINTF("usb_cdc_do_rx: instance %u Rx handler\n", ucs->ucs_instance);

	/*
	 * If we're not open, just eat input from the host
	 */
	if ((ucs->ucs_state & (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) !=
	    (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) {
		DBPRINTF("usb_cdc_do_rx: not open/attached. Flushing input\n");
		while (udi_cdc_multi_is_rx_ready(ucs->ucs_instance))
			(void) udi_cdc_multi_getc(ucs->ucs_instance);
		return;
	}

	display_activity(DISPLAY_ACTIVITY_USB);

	produced = 0;
	rb = ucs->ucs_rb_from_usb;

	while ((len = ringbuff_produce_contig_size(rb)) != 0 &&
	    (rxlen =
	     udi_cdc_multi_get_nb_received_data(ucs->ucs_instance)) != 0) {
		if (rxlen < len)
			len = rxlen;

		rxlen = udi_cdc_multi_read_buf(ucs->ucs_instance,
		    ringbuff_produce_current_buff_pointer(rb), len);

		DBPRINTF("usb_cdc_do_rx: Reading %lu bytes. Result %lu\n",
		    (uint32_t)len, (uint32_t)rxlen);

		if (rxlen != 0)
			break;

		ringbuff_produce_contig(rb, len);
		produced = 1;
	}

	if (produced)
		ringbuff_produce_done(rb);

	DBPRINTF("usb_cdc_do_rx: done. produced %u\n", produced);
}

static void
usb_cdc_do_tx(struct usb_cdc_state *ucs)
{
	uint8_t consumed;
	ringbuff_len_t len;
	ringbuff_t rb;
	iram_size_t txlen;

	if ((ucs->ucs_state & (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) !=
	    (USB_CDC_STATE_ATTACHED | USB_CDC_STATE_OPEN)) {
		DBPRINTF("usb_cdc_do_tx: not open/attached. Ignoring (%02x)\n",
		    ucs->ucs_state);
		/* XXX: Flush ringbuff, if non-NULL? */
		return;
	}

	DBPRINTF("usb_cdc_do_tx: instance %u Tx handler\n", ucs->ucs_instance);

	display_activity(DISPLAY_ACTIVITY_USB);

	consumed = 0;
	rb = ucs->ucs_rb_to_usb;

	while ((len = ringbuff_consume_contig_size(rb)) != 0 &&
	    (txlen = udi_cdc_multi_get_free_tx_buffer(ucs->ucs_instance)) != 0) {
		if (txlen < len)
			len = txlen;

		txlen = udi_cdc_multi_write_buf(ucs->ucs_instance,
		    ringbuff_consume_current_buff_pointer(rb), len);

		DBPRINTF("usb_cdc_do_tx: Sending %lu bytes. Remaining %lu\n",
		    (uint32_t)len, (uint32_t)txlen);

		if (txlen == len)
			break;

		consumed = 1;
		ringbuff_consume_contig(rb, len - txlen);
	}

	if (consumed) {
		ringbuff_consume_done(rb);
		ucs->ucs_state |= USB_CDC_STATE_TX_BUSY;
	} else {
		ucs->ucs_state &= ~USB_CDC_STATE_TX_BUSY;
	}

	DBPRINTF("usb_cdc_do_tx: done. consumed %u\n", consumed);
}

static void
usb_cdc_tx_ready(ringbuff_t rb, void *arg)
{
	struct usb_cdc_state *ucs = arg;
	rtos_saved_ipl_t ipl;

	(void) rb;

	ipl = rtos_ipl_raise(HW_IPL_USB);
	if ((ucs->ucs_state & USB_CDC_STATE_TX_BUSY) == 0)
		usb_cdc_do_tx(ucs);
	rtos_ipl_restore(ipl);
}

void *
usb_cdc_claim(uint8_t instance, usb_cdc_status_callback_t cb, void *arg)
{
	struct usb_cdc_state *ucs;
	bool fail = false;

	assert(instance < UDI_CDC_PORT_NB);

	DBPRINTF("usb_cdc_claim: instance %u\n", instance);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return NULL;

	assert(ucs->ucs_instance == instance);

	usb_cdc_mutex_acquire(ucs);

	if (USB_CDC_INSTANCE_IS_FREE(ucs)) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_USB);
		ucs->ucs_state |= USB_CDC_STATE_CLAIMED;
		ucs->ucs_status_cb = cb;
		ucs->ucs_status_cb_arg = arg;
		rtos_ipl_restore(ipl);

		DBPRINTF("usb_cdc_claim: new owner\n");
	} else {
		DBPRINTF("usb_cdc_claim: instance already claimed\n");
		fail = true;
	}

	usb_cdc_mutex_release(ucs);

	return fail ? NULL : ucs;
}

void
usb_cdc_unclaim(void *arg)
{
	struct usb_cdc_state *ucs = arg;

	assert(ucs->ucs_common_state != NULL);

	DBPRINTF("usb_cdc_unclaim: instance %u\n", ucs->ucs_instance);

	usb_cdc_mutex_acquire(ucs);

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs)) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_USB);

		ucs->ucs_state &= ~USB_CDC_STATE_CLAIMED;
		ucs->ucs_status_cb = NULL;

		if ((ucs->ucs_state & USB_CDC_STATE_OPEN) != 0) {
			ucs->ucs_state &= ~USB_CDC_STATE_OPEN;
			ucs->ucs_rb_to_usb = NULL;
			ucs->ucs_rb_from_usb = NULL;
		}

		rtos_ipl_restore(ipl);

		usb_common_wakeup(ucs->ucs_common_state);
	}

	usb_cdc_mutex_release(ucs);
}

int
usb_cdc_open(void *arg, uint16_t cfg, uint32_t custom_baud, ringbuff_t to_usb,
    ringbuff_t from_usb)
{
	struct usb_cdc_state *ucs = arg;
	rtos_saved_ipl_t ipl;
	int rv = -1;

	assert(ucs->ucs_common_state != NULL);

	usb_cdc_mutex_acquire(ucs);

	ipl = rtos_ipl_raise(HW_IPL_USB);
	if ((ucs->ucs_state & USB_CDC_STATE_ATTACHED) != 0) {
		ucs->ucs_state |= USB_CDC_STATE_OPEN;
		ucs->ucs_cfg = cfg;
		ucs->ucs_custom_baud = custom_baud;
		ucs->ucs_rb_to_usb = to_usb;
		ucs->ucs_rb_from_usb = from_usb;
		rv = 0;
	}
	rtos_ipl_restore(ipl);

	if (rv == 0) {
		DBPRINTF("usb_cdc_open: instance %u opened. rbto %p, "
		    "rbfrom %p\n", ucs->ucs_instance, (void *)to_usb,
		    (void *)from_usb);
		ringbuff_consumer_init(to_usb, usb_cdc_tx_ready, ucs);
		usb_common_wakeup(ucs->ucs_common_state);
	} else {
		DBPRINTF("usb_cdc_open: instance %u not attached\n",
		    ucs->ucs_instance);
	}

	usb_cdc_mutex_release(ucs);

	return rv;
}

void
usb_cdc_close(void *arg)
{
	struct usb_cdc_state *ucs = arg;

	usb_cdc_mutex_acquire(ucs);

	if ((ucs->ucs_state & USB_CDC_STATE_OPEN) != 0) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_USB);
		ucs->ucs_state &= ~USB_CDC_STATE_OPEN;
		ucs->ucs_rb_to_usb = NULL;
		ucs->ucs_rb_from_usb = NULL;
		rtos_ipl_restore(ipl);

		DBPRINTF("usb_cdc_close: instance %u closed\n",
		    ucs->ucs_instance);
		usb_common_wakeup(ucs->ucs_common_state);
	}

	usb_cdc_mutex_release(ucs);
}

uint8_t
usb_cdc_enable(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < UDI_CDC_PORT_NB);

	DBPRINTF("usb_cdc_enable: enabling instance %u\n", instance);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return 1;
	assert(ucs->ucs_instance == instance);

	ucs->ucs_state |= USB_CDC_STATE_ATTACHED;

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs) && ucs->ucs_status_cb != NULL) {
		DBPRINTF("usb_cdc_enable: invoking 'attached' callback\n");
		(ucs->ucs_status_cb)(USB_CDC_STATUS_ATTACHED,
		    ucs->ucs_status_cb_arg);
	}

	DBPRINTF("usb_cdc_enable: setting DSR\n");
	udi_cdc_multi_ctrl_signal_dsr(instance, 1);
	DBPRINTF("usb_cdc_enable: setting DCD\n");
	udi_cdc_multi_ctrl_signal_dcd(instance, 1);

	return 1;
}

void
usb_cdc_disable(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < UDI_CDC_PORT_NB);

	DBPRINTF("usb_cdc_disable: disabling instance %u\n", instance);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	ucs->ucs_state &= ~USB_CDC_STATE_ATTACHED;

	if (USB_CDC_INSTANCE_IS_CLAIMED(ucs) && ucs->ucs_status_cb != NULL) {
		(ucs->ucs_status_cb)(USB_CDC_STATUS_DETACHED,
		    ucs->ucs_status_cb_arg);
	}

	udi_cdc_multi_ctrl_signal_dsr(instance, 0);
	udi_cdc_multi_ctrl_signal_dcd(instance, 0);
}

void
usb_cdc_rx_notify(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < UDI_CDC_PORT_NB);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	usb_cdc_do_rx(ucs);
}

void
usb_cdc_tx_empty_notify(uint8_t instance)
{
	struct usb_cdc_state *ucs;

	assert(instance < UDI_CDC_PORT_NB);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	usb_cdc_do_tx(ucs);
}

void
usb_cdc_set_dtr(uint8_t instance, uint8_t enable)
{
	struct usb_cdc_state *ucs;
	uint8_t dtr;

	assert(instance < UDI_CDC_PORT_NB);

	DBPRINTF("usb_cdc_set_dtr: instance %u, %sable\n", instance,
	   enable ? "En" : "Dis");

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	dtr = ucs->ucs_state;

	if (enable)
		ucs->ucs_state = dtr | USB_CDC_STATE_DTR_SET;
	else
		ucs->ucs_state = dtr & ~USB_CDC_STATE_DTR_SET;

	if (USB_CDC_INSTANCE_IS_FREE(ucs))
		return;

	if (ucs->ucs_status_cb != NULL && ((dtr & USB_CDC_STATE_DTR_SET) ||
	    enable)) {
		(ucs->ucs_status_cb)(enable ? USB_CDC_STATUS_CONNECTED :
		    USB_CDC_STATUS_HANGUP, ucs->ucs_status_cb_arg);
	}
}

void
usb_cdc_set_break(uint8_t instance, uint8_t enable)
{
	struct usb_cdc_state *ucs;
	uint8_t state;

	assert(instance < UDI_CDC_PORT_NB);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	state = ucs->ucs_state;
	if (enable)
		ucs->ucs_state = state | USB_CDC_STATE_SEND_BREAK;
	else
		ucs->ucs_state = state & ~USB_CDC_STATE_SEND_BREAK;

	if (USB_CDC_INSTANCE_IS_FREE(ucs))
		return;

	if (enable && (state & USB_CDC_STATE_SEND_BREAK) == 0 &&
	    ucs->ucs_status_cb != NULL) {
		(ucs->ucs_status_cb)(USB_CDC_STATUS_SEND_BREAK,
		    ucs->ucs_status_cb_arg);
	}
}

static void
usb_cdc_cfg_to_line_coding(uint16_t cfg, uint32_t custom_baud,
    usb_cdc_line_coding_t *clc)
{

	if ((cfg & UART_BAUDRATE_MASK) != UART_BAUDRATE_CUSTOM) {
		static const uint16_t speeds[16] = {
			0, 3, 6, 12, 24, 48, 96, 192,
			384, 576, 1152, 2304, 4608, 9216, 18432, 0
		};

		custom_baud = (uint32_t)speeds[cfg & UART_BAUDRATE_MASK];
		if (custom_baud == 0)
			custom_baud = 150;
		else
			custom_baud *= 100u;
	}

	clc->dwDTERate = custom_baud;

	switch (cfg & UART_STOPBITS_MASK) {
	case UART_STOPBITS_1:
		clc->bCharFormat = CDC_STOP_BITS_1;
		break;
	case UART_STOPBITS_2:
		clc->bCharFormat = CDC_STOP_BITS_2;
		break;
	}

	clc->bParityType = (cfg & UART_PARITY_MASK) >> 5;
	clc->bDataBits = ((cfg & UART_CHARSIZE_MASK) >> 9) + 5;
}

void
usb_cdc_set_coding(uint8_t instance, usb_cdc_line_coding_t *clc)
{
	struct usb_cdc_state *ucs;
	uint32_t baud;
	uint16_t cfg;

	assert(instance < UDI_CDC_PORT_NB);

	ucs = &usb_cdc_states[instance];
	if (ucs->ucs_common_state == NULL)
		return;
	assert(ucs->ucs_instance == instance);

	if (USB_CDC_INSTANCE_IS_FREE(ucs)) {
		/* We normally hit this just the once at startup */
		usb_cdc_cfg_to_line_coding(ucs->ucs_cfg, ucs->ucs_custom_baud,
		    clc);
		return;
	}

	baud = 0;
	cfg = uart_baud_to_cfg(clc->dwDTERate);
	if (cfg == UART_BAUDRATE_CUSTOM)
		baud = clc->dwDTERate;

	switch (clc->bCharFormat) {
	case CDC_STOP_BITS_1:
	default:
		cfg |= UART_STOPBITS_1;
		break;
	case CDC_STOP_BITS_2:
		cfg |= UART_STOPBITS_2;
		break;
	}

	switch (clc->bParityType) {
	default:
	case CDC_PAR_NONE:
		cfg |= UART_PARITY_NONE;
		break;
	case CDC_PAR_ODD:
		cfg |= UART_PARITY_ODD;
		break;
	case CDC_PAR_EVEN:
		cfg |= UART_PARITY_EVEN;
		break;
	}

	switch (clc->bDataBits) {
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
}

uint16_t
usb_cdc_get_config(void *arg, uint32_t *custom_baud)
{
	struct usb_cdc_state *ucs = arg;

	*custom_baud = ucs->ucs_custom_baud;
	return ucs->ucs_cfg;
}

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

void
usb_cdc_workloop(void *arg)
{

	(void) arg;
}

const void *
usb_cdc_attach(void *common_state, void **cdc_state)
{
	struct usb_cdc_state *ucs;
	unsigned int p;

#ifdef CONFIG_USE_CONFIGDB
	uint16_t vid, pid;

	if (cf_get_uint16(CF_KEY_UINT16_USB_VID, &vid) != sizeof(vid))
		vid = 0;
	if (cf_get_uint16(CF_KEY_UINT16_USB_PID, &pid) != sizeof(pid))
		pid = 0;

	if (vid == 0) {
		vid = USB_CDC_DEVICE_VENDOR_ID;
		pid = USB_CDC_DEVICE_PRODUCT_ID;
	}

	udc_device_desc_cdc.idVendor = LE16(vid);
	udc_device_desc_cdc.idProduct = LE16(pid);
#endif	/* CONFIG_USE_CONFIGDB */

	memset(usb_cdc_states, 0, sizeof(usb_cdc_states));

	for (p = 0, ucs = usb_cdc_states; p < UDI_CDC_PORT_NB; p++, ucs++) {
		ucs->ucs_instance = p;
		ucs->ucs_state = 0;
		ucs->ucs_cfg = UART_DEFAULT_CFG;
		ucs->ucs_common_state = common_state;
		ucs->ucs_mutex = rtos_mutex_create();
		assert(ucs->ucs_mutex != NULL);
	}

	*cdc_state = usb_cdc_states;
	return &udc_config_cdc;
}
