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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "programmer.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "proto_jtag2.h"
#include "proto_stk500v2.h"
#include "prog_isp.h"
#include "prog_pdi.h"
#include "prog_updi.h"
#include "target_comms.h"
#include "network.h"
#include "rtos.h"
#include "display.h"
#include "timer.h"
#include "ringbuff.h"
#include "target_console.h"
#include "platform.h"
#include "shell.h"
#ifdef CONFIG_USE_USB
#include "usb_common.h"
#include "uart.h"
#endif
#include "avr_ll.h"
#include "hardware.h"
#include "product.h"

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	programmer_debug_flag
#include "debug.h"

#define	PROG_NETWORK_TIMEOUT	100	/* 100 milli-seconds */

SHELL_CMD_DECL(prog, programmer_cmd, "Configure the programmer");

/*
 * The programmer state
 */
static struct programmer_state programmer_state;

/*
 * Events recorded in programmer_state.ps_events
 */
#define	PROGRAMMER_EVENT_NET_CONNECT		(1u << 0)
#define	PROGRAMMER_EVENT_NET_DISCONNECT		(1u << 1)
#define	PROGRAMMER_EVENT_NET_MASK		0x000flu

#ifdef CONFIG_USE_USB
#define	PROGRAMMER_EVENT_USB_CDC_ATTACH		(1u << 4)
#define	PROGRAMMER_EVENT_USB_CDC_DETACH		(1u << 5)
#define	PROGRAMMER_EVENT_USB_CDC_CONNECT	(1u << 6)
#define	PROGRAMMER_EVENT_USB_CDC_HANGUP		(1u << 7)
#define	PROGRAMMER_EVENT_USB_CDC_MASK		0x00f0lu

#define PROGRAMMER_EVENT_USB_VENDOR_ENABLED	(1u << 8)
#define PROGRAMMER_EVENT_USB_VENDOR_DISABLED	(1u << 9)
#define PROGRAMMER_EVENT_USB_VENDOR_RX_READY	(1u << 10)
#define PROGRAMMER_EVENT_USB_VENDOR_RX_ERROR	(1u << 11)
#define PROGRAMMER_EVENT_USB_VENDOR_TX_ERROR	(1u << 12)
#define	PROGRAMMER_EVENT_USB_VENDOR_MASK	0x1f00lu
#endif /* CONFIG_USE_USB */

/*
 * These variables are updated with the current programming clock rate.
 * For ISP, this is the SPI SCK clock. For PDI, it's the RST pin clock.
 * The clock name variable identifies which is active.
 */
uint32_t programmer_curr_clock_rate;
uint32_t programmer_curr_clock_ticks;
const char *programmer_curr_clock_name;
#ifdef CONFIG_USE_CONFIGDB
char programmer_dev_id[PROGRAMMER_DEV_ID_SIZE];
#endif

static __inline void
programmer_set_events(struct programmer_state *ps, uint32_t ev)
{
	rtos_saved_ipl_t ipl;
	uint32_t old_events;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	old_events = ps->ps_events;
	ps->ps_events = old_events | ev;
	rtos_ipl_restore(ipl);

	if ((old_events & ev) == 0)
		rtos_notify_give_any(ps->ps_task);
}

static __inline uint32_t
programmer_get_events(struct programmer_state *ps)
{
	rtos_saved_ipl_t ipl;
	uint32_t events;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	events = ps->ps_events;
	ps->ps_events = 0;
	rtos_ipl_restore(ipl);

	return events;
}

static void
programmer_hardware_init(struct programmer_state *ps)
{

	prog_isp_quiesce(ps->ps_isp);
	prog_pdi_quiesce(ps->ps_pdi);
	prog_updi_quiesce(ps->ps_updi);

	target_console_control(true);
}

static int
programmer_ringbuff_blocking_open(struct programmer_state *ps)
{

	ps->ps_host2prog_blk = ringbuff_blocking_consumer(ps->ps_host2prog,
	    PROG_NETWORK_TIMEOUT);
	if (ps->ps_host2prog_blk == NULL)
		return -1;

	ps->ps_prog2host_blk = ringbuff_blocking_producer(ps->ps_prog2host,
	    PROG_NETWORK_TIMEOUT);
	if (ps->ps_prog2host_blk == NULL) {
		ringbuff_blocking_close(ps->ps_host2prog_blk);
		ps->ps_host2prog_blk = NULL;
		return -1;
	}

	return 0;
}

static void
programmer_ringbuff_blocking_close(struct programmer_state *ps)
{
	ringbuff_blocking_t rbl;
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	rbl = ps->ps_host2prog_blk;
	ps->ps_host2prog_blk = NULL;
	rtos_ipl_restore(ipl);

	if (rbl != NULL)
		ringbuff_blocking_close(rbl);

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	rbl = ps->ps_prog2host_blk;
	ps->ps_prog2host_blk = NULL;
	rtos_ipl_restore(ipl);

	if (rbl != NULL)
		ringbuff_blocking_close(rbl);
}

static void
programmer_socket_status_cb(network_sock_status_t st, void *arg)
{
	struct programmer_state *ps = arg;
	uint32_t event;

	switch (st) {
	case NETWORK_STATUS_CONNECT:
		event = PROGRAMMER_EVENT_NET_CONNECT;
		break;

	case NETWORK_STATUS_DISCONNECT:
	case NETWORK_STATUS_LINK_DOWN:
	case NETWORK_STATUS_TIMEOUT:
	case NETWORK_STATUS_UNREACHABLE:
		event = PROGRAMMER_EVENT_NET_DISCONNECT;
		break;

	default:
		event = 0;
		break;
	}

	if (event)
		programmer_set_events(ps, event);
}

#ifdef CONFIG_USE_USB
static void
programmer_usb_cdc_status_cb(usb_cdc_status_t st, void *arg)
{
	struct programmer_state *ps = arg;
	uint16_t event;

	/* Note: This is invoked from interrupt mode */

	switch (st) {
	case USB_CDC_STATUS_ATTACHED:
		/* USB interface has just been plugged into a host */
		event = PROGRAMMER_EVENT_USB_CDC_ATTACH;
		break;

	case USB_CDC_STATUS_DETACHED:
		/* USB interface has just been unplugged */
		event = PROGRAMMER_EVENT_USB_CDC_DETACH;
		break;

	case USB_CDC_STATUS_CONNECTED:
		/* User has opened the USB device on host */
		event = PROGRAMMER_EVENT_USB_CDC_CONNECT;
		break;

	case USB_CDC_STATUS_HANGUP:
		/* User has closed the USB device on host */
		event = PROGRAMMER_EVENT_USB_CDC_HANGUP;
		break;

	case USB_CDC_STATUS_CONFIG:
		/*
		 * User has changed the UART settings on host
		 * This makes no sense on the programmer interface.
		 */
		/*FALLTHROUGH*/

	default:
		event = 0;
		break;
	}

	if (event)
		programmer_set_events(ps, event);
}

static void
programmer_usb_vendor_status_cb(usb_vendor_status_t st, void *arg)
{
	struct programmer_state *ps = arg;
	uint16_t event;

	/* Note: This is invoked from interrupt mode */

	switch (st) {
	case USB_VENDOR_STATUS_ENABLE:
		/* USB interface has just been plugged into a host */
		event = PROGRAMMER_EVENT_USB_VENDOR_ENABLED;
		DBFPRINTF("ENABLED\n");
		break;

	case USB_VENDOR_STATUS_DISABLE:
		/* USB interface has just been unplugged */
		event = PROGRAMMER_EVENT_USB_VENDOR_DISABLED;
		DBFPRINTF("DISABLED\n");
		break;

	case USB_VENDOR_STATUS_RX_READY:
		/* A posted read has completed */
		event = PROGRAMMER_EVENT_USB_VENDOR_RX_READY;
		DBFPRINTF("RX READY\n");
		break;

	case USB_VENDOR_STATUS_RX_ERROR:
		/* A posted read has completed with an error */
		event = PROGRAMMER_EVENT_USB_VENDOR_RX_ERROR;
		DBFPRINTF("RX ERROR\n");
		break;

	case USB_VENDOR_STATUS_TX_ERROR:
		/* A posted write has completed with an error */
		event = PROGRAMMER_EVENT_USB_VENDOR_TX_ERROR;
		DBFPRINTF("TX ERROR\n");
		break;

	case USB_VENDOR_STATUS_TX_DONE:
		/*
		 * A posted write has completed. We're not really
		 * interested in these.
		 */
		DBFPRINTF("TX DONE\n");
		/*FALLTHROUGH*/

	default:
		event = 0;
		break;
	}

	if (event)
		programmer_set_events(ps, event);
}
#endif /* CONFIG_USE_USB */

uint32_t
programmer_request_get_byte(struct programmer_state *ps, uint8_t *prv)
{

	if (ps->ps_request_cursor == ps->ps_request_len)
		return 0;

	*prv = ps->ps_request[ps->ps_request_cursor++];

	return 1;
}

uint32_t
programmer_request_get_buff(struct programmer_state *ps, void *buff,
    uint32_t len)
{
	uint32_t remaining = ps->ps_request_len - ps->ps_request_cursor;

	if (remaining < len)
		len = remaining;

	if (len != 0) {
		memcpy(buff, &ps->ps_request[ps->ps_request_cursor], len);
		ps->ps_request_cursor += len;
	}

	return len;
}

uint32_t
programmer_reply_add_byte(struct programmer_state *ps, uint8_t ch)
{

	if ((ps->ps_reply_len + 1) < sizeof(ps->ps_reply)) {
		ps->ps_reply[ps->ps_reply_len++] = ch;
		return 1;
	}

	DBFPRINTF("overflow\n");
	return 0;
}

uint32_t
programmer_reply_add_buff(struct programmer_state *ps, const void *buff,
    uint32_t len)
{

	if ((ps->ps_reply_len + len) < sizeof(ps->ps_reply)) {
		memcpy(&ps->ps_reply[ps->ps_reply_len], buff, len);
		ps->ps_reply_len += len;
		return len;
	}

	DBFPRINTF("overflow\n");
	return 0;
}

static void
programmer_handle_net_events(struct programmer_state *ps, uint16_t events)
{

	if (events & PROGRAMMER_EVENT_NET_CONNECT) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_NOBODY:
			if (network_establish(ps->ps_socket, ps->ps_prog2host,
			    ps->ps_host2prog, 0, &ps->ps_peer) == 0) {
				ps->ps_owner = PROGRAMMER_OWNER_NETWORK;
				DBFPRINTF("network: connected\n");
			} else {
				DBFPRINTF("network: con-failed\n");
				network_close(ps->ps_socket);
				ps->ps_socket = NULL;
			}
			break;

		case PROGRAMMER_OWNER_NETWORK:
			/*
			 * This should never happen - the socket API permits
			 * only one connection on any socket.
			 */
			DBFPRINTF("network: already owner!\n");
			break;

		default:
			/*
			 * Happens if USB owns the uart and someone tries
			 * to connect over the network.
			 */
			DBFPRINTF("network: busy!\n");
			network_close(ps->ps_socket);
			ps->ps_socket = NULL;
			break;
		}
	}

	if (events & PROGRAMMER_EVENT_NET_DISCONNECT) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_NETWORK:
			DBFPRINTF("network: closed\n");
			if (ps->ps_socket) {
				/* Should assert ps->ps_socket != NULL */
				network_close(ps->ps_socket);
				ps->ps_socket = NULL;
			}
			ps->ps_owner = PROGRAMMER_OWNER_NOBODY;
			break;

		default:
			/*
			 * Can happen if the link goes down. We may still
			 * have a valid listen socket.
			 */
			DBFPRINTF("network: disconnect but not owner. "
			    "Link down?\n");
			if (ps->ps_socket != NULL) {
				network_close(ps->ps_socket);
				ps->ps_socket = NULL;
			}
			break;
		}
	}
}

#ifdef CONFIG_USE_USB
static void
programmer_handle_usb_cdc_events(struct programmer_state *ps, uint16_t events)
{

	if (events & PROGRAMMER_EVENT_USB_CDC_ATTACH) {
		/* Not really of any interest */
		DBFPRINTF("CDC attached\n");
	}

	if (events & PROGRAMMER_EVENT_USB_CDC_DETACH) {
		/* Not really of any interest */
		DBFPRINTF("CDC detached\n");
	}

	if (events & PROGRAMMER_EVENT_USB_CDC_CONNECT) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_NOBODY:
			if (usb_cdc_open(ps->ps_usb, UART_DEFAULT_CFG, 0,
			    ps->ps_prog2host, ps->ps_host2prog) >= 0) {
				DBFPRINTF("CDC connected\n");

				/* USB owns the programmer */
				ps->ps_owner = PROGRAMMER_OWNER_USB_CDC;
			} else {
				DBFPRINTF("CDC usb_open() failed\n");
			}
			break;

		case PROGRAMMER_OWNER_USB_CDC:
			/* XXX: Can't happen. Need to get working assert() */
			DBFPRINTF("double connect!\n");
			break;

		default:
			/* Network/USB Vendor has it open already */
			DBFPRINTF("busy\n");
			break;
		}
	}

	if (events & PROGRAMMER_EVENT_USB_CDC_HANGUP) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_USB_CDC:
			if ((events & PROGRAMMER_EVENT_USB_CDC_CONNECT) == 0) {
				DBFPRINTF("closed\n");
				usb_cdc_close(ps->ps_usb);
				ps->ps_owner = PROGRAMMER_OWNER_NOBODY;
			} else {
				DBFPRINTF("ignoring hangup\n");
			}
			break;

		case PROGRAMMER_OWNER_NETWORK:
		case PROGRAMMER_OWNER_USB_VENDOR:
			DBFPRINTF("non-owner closed\n");
			break;

		default:
			DBFPRINTF("closed with no owner\n");
			break;
		}
	}
}

static void
programmer_handle_usb_vendor_events(struct programmer_state *ps,
    uint16_t events)
{

	if (events & PROGRAMMER_EVENT_USB_VENDOR_ENABLED) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_NOBODY:
			DBFPRINTF("vendor enabled\n");
			/* USB owns the programmer */
			ps->ps_owner = PROGRAMMER_OWNER_USB_VENDOR;

			/* Post the first read */
			ps->ps_usb_rx_len = 0;
			usb_vendor_post_read(ps->ps_usb,
			    ps->ps_request, &ps->ps_usb_rx_len,
			    PROGRAMMER_MAX_MESSAGE_LEN);
			break;

		case PROGRAMMER_OWNER_USB_VENDOR:
			/* XXX: Can't happen. */
			DBFPRINTF("double connect!\n");
			break;

		default:
			/* Network/USB CDC has it open already */
			DBFPRINTF("usb: busy\n");
			break;
		}
	}

	if (events & (PROGRAMMER_EVENT_USB_VENDOR_DISABLED |
	    PROGRAMMER_EVENT_USB_VENDOR_RX_ERROR |
	    PROGRAMMER_EVENT_USB_VENDOR_TX_ERROR)) {
		switch (ps->ps_owner) {
		case PROGRAMMER_OWNER_USB_VENDOR:
			DBFPRINTF("vendor closed\n");
			usb_vendor_close(ps->ps_usb);
			ps->ps_usb = NULL;
			ps->ps_owner = PROGRAMMER_OWNER_NOBODY;
			break;

		case PROGRAMMER_OWNER_NETWORK:
		case PROGRAMMER_OWNER_USB_CDC:
			DBFPRINTF("non-owner closed\n");
			break;

		default:
			DBFPRINTF("closed with no owner\n");
			break;
		}
	}

	if (events & PROGRAMMER_EVENT_USB_VENDOR_RX_READY) {
		/* Handled by the mainline code */
		DBFPRINTF("Rx ready (%u)\n", ps->ps_usb_rx_len);
	}
}
#endif /* CONFIG_USE_USB */

void
programmer_task(rtos_task_t me)
{
	struct programmer_state *ps = &programmer_state;
	const struct programmer_protocol *pp;
	int retry, previous_owner, want_header, len;
	uint32_t events, want_len, reply_len, reply_cursor;
	struct target_comms_config tcc;
	bool rv;

	ps->ps_task = me;
	programmer_hardware_init(ps);

	rtos_task_priority_set(me, RTOS_TASK_PRIORITY_MED);

	rv = target_comms_get_config(TARGET_COMMS_PORT_AVR_PROGRAMMER, &tcc);
	assert(rv == true);
	(void)rv;
	assert(tcc.tcc_tcp_port != TARGET_COMMS_TCP_PORT_UNUSED);
#ifdef CONFIG_USE_USB
	assert(tcc.tcc_usb_port != TARGET_COMMS_USB_PORT_UNUSED);
#endif

	pp = ps->ps_protocol;
	want_header = 1;
	want_len = 0;
	reply_cursor = 0;
	reply_len = 0;

	/* If this fails, we have other problems. */
	int x = programmer_ringbuff_blocking_open(ps);
	assert(x >= 0);
	(void) x;

	for (;;) {
		previous_owner = ps->ps_owner;
		retry = 0;

		while ((events = programmer_get_events(ps)) != 0) {
			DBFPRINTF("events %04lx\n", events);

			if (events & PROGRAMMER_EVENT_NET_MASK)
				programmer_handle_net_events(ps, events);

#ifdef CONFIG_USE_USB
			if (events & PROGRAMMER_EVENT_USB_CDC_MASK)
				programmer_handle_usb_cdc_events(ps, events);

			if (events & PROGRAMMER_EVENT_USB_VENDOR_MASK)
				programmer_handle_usb_vendor_events(ps, events);
#endif
		}

		if (ps->ps_socket == NULL) {
			ps->ps_socket = network_socket_listen(tcc.tcc_tcp_port,
			    programmer_socket_status_cb, ps);
			DBFPRINTF("new socket %p\n", ps->ps_socket);
			if (ps->ps_socket == NULL)
				retry = 1;
		}

#ifdef CONFIG_USE_USB
		if (ps->ps_usb == NULL) {
			ps->ps_usb = usb_cdc_claim(tcc.tcc_usb_port,
			    programmer_usb_cdc_status_cb, ps);
			DBFPRINTF("usb CDC claim %p\n", ps->ps_usb);
			if (ps->ps_usb == NULL) {
				ps->ps_usb = usb_vendor_open(
				    programmer_usb_vendor_status_cb, ps);
				DBFPRINTF("usb VENDOR open %p\n",
				    ps->ps_usb);
				if (ps->ps_usb == NULL)
					retry = 1;
			}
		}
#endif

		if (ps->ps_owner != PROGRAMMER_OWNER_NOBODY) {
			if (previous_owner != ps->ps_owner) {
				/*
				 * A new connection. Grab the protocol handler
				 * pointer here in case a wily user tries to
				 * change it mid-connection.
				 */
				pp = ps->ps_protocol;
				want_len = 0;
				reply_len = 0;
				DBFPRINTF("new %s connection\n",
				    (ps->ps_owner == PROGRAMMER_OWNER_NETWORK) ?
				    "network" : "usb");

				/*
				 * Disable target console for the duration
				 * of the connection - some targets reuse
				 * programming pins for Debug Tx/Rx, so
				 * we don't want contention.
				 */
				target_console_control(false);
			}
		} else
		if (previous_owner != PROGRAMMER_OWNER_NOBODY) {
			/*
			 * Looks like the host disconnected. Get back
			 * to a known state.
			 */
			DBFPRINTF("%s connection closed\n",
			    (previous_owner == PROGRAMMER_OWNER_NETWORK) ?
			    "network" : "usb");

			/* Tell the protocol handler, if necessary */
			if (pp->pp_closed != NULL)
				(pp->pp_closed)(ps);

			/* Make sure ringbuff blocking I/O is closed */
			programmer_ringbuff_blocking_close(ps);

			/* Clear the ring buffers */
			ringbuff_init(ps->ps_prog2host);
			ringbuff_init(ps->ps_host2prog);

			/* Re-open: should never fail */
			x = programmer_ringbuff_blocking_open(ps);
			assert(x >= 0);
			(void) x;

			/* Get the hardware to a known state */
			programmer_hardware_init(ps);
		}

		/*
		 * If there's no connection, either block until we have
		 * one or sleep for a bit if we're still trying to get
		 * a network socket or claim the USB port.
		 */
		if (ps->ps_owner == PROGRAMMER_OWNER_NOBODY) {
			if (retry) {
				DBFPRINTF("sleep\n");
				rtos_task_sleep(1000u);
			} else {
				DBFPRINTF("block\n");
				rtos_notify_take(1);
			}
			continue;
		}

		if (reply_len) {
			/*
			 * We're writing out the reply to a previous
			 * request.
			 */
			DBFPRINTF("writing %lu reply bytes\n",
			    reply_len - reply_cursor);

#ifdef CONFIG_USE_USB
			if (ps->ps_owner == PROGRAMMER_OWNER_USB_VENDOR) {
				/*
				 * USB vendor API doesn't use ring buffers.
				 * Before writing the response, post the
				 * read for the next request.
				 */
				/* Now write out response */
				DBFPRINTF("post write\n");
				usb_vendor_post_write(ps->ps_usb,
				    ps->ps_reply, reply_len);

				rtos_task_sleep(50);

				/*
				 * The write will complete asynchronously
				 * so we go back to waiting for a request.
				 */
				reply_len = 0;
				want_len = 0;

				DBFPRINTF("post read after write\n");
				ps->ps_usb_rx_len = 0;
				usb_vendor_post_read(ps->ps_usb,
				    ps->ps_request, &ps->ps_usb_rx_len,
				    PROGRAMMER_MAX_MESSAGE_LEN);
			} else
#endif /* CONFIG_USE_USB */
			{
				len = ringbuff_blocking_write(
				    ps->ps_prog2host_blk,
				    &ps->ps_reply[reply_cursor],
				    reply_len - reply_cursor);

				if (len < 0) {
					/* Should never happen! */
					DBFPRINTF("write error\n");
					reply_len = 0;
				} else {
					/* Update how much has been written */
					DBFPRINTF("wrote %d bytes\n", len);
					reply_cursor += len;
					reply_len -= len;
					ringbuff_produce_done(ps->ps_prog2host);
				}

				/* If there's still more, keep going */
				if (reply_len)
					continue;

				/*
				 * Otherwise back to reading some data
				 * from the host.
				 */
				want_len = 0;
			}
		}

		if (want_len == 0) {
			DBFPRINTF("want_len == 0. resetting\n\n");
#ifdef CONFIG_USE_USB
			if (ps->ps_owner == PROGRAMMER_OWNER_USB_VENDOR) {
				want_header = 0;
				want_len = PROGRAMMER_MAX_MESSAGE_LEN;
			} else
#endif	/* CONFIG_USE_USB */
			{
				want_header = 1;
				want_len = pp->pp_header_len;
			}
			ps->ps_request_len = 0;
			ps->ps_request_cursor = 0;
			ps->ps_reply_len = 0;
			reply_cursor = 0;
			reply_len = 0;
		}

		/*
		 * Go see if there's any work to do.
		 */
#ifdef CONFIG_USE_USB
		if (ps->ps_owner == PROGRAMMER_OWNER_USB_VENDOR) {
			/*
			 * The USB vendor API will callback when a complete
			 * Rx frame has been received. So just block for a
			 * while.
			 */
			rtos_notify_take_timed(1, 1000);
			len = (int) ps->ps_usb_rx_len;
			ps->ps_usb_rx_len = 0;
		} else
#endif /* CONFIG_USE_USB */
		{
			len = ringbuff_blocking_read(ps->ps_host2prog_blk,
			    &ps->ps_request[ps->ps_request_len],
			    want_len - ps->ps_request_len);

			if (len < 0) {
				/* Shouldn't happen */
				DBFPRINTF("read error\n");
				want_len = 0;
				continue;
			}

			if (len != 0) {
				DBFPRINTF("read %d %s bytes\n", len,
				    want_header ? "header" : "body");
			}
		}

		want_len -= len;
		ps->ps_request_len += len;

		/* Did we get enough */
		if (want_len == 0
#ifdef CONFIG_USE_USB
		    || (len > 0 && ps->ps_owner == PROGRAMMER_OWNER_USB_VENDOR)
#endif
		    ) {
			/* We have all the data needed */
			want_len = 0;
			if (want_header) {
				DBFPRINTF("got header\n");

				/* Parse the header */
				want_len = (pp->pp_got_header)(ps);

				DBFPRINTF("want body len %lu\n", want_len);

				/* Catch corruption */
				if (want_len > sizeof(ps->ps_request))
					want_len = 0;

				/*
				 * Return of zero indicates the
				 * header didn't checkout so try
				 * again for another header.
				 */
				if (want_len != 0) {
					want_header = 0;
				}

				ps->ps_request_len = 0;
				ps->ps_request_cursor = 0;
			} else {
				/*
				 * Parse the body and provide the reply.
				 * If there is no reply, we'll revert to
				 * header mode later because want_len == 0.
				 */
				DBFPRINTF("got body. len %d\n", len);

#ifdef CONFIG_USE_USB
				if (ps->ps_owner ==
				    PROGRAMMER_OWNER_USB_VENDOR) {
					(pp->pp_got_frame)(ps);
				} else
#endif
				{
					(pp->pp_got_body)(ps);
				}

				reply_len = ps->ps_reply_len;

				DBFPRINTF("reply length is %lu\n", reply_len);
			}
		}
	}
}

static const char programmer_usage[] = {
	"usage:\n"
	"prog\n"
	"\tDisplay current status\n"
	"prog rclk <rescue-clock-frequency>\n"
	"\tSet the rescue clock (pin 8 of " PRODUCT_NAME "'s 10-pin target "
	  "header) to the\n"
	"\tspecified frequency. Supported rates are 1-%u, where 0\n"
	"\tdisables the rescue clock. Note that the actual rate will be as\n"
	"\tclose as possible to the requested frequency but due to timer\n"
	"\tconstraints it will usually not be exact.\n"
	"prog mode [<wavr> | <dragon> | <stk500>]\n"
	"\tSet protocol supported over the network.\n"
	"\t\"wavr\" will be supported in a future version of avrdude and will\n"
	"\tpermit ISP, PDI and uPDI targets to be programmed by your "
	  PRODUCT_NAME " device.\n"
	"\t\"dragon\" is suitable for programming ISP and PDI targets via\n"
	"\tavrdude (at least version 6.3). " PRODUCT_NAME " will identify "
	  "itself as an\n"
	"\tAVR Dragon in this case.\n"
	"\t\"stk500\" is provided for Atmel Studio compatibility, although\n"
	"\tits use is discouraged as sadly Atmel Studio does not support\n"
	"\tprogramming in either STK500 mode or Dragon mode. In fact it\n"
	"\tdoesn't know that either of these devices can be reached over the\n"
	"\tnetwork, so " PRODUCT_NAME " is best used as an external "
	  "programmer. If you\n"
	"\treally want to use " PRODUCT_NAME " on Atmel Studio as a native "
	  "programmer,\n"
	"\tthen you need to set the device ID to \"STK600\" using the \n"
	"\t\"prog stkid STK600\" command and connect via USB with " PRODUCT_NAME
	  "'s USB mode\n"
	"\tset to \"stk600\". This is not a supported configuration and we "
	  "may\n"
	"\tremove USB emulation of STK600 in a future firmware update.\n"
	"prog stkid <stk500-id-string>\n"
	"\tSet the ID string reported to the host when the stk500 mode is\n"
	"\tselected. Default is \"STK500_2. The appropriate ID will be chosen\n"
	"\tautomatically for the other two modes.\n"
};

static void
programmer_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct programmer_state *ps = &programmer_state;
	uint8_t changed = 0;
	uint8_t prog_proto;
	char tvcc[16];

	prog_proto = programmer_get_proto();

	if (argc == 2 && strcasecmp(argv[0], "rclk") == 0) {
		char *p;
		uint32_t rclk;

		rclk = strtoul(argv[1], &p, 10);
		if (p != NULL && *p == '\0') {
			if (rclk != 0 && rclk <= PLATFORM_MAX_RESCUE_CLK) {
				fprintf(os, "Clock: %lu\n", rclk);
				ps->ps_rescue_clock = rclk;
				avr_ll_isp_clk_start(rclk);
			} else
			if (rclk == 0) {
				fprintf(os, "Clock stopped\n");
				ps->ps_rescue_clock = 0;
				avr_ll_isp_clk_stop();
			} else {
				fprintf(os, "Clock must be in the range "
				    "0 - %u\n", PLATFORM_MAX_RESCUE_CLK);
			}
		}
	} else
	if (argc == 2 && strcasecmp(argv[0], "mode") == 0) {
		uint8_t new_proto = prog_proto;

		if (strcasecmp(argv[1], "dragon") == 0) {
			ps->ps_protocol = &proto_jtag2_protocol;
			new_proto = PROG_PROTO_DRAGON;
		} else
		if (strcasecmp(argv[1], "wavr") == 0 ||
		    strcasecmp(argv[1], "native") == 0) {
			ps->ps_protocol = &proto_jtag2_protocol;
			new_proto = PROG_PROTO_WAVR;
		} else
		if (strcasecmp(argv[1], "stk500") == 0) {
			ps->ps_protocol = &proto_stk500v2_protocol;
			new_proto = PROG_PROTO_STK500V2;
		} else {
			fprintf(os, "Mode not recognised. Must be one of "
			    "\"wavr\", \"dragon\", or \"stk500\"\n");
		}

		if (prog_proto != new_proto) {
#ifndef CONFIG_USE_CONFIGDB
			glob.glob_programmer_proto = new_proto;
#else
			cf_set_uint8(CF_KEY_UINT8_PROGRAMMER_PROTO, new_proto);
#endif
			changed = 1;
			prog_proto = new_proto;
		}
	} else
	if (argc == 2 && strcasecmp(argv[0], "stkid") == 0) {
		unsigned int l = strlen(argv[1]);

		if (l > PROGRAMMER_DEV_ID_SIZE || l == 0) {
			fprintf(os, "Id must be between 1 and %u characters "
			    "in length\n", PROGRAMMER_DEV_ID_SIZE - 1);
		} else
		if (strcmp(PROGRAMMER_DEV_ID, argv[1])) {
			strlcpy(PROGRAMMER_DEV_ID, argv[1],
			    PROGRAMMER_DEV_ID_SIZE);
#ifdef CONFIG_USE_CONFIGDB
			cf_set_string(CF_KEY_STRING_PROGRAMMER_DEV_ID, argv[1]);
#endif
			changed = 1;
		}
	} else
#ifdef DEBUG_ENABLED
	if (argc == 2 && strcasecmp(argv[0], "debug") == 0) {
		if (strcasecmp(argv[1], "on") == 0) {
			DEBUG_FLAG = 1;
		} else
		if (strcasecmp(argv[1], "off") == 0) {
			DEBUG_FLAG = 0;
		}
	} else
#endif /* DEBUG_ENABLED */
	if (argc != 0) {
		fprintf(os, programmer_usage, PLATFORM_MAX_RESCUE_CLK);
		return;
	}

	fprintf(os, "Programmer mode: %s\n",
	    (prog_proto == PROG_PROTO_WAVR) ?
	    "wavr" : ((prog_proto == PROG_PROTO_DRAGON) ?
	    "dragon" : "stk500"));
	fprintf(os, "Programmer STK500 ID: %s\n", PROGRAMMER_DEV_ID);
	if (ps->ps_rescue_clock == 0)
		fprintf(os, "Rescue clock: Off\n");
	else
		fprintf(os, "Rescue clock: %lu\n", ps->ps_rescue_clock);
	fix16_to_str(power_target_vcc, tvcc, 2);
	fprintf(os, "TVcc: %s\n", tvcc);
#ifdef DEBUG_ENABLED
	fprintf(os, "Debug state is: %s\n", DEBUG_FLAG ? "On" : "Off");
#endif

	if (changed) {
#ifndef CONFIG_USE_CONFIGDB
		glob_save();
#else
		configdb_persist();
#endif
	}
}

const char *
programmer_status(uint32_t *pclk)
{

	*pclk = programmer_curr_clock_rate;
	return programmer_curr_clock_name;
}

void
programmer_status_tick(void)
{

	if (programmer_curr_clock_ticks != 0 &&
	    --programmer_curr_clock_ticks == 0) {
		programmer_curr_clock_name = NULL;
	}
}

void
programmer_init(void)
{
	struct programmer_state *ps = &programmer_state;
	uint8_t prog_proto;

	prog_proto = programmer_get_proto();

#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_string(CF_KEY_STRING_PROGRAMMER_DEV_ID,
	    sizeof(PROGRAMMER_DEV_ID), PROGRAMMER_DEV_ID) == 0) {
		strcpy(PROGRAMMER_DEV_ID, "STK500_2");
	}
#endif

	/*
	 * Initialise the programming hardware
	 */
	avr_ll_init_target();

	/*
	 * Initialise low-level programmers
	 */
	ps->ps_isp = prog_isp_init(ps);
	ps->ps_pdi = prog_pdi_init(ps);
	ps->ps_updi = prog_updi_init(ps);

	/*
	 * Initialise protocol handlers
	 */
	proto_jtag2_init();
	proto_stk500v2_init();

	/*
	 * Initialise the target console.
	 */
	target_comms_init();
	target_comms_start();

	/*
	 * Configure the default protocol handlers
	 */
	if (prog_proto == PROG_PROTO_STK500V2)
		ps->ps_protocol = &proto_stk500v2_protocol;
	else
		ps->ps_protocol = &proto_jtag2_protocol;

	/*
	 * Register the 'prog' command for programmer configuration
	 */
	SHELL_CMD_ADD(prog);

	/*
	 * Create ring buffers for communication with host
	 */
	ps->ps_prog2host = ringbuff_alloc(ps->ps_rb_prog2host,
	    sizeof(ps->ps_rb_prog2host));
	assert(ps->ps_prog2host != NULL);

	ps->ps_host2prog = ringbuff_alloc(ps->ps_rb_host2prog,
	    sizeof(ps->ps_rb_host2prog));
	assert(ps->ps_host2prog != NULL);

	/*
	 * No-one "owns" the programming interface
	 */
	ps->ps_owner = PROGRAMMER_OWNER_NOBODY;
}

uint8_t
programmer_get_proto(void)
{
	uint8_t proto;

#ifndef CONFIG_USE_CONFIGDB
	proto = glob.glob_programmer_proto;
#else
	if (cf_get_uint8(CF_KEY_UINT8_PROGRAMMER_PROTO, &proto) == 0)
		proto = PROG_PROTO_DRAGON;
#endif

	return proto;
}

void
programmer_load_defaults(void)
{

	strcpy(PROGRAMMER_DEV_ID, "STK500_2");

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_programmer_proto = PROG_PROTO_DRAGON;
#else
	cf_set_string(CF_KEY_STRING_PROGRAMMER_DEV_ID, PROGRAMMER_DEV_ID);
	cf_set_uint8(CF_KEY_UINT8_PROGRAMMER_PROTO, PROG_PROTO_DRAGON);
#endif

	prog_isp_load_defaults();
	prog_pdi_load_defaults();
	prog_updi_load_defaults();
}
