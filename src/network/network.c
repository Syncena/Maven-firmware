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
#include <inttypes.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rtos.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "network.h"
#include "ringbuff.h"
#include "linked-lists.h"
#include "zone_alloc.h"
#ifndef NETWORK_OPT_NO_TELNET
#include "telnet.h"
#endif
#ifdef NETWORK_OPT_DHCP_CLIENT
#include "native_dhcp.h"
#endif
#ifdef NETWORK_OPT_COMMAND
#include "shell.h"
#endif

//#define DEBUG_FLAG_INIT 1
#include "debug.h"

#ifndef NETWORK_OPT_NO_TELNET
#ifndef NETWORK_TELNET_BUFF
#ifdef RINGBUFF_SIZE
#define	NETWORK_TELNET_BUFF	RINGBUFF_SIZE
#else
#error "NETWORK_TELNET_BUFF must be defined if using custom ringbuff size"
#endif	/* RINGBUFF_SIZE */
#endif	/* NETWORK_TELNET_BUFF */
#endif /* NETWORK_OPT_NO_TELNET */

struct network_state;

#ifndef NETWORK_OPT_NO_TELNET
struct network_telnet_state {
	struct telnet_state nts_telnet;
	bool nts_established;
	uint8_t nts_telnet_buff[NETWORK_TELNET_BUFF];
};
#endif /* NETWORK_OPT_NO_TELNET */

struct network_socket {
	ringbuff_t s_rb_nw2app;		/* Data from network -> app */
	ringbuff_t s_rb_app2nw;		/* Data from app -> network */
	network_sock_t s_dsock;		/* Associated driver socket */
	uint32_t s_dip;
	uint16_t s_port;
	uint16_t s_pending;
#define	NETWORK_SOCK_WORK_PENDING	1
#define	NETWORK_SOCK_RETRY_PENDING	2
	TAILQ_ENTRY(network_socket) s_qent;
	network_sock_type_t s_type;
	network_sock_status_callback_t s_cb;
	void *s_cbarg;
	volatile bool s_connected;
	struct network_state *s_driver;
	rtos_mutex_t s_mutex;
#ifndef NETWORK_OPT_NO_TELNET
	struct network_telnet_state *s_telnet;
#endif
};

TAILQ_HEAD(network_socket_qhead, network_socket);

#define	NETWORK_LINK_STATUS_SLOTS	4
#define	NETWORK_ACTIVITY_SLOTS		1

struct network_state {
	const struct network_driver *ns_driver;

	unsigned int ns_state;
#define	NETWORK_STATE_DOWN		0
#define	NETWORK_STATE_RUNNING		1
#define	NETWORK_STATE_SUSPENDED		2

#define	NETWORK_EVENT_INTERRUPT		(1u << 0)
#define	NETWORK_EVENT_LINK		(1u << 1)
#define	NETWORK_EVENT_LINK_STATUS	(1u << 2)
#define	NETWORK_EVENT_SUSPEND		(1u << 3)
#define	NETWORK_EVENT_IP_CONFIGURE	(1u << 4)

	bool ns_link_up;
	struct network_suspend_args ns_suspend_args;
	network_link_status_t ns_link_status;
	network_ip_params_t ns_ip_params;
	struct {
		network_link_status_callback_t nsl_cb;
		void *nsl_arg;
	} ns_link_status_hooks[NETWORK_LINK_STATUS_SLOTS];
	struct {
		network_activity_callback_t nsa_cb;
		void *nsa_arg;
	} ns_activity_hooks[NETWORK_ACTIVITY_SLOTS];
	struct network_socket_qhead ns_sockq;
	struct network_socket_qhead ns_sock_retryq;
	struct network_socket ns_sockets[NETWORK_NSOCKETS];
#if (RELEASE_BUILD == 0)
	rtos_task_t ns_driver_mutex_owner;
#endif
	rtos_task_t ns_driver_task;
	rtos_task_t ns_socket_task;
	rtos_mutex_t ns_driver_mutex;
	rtos_timer_t ns_soft_timer;
	rtos_timer_t ns_link_timer;
	rtos_timer_t ns_retry_timer;
};
static struct network_state network_state;

#ifndef NETWORK_LWIP
#define	NETWORK_DRIVER_STACK_SIZE	RTOS_STACK_SIZE(1400)
#define	NETWORK_SOCKET_STACK_SIZE	RTOS_STACK_SIZE(1024)
#else
#define	NETWORK_DRIVER_STACK_SIZE	RTOS_STACK_SIZE(2000)
#define	NETWORK_SOCKET_STACK_SIZE	RTOS_STACK_SIZE(1400)
#endif

#ifdef NETWORK_OPT_COMMAND
SHELL_CMD_DECL(net, network_cmd, "Displays network status");
#endif

static void network_socket_attention(struct network_socket *s);
static void network_ip_configure(struct network_state *ns);

#define	network_socket_thread_wake(s)	\
		rtos_notify_give_any((s)->s_driver->ns_socket_task)
static __always_inline void
network_mutex_acquire(rtos_mutex_t m)
{

	rtos_mutex_acquire(m);
}

static __always_inline void
network_mutex_release(rtos_mutex_t m)
{

	rtos_mutex_release(m);
}

void
network_assert_driver_locked(void)
{
#if (RELEASE_BUILD == 0)
	struct network_state *ns = &network_state;

	if (ns->ns_driver == NULL)
		return;

	assert(_rtos_interrupt_context() == 0);

	assert(ns->ns_driver_mutex_owner == rtos_task_current());
#endif
}

static  void
network_mutex_acquire_driver(struct network_state *ns)
{
#if (RELEASE_BUILD == 0)
	rtos_task_t me;

	assert(ns->ns_driver != NULL);
	assert(_rtos_interrupt_context() == 0);
	me = rtos_task_current();
	assert(ns->ns_driver_mutex_owner != me);
#endif

	rtos_mutex_acquire(ns->ns_driver_mutex);

#if (RELEASE_BUILD == 0)
	assert(ns->ns_driver_mutex_owner == NULL);
	ns->ns_driver_mutex_owner = me;
#endif
}

static void
network_mutex_release_driver(struct network_state *ns)
{
#if (RELEASE_BUILD == 0)
	assert(ns->ns_driver != NULL);
	assert(_rtos_interrupt_context() == 0);
	assert(ns->ns_driver_mutex_owner == rtos_task_current());
	ns->ns_driver_mutex_owner = NULL;
#endif

	rtos_mutex_release(ns->ns_driver_mutex);
}

static void
network_socket_attention(struct network_socket *s)
{
	rtos_saved_ipl_t ipl;
	int do_notify = 0;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	if (s->s_pending == 0) {
		s->s_pending = do_notify = NETWORK_SOCK_WORK_PENDING;
		TAILQ_INSERT_TAIL(&s->s_driver->ns_sockq, s, s_qent);
	}
	rtos_ipl_restore(ipl);

	if (do_notify)
		network_socket_thread_wake(s);
}

static void
network_retry_timer_event(rtos_timer_t t)
{
	struct network_state *ns = rtos_timer_get_id(t);
	struct network_socket *s;
	rtos_saved_ipl_t ipl;
	int do_notify = 0;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	while ((s = TAILQ_FIRST(&ns->ns_sock_retryq)) != NULL) {
		TAILQ_REMOVE(&ns->ns_sock_retryq, s, s_qent);
		TAILQ_INSERT_TAIL(&ns->ns_sockq, s, s_qent);
		s->s_pending = do_notify = NETWORK_SOCK_WORK_PENDING;
	}
	rtos_ipl_restore(ipl);

	if (do_notify)
		rtos_notify_give(ns->ns_socket_task);
}

static __always_inline bool
network_running(struct network_state *ns)
{

	return ns->ns_state == NETWORK_STATE_RUNNING;
}

static __always_inline int8_t
network_drv_ioctl(struct network_state *ns, uint8_t cmd, void *arg)
{
	int8_t rv = 0;

	if (ns->ns_driver->nd_ioctl != NULL) {
		rv = (ns->ns_driver->nd_ioctl)(ns->ns_driver->nd_cookie,
		    cmd, arg);
	}

	return rv;
}

static __always_inline void
network_drv_get_mac(struct network_state *ns, uint8_t *mac)
{

	network_mutex_acquire_driver(ns);
	(ns->ns_driver->nd_get_mac)(ns->ns_driver->nd_cookie, mac);
	network_mutex_release_driver(ns);
}

static __always_inline void
network_drv_set_ip_params(struct network_state *ns,
    const network_ip_params_t *ip)
{
	network_mutex_acquire_driver(ns);
	(ns->ns_driver->nd_set_ip_params)(ns->ns_driver->nd_cookie, ip);
	network_mutex_release_driver(ns);
}

static __always_inline network_sock_t
network_drv_open(struct network_state *ns, network_sock_type_t t,
    network_sock_status_callback_t callback, void *arg)
{
	network_sock_t s;

	/* Note: must be called with ns_driver_mutex held */
	assert(ns->ns_driver_mutex_owner == rtos_task_current());

	s = (ns->ns_driver->nd_open)(ns->ns_driver->nd_cookie, t, callback,arg);

	return s;
}

static __always_inline void
network_drv_close(struct network_state *ns, network_sock_t sn)
{

	/* Note: must be called with ns_driver_mutex held */
	assert(ns->ns_driver_mutex_owner == rtos_task_current());

	(ns->ns_driver->nd_close)(ns->ns_driver->nd_cookie, sn);
}

static __always_inline network_sock_state_t
network_drv_sock_status(struct network_state *ns, network_sock_t sn,
    network_sock_params_t *sp)
{
	network_sock_state_t s;

	network_mutex_acquire_driver(ns);
	s = (ns->ns_driver->nd_sock_status)(ns->ns_driver->nd_cookie, sn, sp);
	network_mutex_release_driver(ns);

	return s;
}

static __always_inline uint8_t
network_drv_connect(struct network_state *ns, network_sock_t sn,
    const network_sock_params_t *sp)
{
	uint8_t rv;

	/* Note: must be called with ns_driver_mutex held */
	assert(ns->ns_driver_mutex_owner == rtos_task_current());

	rv = (ns->ns_driver->nd_connect)(ns->ns_driver->nd_cookie, sn, sp);

	return rv;
}

static __always_inline uint8_t
network_drv_listen(struct network_state *ns, network_sock_t sn, uint16_t pn)
{
	uint8_t rv;

	/* Note: must be called with ns_driver_mutex held */
	assert(ns->ns_driver_mutex_owner == rtos_task_current());

	rv = (ns->ns_driver->nd_listen)(ns->ns_driver->nd_cookie, sn, pn);

	return rv;
}

static __always_inline uint16_t
network_drv_read(struct network_state *ns, network_sock_t sn, void *buff,
    uint16_t len)
{
	uint16_t rv;

	network_mutex_acquire_driver(ns);
	rv = (ns->ns_driver->nd_read)(ns->ns_driver->nd_cookie, sn, buff, len);
	network_mutex_release_driver(ns);

	return rv;
}

static __always_inline uint16_t
network_drv_recv(struct network_state *ns, network_sock_t sn, void *buff,
    uint16_t len, network_sock_params_t *sp)
{
	uint16_t rv;

	network_mutex_acquire_driver(ns);
	rv = (ns->ns_driver->nd_recv)(ns->ns_driver->nd_cookie, sn, buff, len,
	    sp);
	network_mutex_release_driver(ns);

	return rv;
}

static __always_inline uint16_t
network_drv_write(struct network_state *ns, network_sock_t sn, const void *buff,
    uint16_t len, const uint8_t *dstmac, bool push)
{
	uint16_t rv;

	network_mutex_acquire_driver(ns);
	rv = (ns->ns_driver->nd_write)(ns->ns_driver->nd_cookie, sn, buff, len,
	    dstmac, push);
	network_mutex_release_driver(ns);

	return rv;
}

static __always_inline uint16_t
network_drv_rx_available(struct network_state *ns, network_sock_t sn)
{
	uint16_t rv;

	network_mutex_acquire_driver(ns);
	rv = (ns->ns_driver->nd_rx_avail)(ns->ns_driver->nd_cookie, sn);
	network_mutex_release_driver(ns);

	return rv;
}

static __always_inline uint16_t
network_drv_tx_free(struct network_state *ns, network_sock_t sn)
{
	uint16_t rv;

	network_mutex_acquire_driver(ns);
	rv = (ns->ns_driver->nd_tx_free)(ns->ns_driver->nd_cookie, sn);
	network_mutex_release_driver(ns);

	return rv;
}

static __always_inline void
network_drv_link_status(struct network_state *ns,
    network_link_status_callback_t cb, void *cbarg)
{

	network_mutex_acquire_driver(ns);
	(ns->ns_driver->nd_link_status)(ns->ns_driver->nd_cookie, cb, cbarg);
	network_mutex_release_driver(ns);
}

static void
network_invoke_activity_callbacks(struct network_state *ns)
{

	/*
	 * There's been some network activity.
	 * Inform interested parties.
	 */
	for (unsigned int i = 0; i < NETWORK_ACTIVITY_SLOTS; i++) {
		if (ns->ns_activity_hooks[i].nsa_cb != NULL) {
			(ns->ns_activity_hooks[i].nsa_cb)
			    (ns->ns_activity_hooks[i].nsa_arg);
		}
	}
}

static void
network_ringbuff_cb(ringbuff_t rb, void *arg)
{
	struct network_socket *s = arg;

	(void) rb;

	if (s->s_connected)
		network_socket_attention(s);
	else {
		DBFPRINTF("socket not connected: port %" PRIu16 "\n",
		    s->s_port);
	}
}

#define	mymin(a,b)	((a) < (b) ? (a) : (b))

static __attribute__ ((noinline)) void
network_nw2app(struct network_socket *s)
{
	struct network_state *ns = s->s_driver;
	ringbuff_len_t rbl;
	uint16_t nl, rv;
	bool produced;
	uint8_t *p;

	produced = false;

	while (s->s_connected &&
	    (rbl = ringbuff_get_space(s->s_rb_nw2app)) &&
	    (nl = network_drv_rx_available(ns, s->s_dsock))) {
		/*
		 * There's space in the ring buffer and data might be
		 * available. Check for network errors though.
		 */
		if (NETWORK_SOCK_ERR(nl))
			break;

		/*
		 * What's the largest chunk we can transfer?
		 */
		nl = mymin((uint16_t)rbl, nl);

		/*
		 * Now see how many contiguous bytes we can transfer
		 * to the ring buffer
		 */
		rv = ringbuff_produce_contig_size(s->s_rb_nw2app);

		/*
		 * Fold that into the max #bytes we can transfer this
		 * time around.
		 */
		nl = mymin(nl, rv);

#ifndef NETWORK_OPT_NO_TELNET
		if (s->s_telnet == NULL) {
			p = ringbuff_produce_current_buff_pointer(
			    s->s_rb_nw2app);
		} else {
			p = s->s_telnet->nts_telnet_buff;
			nl = mymin(nl, sizeof(s->s_telnet->nts_telnet_buff));
		}
#else
		p = ringbuff_produce_current_buff_pointer(s->s_rb_nw2app);
#endif /* NETWORK_OPT_NO_TELNET */

		/*
		 * Read data from the network socket directly into the buffer
		 */
		rv = network_drv_read(ns, s->s_dsock, p, nl);
		if (NETWORK_SOCK_ERR(rv))
			break;

		rbl = (ringbuff_len_t)rv;

#ifndef NETWORK_OPT_NO_TELNET
		if (s->s_telnet == NULL) {
#endif
			/*
			 * Write it to the application
			 */
			produced = true;
			ringbuff_produce_contig(s->s_rb_nw2app, rbl);
#ifndef NETWORK_OPT_NO_TELNET
		} else {
			/*
			 * Since telnet is enabled, we need to check
			 * every byte for escape sequences.
			 */
			while (s->s_connected && rbl--) {
				uint16_t td;

				td = telnet_input(&s->s_telnet->nts_telnet,
				    *p++);
				if ((td & TELNET_VALID_DATA) != 0) {
					produced = true;
					ringbuff_produce(s->s_rb_nw2app,
					    td & TELNET_DATA_MASK);
				}
				else
				if ((td & TELNET_SEND_BREAK) != 0 &&
				    s->s_cb != NULL) {
					(s->s_cb)(NETWORK_STATUS_TELNET_BREAK,
					    s->s_cbarg);
				}
			}
		}
#endif /* NETWORK_OPT_NO_TELNET */
	}

	if (s->s_connected && produced)
		ringbuff_produce_done(s->s_rb_nw2app);
}

static __attribute__ ((noinline)) bool
network_app2nw(struct network_socket *s)
{
	struct network_state *ns = s->s_driver;
	ringbuff_len_t rbl;
	uint16_t nl, rv;
	bool consumed, retry, push;

	consumed = retry = push = false;

	while (s->s_connected &&
	    (rbl = ringbuff_get_count(s->s_rb_app2nw)) &&
	    (nl = network_drv_tx_free(ns, s->s_dsock))) {
		/*
		 * There's data in the ring buffer and space on the
		 * network Tx queue. Check for network errors though.
		 */
		if (NETWORK_SOCK_ERR(nl)) {
			DBFPRINTF("txfree err %" PRIu16 "\n", nl);
			break;
		}

		/*
		 * What's the largest chunk we can transfer?
		 */
		nl = mymin((uint16_t)rbl, nl);

		/*
		 * Now see how many contiguous bytes we can transfer
		 * from the ring buffer
		 */
		rv = ringbuff_consume_contig_size(s->s_rb_app2nw);

		/*
		 * Fold that into the max #bytes we can transfer this
		 * time around.
		 */
		nl = mymin(nl, rv);
		assert(nl > 0);

		push = (rbl == nl);

		/*
		 * Write data to the network socket directly from
		 * the ring buffer.
		 */
		rv = network_drv_write(ns, s->s_dsock,
		    ringbuff_consume_current_buff_pointer(s->s_rb_app2nw), nl,
		    NULL, push);
		if (NETWORK_SOCK_ERR(rv)) {
			debug_print("network_app2nw: write err %" PRIu16 ". nl "
			    "%" PRIu16 "\n",rv, nl);
			break;
		}

		if (rv == 0) {
			retry = true;
			break;
		}

		/*
		 * Mark the data as consumed.
		 */
		ringbuff_consume_contig(s->s_rb_app2nw, rv);
		consumed = true;
	}

	if (s->s_connected && consumed)
		ringbuff_consume_done(s->s_rb_app2nw);

	return retry;
}

static void
network_expedite_cb(ringbuff_t rb, void *arg)
{

	(void) rb;
	(void) arg;

	rtos_yield();
}

static bool
network_socket_worker(struct network_socket *s)
{
	bool retry = false;

	if (s->s_dsock != NETWORK_SOCKET_INVALID) {
#ifndef NETWORK_OPT_NO_TELNET
		if (s->s_telnet && s->s_telnet->nts_established == false) {
			s->s_telnet->nts_established = true;
			telnet_init(&s->s_telnet->nts_telnet, s->s_rb_app2nw);
			ringbuff_produce_done(s->s_rb_app2nw);
			if (s->s_cb != NULL) {
				(s->s_cb)(NETWORK_STATUS_TELNET_READY,
				    s->s_cbarg);
			}
		}
#endif /* NETWORK_OPT_NO_TELNET */

		/* Push data from application to network */
		if (s->s_rb_app2nw != NULL)
			retry = network_app2nw(s);

		/* Pull data from network to application */
		if (s->s_rb_nw2app != NULL)
			network_nw2app(s);
	}

	return retry;
}

static void
network_socket_thread(void *arg)
{
	struct network_state *ns = arg;
	struct network_socket *s;
	rtos_saved_ipl_t ipl;

	rtos_task_sleep(500);

	for (;;) {
		do {
			ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
			s = TAILQ_FIRST(&ns->ns_sockq);
			if (s != NULL) {
				TAILQ_REMOVE(&ns->ns_sockq, s, s_qent);
				s->s_pending = 0;
			}
			rtos_ipl_restore(ipl);

			if (s == NULL)
				rtos_notify_take(1);
		} while (s == NULL);

		network_mutex_acquire(s->s_mutex);

		if (network_socket_worker(s)) {
			ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
			if (s->s_pending == 0) {
				TAILQ_INSERT_TAIL(&s->s_driver->ns_sock_retryq,
				    s, s_qent);
				s->s_pending = NETWORK_SOCK_RETRY_PENDING;
			}
			rtos_ipl_restore(ipl);

			rtos_timer_start(ns->ns_retry_timer);
		}

		network_mutex_release(s->s_mutex);

		network_invoke_activity_callbacks(ns);
	}
}

static void
network_schedule_link_check(rtos_timer_t handle)
{
	struct network_state *ns = rtos_timer_get_id(handle);

	rtos_notify(ns->ns_driver_task, NETWORK_EVENT_LINK,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

const char *
network_status_string(network_sock_status_t st)
{
	const char *rv;

	switch (st) {
	case NETWORK_STATUS_IDLE:
		rv = "Idle";
		break;
	case NETWORK_STATUS_CONNECT:
		rv = "Connect";
		break;
	case NETWORK_STATUS_DISCONNECT:
		rv = "Disconnect";
		break;
	case NETWORK_STATUS_LINK_UP:
		rv = "Link-Up";
		break;
	case NETWORK_STATUS_LINK_DOWN:
		rv = "Link-Down";
		break;
	case NETWORK_STATUS_TIMEOUT:
		rv = "Timeout";
		break;
	case NETWORK_STATUS_RX_READY:
		rv = "Rx-Ready";
		break;
	case NETWORK_STATUS_TX_DONE:
		rv = "Tx-Done";
		break;
	case NETWORK_STATUS_UNREACHABLE:
		rv = "Unreachable";
		break;
	default:
		rv = "Bogosity";
		break;
	}

	return rv;
}

int8_t
network_establish(void *arg, ringbuff_t to_net, ringbuff_t from_net,
    bool do_telnet, network_sock_params_t *pp)
{
	struct network_socket *s = arg;

	network_mutex_acquire(s->s_mutex);

#ifndef NETWORK_OPT_NO_TELNET
	if (s->s_type == NETWORK_SOCK_TYPE_TCP && to_net != NULL && do_telnet) {
		s->s_telnet = zone_calloc(1, sizeof(*s->s_telnet));
		if (s->s_telnet == NULL) {
			network_mutex_release(s->s_mutex);
			return -1;
		}
		s->s_telnet->nts_established = false;
	} else {
		s->s_telnet = NULL;
	}
#else
	if (do_telnet)
		DBFPRINTF("telnet not supported\n");
#endif /* NETWORK_OPT_NO_TELNET */

	/*
	 * Allocate the network -> app ringbuff. We are its producer.
	 */
	if ((s->s_rb_nw2app = from_net) != NULL)
		ringbuff_producer_init(s->s_rb_nw2app, network_ringbuff_cb, s);

	/*
	 * Allocate the app -> network ringbuff. We are its consumer.
	 */
	if ((s->s_rb_app2nw = to_net) != NULL) {
		ringbuff_consumer_init(s->s_rb_app2nw, network_ringbuff_cb, s);
		ringbuff_consumer_expedite_init(s->s_rb_app2nw,
		    network_expedite_cb);
	}

	if (to_net || from_net) {
		s->s_connected = true;
		network_socket_attention(s);

		if (pp)
			network_drv_sock_status(&network_state, s->s_dsock, pp);
	}

	network_mutex_release(s->s_mutex);

	return 0;
}

void
network_tcp_nagle(void *arg, bool enable)
{
	struct network_socket *s = arg;

	network_mutex_acquire(s->s_mutex);
	network_mutex_acquire_driver(s->s_driver);

	network_drv_ioctl(s->s_driver, enable ?
	    NETWORK_IOCTL_TCP_NAGLE_ON : NETWORK_IOCTL_TCP_NAGLE_OFF,
	    s->s_dsock);

	network_mutex_release_driver(s->s_driver);
	network_mutex_release(s->s_mutex);
}

static void
network_tcp_status_cb(network_sock_status_t st, void *arg)
{
	struct network_socket *s = arg;

	switch (st) {
	case NETWORK_STATUS_CONNECT:
	case NETWORK_STATUS_DISCONNECT:
	case NETWORK_STATUS_LINK_UP:
	case NETWORK_STATUS_LINK_DOWN:
	case NETWORK_STATUS_TIMEOUT:
	case NETWORK_STATUS_UNREACHABLE:
		network_invoke_activity_callbacks(s->s_driver);
		if (s->s_cb != NULL)
			(s->s_cb)(st, s->s_cbarg);
		break;

	case NETWORK_STATUS_RX_READY:
		if (s->s_rb_nw2app != NULL) {
			if (s->s_connected)
				network_socket_attention(s);
			else {
				DBFPRINTF("RX: Port %" PRIu16 " not "
				    "connected\n", s->s_port);
			}
		} else
		if (s->s_cb != NULL)
			(s->s_cb)(st, s->s_cbarg);
		break;

	case NETWORK_STATUS_TX_DONE:
		if (s->s_rb_app2nw != NULL) {
			if (s->s_connected)
				network_socket_attention(s);
			else {
				DBFPRINTF("TX: Port %" PRIu16 " not "
				    "connected\n", s->s_port);
			}
		} else
		if (s->s_cb != NULL)
			(s->s_cb)(st, s->s_cbarg);
		break;

	default:
		break;
	}
}

static struct network_socket *
network_alloc_socket(struct network_state *ns)
{
	struct network_socket *s;
	unsigned int i;

	/*
	 * This must be invoked with ns_driver_mutex held
	 */
	for (i = 0; i < NETWORK_NSOCKETS; i++) {
		s = &ns->ns_sockets[i]; 
		if (s->s_dsock == NETWORK_SOCKET_INVALID) {
			s->s_connected = false;
#ifndef NETWORK_OPT_NO_TELNET
			s->s_telnet = NULL;
#endif
			return s;
		}
	}

	return NULL;
}

void *
network_socket_listen(uint16_t port, network_sock_status_callback_t cb,
    void *cbarg)
{
	struct network_state *ns = &network_state;
	struct network_socket *s;
	network_sock_t sn;

	network_mutex_acquire_driver(ns);

	if (!network_running(ns) || !ns->ns_link_up ||
	    (s = network_alloc_socket(ns)) == NULL) {
		network_mutex_release_driver(ns);
		return NULL;
	}

	sn = network_drv_open(ns, NETWORK_SOCK_TYPE_TCP,
	    network_tcp_status_cb, s);
	if (sn != NETWORK_SOCKET_INVALID) {
		s->s_dsock = sn;
		s->s_type = NETWORK_SOCK_TYPE_TCP;
		s->s_cb = cb;
		s->s_cbarg = cbarg;
		s->s_dip = ns->ns_ip_params.ip_addr;
		s->s_port = port;
		s->s_rb_nw2app = NULL;
		s->s_rb_app2nw = NULL;

		if (network_drv_listen(ns, sn, port) == 0) {
			network_drv_close(ns, sn);
			s->s_dsock = NETWORK_SOCKET_INVALID;
			s = NULL;
		}
	} else {
		s = NULL;
	}

	network_mutex_release_driver(ns);

	return s;
}

void *
network_socket_udp(const network_sock_params_t *sp,
    network_sock_status_callback_t cb, void *cbarg)
{
	struct network_state *ns = &network_state;
	struct network_socket *s;
	network_sock_t sn;

	network_mutex_acquire_driver(ns);

	/*
	 * XXX: This is a bit of a hack to permit DHCP to work when
	 * we don't yet have an IP address.
	 */
	if ((sp->sp_dport != 67 && !network_running(ns)) ||
	    !ns->ns_link_up || (s = network_alloc_socket(ns)) == NULL) {
		network_mutex_release_driver(ns);
		return NULL;
	}

	sn = network_drv_open(ns, NETWORK_SOCK_TYPE_UDP,
	    network_tcp_status_cb, s);
	if (sn != NETWORK_SOCKET_INVALID) {
		s->s_dsock = sn;
		s->s_type = NETWORK_SOCK_TYPE_UDP;
		s->s_cb = cb;
		s->s_cbarg = cbarg;
		s->s_dip = sp->sp_dest_ip;
		s->s_port = sp->sp_sport;
		s->s_rb_nw2app = NULL;
		s->s_rb_app2nw = NULL;

		if (network_drv_connect(ns, sn, sp) == 0) {
			network_drv_close(ns, sn);
			s->s_dsock = NETWORK_SOCKET_INVALID;
			s = NULL;
		}
	} else {
		s = NULL;
	}

	network_mutex_release_driver(ns);

	return s;
}

uint16_t
network_sock_rx_avail(void *arg)
{
	struct network_socket *s = arg;

	if (s->s_rb_nw2app != NULL)
		return 0;

	return network_drv_rx_available(&network_state, s->s_dsock);
}

uint16_t
network_sock_recv(void *arg, void *dest, uint16_t len,
    network_sock_params_t *sp)
{
	struct network_socket *s = arg;

	if (s->s_rb_nw2app != NULL)
		return NETWORK_SOCK_ERR_BUSY;

	return network_drv_recv(&network_state, s->s_dsock, dest, len, sp);
}

uint16_t
network_sock_sendto(void *arg, const void *src, uint16_t len,
    const uint8_t *dstmac)
{
	struct network_socket *s = arg;
	uint16_t rv;

	if (s->s_rb_app2nw != NULL) {
		DBFPRINTF("busy\n");
		return NETWORK_SOCK_ERR_BUSY;
	}

	rv = network_drv_write(&network_state, s->s_dsock, src, len, dstmac,
	    true);

	return rv;
}

network_sock_state_t
network_sock_status(void *arg, network_sock_params_t *sp)
{
	struct network_socket *s = arg;

	return network_drv_sock_status(&network_state, s->s_dsock, sp);
}

void
network_close(void *arg)
{
	struct network_socket *s = arg;
	struct network_state *ns = &network_state;
	rtos_saved_ipl_t ipl;

	network_mutex_acquire(s->s_mutex);
	network_mutex_acquire_driver(ns);

	s->s_connected = false;
	network_drv_close(ns, s->s_dsock);

	if (s->s_rb_app2nw != NULL) {
		ringbuff_consumer_init(s->s_rb_app2nw, NULL, NULL);
		ringbuff_consumer_expedite_init(s->s_rb_app2nw, NULL);
		s->s_rb_app2nw = NULL;
	}

	if (s->s_rb_nw2app != NULL) {
		ringbuff_producer_init(s->s_rb_nw2app, NULL, NULL);
		s->s_rb_nw2app = NULL;
	}

	s->s_dsock = NETWORK_SOCKET_INVALID;
	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	if (s->s_pending == NETWORK_SOCK_WORK_PENDING) {
		TAILQ_REMOVE(&ns->ns_sockq, s, s_qent);
	} else
	if (s->s_pending == NETWORK_SOCK_RETRY_PENDING) {
		TAILQ_REMOVE(&ns->ns_sock_retryq, s, s_qent);
	}
	s->s_pending = 0;
	rtos_ipl_restore(ipl);

#ifndef NETWORK_OPT_NO_TELNET
	if (s->s_telnet != NULL) {
		zone_free(s->s_telnet);
		s->s_telnet = NULL;
	}
#endif

	network_mutex_release_driver(ns);
	network_mutex_release(s->s_mutex);
}

void
network_set_ip_params_drv(const network_ip_params_t *ip)
{

	network_state.ns_ip_params = *ip;

	if (network_state.ns_state == NETWORK_STATE_DOWN && ip->ip_addr != 0) {
		network_state.ns_state = NETWORK_STATE_RUNNING;
	} else
	if (ip->ip_addr == 0) {
	}

	rtos_notify(network_state.ns_driver_task, NETWORK_EVENT_LINK_STATUS,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

void
network_set_ip_params_drv_persist(const network_ip_params_t *ip)
{

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_network_ip = ip->ip_addr;
	glob.glob_network_mask = ip->ip_mask;
	glob.glob_network_gate = ip->ip_gate;
	glob_save();
#else	/* CONFIG_USE_CONFIGDB */
	cf_set_uint32(CF_KEY_UINT32_NETWORK_IP, ip->ip_addr);
	cf_set_uint32(CF_KEY_UINT32_NETWORK_MASK, ip->ip_mask);
	cf_set_uint32(CF_KEY_UINT32_NETWORK_GATE, ip->ip_gate);
	configdb_persist();
#endif /* CONFIG_USE_CONFIGDB */

	rtos_notify(network_state.ns_driver_task, NETWORK_EVENT_IP_CONFIGURE,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

void
network_set_ip_params(const network_ip_params_t *ip)
{
	struct network_state *ns = &network_state;

	network_drv_set_ip_params(ns, ip);
	network_set_ip_params_drv(ip);
}

const network_ip_params_t *
network_get_ip_params(void)
{

	return &network_state.ns_ip_params;
}

void
network_get_mac(uint8_t *mac)
{

	network_drv_get_mac(&network_state, mac);
}

void
network_driver_hw_interrupt(struct network_state *ns)
{

	/*
	 * Notify the worker thread
	 */
	rtos_notify_isr(ns->ns_driver_task, NETWORK_EVENT_INTERRUPT,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

static void
network_schedule_soft_event(struct network_state *ns)
{

	/* Notify the worker thread */
	rtos_notify(ns->ns_driver_task, NETWORK_EVENT_INTERRUPT,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

static void
network_soft_timer_event(rtos_timer_t handle)
{
	struct network_state *ns = rtos_timer_get_id(handle);

	network_schedule_soft_event(ns);
}

void
network_driver_soft_timeout(struct network_state *ns, uint32_t msecs)
{

	/*
	 * The back-end driver's worker handler sometimes needs to be
	 * invoked after a period of time with no h/w interrupt.
	 * Create a timer for this purpose.
	 */
	if (msecs != 0) {
		rtos_timer_stop(ns->ns_soft_timer);
		rtos_timer_change_wait_period(ns->ns_soft_timer, msecs);
		rtos_timer_start(ns->ns_soft_timer);
	} else {
		network_schedule_soft_event(ns);
	}
}

static void
network_link_status_handler(const network_link_status_t *st, void *arg)
{
	struct network_state *ns = arg;

	/*
	 * ns_driver_mutex is already held since this is invoked by the
	 * driver's worker thread.
	 */

	if (memcmp(&ns->ns_link_status, st, sizeof(*st)) != 0) {
		rtos_saved_ipl_t ipl;

		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		ns->ns_link_status = *st;
		rtos_ipl_restore(ipl);

		rtos_notify(ns->ns_driver_task, NETWORK_EVENT_LINK_STATUS,
		    RTOS_NOTIFY_ACTION_SET_BITS);
	}
}

void
network_get_link_status(network_link_status_t *ls)
{
	struct network_state *ns = &network_state;

	network_mutex_acquire_driver(ns);
	*ls = ns->ns_link_status;
	network_mutex_release_driver(ns);
}

void
network_unhook_link_status(int8_t v)
{
	struct network_state *ns = &network_state;

	if (v > 0 && v < NETWORK_LINK_STATUS_SLOTS) {
		ns->ns_link_status_hooks[v].nsl_cb = NULL;
		ns->ns_link_status_hooks[v].nsl_arg = NULL;
	}
}

int8_t
network_hook_link_status(network_link_status_callback_t cb, void *arg)
{
	struct network_state *ns = &network_state;

	for (unsigned int i = 0; i < NETWORK_LINK_STATUS_SLOTS; i++) {
		if (ns->ns_link_status_hooks[i].nsl_cb == NULL) {
			ns->ns_link_status_hooks[i].nsl_cb = cb;
			ns->ns_link_status_hooks[i].nsl_arg = arg;
			return (int8_t)i;
		}
	}

	return -1;
}

void
network_unhook_activity(int8_t v)
{
	struct network_state *ns = &network_state;

	if (v > 0 && v < NETWORK_ACTIVITY_SLOTS) {
		ns->ns_activity_hooks[v].nsa_cb = NULL;
		ns->ns_activity_hooks[v].nsa_arg = NULL;
	}
}

int8_t
network_hook_activity(network_activity_callback_t cb, void *arg)
{
	struct network_state *ns = &network_state;

	for (unsigned int i = 0; i < NETWORK_ACTIVITY_SLOTS; i++) {
		if (ns->ns_activity_hooks[i].nsa_cb == NULL) {
			ns->ns_activity_hooks[i].nsa_cb = cb;
			ns->ns_activity_hooks[i].nsa_arg = arg;
			return (int8_t)i;
		}
	}

	return -1;
}

static void
network_suspend(struct network_state *ns, struct network_suspend_args *sa)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	ns->ns_state = NETWORK_STATE_SUSPENDED;
	ns->ns_link_status.ls_state = NETWORK_LINK_DOWN;
	ns->ns_suspend_args = *sa;

	rtos_ipl_restore(ipl);

	rtos_notify(ns->ns_driver_task, NETWORK_EVENT_LINK_STATUS,
	    RTOS_NOTIFY_ACTION_SET_BITS);
}

static void
network_resume(struct network_state*ns)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	ns->ns_state = NETWORK_STATE_DOWN;
	ns->ns_suspend_args.sa_cb = NULL;

	rtos_ipl_restore(ipl);
}

int8_t
network_ioctl(struct network_state *ns, uint8_t cmd, void *arg)
{
	int8_t rv = 0;

	switch (cmd) {
	case NETWORK_IOCTL_SUSPEND:
		/*
		 * Return an error if already suspended.
		 */
		if (ns->ns_state == NETWORK_STATE_SUSPENDED || arg == NULL)
			rv = -1;
		else
			network_suspend(ns, arg);
		break;

	case NETWORK_IOCTL_RESUME:
		if (ns->ns_state != NETWORK_STATE_SUSPENDED)
			rv = -1;
		else
			network_resume(ns);
		break;

	default:
		network_mutex_acquire_driver(ns);
		rv = network_drv_ioctl(ns, cmd, arg);
		network_mutex_release_driver(ns);
		break;
	}


	return rv;
}

static uint8_t
network_get_event(struct network_state *ns)
{
	uint32_t bits;

	(void) ns;

	while (rtos_notify_wait(0, 0xfful, &bits, RTOS_MAX_SLEEP_MS) == pdFALSE)
		;

	return (uint8_t)bits;
}

static void
network_get_ip_config(struct network_state *ns, network_ip_params_t *ip)
{

	memset(ip, 0, sizeof(*ip));

	if (ns->ns_link_up == false)
		return;

#ifndef CONFIG_USE_CONFIGDB
	ip->ip_addr = glob.glob_network_ip;
	ip->ip_mask = glob.glob_network_mask;
	ip->ip_gate = glob.glob_network_gate;

#else	/* CONFIG_USE_CONFIGDB */
	cf_get_uint32(CF_KEY_UINT32_NETWORK_IP, &ip->ip_addr);
	cf_get_uint32(CF_KEY_UINT32_NETWORK_MASK, &ip->ip_mask);
	cf_get_uint32(CF_KEY_UINT32_NETWORK_GATE, &ip->ip_gate);
#endif /* CONFIG_USE_CONFIGDB */
}

static void
network_driver_process_events(struct network_state *ns, uint8_t events)
{
	unsigned int i;

	network_mutex_acquire_driver(ns);

	if ((events & NETWORK_EVENT_INTERRUPT) != 0) {
		(ns->ns_driver->nd_worker)(ns->ns_driver->nd_cookie);
	}

	if ((events & NETWORK_EVENT_LINK) != 0 &&
	    ns->ns_state != NETWORK_STATE_SUSPENDED &&
	    ns->ns_driver->nd_link_check != NULL) {
		(ns->ns_driver->nd_link_check)(ns->ns_driver->nd_cookie);
	}

	network_mutex_release_driver(ns);

	if (events & NETWORK_EVENT_IP_CONFIGURE)
		network_ip_configure(ns);

	/*
	 * If networking is suspended and we have a valid suspend callback,
	 * check if all sockets are closed.
	 */
	if (ns->ns_state == NETWORK_STATE_SUSPENDED &&
	    ns->ns_suspend_args.sa_cb != NULL) {
		struct network_socket *s = &ns->ns_sockets[0];

		DBFPRINTF("suspended\n");

		for (i = 0; i < NETWORK_NSOCKETS; i++, s++) {
			if (s->s_dsock != NETWORK_SOCKET_INVALID)
				break;
		}

		if (i == NETWORK_NSOCKETS) {
			void (*cb)(void *);

			/*
			 * All sockets are closed. Invoke the suspend
			 * callback.
			 */
			cb = ns->ns_suspend_args.sa_cb;
			ns->ns_suspend_args.sa_cb = NULL;
			(cb)(ns->ns_suspend_args.sa_cookie);
		}
	}

	/*
	 * Has the link status changed?
	 */
	if ((events & NETWORK_EVENT_LINK_STATUS) == 0)
		return;

#ifdef DEBUG_ENABLED
	{
		static network_link_status_t ls;
		static network_ip_params_t ip;
		bool ls_copy, ip_copy;

		ls_copy = false;
		ip_copy = false;

		if (ns->ns_link_status.ls_state != ls.ls_state) {
			DBFPRINTF("link status change: %u\n",
			    (unsigned int)ns->ns_link_status.ls_state);
			ls_copy = true;
		}
		if (strcmp(ns->ns_link_status.ls_bssid, ls.ls_bssid) != 0 ||
		    strcmp(ns->ns_link_status.ls_key, ls.ls_key) != 0) {
			DBFPRINTF("WiFi change: '%s'\n",
			    ns->ns_link_status.ls_bssid);
			ls_copy = true;
		}
		if (memcmp(&ns->ns_ip_params, &ip, sizeof(ip)) != 0) {
			char ipstr[NETWORK_IP_STR_LEN];
			network_ip2str(ns->ns_ip_params.ip_addr, ipstr);
			DBFPRINTF("IP change: A %s, ", ipstr);
			network_ip2str(ns->ns_ip_params.ip_mask, ipstr);
			DBPRINTF("M %s, ", ipstr);
			network_ip2str(ns->ns_ip_params.ip_gate, ipstr);
			DBPRINTF("G %s\n", ipstr);
			ip_copy = true;
		}

		if (ls_copy)
			ls = ns->ns_link_status;
		if (ip_copy)
			ip = ns->ns_ip_params;
	}
#endif	/* DEBUG_ENABLED */

	/*
	 * Link status has changed. Inform interested parties now
	 * that we're no longer holding ns_driver_mutex.
	 */
	for (i = 0; i < NETWORK_LINK_STATUS_SLOTS; i++) {
		if (ns->ns_link_status_hooks[i].nsl_cb != NULL) {
			(ns->ns_link_status_hooks[i].nsl_cb)
			    (&ns->ns_link_status,
			    ns->ns_link_status_hooks[i].nsl_arg);
		}
	}

	/*
	 * Check if the connection state has changed. If not,
	 * there's nothing more to do.
	 */
	bool conn_state = (ns->ns_link_status.ls_state == NETWORK_LINK_UP);
	if (ns->ns_link_up == conn_state)
		return;

	DBFPRINTF("Connection status change\n");

	/*
	 * Connection state has changed. We have work to do.
	 */
	ns->ns_link_up = conn_state;

	/*
	 * If the link is down, notify all remaining open sockets.
	 */
	if (ns->ns_link_up == false) {
		struct network_socket *s = &ns->ns_sockets[0];

		for (i = 0; i < NETWORK_NSOCKETS; i++, s++) {
			bool changed;

			network_mutex_acquire(s->s_mutex);
			changed = (s->s_dsock != NETWORK_SOCKET_INVALID);
			network_mutex_release(s->s_mutex);

			if (changed && s->s_cb != NULL)
				(s->s_cb)(NETWORK_STATUS_LINK_DOWN, s->s_cbarg);
		}
	}

	network_ip_params_t ip;
	network_get_ip_config(ns, &ip);

	if (ip.ip_addr != 0)
		network_set_ip_params(&ip);
}

static void
network_driver_thread(void *arg)
{
	struct network_state *ns = arg;
	uint8_t events;

	if (ns->ns_driver->nd_link_check != NULL)
		rtos_timer_start(ns->ns_link_timer);

	for (;;) {
		events = network_get_event(ns);
		network_driver_process_events(ns, events);
	}
}

char *
network_ip2str(uint32_t ip, char *s)
{

	if (ip == 0) {
#if defined(NETWORK_OPT_DHCP_CLIENT) || \
    defined(NETWORK_OPT_EXTERNAL_DHCP_CLIENT)
		strcpy(s, "Acquiring IP");
#else
		strcpy(s, "IP Not Set");
#endif
	} else {
		sprintf(s, "%u.%u.%u.%u",
		    (unsigned int)(ip >> 24) & 0xffu,
		    (unsigned int)(ip >> 16) & 0xffu,
		    (unsigned int)(ip >> 8) & 0xffu,
		    (unsigned int)(ip >> 0) & 0xffu);
	}

	return s;
}

const char *
network_str2ip(const char *str, uint32_t *ip)
{
	uint32_t acc, v;
	char *p;

	acc = strtoul(str, &p, 10);
	if (acc > 255 || p == str || *p != '.')
		return NULL;

	str = p + 1;
	v = strtoul(str, &p, 10);
	if (v > 255 || p == str || *p != '.')
		return NULL;
	acc = (acc << 8) | v;

	str = p + 1;
	v = strtoul(str, &p, 10);
	if (v > 255 || p == str || *p != '.')
		return NULL;
	acc = (acc << 8) | v;

	str = p + 1;
	v = strtoul(str, &p, 10);
	if (v > 255 || p == str || (*p != '\0' && *p != '/'))
		return NULL;
	acc = (acc << 8) | v;
	*ip = acc;
	return p;
}

const char *
network_parse_ip_params(const char *ip_addr, const char *ip_mask,
    const char *ip_gate, network_ip_params_t *ipp)
{
	uint8_t net;
	int i;

	assert(ip_addr != NULL || ipp->ip_addr != 0);
	assert(ip_mask != NULL || ipp->ip_mask != 0);

	if (ip_addr != NULL && network_str2ip(ip_addr, &ipp->ip_addr) == NULL)
		return "Malformed IP address";

	if (ip_mask != NULL && network_str2ip(ip_mask, &ipp->ip_mask) == NULL)
		return "Malformed IP mask";

	/*
	 * MSB of IP address must be non-zero, between 1 and 223,
	 * excluding loopback at 127.
	 */
	net = (uint8_t)(ipp->ip_addr >> 24);
	if (net < 1 || net == 127 || net > 223)
		return "Invalid unicast IP address";

	/*
	 * Ensure a sensible mask. This rejects subnets smaller than /30
	 * and larger than /8.
	 */
	if (ipp->ip_mask < 4 || (ipp->ip_mask >> 24) != 0xffu)
		return "Invalid mask size";

	/* Ensure the mask is contiguous. */
	i = ffs((int)ipp->ip_mask) - 1;
	if ((ipp->ip_mask ^ ~((1u << i) - 1)) != 0)
		return "Mask is not contiguous";

	/* Reject attempts to set ip_addr to the subnet's broadcast address. */
	if ((ipp->ip_addr & ~ipp->ip_mask) == (0xffffffffu & ~ipp->ip_mask))
		return "IP is broadcast address";

	if (ip_gate != NULL) {
		ipp->ip_gate = 0;
		if (network_str2ip(ip_gate, &ipp->ip_gate) == NULL)
			return "Malformed gateway IP";
	}

	if (ipp->ip_gate != 0) {
		/* We cannot be our own gateway. */
		if (ipp->ip_addr == ipp->ip_gate)
			return "We cannot be our own gateway";

		/* Ensure gateway is on same subnet as our IP. */
		if ((ipp->ip_addr & ipp->ip_mask) !=
		    (ipp->ip_gate & ipp->ip_mask)) {
			return "Gateway not on subnet";
		}

		/* Gateway cannot be a broadcast address. */
		if ((ipp->ip_addr & ~ipp->ip_mask) ==
		    (0xffffffffu & ~ipp->ip_mask)) {
			return "Gateway is broadcast IP";
		}
	}

	return NULL;
}

static void
network_ip_configure(struct network_state *ns)
{
	network_ip_params_t ip;

	network_get_ip_config(ns, &ip);

	if (ip.ip_addr == 0) {
#ifdef NETWORK_OPT_DHCP_CLIENT
		native_dhcp_init();
#elif defined(NETWORK_OPT_EXTERNAL_DHCP_CLIENT)
		network_mutex_acquire_driver(ns);
		network_drv_ioctl(ns, NETWORK_IOCTL_DHCP_CLIENT_START, NULL);
		network_mutex_release_driver(ns);
#endif
	} else {
#ifdef NETWORK_OPT_DHCP_CLIENT
		native_dhcp_deinit();
#elif defined(NETWORK_OPT_EXTERNAL_DHCP_CLIENT)
		network_mutex_acquire_driver(ns);
		network_drv_ioctl(ns, NETWORK_IOCTL_DHCP_CLIENT_STOP, NULL);
		network_mutex_release_driver(ns);
#endif
		if (ns->ns_link_up)
			network_set_ip_params(&ip);
	}
}

#ifdef NETWORK_OPT_COMMAND
#ifndef NETWORK_OPT_DHCP_ONLY
static bool
network_parse_ip(const char *str, uint32_t *pip, uint32_t *pmask)
{
	const char *q;
	char *p;
	uint32_t m;

	if ((q = network_str2ip(str, pip)) == NULL)
		return false;

	if (pmask == NULL)
		return *q == '\0';

	if (*q != '/' || !isdigit((unsigned char)q[1]))
		return false;

	str = q + 1;
	m = strtoul(str, &p, 10);
	if (p == str || *p != '\0')
		return false;

	if (m < 8 || m > 31)
		return false;

	*pmask = 0xfffffffful << (32 - m);

	return true;
}

static bool
network_cmd_gate(FILE *os, const char * const *argv)
{
	network_ip_params_t ipp;
	uint32_t cur_gate;
	const char *str;

	memset(&ipp, 0, sizeof(ipp));
#ifndef CONFIG_USE_CONFIGDB
	ipp.ip_addr = glob.glob_network_ip;
	ipp.ip_mask = glob.glob_network_mask;
	ipp.ip_gate = glob.glob_network_gate;
#else
	cf_get_uint32(CF_KEY_UINT32_NETWORK_IP, &ipp.ip_addr);
	cf_get_uint32(CF_KEY_UINT32_NETWORK_MASK, &ipp.ip_mask);
	cf_get_uint32(CF_KEY_UINT32_NETWORK_GATE, &ipp.ip_gate);
#endif

	if (ipp.ip_addr == 0) {
		fputs("IP address must be assigned before specifying"
		    " default gateway\n", os);
		return false;
	}

	cur_gate = ipp.ip_gate;
	if (strcasecmp(argv[0], "delete") == 0) {
		ipp.ip_gate = 0;
	} else
	if ((str = network_parse_ip_params(NULL, NULL, argv[0], &ipp)) != NULL){
		fprintf(os, "%s\n", str);
		return false;
	}

	if (cur_gate == ipp.ip_gate) {
		fputs("No change.\n", os);
		return false;
	}

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_network_gate = ipp.ip_gate;
#else
	cf_set_uint32(CF_KEY_UINT32_NETWORK_GATE, ipp.ip_gate);
#endif

	return true;
}

static bool
network_cmd_ip(FILE *os, const char * const *argv)
{
	network_ip_params_t ipp;
	const char *str;

#if defined(NETWORK_OPT_DHCP_CLIENT) || \
    defined(NETWORK_OPT_EXTERNAL_DHCP_CLIENT)
	if (strcasecmp(argv[0], "dhcp") == 0 ||
	    strcasecmp(argv[0], "auto") == 0) {
#ifndef CONFIG_USE_CONFIGDB
		glob.glob_network_ip = 0;
		glob.glob_network_gate = 0;
		glob.glob_network_mask = 0;
#else /* CONFIG_USE_CONFIGDB */
		cf_set_uint32(CF_KEY_UINT32_NETWORK_IP, 0);
		cf_set_uint32(CF_KEY_UINT32_NETWORK_GATE, 0);
		cf_set_uint32(CF_KEY_UINT32_NETWORK_MASK, 0);
#endif /* CONFIG_USE_CONFIGDB */
		return true;
	}
#endif

	memset(&ipp, 0, sizeof(ipp));
	if (network_parse_ip(argv[0], &ipp.ip_addr, &ipp.ip_mask) == false ||
	    ipp.ip_addr == 0 || ipp.ip_mask == 0) {
		fputs("Malformed IP address or CIDR\n", os);
		return false;
	}

	if ((str = network_parse_ip_params(NULL, NULL, NULL, &ipp)) != NULL) {
		fprintf(os, "%s\n", str);
		return false;
	}

#ifndef CONFIG_USE_CONFIGDB
	if (glob.glob_network_ip == ipp.ip_addr &&
	    glob.glob_network_mask == ipp.ip_mask) {
		fputs("No change.\n", os);
		return false;
	}
	ipp.ip_gate = glob.glob_network_gate;
#else
	uint32_t ip;
	if (cf_get_uint32(CF_KEY_UINT32_NETWORK_IP, &ip) && ip == ipp.ip_addr &&
	    cf_get_uint32(CF_KEY_UINT32_NETWORK_MASK, &ip) && ip==ipp.ip_mask) {
		fputs("No change.\n", os);
		return false;
	}
	cf_get_uint32(CF_KEY_UINT32_NETWORK_GATE, &ipp.ip_gate);
#endif

	/* IP address/mask have changed. Verify the gateway is still valid. */

	if (ipp.ip_gate != 0 &&
	    network_parse_ip_params(NULL, NULL, NULL, &ipp) != NULL) {
		/* Gateway not on subnet. Invalidate it. */
		fputs("Removing existing gateway; it is not on this subnet.\n",
		    os);
#ifndef CONFIG_USE_CONFIGDB
		glob.glob_network_gate = 0;
#else
		cf_set_uint32(CF_KEY_UINT32_NETWORK_GATE, 0);
#endif
	}

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_network_ip = ipp.ip_addr;
	glob.glob_network_mask = ipp.ip_mask;
#else
	cf_set_uint32(CF_KEY_UINT32_NETWORK_IP, ipp.ip_addr);
	cf_set_uint32(CF_KEY_UINT32_NETWORK_MASK, ipp.ip_mask);
#endif /* CONFIG_USE_CONFIGDB */

	return true;
}
#endif /* NETWORK_OPT_DHCP_ONLY */

static const char network_cmd_usage[] =
"usage:\n"
"net\n"
"\tShow current network configuration and connections.\n"
#ifndef NETWORK_OPT_DHCP_ONLY
"net ip <ipv4-address/cidr>\n"
"\tConfigure a static IPv4 address and subnet mask. The IP is standard\n"
"\tdotted quad format, followed by a single '/' and a number between 8\n"
"\tand 30 to indicate the CIDR. In most cases, /24 will be appropriate.\n"
"net ip dhcp\n"
"\tConfigure dynamic IPv4 address assignment via DHCP.\n"
"net gate <ipv4-address>\n"
"\tSpecifies the IPv4 address of the subnet's router if DHCP is not used.\n"
"\tA default gateway/router is not required for normal operation, but you\n"
"\tmay need to configure one if access is required from a different\n"
"\tnetwork.\n"
"net gate delete\n"
"\tDeletes the IPv4 address of the subnet's router.\n"
#endif /* NETWORK_OPT_DHCP_ONLY */
;

static void
network_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct network_state *ns = &network_state;
	struct network_socket *s;
	network_sock_params_t sp;
	network_sock_state_t st;
	const char *tstr, *cstr;
	char laddr[NETWORK_IP_STR_LEN+7], raddr[NETWORK_IP_STR_LEN+7];
	uint8_t i, mac[6];
	char *p, ipstr[NETWORK_IP_STR_LEN];
	uint32_t ip;

	(void) argv;

	if (argc >= 1) {
		bool cfg;
#ifndef NETWORK_OPT_DHCP_ONLY
		if (argc == 2 && strcasecmp(argv[0], "ip") == 0) {
			cfg = network_cmd_ip(os, argv + 1);
		} else
		if (argc == 2 && strcasecmp(argv[0], "gate") == 0) {
			cfg = network_cmd_gate(os, argv + 1);
		} else
#endif
		{
			fputs(network_cmd_usage, os);
			cfg = false;
		}

		if (cfg) {
#ifndef CONFIG_USE_CONFIGDB
			if (glob_save() != GLOB_RDWR_OK)
				fputs("Warning: globals didn't save!\n", os);
#else
			configdb_persist();
#endif
			if (cfg)
				network_ip_configure(ns);
		}

		return;
	}

	network_get_mac(mac);
	fprintf(os, "Mac Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	    (unsigned int)mac[0], (unsigned int)mac[1], (unsigned int)mac[2],
	    (unsigned int)mac[3], (unsigned int)mac[4], (unsigned int)mac[5]);
	fprintf(os, "Link %s, ", ns->ns_link_up ? "Up" : "Down");

#ifndef CONFIG_USE_CONFIGDB
	ip = glob.glob_network_ip;
#else
	ip = 0;
	cf_get_uint32(CF_KEY_UINT32_NETWORK_IP, &ip);
#endif

	if (ip == 0)
		fputs("DHCP ", os);
	else
		fputs("Static ", os);

	if (ns->ns_ip_params.ip_addr != 0) {
		network_ip2str(ns->ns_ip_params.ip_addr, ipstr);
		fprintf(os, "Address %s/%u", ipstr,
		    33 - ffs(ns->ns_ip_params.ip_mask));
		if (ns->ns_ip_params.ip_gate != 0) {
			network_ip2str(ns->ns_ip_params.ip_gate, ipstr);
			fprintf(os, ", Gateway %s", ipstr);
		} else
			fputs(", No default gateway", os);
		fputc('\n', os);
	} else
		fputs("address not yet configured.\n", os);

	for (i = 0; i < NETWORK_NSOCKETS; i++) {
		s = &network_state.ns_sockets[i];
		if (s->s_dsock == NETWORK_SOCKET_INVALID)
			continue;

		st = network_sock_status(s, &sp);

		network_ip2str(ns->ns_ip_params.ip_addr, laddr);
		sprintf(&laddr[strlen(laddr)], ".%" PRIu16, sp.sp_sport);

		if (sp.sp_dest_ip != 0) {
			network_ip2str(sp.sp_dest_ip, raddr);
			p = &raddr[strlen(raddr)];
		} else {
			p = raddr;
			*p++ = '*';
			*p = '\0';
		}

		if (sp.sp_dport != 0)
			sprintf(p, ".%" PRIu16, sp.sp_dport);
		else {
			p[0] = '.';
			p[1] = '*';
			p[2] = '\0';
		}

		switch (st) {
		default:
			cstr = "";
			break;
		case NETWORK_SOCKS_LISTEN:
			cstr = "LISTEN";
			break;
		case NETWORK_SOCKS_ESTABLISHED:
			cstr = "ESTABLISHED";
			break;
		case NETWORK_SOCKS_CLOSE_WAIT:
			cstr = "CLOSE_WAIT";
			break;
		case NETWORK_SOCKS_SYNSENT:
			cstr = "SYNSENT";
			break;
		case NETWORK_SOCKS_SYNRECV:
			cstr = "SYNRECV";
			break;
		case NETWORK_SOCKS_FIN_WAIT:
			cstr = "FIN_WAIT";
			break;
		case NETWORK_SOCKS_CLOSING:
			cstr = "CLOSING";
			break;
		case NETWORK_SOCKS_TIME_WAIT:
			cstr = "TIME_WAIT";
			break;
		case NETWORK_SOCKS_LAST_ACK:
			cstr = "TIME_WAIT";
			break;
		}

		switch (s->s_type) {
		case NETWORK_SOCK_TYPE_TCP:
			tstr = "tcp";
			break;
		case NETWORK_SOCK_TYPE_UDP:
			tstr = "udp";
			break;
		default:
			tstr = "unknown";
			break;
		}

		fprintf(os, "%-6s %-22s %-22s %s\n", tstr, laddr, raddr, cstr);
#ifndef NDEBUG
		if (s->s_connected) {
			fprintf(os, "\ts_pending %" PRIu16, s->s_pending);
			fprintf(os, ", txfree %" PRIu16,
			    network_drv_tx_free(ns, s->s_dsock));
			if (s->s_rb_nw2app != NULL) {
				fprintf(os, ", nw2app %lu",
				    ringbuff_get_count(s->s_rb_nw2app));
			}
			if (s->s_rb_app2nw != NULL) {
				fprintf(os, ", app2nw %" PRIuRBLEN,
				    ringbuff_get_count(s->s_rb_app2nw));
			}
			fputc('\n', os);
		}
#endif
	}

	if (os == stdout)
		network_ioctl(ns, NETWORK_IOCTL_STATS, NULL);
}
#endif /* NETWORK_OPT_COMMAND */

struct network_state *
network_register(const struct network_driver *nd)
{
	struct network_state *ns = &network_state;
	struct network_socket *s;

	/*
	 * Prevent multiple drivers (for now)
	 */
	if (ns->ns_driver)
		return NULL;

	/*
	 * Let's get going
	 */
	ns->ns_driver = nd;
	ns->ns_state = NETWORK_STATE_DOWN;
	ns->ns_link_up = false;
	ns->ns_link_status.ls_state = NETWORK_LINK_DOWN;

	TAILQ_INIT(&ns->ns_sockq);
	TAILQ_INIT(&ns->ns_sock_retryq);

	/*
	 * The back-end driver is not re-entrant so we must serialise
	 * access to it using a mutex when running under a pre-emtive
	 * RTOS.
	 *
	 * Note: This mutex needs to be initialised early on.
	 */
	ns->ns_driver_mutex = rtos_mutex_create();
	assert(ns->ns_driver_mutex != NULL);

	/*
	 * Register a link-change callback with the driver.
	 */
	network_drv_link_status(ns, network_link_status_handler, ns);

	/*
	 * Start a thread for processing driver events.
	 * The back-end driver is expected to generate hardware interrupts
	 * which cause this thread to be scheduled.
	 *
	 * If the driver declares a link-check entry point then we are
	 * expected to call it periodically to check if the network 'link'
	 * is still active (as in, Ethernet cable is plugged in).
	 * Some devices can notify link status via interrupt, so this
	 * entry point is NULL.
	 */
	if (ns->ns_driver->nd_link_check != NULL) {
		/*
		 * Check link every 5 seconds
		 */
		ns->ns_link_timer = rtos_timer_create("NetLinkTimer", 5000, 1,
		    network_schedule_link_check, ns);
		assert(ns->ns_link_timer != NULL);
	}

	ns->ns_soft_timer = rtos_timer_create("NetSoftTimer", 1000, 0,
	    network_soft_timer_event, ns);
	assert(ns->ns_soft_timer != NULL);
	rtos_timer_start(ns->ns_soft_timer);

	/*
	 * Start the driver event thread.
	 */
	ns->ns_driver_task = rtos_task_create("NetDriver",
	    network_driver_thread, ns, RTOS_TASK_PRIORITY_MED + 1,
	    NETWORK_DRIVER_STACK_SIZE);
	assert(ns->ns_driver_task != NULL);

	rtos_notify(ns->ns_driver_task, NETWORK_EVENT_INTERRUPT,
	    RTOS_NOTIFY_ACTION_SET_BITS);

	/*
	 * Initialise the sockets.
	 */
	s = &ns->ns_sockets[0];
	for (unsigned int i = 0; i < NETWORK_NSOCKETS; i++, s++) {
		memset(s, 0, sizeof(*s));
		s->s_dsock = NETWORK_SOCKET_INVALID;
		s->s_driver = ns;
		s->s_mutex = rtos_mutex_create();
		assert(s->s_mutex != NULL);
	}

	/*
	 * Create the socket worker task
	 */
	ns->ns_socket_task = rtos_task_create("NetSocket",
	    network_socket_thread, ns, RTOS_TASK_PRIORITY_MED,
	    NETWORK_SOCKET_STACK_SIZE);
	assert(ns->ns_socket_task != NULL);

	ns->ns_retry_timer = rtos_timer_create("NetRetryTimer", 2, 0,
	    network_retry_timer_event, ns);
	assert(ns->ns_retry_timer != NULL);

#ifndef NETWORK_LWIP
	network_ip_configure(ns);
#endif

#ifdef NETWORK_OPT_COMMAND
	SHELL_CMD_ADD(net);
#endif

	return ns;
}

void
network_load_defaults(void)
{

#if defined(NETWORK_OPT_DHCP_CLIENT) || \
    defined(NETWORK_OPT_EXTERNAL_DHCP_CLIENT)
	const uint32_t ip = 0;
	const uint32_t mask = 0;
	const uint32_t gate = 0;
#else	/* NETWORK_OPT_DHCP_CLIENT */
	const uint32_t ip = 0xc0a802d3u;
	const uint32_t mask = 0xffffff00u;
	const uint32_t gate = 0xc0a80201u;
#endif	/* NETWORK_OPT_DHCP_CLIENT || NETWORK_OPT_EXTERNAL_DHCP_CLIENT */

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_network_ip = ip;
	glob.glob_network_mask = mask;
	glob.glob_network_gate = gate;
#else /* CONFIG_USE_CONFIGDB */
	cf_set_uint32(CF_KEY_UINT32_NETWORK_IP, ip);
	cf_set_uint32(CF_KEY_UINT32_NETWORK_MASK, mask);
	cf_set_uint32(CF_KEY_UINT32_NETWORK_GATE, gate);
#endif /* CONFIG_USE_CONFIGDB */
}
