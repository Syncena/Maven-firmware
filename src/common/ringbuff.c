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
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include "rtos.h"
#include "ringbuff.h"
#include "zone_alloc.h"

#ifdef RINGBUFF_BLOCKING
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "rtos.h"
#endif /* RINGBUFF_BLOCKING */

#ifdef RINGBUFF_BLOCKING
typedef struct ringbuff_blocking {
	ringbuff_t rbl_rb;
	int rbl_flags;
#define	RINGBUFF_BLOCKING_FLAG_CONSUMER	(1u << 15)
	rtos_tick_t rbl_timeout;
	volatile uint32_t rbl_dead;
	volatile uint32_t rbl_blocked;
#if (configSUPPORT_STATIC_ALLOCATION != 0)
	rtos_sema_static_t rbl_sem_store;
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */
} *ringbuff_blocking_t;
static rtos_mutex_t ringbuff_blocking_mutex;
#endif /* RINGBUFF_BLOCKING */
static uint8_t ringbuff_init_done;

#define	RINGBUFF_COUNT_UPDATE(rb,op,cnt)		\
	do {						\
		rtos_saved_ipl_t _ipl;			\
		_ipl = rtos_ipl_raise(HW_IPL_MAX);	\
		(rb)->rb_count op cnt;			\
		rtos_ipl_restore(_ipl);			\
	} while (/*CONSTCOND*/0)

void
ringbuff_init(ringbuff_t rb)
{

#if defined(RINGBUFF_BLOCKING) && defined(DEBUG)
	assert(rb->rb_consumer_sem == NULL && rb->rb_producer_sem == NULL);
#endif

	rb->rb_producer_p = rb->rb_buff;
	rb->rb_producer_cb = NULL;
	rb->rb_consumer_p = rb->rb_buff;
	rb->rb_consumer_cb = NULL;
	rb->rb_consumer_expedite_cb = NULL;
	rb->rb_count = 0;
}

ringbuff_t
#ifdef RINGBUFF_SIZE
ringbuff_alloc(void)
#else
ringbuff_alloc(uint8_t *buff, ringbuff_len_t size)
#endif
{
	rtos_saved_ipl_t ipl;
	ringbuff_t rb;
	uint8_t init_done;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	init_done = ringbuff_init_done;
	ringbuff_init_done = 1;
	rtos_ipl_restore(ipl);

	if (init_done == 0) {
#ifdef RINGBUFF_BLOCKING
		ringbuff_blocking_mutex = rtos_mutex_create();
		assert(ringbuff_blocking_mutex != NULL);
#endif /* RINGBUFF_BLOCKING */
	}

	rb = zone_calloc(1, sizeof(*rb));
	if (rb != NULL) {
#ifndef RINGBUFF_SIZE
		rb->rb_buff = buff;
		rb->rb_size = size;
#endif
		ringbuff_init(rb);
	}

	return rb;
}

void
ringbuff_free(ringbuff_t rb)
{

	zone_free(rb);
}

void
ringbuff_producer_init(ringbuff_t rb, ringbuff_cb_t cb, void *arg)
{

	rb->rb_producer_cookie = arg;
	rb->rb_producer_cb = cb;
}

void
ringbuff_consumer_init(ringbuff_t rb, ringbuff_cb_t cb, void *arg)
{

	rb->rb_consumer_cookie = arg;
	rb->rb_consumer_cb = cb;
}

void
ringbuff_consumer_expedite_init(ringbuff_t rb, ringbuff_cb_t cb)
{

	rb->rb_consumer_expedite_cb = cb;
}

#ifdef RINGBUFF_OOBMSG
void
ringbuff_consumer_oobmsg_init(ringbuff_t rb, ringbuff_oobmsg_cb_t cb)
{

	rb->rb_consumer_oobmsg_cb = cb;
}
#endif

void
ringbuff_produce(ringbuff_t rb, uint8_t d)
{

	if (ringbuff_is_full(rb))
		return;

	*rb->rb_producer_p++ = d;
	if (rb->rb_producer_p == &rb->rb_buff[_RB_SIZE])
		rb->rb_producer_p = rb->rb_buff;

	RINGBUFF_COUNT_UPDATE(rb, +=, 1);
}

#ifdef RINGBUFF_OOBMSG
void
ringbuff_produce_oobmsg(ringbuff_t rb, ringbuff_oobmsg_t msg)
{

	if (rb->rb_consumer_oobmsg_cb != NULL) {
		(void)(rb->rb_consumer_oobmsg_cb)(rb, msg,
		    rb->rb_consumer_cookie);
	}
}
#endif

void
ringbuff_produce_done(ringbuff_t rb)
{

	if (rb->rb_consumer_cb != NULL)
		(void)(rb->rb_consumer_cb)(rb, rb->rb_consumer_cookie);

#ifdef RINGBUFF_BLOCKING
	if (rb->rb_consumer_sem != NULL)
		rtos_sema_give_any(rb->rb_consumer_sem);
#endif /* RINGBUFF_BLOCKING */
}

uint8_t
ringbuff_consume(ringbuff_t rb)
{
	uint8_t rv;

	if (ringbuff_is_empty(rb))
		return 0;

	rv = *rb->rb_consumer_p++;
	if (rb->rb_consumer_p == &rb->rb_buff[_RB_SIZE])
		rb->rb_consumer_p = rb->rb_buff;

	RINGBUFF_COUNT_UPDATE(rb, -=, 1);

	return rv;
}

void
ringbuff_consume_done(ringbuff_t rb)
{

	if (rb->rb_producer_cb != NULL)
		(void)(rb->rb_producer_cb)(rb, rb->rb_producer_cookie);

#ifdef RINGBUFF_BLOCKING
	if (rb->rb_producer_sem != NULL)
		rtos_sema_give_any(rb->rb_producer_sem);
#endif /* RINGBUFF_BLOCKING */
}

void
ringbuff_consume_expedite(ringbuff_t rb)
{

	if (rb->rb_consumer_expedite_cb != NULL) {
		(void)(rb->rb_consumer_expedite_cb)(rb,
		    rb->rb_consumer_cookie);
	}
}

ringbuff_len_t
ringbuff_consume_contig_size(ringbuff_t rb)
{
	ringbuff_len_t l, c;

	l = &rb->rb_buff[_RB_SIZE] - rb->rb_consumer_p;
	c = rb->rb_count;

	return (l > c) ? c : l;
}

void
ringbuff_consume_contig(ringbuff_t rb, ringbuff_len_t len)
{

	rb->rb_consumer_p += len;
	if (rb->rb_consumer_p >= &rb->rb_buff[_RB_SIZE])
		rb->rb_consumer_p -= _RB_SIZE;

	RINGBUFF_COUNT_UPDATE(rb, -=, len);
}

ringbuff_len_t
ringbuff_produce_contig_size(ringbuff_t rb)
{
	ringbuff_len_t len, c;

	len = &rb->rb_buff[_RB_SIZE] - rb->rb_producer_p;
	c = rb->rb_count;

	c = _RB_SIZE - c;

	return (len > c) ? c : len;
}

ringbuff_len_t
ringbuff_produce_contig_next_size(ringbuff_t rb, ringbuff_len_t len)
{
	ringbuff_len_t c;
	uint8_t *p;

	p = rb->rb_producer_p;
	c = rb->rb_count;

	/* Fake 'ringbuff_produce_contig() */
	p += len;
	if (p >= &rb->rb_buff[_RB_SIZE])
		p -= _RB_SIZE;
	c += len;

	/* Now fake 'ringbuff_produce_contig_size()' */
	len = &rb->rb_buff[_RB_SIZE] - p;
	c = _RB_SIZE - c;

	return (len > c) ? c : len;
}

void
ringbuff_produce_contig(ringbuff_t rb, ringbuff_len_t len)
{

	rb->rb_producer_p += len;
	if (rb->rb_producer_p >= &rb->rb_buff[_RB_SIZE])
		rb->rb_producer_p -= _RB_SIZE;

	RINGBUFF_COUNT_UPDATE(rb, +=, len);
}

void *
ringbuff_produce_current_buff_pointer(ringbuff_t rb)
{

	return (void *)rb->rb_producer_p;
}

void *
ringbuff_produce_next_buff_pointer(ringbuff_t rb, ringbuff_len_t len)
{
	uint8_t *p;

	p = rb->rb_producer_p;

	/* Fake 'ringbuff_produce_contig() */
	p += len;
	if (p >= &rb->rb_buff[_RB_SIZE])
		p -= _RB_SIZE;

	return (void *)p;
}

ringbuff_len_t
ringbuff_consume_copy(ringbuff_t rb, void *dst, ringbuff_len_t len)
{
	ringbuff_len_t l;
	uint8_t *p;

	assert(len <= rb->rb_count);

	for (p = dst, l = len; l > 0; l--)
		*p++ = ringbuff_consume(rb);

	return len;
}

ringbuff_len_t
ringbuff_produce_copy(ringbuff_t rb, const void *dst, ringbuff_len_t len)
{
	ringbuff_len_t l;
	const uint8_t *p;

	assert(len <= ringbuff_get_space(rb));

	for (p = dst, l = len; l > 0; l--)
		ringbuff_produce(rb, *p++);

	return len;
}

void *
ringbuff_consume_current_buff_pointer(ringbuff_t rb)
{

	return (void *)rb->rb_consumer_p;
}

#ifdef RINGBUFF_BLOCKING
static ringbuff_blocking_t
ringbuff_blocking_alloc(void)
{
	ringbuff_blocking_t rbl;

	rbl = zone_calloc(1, sizeof(*rbl));
	return rbl;
}

static void
ringbuff_blocking_free(ringbuff_blocking_t rbl)
{

	zone_free(rbl);
}

#ifndef min
#define	min(a,b)	(((a) <= (b)) ? (a) : (b))
#endif

static __always_inline int
ringbuff_blocking_wait(rtos_sema_t sem, ringbuff_blocking_t rbl)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t st;

	ipl = rtos_ipl_raise(HW_IPL_MAX);
	if (rbl->rbl_dead == 0)
		rbl->rbl_blocked = 1;
	rtos_ipl_restore(ipl);

	if (rbl->rbl_blocked) {
		if (rbl->rbl_timeout)
			st = rtos_sema_take_timed(sem, rbl->rbl_timeout);
		else {
			st = 1;
			rtos_sema_take(sem);
		}

		rbl->rbl_blocked = 0;

		if (st == 0)
			return 0;
	}

	if (rbl->rbl_dead)
		return -1;

	return 1;
}

int
ringbuff_blocking_read(ringbuff_blocking_t rbl, void *p, ringbuff_len_t len)
{
	ringbuff_t rb = rbl->rbl_rb;
	ringbuff_len_t rblen, total = len;
	uint8_t *buff = p;
	int rv;

	while (len) {
		while (ringbuff_is_empty(rb)) {
			rv = ringbuff_blocking_wait(rb->rb_consumer_sem, rbl);
			if (rv < 0)
				return rv;
			if (rv == 0 && ringbuff_is_empty(rb))
				goto out;
		}

		rblen = ringbuff_consume_contig_size(rb);
		rblen = min(rblen, len);

		if (rblen > 1) {
			memcpy(buff, (void *)rb->rb_consumer_p, rblen);
			ringbuff_consume_contig(rb, rblen);
		} else {
			*buff = ringbuff_consume(rb);
		}

		buff += rblen;
		len -= rblen;

		ringbuff_consume_done(rb);
	}

 out:
	return (int)(total - len);
}

int
ringbuff_blocking_getc(ringbuff_blocking_t rbl)
{
	uint8_t ch;

	if (ringbuff_blocking_read(rbl, &ch, 1) == 1)
		return (int)ch & 0xff;
	return -1;
}

int
ringbuff_blocking_write(ringbuff_blocking_t rbl, const void *p,
    ringbuff_len_t len)
{
	ringbuff_t rb = rbl->rbl_rb;
	ringbuff_len_t rblen, total = len;
	const uint8_t *buff = p;
	int rv;

	while (len) {
		while (ringbuff_is_full(rb)) {
			ringbuff_produce_done(rb);
			rv = ringbuff_blocking_wait(rb->rb_producer_sem, rbl);
			if (rv < 0)
				return rv;
			if (rv == 0 && ringbuff_is_full(rb))
				goto out;
		}

		rblen = ringbuff_produce_contig_size(rb);
		rblen = min(rblen, len);

		if (rblen > 1) {
			memcpy((void *)rb->rb_producer_p, buff, rblen);
			ringbuff_produce_contig(rb, rblen);
		} else {
			ringbuff_produce(rb, *buff);
		}

		buff += rblen;
		len -= rblen;
	}

 out:
	return (int)(total - len);
}

int
ringbuff_blocking_putc(ringbuff_blocking_t rbl, uint8_t ch)
{

	if (ringbuff_blocking_write(rbl, &ch, 1) == 1)
		return 1;
	return -1;
}

ringbuff_blocking_t
ringbuff_blocking_consumer(ringbuff_t rb, rtos_tick_t timeout)
{
	ringbuff_blocking_t rbl;
	rtos_sema_t sem;

	/* There can be only one blocking consumer */
	if (rb->rb_consumer_sem != NULL)
		return NULL;

	rtos_mutex_acquire(ringbuff_blocking_mutex);

	if ((rbl = ringbuff_blocking_alloc()) != NULL) {
#if (configSUPPORT_STATIC_ALLOCATION != 0)
		sem = rtos_sema_create_binary_static(&rbl->rbl_sem_store);
#else
		sem = rtos_sema_create_binary();
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

		if (sem != NULL) {
			rbl->rbl_flags = RINGBUFF_BLOCKING_FLAG_CONSUMER;
			rbl->rbl_rb = rb;
			rbl->rbl_timeout = timeout;
			rb->rb_consumer_sem = sem;
		} else {
			ringbuff_blocking_free(rbl);
			rbl = NULL;
		}
	}

	rtos_mutex_release(ringbuff_blocking_mutex);

	return rbl;
}

ringbuff_blocking_t
ringbuff_blocking_producer(ringbuff_t rb, rtos_tick_t timeout)
{
	ringbuff_blocking_t rbl;
	rtos_sema_t sem;

	/* There can be only one blocking producer */
	if (rb->rb_producer_sem != NULL)
		return NULL;

	rtos_mutex_acquire(ringbuff_blocking_mutex);

	if ((rbl = ringbuff_blocking_alloc()) != NULL) {
#if (configSUPPORT_STATIC_ALLOCATION != 0)
		sem = rtos_sema_create_binary_static(&rbl->rbl_sem_store);
#else
		sem = rtos_sema_create_binary();
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

		if (sem != NULL) {
			rbl->rbl_flags = 0;
			rbl->rbl_rb = rb;
			rbl->rbl_timeout = timeout;
			rb->rb_producer_sem = sem;
		} else {
			ringbuff_blocking_free(rbl);
			rbl = NULL;
		}
	}

	rtos_mutex_release(ringbuff_blocking_mutex);

	return rbl;
}

void
ringbuff_blocking_close(ringbuff_blocking_t rbl)
{
	rtos_sema_t sem;

	rtos_mutex_acquire(ringbuff_blocking_mutex);

	if ((rbl->rbl_flags & RINGBUFF_BLOCKING_FLAG_CONSUMER) != 0) {
		if (rbl->rbl_rb != NULL) {
			sem = rbl->rbl_rb->rb_consumer_sem;
			rbl->rbl_rb->rb_consumer_sem = NULL;
		} else {
			sem = NULL;
		}

		if (sem != NULL) {
			rbl->rbl_dead = 1;
			while (rbl->rbl_blocked) {
				rtos_sema_give(sem);
				rtos_task_sleep(500);
			}
#if (configSUPPORT_STATIC_ALLOCATION == 0)
#error foo
			rtos_sema_delete(sem);
#endif
		}
	} else {
		if (rbl->rbl_rb != NULL) {
			sem = rbl->rbl_rb->rb_producer_sem;
			rbl->rbl_rb->rb_producer_sem = NULL;
		} else {
			sem = NULL;
		}

		if (sem != NULL) {
			rbl->rbl_dead = 1;
			while (rbl->rbl_blocked) {
				rtos_sema_give(sem);
				rtos_task_sleep(10);
			}
#if (configSUPPORT_STATIC_ALLOCATION == 0)
			rtos_sema_delete(sem);
#endif
		}
	}

	ringbuff_blocking_free(rbl);

	rtos_mutex_release(ringbuff_blocking_mutex);
}
#endif /* RINGBUFF_BLOCKING */
