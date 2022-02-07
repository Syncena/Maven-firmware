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

#ifndef RINGBUFF_IMPL_H
#define RINGBUFF_IMPL_H

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#if (RINGBUFF_BLOCKING_COUNT != 0)
#define	RINGBUFF_BLOCKING
#endif
#if !defined(RINGBUFF_BLOCKING)
#define	RINGBUFF_BLOCKING
#endif

#ifdef RINGBUFF_BLOCKING
#include "rtos.h"
#endif

typedef uint32_t ringbuff_len_t;
#define	PRIuRBLEN	PRIu32

struct ringbuff;
typedef struct ringbuff *ringbuff_t;
typedef void (*ringbuff_cb_t)(ringbuff_t, void *);

struct ringbuff {
	uint8_t *rb_producer_p;
	ringbuff_cb_t rb_producer_cb;
	void *rb_producer_cookie;

	uint8_t *rb_consumer_p;
	ringbuff_cb_t rb_consumer_cb;
	ringbuff_cb_t rb_consumer_expedite_cb;
#ifdef RINGBUFF_OOBMSG
	ringbuff_oobmsg_cb_t rb_consumer_oobmsg_cb;
#endif
	void *rb_consumer_cookie;

	volatile ringbuff_len_t rb_count;

#ifdef RINGBUFF_BLOCKING
	rtos_sema_t rb_producer_sem;
	rtos_sema_t rb_consumer_sem;
#endif /* RINGBUFF_BLOCKING */

#ifdef RINGBUFF_SIZE
#define	_RB_SIZE	RINGBUFF_SIZE
	uint8_t rb_buff[RINGBUFF_SIZE];
#else
#define	_RB_SIZE	rb->rb_size
	uint8_t *rb_buff;
	ringbuff_len_t rb_size;
#endif
};

#ifdef RINGBUFF_OOBMSG
typedef enum {
	RINGBUFF_OOBMSG_SEND_BREAK,
} ringbuff_oobmsg_t;

typedef void (*ringbuff_oobmsg_cb_t)(ringbuff_t, ringbuff_oobmsg_t, void *);
#endif

#ifdef RINGBUFF_SIZE
extern ringbuff_t	ringbuff_alloc(void);
#else
extern ringbuff_t	ringbuff_alloc(uint8_t *, ringbuff_len_t);
#endif
extern void		ringbuff_free(ringbuff_t);

extern void		ringbuff_init(ringbuff_t);

static __inline ringbuff_len_t
ringbuff_get_count(ringbuff_t rb)
{

	return rb->rb_count;
}

static __inline ringbuff_len_t
ringbuff_get_space(ringbuff_t rb)
{

	return _RB_SIZE - rb->rb_count;
}

static __inline bool
ringbuff_is_full(ringbuff_t rb)
{

	return ringbuff_get_space(rb) == 0;
}

static __inline bool
ringbuff_is_empty(ringbuff_t rb)
{

	return ringbuff_get_count(rb) == 0;
}

extern void		ringbuff_producer_init(ringbuff_t, ringbuff_cb_t,
			    void *);
extern void		ringbuff_produce(ringbuff_t, uint8_t);
extern void		ringbuff_produce_done(ringbuff_t);
extern ringbuff_len_t	ringbuff_produce_contig_size(ringbuff_t);
extern ringbuff_len_t	ringbuff_produce_contig_next_size(ringbuff_t,
			    ringbuff_len_t);
extern void		ringbuff_produce_contig(ringbuff_t, ringbuff_len_t);
extern void		*ringbuff_produce_current_buff_pointer(ringbuff_t);
extern void		*ringbuff_produce_next_buff_pointer(ringbuff_t,
			    ringbuff_len_t);
extern ringbuff_len_t	ringbuff_produce_copy(ringbuff_t, const void *,
			    ringbuff_len_t);
#ifdef RINGBUFF_OOBMSG
extern void		ringbuff_produce_oobmsg(ringbuff_t, ringbuff_oobmsg_t);
#endif

extern void		ringbuff_consumer_init(ringbuff_t, ringbuff_cb_t,
			    void *);
#ifdef RINGBUFF_OOBMSG
extern void		ringbuff_consumer_oobmsg_init(ringbuff_t,
			    ringbuff_oobmsg_cb_t);
#endif
extern uint8_t		ringbuff_consume(ringbuff_t);
extern void 		ringbuff_consume_done(ringbuff_t);
extern void		ringbuff_consumer_expedite_init(ringbuff_t,
			    ringbuff_cb_t);
extern void		ringbuff_consume_expedite(ringbuff_t);
extern ringbuff_len_t	ringbuff_consume_contig_size(ringbuff_t);
extern void		ringbuff_consume_contig(ringbuff_t, ringbuff_len_t);
extern void		*ringbuff_consume_current_buff_pointer(ringbuff_t);
extern ringbuff_len_t	ringbuff_consume_copy(ringbuff_t, void *,
			    ringbuff_len_t);

#ifdef RINGBUFF_BLOCKING
struct ringbuff_clocking;
typedef struct ringbuff_blocking *ringbuff_blocking_t;
extern ringbuff_blocking_t ringbuff_blocking_consumer(ringbuff_t, rtos_tick_t);
extern ringbuff_blocking_t ringbuff_blocking_producer(ringbuff_t, rtos_tick_t);
extern void		ringbuff_blocking_close(ringbuff_blocking_t);
extern int		ringbuff_blocking_read(ringbuff_blocking_t,
			    void *, ringbuff_len_t);
extern int		ringbuff_blocking_write(ringbuff_blocking_t,
			    const void *, ringbuff_len_t);
extern int		ringbuff_blocking_getc(ringbuff_blocking_t);
extern int		ringbuff_blocking_putc(ringbuff_blocking_t, uint8_t);
#endif /* RINGBUFF_BLOCKING */

#endif /* RINGBUFF_IMPL */
