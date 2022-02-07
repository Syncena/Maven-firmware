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

#ifndef TUSB_OSAL_CUSTOM_H
#define TUSB_OSAL_CUSTOM_H

#include "rtos.h"

#define	OSAL_MSEC_TO_SCW(msec)	\
	(((msec) == OSAL_TIMEOUT_WAIT_FOREVER) ? 0 : (msec))

//--------------------------------------------------------------------+
// TASK API
//--------------------------------------------------------------------+
static inline void
osal_task_delay(uint32_t msec)
{

	rtos_task_sleep((rtos_tick_t)msec);
}

//--------------------------------------------------------------------+
// Semaphore API
//--------------------------------------------------------------------+
typedef rtos_sema_static_t osal_semaphore_def_t;
typedef rtos_sema_t osal_semaphore_t;

static inline osal_semaphore_t
osal_semaphore_create(osal_semaphore_def_t *semdef)
{

	return rtos_sema_create_binary_static(semdef);
}

static inline bool
osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr)
{
	rtos_base_type_t rv;

	(void) in_isr;

	rv = rtos_sema_give_any(sem_hdl);

	return rv != 0;
}

static inline bool
osal_semaphore_wait(osal_semaphore_t sem_hdl, uint32_t msec)
{

	return rtos_sema_take_timed(sem_hdl, OSAL_MSEC_TO_SCW(msec));
}

static inline void
osal_semaphore_reset(osal_semaphore_t const sem_hdl)
{

	rtos_sema_reset(sem_hdl);
}

//--------------------------------------------------------------------+
// MUTEX API (priority inheritance)
//--------------------------------------------------------------------+
typedef rtos_mutex_static_t osal_mutex_def_t;
typedef rtos_mutex_t osal_mutex_t;

static inline osal_mutex_t
osal_mutex_create(osal_mutex_def_t *mdef)
{

	return rtos_mutex_create_static(mdef);
}

static inline bool
osal_mutex_lock(osal_mutex_t mutex_hdl, uint32_t msec)
{

	return rtos_mutex_acquire_timed(mutex_hdl, OSAL_MSEC_TO_SCW(msec)) != 0;
}

static inline bool
osal_mutex_unlock(osal_mutex_t mutex_hdl)
{

	return rtos_mutex_release(mutex_hdl) != 0;
}

//--------------------------------------------------------------------+
// QUEUE API
//--------------------------------------------------------------------+

// role device/host is used by OS NONE for mutex (disable usb isr) only
#define OSAL_QUEUE_DEF(_role, _name, _depth, _type) 	\
	static _type _name##_##buf[_depth];		\
	osal_queue_def_t _name = {			\
		.depth = _depth,			\
		.item_sz = sizeof(_type),		\
		.buf = _name##_##buf }

typedef struct {
	uint16_t	depth;
	uint16_t	item_sz;
	void		*buf;
	StaticQueue_t	sq;
} osal_queue_def_t;

typedef QueueHandle_t osal_queue_t;

static inline osal_queue_t
osal_queue_create(osal_queue_def_t* qdef)
{

	return xQueueCreateStatic(qdef->depth, qdef->item_sz,
	    (uint8_t*) qdef->buf, &qdef->sq);
}

static inline bool
osal_queue_receive(osal_queue_t qhdl, void* data)
{

	return xQueueReceive(qhdl, data, portMAX_DELAY);
}

static inline bool
osal_queue_send(osal_queue_t qhdl, void const * data, bool in_isr)
{
	BaseType_t rv;

	(void) in_isr;

	if (_rtos_interrupt_context() == 0) {
		rv = xQueueSendToBack(qhdl, data, OSAL_TIMEOUT_WAIT_FOREVER);
	} else {
		rtos_saved_ipl_t ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		rv = xQueueSendToBackFromISR(qhdl, data,
		    RTOS_NEED_CTX_SWITCH_PTR);
		rtos_ipl_restore(ipl);
	}

	return rv != 0;
}

static inline bool
osal_queue_empty(osal_queue_t qhdl)
{

	return uxQueueMessagesWaiting(qhdl) == 0;
}

#endif /* TUSB_OSAL_CUSTOM_H */
