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

/*
 * A thin veneer around FreeRTOS.
 * Because the FreeRTOS API is too long-winded for me.
 */

#ifndef RTOS_H
#define RTOS_H

#include <stdbool.h>
#include <stdio.h>

/*
 * Grab the FreeRTOS kernel headers.
 * XXX: All their header file names should be prefixed with something like
 * 'FreeRTOS-' to avoid clashing with local headers.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#if (!defined(__OPTIMIZE__) || (__OPTIMIZE__ == 0)) || (__FPU_USED)
#define	RTOS_STACK_SIZE(x)	((x) + ((x) / 2))
#else
#define	RTOS_STACK_SIZE(x)	(x)
#endif

#define	rtos_scheduler_end()		vTaskEndScheduler()
#define	rtos_scheduler_suspend()				\
		do {						\
			__asm __volatile("" ::: "memory");	\
			if (_rtos_scheduler_started)		\
				vTaskSuspendAll();		\
		} while (0)
#define	rtos_scheduler_resume()					\
		do {						\
			if (_rtos_scheduler_started)		\
				xTaskResumeAll();		\
			__asm __volatile("" ::: "memory");	\
		} while (0)

#ifdef likely
#undef likely
#endif
#ifdef unlikely
#undef unlikely
#endif
#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

typedef	BaseType_t	rtos_base_type_t;
typedef	UBaseType_t	rtos_ubase_type_t;
typedef TickType_t	rtos_tick_t;

/* ========================================================================== */

struct rtos_task;
typedef struct rtos_task *rtos_task_t;

typedef	TaskFunction_t	rtos_task_function_t;
typedef struct {
	StaticTask_t	ts_task;
} rtos_task_static_t;
typedef eTaskState	rtos_task_state_t;
#define	RTOS_TASK_STATE_RUNNING		eRunning
#define	RTOS_TASK_STATE_READY		eReady
#define	RTOS_TASK_STATE_BLOCKED		eBlocked
#define	RTOS_TASK_STATE_SUSPENDED	eSuspended
#define	RTOS_TASK_STATE_DELETED		eDeleted
#define	RTOS_TASK_STATE_INVALID		eInvalid
typedef struct {
	rtos_task_t ts_task;
	const char *ts_name;
	rtos_ubase_type_t ts_number;
	rtos_task_state_t ts_state;
	rtos_ubase_type_t ts_priority_cur;
	rtos_ubase_type_t ts_priority;
	uint32_t ts_cpu_time;
	void *ts_stack_base;
	uint16_t ts_stack_high_water;
} rtos_task_status_t;
#define	RTOS_TASK_PRIORITY_BASE		tskIDLE_PRIORITY
#define	RTOS_TASK_PRIORITY_LOW		(tskIDLE_PRIORITY + 1)
#define	RTOS_TASK_PRIORITY_MED		(configMAX_PRIORITIES / 2)
#define	RTOS_TASK_PRIORITY_MAX		(configMAX_PRIORITIES - 1)

/* ========================================================================== */

typedef eNotifyAction	rtos_notify_action_t;
#define	RTOS_NOTIFY_ACTION_NONE		eNoAction
#define	RTOS_NOTIFY_ACTION_SET_BITS	eSetBits
#define	RTOS_NOTIFY_ACTION_INCREMENT	eIncrement
#define	RTOS_NOTIFY_ACTION_FORCE_VALUE	eSetValueWithOverwrite
#define	RTOS_NOTIFY_ACTION_SET_VALUE	eSetValueWithoutOverwrite

/* ========================================================================== */

#if (configUSE_TIMERS != 0)
struct rtos_timer;
typedef struct rtos_timer *rtos_timer_t;
typedef	StaticTimer_t		rtos_timer_static_t;
typedef void (*rtos_timer_function_t)(rtos_timer_t);
typedef PendedFunction_t	rtos_timer_pend_function_t;
#endif /* (configUSE_TIMERS != 0) */

/* ========================================================================== */

struct rtos_sema;
typedef struct rtos_sema *rtos_sema_t;
typedef StaticSemaphore_t	rtos_sema_static_t;

/* ========================================================================== */

struct rtos_mutex;
typedef struct rtos_mutex *rtos_mutex_t;
typedef StaticSemaphore_t	rtos_mutex_static_t;

/* ========================================================================== */

extern UBaseType_t		_rtos_scheduler_started;
extern volatile BaseType_t	_rtos_need_context_switch;
#define	RTOS_NEED_CTX_SWITCH_PTR	\
	((BaseType_t *)(uintptr_t)&_rtos_need_context_switch)


/* ========================================================================== */

#define	RTOS_MAX_SLEEP_TICKS	portMAX_DELAY
#define	RTOS_MAX_SLEEP_MS	portMAX_DELAY

static __always_inline TickType_t
_rtos_timer_ms_to_ticks(rtos_tick_t ms)
{

	if (ms == 0 || ms == RTOS_MAX_SLEEP_MS)
		return RTOS_MAX_SLEEP_TICKS;
	else
		return pdMS_TO_TICKS(ms);  /* XXX: This overflows to easily */
}

/* ========================================================================== */

#ifdef RTOS_DEBUG
#define	RTOS_CHECK_IRQ_CONTEXT()					    \
	do {								    \
		if (_rtos_interrupt_context() != 0) {			    \
			extern void debug_print(const char *, ...);	    \
			extern void debug_flush(void);			    \
			uintptr_t _ret;					    \
			_ret = (uintptr_t)__builtin_return_address(0);	    \
			debug_print("%s: called from interrupt context. "   \
			    "caller %p\n", __func__, (void *)_ret);	    \
			debug_print("task %s\n", pcTaskGetName(NULL));	    \
			debug_flush();					    \
			for (;;);					    \
		}							    \
	} while (0)
#else
#define	RTOS_CHECK_IRQ_CONTEXT()	do { } while (0)
#endif

/* ========================================================================== */

#if (configSUPPORT_DYNAMIC_ALLOCATION != 0)
static 	__always_inline rtos_task_t
rtos_task_create(const char *name, rtos_task_function_t entry_point,
		 void *entry_point_arg, uint16_t priority,
		 size_t stack_len)
{
	TaskHandle_t task_handle;

	xTaskCreate((TaskFunction_t)entry_point,
		     name,
		     (uint16_t) (stack_len / sizeof(StackType_t)),
		     entry_point_arg,
		     (UBaseType_t) priority,
		     &task_handle);

	return (rtos_task_t) task_handle;
}

static __always_inline void
rtos_task_delete(rtos_task_t task)
{

	vTaskDelete((TaskHandle_t) task);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) */

#if (configSUPPORT_STATIC_ALLOCATION != 0)
static 	__always_inline rtos_task_t
rtos_task_create_static(const char *name, rtos_task_function_t entry_point,
			void *entry_point_arg, uint16_t priority,
			size_t stack_len, void *stack_buffer,
			rtos_task_static_t *task_buffer)
{

	xTaskCreateStatic((TaskFunction_t)entry_point,
		          name,
		          (uint16_t) (stack_len / sizeof(StackType_t)),
		          entry_point_arg,
		          (UBaseType_t) priority,
		          stack_buffer,
			  &task_buffer->ts_task);

	return (rtos_task_t) task_buffer;
}
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

static __always_inline void
rtos_task_sleep(const rtos_tick_t ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(ms);

	vTaskDelay(ticks);
}

static __always_inline void
rtos_task_sleep_until(rtos_tick_t * const prev, const rtos_tick_t ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(ms);

	vTaskDelayUntil(prev, ticks);
}

static __always_inline rtos_base_type_t
rtos_task_sleep_abort(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xTaskAbortDelay((TaskHandle_t)task);
}

static __always_inline rtos_ubase_type_t
rtos_task_priority_get(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_ubase_type_t)uxTaskPriorityGet((TaskHandle_t)task);
}

static __always_inline rtos_ubase_type_t
rtos_task_priority_get_isr(rtos_task_t task)
{
	rtos_saved_ipl_t ipl;
	rtos_ubase_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_ubase_type_t)uxTaskPriorityGetFromISR((TaskHandle_t)task);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_task_state_t
rtos_task_get_state(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_task_state_t)eTaskGetState((TaskHandle_t)task);
}

static __always_inline void
rtos_task_get_info(rtos_task_t task, rtos_task_status_t *pts,
    rtos_base_type_t get_free_stack)
{
	TaskStatus_t st;

	RTOS_CHECK_IRQ_CONTEXT();

	vTaskGetInfo((TaskHandle_t) task, &st, (BaseType_t) get_free_stack,
	    eInvalid);

	pts->ts_task = (rtos_task_t) st.xHandle;
	pts->ts_name = st.pcTaskName;
	pts->ts_number = (rtos_ubase_type_t) st.xTaskNumber;
	pts->ts_state = (rtos_task_state_t) st.eCurrentState;
	pts->ts_priority_cur = (rtos_ubase_type_t) st.uxCurrentPriority;
	pts->ts_priority = (rtos_ubase_type_t) st.uxBasePriority;
	pts->ts_cpu_time = st.ulRunTimeCounter;
	pts->ts_stack_base = (void *)st.pxStackBase;
	pts->ts_stack_high_water = st.usStackHighWaterMark;
}

static __always_inline void
rtos_task_priority_set(rtos_task_t task, rtos_ubase_type_t priority)
{

	RTOS_CHECK_IRQ_CONTEXT();

	vTaskPrioritySet((TaskHandle_t) task, (UBaseType_t) priority);
}

static __always_inline void
rtos_task_suspend(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	vTaskSuspend((TaskHandle_t) task);
}

static __always_inline void
rtos_task_resume(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	vTaskResume((TaskHandle_t) task);
}

static __always_inline rtos_ubase_type_t
rtos_task_resume_isr(rtos_task_t task)
{
	rtos_saved_ipl_t ipl;
	rtos_ubase_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	/* Returns non-zero if a context-switch may be required */
	rv = (rtos_ubase_type_t) xTaskResumeFromISR((TaskHandle_t) task);
	if (rv)
		_rtos_need_context_switch = 1;

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_ubase_type_t
rtos_task_get_count(void)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_ubase_type_t) uxTaskGetNumberOfTasks();
}

static __always_inline const char *
rtos_task_get_name(rtos_task_t task)
{

	return pcTaskGetName((TaskHandle_t) task);
}

static __always_inline rtos_task_t
rtos_task_from_name(const char *name)
{

	return (rtos_task_t) xTaskGetHandle(name);
}

static __always_inline rtos_task_t
rtos_task_current(void)
{

	return (rtos_task_t) xTaskGetCurrentTaskHandle();
}

/* ========================================================================== */

static __always_inline rtos_base_type_t
rtos_notify(rtos_task_t task, uint32_t v, rtos_notify_action_t a)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xTaskNotify((TaskHandle_t) task, v,
	    (eNotifyAction) a);
}

static __always_inline rtos_base_type_t
rtos_notify_isr(rtos_task_t task, uint32_t v, rtos_notify_action_t a)
{
	rtos_saved_ipl_t ipl;
	rtos_ubase_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTaskNotifyFromISR((TaskHandle_t) task, v,
	    (eNotifyAction) a, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_notify_any(rtos_task_t task, uint32_t v, rtos_notify_action_t a)
{

	if (_rtos_interrupt_context())
		return rtos_notify_isr(task, v, a);

	return rtos_notify(task, v, a);
}

static __always_inline rtos_base_type_t
rtos_notify_give(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return rtos_notify(task, 0, RTOS_NOTIFY_ACTION_INCREMENT);
}

static __always_inline rtos_base_type_t
rtos_notify_give_isr(rtos_task_t task)
{

	return rtos_notify_isr(task, 0, RTOS_NOTIFY_ACTION_INCREMENT);
}

static __always_inline rtos_base_type_t
rtos_notify_give_any(rtos_task_t task)
{

	if (_rtos_interrupt_context())
		return rtos_notify_isr(task, 0, RTOS_NOTIFY_ACTION_INCREMENT);

	return rtos_notify(task, 0, RTOS_NOTIFY_ACTION_INCREMENT);
}

static __always_inline rtos_base_type_t
rtos_notify_and_query(rtos_task_t task, uint32_t v,
    rtos_notify_action_t a, uint32_t *p)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xTaskNotifyAndQuery((TaskHandle_t) task, v,
	    (eNotifyAction) a, p);
}

static __always_inline rtos_base_type_t
rtos_notify_and_query_isr(rtos_task_t task, uint32_t v,
    rtos_notify_action_t a, uint32_t *p)
{
	rtos_saved_ipl_t ipl;
	rtos_ubase_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTaskNotifyAndQueryFromISR((TaskHandle_t) task,
	    v, (eNotifyAction) a, p, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_notify_and_query_any(rtos_task_t task, uint32_t v,
    rtos_notify_action_t a, uint32_t *p)
{

	if (_rtos_interrupt_context())
		return rtos_notify_and_query_isr(task, v, a, p);

	return rtos_notify_and_query(task, v, a, p);
}

static __always_inline rtos_base_type_t
rtos_notify_wait(uint32_t cbits_entry, uint32_t cbits_exit,
    uint32_t *value, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTaskNotifyWait(cbits_entry, cbits_exit,
	    value, ticks);
}

static __always_inline void
rtos_notify_take(rtos_base_type_t clr_on_exit)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (ulTaskNotifyTake((BaseType_t) clr_on_exit,
	    RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline uint32_t
rtos_notify_take_timed(rtos_base_type_t clr_on_exit, rtos_tick_t ms_to_wait)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(ms_to_wait);

	return ulTaskNotifyTake((BaseType_t) clr_on_exit, ticks);
}

static __always_inline rtos_base_type_t
rtos_notify_clear(rtos_task_t task)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xTaskNotifyStateClear((TaskHandle_t) task);
}

/* ========================================================================== */

static __always_inline rtos_tick_t
rtos_tick_get_count(void)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_tick_t) xTaskGetTickCount();
}

static __always_inline rtos_tick_t
rtos_tick_get_count_isr(void)
{
	rtos_saved_ipl_t ipl;
	rtos_tick_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_tick_t) xTaskGetTickCountFromISR();

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_tick_t
rtos_tick_get_count_any(void)
{

	if (_rtos_interrupt_context())
		return rtos_tick_get_count_isr();

	return rtos_tick_get_count();
}

/* ========================================================================== */

#if (configUSE_TIMERS != 0 && ((configSUPPORT_DYNAMIC_ALLOCATION != 0) || \
    (configSUPPORT_STATIC_ALLOCATION != 0)))
#if (configSUPPORT_DYNAMIC_ALLOCATION != 0)
static __always_inline rtos_timer_t
rtos_timer_create(const char *name, rtos_tick_t ms,
    rtos_ubase_type_t auto_reload, rtos_timer_function_t func,
    void *timer_id)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();
 
 	ticks = _rtos_timer_ms_to_ticks(ms);

	return (rtos_timer_t) xTimerCreate(name, ticks,
	    (UBaseType_t) auto_reload, timer_id,
	    (TimerCallbackFunction_t) func);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) */

static __always_inline void
rtos_timer_delete(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (xTimerDelete((TimerHandle_t) timer, RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_delete_timed(rtos_timer_t timer, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerDelete((TimerHandle_t) timer, ticks);
}

#if (configSUPPORT_STATIC_ALLOCATION != 0)
static __always_inline rtos_timer_t
rtos_timer_create_static(const char *name, rtos_tick_t ms,
    rtos_ubase_type_t auto_reload, rtos_timer_function_t func,
    void *timer_id, rtos_timer_static_t *timer_buffer)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(ms);

	return (rtos_timer_t) xTimerCreateStatic(name, ticks,
	    (UBaseType_t) auto_reload, timer_id,
	    (TimerCallbackFunction_t) func,
	    (StaticTimer_t *) timer_buffer);
}
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

static __always_inline void *
rtos_timer_get_id(const rtos_timer_t timer)
{

	return pvTimerGetTimerID((const TimerHandle_t) timer);
}

static __always_inline void
rtos_timer_set_id(rtos_timer_t timer, void *id)
{

	RTOS_CHECK_IRQ_CONTEXT();

	vTimerSetTimerID((TimerHandle_t) timer, id);
}

static __always_inline rtos_base_type_t
rtos_timer_is_active(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xTimerIsTimerActive((TimerHandle_t)timer);
}

static __always_inline void
rtos_timer_start(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (xTimerStart((TimerHandle_t) timer, RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_start_timed(rtos_timer_t timer, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerStart((TimerHandle_t) timer, ticks);
}

static __always_inline rtos_base_type_t
rtos_timer_start_isr(rtos_timer_t timer)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTimerStartFromISR((TimerHandle_t) timer,
	    RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_timer_start_any(rtos_timer_t timer)
{

	if (_rtos_interrupt_context())
		return rtos_timer_start_isr(timer);

	rtos_timer_start(timer);
	return 0;
}

static __always_inline void
rtos_timer_stop(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (xTimerStop((TimerHandle_t) timer, RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_stop_timed(rtos_timer_t timer, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerStop((TimerHandle_t) timer, ticks);
}

static __always_inline rtos_base_type_t
rtos_timer_stop_isr(rtos_timer_t timer)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTimerStopFromISR((TimerHandle_t) timer,
	    RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_timer_stop_any(rtos_timer_t timer)
{

	if (_rtos_interrupt_context())
		return rtos_timer_stop_isr(timer);

	rtos_timer_stop(timer);
	return 0;
}

static __always_inline void
rtos_timer_change_wait_period(rtos_timer_t timer, rtos_tick_t new_period)
{
	TickType_t period_ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	period_ticks = _rtos_timer_ms_to_ticks(new_period);

	while (xTimerChangePeriod((TimerHandle_t) timer, period_ticks,
	    RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_change_wait_period_timed(rtos_timer_t timer, rtos_tick_t new_period,
    rtos_tick_t wait_ms)
{
	TickType_t period_ticks, wait_ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	period_ticks = _rtos_timer_ms_to_ticks(new_period);
 	wait_ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerChangePeriod((TimerHandle_t) timer,
	    period_ticks, wait_ticks);
}

static __always_inline rtos_base_type_t
rtos_timer_change_wait_period_isr(rtos_timer_t timer, rtos_tick_t new_period)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;
	TickType_t ticks;

 	ticks = _rtos_timer_ms_to_ticks(new_period);

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTimerChangePeriodFromISR((TimerHandle_t) timer,
	    ticks, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_timer_change_wait_period_any(rtos_timer_t timer, rtos_tick_t new_period)
{

	if (_rtos_interrupt_context())
		return rtos_timer_change_wait_period_isr(timer, new_period);

	rtos_timer_change_wait_period(timer, new_period);
	return 0;
}

static __always_inline void
rtos_timer_reset(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (xTimerReset((TimerHandle_t) timer, RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_reset_timed(rtos_timer_t timer, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerReset((TimerHandle_t) timer, ticks);
}


static __always_inline rtos_base_type_t
rtos_timer_reset_isr(rtos_timer_t timer)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTimerResetFromISR((TimerHandle_t) timer,
	    RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_timer_reset_any(rtos_timer_t timer)
{

	if (_rtos_interrupt_context())
		return rtos_timer_reset_isr(timer);

	rtos_timer_reset(timer);
	return 0;
}

static __always_inline void
rtos_timer_pend_function(rtos_timer_pend_function_t func, void *arg1,
    uint32_t arg2)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (xTimerPendFunctionCall((PendedFunction_t) func, arg1, arg2,
	    RTOS_MAX_SLEEP_TICKS) == 0)
		;
}

static __always_inline rtos_base_type_t
rtos_timer_pend_function_timed(rtos_timer_pend_function_t func, void *arg1,
    uint32_t arg2, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xTimerPendFunctionCall(
	    (PendedFunction_t) func, arg1, arg2, ticks);
}

static __always_inline rtos_base_type_t
rtos_timer_pend_function_isr(rtos_timer_pend_function_t func, void *arg1,
    uint32_t arg2)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xTimerPendFunctionCallFromISR(
	    (PendedFunction_t) func, arg1, arg2, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_timer_pend_function_any(rtos_timer_pend_function_t func, void *arg1,
    uint32_t arg2)
{

	if (_rtos_interrupt_context())
		return rtos_timer_pend_function_isr(func, arg1, arg2);

	rtos_timer_pend_function(func, arg1, arg2);
	return 0;
}

static __always_inline const char *
rtos_timer_get_name(rtos_timer_t timer)
{

	return pcTimerGetName((TimerHandle_t) timer);
}

static __always_inline rtos_tick_t
rtos_timer_get_period(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_tick_t) xTimerGetPeriod((TimerHandle_t) timer);
}

static __always_inline rtos_tick_t
rtos_timer_get_expiry_time(rtos_timer_t timer)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_tick_t) xTimerGetExpiryTime((TimerHandle_t) timer);
}
#endif /* (configUSE_TIMERS != 0) && ((configSUPPORT_DYNAMIC_ALLOCATION != 0)||\
	  (configSUPPORT_STATIC_ALLOCATION != 0)) */

/* ========================================================================== */

#if (configSUPPORT_DYNAMIC_ALLOCATION != 0) || \
    (configSUPPORT_STATIC_ALLOCATION != 0)
#if (configSUPPORT_DYNAMIC_ALLOCATION != 0)
static __always_inline rtos_sema_t
rtos_sema_create_binary(void)
{

	return (rtos_sema_t) xSemaphoreCreateBinary();
}

static __always_inline rtos_sema_t
rtos_sema_create_counting(rtos_ubase_type_t max_count,
    rtos_ubase_type_t start_count)
{

	return (rtos_sema_t) xSemaphoreCreateCounting(
	    (UBaseType_t) max_count, (UBaseType_t) start_count);
}

static __always_inline void
rtos_sema_delete(rtos_sema_t sema)
{

	vSemaphoreDelete((SemaphoreHandle_t) sema);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) */

#if (configSUPPORT_STATIC_ALLOCATION != 0)
static __always_inline rtos_sema_t
rtos_sema_create_binary_static(rtos_sema_static_t *psema)
{

	return (rtos_sema_t) xSemaphoreCreateBinaryStatic(
	    (StaticSemaphore_t *) psema);
}

static __always_inline rtos_sema_t
rtos_sema_create_counting_static(rtos_ubase_type_t max_count,
    rtos_ubase_type_t start_count, rtos_sema_static_t *psema)
{

	return (rtos_sema_t) xSemaphoreCreateCountingStatic(
	    (UBaseType_t) max_count, (UBaseType_t) start_count,
	    (StaticSemaphore_t *) psema);
}
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

static __always_inline void
rtos_sema_take(rtos_sema_t sema)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (!xSemaphoreTake((SemaphoreHandle_t) sema, RTOS_MAX_SLEEP_TICKS))
		;
}

static __always_inline rtos_base_type_t
rtos_sema_take_timed(rtos_sema_t sema, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xSemaphoreTake((SemaphoreHandle_t) sema,
	    ticks);
}

static __always_inline rtos_base_type_t
rtos_sema_take_isr(rtos_sema_t sema)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xSemaphoreTakeFromISR(
	    (SemaphoreHandle_t) sema, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_sema_take_any(rtos_sema_t sema)
{

	if (_rtos_interrupt_context())
		return rtos_sema_take_isr(sema);

	rtos_sema_take(sema);
	return 0;
}

static __always_inline rtos_base_type_t
rtos_sema_take_timed_any(rtos_sema_t sema, rtos_tick_t wait_ms)
{

	if (_rtos_interrupt_context())
		return rtos_sema_take_isr(sema);

	return rtos_sema_take_timed(sema, wait_ms);
}

static __always_inline rtos_base_type_t
rtos_sema_give(rtos_sema_t sema)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xSemaphoreGive((SemaphoreHandle_t) sema);
}

static __always_inline rtos_base_type_t
rtos_sema_give_isr(rtos_sema_t sema)
{
	rtos_saved_ipl_t ipl;
	rtos_base_type_t rv;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);

	rv = (rtos_base_type_t) xSemaphoreGiveFromISR(
	    (SemaphoreHandle_t) sema, RTOS_NEED_CTX_SWITCH_PTR);

	rtos_ipl_restore(ipl);

	return rv;
}

static __always_inline rtos_base_type_t
rtos_sema_give_any(rtos_sema_t sema)
{

	if (_rtos_interrupt_context())
		return rtos_sema_give_isr(sema);

	return rtos_sema_give(sema);
}

#if (configUSE_RECURSIVE_MUTEXES != 0)
static __always_inline void
rtos_sema_take_recursive(rtos_sema_t sema)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (!xSemaphoreTakeRecursive((SemaphoreHandle_t) sema,
	    RTOS_MAX_SLEEP_TICKS)) {
		/* Do nothing */
	}
}

static __always_inline rtos_base_type_t
rtos_sema_take_recursive_timed(rtos_sema_t sema, rtos_tick_t block_ticks)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(block_ticks);

	return (rtos_base_type_t) xSemaphoreTakeRecursive(
	    (SemaphoreHandle_t) sema, ticks);
}

static __always_inline rtos_base_type_t
rtos_sema_give_recursive(rtos_sema_t sema)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xSemaphoreGiveRecursive(
	    (SemaphoreHandle_t) sema);
}
#endif /* (configUSE_RECURSIVE_MUTEXES != 0) */

static __always_inline rtos_ubase_type_t
rtos_sema_get_count(rtos_sema_t sema)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_ubase_type_t) uxSemaphoreGetCount(
	    (SemaphoreHandle_t) sema);
}

static __always_inline void
rtos_sema_reset(rtos_sema_t sema)
{

	xQueueReset((SemaphoreHandle_t) sema);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) || \
	  (configSUPPORT_STATIC_ALLOCATION != 0) */

/* ========================================================================== */
#if (configUSE_MUTEXES != 0)
#if (configSUPPORT_DYNAMIC_ALLOCATION != 0)
static __always_inline rtos_mutex_t
rtos_mutex_create(void)
{

	return (rtos_mutex_t) xSemaphoreCreateMutex();
}

#if (configUSE_RECURSIVE_MUTEXES != 0)
static __always_inline rtos_mutex_t
rtos_mutex_create_recursive(void)
{

	return (rtos_mutex_t) xSemaphoreCreateRecursiveMutex();
}
#endif /* (configUSE_RECURSIVE_MUTEXES != 0) */

static __always_inline void
rtos_mutex_delete(rtos_mutex_t mutex)
{

	vSemaphoreDelete((SemaphoreHandle_t) mutex);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) */

#if (configSUPPORT_STATIC_ALLOCATION != 0)
static __always_inline rtos_mutex_t
rtos_mutex_create_static(rtos_mutex_static_t *pmutex)
{

	return (rtos_mutex_t) xSemaphoreCreateMutexStatic(
	    (StaticSemaphore_t *) pmutex);
}

#if (configUSE_RECURSIVE_MUTEXES != 0)
static __always_inline rtos_mutex_t
rtos_mutex_create_recursive_static(rtos_mutex_static_t *pmutex)
{

	return (rtos_mutex_t) xSemaphoreCreateRecursiveMutexStatic(
	    (StaticSemaphore_t *) pmutex);
}
#endif /* (configUSE_RECURSIVE_MUTEXES != 0) */
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

static __always_inline void
rtos_mutex_acquire(rtos_mutex_t mutex)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (!xSemaphoreTake((SemaphoreHandle_t) mutex, RTOS_MAX_SLEEP_TICKS))
		;
}

static __always_inline rtos_base_type_t
rtos_mutex_acquire_timed(rtos_mutex_t mutex, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xSemaphoreTake((SemaphoreHandle_t) mutex,
	    ticks);
}

static __always_inline rtos_base_type_t
rtos_mutex_release(rtos_mutex_t mutex)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xSemaphoreGive((SemaphoreHandle_t) mutex);
}

#if (configUSE_RECURSIVE_MUTEXES != 0)
static __always_inline void
rtos_mutex_acquire_recursive(rtos_mutex_t mutex)
{

	RTOS_CHECK_IRQ_CONTEXT();

	while (!xSemaphoreTakeRecursive((SemaphoreHandle_t)mutex,
	    RTOS_MAX_SLEEP_TICKS))
		;
}

static __always_inline rtos_base_type_t
rtos_mutex_acquire_recursive_timed(rtos_mutex_t mutex, rtos_tick_t wait_ms)
{
	TickType_t ticks;

	RTOS_CHECK_IRQ_CONTEXT();

 	ticks = _rtos_timer_ms_to_ticks(wait_ms);

	return (rtos_base_type_t) xSemaphoreTakeRecursive(
	    (SemaphoreHandle_t) mutex, ticks);
}

static __always_inline rtos_base_type_t
rtos_mutex_release_recursive(rtos_mutex_t mutex)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_base_type_t) xSemaphoreGiveRecursive(
	    (SemaphoreHandle_t) mutex);
}
#endif /* (configUSE_RECURSIVE_MUTEXES != 0) */

static __always_inline rtos_task_t
rtos_mutex_get_holder(rtos_mutex_t mutex)
{

	RTOS_CHECK_IRQ_CONTEXT();

	return (rtos_task_t) xSemaphoreGetMutexHolder(
	    (SemaphoreHandle_t) mutex);
}
#endif /* (configUSE_MUTEXES != 0) */

/* ========================================================================== */

static __always_inline void
rtos_yield(void)
{

	RTOS_CHECK_IRQ_CONTEXT();

	taskYIELD();
}

static __always_inline bool
rtos_interrupt_mode(void)
{

	return _rtos_interrupt_context() != 0;
}

static __always_inline bool
rtos_scheduler_running(void)
{

	if (_rtos_scheduler_started == 0)
		return false;
#if (configUSE_TIMERS == 1)
	return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
#else
	return true;
#endif
}

/*
 * Must be invoked from main() before any further rtos API functions
 * are called.
 */
extern void rtos_init(void);

/*
 * Must be invoked from main() in order to start normal task scheduling.
 * This function does not return.
 */
extern void rtos_scheduler_start(void);

#ifdef NDEBUG
#define	rtos_irq_register(v,p,f,a)	   rtos_irq_register_impl(v,p,f,a)
#define	rtos_irq_register_named(n,v,p,f,a) rtos_irq_register_impl(v,p,f,a)
extern void *rtos_irq_register_impl(int vec, uint32_t pri,
    void (*func)(void *), void *arg);
#else
#define	rtos_irq_register(v,p,f,a)	   rtos_irq_register_impl(NULL,v,p,f,a)
#define	rtos_irq_register_named(n,v,p,f,a) rtos_irq_register_impl(n,v,p,f,a)
extern void *rtos_irq_register_impl(const char *, int vec, uint32_t pri,
    void (*func)(void *), void *arg);
#endif
extern void rtos_irq_unregister(void *handle);

#ifdef RTOS_OPTION_SOFT_IRQ
#ifdef NDEBUG
#define	rtos_soft_irq_register(f,a)		\
		rtos_soft_irq_register_impl(f,a)
#define	rtos_soft_irq_register_named(n,f,a)	\
		rtos_soft_irq_register_impl(f,a)
extern void *rtos_soft_irq_register_impl(void (*func)(void *), void *arg);
#else
#define	rtos_soft_irq_register(f,a)		\
		rtos_soft_irq_register_impl(NULL,f,a)
#define	rtos_soft_irq_register_named(n,f,a)	\
		rtos_soft_irq_register_impl(n,f,a)
extern void *rtos_soft_irq_register_impl(const char *,
		void (*func)(void *), void *arg);
#endif /* NDEBUG */
extern void rtos_soft_irq_unregister(void *);
extern void FAST_RAMFUNC rtos_soft_irq_schedule(void *);
#endif /* RTOS_OPTION_SOFT_IRQ */

extern bool rtos_catch_fault(void (*)(void *), void *);

extern void rtos_assert_fail(const char *, const char *, int);

#endif /* RTOS_H */
