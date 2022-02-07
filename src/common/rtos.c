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
 * A thin veneer around FreeRTOS
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include "rtos.h"
#include "fix16.h"
#include "timer.h"
#ifdef RTOS_OPTION_SHELL
#include "shell.h"
#endif
#include "zone_alloc.h"
#include "debug.h"

UBaseType_t _rtos_scheduler_started;
volatile BaseType_t _rtos_need_context_switch;

#ifndef PLATFORM_RTOS_ZONE
#define	PLATFORM_RTOS_ZONE	NULL
#endif

#if (configSUPPORT_STATIC_ALLOCATION != 0)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
void
vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
#if (configSUPPORT_DYNAMIC_ALLOCATION == 0)
	static struct {
		rtos_task_static_t task;
		rtos_ubase_type_t stack[RTOS_STACK_SIZE(configMINIMAL_STACK_SIZE)];
	} rtos_idle_memory;

	*ppxIdleTaskTCBBuffer = &rtos_idle_memory.task.ts_task;
	*ppxIdleTaskStackBuffer = rtos_idle_memory.stack;
	*pulIdleTaskStackSize = sizeof(rtos_idle_memory.stack /
	    sizeof(rtos_idle_memory.stack[0]);
#else
	size_t stack_size = RTOS_STACK_SIZE(configMINIMAL_STACK_SIZE *
	    sizeof(rtos_ubase_type_t));

	*ppxIdleTaskStackBuffer = pvPortMalloc(stack_size);
	*ppxIdleTaskTCBBuffer = pvPortMalloc(sizeof(rtos_task_static_t));
	*pulIdleTaskStackSize = stack_size / sizeof(rtos_ubase_type_t);
#endif
}

#if (configUSE_TIMERS != 0)
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTickTaskTCBBuffer,
    StackType_t **ppxTickTaskStackBuffer, uint32_t *pulTickTaskStackSize);
void
vApplicationGetTimerTaskMemory(StaticTask_t **ppxTickTaskTCBBuffer,
    StackType_t **ppxTickTaskStackBuffer, uint32_t *pulTickTaskStackSize)
{
#if (configSUPPORT_DYNAMIC_ALLOCATION == 0)
	static struct {
		rtos_task_static_t task;
		rtos_ubase_type_t stack[RTOS_STACK_SIZE(configTIMER_TASK_STACK_DEPTH)];
	} rtos_systick_memory;
	*ppxTickTaskTCBBuffer = &rtos_systick_memory.task.ts_task;
	*ppxTickTaskStackBuffer = rtos_systick_memory.stack;
	*pulTickTaskStackSize = sizeof(rtos_systick_memory.stack) /
	    sizeof(rtos_ubase_type_t);
#else
	const size_t stack_size = RTOS_STACK_SIZE(configTIMER_TASK_STACK_DEPTH *
	    sizeof(rtos_ubase_type_t));

	*ppxTickTaskStackBuffer = pvPortMalloc(stack_size);
	*ppxTickTaskTCBBuffer = pvPortMalloc(sizeof(rtos_task_static_t));
	*pulTickTaskStackSize = stack_size / sizeof(rtos_ubase_type_t);
#endif
}
#endif /* (configUSE_TIMERS != 0) */
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */

#if (configCHECK_FOR_STACK_OVERFLOW != 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void
vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{

	(void) xTask;

	__disable_irq();

	debug_print("Task '%s' blew its stack\n", pcTaskName);
	debug_flush();
	for (;;);
}
#endif /* (configCHECK_FOR_STACK_OVERFLOW != 0) */

#ifdef RTOS_OPTION_SHELL
#if (configUSE_TRACE_FACILITY == 0)
#error "You must set configUSE_TRACE_FACILITY to 1 for ps support"
#endif
SHELL_CMD_DECL(ps, rtos_cmd_ps, "Displays thread info");

#if (configSUPPORT_STATIC_ALLOCATION != 0)
static rtos_mutex_static_t rtos_cmd_mutex_store;
#endif /* (configSUPPORT_STATIC_ALLOCATION != 0) */
static rtos_mutex_t rtos_cmd_mutex;

static __always_inline void
rtos_cmd_mutex_acquire(void)
{

	if (_rtos_scheduler_started == 0)
		return;

	rtos_mutex_acquire(rtos_cmd_mutex);
}

static __always_inline void
rtos_cmd_mutex_release(void)
{

	if (_rtos_scheduler_started == 0)
		return;

	rtos_mutex_release(rtos_cmd_mutex);
}

static int
rtos_task_sorter(void *arg, const void *t1, const void *t2)
{
	const TaskStatus_t *ts1 = t1, *ts2 = t2;

#if (configGENERATE_RUN_TIME_STATS == 0)
	(void) arg;
#else
	if (*(int *)arg != 0)
		return (int)ts1->ulRunTimeCounter - (int)ts2->ulRunTimeCounter;
	else
#endif /* (configGENERATE_RUN_TIME_STATS != 0) */

	return (int)ts1->xTaskNumber - (int)ts2->xTaskNumber;
}

static void
rtos_cmd_ps(FILE *os, uint8_t argc, const char * const *argv)
{
	TaskStatus_t *all_tasks;
	TaskStatus_t *ts;
	uint32_t ntasks;
	unsigned long uptime;
	const char *state;
	uint8_t i;
	int sort_field;
	struct timeval tv;
	char str[24];
#if (configGENERATE_RUN_TIME_STATS != 0)
	uint32_t t1;
#endif

	timer_get_mono_time(&tv);

#if (configGENERATE_RUN_TIME_STATS != 0)
	/* "ps -I" sorts by ID, otherwise by CPU usage */
	sort_field = !(argc == 1 && strcmp(argv[0], "-I") == 0);
#else
	sort_field = 0;
	(void) argc;
	(void) argv;
#endif

	rtos_cmd_mutex_acquire();

	ntasks = (uint32_t)uxTaskGetNumberOfTasks();
	all_tasks = zone_malloc(ntasks * sizeof(*all_tasks));
	if (all_tasks == NULL) {
#ifdef RTOS_DEBUG
		fprintf(os, "Out of memory for %" PRIu32 " tasks\n", ntasks);
#endif
		return;
	}

	(void) uxTaskGetSystemState(all_tasks, ntasks, &uptime);

	if (ntasks > 1) {
		qsort_r(all_tasks, ntasks, sizeof(all_tasks[0]),
		    &sort_field, rtos_task_sorter);
	}

	fprintf(os, "%" PRIu32 " tasks:\n", ntasks);
#if (configGENERATE_RUN_TIME_STATS != 0)
	fprintf(os,
	    "TID Name         State     BasePri CurPri Stack  CPU Usage\n");
#else
	fprintf(os,
	    "TID Name         State     BasePri CurPri Stack\n");
#endif

	for (i = 0, ts = all_tasks; i < ntasks; i++, ts++) {
		switch (ts->eCurrentState) {
		case eRunning:
			state = "Running";
			break;
		case eReady:
			state = "Ready";
			break;
		case eBlocked:
			state = "Blocked";
			break;
		case eSuspended:
			state = "Suspend";
			break;
		case eDeleted:
			state = "Deleted";
			break;
		default:
			state = "Invalid";
			break;
		}

#if (configGENERATE_RUN_TIME_STATS != 0)
		t1 = ts->ulRunTimeCounter;
#endif

#if (configGENERATE_RUN_TIME_STATS != 0)
		fprintf(os,
		    "%3lu %-12s %-8s %5lu   %5lu   %5u"
		    "  %2lu.%06lu\n"
		    , ts->xTaskNumber, ts->pcTaskName, state,
		    ts->uxBasePriority, ts->uxCurrentPriority,
		    ts->usStackHighWaterMark * sizeof(rtos_ubase_type_t)
		    , t1 / 1000000ul, t1 % 1000000ul);
#else
		fprintf(os,
		    "%3lu %-12s %-8s %5lu   %5lu   %5u\n"
		    , ts->xTaskNumber, ts->pcTaskName, state,
		    ts->uxBasePriority, ts->uxCurrentPriority,
		    ts->usStackHighWaterMark * sizeof(rtos_ubase_type_t));
#endif

	}

	timer_timeval_to_string(str, sizeof(str), &tv);
	fprintf(os, "System up time: %s\n", str);

	zone_free(all_tasks);

	rtos_cmd_mutex_release();

#ifdef RTOS_OPTION_IRQ_STATS
	if (argc >= 1 && strcmp(argv[0], "-i") == 0) {
		extern void _rtos_print_irq_stats(FILE *);

		_rtos_print_irq_stats(os);
	}
#endif /* RTOS_OPTION_IRQ_STATS */
}
#endif /* RTOS_OPTION_SHELL */

#if (configSUPPORT_DYNAMIC_ALLOCATION != 0)
void *
pvPortMalloc(size_t xSize)
{

	return zone_malloc_private(PLATFORM_RTOS_ZONE, xSize);
}

void
vPortFree(void *pv)
{

	zone_free_private(PLATFORM_RTOS_ZONE, pv);
}
#endif /* (configSUPPORT_DYNAMIC_ALLOCATION != 0) */

void
rtos_init(void)
{

#ifdef	RTOS_OPTION_SHELL
	rtos_cmd_mutex = rtos_mutex_create_static(&rtos_cmd_mutex_store);
	SHELL_CMD_ADD(ps);
#endif
}

void
rtos_scheduler_start(void)
{

	_rtos_scheduler_started = 1;
	vTaskStartScheduler();
}

void
rtos_assert_fail(const char *condstr, const char *file, int line)
{

	__disable_irq();

	debug_print("Assertion failed: \"%s\" at %s:%d\n", condstr, file, line);
	debug_flush();

	abort();
	for (;;);
}
