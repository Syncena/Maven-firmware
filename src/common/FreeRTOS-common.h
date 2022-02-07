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

#ifndef FREERTOS_COMMON_H
#define FREERTOS_COMMON_H

#if (RELEASE_BUILD == 0)
#define	configDEBUG_OPTIONS			1
#else
#define	configDEBUG_OPTIONS			0
#endif

#define configUSE_PREEMPTION			1
#define configUSE_TIME_SLICING			1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			0
#define	configUSE_TIMERS			1
#define	configTIMER_QUEUE_LENGTH		16
#define	configTIMER_TASK_PRIORITY		(configMAX_PRIORITIES - 1)
#define	configTIMER_TASK_STACK_DEPTH		150u
#define	configUSE_MUTEXES			1
#define	configUSE_RECURSIVE_MUTEXES		1
#define	configUSE_COUNTING_SEMAPHORES		0
#define	configUSE_APPLICATION_TASK_TAG		0
#if configDEBUG_OPTIONS
#define configUSE_TRACE_FACILITY		1
#else
#define configUSE_TRACE_FACILITY		0
#endif
#define	configUSE_QUEUE_SETS			0
#define configUSE_16_BIT_TICKS			0

/* These are FreeRTOS' built-in usage stats... */
#if 0
#define	configGENERATE_RUN_TIME_STATS		1
extern uint32_t timer_get_mono_usecs(void);
#define	portGET_RUN_TIME_COUNTER_VALUE()	timer_get_mono_usecs()
#define	portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() do { } while (0)
#endif

#define configCPU_CLOCK_HZ			F_CPU
#define configTICK_RATE_HZ			1000

#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE			1500u
#endif /* configTOTAL_HEAP_SIZE */

#define configMAX_TASK_NAME_LEN			16
#define configIDLE_SHOULD_YIELD			0
#define configQUEUE_REGISTRY_SIZE		0
#define	configSUPPORT_STATIC_ALLOCATION		1
#define	configSUPPORT_DYNAMIC_ALLOCATION	1

/* We're not using co-routines */
#define configUSE_CO_ROUTINES 			0

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete			0
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend			0
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1
#define INCLUDE_xQueueGetMutexHolder		1

#ifndef NDEBUG
extern void rtos_assert_fail(const char *, const char *, int);
#define configASSERT(x)							\
	do {								\
		if ((x) == 0)						\
			rtos_assert_fail(#x, __FILE__, __LINE__);	\
	} while (0)
#endif

#endif /* FREERTOS_COMMON_H */
