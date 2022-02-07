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

#ifndef PORTMACRO_H
#define PORTMACRO_H

#include "platform.h"
#include "platform_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Port-specific type definitions and constants.
 */
#define	portCHAR			signed char
#define	portFLOAT			float
#define	portDOUBLE			double
#define	portLONG			long
#define	portSHORT			short
#define	portSTACK_TYPE			uint32_t
#define	portBASE_TYPE			int32_t
#define	portUBASE_TYPE			uint32_t

typedef	portSTACK_TYPE			StackType_t;
typedef	portBASE_TYPE			BaseType_t;
typedef	portUBASE_TYPE			UBaseType_t;
typedef	portUBASE_TYPE			TickType_t;
typedef	uint32_t			rtos_saved_ipl_t;

#define	portMAX_DELAY			((TickType_t)0xfffffffful)
#define	portTICK_TYPE_IS_ATOMIC		1

#define	portSTACK_GROWTH		(-1)
#define	portTICK_PERIOD_MS		((TickType_t)1000 / configTICK_RATE_HZ)
#define	portBYTE_ALIGNMENT		8

#define	portTASK_FUNCTION_PROTO(f, a)	void f(void *a)
#define	portTASK_FUNCTION(f, a)		void f(void *a)

#if (__CORTEX_M > 0U) && (!defined(__NVIC_PRIO_BITS) || (__NVIC_PRIO_BITS < 3))
#error "No valid definition for __NVIC_PRIO_BITS"
#endif
#define	portINTERRUPT_PRIORITIES	(1u << __NVIC_PRIO_BITS)
#define	portINTERRUPT_LOWEST_PRIORITY	(portINTERRUPT_PRIORITIES - 1u)
#define	portINTERRUPT_HIGHEST_PRIORITY	(1u)

#if (defined (__ARM_ARCH_8M_MAIN__ ) && (__ARM_ARCH_8M_MAIN__ == 1))
#define	portHAS_STACK_OVERFLOW_CHECKING	1
#else
#define	portHAS_STACK_OVERFLOW_CHECKING	0
#endif

#define	portMEMORY_BARRIER()		__asm __volatile("":::"memory")

/*
 * Port-specific functions
 */
extern void portYIELD(void);

#if (__CORTEX_M > 0U)
static __always_inline rtos_saved_ipl_t
rtos_ipl_raise(uint32_t ipl)
{
	uint32_t pri;

	pri = __get_BASEPRI();
#if (defined(__CM7_REV) && (__CM7_REV <= 1U)) || \
    defined(configPORT_FORCE_M7_R0P1_ERRATUM)
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	__set_BASEPRI_MAX((ipl << (8u - __NVIC_PRIO_BITS)) & 0xffu);
	__set_PRIMASK(pm);
#else
	__set_BASEPRI_MAX((ipl << (8u - __NVIC_PRIO_BITS)) & 0xffu);
#endif

	return (rtos_saved_ipl_t) pri;
}

static __always_inline void
rtos_ipl_restore(rtos_saved_ipl_t pri)
{

#if (defined(__CM7_REV) && (__CM7_REV <= 1U)) || \
    defined(configPORT_FORCE_M7_R0P1_ERRATUM)
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	__set_BASEPRI((uint32_t)pri);
	__set_PRIMASK(pm);
#else
	__set_BASEPRI((uint32_t)pri);
#endif
}
#else /* Cortex-M0/M0+ */
static __always_inline rtos_saved_ipl_t
rtos_ipl_raise(uint32_t ipl)
{
	uint32_t rv;

	(void) ipl;

	rv = __get_PRIMASK();
	__disable_irq();

	return (rtos_saved_ipl_t) rv;
}

static __always_inline void
rtos_ipl_restore(rtos_saved_ipl_t pri)
{

	__set_PRIMASK((uint32_t)pri);
}
#endif /* Cortex-M0/M0+ */

#define portDISABLE_INTERRUPTS()		\
		(void)rtos_ipl_raise(HW_IPL_SCHEDULER)
#define	portSET_INTERRUPT_MASK_FROM_ISR()	\
		(UBaseType_t)rtos_ipl_raise(HW_IPL_SCHEDULER)
#define	portCLEAR_INTERRUPT_MASK_FROM_ISR(x)	\
		rtos_ipl_restore((rtos_saved_ipl_t)(x))

extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);
#define portENTER_CRITICAL()		vPortEnterCritical()
#define portEXIT_CRITICAL()		vPortExitCritical()

static __always_inline uint8_t
_rtos_interrupt_context(void)
{

	return __get_IPSR() != 0;
}

/*
 * We can use the ARM "clz" instruction to optimise task selection
 * if the number of priorities will fit into 32-bits.
 */
#if (configMAX_PRIORITIES <= 32)
#define	configUSE_PORT_OPTIMISED_TASK_SELECTION 1

#define	portRECORD_READY_PRIORITY(p, rp)	\
	do {					\
		(rp) |= 1u << (p);		\
	} while (0)

#define portRESET_READY_PRIORITY(p, rp)		\
	do {					\
		(rp) &= ~(1u << (p));		\
	} while (0)

#define	portGET_HIGHEST_PRIORITY(p, rp)		\
	do {					\
		(p) = 31u - __CLZ(rp);		\
	} while (0)
#else
#define	configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#endif	/* configMAX_PRIORITIES <= 32) */

#if defined(configASSERT) && (configASSERT != 0)
extern void vPortValidateInterruptPriority(void);
#define	portASSERT_IF_INTERRUPT_PRIORITY_INVALID()	\
		vPortValidateInterruptPriority()
#endif

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
