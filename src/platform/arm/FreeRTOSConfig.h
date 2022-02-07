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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "platform.h"
#include "FreeRTOS-common.h"

#if configDEBUG_OPTIONS
#if (defined (__ARM_ARCH_8M_MAIN__ ) && (__ARM_ARCH_8M_MAIN__ == 1))
#define	configCHECK_FOR_STACK_OVERFLOW		0
#else
#define	configCHECK_FOR_STACK_OVERFLOW		2
#endif
#else
#define	configCHECK_FOR_STACK_OVERFLOW		0
//#define	configQUIET_FATAL_EXCEPTION		1
#endif

#define	configUSE_PORT_OPTIMISED_TASK_SELECTION	1
#define configCPU_CLOCK_HZ			F_CPU
#define configTICK_RATE_HZ			1000
#define configMAX_PRIORITIES			8
#define configMINIMAL_STACK_SIZE		40u
#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE			1500u
#endif /* configTOTAL_HEAP_SIZE */

/*
 * Interrupt priorities.
 */
#define	HW_IPL_MAX		1
#define	HW_IPL_USART_TARGET	2
#define	HW_IPL_SCHEDULER	3
#define	HW_IPL_USART_CONSOLE	3
#define	HW_IPL_USB		4
#define	HW_IPL_NETWORK		5
#define	HW_IPL_TIMER		6
#define	HW_IPL_ADC		7
#define	HW_IPL_BUTTON		7
#define	HW_IPL_DISPLAY		7
#define	HW_IPL_I2C		7
#define	HW_IPL_SOFTIRQ		7

#endif /* FREERTOS_CONFIG_H */
