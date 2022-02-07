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

#ifndef BUILD_OPTIONS_H
#define BUILD_OPTIONS_H

#define	F_CPU			120000000u
#define	FW_ID			0x20180001

/* == Zone debugging options == */
#if (RELEASE_BUILD == 0)
#define	ZONE_OPT_STATS		1
#define	ZONE_OPT_BOOKENDS	1
#endif

/* == PLATFORM OPTIONS == */
#define	PLATFORM_HAS_CPU_DELAY		1
#define	PLATFORM_RTOS_ZONE_LEN		(16u*1024u)

/* == Network Options */
#define	CONFIG_NETWORK_NSOCKETS		4

/* == RTOS OPTIONS == */
#if (RELEASE_BUILD == 0)
#define RTOS_OPTION_SHELL		1
#define RTOS_OPTION_IRQ_STATS   	1
#endif
#define RTOS_OPTION_SOFT_IRQ    	1


/* == UART OPTIONS == */
#define	UART_NUARTS			2	/* Console/Target */
#define	UART_OPT_NO_FLOW
#define	UART_OPT_FIXED_DATABITS		1
#define	UART_OPT_FIXED_STOPBITS		1
#if (RELEASE_BUILD != 0)
#define	UART_OPT_COMMAND_SINGLE_UART	1
#endif

#endif /* BUILD_OPTIONS_H */
