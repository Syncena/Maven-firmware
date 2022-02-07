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

/* == Zone debugging options == */
#if (RELEASE_BUILD == 0)
#define	ZONE_OPT_STATS		1
#endif

/* == Network Options */
#define	CONFIG_NETWORK_NSOCKETS		4

/* == RTOS OPTIONS == */
#if (RELEASE_BUILD == 0)
#define RTOS_OPTION_SHELL		1
#define RTOS_OPTION_IRQ_STATS   	1
#endif
#define RTOS_OPTION_SOFT_IRQ    	1


/* == UART OPTIONS == */
#define	UART_NUARTS			3	/* Console/Target/SWO */
#define	UART_OPT_NO_FLOW
#define	UART_OPT_FIXED_DATABITS		1
#define	UART_OPT_FIXED_STOPBITS		1
#if (RELEASE_BUILD != 0)
#define	UART_OPT_COMMAND_SINGLE_UART	1
#endif


/* == Target console == */
/* Override default in an attempt to reduce RAM usage. */
#define	TARGET_CONSOLE_RINGBUFF_SIZE_BOUNCE	2048


/* == Debug == */
#define	CONFIG_GDB_SERVERS		1
#define	CONFIG_MAX_TARGETS		1
#define	ADIV5_SPEED_2CFG		64,32,13,9,6,3,2,1,0
#define	ADIV5_SPEED_2STR		"250 KHz", "500 KHz", "1.0 MHz", \
					"1.5 MHz", "2.0 MHz", "3.0 MHz", \
					"3.5 MHz", "4.5 MHz", "6.5 MHz"

#endif /* BUILD_OPTIONS_H */
