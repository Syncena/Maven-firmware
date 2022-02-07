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

#define	F_CPU			50000000u
#define	CONFIG_USE_CONFIGDB	1

/* == Zone debugging options == */
#if (RELEASE_BUILD == 0)
#define	ZONE_OPT_STATS		1
#define	ZONE_OPT_BOOKENDS	1
#endif

/* == PLATFORM OPTIONS == */
#define	PLATFORM_RTOS_ZONE_LEN		(16u*1024u)
#if (RELEASE_BUILD == 0)
#define	CONSOLE_TX_RING_SIZE		(128u*1024)
#else
#define	CONSOLE_TX_RING_SIZE		4096u
#endif
#define	CONSOLE_FLUSH_IN_IDLE_TASK	1


/* == RTOS OPTIONS == */
#if (RELEASE_BUILD == 0)
#define RTOS_OPTION_SHELL		1
#define RTOS_OPTION_IRQ_STATS   	1
#endif
#define RTOS_OPTION_SOFT_IRQ    	1


/* == UART OPTIONS == */
#define	UART_NUARTS			3	/* Console/Target/SWO */


/* == Debug == */
#define	CONFIG_GDB_SERVERS		1
#define	ADIV5_SPEED_2CFG		77,39,17,11,7,4,3,2,0
#define	ADIV5_SPEED_2STR		"300 KHz", "600 KHz", "1.3 MHz", \
					"2.0 MHz", "3.0 MHz", "4.5 MHz", \
					"5.5 MHz", "7.0 MHz", "13.5 MHz"
#define	ADIV5_SPEED_DEFAULT		3

#endif /* BUILD_OPTIONS_H */
