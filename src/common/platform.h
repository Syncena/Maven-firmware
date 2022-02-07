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

#ifndef PLATFORM_H
#define PLATFORM_H

#include "serial_number.h"
#include "platform_soc.h"
#include "platform_cpu.h"
#include "jtag_if.h"

/*
 * Declare a decorator which can be used to place a function in SRAM.
 */
#ifdef RAMFUNC
#undef RAMFUNC	/* Some platforms have this defined in a system header... */
#endif
#define	RAMFUNC	__attribute__ ((__noinline__,section(".ramfunc")))

/*
 * Some platforms have tightly-coupled SRAM in the Cortex-M code region
 * of the address space. These will define FAST_RAMFUNC as necessary in
 * order to locate key functions there. Otherwise we declare it empty
 * here so those functions are in regular Flash.
 */
#ifndef FAST_RAMFUNC
#define	FAST_RAMFUNC	/* Nothing */
#endif

extern void platform_early_init(void);
extern void platform_init(void);
extern void platform_watchdog_disable(void);
extern void platform_attach_devices(void);
extern void platform_reboot(void);
extern void platform_fabricate_serial(serial_number_t *);
extern int platform_flash_program(uint32_t, const void *, uint32_t);
extern void platform_wifi_led_init(void);
extern void platform_wifi_led_set(uint8_t);
extern void platform_usb_init(void);
extern void platform_usb_load_defaults(void);

typedef enum {
	PLATFORM_UART_CONSOLE,
	PLATFORM_UART_TARGET,
	PLATFORM_UART_SWO,
	PLATFORM_UART_WILC,
} platform_uart_t;
extern int platform_uart_attach(platform_uart_t);
extern void platform_uart_target_enable(void);
extern void platform_uart_target_disable(void);

#ifndef CONFIG_USE_CONFIGDB
extern void platform_wipe_globals(void);
#else
extern void platform_attach_configdb(void);
#endif

#ifndef CONFIG_HARDWARE_JTAG
#include "jtag_bitbang.h"
#define	platform_jtag_init	jtag_bitbang_init
#else
extern jtag_state_t platform_jtag_init(jtag_if_t *);
#endif

extern void platform_hw_init_target(void);
extern void platform_hw_isp_clk_start(uint32_t);
extern void platform_hw_isp_clk_stop(void);
extern void platform_hw_pdi_enable(uint32_t pdi_clk);
extern void platform_hw_pdi_rx_purge(void);
extern void platform_hw_pdi_send_break(int how_many, int change_baud);
extern void platform_hw_pdi_disable(void);
extern uint8_t platform_hw_pdi_transfer(const uint8_t *txp, uint32_t txlen,
					uint8_t *rxp, uint32_t rxlen);
extern uint8_t platform_hw_pdi_rx_status(void);

extern uint8_t platform_hw_updi_enable(uint32_t baud);
extern void platform_hw_updi_function_enable(void);
extern uint8_t platform_hw_updi_change_baud(uint32_t baud);

#endif /* PLATFORM_H */
