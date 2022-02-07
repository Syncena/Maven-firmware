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

#ifndef AVR_LL_H
#define AVR_LL_H

#include "hardware.h"

extern void avr_ll_init_target(void);

extern void avr_ll_isp_start(void);
extern void avr_ll_isp_stop(void);
extern void avr_ll_isp_clk_start(uint32_t);
extern void avr_ll_isp_clk_stop(void);
extern void avr_ll_isp_rst_high(void);
extern void avr_ll_isp_rst_low(void);
extern void avr_ll_isp_sck_high(void);
extern void avr_ll_isp_sck_low(void);
extern void avr_ll_isp_mosi_high(void);
extern void avr_ll_isp_mosi_low(void);
extern uint8_t avr_ll_isp_xfer(uint8_t data, uint32_t clk_delay);

extern void avr_ll_pdi_enable(uint32_t);
extern uint8_t avr_ll_updi_enable(uint32_t, int);
#define	AVR_LL_UPDI_RESET_SUCCESS	0
#define	AVR_LL_UPDI_RESET_NOT_HIGH	1
#define	AVR_LL_UPDI_RESET_STILL_HIGH	2
#define	AVR_LL_UPDI_RESET_LOW_TOO_LONG	3
extern uint8_t avr_ll_updi_change_baud(uint32_t);
extern void avr_ll_pdi_disable(void);
extern uint8_t avr_ll_pdi_transfer(const uint8_t *txp, uint32_t txlen,
			 uint8_t *rxp, uint32_t rxlen);
extern uint8_t avr_ll_pdi_rx_status(void);
#define	AVR_LL_PDI_RX_DONE	0
#define	AVR_LL_PDI_RX_BUSY	1
#define	AVR_LL_PDI_RX_IDLE	2
#define	AVR_LL_PDI_RX_CORRUPT	3
extern void avr_ll_pdi_rx_purge(void);
extern void avr_ll_pdi_send_break(uint8_t, int);

HW_PIN(ISP_RST, EXT5)
HW_PIN(ISP_SCK, EXT3)
HW_PIN(ISP_MISO, EXT1)
HW_PIN(ISP_MOSI, EXT4)
#define	HW_PIN_ISP_RST_BUFFER_OUT	HW_PIN_ISP_RST_DIR_set
#define	HW_PIN_ISP_RST_BUFFER_IN	HW_PIN_ISP_RST_DIR_clr
#define	HW_PIN_ISP_SCK_BUFFER_OUT	HW_PIN_ISP_SCK_DIR_set
#define	HW_PIN_ISP_SCK_BUFFER_IN	HW_PIN_ISP_SCK_DIR_clr
#define	HW_PIN_ISP_MISO_BUFFER_OUT	HW_PIN_ISP_MISO_DIR_set
#define	HW_PIN_ISP_MISO_BUFFER_IN	HW_PIN_ISP_MISO_DIR_clr
#define	HW_PIN_ISP_MOSI_BUFFER_OUT	HW_PIN_ISP_MOSI_DIR_set
#define	HW_PIN_ISP_MOSI_BUFFER_IN	HW_PIN_ISP_MOSI_DIR_clr

HW_PIN(PDI_SCK, EXT5)
HW_PIN(PDI_RXD, EXT1)
HW_PIN(PDI_TXD, EXT4)
#define	HW_PIN_PDI_SCK_BUFFER_OUT	HW_PIN_PDI_SCK_DIR_set
#define	HW_PIN_PDI_SCK_BUFFER_IN	HW_PIN_PDI_SCK_DIR_clr
#define	HW_PIN_PDI_RXD_BUFFER_OUT	HW_PIN_PDI_RXD_DIR_set
#define	HW_PIN_PDI_RXD_BUFFER_IN	HW_PIN_PDI_RXD_DIR_clr
#define	HW_PIN_PDI_TXD_BUFFER_OUT	HW_PIN_PDI_TXD_DIR_set
#define	HW_PIN_PDI_TXD_BUFFER_IN	HW_PIN_PDI_TXD_DIR_clr
#define	HW_PIN_PDI_BRIDGE_ACTIVATE	HAL_GPIO_BRIDGE_set
#define	HW_PIN_PDI_BRIDGE_DEACTIVATE	HAL_GPIO_BRIDGE_clr

static __inline void
avr_ll_pdi_tx_enable(void)
{

	HW_PIN_PDI_BRIDGE_ACTIVATE();
	HW_PIN_PDI_RXD_BUFFER_OUT();
}

static __inline void
avr_ll_pdi_tx_disable(void)
{

	HW_PIN_PDI_RXD_BUFFER_IN();
	HW_PIN_PDI_BRIDGE_DEACTIVATE();
}

#endif /* AVR_LL_H */
