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

#ifndef CF_REGISTRY_H
#define CF_REGISTRY_H

/* Uint8 */
#define	CF_KEY_UINT8_DISPLAY_FLAGS		0x0000u
#define CF_KEY_UINT8_ADIV5_IDLES		0x0010u
#define CF_KEY_UINT8_ADIV5_TURNAROUND		0x0011u
#define CF_KEY_UINT8_ADIV5_SPEED		0x0012u
#define CF_KEY_UINT8_ADIV5_INTERFACE		0x0013u
#define	CF_KEY_UINT8_HW_CONFIG			0x0020u
#define	CF_KEY_UINT8_SEMIHOSTING_ENABLE		0x0030u
#define	CF_KEY_UINT8_ATTACH_RESET		0x0040u
#define	CF_KEY_UINT8_SCAN_TYPE			0x0041u
#define	CF_KEY_UINT8_ATTACH_AUTO		0x0042u
#define	CF_KEY_UINT8_RTOS_TYPE			0x0043u
#define	CF_KEY_UINT8_RTOS_INIT_SYMBOLS		0x0044u
#define	CF_KEY_UINT8_FLASH_VERIFY		0x0045u
#define	CF_KEY_UINT8_USB_PERSONALITY		0x0050u
#define	CF_KEY_UINT8_WIFI_ENC_TYPE		0x1000u
#define	CF_KEY_UINT8_WIFI_PROV_CHANNEL		0x1001u
#define	CF_KEY_UINT8_WILC_TX_BOGONS		0x1010u
#define	CF_KEY_UINT8_PROGRAMMER_PROTO		0x2000u

/* Uint16 */
#define	CF_KEY_UINT16_USB_VID			0x0050u
#define	CF_KEY_UINT16_USB_PID			0x0051u

/* Uint32 */
#define	CF_KEY_UINT32_NETWORK_IP		0x0000u
#define	CF_KEY_UINT32_NETWORK_MASK		0x0001u
#define	CF_KEY_UINT32_NETWORK_GATE		0x0002u
#define	CF_KEY_UINT32_NETWORK_SNTP_IP		0x0003u
#define	CF_KEY_UINT32_WIFI_PROV_IP		0x1000u
#define	CF_KEY_UINT32_CORTEXM_VC		0x2000u
#define	CF_KEY_UINT32_CORTEXM_FLAGS		0x2001u
#define	CF_KEY_UINT32_ISP_CLOCK			0x3000u
#define	CF_KEY_UINT32_PDI_CLOCK			0x3001u
#define	CF_KEY_UINT32_UPDI_BAUD			0x3002u
#define	CF_KEY_UINT32_TARGET_ARCH_LICENCE	0x4000u

/* Strings. */
#define	CF_KEY_STRING_TIMEZONE			0x0000u
#define	CF_KEY_STRING_DISPLAY_DRIVER		0x0010u
#define	CF_KEY_STRING_WIFI_NAME			0x1000u
#define	CF_KEY_STRING_WIFI_SSID			0x1001u
#define	CF_KEY_STRING_WIFI_KEY			0x1002u
#define	CF_KEY_STRING_WIFI_PROV_SSID		0x1004u
#define	CF_KEY_STRING_WIFI_PROV_PASS		0x1005u
#define	CF_KEY_STRING_PROGRAMMER_DEV_ID		0x2000u

/* Data. */
#define	CF_KEY_DATA_TARGET_COMMS_CONFIG(n)	(0x0000u + ((n) & 0x0fu))
#define	CF_KEY_DATA_RTOS_PARAMS(n)		(0x0100u + ((n) & 0xffu))

#endif /* CF_REGISTRY_H */
