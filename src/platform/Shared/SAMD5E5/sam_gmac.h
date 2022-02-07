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

#ifndef SAM_GMAC_H
#define SAM_GMAC_H

typedef enum {
	SAM_GMAC_MODE_MII,
	SAM_GMAC_MODE_RMII
} sam_gmac_hw_mode_t;

typedef struct {
	const char *ga_hostname;
	Gmac *ga_regs;
	uint32_t ga_clock;
	void *ga_desc_memory;
	uint16_t ga_tx_descs;
	uint16_t ga_rx_descs;
	sam_gmac_hw_mode_t ga_mode;
	uint8_t ga_phy_clause45;
	uint16_t ga_phy_num;
	uint8_t ga_mac[6];
} sam_gmac_attach_args_t;

extern void *sam_gmac_attach(const sam_gmac_attach_args_t *);
extern void sam_gmac_hw_interrupt(void *);

#define	SAM_GMAC_DESC_MEM_SIZE(tx,rx)	(((tx)+(rx)) * 8u)

#endif /* SAM_GMAC_H */
