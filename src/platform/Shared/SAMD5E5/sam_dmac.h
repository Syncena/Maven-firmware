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

#ifndef SAM_DMAC_H
#define SAM_DMAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "platform.h"

/*
 * NOTE ...
 *
 * The DMAC is screwy. Source and destination pointers must be initialised
 * to point to the last beat transfer address of the block. For regular byte-
 * sized transfers, this actually implies 'dd_src/dd_dst + dd_cnt'.
 */
typedef struct sam_dmac_desc {
	volatile uint16_t dd_ctrl;
	volatile uint16_t dd_cnt;
	volatile const void *dd_src;
	volatile void *dd_dst;
	volatile struct sam_dmac_desc *dd_next;
} *sam_dmac_desc_t;

#define	SAM_DMAC_CTRL_STEPSIZE(x)	(((x) & 7u) << 13)
#define	SAM_DMAC_CTRL_STEPSEL_DST	(0u << 12)
#define	SAM_DMAC_CTRL_STEPSEL_SRC	(1u << 12)
#define	SAM_DMAC_CTRL_DSTINC		(1u << 11)
#define	SAM_DMAC_CTRL_SRCINC		(1u << 10)
#define	SAM_DMAC_CTRL_BEATSIZE(x)	(((x) & 3u) << 8)
#define	 SAM_DMAC_BEAT_8BITS		0u
#define	 SAM_DMAC_BEAT_16BITS		1u
#define	 SAM_DMAC_BEAT_32BITS		2u
#define	SAM_DMAC_CTRL_BLOCKACT(x)	(((x) & 3u) << 3)
#define	 SAM_DMAC_BLOCKACT_NOACT	0u
#define	 SAM_DMAC_BLOCKACT_INT		1u
#define	 SAM_DMAC_BLOCKACT_SUSPEND	2u
#define	 SAM_DMAC_BLOCKACT_BOTH		3u
#define	SAM_DMAC_CTRL_EVOSEL(x)	(((x) & 3u) << 1)
#define	 SAM_DMAC_EVOSEL_DISABLE	0u
#define	 SAM_DMAC_EVOSEL_BLOCK		1u
#define	 SAM_DMAC_EVOSEL_BEAT		3u
#define	SAM_DMAC_CTRL_VALID		(1u << 0)


typedef struct sam_dmac_config {
	uint8_t cfg_threshold;
	uint8_t cfg_burstlen;
	uint8_t cfg_trigact;
	uint8_t cfg_trigsrc;
	uint8_t cfg_priority;
	uint8_t cfg_evomode;
	uint8_t cfg_evact;
} sam_dmac_config_t;

#define	SAM_DMAC_THRESHOLD_1BEAT	0u
#define	SAM_DMAC_THRESHOLD_2BEATS	1u
#define	SAM_DMAC_THRESHOLD_4BEATS	2u
#define	SAM_DMAC_THRESHOLD_8BEATS	3u

#define	SAM_DMAC_BURSTLEN(x)		(((x) - 1u) & 0xfu)

#define	SAM_DMAC_TRIGACT_BLOCK		0x00u
#define	SAM_DMAC_TRIGACT_BURST		0x02u
#define	SAM_DMAC_TRIGACT_TRANSACTION	0x03u

#define	SAM_DMAC_TRIGSRC_DISABLE	0x00u
#define	SAM_DMAC_TRIGSRC_RTC_TIMESTAMP	0x01u
#define	SAM_DMAC_TRIGSRC_DSU(c)		(0x02u + ((c) & 1u))
#define	SAM_DMAC_TRIGSRC_SERCOM_RX(i)	(0x04u + (((i) & 7u) * 2u))
#define	SAM_DMAC_TRIGSRC_SERCOM_TX(i)	(0x05u + (((i) & 7u) * 2u))
#define	SAM_DMAC_TRIGSRC_CAN(i)		(0x14u + ((c) & 1u))
/* Can't be bothered figuring out the TCC/TC sources at the moment. */
#define	SAM_DMAC_TRIGSRC_ADC_RESRDY(i)	(0x44u + (((i) & 1u) * 2u))
#define	SAM_DMAC_TRIGSRC_ADC_SEQ(i)	(0x45u + (((i) & 1u) * 2u))
#define	SAM_DMAC_TRIGSRC_PCC		0x50u
#define	SAM_DMAC_TRIGSRC_AES_WR		0x51u
#define	SAM_DMAC_TRIGSRC_AES_RD		0x52u
#define	SAM_DMAC_TRIGSRC_QSPI_RX	0x53u
#define	SAM_DMAC_TRIGSRC_QSPI_TX	0x54u

#define	SAM_DMAC_PRIORITY_LOWEST	0x0u
#define	SAM_DMAC_PRIORITY_LOW		0x1u
#define	SAM_DMAC_PRIORITY_HIGH		0x2u
#define	SAM_DMAC_PRIORITY_HIGHEST	0x3u

#define	SAM_DMAC_EVOMODE_NONE		0x0u
#define	SAM_DMAC_EVOMODE_DEFAULT	0x1u
#define	SAM_DMAC_EVOMODE_TRIGACT	0x2u

#define	SAM_DMAC_EVACT_NOACT		0x0u
#define	SAM_DMAC_EVACT_TRIG		0x1u
#define	SAM_DMAC_EVACT_CTRIG		0x2u
#define	SAM_DMAC_EVACT_CBLOCK		0x3u
#define	SAM_DMAC_EVACT_SUSPEND		0x4u
#define	SAM_DMAC_EVACT_RESUME		0x5u
#define	SAM_DMAC_EVACT_SSKIP		0x6u
#define	SAM_DMAC_EVACT_INCPRI		0x7u

typedef enum {
	SAM_DMAC_STATUS_BUSY,
	SAM_DMAC_STATUS_COMPLETE,
	SAM_DMAC_STATUS_SUSPENDED,
	SAM_DMAC_STATUS_ERROR,
	SAM_DMAC_STATUS_TIMEOUT,
	SAM_DMAC_STATUS_SPURIOUS
} sam_dmac_status_t;

extern void *sam_dmac_channel_alloc(const sam_dmac_config_t *,
				    sam_dmac_desc_t *);
extern void sam_dmac_channel_start(void *, bool irq_mode);
extern void sam_dmac_channel_stop(void *);
extern sam_dmac_status_t sam_dmac_channel_status(void *);

extern void *sam_dmac_irq_register(void *, const char *, uint32_t,
			void (*)(void *, sam_dmac_status_t), void *);
extern void sam_dmac_irq_unregister(void *);

extern void sam_dmac_attach(void);

#endif /* SAM_DMAC_H */
