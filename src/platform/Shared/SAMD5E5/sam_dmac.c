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

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "rtos.h"
#include "platform.h"
#include "sam_clocks.h"
#include "sam_dmac.h"
#include "sam_cmcc.h"

/*
 * The DMA controller supports 'DMAC_CH_NUM' channels, normally 32.
 * This is way more than we need, so we'll constrain ourselves to a
 * small subset to reduce our SRAM footprint.
 */
#define	DMAC_CHANNELS	4
_Static_assert(DMAC_CHANNELS <= DMAC_CH_NUM, "Invalid DMAC_CHANNELS");

/*
 * There are five DMA interrupt lines. The first four correspond to the
 * first four DMA channels. The remaining line is shared between all
 * other channels.
 */
#define	DMAC_NIRQ	5

/*
 * Per-channel primary DMA descriptors.
 *
 * The data sheet says the initial and write-back descriptor regions
 * can be overlayed. That is, they can share the same memory.
 *
 * The data sheet is wrong.
 */
static __attribute__((aligned(DMAC_CHANNELS * sizeof(struct sam_dmac_desc))))
	struct sam_dmac_desc sam_dmac_descs[DMAC_CHANNELS];
static __attribute__((aligned(DMAC_CHANNELS * sizeof(struct sam_dmac_desc))))
	struct sam_dmac_desc sam_dmac_descs_wb[DMAC_CHANNELS];

struct sam_dmac_state;

/* Per-channel state. */
struct sam_dmac_channel {
	DmacChannel *dc_regs;
	struct sam_dmac_state *dc_state;
	unsigned int dc_chan;
	void (*dc_callback)(void *, sam_dmac_status_t);
	void *dc_arg;
};

/* Overall state. */
struct sam_dmac_state {
	void *ds_irq[DMAC_NIRQ];
	unsigned int ds_irq_shared_refs;
	uint32_t ds_irq_shared_ipl;
	struct sam_dmac_channel ds_channels[DMAC_CHANNELS];
};
static struct sam_dmac_state sam_dmac_state;

static __inline sam_dmac_status_t
sam_dmac_flag2status(uint8_t flag)
{

	if (flag & DMAC_CHINTFLAG_TERR)
		return SAM_DMAC_STATUS_ERROR;
	else
	if (flag & DMAC_CHINTFLAG_TCMPL)
		return SAM_DMAC_STATUS_COMPLETE;
	else
	if (flag & DMAC_CHINTFLAG_SUSP)
		return SAM_DMAC_STATUS_SUSPENDED;

	return SAM_DMAC_STATUS_SPURIOUS;
}

static void
sam_dmac_chan_interrupt(void *arg)
{
	struct sam_dmac_channel *dc = arg;
	DmacChannel *r = dc->dc_regs;
	uint8_t flag;

	assert(dc->dc_callback != NULL);

	flag = r->CHINTFLAG.reg;
	r->CHINTFLAG.reg = flag;
	r->CHINTENCLR.reg = DMAC_CHINTENCLR_MASK;

	(dc->dc_callback)(dc->dc_arg, sam_dmac_flag2status(flag));
}

static void
sam_dmac_shared_interrupt(void *arg)
{
	struct sam_dmac_state *ds = arg;
	unsigned int chan;
	uint32_t reg;

	while (((reg = DMAC->INTPEND.reg) & DMAC_INTPEND_PEND) != 0) {
		chan = (reg & DMAC_INTPEND_ID_Msk) >> DMAC_INTPEND_ID_Pos;
		sam_dmac_chan_interrupt(&ds->ds_channels[chan]);
	}
}

void *
sam_dmac_channel_alloc(const sam_dmac_config_t *cfg,
    sam_dmac_desc_t *ddp)
{
	struct sam_dmac_state *ds = &sam_dmac_state;
	struct sam_dmac_channel *dc;
	unsigned int chan;
	uint8_t chevctrl;
	DmacChannel *r;

	for (dc = NULL, chan = 0; chan < DMAC_CHANNELS; chan++) {
		if (ds->ds_channels[chan].dc_regs == NULL) {
			dc = &ds->ds_channels[chan];
			break;
		}
	}

	if (dc == NULL)
		return NULL;

	*ddp = &sam_dmac_descs[chan];
	memset(*ddp, 0, sizeof(sam_dmac_descs[chan]));
	r = dc->dc_regs = &DMAC->Channel[chan];
	dc->dc_chan = chan;
	dc->dc_state = ds;
	dc->dc_callback = NULL;
	dc->dc_arg = NULL;

	r->CHCTRLA.reg =
	    DMAC_CHCTRLA_THRESHOLD(cfg->cfg_threshold) |
	    DMAC_CHCTRLA_BURSTLEN(cfg->cfg_burstlen) |
	    DMAC_CHCTRLA_TRIGACT(cfg->cfg_trigact) |
	    DMAC_CHCTRLA_TRIGSRC(cfg->cfg_trigsrc);
	r->CHPRILVL.reg = DMAC_CHPRILVL_PRILVL(cfg->cfg_priority);

	chevctrl = 0;
	if (cfg->cfg_evomode != SAM_DMAC_EVOMODE_NONE) {
		chevctrl |= DMAC_CHEVCTRL_EVOMODE(cfg->cfg_evomode - 1);
		chevctrl |= DMAC_CHEVCTRL_EVOE;
	}
	if (cfg->cfg_evact != SAM_DMAC_EVACT_NOACT) {
		chevctrl |= DMAC_CHEVCTRL_EVACT(cfg->cfg_evact);
		chevctrl |= DMAC_CHEVCTRL_EVIE;
	}
	r->CHEVCTRL.reg = chevctrl;

	return dc;
}

void
sam_dmac_channel_start(void *cookie, bool irq_mode)
{
	struct sam_dmac_channel *dc = cookie;
	DmacChannel *r = dc->dc_regs;
	uint8_t flag;

	flag = r->CHINTFLAG.reg;
	r->CHINTFLAG.reg = flag;

	if (irq_mode)
		r->CHINTENSET.reg = DMAC_CHINTENSET_MASK;
	else
		r->CHINTENCLR.reg = DMAC_CHINTENCLR_MASK;

	__DSB();
	__DMB();
	__ISB();

	r->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void
sam_dmac_channel_stop(void *cookie)
{
	struct sam_dmac_channel *dc = cookie;
	DmacChannel *r = dc->dc_regs;
	uint8_t flag;

	r->CHINTENCLR.reg = DMAC_CHINTENCLR_MASK;

	r->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

	while (r->CHCTRLA.reg & DMAC_CHCTRLA_ENABLE)
		;

	flag = r->CHINTFLAG.reg;
	r->CHINTFLAG.reg = flag;
}

sam_dmac_status_t
sam_dmac_channel_status(void *cookie)
{
	struct sam_dmac_channel *dc = cookie;
	DmacChannel *r = dc->dc_regs;
	uint8_t flag;

	flag = r->CHINTFLAG.reg;

	if ((flag & DMAC_CHINTFLAG_MASK) == 0)
		return SAM_DMAC_STATUS_BUSY;

	return sam_dmac_flag2status(flag);
}

void *
sam_dmac_irq_register(void *cookie, const char *name, uint32_t ipl,
    void (*callback)(void *, sam_dmac_status_t), void *arg)
{
	struct sam_dmac_channel *dc = cookie;
	struct sam_dmac_state *ds = dc->dc_state;

#ifdef NDEBUG
	(void) name;
#endif

	dc->dc_callback = callback;
	dc->dc_arg = arg;

	if (dc->dc_chan < (DMAC_NIRQ - 1)) {
		assert(ds->ds_irq[dc->dc_chan] == NULL);

		ds->ds_irq[dc->dc_chan] = rtos_irq_register_named(name,
		    DMAC_0_IRQn + dc->dc_chan, ipl,
		    sam_dmac_chan_interrupt, dc);
	} else {
		if (ds->ds_irq[DMAC_NIRQ - 1] == NULL) {
			ds->ds_irq_shared_ipl = ipl;
			ds->ds_irq_shared_refs = 0;
			ds->ds_irq[DMAC_NIRQ - 1] =
			    rtos_irq_register_named("DMAC-Shared", DMAC_4_IRQn,
			    ipl, sam_dmac_shared_interrupt, ds);
		} else {
			assert(ipl == ds->ds_irq_shared_ipl);
			ds->ds_irq_shared_refs++;
		}
	}

	return dc;
}

void
sam_dmac_irq_unregister(void *cookie)
{
	struct sam_dmac_channel *dc = cookie;
	struct sam_dmac_state *ds = dc->dc_state;
	DmacChannel *r = dc->dc_regs;

	r->CHINTENCLR.reg = DMAC_CHINTENCLR_MASK;
	r->CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;

	dc->dc_callback = NULL;
	dc->dc_arg = NULL;

	if (dc->dc_chan <= 3) {
		assert(ds->ds_irq[dc->dc_chan] != NULL);

		rtos_irq_unregister(ds->ds_irq[dc->dc_chan]);

		ds->ds_irq[dc->dc_chan] = NULL;
	} else {
		assert(ds->ds_irq_shared_refs > 0);

		if (--(ds->ds_irq_shared_refs) == 0) {
			assert(ds->ds_irq[DMAC_NIRQ - 1] != NULL);

			rtos_irq_unregister(ds->ds_irq[DMAC_NIRQ - 1]);

			ds->ds_irq[DMAC_NIRQ - 1] = NULL;
		}
	}
}

void
sam_dmac_attach(void)
{
	Dmac *r = DMAC;

	sam_periph_clock_enable(ID_DMAC);

	if (r->CTRL.bit.DMAENABLE != 0) {
		r->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
		while ((r->CTRL.reg & DMAC_CTRL_DMAENABLE) != 0)
			;
	}

	r->CTRL.reg = DMAC_CTRL_SWRST;
	while ((r->CTRL.reg & DMAC_CTRL_SWRST) != 0);
		;

	r->DBGCTRL.reg = DMAC_DBGCTRL_DBGRUN;

	r->CTRL.reg = DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 |
	    DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3;
	r->PRICTRL0.reg =
	    DMAC_PRICTRL0_RRLVLEN0 | DMAC_PRICTRL0_QOS0_SENSITIVE |
	    DMAC_PRICTRL0_RRLVLEN1 | DMAC_PRICTRL0_QOS1_SENSITIVE |
	    DMAC_PRICTRL0_RRLVLEN2 | DMAC_PRICTRL0_QOS2_SENSITIVE |
	    DMAC_PRICTRL0_RRLVLEN3 | DMAC_PRICTRL0_QOS3_SENSITIVE;

	r->BASEADDR.reg = (uint32_t)(uintptr_t)sam_dmac_descs;
	(void) r->BASEADDR.reg;
	r->WRBADDR.reg = (uint32_t)(uintptr_t)sam_dmac_descs_wb;
	(void) r->WRBADDR.reg;
	__DMB();
	__DSB();
	__ISB();

	r->CTRL.reg |= DMAC_CTRL_DMAENABLE;
	while ((r->CTRL.reg & DMAC_CTRL_DMAENABLE) == 0)
		;
}
