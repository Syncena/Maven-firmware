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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "target.h"
#include "tmon.h"
#include "cortexm.h"
#include "microchip_dsu.h"
#include "nxp_debug_mbox.h"
#include "adiv5.h"
#include "adiv5_ll.h"
#include "semihosting.h"
#include "swo_serve.h"

/*
 * Supported targets are looked-up using the following...
 */
struct cortexm_target_id {
	uint64_t ti_pidr;
	uint64_t ti_pidr_mask;
	int (*ti_attach)(target_t, target_addr_t);
	const char *ti_desc;
};

/*
 * The list of supported target CPU cores, matched by the ADIv5 PIDR code
 * of their ROM Table components.
 */
static const struct cortexm_target_id cortexm_target_ids[] = {
	/* DSU-equipped Microchip devices. Must match before the core. */
	{0x00009fcd0u,	0xf000fffffu,	microchip_dsu_attach, "Microchip DSU"},

	{0x4000bb471u,	0xf000fffffu,	cortexm_attach, "M0"},
	{0x4000bb4c0u,	0xf000fffffu,	cortexm_attach, "M0+"},
	{0x4000bb4c3u,	0xf000fffffu,	cortexm_attach, "M3"},
	{0x4000bb4c4u,	0xf000fffffu,	cortexm_attach, "M4"},
	{0x4000bb4c7u,	0xf000fffffu,	cortexm_attach, "M7"},
	{0x4000bb4c9u,	0xf000fffffu,	cortexm_attach, "M33"},
	{0x4000bb4cbu,	0xf000fffffu,	cortexm_attach, "M23"},

	/*
	 * Failed to match anything if we get here. Cast a wider net this time
	 * to see if there's any ROM table entry with the standard CoreSight
	 * address of 0xe00ff000. Yes, this is a bit of a hack, but it should
	 * catch SoCs from vendors who play fast and loose with the standard.
	 * Yes, I'm looking at you, ST Micro...
	 */
	{0x000000000u,  0x000000000u,   cortexm_attach, "M? (Wildcard)"},
};
#define	CORTEXM_TARGET_NIDS	(sizeof(cortexm_target_ids) / \
				 sizeof(cortexm_target_ids[0]))
#define	CORESIGHT_ROM_ADDRESS	((target_addr_t)0xe00ff000ul)

/*
 * Supported "Other-APs" are looked-up using the following...
 */
struct cortexm_other_ap {
	uint32_t oa_idr;
	uint32_t oa_idr_mask;
	int (*oa_match)(const struct adiv5_ap_interface *, void *);
	const char *oa_desc;
};

/*
 * The list of supported "Other-AP" devices, matched by their IDR.
 */
static const struct cortexm_other_ap cortexm_other_aps[] = {
	{NXP_LPC55S6X_IDR, NXP_LPC55S6X_IDR_MASK,
	 nxp_debug_mailbox_probe, "LPC55S6x Debugger Mailbox AP"},
};
#define	CORTEXM_OTHER_APS	(sizeof(cortexm_other_aps) / \
				 sizeof(cortexm_other_aps[0]))

uint32_t cortexm_hw_cfg_extra_set, cortexm_hw_cfg_extra_clr;

static bool
target_cortexm_probe(void *arg)
{

	(void)arg;

	/* Go looking for ADIv5 devices */
	if (adiv5_probe() < 0) {
		/* Try again with an extended reset sequence. */
		tmon_log("Nothing found. Retry using Extended Reset "
		    "Sequence.\n");
		adiv5_ll_cfg_extra_set |= ADIV5_LL_NRST_EXTENDED;
		if (adiv5_probe() < 0)
			return false;
		tmon_log("Extended Reset Sequence succeeded.\n");
		adiv5_ll_cfg_extra_set &= ~ADIV5_LL_NRST_EXTENDED;
	}

	return true;
}

static int
mem_ap_match_and_attach(target_link_ops_t lo, void *arg)
{
	struct adiv5_memap_rom_table_lookup rt;
	const struct cortexm_target_id *ti;
	target_t t;
	int rv;

	if ((t = target_alloc(lo, arg)) == NULL)
		return TARGET_ATTACH_NO_MATCH;

	/* Try to match a target with its ROM table */
	for (ti = cortexm_target_ids;
	    ti < &cortexm_target_ids[CORTEXM_TARGET_NIDS]; ti++) {
		rt.rt_pidr = ti->ti_pidr;
		rt.rt_pidr_mask = ti->ti_pidr_mask;

		if (TARGET_LINK_IOCTL(t, ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP,
		    &rt) == 1 &&
		    (ti->ti_pidr != 0ull ||
		    rt.rt_paddr == CORESIGHT_ROM_ADDRESS)) {
			/*
			 * Match found.
			 */
			if (ti->ti_pidr != 0ull) {
				tmon_log("Exact match %08" PRIx32 ".%08" PRIx32
				    ": %s\n", (uint32_t)(ti->ti_pidr >> 32),
				    (uint32_t)ti->ti_pidr, ti->ti_desc);
			} else {
				tmon_log("Wildcard match on CoreSight ROM "
				    "table address.\n");
			}

			rv = (ti->ti_attach)(t, rt.rt_paddr);
			(void)TARGET_LINK_ERROR(t);
			if (rv == TARGET_ATTACH_OK) {
				/* Match found. */
				adiv5_ll_cfg_extra_set =
				    cortexm_hw_cfg_extra_set;
				cortexm_hw_cfg_extra_set = 0;

				adiv5_ll_cfg_extra_clr =
				    cortexm_hw_cfg_extra_clr;
				cortexm_hw_cfg_extra_clr = 0;

				tmon_log("Attach succeeded.\n");
				return TARGET_ATTACH_OK;
			} else
			if (rv == TARGET_ATTACH_RESCAN) {
				/* Target wants a forced rescan. */
				tmon_log("Forced rescan requested.\n");
				break;
			} else {
				tmon_log("Attach failed.\n");
			}
		}
	}

	adiv5_ll_cfg_extra_set = cortexm_hw_cfg_extra_set;
	adiv5_ll_cfg_extra_clr = cortexm_hw_cfg_extra_clr;
	cortexm_hw_cfg_extra_set = cortexm_hw_cfg_extra_clr = 0;

	/* Otherwise there's nothing of interest on this target */
	target_free(t);

	return TARGET_ATTACH_NO_MATCH;
}

static int
other_ap_match_and_attach(const struct adiv5_ap_interface *api, void *arg,
    uint32_t idr)
{
	const struct cortexm_other_ap *oa;
	int rv = TARGET_ATTACH_NO_MATCH;

	for (oa = cortexm_other_aps;
	    oa < &cortexm_other_aps[CORTEXM_OTHER_APS]; oa++) {
		if (oa->oa_idr == (idr & oa->oa_idr_mask)) {
			rv = (oa->oa_match)(api, arg);
			break;
		}
	}

	return rv;
}

static int
target_cortexm_attach(void *arg)
{

	(void) arg;

	return adiv5_attach(mem_ap_match_and_attach, other_ap_match_and_attach);
}

static void
target_cortexm_detach(void *arg)
{

	(void) arg;

	adiv5_cleanup();
}

static void *
target_cortexm_init(void)
{

	adiv5_init();

	semihosting_init();

	swo_serve_init();

	return NULL;
}

#ifdef NETWORK_OPT_HTTPD
static void
target_cortexm_debug_update(void *arg, stringio_t ss)
{

	(void)arg;

	adiv5_debug_update(ss);
	cortexm_debug_update(ss);
}

static int
target_cortexm_debug_update_settings(void *arg, int nkeys,
    const struct network_http_furl_key_value *kv)
{
	int changed;

	(void) arg;

	changed = adiv5_update_settings(nkeys, kv);
	changed += cortexm_update_settings(nkeys, kv);

	return changed;
}
#endif /* NETWORK_OPT_HTTPD */

static void
target_cortexm_load_defaults(void)
{

	adiv5_load_defaults();
	cortexm_load_defaults();
}

TARGET_ARCH_DECLARE(cortexm);
