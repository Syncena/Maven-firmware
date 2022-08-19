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

#ifndef ADIV5_H
#define ADIV5_H

#include <sys/types.h>
#include <stdbool.h>

#include "target.h"
#include "linked-lists.h"

#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "stringio.h"
#endif
#endif /* CONFIG_USE_NETWORK */

typedef int (*adiv5_mem_ap_iterator_t)(target_link_ops_t, void *);

struct adiv5_ap_interface;
typedef int (*adiv5_other_ap_iterator_t)(const struct adiv5_ap_interface *,
		void *, uint32_t);

/*
 * Go looking for an ADIv5-capable interface and explore what lurks
 * behind it. Returns non-zero if something interesting was found.
 * If adiv5_probe() returns non-zero, invoke adiv5_attach() to
 * iterate over the devices found, and adiv5_ap_match() to locate
 * any additional APs required.
 */
extern int adiv5_probe(void);
extern int adiv5_attach(adiv5_mem_ap_iterator_t, adiv5_other_ap_iterator_t);

extern bool adiv5_ap_exists(unsigned int ap_number);
extern void *adiv5_other_ap_match(uint32_t idr, uint32_t idr_mask,
				  const struct adiv5_ap_interface **);
extern void adiv5_cleanup(void);

/*
 * ADIv5 DP interface definition.
 */
struct adiv5_dp;
typedef struct adiv5_dp *adiv5_dp_t;
struct adiv5_dp_interface {
	const char dpi_name[8];
	u_int dpi_type;
	int (*dpi_init)(adiv5_dp_t, uint32_t);
	void (*dpi_done)(adiv5_dp_t);
	uint32_t (*dpi_xfer)(adiv5_dp_t, uint32_t, uint32_t *);
	void (*dpi_line_reset)(adiv5_dp_t);
	void (*dpi_abort)(adiv5_dp_t, uint32_t);
};

/*
 * Overall state for a Debug Port
 */
struct adiv5_dp {
	uint32_t dp_select;
	uint32_t dp_cfg;
	uint32_t dp_last_ack;
	uint32_t dp_idr;
	uint32_t dp_posted_reg;
	u_int dp_flags;
#define	ADIV5_DP_TYPE(dp)		((dp)->dp_flags & 3u)
#define	ADIV5_DP_STICKY_ERROR(dp)	(((dp)->dp_flags & (1u << 2)) != 0u)
#define	ADIV5_DP_STICKY_ERROR_SET(dp)	((dp)->dp_flags |= (1u << 2))
#define	ADIV5_DP_STICKY_ERROR_CLR(dp)	((dp)->dp_flags &= ~(1u << 2))
#define	ADIV5_DP_IS_MULTIDROP(dp)	(((dp)->dp_flags & (1u << 3)) != 0u)
#define	ADIV5_DP_SET_MULTIDROP(dp)	((dp)->dp_flags |= (1u << 3))
#define	ADIV5_DP_FORCE_SELECT(dp)	(((dp)->dp_flags & (1u << 4)) != 0u)
#define	ADIV5_DP_FORCE_SELECT_SET(dp)	((dp)->dp_flags |= (1u << 4))
#define	ADIV5_DP_FORCE_SELECT_CLR(dp)	((dp)->dp_flags &= ~(1u << 4))
	u_int dp_refcnt;
	uint32_t dp_targetsel;
	const struct adiv5_dp_interface *dp_if;
	void *dp_if_cookie;
	TAILQ_ENTRY(adiv5_dp) dp_qent;
};

/*
 * If adiv5_ap_match() returns a match, it also returns a pointer to the
 * methods used to access the AP.
 */
struct adiv5_ap_interface {
	/*
	 * Read/Write a single 32-bit value in the AP's register space.
	 */
	uint32_t (*ai_read)(void *, uint32_t reg);
	int (*ai_write)(void *, uint32_t reg, uint32_t data, bool flush);

	/*
	 * Returns non-zero if a DP error has been flagged since the last
	 * invocation. The return value is the actual error status.
	 */
	uint32_t (*ai_dp_error)(void *);

	/*
	 * Call this when you're finished with the AP.
	 */
	void (*ai_free)(void *);
};

/*
 * Mem-AP ioctl codes
 */
/* 'arg' is a pointer to uint8_t */
#define	ADIV5_MEMAP_CTL_CSW_PROT	0u
#define	ADIV5_MEMAP_CSW_PROT(n)		(1u << (n))

#define	ADIV5_MEMAP_CTL_ROM_TABLE_LOOKUP	1u
struct adiv5_memap_rom_table_lookup {
	uint64_t rt_pidr;
	uint64_t rt_pidr_mask;
	target_addr_t rt_paddr;
};

#define	ADIV5_MEMAP_CTL_LINK_DETAILS	2u
struct adiv5_memap_link_details {
	u_int ld_link_type;
#define	ADIV5_TYPE_SWDP		0u
#define	ADIV5_TYPE_SWJDP	1u
#define	ADIV5_TYPE_JTAGDP	2u
	const char *ld_link_name;
	uint32_t ld_apsel;
	uint32_t ld_targetsel;
};

/* 'arg' is a pointer to an unsigned int. */
#define	ADIV5_MEMAP_CTL_SET_TAR_BITS	3u

/* 'arg' is ignored. */
#define	ADIV5_MEMAP_CTL_DP_POWER_UP	4u

/* Bit definitions for PIDR */
#define	ADIV5_PIDR_PART(p)	((unsigned int)(((p) >> 0) & 0xfffu))
#define	ADIV5_PIDR_DES(p)	((unsigned int)((((p) >> 12) & 0x7fu) | \
				 (((p) >> 25) & 0x780u)))
#define	ADIV5_PIDR_JEDEC(p)	((unsigned int)(((p) >> 19) & 0x1u))
#define	ADIV5_PIDR_REVISION(p)	((unsigned int)(((p) >> 20) & 0xfu))
#define	ADIV5_PIDR_CMOD(p)	((unsigned int)(((p) >> 24) & 0xfu))
#define	ADIV5_PIDR_REVAND(p)	((unsigned int)(((p) >> 28) & 0xfu))
#define	ADIV5_PIDR_SIZE(p)	((unsigned int)(((p) >> 36) & 0xfu))

/* For ARM parts, ADIV5_PIDR_DES() evaluates to this... */
#define	ADIV5_DESIGNER_ARM	0x23bu

/*
 * Definitions for ROM Tables
 */
/* ROM Tables have a MEMTYPE register */
#define	ADIV5_ROMTBL_REG_MEMTYPE	0xfccu
#define  ADIV5_ROMTBL_MEMTYPE_SYSMEM(m)	(((m) & 0x1u) != 0)

/* Definitions for ROM Tables Entries */
#define	ADIV5_ROMTBL1_MAX_ENTRIES	960u
#define	ADIV5_ROMTBL9_MAX_ENTRIES	512u
#define	ADIV5_ROMENTRY_VALID(r)		((r) != 0)
#define	ADIV5_ROMENTRY_PRESENT(r)	(((r) & 0x1u) != 0)
#define	ADIV5_ROMENTRY_FORMAT_32(r)	(((r) & 0x2u) != 0)
#define	ADIV5_ROMENTRY_POWERID_VALID(r)	(((r) & 0x4u) != 0)
#define	ADIV5_ROMENTRY_POWERID(r)	((unsigned int)(((r) >> 4) & 0x1fu))
#define	ADIV5_ROMENTRY_OFFSET(r)	((int64_t)(int32_t)((r) & 0xfffff000u))

/*
 * Given a ROM table base address and a ROM entry, work out the target
 * address of the component.
 */
#define	ADIV5_RTE2ADDR(b,rte)	((target_addr_t) \
				 (((int64_t)(b)) + ADIV5_ROMENTRY_OFFSET(rte)))

#ifndef CONFIG_USE_CONFIGDB
#define	ADIV5_GLOBAL_VARS		\
	uint8_t	glob_adiv5_idles;	\
	uint8_t	glob_adiv5_turnaround;	\
	uint8_t	glob_adiv5_speed;	\
	uint8_t	glob_adiv5_ahb_ap;	\
	uint8_t	glob_adiv5_interface;	\
	ADIV5_LL_GLOBAL_VARS
#endif

#ifdef NETWORK_OPT_HTTPD
struct network_http_furl_key_value;
extern int adiv5_update_settings(int,
		const struct network_http_furl_key_value *);
extern void adiv5_debug_update(stringio_t);
#endif
extern void adiv5_load_defaults(void);
extern void adiv5_init(void);
extern void adiv5_dp_get_speed_idles(unsigned int *, unsigned int *);
extern const uint8_t adiv5_speed_2cfg[];

#endif /* ADIV5_H */
