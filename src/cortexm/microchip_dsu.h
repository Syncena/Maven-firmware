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

#ifndef MICROCHIP_DSU_H
#define MICROCHIP_DSU_H

#include "adiv5.h"
#include "target.h"

#define	 MCP_DSU_DID_PROCESSOR(d)		(((d) >> 28) & 0xfu)
#define	 MCP_DSU_DID_FAMILY(d)			(((d) >> 23) & 0x1fu)
#define	 MCP_DSU_DID_SERIES(d)			(((d) >> 16) & 0x3fu)
#define	 MCP_DSU_DID_DIE(d)			(((d) >> 12) & 0xfu)
#define	 MCP_DSU_DID_REVISION(d)		(((d) >> 8) & 0xfu)
#define	 MCP_DSU_DID_DEVSEL(d)			(((d) >> 0) & 0xffu)

#define	MCP_DSU_FLAGS_TYPE(x)	((x) & 3u)
#define	MCP_DSU_FLAGS_TYPE_M0	(0u << 0)
#define	MCP_DSU_FLAGS_TYPE_M4	(1u << 0)
#define	MCP_DSU_FLAGS_TYPE_ROM	(2u << 0)
#define	MCP_DSU_FLAGS_TRUSTZONE	(1u << 2)

typedef enum {
	MCP_DSU_PROT_NONE = 0,	/* Device is unlocked. */
	MCP_DSU_PROT_PARTIAL,	/* Locked, but CPU still available (TZ DAL1) */
	MCP_DSU_PROT_LOCKED,	/* Locked, but Chip Erase possible. */
	MCP_DSU_PROT_SECURE	/* Permanently locked. */
} microchip_dsu_protection_t;

extern int microchip_dsu_attach(target_t t, target_addr_t rom_table);

extern void *microchip_dsu_core_probe_did(target_t, uint32_t *);
extern int microchip_dsu_protection_status(void *, uint32_t,
					   microchip_dsu_protection_t *);
extern void microchip_dsu_done(void *);
extern int microchip_dsu_ctl(void *, uint32_t *, void *);

#endif /* MICROCHIP_DSU_H */
