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

#ifndef CORTEXM_H
#define CORTEXM_H

#include "adiv5.h"
#include "target.h"
#include "microchip_dsu.h"
#include "bitmap.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "stringio.h"
#endif
#endif /* CONFIG_USE_NETWORK */

struct cortexm;
typedef struct cortexm *cortexm_t;

struct cortexm_txml_regmap;

struct cortexm {
	target_t cm_target;
	char cm_core_name[32];
	u_int cm_flags;
#define	CORTEXM_FLAG_ARCH_MASK		(3u << 0)
#define	CORTEXM_FLAG_ARCH(cm)		((cm)->cm_flags&CORTEXM_FLAG_ARCH_MASK)
#define	CORTEXM_FLAG_ARCH_V6M		0u
#define	CORTEXM_FLAG_ARCH_V7M		1u
#define	CORTEXM_FLAG_ARCH_V8M		2u
#define	CORTEXM_FLAG_V8M_EXT_BASE	(0u << 2)
#define	CORTEXM_FLAG_V8M_EXT_MAIN	(1u << 2)
#define	CORTEXM_FLAG_V8M_EXT_SECURITY	(1u << 3)
#define	CORTEXM_FLAG_V8M_EXT_DSP	(1u << 4)
#define	CORTEXM_FLAG_HAS_FPU		(1u << 8)
#define	CORTEXM_FLAG_FPU_DP		(1u << 9)
#define	CORTEXM_FLAG_HAS_ARM_CACHE	(1u << 10)
#define	CORTEXM_FLAG_HAS_SOC_CACHE	(1u << 11)
#define	CORTEXM_FLAG_HAS_MPU		(1u << 12)
#define	CORTEXM_FLAG_ATTACHED		(1u << 13)
#define	CORTEXM_FLAG_BKPT		(1u << 14)
#define	CORTEXM_FLAG_SINGLE_STEP	(1u << 15)
#define	CORTEXM_FLAG_COMP_V2		(1u << 16)
#define	CORTEXM_FLAG_APPLET_RUNNING	(1u << 17)
#define	CORTEXM_FLAG_UNTRUSTED_ONLY	(1u << 18)
#define	CORTEXM_FLAG_RESUME_CALLED	(1u << 19)
#define	CORTEXM_FLAG_NO_RESET		(1u << 20)

	uint32_t cm_cpuid;
	uint32_t cm_ccr;	/* Valid only when core halted. */
	uint32_t cm_fpccr;	/* Valid only when core halted. */
	uint32_t cm_fpcar;	/* Valid only when core halted. */
	uint32_t cm_ctr;
	uint32_t cm_clidr;
	uint32_t cm_demcr;
	uint32_t cm_demcr_cfg_bits;
	uint32_t cm_mpu_regions;
	uint32_t cm_mpu_ctrl_save;
	u_int cm_last_halt_reason;
	struct target_watchbreak cm_last_watchbreak;
	struct target_syscall cm_syscall;

	/* RTOS stack frame support. */
	const struct cortexm_txml_regframe *cm_rtos_frame;
	bool cm_rtos_frame_dynamic;
	unsigned int cm_rtos_frame_count;
	target_addr_t cm_rtos_frame_base;

	/* System peripheral bases. */
	uint32_t cm_scs_base;
	uint32_t cm_fpb_base;
	uint32_t cm_dwt_base;

	u_int cm_features;
	const struct cortexm_txml_regmap *cm_regmap_arch;
	const struct cortexm_txml_regmap *cm_regmap_security;

	void *cm_soc;
	void (*cm_soc_free)(cortexm_t);
	int (*cm_soc_ctl)(cortexm_t, uint32_t *, void *);
	void (*cm_soc_state_changed)(cortexm_t, u_int);

	int (*cm_core_reset)(cortexm_t, int);

/* More than enough h/w watchpoints for any extant Cortex-M device. */
#define	CORTEXM_MAX_WATCHPOINTS		16
	unsigned int cm_num_hw_watchpoints;
	BITMAP_DECLARE(cm_hw_watchpoints, CORTEXM_MAX_WATCHPOINTS);

/* More than enough h/w breakpoints for any supported Cortex-M device. */
#define	CORTEXM_MAX_HW_BREAKPOINTS	16
	unsigned int cm_num_hw_breakpoints;
	BITMAP_DECLARE(cm_hw_breakpoints, CORTEXM_MAX_HW_BREAKPOINTS);

/*
 * Don't get carried away with the number of s/w breakpoints. We need to
 * clear them every time the CPU core halts, and restore them on resume.
 * This will slow things down if the user sets a whole bunch of them.
 */
#define	CORTEXM_MAX_SW_BREAKPOINTS	64
	BITMAP_DECLARE(cm_sw_breakpoints, CORTEXM_MAX_SW_BREAKPOINTS);
	uint16_t cm_sw_breakpoint_insn[CORTEXM_MAX_SW_BREAKPOINTS];
	target_addr_t cm_sw_breakpoint_addr[CORTEXM_MAX_SW_BREAKPOINTS];
#define	CORTEXM_IS_BKPT_INSN(i)		(((i) & 0xff00u) == 0xbe00u)
#define	CORTEXM_MAKE_BKPT_INSN(id)	(0xbe00u | ((id) & 0x7fu))
#define	CORTEXM_BKPT_ID(i)		((i) & 0xffu)
#define	CORTEXM_BKPT_SEMIHOSTING	0xabu
};

#define	CORTEXM_V8M_HAS_EXT_MAIN(cm)					      \
		(((cm)->cm_flags &					      \
		 (CORTEXM_FLAG_ARCH_MASK | CORTEXM_FLAG_V8M_EXT_MAIN)) ==     \
		(CORTEXM_FLAG_ARCH_V8M | CORTEXM_FLAG_V8M_EXT_MAIN))
#define	CORTEXM_V8M_HAS_EXT_SECURITY(cm)				      \
		(((cm)->cm_flags &					      \
		 (CORTEXM_FLAG_ARCH_MASK | CORTEXM_FLAG_V8M_EXT_SECURITY)) == \
		(CORTEXM_FLAG_ARCH_V8M | CORTEXM_FLAG_V8M_EXT_SECURITY))
#define	CORTEXM_V8M_HAS_EXT_DSP(cm)					      \
		(((cm)->cm_flags &					      \
		 (CORTEXM_FLAG_ARCH_MASK | CORTEXM_FLAG_V8M_EXT_DSP)) ==      \
		(CORTEXM_FLAG_ARCH_V8M | CORTEXM_FLAG_V8M_EXT_DSP))

#define	CORTEXM_HAS_MAIN(cm)	\
	((CORTEXM_FLAG_ARCH(cm) == CORTEXM_FLAG_ARCH_V7M) || \
	CORTEXM_V8M_HAS_EXT_MAIN(cm))

extern uint32_t cortexm_core_reg_read(cortexm_t cm, u_int reg);
extern void cortexm_core_reg_write(cortexm_t cm, u_int reg, uint32_t v);

extern int cortexm_attach(target_t, target_addr_t);

#define	CORTEXM_SOC_ATTACH_OK		1
#define	CORTEXM_SOC_ATTACH_NO_MATCH	0
#define	CORTEXM_SOC_ATTACH_RESCAN	(-1)

extern bool cortexm_v8m_secure_debug_available(cortexm_t);

/*
 * Cortex-M CTL codes, used to query SoC-specific code.
 */
/* 'arg' points to struct cortexm_cache_params */
#define	CORTEXM_CTL_CACHE_PARAMS	100
struct cortexm_cache_params {
	const char *cp_type;
	uint32_t cp_sets;
	uint16_t cp_ways;
	uint16_t cp_linesize;
};

#ifdef NETWORK_OPT_HTTPD
extern int cortexm_update_settings(int,
		const struct network_http_furl_key_value *);
extern void cortexm_debug_update(stringio_t);
#endif

extern void cortexm_load_defaults(void);

#ifndef CONFIG_USE_CONFIGDB
#define CORTEXM_GLOBAL_VARS			\
	uint32_t	glob_cortexm_vc;	\
	unsigned int	glob_cortexm_flags;
#endif

extern uint32_t cortexm_hw_cfg_extra_set, cortexm_hw_cfg_extra_clr;

#endif /* CORTEXM_H */
