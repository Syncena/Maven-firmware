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

#ifndef GDB_RTOS_NUTTX_H
#define GDB_RTOS_NUTTX_H

#include "gdb_rtos.h"

/*
 * An instance of this structure will be present in a NuttX firmware image if
 * the configuration constant CONFIG_DEBUG_INCLUDE_DEBUGGER_HINTS is defined.
 * Look for the symbol 'g_nuttx_debugger_hints'.
 */
struct gdb_rtos_nuttx_debugger_hints {
	uint32_t magic;

	/* sizeof(FAR void *) */
	uint16_t sizeof_far_pointer;

	/* offsetof(struct tcb_s, pid) */
	uint16_t offsetof_tcb_pid;

	/* offsetof(struct tcb_s, sched_priority) */
	uint16_t offsetof_tcb_sched_priority;

	/* offsetof(struct tcb_s, xcp) */
	uint16_t offsetof_tcb_xcp;

	/* offsetof(struct tcb_s, name) */
	uint16_t offsetof_tcb_name;	/* '0' if CONFIG_TASK_NAME_SIZE == 0 */

	/* offsetof(struct xcptcontext, regs) */
	uint16_t offsetof_xcptcontext_regs;

	/* sizeof(struct xcptcontext) */
	uint16_t sizeof_xcptcontext_regs;

	/* CONFIG_MAX_TASKS */
	uint16_t config_max_tasks;
};
#define	GDB_RTOS_NUTTX_DEBUGGER_HINTS_MAGIC		0xDEBF0000u
#define	GDB_RTOS_NUTTX_DEBUGGER_HINTS_MAGIC_MASK	0xffffff00u
#define	GDB_RTOS_NUTTX_DEBUGGER_HINTS_SIZEOF_MASK	0x000000ffu

enum {
#define GDB_RTOS_PARAM(e,f,desc,def)	e,
#include "gdb_rtos_nuttx.defs"
#undef GDB_RTOS_PARAM
	GDB_RTOS_NUTTX_NPARAMS
};

struct gdb_rtos_nuttx_target_params {
	/* Current hints, to be verified/adjusted as necessary. */
	struct gdb_rtos_nuttx_debugger_hints *tp_hints;
};

extern const struct gdb_rtos_backend gdb_rtos_backend_nuttx;

GDB_RTOS_PARAMS_DEF(gdb_rtos_nuttx_params_t, GDB_RTOS_NUTTX_NPARAMS);

#endif /* GDB_RTOS_NUTTX_H */
