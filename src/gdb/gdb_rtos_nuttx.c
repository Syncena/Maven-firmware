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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "target.h"
#include "tmon.h"
#include "gdb_rtos.h"
#include "gdb_rtos_nuttx.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT 1
#include "debug.h"

/*
 * NuttX task support.
 *
 * XXX: This is heavily biased towards single-core, little-endian ILP32 targets.
 */

enum gdb_nuttx_symbol_enums {
#define	GDB_RTOS_SYMBOL(e,n,f)	e,
#include "gdb_rtos_nuttx.defs"
#undef GDB_RTOS_SYMBOL
	GDB_RTOS_NUTTX_NSYMBOLS
};

/* See 'enum tstate_e' in nuttx/include/nuttx/sched.h */
#define	GDB_RTOS_NUTTX_NUM_TASK_STATES_MAX	12

struct gdb_nuttx_state {
	tmon_cookie_t fs_tmon;
	const struct gdb_rtos_symbol_value *fs_syms;
	target_addr_t fs_current_task;
	gdb_rtos_params_t fs_params;
	char *fs_task_name;
	size_t fs_task_name_size;
	struct gdb_rtos_nuttx_debugger_hints fs_hints;
};

/* Simple, basic pointer integrity check. */
#define	GDB_NUTTX_BAD_POINTER(fs,v)	(((v) & \
	((fs)->fs_hints.sizeof_far_pointer - 1)) != 0)

static bool
gdb_nuttx_read_value(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t *pv, uint8_t vsize)
{
	bool rv;

	/* XXX: This function could be common for all RTOSes */
	/* XXX: This assumes Maven and target have matching endianness. */

	if (vsize == sizeof(uint8_t)) {
		uint8_t u8;
		rv = tmon_mem_read(fs->fs_tmon, addr, sizeof(u8), &u8) ==
		    TMON_MR_OK;
		if (rv)
			*pv = (uint32_t)u8;
	} else
	if (vsize == sizeof(uint16_t)) {
		uint16_t u16;
		rv = tmon_mem_read(fs->fs_tmon, addr, sizeof(u16), &u16) ==
		    TMON_MR_OK;
		if (rv)
			*pv = (uint32_t)u16;
	} else {
		rv = tmon_mem_read(fs->fs_tmon, addr, sizeof(*pv), pv) ==
		    TMON_MR_OK;
	}

	return rv;
}

static bool
gdb_nuttx_write_value(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t v, uint8_t vsize)
{
	bool rv;

	/* XXX: This function could be common for all RTOSes */
	/* XXX: This assumes Maven and target have matching endianness. */

	if (vsize == sizeof(uint8_t)) {
		uint8_t u8 = (uint8_t)v;
		rv = tmon_mem_write(fs->fs_tmon, addr, sizeof(u8), &u8) ==
		    TMON_MR_OK;
	} else
	if (vsize == sizeof(uint16_t)) {
		uint16_t u16 = (uint16_t)v;
		rv = tmon_mem_write(fs->fs_tmon, addr, sizeof(u16), &u16) ==
		    TMON_MR_OK;
	} else {
		rv = tmon_mem_write(fs->fs_tmon, addr, sizeof(v), &v) ==
		    TMON_MR_OK;
	}

	return rv;
}

static __inline bool
gdb_nuttx_read_uint8(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_nuttx_read_value(fs, addr, pv, sizeof(uint8_t));
}

static __inline bool
gdb_nuttx_read_uint16(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_nuttx_read_value(fs, addr, pv, sizeof(uint16_t));
}

static __inline bool
gdb_nuttx_read_pointer(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_nuttx_read_value(fs, addr, pv,
	    fs->fs_hints.sizeof_far_pointer);
}

static __inline bool
gdb_nuttx_write_pointer(struct gdb_nuttx_state *fs, target_addr_t addr,
    uint32_t v)
{

	return gdb_nuttx_write_value(fs, addr, v,
	    fs->fs_hints.sizeof_far_pointer);
}

static void *
gdb_nuttx_attach(tmon_cookie_t tc, const struct gdb_rtos_symbol_value *sv,
    gdb_rtos_params_t rp)
{
	struct gdb_nuttx_state *fs;

	/* XXX: SMP not yet supported. */
	if (sv[GDB_NUTTX_SYM_config_smp].sv_found) {
		DBFPRINTF("No support for SMP NuttX\n");
		return NULL;
	}

	if ((fs = zone_malloc(sizeof(*fs))) == NULL)
		return NULL;

	fs->fs_tmon = tc;
	fs->fs_syms = sv;
	fs->fs_params = rp;
	fs->fs_current_task = 0;
	fs->fs_task_name = NULL;
	fs->fs_task_name_size = 0;

	return fs;
}

static void
gdb_nuttx_free(void *cookie)
{

	zone_free(cookie);
}

static bool
gdb_nuttx_probe_hints(struct gdb_nuttx_state *fs)
{
	const struct gdb_rtos_symbol_value *sv = fs->fs_syms;
	struct gdb_rtos_nuttx_target_params tp;
	target_addr_t addr;
	uint32_t u32;
	uint16_t ptr_size, u16;

	addr = sv[GDB_NUTTX_SYM_debugger_hints].sv_value;
	if (tmon_mem_read(fs->fs_tmon, addr, sizeof(fs->fs_hints),
	    &fs->fs_hints) != TMON_MR_OK) {
		DBFPRINTF("Failed to read hints at 0x%" PRIxTADDR "\n", addr);
		return false;
	}

	u32 = fs->fs_hints.magic;

	if ((u32 & GDB_RTOS_NUTTX_DEBUGGER_HINTS_MAGIC_MASK) !=
	    GDB_RTOS_NUTTX_DEBUGGER_HINTS_MAGIC) {
		DBFPRINTF("Bad hints magic: 0x%" PRIx32 "\n", u32);
		return false;
	}

	if ((u32 & GDB_RTOS_NUTTX_DEBUGGER_HINTS_SIZEOF_MASK) <
	    sizeof(fs->fs_hints)) {
		DBFPRINTF("Hints sizeof mismatch. Wanted %u, got %" PRIu32 "\n",
		    (unsigned int)sizeof(fs->fs_hints),
		    u32 & GDB_RTOS_NUTTX_DEBUGGER_HINTS_SIZEOF_MASK);
		return false;
	}

	DBFPRINTF("Hints magic checks out.\n");

	/*
	 * The target back-end will validate sizeof_far_pointer,
	 * offsetof_xcptcontext_regs, and sizeof_xcptcontext_regs.
	 */
	tp.tp_hints = &fs->fs_hints;
	if (tmon_rtos_load_params(fs->fs_tmon, TARGET_RTOS_NUTTX, &tp) !=
	    TMON_RLP_OK) {
		DBFPRINTF("Back-end rejected hints\n");
		return false;
	}

	ptr_size = fs->fs_hints.sizeof_far_pointer;
	DBFPRINTF("sizeof_far_pointer %" PRIu16 "\n", ptr_size);

	u16 = fs->fs_hints.offsetof_tcb_pid;
	if (u16 < ptr_size || u16 > 255) {
		DBFPRINTF("Bad offsetof_tcb_pid: %" PRIu16 "\n", u16);
		return false;
	}
	DBFPRINTF("offsetof_tcb_pid %" PRIu16 "\n", u16);

	u16 = fs->fs_hints.offsetof_tcb_sched_priority;
	if (u16 < ptr_size || u16 > 255) {
		DBFPRINTF("Bad offsetof_tcb_sched_priority: %" PRIu16 "\n",
		    u16);
		return false;
	}
	DBFPRINTF("offsetof_tcb_sched_priority %" PRIu16 "\n", u16);

	u16 = fs->fs_hints.offsetof_tcb_xcp;
	if (u16 < fs->fs_hints.offsetof_tcb_sched_priority || u16 > 511) {
		DBFPRINTF("Bad offsetof_tcb_xcp: %" PRIu16 "\n", u16);
		return false;
	}
	DBFPRINTF("offsetof_tcb_xcp %" PRIu16 "\n", u16);

	u16 = fs->fs_hints.offsetof_tcb_name;
	if (u16 > 0 && u16 <= fs->fs_hints.offsetof_tcb_xcp) {
		DBFPRINTF("Bad offsetof_tcb_name: %" PRIu16 "\n", u16);
		return false;
	}
	DBFPRINTF("offsetof_tcb_name %" PRIu16 "\n", u16);

	u16 = fs->fs_hints.config_max_tasks;
	if (u16 == 0 || u16 > GDB_RTOS_MAX_THREADS) {
		DBFPRINTF("Bogus config_max_tasks: %" PRIu16 "\n", u16);
		fs->fs_hints.config_max_tasks = GDB_RTOS_MAX_THREADS;
	}
	DBFPRINTF("config_max_tasks %" PRIu16 "\n", u16);

	DBFPRINTF("Hints appear to be good.\n");

	return true;
}

static bool
gdb_nuttx_task_name_bad(struct gdb_nuttx_state *fs)
{
	unsigned int idx;
	const uint8_t *p;

	for (idx = 0, p = (const uint8_t *)fs->fs_task_name;
	    idx < (unsigned int)fs->fs_task_name_size;
	    idx++, p++) {
		uint8_t ch = (uint8_t)fs->fs_task_name[idx];

		if (ch == (uint8_t)'\0')
			break;

		if (ch < 0x20u || ch >= 0x80u) {
			DBFPRINTF("Bad character 0x%02x at idx %u\n",
			    ch, idx);
			return true;
		}
	}

	if (idx == 0)
		DBFPRINTF("Empty task name.\n");

	return idx == 0;
}

static struct gdb_rtos_thread *
gdb_nuttx_parse_task_queue(struct gdb_nuttx_state *fs, target_addr_t queue,
    struct gdb_rtos_thread *rt, unsigned int remaining_space,
    const char *task_state)
{
	uint32_t task32, pid, priority;
	target_addr_t addr;
	uint32_t task;

	DBFPRINTF("State '%s', queue at 0x%" PRIxTADDR "\n", task_state, queue);

	if (queue == 0)
		return rt;

	if (GDB_NUTTX_BAD_POINTER(fs, queue)) {
		DBFPRINTF("queue pointer is bogus\n");
		return NULL;
	}

	if (gdb_nuttx_read_pointer(fs, queue, &task) == false) {
		DBFPRINTF("Failed to read queue->head\n");
		return NULL;
	}

	DBFPRINTF("First task at 0x%" PRIxTADDR "\n", task);

	if (queue == fs->fs_syms[GDB_NUTTX_SYM_g_readytorun].sv_value) {
		/*
		 * The current task is at the head of g_readytorun.
		 */
		fs->fs_current_task = (target_addr_t)task;
	}

	/* Walk the task list. */
	while (remaining_space && task != 0) {
		if (GDB_NUTTX_BAD_POINTER(fs, task)) {
			DBFPRINTF("task is bogus\n");
			return NULL;
		}

		/* The tcb_s address is used as the RTOS Thread ID. */
		rt->rt_tid = task;

		/* Fetch task->pid */
		addr = task + fs->fs_hints.offsetof_tcb_pid;
		if (gdb_nuttx_read_uint16(fs, addr, &pid) == false) {
			DBFPRINTF("Failed to read task->pid\n");
			return NULL;
		}

		/* Fetch task->sched_priority */
		addr = task + fs->fs_hints.offsetof_tcb_sched_priority;
		if (gdb_nuttx_read_uint8(fs, addr, &priority) == false) {
			DBFPRINTF("Failed to read task->sched_priority\n");
			return NULL;
		}

		/*
		 * Fetch the task name (assumes target is using the same
		 * locale/charset as us...).
		 */
		if (fs->fs_hints.offsetof_tcb_name != 0) {
			addr = rt->rt_tid + fs->fs_hints.offsetof_tcb_name;
			DBFPRINTF("Task name at 0x%" PRIxTADDR "\n", addr);

			fs->fs_task_name[fs->fs_task_name_size] = '\0';
			if (tmon_mem_read(fs->fs_tmon, addr,
			    fs->fs_task_name_size,
			    fs->fs_task_name) != TMON_MR_OK ||
			    gdb_nuttx_task_name_bad(fs)) {
				strcpy(fs->fs_task_name, "Unknown");
			}
		} else {
			strcpy(fs->fs_task_name, "Task");
		}

		if (fs->fs_current_task == task)
			task_state = "Running";

		DBFPRINTF("Task 0x%" PRIxTADDR ": %s, %s, Pri %" PRIu32 "\n",
		    rt->rt_tid, fs->fs_task_name, task_state, priority);

		snprintf(rt->rt_info, sizeof(rt->rt_info), "%s, %s, Pri %"
		    PRIu32, fs->fs_task_name, task_state, priority);

		rt++;
		remaining_space--;

		/*
		 * Link to the next task is at the start of struct tcb_s
		 */
		if (gdb_nuttx_read_pointer(fs, task, &task32) == false) {
			DBFPRINTF("Failed to read task->flink\n");
			return NULL;
		}
		task = (target_addr_t)task32;
		DBFPRINTF("Next task at 0x%" PRIxTADDR "\n", task);
	}

	return rt;
}

static int
gdb_nuttx_threads(void *cookie, struct gdb_rtos_thread **rtp,
    target_addr_t *current_thread)
{
	struct gdb_nuttx_state *fs = cookie;
	struct gdb_rtos_thread *rt, *srt, *ert;
	target_addr_t queue;
	uint32_t max_tasks;
	size_t rt_size;
	int rv;

	fs->fs_current_task = 0;

	if (gdb_nuttx_probe_hints(fs) == false)
		return 0;

	max_tasks = (uint32_t) fs->fs_hints.config_max_tasks;

	/*
	 * XXX: Clamping GDB_RTOS_NUTTX_config_task_name_size to an
	 * arbitrary limit is bogus...
	 */
	static_assert(sizeof(rt->rt_info) >= (16 + 25), "rt_info fail");
	fs->fs_task_name_size =
	    (size_t)fs->fs_params->rp_val[GDB_RTOS_NUTTX_config_task_name_size];
	if (fs->fs_task_name_size == 0)
		fs->fs_task_name_size = 15;
	else
	if (fs->fs_task_name_size >= (sizeof(rt->rt_info) - 25))
		fs->fs_task_name_size = sizeof(rt->rt_info) - 25;

	rt_size = (max_tasks * sizeof(*rt)) + (fs->fs_task_name_size + 1);
	if ((rt = zone_calloc(1, rt_size)) == NULL)
		return 0;
	srt = rt;
	ert = &rt[max_tasks];
	fs->fs_task_name = (char *)ert;

	/* Parse g_readytorun. */
	queue = fs->fs_syms[GDB_NUTTX_SYM_g_readytorun].sv_value;
	rt = gdb_nuttx_parse_task_queue(fs, queue, rt, (unsigned int)(ert - rt),
	    "Ready");
	assert(rt <= ert);
	if (rt == NULL) {
		DBFPRINTF("Parsing g_readytorun failed\n");
		goto fail;
	}

	/* Parse g_pendingtasks. */
	queue = fs->fs_syms[GDB_NUTTX_SYM_g_pendingtasks].sv_value;
	rt = gdb_nuttx_parse_task_queue(fs, queue, rt, (unsigned int)(ert - rt),
	    "Pending");
	assert(rt <= ert);
	if (rt == NULL) {
		DBFPRINTF("Parsing g_pendingtasks failed\n");
		goto fail;
	}

	/* Parse g_inactivetasks. */
	queue = fs->fs_syms[GDB_NUTTX_SYM_g_inactivetasks].sv_value;
	rt = gdb_nuttx_parse_task_queue(fs, queue, rt, (unsigned int)(ert - rt),
	    "Inactive");
	assert(rt <= ert);
	if (rt == NULL) {
		DBFPRINTF("Parsing g_inactivetasks failed\n");
		goto fail;
	}

	/* Parse g_waitingforsemaphore. */
	queue = fs->fs_syms[GDB_NUTTX_SYM_g_waitingforsemaphore].sv_value;
	rt = gdb_nuttx_parse_task_queue(fs, queue, rt, (unsigned int)(ert - rt),
	    "WaitSem");
	assert(rt <= ert);
	if (rt == NULL) {
		DBFPRINTF("Parsing g_waitingforsemaphore failed\n");
		goto fail;
	}

	/* Parse g_waitingforsignal. */
	queue = fs->fs_syms[GDB_NUTTX_SYM_g_waitingforsignal].sv_value;
	rt = gdb_nuttx_parse_task_queue(fs, queue, rt, (unsigned int)(ert - rt),
	    "WaitSig");
	assert(rt <= ert);
	if (rt == NULL) {
		DBFPRINTF("Parsing g_waitingforsignal failed\n");
		goto fail;
	}

	if (fs->fs_syms[GDB_NUTTX_SYM_g_waitingformqnotempty].sv_found) {
		/* Parse g_waitingformqnotempty. */
		queue =
		    fs->fs_syms[GDB_NUTTX_SYM_g_waitingformqnotempty].sv_value;
		rt = gdb_nuttx_parse_task_queue(fs, queue, rt,
		    (unsigned int)(ert - rt), "WaitMQNotEmpty");
		assert(rt <= ert);
		if (rt == NULL) {
			DBFPRINTF("Parsing g_waitingformqnotempty failed\n");
			goto fail;
		}
	}

	if (fs->fs_syms[GDB_NUTTX_SYM_g_waitingformqnotfull].sv_found) {
		/* Parse g_waitingformqnotfull. */
		queue =
		    fs->fs_syms[GDB_NUTTX_SYM_g_waitingformqnotfull].sv_value;
		rt = gdb_nuttx_parse_task_queue(fs, queue, rt,
		    (unsigned int)(ert - rt), "WaitMQNotFull");
		assert(rt <= ert);
		if (rt == NULL) {
			DBFPRINTF("Parsing g_waitingformqnotfull failed\n");
			goto fail;
		}
	}

	if (fs->fs_syms[GDB_NUTTX_SYM_g_waitingforfill].sv_found) {
		/* Parse g_waitingforfill. */
		queue = fs->fs_syms[GDB_NUTTX_SYM_g_waitingforfill].sv_value;
		rt = gdb_nuttx_parse_task_queue(fs, queue, rt,
		    (unsigned int)(ert - rt), "WaitPageFill");
		assert(rt <= ert);
		if (rt == NULL) {
			DBFPRINTF("Parsing g_waitingforfill failed\n");
			goto fail;
		}
	}

	if (fs->fs_syms[GDB_NUTTX_SYM_g_stoppedtasks].sv_found) {
		/* Parse g_stoppedtasks. */
		queue = fs->fs_syms[GDB_NUTTX_SYM_g_stoppedtasks].sv_value;
		rt = gdb_nuttx_parse_task_queue(fs, queue, rt,
		    (unsigned int)(ert - rt), "Stopped");
		assert(rt <= ert);
		if (rt == NULL) {
			DBFPRINTF("Parsing g_stoppedtasks failed\n");
			goto fail;
		}
	}

	rv = (int)(rt - srt);
	DBFPRINTF("Found %d tasks\n", rv);

	*rtp = srt;
	*current_thread = fs->fs_current_task;
	return rv;

 fail:
	zone_free(srt);
	return -1;
}

static target_addr_t
gdb_nuttx_get_frame(void *cookie, target_addr_t tid, void **pparams)
{
	struct gdb_nuttx_state *fs = cookie;
	target_addr_t sp;

	if (tid != fs->fs_current_task) {
		sp = tid + fs->fs_hints.offsetof_tcb_xcp +
		    fs->fs_hints.offsetof_xcptcontext_regs;
	} else {
		sp = 0;
	}

	*pparams = fs->fs_params;

	return sp;
}

static const gdb_rtos_nuttx_params_t gdb_rtos_nuttx_params = {
	.rp_source = GDB_RTOS_PARAM_DEFAULT,
#define	GDB_RTOS_PARAM(e,f,desc,def) .rp_val[e] = def,
#include "gdb_rtos_nuttx.defs"
#undef GDB_RTOS_PARAM
};

static const struct gdb_rtos_param_desc gdb_rtos_nuttx_param_descs[] = {
#define	GDB_RTOS_PARAM(e,f,desc,def)	GDB_RTOS_PARAM_DESC_INIT(desc, f),
#include "gdb_rtos_nuttx.defs"
#undef GDB_RTOS_PARAM
};

static const struct gdb_rtos_symbol gdb_nuttx_symbols[] = {
#define	GDB_RTOS_SYMBOL(e,n,f)	GDB_RTOS_SYMBOL_INIT(n,f),
#include "gdb_rtos_nuttx.defs"
#undef GDB_RTOS_SYMBOL
};

const struct gdb_rtos_backend gdb_rtos_backend_nuttx = {
	.rb_name = "NuttX",
	.rb_type = TARGET_RTOS_NUTTX,
	.rb_nsymbols = GDB_RTOS_NUTTX_NSYMBOLS,
	.rb_symbols = gdb_nuttx_symbols,
	.rb_nparams = GDB_RTOS_NUTTX_NPARAMS,
	.rb_params_size = sizeof(gdb_rtos_nuttx_params_t),
	.rb_params_default =
	    (const gdb_rtos_params_t)(uintptr_t)&gdb_rtos_nuttx_params,
	.rb_params_desc = gdb_rtos_nuttx_param_descs,
	.rb_attach = gdb_nuttx_attach,
	.rb_free = gdb_nuttx_free,
	.rb_reset = NULL,
	.rb_threads = gdb_nuttx_threads,
	.rb_get_frame = gdb_nuttx_get_frame,
};
