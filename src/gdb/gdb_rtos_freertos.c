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
#include "gdb_rtos_freertos.h"
#include "zone_alloc.h"

//#define DEBUG_FLAG_INIT 1
#include "debug.h"

/*
 * FreeRTOS task support.
 *
 * XXX: This is heavily biased towards single-core, little-endian ILP32 targets.
 */

enum {
#define	GDB_RTOS_SYMBOL(e,n,f)	e,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_SYMBOL
	GDB_RTOS_FREERTOS_NSYMBOLS
};

struct gdb_freertos_state {
	tmon_cookie_t fs_tmon;
	const struct gdb_rtos_symbol_value *fs_syms;
	target_addr_t fs_current_task;
	gdb_rtos_params_t fs_params;
	char fs_task_name[32+1];
};

/* Simple, basic pointer integrity check. */
#define	GDB_FREERTOS_BAD_POINTER(fs,v)	(((v) & \
	((fs)->fs_params->rp_val[GDB_RTOS_FREERTOS_sizeof_pointer] - 1)) != 0)

#define	GDB_FREERTOS_ITEM_VALUE32	0xffffffffu
#define	GDB_FREERTOS_INTEGRITY_VALUE32	0x5a5a5a5au
#define	GDB_FREERTOS_ITEM_VALUE16	0xffffu
#define	GDB_FREERTOS_INTEGRITY_VALUE16	0x5a5au

static bool
gdb_freertos_read_value(struct gdb_freertos_state *fs, target_addr_t addr,
    uint32_t *pv, uint8_t vsize)
{
	bool rv;

	/* XXX: This function could be common for all RTOSes */

	assert(vsize == sizeof(uint16_t) || vsize == sizeof(uint32_t));

	/* XXX: This assumes Maven and target have matching endianness. */

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

static __inline bool
gdb_freertos_read_word(struct gdb_freertos_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_freertos_read_value(fs, addr, pv,
	    fs->fs_params->rp_val[GDB_RTOS_FREERTOS_sizeof_BaseType_t]);
}

static __inline bool
gdb_freertos_read_pointer(struct gdb_freertos_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_freertos_read_value(fs, addr, pv,
	    fs->fs_params->rp_val[GDB_RTOS_FREERTOS_sizeof_pointer]);
}

static __inline bool
gdb_freertos_read_ticktype(struct gdb_freertos_state *fs, target_addr_t addr,
    uint32_t *pv)
{

	return gdb_freertos_read_value(fs, addr, pv,
	    fs->fs_params->rp_val[GDB_RTOS_FREERTOS_sizeof_TickType_t]);
}

static void *
gdb_freertos_attach(tmon_cookie_t tc, const struct gdb_rtos_symbol_value *sv,
    gdb_rtos_params_t rp)
{
	struct gdb_freertos_state *fs;

	if ((fs = zone_malloc(sizeof(*fs))) == NULL)
		return NULL;

	fs->fs_tmon = tc;
	fs->fs_syms = sv;
	fs->fs_params = rp;

	/*
	 * If we've been supplied with our default parameters, try to
	 * intuit the correct values based on provided symbols and
	 * probing the target.
	 */
	if (fs->fs_params->rp_source < GDB_RTOS_PARAM_PROBED) {
		struct gdb_rtos_freertos_target_params tp;

		tp.tp_params = fs->fs_params;
		tp.tp_short_ticktype = false;
		tp.tp_integrity = false;
		tp.tp_mpu_wrappers =
		    sv[GDB_FREERTOS_SYM_vPortStoreTaskMPUSettings].sv_found ||
		    sv[GDB_FREERTOS_SYM_vTaskAllocateMPURegions].sv_found;

		if (tmon_rtos_load_params(tc, TARGET_RTOS_FREERTOS, &tp) !=
		    TMON_RLP_OK) {
			DBFPRINTF("Warning: Failed to get target params!\n");
		}
	}

	return fs;
}

static void
gdb_freertos_free(void *cookie)
{

	zone_free(cookie);
}

static bool
gdb_freertos_intuit_params(struct gdb_freertos_state *fs,
    target_addr_t list_addr)
{
	struct gdb_rtos_freertos_target_params tp;
	target_addr_t minilist_addr;
	const gdb_rtos_param_t *p = fs->fs_params->rp_val;
	uint32_t u32;

	tp.tp_params = fs->fs_params;
	tp.tp_mpu_wrappers =
	    fs->fs_syms[GDB_FREERTOS_SYM_vPortStoreTaskMPUSettings].sv_found ||
	    fs->fs_syms[GDB_FREERTOS_SYM_vTaskAllocateMPURegions].sv_found;

	/*
	 * Try to intuit if configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES is
	 * active by checking for the magic list integrity value at the
	 * start of the list.
	 *
	 * XXX: This will break if sizeof(TickType_t) > sizeof(u32).
	 */
	if (tmon_mem_read(fs->fs_tmon, list_addr, sizeof(u32), &u32) !=
	    TMON_MR_OK) {
		DBFPRINTF("Failed to read *List_t at 0x%" PRIxTADDR "\n",
		    list_addr);
		return false;
	}

	DBFPRINTF("First word is 0x%" PRIx32 "\n", u32);

	if (u32 == GDB_FREERTOS_INTEGRITY_VALUE32) {
		tp.tp_integrity = true;
		tp.tp_short_ticktype = false;
	} else
	if ((u32 & 0xffffu) == GDB_FREERTOS_INTEGRITY_VALUE16) {
		tp.tp_integrity = true;
		tp.tp_short_ticktype = true;
	} else {
		/*
		 * List integrity is not in use. We still need to work
		 * out sizeof(TickType_t).
		 * Try first with sizeof(TickType_t) == sizeof(uint32_t).
		 */
		tp.tp_integrity = false;
		tp.tp_short_ticktype = false;
	}

	/* Fetch magic based on integrity and ticktype size. */
	if (tmon_rtos_load_params(fs->fs_tmon, TARGET_RTOS_FREERTOS, &tp) !=
	    TMON_RLP_OK) {
		DBFPRINTF("tmon_rtos_load_params() failed\n");
		return false;
	}

	/* If integrity is being used, we're good to go. */
	if (tp.tp_integrity) {
		DBFPRINTF("Detected integrity, sizeof(TickType_t) == %"
		    PRIuRTPARAM "\n", p[GDB_RTOS_FREERTOS_sizeof_TickType_t]);
		return true;
	}

	/*
	 * Try reading list->xListEnd.xItemValue.
	 * If we read 0xffffffff, then sizeof(TickType_t) is indeed 4.
	 * If we read 0x....ffff, then sizeof(TickType_t) is 2.
	 */
	minilist_addr = list_addr +
	    p[GDB_RTOS_FREERTOS_offsetof_List_t_xListEnd];
	minilist_addr += p[GDB_RTOS_FREERTOS_offsetof_MiniList_t_xItemValue];

	DBFPRINTF("Integrity not in use. Checking MiniList_t.xItemValue at 0x%"
	    PRIxTADDR "\n", minilist_addr);

	if (tmon_mem_read(fs->fs_tmon, minilist_addr, sizeof(u32), &u32) !=
	    TMON_MR_OK) {
		DBFPRINTF("Failed to read MiniList_t.xItemValue\n");
		return false;
	}

	if (u32 == GDB_FREERTOS_ITEM_VALUE32) {
		/* We guessed correctly. */
		DBFPRINTF("sizeof(TickType_t) == 4\n");
		return true;
	}

	/* Try for sizeof(TickType_t) == sizeof(uint16_t). */
	if ((u32 & 0xffffu) != GDB_FREERTOS_ITEM_VALUE16) {
		DBFPRINTF("Bogus MiniList_t.xItemValue: 0x%08" PRIxTADDR "\n",
		    u32);
		return false;
	}

	/* sizeof(TickType_t) seems to be sizeof(uint16_t). */
	tp.tp_short_ticktype = true;
	DBFPRINTF("sizeof(TickType_t) == sizeof(uint16_t)\n");

	/* Re-fetch params based on updated ticktype size. */
	if (tmon_rtos_load_params(fs->fs_tmon, TARGET_RTOS_FREERTOS, &tp) !=
	    TMON_RLP_OK) {
		DBFPRINTF("tmon_rtos_load_params() failed\n");
		return false;
	}

	return true;
}

static struct gdb_rtos_thread *
gdb_freertos_parse_list(struct gdb_freertos_state *fs, target_addr_t list_addr,
    struct gdb_rtos_thread *rt, unsigned int remaining_space, const char *info)
{
	target_addr_t addr, item_addr, list_end;
	uint32_t x, nitems, value, prev, next;
	const gdb_rtos_param_t *p = fs->fs_params->rp_val;

	DBFPRINTF("List_t at 0x%" PRIxTADDR ", space for %u tasks\n",
	    list_addr, remaining_space);

	/* Fetch list->uxNumberOfItems */
	addr = list_addr + p[GDB_RTOS_FREERTOS_offsetof_List_t_uxNumberOfItems];

	if (gdb_freertos_read_word(fs, addr, &nitems) == false) {
		DBFPRINTF("Failed to read uxNumberOfItems\n");
		return NULL;
	}
	DBFPRINTF("uxNumberOfItems = %" PRIu32 "\n", nitems);
	if (nitems > (uint32_t)remaining_space) {
		DBFPRINTF("uxNumberOfItems exceeds remaining space!\n");
		return NULL;
	}

	/* Fetch list->xListEnd.xItemValue */
	list_end = list_addr + p[GDB_RTOS_FREERTOS_offsetof_List_t_xListEnd];
	addr = list_end + p[GDB_RTOS_FREERTOS_offsetof_MiniList_t_xItemValue];
	if (gdb_freertos_read_ticktype(fs, addr, &value) == false) {
		DBFPRINTF("Failed to read xListEnd.xItemValue\n");
		return NULL;
	}
	DBFPRINTF("xListEnd.xItemValue = 0x%" PRIx32 "\n", value);
	if (p[GDB_RTOS_FREERTOS_sizeof_TickType_t] == sizeof(uint16_t))
		x = GDB_FREERTOS_ITEM_VALUE16;
	else
		x = GDB_FREERTOS_ITEM_VALUE32;
	if (value != x) {
		DBFPRINTF("xListEnd.xItemValue is bogus\n");
		return NULL;
	}

	/* Fetch list->xListEnd.pxNext */
	addr = list_end + p[GDB_RTOS_FREERTOS_offsetof_MiniList_t_pxNext];
	if (gdb_freertos_read_pointer(fs, addr, &next) == false) {
		DBFPRINTF("Failed to read xListEnd.pxNext\n");
		return NULL;
	}
	DBFPRINTF("xListEnd.pxNext = 0x%" PRIx32 "\n", next);
	if (GDB_FREERTOS_BAD_POINTER(fs, next)) {
		DBFPRINTF("xListEnd.pxNext is bogus\n");
		return NULL;
	}

	/* Fetch list->xListEnd.pxPrevious */
	addr = list_end + p[GDB_RTOS_FREERTOS_offsetof_MiniList_t_pxPrevious];
	if (gdb_freertos_read_pointer(fs, addr, &prev) == false) {
		DBFPRINTF("Failed to read xListEnd.pxPrevious\n");
		return NULL;
	}
	DBFPRINTF("xListEnd.pxPrevious = 0x%" PRIx32 "\n", prev);
	if (GDB_FREERTOS_BAD_POINTER(fs, prev)) {
		DBFPRINTF("xListEnd.pxPrevious is bogus\n");
		return NULL;
	}

	item_addr = (target_addr_t)next;

	/*
	 * Walk the list.
	 */
	while (nitems-- && item_addr != 0 && item_addr != list_end) {
		uint32_t owner, container;

		DBFPRINTF("ListItem_t at 0x%" PRIxTADDR "\n", item_addr);

		/* Ignore ListItem_t.xItemValue; it's uninitialised. */

		/* Fetch list_item->pxNext */
		addr = item_addr +
		    p[GDB_RTOS_FREERTOS_offsetof_ListItem_t_pxNext];
		if (gdb_freertos_read_pointer(fs, addr, &next) == false) {
			DBFPRINTF("Failed to read list_item->pxNext\n");
			return NULL;
		}
		DBFPRINTF("list_item->pxNext = 0x%" PRIx32 "\n", next);
		if (GDB_FREERTOS_BAD_POINTER(fs, next)) {
			DBFPRINTF("list_item->pxNext is bogus\n");
			return NULL;
		}

		/* Fetch list_item->pxPrevious */
		addr = item_addr +
		    p[GDB_RTOS_FREERTOS_offsetof_ListItem_t_pxPrevious];
		if (gdb_freertos_read_pointer(fs, addr, &prev) == false) {
			DBFPRINTF("Failed to read list_item->pxPrevious\n");
			return NULL;
		}
		DBFPRINTF("list_item->pxPrevious = 0x%" PRIx32 "\n", prev);
		if (GDB_FREERTOS_BAD_POINTER(fs, prev)) {
			DBFPRINTF("list_item->pxPrevious is bogus\n");
			return NULL;
		}

		/* Fetch list_item->pvOwner */
		addr = item_addr +
		    p[GDB_RTOS_FREERTOS_offsetof_ListItem_t_pvOwner];
		if (gdb_freertos_read_pointer(fs, addr, &owner) == false) {
			DBFPRINTF("Failed to read list_item->pvOwner\n");
			return NULL;
		}
		DBFPRINTF("list_item->pvOwner = 0x%" PRIx32 "\n", owner);
		if (GDB_FREERTOS_BAD_POINTER(fs, owner)) {
			DBFPRINTF("list_item->pvOwner is bogus\n");
			return NULL;
		}

		/* Fetch list_item->pxContainer */
		addr = item_addr +
		    p[GDB_RTOS_FREERTOS_offsetof_ListItem_t_pxContainer];
		if (gdb_freertos_read_pointer(fs, addr, &container) == false) {
			DBFPRINTF("Failed to read list_item->pxContainer\n");
			return NULL;
		}
		DBFPRINTF("list_item->pxContainer = 0x%" PRIx32 "\n", container);
		if (container != (uint32_t)list_addr) {
			DBFPRINTF("list_item->pxContainer is bogus\n");
			return NULL;
		}

		/* The TCB_t address is used as the RTOS Thread ID. */
		rt->rt_tid = (target_addr_t)owner;

		/*
		 * Fetch the task name (assumes target is using the same
		 * locale/charset as us...).
		 */
		addr = rt->rt_tid +
		    p[GDB_RTOS_FREERTOS_offsetof_TCB_t_pcTaskName];
		fs->fs_task_name[sizeof(fs->fs_task_name) - 1] = '\0';
		if (tmon_mem_read(fs->fs_tmon, addr,
		    sizeof(fs->fs_task_name) - 1,
		    fs->fs_task_name) != TMON_MR_OK ||
		    strlen(fs->fs_task_name) == 0) {
			strcpy(fs->fs_task_name, "Unknown");
		}

		DBFPRINTF("Task 0x%" PRIxTADDR ": %s, %s\n", rt->rt_tid,
		    fs->fs_task_name, info);

		snprintf(rt->rt_info, sizeof(rt->rt_info), "%s, %s",
		    fs->fs_task_name,
		    (fs->fs_current_task == rt->rt_tid) ? "Running" : info);

		/* Next task on the list. */
		item_addr = (target_addr_t)next;
		rt++;
	}

	return rt;
}

static int
gdb_freertos_threads(void *cookie, struct gdb_rtos_thread **rtp,
    target_addr_t *current_thread)
{
	struct gdb_freertos_state *fs = cookie;
	struct gdb_rtos_thread *rt, *srt, *ert;
	uint32_t ntasks, current_task, priorities;
	target_addr_t list_addr;
	int rv;

	/*
	 * Fetch the FreeRTOS configuration option "configMAX_PRIORITIES".
	 * This will actually be configMAX_PRIORITIES-1, so correct afterwards.
	 */
	if (gdb_freertos_read_word(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_uxTopUsedPriority].sv_value,
	    &priorities) == false) {
		DBFPRINTF("Failed to read uxTopUsedPriority\n");
		return -1;
	}
	DBFPRINTF("uxTopUsedPriority = %" PRIu32 "\n", priorities);

	/*
	 * The value could be zero if the target has just reset. We also
	 * need to constrain it to something reasonable.
	 * XXX: It would be nice to tell the user in the latter case.
	 */
	if (priorities++ == 0 || priorities > GDB_RTOS_MAX_THREADS) {
		DBFPRINTF("uxTopUsedPriority is invalid.\n");
		return 0;
	}

	/*
	 * Fetch the current number of FreeRTOS tasks.
	 */
	if (gdb_freertos_read_word(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_uxCurrentNumberOfTasks].sv_value,
	    &ntasks) == false) {
		DBFPRINTF("Failed to read uxCurrentNumberOfTasks\n");
		return -1;
	}
	DBFPRINTF("uxCurrentNumberOfTasks = %" PRIu32 "\n", ntasks);

	/*
	 * Bail if there are no tasks, or if our configured maximum
	 * has been exceeded.
	 * XXX: It would be nice to tell the user in the latter case.
	 */
	if (ntasks == 0 || ntasks > GDB_RTOS_MAX_THREADS) {
		DBFPRINTF("uxCurrentNumberOfTasks is invalid.\n");
		return 0;
	}

	/*
	 * Fetch the current task. This can be NULL under certain conditions.
	 */
	if (gdb_freertos_read_pointer(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_pxCurrentTCB].sv_value,
	    &current_task) == false) {
		DBFPRINTF("Failed to read pxCurrentTCB\n");
		return -1;
	}

	fs->fs_current_task = (target_addr_t)current_task;
	DBFPRINTF("pxCurrentTCB is 0x%" PRIx32 "\n", current_task);

	if ((rt = zone_calloc(ntasks, sizeof(*rt))) == NULL)
		return 0;
	srt = rt;
	ert = &rt[ntasks];

	/*
	 * FreeRTOS does not maintain a central list of all tasks.
	 * We need to scan all the places where a task could be linked
	 * in order to account for them all.
	 */

	/* Start with the ready lists. */
	for (unsigned int rdy = 0; rdy < priorities; rdy++) {
		char qname[16];

		snprintf(qname, sizeof(qname), "Ready, Pri %u", rdy);

		list_addr =
		    fs->fs_syms[GDB_FREERTOS_SYM_pxReadyTasksLists].sv_value +
		    (rdy *
		    fs->fs_params->rp_val[GDB_RTOS_FREERTOS_sizeof_List_t]);

		/*
		 * If required, ensure we'll use the correct params
		 * for the FreeRTOS data structure.
		 */
		if (rdy == 0 &&
		    fs->fs_params->rp_source <= GDB_RTOS_PARAM_PROBED &&
		    gdb_freertos_intuit_params(fs, list_addr) == false) {
			DBFPRINTF("Params requested, but failed.\n");
			goto fail;
		}

		DBFPRINTF("Parsing pxReadyTasksLists[%u]\n", rdy);
		rt = gdb_freertos_parse_list(fs, list_addr, rt,
		    (unsigned int)(ert - rt), qname);
		assert(rt <= ert);

		if (rt == NULL) {
			DBFPRINTF("Parse failed\n");
			goto fail;
		}
	}

	/* Delayed task lists 1 and 2. */
	DBFPRINTF("Parsing xDelayedTaskList1\n");
	rt = gdb_freertos_parse_list(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_xDelayedTaskList1].sv_value, rt,
	    (unsigned int)(rt - ert), "Blocked");
	if (rt == NULL) {
		DBFPRINTF("Parse failed\n");
		goto fail;
	}
	assert(rt <= ert);

	DBFPRINTF("Parsing xDelayedTaskList2\n");
	rt = gdb_freertos_parse_list(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_xDelayedTaskList2].sv_value, rt,
	    (unsigned int)(rt - ert), "Blocked");
	if (rt == NULL) {
		DBFPRINTF("Parse failed\n");
		goto fail;
	}
	assert(rt <= ert);

	/* Pending ready list. */
	DBFPRINTF("Parsing xPendingReadyList\n");
	rt = gdb_freertos_parse_list(fs,
	    fs->fs_syms[GDB_FREERTOS_SYM_xPendingReadyList].sv_value, rt,
	    (unsigned int)(rt - ert), "Pending Ready");
	if (rt == NULL) {
		DBFPRINTF("Parse failed\n");
		goto fail;
	}
	assert(rt <= ert);

	/* Suspended task list is optional. */
	if (fs->fs_syms[GDB_FREERTOS_SYM_xSuspendedTaskList].sv_found) {
		DBFPRINTF("Parsing xSuspendedTaskList\n");
		rt = gdb_freertos_parse_list(fs,
		    fs->fs_syms[GDB_FREERTOS_SYM_xSuspendedTaskList].sv_value,
		    rt, (unsigned int)(rt - ert), "Suspended");
		if (rt == NULL) {
			DBFPRINTF("Parse failed\n");
			goto fail;
		}
		assert(rt <= ert);
	}

	/* Tasks Waiting Termination list is optional. */
	if (fs->fs_syms[GDB_FREERTOS_SYM_xTasksWaitingTermination].sv_found) {
		DBFPRINTF("Parsing xTasksWaitingTermination\n");
		rt = gdb_freertos_parse_list(fs,
		    fs->fs_syms[GDB_FREERTOS_SYM_xTasksWaitingTermination].sv_value,
		    rt, (unsigned int)(rt - ert), "Terminated");
		if (rt == NULL) {
			DBFPRINTF("Parse failed\n");
			goto fail;
		}
		assert(rt <= ert);
	}

	if (rt == srt) {
		/* This shouldn't happen. */
		zone_free(srt);
		DBFPRINTF("rt == srt!\n");
		return 0;
	}

	rv = (int)(rt - srt);

	if (rv != (int)ntasks) {
		DBFPRINTF("Expected %" PRIu32 " tasks, but got %d\n", ntasks,
		    rv);
	} else {
		DBFPRINTF("Found %d tasks\n", rv);
	}

	*rtp = srt;
	*current_thread = (target_addr_t)current_task;
	return rv;

 fail:
	zone_free(srt);
	return -1;
}

static target_addr_t
gdb_freertos_get_frame(void *cookie, target_addr_t tid, void **pparam)
{
	struct gdb_freertos_state *fs = cookie;
	uint32_t sp;

	if (gdb_freertos_read_pointer(fs, tid, &sp) == false ||
	    GDB_FREERTOS_BAD_POINTER(fs, sp)) {
		sp = 0;
	}

	*pparam = fs->fs_params;

	return (target_addr_t)sp;
}

const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt4 = {
	.rp_source = GDB_RTOS_PARAM_DEFAULT,
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int) .rp_val[e] = tt4,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
};

const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt2 = {
	.rp_source = GDB_RTOS_PARAM_DEFAULT,
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int) .rp_val[e] = tt2,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
};

const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt4_int = {
	.rp_source = GDB_RTOS_PARAM_DEFAULT,
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int) .rp_val[e] = tt4_int,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
};

const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt2_int = {
	.rp_source = GDB_RTOS_PARAM_DEFAULT,
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int) .rp_val[e] = tt2_int,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
};

static const struct gdb_rtos_param_desc gdb_freertos_param_descs[] = {
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int)	\
		GDB_RTOS_PARAM_DESC_INIT(desc, f),
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
};

static const struct gdb_rtos_symbol gdb_freertos_symbols[] = {
#define	GDB_RTOS_SYMBOL(e,n,f)	GDB_RTOS_SYMBOL_INIT(n,f),
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_SYMBOL
};

const struct gdb_rtos_backend gdb_rtos_backend_freertos = {
	.rb_name = "FreeRTOS",
	.rb_type = TARGET_RTOS_FREERTOS,
	.rb_nsymbols = GDB_RTOS_FREERTOS_NSYMBOLS,
	.rb_symbols = gdb_freertos_symbols,
	.rb_nparams = GDB_RTOS_FREERTOS_NPARAMS,
	.rb_params_size = sizeof(gdb_rtos_freertos_params_t),
	.rb_params_default =
	    (const gdb_rtos_params_t)(uintptr_t)&gdb_freertos_params_ilp32_tt4,
	.rb_params_desc = gdb_freertos_param_descs,
	.rb_attach = gdb_freertos_attach,
	.rb_free = gdb_freertos_free,
	.rb_reset = NULL,
	.rb_threads = gdb_freertos_threads,
	.rb_get_frame = gdb_freertos_get_frame,
};
