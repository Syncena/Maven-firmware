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

#define	__need_getopt_newlib
#include <assert.h>
#include <inttypes.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "rtos.h"
#include "tmon.h"
#include "gdb_server.h"
#include "gdb_signals.h"
#include "hardware.h"
#include "platform.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "shell.h"
#include "product.h"
#include "semihosting.h"
#include "swo_serve.h"
#include "sys_event.h"
#ifdef CONFIG_USE_DISPLAY
#include "display.h"
#endif
#include "timer.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "fsdata_custom.h"
#include "network_httpd.h"
#include "stringio.h"
#include "json_utils.h"
#endif
#endif /* CONFIG_USE_NETWORK */

#define _TARGET_PRIVATE
#include "target.h"

enum tmon_scan_type {
	TMON_SCAN_AUTO,
	TMON_SCAN_MANUAL,
	TMON_SCAN_ONCE,
};

struct tmon_syscall_handlers {
	tmon_target_syscall_handler_t sh_handler;
	void *sh_arg;
};

struct tmon_state {
	rtos_task_t ts_task;
	rtos_mutex_t ts_mutex;
	volatile tmon_cookie_t ts_base_target;
	unsigned int ts_num_targets;
	target_t *ts_targets;
	bool ts_last_step[CONFIG_MAX_TARGETS];

	target_addr_t ts_last_address;
	uint32_t ts_last_rd_value;
	uint32_t ts_last_wr_value;
	int ts_last_size;
	volatile bool ts_forced_rescan;
	volatile enum tmon_scan_type ts_scan_type;
	enum tmon_scan_type ts_scan_type_saved;

	struct timeval ts_last_ping;

	struct tmon_syscall_handlers ts_syscall_handlers[CONFIG_MAX_TARGETS];

	void *ts_swo;

	FILE *ts_log_fp;
	volatile bool ts_log_saw_output;
	volatile bool ts_log_complete;

#ifdef NETWORK_OPT_HTTPD
	void *ts_furl_cookie;
#endif
};
static struct tmon_state tmon_state;

#ifndef CONFIG_USE_CONFIGDB
#define	tmon_attach_reset	glob.glob_attach_reset
#else
static uint8_t tmon_attach_reset;
#endif

SHELL_CMD_DECL_CTX(info, tmon_cmd_info, "Display device details");
SHELL_CMD_DECL_CTX(reset, tmon_cmd_reset, "Resets target");
SHELL_CMD_DECL_CTX(protect, tmon_cmd_protect, "Query/Configure device protection");
SHELL_CMD_DECL_CTX(unlock, tmon_cmd_unlock, "Unlock a locked device");
SHELL_CMD_DECL(gdb_attach, tmon_cmd_attach, "Configures GDB attach behaviour");
SHELL_CMD_DECL_CTX(rescan, tmon_cmd_rescan, "Initiates target rescan");
SHELL_CMD_DECL(flash, tmon_cmd_flash, "Configures Flash verify behaviour");
#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL_CTX(memory, tmon_cmd_memory, "Access target memory");
#endif

static int tmon_handle_syscall(struct tmon_state *, tmon_cookie_t,
		union target_halt_state *, unsigned int);
static void tmon_reinit_syscall(struct tmon_state *, unsigned int);

static target_t
tmon_cookie_to_target(struct tmon_state *ts, tmon_cookie_t tc)
{

	tc -= ts->ts_base_target;

	return (tc < ts->ts_num_targets) ? ts->ts_targets[tc] : NULL;
}

static int
tmon_target_ctl(target_t t, uint32_t cmd, void *arg)
{
	int rv;

	rv = target_ctl(t, cmd, arg);

	if (rv >= 0)
		timer_get_mono_time(&tmon_state.ts_last_ping);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_get_target_name */
/* -------------------------------------------------------------------------- */
static int
tmon_get_target_name_impl(target_t t, struct tmon_get_target_name *tn)
{
	struct target_ctl_get_target_name ttn;

	if (tmon_target_ctl(t, TARGET_CTL_GET_TARGET_NAME, &ttn) < 0)
		return TMON_QN_ERROR;

	tn->tn_core_name = ttn.tn_core_name;
	tn->tn_vendor_name = ttn.tn_vendor_name;
	tn->tn_soc_name = ttn.tn_soc_name;
	tn->tn_soc_extra[0] = ttn.tn_soc_extra[0];
	tn->tn_soc_extra[1] = ttn.tn_soc_extra[1];

	return TMON_QN_OK;
}

int
tmon_get_target_name(tmon_cookie_t tc, struct tmon_get_target_name *tn)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_get_target_name_impl(t, tn);
	else
		rv = TMON_QN_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_query_attach */
/* -------------------------------------------------------------------------- */
static int
tmon_query_attach_impl(target_t t, int do_attach)
{
	int rv;

	if (tmon_target_ctl(t, TARGET_CTL_QUERY_ATTACH, &rv) < 0)
		return TMON_QA_ERROR;

	if (rv)
		return TMON_QA_ATTACHED;

	if (do_attach == 0)
		return TMON_QA_DETACHED;

	rv = (int)tmon_attach_reset;
	if (tmon_target_ctl(t, TARGET_CTL_ATTACH, &rv) < 0)
		return TMON_QA_ERROR;

	switch (rv) {
	case 0:
		rv = TMON_QA_TIMEOUT;
		break;
	case 1:
		rv = TMON_QA_ATTACHED;
		break;
	default:
		rv = TMON_QA_ERROR;
		break;
	}

	return rv;
}

int
tmon_query_attach(tmon_cookie_t *tc, unsigned int target_idx, int do_attach)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, *tc)) == NULL) {
		if (target_idx >= ts->ts_num_targets || do_attach == 0) {
			rv = TMON_QA_ERROR;
			goto out;
		}

		t = ts->ts_targets[target_idx];
		*tc = ~0u;
	}

	rv = tmon_query_attach_impl(t, do_attach);
	if (rv >= 0) {
		if (do_attach)
			*tc = ts->ts_base_target + target_idx;
	} else {
		rv = TMON_QA_ERROR;
	}

 out:
	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_detach */
/* -------------------------------------------------------------------------- */
static int
tmon_detach_impl(target_t t)
{

	if (tmon_target_ctl(t, TARGET_CTL_DETACH, NULL) < 0)
		return TMON_DT_ERROR;

	return TMON_DT_DETACHED;
}

int
tmon_detach(tmon_cookie_t tc)
{
	struct tmon_state *ts = &tmon_state;
	unsigned int idx;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
		idx = t->t_index;
		rv = tmon_detach_impl(t);
	} else {
		rv = TMON_DT_ERROR;
	}

	rtos_mutex_release(ts->ts_mutex);

	if (rv == TMON_DT_DETACHED)
		tmon_reinit_syscall(ts, idx);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_halt_status */
/* -------------------------------------------------------------------------- */
static int
tmon_halt_status_impl(target_t t, union target_halt_state *ths, int just_query)
{
	int rv;

	rv = tmon_target_ctl(t, just_query ? TARGET_CTL_HALT_QUERY :
	    TARGET_CTL_HALT_POLL, ths);

	switch (rv) {
	case TARGET_HALT_STATE_NOT_ATTACHED:
		rv = TMON_HS_DETACHED;
		break;
	case TARGET_HALT_STATE_FAULT:
		rv = TMON_HS_FAULT;
		break;
	case TARGET_HALT_STATE_BREAKPOINT:
		rv = TMON_HS_BREAKPOINT;
		break;
	case TARGET_HALT_STATE_WATCHPOINT:
		rv = TMON_HS_WATCHPOINT;
		break;
	case TARGET_HALT_STATE_FORCED:
		rv = TMON_HS_FORCED;
		ths->ths_halt.th_type = GDB_SIGNAL_INT;
		break;
	case TARGET_HALT_STATE_SINGLE_STEP:
		rv = TMON_HS_SINGLE_STEP;
		break;
	case TARGET_HALT_STATE_RUNNING:
		rv = TMON_HS_RUNNING;
		break;
	case TARGET_HALT_STATE_SYSCALL:
		rv = TMON_HS_SYSCALL;
		break;
	default:
		rv = TMON_HS_ERROR;
		break;
	}

	return rv;
}

int
tmon_halt_status(tmon_cookie_t tc, struct tmon_watch *tw, int just_query)
{
	struct tmon_state *ts = &tmon_state;
	union target_halt_state ths;
	unsigned int idx = 0;
	target_t t;
	int rv;

	do {
		rtos_mutex_acquire(ts->ts_mutex);

		if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
			rv = tmon_halt_status_impl(t, &ths, just_query);
			idx = t->t_index;
		} else {
			rv = TMON_HS_ERROR;
		}

		rtos_mutex_release(ts->ts_mutex);

		switch (rv) {
		case TMON_HS_BREAKPOINT:
		case TMON_HS_WATCHPOINT:
			tw->tw_type = ths.ths_halt.th_type;
			tw->tw_addr = ths.ths_halt.th_addr;
			break;

		case TMON_HS_SYSCALL:
			/*
			 * Target system calls are handled here *after* we've
			 * released the tmon mutex. They may block for an
			 * indeterminite period of time so best not hold
			 * things up.
			 *
			 * The fly in the ointment with this is that we must
			 * take care to deal with the target disappearing in
			 * the middle of things.
			 *
			 * System calls are also invisible to our caller; as
			 * far as they're concerned, the target is running
			 * normally.
			 */
			if (just_query) {
				rv = TMON_HS_RUNNING;
			} else {
				rv = tmon_handle_syscall(ts, tc, &ths, idx);
				if (rv != TMON_HS_RUNNING)
					tw->tw_type = ths.ths_halt.th_type;
			}
			break;

		case TMON_HS_FORCED:
			/* Some kind of signal. */
			tw->tw_type = ths.ths_halt.th_type;
			break;

		default:
			break;
		}
	} while (rv == TMON_HS_SYSCALL);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_halt */
/* -------------------------------------------------------------------------- */
static int
tmon_halt_impl(target_t t)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_HALT, NULL);

	return rv;
}

int
tmon_halt(tmon_cookie_t tc)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_halt_impl(t);
	else
		rv = TMON_HS_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_resume */
/* -------------------------------------------------------------------------- */
static int
tmon_resume_impl(target_t t, struct target_resume *tr)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_RESUME, tr);

	if (rv < 0)
		return TMON_RS_ERROR;

	return rv ? TMON_RS_OK : TMON_RS_DETACHED;
}

int
tmon_resume(tmon_cookie_t tc, bool step, const target_addr_t *addr)
{
	struct tmon_state *ts = &tmon_state;
	struct target_resume tr;
	target_t t;
	int rv;

	tr.tr_flags = step ? TARGET_RESUME_FLAG_SINGLE_STEP : 0;
	if (addr != NULL) {
		tr.tr_flags |= TARGET_RESUME_FLAG_ADDR_VALID;
		tr.tr_addr = *addr;
	}

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
		assert(t->t_index < CONFIG_MAX_TARGETS);
		ts->ts_last_step[t->t_index] = step;
		rv = tmon_resume_impl(t, &tr);
	} else {
		rv = TMON_RS_ERROR;
	}

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_reset */
/* -------------------------------------------------------------------------- */
static int
tmon_reset_impl(target_t t, int reset_type)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_RESET, &reset_type);

	if (rv < 0)
		return TMON_RT_ERROR;
	else
	if (rv == 1)
		return TMON_RT_RESCAN;

	return rv;
}

int
tmon_reset(tmon_cookie_t tc, int reset_type)
{
	struct tmon_state *ts = &tmon_state;
	tmon_cookie_t old;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	old = ts->ts_base_target;

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_reset_impl(t, reset_type);
	else
		rv = TMON_RT_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	/*
	 * If the target is likely to have disappeared (because we
	 * hit h/w reset) then wait around for the tmon task to notice.
	 */
	if (reset_type == TARGET_RESET_HARD || rv == TMON_RT_RESCAN) {
		int retry = 30;

		while (retry--) {
			if (old != ts->ts_base_target)
				break;
			rtos_task_sleep(100);
		}
	}
	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_read_regs */
/* -------------------------------------------------------------------------- */
static int
tmon_read_regs_impl(target_t t, void *regs)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_READ_REGS, regs);

	if (rv < 0)
		return TMON_RR_ERROR;

	return rv;
}

int
tmon_read_regs(tmon_cookie_t tc, void *regs)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_read_regs_impl(t, regs);
	else
		rv = TMON_RR_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_write_regs */
/* -------------------------------------------------------------------------- */
static int
tmon_write_regs_impl(target_t t, void *regs)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_WRITE_REGS, regs);

	if (rv < 0)
		return TMON_WR_ERROR;

	return TMON_WR_OK;
}

int
tmon_write_regs(tmon_cookie_t tc, void *regs)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_write_regs_impl(t, regs);
	else
		rv = TMON_WR_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_read_one_reg */
/* -------------------------------------------------------------------------- */
static int
tmon_read_one_reg_impl(target_t t, int reg, void *regbuff)
{
	struct target_rdwr_one_reg or;
	int rv;

	or.or_reg = reg;
	or.or_regbuff = regbuff;

	rv = tmon_target_ctl(t, TARGET_CTL_READ_ONE_REG, &or);

	if (rv < 0)
		return TMON_RO_ERROR;

	return rv;
}

int
tmon_read_one_reg(tmon_cookie_t tc, int reg, void *regbuff)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_read_one_reg_impl(t, reg, regbuff);
	else
		rv = TMON_RO_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_write_one_reg */
/* -------------------------------------------------------------------------- */
static int
tmon_write_one_reg_impl(target_t t, int reg, void *regbuff)
{
	struct target_rdwr_one_reg or;
	int rv;

	or.or_reg = reg;
	or.or_regbuff = regbuff;

	rv = tmon_target_ctl(t, TARGET_CTL_WRITE_ONE_REG, &or);

	if (rv < 0)
		return TMON_WO_ERROR;

	return TMON_WO_OK;
}

int
tmon_write_one_reg(tmon_cookie_t tc, int reg, void *regbuff)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_write_one_reg_impl(t, reg, regbuff);
	else
		rv = TMON_WO_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_breakwatch */
/* -------------------------------------------------------------------------- */
static int
tmon_breakwatch_impl(target_t t, const struct tmon_breakwatch *tb, int set)
{
	struct target_watchbreak tw;
	int rv;

	tw.tw_set = set;
	tw.tw_addr = tb->tb_address;
	tw.tw_type = tb->tb_type;
	tw.tw_kind = tb->tb_kind;

	rv = tmon_target_ctl(t, TARGET_CTL_WATCHBREAK, &tw);

	switch (rv) {
	case TARGET_BKWP_STATUS_OK:
		rv = TMON_BW_OK;
		break;
	case TARGET_BKWP_STATUS_UNSUPPORTED:
		rv = TMON_BW_UNSUPPORTED;
		break;
	default:
		rv = TMON_BW_ERROR;
		break;
	}

	return rv;
}

int
tmon_breakwatch(tmon_cookie_t tc, const struct tmon_breakwatch *tb, int set)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_breakwatch_impl(t, tb, set);
	else
		rv = TMON_BW_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}
/* -------------------------------------------------------------------------- */
/* tmon_mem_read */
/* -------------------------------------------------------------------------- */
static int
tmon_mem_read_impl(target_t t, target_addr_t addr, uint32_t len, void *dest)
{
	struct target_mem_readwrite mr;
	int rv;

	mr.mr_write = false;
	mr.mr_target_addr = addr;
	mr.mr_length = len;
	mr.mr_dest = dest;

	rv = tmon_target_ctl(t, TARGET_CTL_MEM_READWRITE, &mr);

	if (rv < 0)
		return TMON_MR_ERROR;

	return TMON_MR_OK;
}

int
tmon_mem_read(tmon_cookie_t tc, target_addr_t addr, uint32_t len, void *dest)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_mem_read_impl(t, addr, len, dest);
	else
		rv = TMON_MR_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_mem_write */
/* -------------------------------------------------------------------------- */
static int
tmon_mem_write_impl(target_t t, target_addr_t addr, uint32_t len,
    const void *src)
{
	struct target_mem_readwrite mr;
	int rv;

	mr.mr_write = true;
	mr.mr_target_addr = addr;
	mr.mr_length = len;
	mr.mr_src = src;

	rv = tmon_target_ctl(t, TARGET_CTL_MEM_READWRITE, &mr);

	if (rv < 0)
		return TMON_MW_ERROR;

	return TMON_MW_OK;
}

int
tmon_mem_write(tmon_cookie_t tc, target_addr_t addr, uint32_t len,
    const void *src)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_mem_write_impl(t, addr, len, src);
	else
		rv = TMON_MW_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_get_xml_regs */
/* -------------------------------------------------------------------------- */
static int
tmon_get_xml_regs_impl(target_t t, stringio_t s)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_GET_XML_REGS, s);

	if (rv < 0)
		return TMON_XR_ERROR;

	return rv;
}

int
tmon_get_xml_regs(tmon_cookie_t tc, stringio_t s)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_get_xml_regs_impl(t, s);
	else
		rv = TMON_XR_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_get_xml_memory */
/* -------------------------------------------------------------------------- */
static int
tmon_get_xml_memory_impl(target_t t, stringio_t s)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_GET_XML_MEMORY, s);

	if (rv < 0)
		return TMON_XM_ERROR;

	return TMON_XM_OK;
}

int
tmon_get_xml_memory(tmon_cookie_t tc, stringio_t s)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_get_xml_memory_impl(t, s);
	else
		rv = TMON_XM_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_flash_erase */
/* -------------------------------------------------------------------------- */
static int
tmon_flash_erase_impl(target_t t, target_addr_t addr, uint32_t len)
{
	struct target_mem_readwrite mr;
	int rv;

	mr.mr_target_addr = addr;
	mr.mr_length = len;

	rv = tmon_target_ctl(t, TARGET_CTL_FLASH_ERASE, &mr);

	if (rv < 0)
		return TMON_FL_ERROR;

	return TMON_FL_OK;
}

int
tmon_flash_erase(tmon_cookie_t tc, target_addr_t addr, uint32_t len)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_flash_erase_impl(t, addr, len);
	else
		rv = TMON_FL_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_flash_write */
/* -------------------------------------------------------------------------- */
static int
tmon_flash_write_impl(target_t t, target_addr_t addr, uint32_t len,
    const void *src)
{
	struct target_mem_readwrite mr;
	int rv;

	mr.mr_target_addr = addr;
	mr.mr_length = len;
	mr.mr_src = src;

	rv = tmon_target_ctl(t, TARGET_CTL_FLASH_WRITE, &mr);

	if (rv < 0)
		return TMON_FL_ERROR;

	return TMON_FL_OK;
}

int
tmon_flash_write(tmon_cookie_t tc, target_addr_t addr, uint32_t len,
    const void *src)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_flash_write_impl(t, addr, len, src);
	else
		rv = TMON_FL_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_flash_done */
/* -------------------------------------------------------------------------- */
static int
tmon_flash_done_impl(target_t t)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_FLASH_DONE, NULL);

	if (rv < 0)
		return TMON_FL_ERROR;

	return TMON_FL_OK;
}

int
tmon_flash_done(tmon_cookie_t tc)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_flash_done_impl(t);
	else
		rv = TMON_FL_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_target_info */
/* -------------------------------------------------------------------------- */
static int
tmon_target_info_impl(target_t t, struct target_info *ti)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_TARGET_INFO, ti);

	if (rv < 0)
		return TMON_TI_ERROR;

	return TMON_TI_OK;
}

int
tmon_target_info(tmon_cookie_t tc, struct target_info *ti)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_target_info_impl(t, ti);
	else
		rv = TMON_TI_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_rtos_set_frame */
/* -------------------------------------------------------------------------- */
static int
tmon_rtos_set_frame_impl(target_t t, struct target_rtos *tr)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_SET_RTOS_FRAME, tr);

	if (rv < 0)
		return TMON_RSF_ERROR;

	return TMON_RSF_OK;
}

int
tmon_rtos_set_frame(tmon_cookie_t tc, target_rtos_type_t rtos,
    target_addr_t frame, void *params)
{
	struct tmon_state *ts = &tmon_state;
	struct target_rtos tr;
	target_t t;
	int rv;

	tr.tr_rtos = rtos;
	tr.tr_frame = frame;
	tr.tr_params = params;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_rtos_set_frame_impl(t, &tr);
	else
		rv = TMON_RSF_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_rtos_load_params */
/* -------------------------------------------------------------------------- */
static int
tmon_rtos_load_params_impl(target_t t, struct target_rtos_params *rp)
{
	int rv;

	rv = tmon_target_ctl(t, TARGET_CTL_LOAD_RTOS_PARAMS, rp);

	if (rv < 0)
		return TMON_RLP_ERROR;

	return TMON_RLP_OK;
}

int
tmon_rtos_load_params(tmon_cookie_t tc, target_rtos_type_t rtos,
    void *params)
{
	struct tmon_state *ts = &tmon_state;
	struct target_rtos_params rp;
	target_t t;
	int rv;

	rp.rp_rtos = rtos;
	rp.rp_params = params;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL)
		rv = tmon_rtos_load_params_impl(t, &rp);
	else
		rv = TMON_RLP_ERROR;

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_target_protection_query (internal use only)                           */
/* -------------------------------------------------------------------------- */
static int
tmon_target_protection_query_impl(target_t t, struct target_protection_info *pi)
{

	return tmon_target_ctl(t, TARGET_CTL_PROTECTION_QUERY, pi);
}

/* -------------------------------------------------------------------------- */
/* tmon_target_protection_set (internal use only)                             */
/* -------------------------------------------------------------------------- */
static int
tmon_target_protection_set_impl(target_t t, struct target_protection_info *pi)
{

	return tmon_target_ctl(t, TARGET_CTL_PROTECTION_SET, pi);
}

/* -------------------------------------------------------------------------- */
/* tmon_get_link_error							      */
/* -------------------------------------------------------------------------- */
static void
tmon_get_link_error_impl(target_t t, uint32_t *dperr)
{

	(void) tmon_target_ctl(t, TARGET_CTL_GET_LINK_ERROR, dperr);
}

int
tmon_get_link_error(tmon_cookie_t tc, uint32_t *dperr)
{
	struct tmon_state *ts = &tmon_state;
	target_t t;
	int rv;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
		tmon_get_link_error_impl(t, dperr);
		rv = TMON_LINK_OK;
	} else {
		rv = TMON_LINK_ERROR;
	}

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

/* -------------------------------------------------------------------------- */
/* tmon_target_unlock							      */
/* -------------------------------------------------------------------------- */
static int
tmon_target_unlock_impl(target_t t, FILE *fp)
{

	return tmon_target_ctl(t, TARGET_CTL_UNLOCK, fp);
}

/* -------------------------------------------------------------------------- */
/* tmon_rescan */
/* -------------------------------------------------------------------------- */
static void
tmon_rescan_impl(struct tmon_state *ts)
{
	unsigned int x = ts->ts_num_targets;

	ts->ts_base_target += ts->ts_num_targets;
	ts->ts_num_targets = 0;
	ts->ts_targets = NULL;
	ts->ts_forced_rescan = false;

	for (unsigned int i = 0; i < CONFIG_MAX_TARGETS; i++)
		ts->ts_last_step[i] = false;

	if (x != 0)
		sys_event_notify(SYS_EVENT_TMON_DISCONNECT, NULL);

	target_cleanup();
}

int
tmon_rescan(void)
{
	struct tmon_state *ts = &tmon_state;

	rtos_mutex_acquire(ts->ts_mutex);

	tmon_rescan_impl(ts);

	rtos_mutex_release(ts->ts_mutex);

	return TMON_TR_OK;
}

unsigned int
tmon_get_target_count(void)
{
	struct tmon_state *ts = &tmon_state;

	return ts->ts_num_targets;
}

uintptr_t
tmon_locked_callback(uintptr_t (*cb)(void *), void *arg)
{
	struct tmon_state *ts = &tmon_state;
	uintptr_t rv;

	rtos_mutex_acquire(ts->ts_mutex);

	rv = (cb)(arg);

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

const char *
tmon_status(uint32_t *p)
{

	(void) p;
	return NULL;
}

static void
tmon_reinit_syscall(struct tmon_state *ts, unsigned int idx)
{
	struct tmon_syscall_handlers *sh;

	sh = &ts->ts_syscall_handlers[idx];

	if (sh->sh_handler != NULL) {
		/* Reinitialise the syscall interface. */
		(void) (sh->sh_handler)(sh->sh_arg, 0, NULL, NULL);
	}
}

static int
tmon_handle_syscall(struct tmon_state *ts, tmon_cookie_t tc,
    union target_halt_state *ths, unsigned int idx)
{
	struct target_syscall *sc = &ths->ths_syscall;
	struct tmon_syscall_handlers *sh = &ts->ts_syscall_handlers[idx];
	struct target_resume tr;
	uint32_t retval;
	target_t t;
	int rv;

	/*
	 * Is a syscall handler registered?
	 */
	if (sh->sh_handler == NULL) {
		ths->ths_halt.th_type = GDB_SIGNAL_ILL;
		return TMON_HS_FORCED;
	}

	/*
	 * Invoke it.
	 * It will read the args from 'sc', optionally updating sc->sc_pc,
	 * and write the return value into tr.tr_rv;
	 */
	rv = (sh->sh_handler)(sh->sh_arg, tc, sc, &retval);

	switch (rv) {
	case TMON_HS_RUNNING:
		/* Syscall processed normally. */
		tr.tr_rv = retval;
		break;

	case TMON_HS_EXITED:
		/*
		 * Target has executed its exit(2) syscall. 'retval'
		 * holds its exit value. Our caller expects this in
		 * ths->ths_halt.th_type.
		 */
		ths->ths_halt.th_type = retval;
		return rv;

	case TMON_HS_FORCED:
		/*
		 * User has interrupted using CTRL-C, or STOP.
		 */
		ths->ths_halt.th_type = GDB_SIGNAL_INT;
		return rv;

	default:
		/* Anything else is pretty much fatal. */
		ths->ths_halt.th_type = GDB_SIGNAL_TRAP;
		return rv;
	}

	/*
	 * We need to resume at the call site, or at the updated PC (for
	 * example, PC will have been adjusted to skip the BKPT instruction
	 * on Cortex-M).
	 */
	tr.tr_flags = TARGET_RESUME_FLAG_ADDR_VALID |
	    TARGET_RESUME_FLAG_RV_VALID;
	tr.tr_addr = sc->ts_pc;

	rtos_mutex_acquire(ts->ts_mutex);

	/* Now resume the target. */
	if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
		/*
		 * Ensure we use the same step/continue parameter
		 * from last time - remember that syscalls are not
		 * visible to upper layers.
		 */
		if (ts->ts_last_step[t->t_index])
			tr.tr_flags |= TARGET_RESUME_FLAG_SINGLE_STEP;

		if (tmon_resume_impl(t, &tr) != TMON_RS_OK)
			rv = TMON_HS_ERROR;
	} else {
		rv = TMON_HS_ERROR;
	}

	rtos_mutex_release(ts->ts_mutex);

	return rv;
}

void *
tmon_register_syscall_handler(tmon_cookie_t tc,
	tmon_target_syscall_handler_t handler, void *cookie)
{
	struct tmon_state *ts = &tmon_state;
	struct tmon_syscall_handlers *sh;
	target_t t;

	rtos_mutex_acquire(ts->ts_mutex);

	if ((t = tmon_cookie_to_target(ts, tc)) != NULL) {
		assert(t->t_index < CONFIG_MAX_TARGETS);
		sh = &ts->ts_syscall_handlers[t->t_index];
		sh->sh_handler = handler;
		sh->sh_arg = cookie;
	} else {
		sh = NULL;
	}

	rtos_mutex_release(ts->ts_mutex);

	return sh;
}

void
tmon_delete_syscall_handler(void *arg)
{
	struct tmon_state *ts = &tmon_state;
	struct tmon_syscall_handlers *sh = arg;

	rtos_mutex_acquire(ts->ts_mutex);
	sh->sh_handler = NULL;
	rtos_mutex_release(ts->ts_mutex);
}

#ifdef NETWORK_OPT_HTTPD
static void
tmon_update_target_json(struct tmon_get_target_name *tn)
{
	const char *json;
	stringio_t ss;
	char tvcc[8];

	if (tn == NULL) {
		fsdata_delete("/target.json");
		network_httpd_update_ping(NETWORK_HTTPD_PING_TARGET);
		return;
	}

	if ((ss = stringio_new()) == NULL)
		return;

	stringio_printf(ss, JSON_OBJECT_START);

	stringio_printf(ss, JSON_STRING("vendor",
	    (tn->tn_vendor_name != NULL) ? tn->tn_vendor_name : "Unknown"));
	stringio_printf(ss, JSON_SEPARATOR);

	stringio_printf(ss, JSON_STRING("soc",
	    (tn->tn_soc_name != NULL) ? tn->tn_soc_name : "Unknown"));
	stringio_printf(ss, JSON_SEPARATOR);

	stringio_printf(ss, JSON_STRING("extra0",
	    (tn->tn_soc_extra[0] != NULL) ? tn->tn_soc_extra[0] : ""));
	stringio_printf(ss, JSON_SEPARATOR);

	stringio_printf(ss, JSON_STRING("extra1",
	    (tn->tn_soc_extra[1] != NULL) ? tn->tn_soc_extra[1] : ""));
	stringio_printf(ss, JSON_SEPARATOR);

	fix16_to_str(power_target_vcc, tvcc, 2);
	stringio_printf(ss, JSON_STRING("tvcc", tvcc));
	stringio_printf(ss, JSON_SEPARATOR);

	target_json_all(ss);

	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL) {
		fsdata_create("/target.json", json, strlen(json), true);
		network_httpd_update_ping(NETWORK_HTTPD_PING_TARGET);
	}

	stringio_done(ss);
}

static void
tmon_debug_update(void)
{
	const char *json;
	stringio_t ss;
	uint8_t auto_attach;

	if ((ss = stringio_new()) == NULL)
		return;

	stringio_printf(ss, JSON_OBJECT_START);

	stringio_printf(ss, JSON_STRING("attach_reset", tmon_attach_reset ?
	    "reset" : "halt"));

	stringio_printf(ss, JSON_SEPARATOR);

	if (cf_get_uint8(CF_KEY_UINT8_ATTACH_AUTO, &auto_attach) == 0)
		auto_attach = 1;
	stringio_printf(ss, JSON_STRING("attach_auto", auto_attach ?
	    "auto" : "manual"));

	target_debug_update(ss);

	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL) {
		fsdata_create("/debug.json", json, strlen(json), true);
		network_httpd_update_ping(NETWORK_HTTPD_PING_DEBUG);
	}

	stringio_done(ss);
}

static const char *
tmon_debug_post_handler(void *arg, int nkeys,
    const struct network_http_furl_key_value *kv)
{
	int i, val, changed;
	uint8_t auto_attach;

	(void) arg;

	if (cf_get_uint8(CF_KEY_UINT8_ATTACH_AUTO, &auto_attach) == 0)
		auto_attach = 1;

	changed = target_debug_update_settings(nkeys, kv);

	for (i = 0; i < nkeys; i++) {
		if (strcmp(kv[i].kv_key, "gdb-attach-reset") == 0 &&
		    sscanf(kv[i].kv_value, "%d", &val) == 1 &&
		    (val == 0 || val == 1) &&
		    tmon_attach_reset != (uint8_t)val) {
			tmon_attach_reset = (uint8_t)val;
			cf_set_uint8(CF_KEY_UINT8_ATTACH_RESET,
			    tmon_attach_reset);
			changed++;
			break;
		}

		if (strcmp(kv[i].kv_key, "gdb-attach-auto") == 0 &&
		    sscanf(kv[i].kv_value, "%d", &val) == 1 &&
		    (val == 0 || val == 1) &&
		    auto_attach != (uint8_t)val) {
			auto_attach = (uint8_t)val;
			cf_set_uint8(CF_KEY_UINT8_ATTACH_AUTO, auto_attach);
			changed++;
			break;
		}
	}

	if (changed) {
		configdb_persist();

		/* XXX: Watch stack depth here... */
		tmon_debug_update();
	}

	return "/saved.txt";
}
#endif /* NETWORK_OPT_HTTPD */

bool
tmon_attach_will_reset(void)
{

	return tmon_attach_reset != 0;
}

void
tmon_task(rtos_task_t task)
{
	struct tmon_state *ts = &tmon_state;
	struct tmon_get_target_name tn;
	struct timeval t_now, t_diff;
	int rv, prot_cmd, unlock_cmd;
	unsigned int num_targets;
	target_t t0;

	ts->ts_task = task;

	SHELL_CMD_ADD(gdb_attach);
	SHELL_CMD_ADD_CTX(rescan, ts);
#if (RELEASE_BUILD == 0)
	SHELL_CMD_ADD_CTX(info, ts);
#endif
	SHELL_CMD_ADD(flash);

	gdb_server_init();

#ifdef NETWORK_OPT_HTTPD
	ts->ts_furl_cookie = network_httpd_add_post_handler_furl(
	    tmon_debug_post_handler, ts, "/debug.cgi");
	assert(ts->ts_furl_cookie != NULL);
#endif

	rtos_task_priority_set(task, RTOS_TASK_PRIORITY_MED - 1);
	rtos_task_sleep(1000);

#ifdef NETWORK_OPT_HTTPD
	tmon_debug_update();
#endif

	for (;;) {
		num_targets = 0;
#ifdef NETWORK_OPT_HTTPD
		tmon_update_target_json(NULL);
#endif

		do {
			if (POWER_TARGET_VCC_GOOD) {
				rtos_mutex_acquire(ts->ts_mutex);

				if (ts->ts_scan_type != TMON_SCAN_MANUAL) {
					sys_event_notify(
					    SYS_EVENT_TMON_SCANNING, NULL);

					ts->ts_targets =
					    target_probe(&num_targets);

					if (ts->ts_log_saw_output &&
					    ts->ts_log_fp != NULL) {
						fflush(ts->ts_log_fp);
						ts->ts_log_fp = NULL;
						ts->ts_log_saw_output = false;
						ts->ts_log_complete = true;
					}

					if (ts->ts_scan_type == TMON_SCAN_ONCE){
						ts->ts_scan_type =
						    ts->ts_scan_type_saved;
					}
				}

				rtos_mutex_release(ts->ts_mutex);
			}

			if (ts->ts_targets == NULL)
				rtos_task_sleep(467);
		} while (ts->ts_targets == NULL);

		assert(num_targets != 0);
		t0 = ts->ts_targets[0];

		unlock_cmd = (t0->t_flags & TARGET_FLAG_IS_LOCKED) != 0;
		memset(&tn, 0, sizeof(tn));

		rtos_mutex_acquire(ts->ts_mutex);
		rv = tmon_get_target_name_impl(t0, &tn);
		rtos_mutex_release(ts->ts_mutex);

		if (rv == TMON_QN_OK) {
			if (tn.tn_soc_extra[1] == NULL && unlock_cmd) {
				tn.tn_soc_extra[1] = "** LOCKED **";
				target_register_name(NULL, NULL, NULL,
				    tn.tn_soc_extra[1]);
			}

#ifdef CONFIG_USE_DISPLAY
			if (tn.tn_vendor_name != NULL) {
				assert(tn.tn_soc_name != NULL);
				fprintf(display_stream, "\f%s\n%s\n",
				    tn.tn_vendor_name, tn.tn_soc_name);
				if (tn.tn_soc_extra[0] != NULL) {
					fprintf(display_stream, "%s\n",
					    tn.tn_soc_extra[0]);
				}
				if (tn.tn_soc_extra[1] != NULL) {
					fprintf(display_stream, "%s\n",
					    tn.tn_soc_extra[1]);
				}
			} else
			if (tn.tn_core_name != NULL) {
				fprintf(display_stream, "%s\n",
				    tn.tn_core_name);
			}
#endif /* CONFIG_USE_DISPLAY */
		}

#ifdef NETWORK_OPT_HTTPD
		tmon_update_target_json(&tn);
#endif

#if (RELEASE_BUILD != 0)
		SHELL_CMD_ADD_CTX(info, ts);
#endif
		SHELL_CMD_ADD_CTX(reset, ts);
		ts->ts_forced_rescan = false;

		prot_cmd = (t0->t_flags & TARGET_FLAG_SUPPORTS_PROTECTION) != 0;
		if (prot_cmd)
			SHELL_CMD_ADD_CTX(protect, ts);

		if (unlock_cmd)
			SHELL_CMD_ADD_CTX(unlock, ts);

#if (RELEASE_BUILD == 0)
		SHELL_CMD_ADD_CTX(memory, ts);
#endif

		timer_get_mono_time(&ts->ts_last_ping);

		ts->ts_num_targets = num_targets;

		sys_event_notify(SYS_EVENT_TMON_ATTACHED, &tn);

		do {
			rtos_saved_ipl_t ipl;

			rtos_task_sleep(163);

			timer_get_mono_time(&t_now);
			ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
			timer_sub(&t_now, &ts->ts_last_ping, &t_diff);
			rtos_ipl_restore(ipl);

			rv = 1;
			if (t_diff.tv_sec || t_diff.tv_usec > 467000 ||
			    ts->ts_forced_rescan) {
				t0 = NULL;
				rtos_mutex_acquire(ts->ts_mutex);
				if (ts->ts_targets != NULL &&
				    (t0 = ts->ts_targets[0]) != NULL) {
					rv = TARGET_LINK_PING(t0);
					if (rv < 0 || ts->ts_forced_rescan) {
						t0 = NULL;
						tmon_rescan_impl(ts);
					} else {
						timer_get_mono_time(
						    &ts->ts_last_ping);
					}
				}
				rtos_mutex_release(ts->ts_mutex);
			}
		} while (t0 != NULL && rv >= 0);

#if (RELEASE_BUILD == 0)
		SHELL_CMD_DEL(memory);
#endif
		if (unlock_cmd)
			SHELL_CMD_DEL(unlock);
		if (prot_cmd)
			SHELL_CMD_DEL(protect);
#if (RELEASE_BUILD != 0)
		SHELL_CMD_DEL(info);
#endif
		SHELL_CMD_DEL(reset);

#ifdef CONFIG_USE_DISPLAY
		fputc('\f', display_stream);
#endif
	}
}

void
tmon_init(void)
{
	struct tmon_state *ts = &tmon_state;
	uint8_t st;

	ts->ts_mutex = rtos_mutex_create();
	assert(ts->ts_mutex != NULL);

	ts->ts_base_target = 0;
	ts->ts_num_targets = 0;
	ts->ts_targets = NULL;
	for (unsigned int i = 0; i < CONFIG_MAX_TARGETS; i++)
		ts->ts_last_step[i] = false;
	ts->ts_last_address = 0;
	ts->ts_last_size = sizeof(uint32_t);
	ts->ts_last_rd_value = 1;
	ts->ts_last_wr_value = 0;
	ts->ts_forced_rescan = false;
	timer_get_mono_time(&ts->ts_last_ping);

#ifndef CONFIG_USE_CONFIGDB
	st = glob.glob_scan_type;
#else
	cf_get_uint8(CF_KEY_UINT8_ATTACH_RESET, &tmon_attach_reset);

	if (cf_get_uint8(CF_KEY_UINT8_SCAN_TYPE, &st) == 0)
		st = (uint8_t) TMON_SCAN_AUTO;
#endif
	ts->ts_scan_type = ts->ts_scan_type_saved = (enum tmon_scan_type)st;

	target_init();
}

void
tmon_load_defaults(void)
{
	const int attach_reset = 1;

#ifndef CONFIG_USE_CONFIGDB
	glob.glob_db_type = TMON_DP_TYPE_SWDP;	/* XXX: Unused */
	glob.glob_attach_reset = attach_reset;
	glob.glob_scan_type = (uint8_t) TMON_SCAN_AUTO;
	glob.glob_flash_verify = 1;
#else
	cf_set_uint8(CF_KEY_UINT8_ATTACH_RESET, (uint8_t)attach_reset);
	cf_set_uint8(CF_KEY_UINT8_ATTACH_AUTO, 1u);
	cf_del_uint8(CF_KEY_UINT8_SCAN_TYPE);
	cf_del_uint8(CF_KEY_UINT8_FLASH_VERIFY);
#endif

	gdb_server_load_defaults();
	target_load_defaults();
}

static void
tmon_cmd_info(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	struct tmon_get_target_name tn;
	char vcc[16];
	target_t t;
	int rv;

	(void) argc;
	(void) argv;

	rtos_mutex_acquire(ts->ts_mutex);

	if (ts->ts_targets != NULL && (t = ts->ts_targets[0]) != NULL)
		rv = tmon_get_target_name_impl(t, &tn);
	else
		rv = TMON_QN_ERROR;

	if (rv != TMON_QN_OK)
		memset(&tn, 0, sizeof(tn));

	fprintf(os, "Vendor: %s\n",
	    (tn.tn_vendor_name != NULL) ? tn.tn_vendor_name : "Unknown");
	fprintf(os, "Device: %s\n",
	    (tn.tn_soc_name != NULL) ? tn.tn_soc_name : "Unknown");
	if (tn.tn_soc_extra[0] != NULL) {
		fprintf(os, "Additional Info: %s", tn.tn_soc_extra[0]);
		if (tn.tn_soc_extra[1] != NULL)
			fprintf(os, ", %s\n", tn.tn_soc_extra[1]);
		else
			fputc('\n', os);
	}

	target_info_all(os);

	rtos_mutex_release(ts->ts_mutex);

	fputs("Power supply voltages:\n", os);

	fix16_to_str(power_target_vcc, vcc, 2);
	fprintf(os, "Target: %s volts\n", vcc);

	fix16_to_str(power_vddio, vcc, 2);
	fprintf(os, "Maven: %s volts%s\n", vcc,
	    (power_vddio_state == 0) ? " <-- WARNING - Too Low!" : "");

#if defined(POWER_VBUS_AVAILABLE) && (POWER_VBUS_AVAILABLE != 0)
	fix16_to_str(power_vbus(), vcc, 2);
	fprintf(os, "USB VBUS: %s volts\n", vcc);
#endif
}

FILE *
tmon_log_fp(void)
{

	return tmon_state.ts_log_fp;
}

void
tmon_log(const char *fmt, ...)
{
	struct tmon_state *ts = &tmon_state;

	if (ts->ts_log_fp != NULL) {
		va_list ap;
		ts->ts_log_saw_output = true;
		va_start(ap, fmt);
		vfprintf(ts->ts_log_fp, fmt, ap);
		va_end(ap);
	}
}

static void
tmon_cmd_reset(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	target_t t = NULL;

	(void) argc;
	(void) argv;

	rtos_mutex_acquire(ts->ts_mutex);
	if (ts->ts_targets != NULL && (t = ts->ts_targets[0]) != NULL)
		tmon_reset_impl(t, TARGET_RESET_HARD);
	rtos_mutex_release(ts->ts_mutex);

	if (t != NULL)
		fputs("Target reset.\n", os);
	else
		fputs("Target disappeared.\n", os);
}

static const char tmon_cmd_rescan_usage[] =
"usage:\n"
"rescan [log | auto | man]\n"
"\tWith no parameters, this command initiates a rescan for targets on\n"
"\tthe debug connector. Specify 'log' to initiate a verbose rescan.\n"
"\tThis can provide useful information to help identify devices which\n"
"\tare not yet supported.\n"
"\tSpecify 'auto' to select auto-rescan. In this mode, Maven will scan\n"
"\tcontinuously the SWD interface looking for valid debug targets,\n"
"\tstopping only when one is found. If the target disconnects, auto\n"
"\tscanning will recommence.\n"
"\tSpecify 'manual' to select manual rescan. The SWD interface will\n"
"\tbe scanned once when a 'rescan' command is issued.\n";

static void
tmon_cmd_rescan(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	enum tmon_scan_type st;
	timer_timeout_t to;
	bool do_log = false;
	bool do_scan = false;

	(void) argc;
	(void) argv;

	rtos_mutex_acquire(ts->ts_mutex);

	st = ts->ts_scan_type;

	if (argc == 1) {
		size_t l = strlen(argv[0]);

		if (strncasecmp(argv[0], "auto", l) == 0)
			st = TMON_SCAN_AUTO;
		else
		if (strncasecmp(argv[0], "manual", l) == 0)
			st = TMON_SCAN_MANUAL;
		else
		if (strncasecmp(argv[0], "log", l) == 0) {
			do_log = true;
			do_scan = true;
		} else {
 usage:
			rtos_mutex_release(ts->ts_mutex);
			fputs(tmon_cmd_rescan_usage, os);
			return;
		}

		if (ts->ts_scan_type != st) {
			ts->ts_scan_type = st;

#ifndef CONFIG_USE_CONFIGDB
			glob.glob_scan_type = (uint8_t) st;
			glob_save();
#else
			cf_set_uint8(CF_KEY_UINT8_SCAN_TYPE, (uint8_t)st);
			configdb_persist();
#endif
		}
	} else
	if (argc != 0)
		goto usage;
	else
		do_scan = true;

	if (st != ts->ts_scan_type) {
		fprintf(os, "Scan mode: %s\n",
		    (st == TMON_SCAN_AUTO) ? "Auto" : "Manual");
	}

	if (do_scan == false)
		goto out;

	if (!POWER_TARGET_VCC_GOOD) {
		fputs("Target Vcc is not valid. Scan aborted.\n", os);
		goto out;
	}

	if (ts->ts_targets != NULL && ts->ts_targets[0] != NULL)
		ts->ts_forced_rescan = true;

	ts->ts_scan_type_saved = ts->ts_scan_type;
	ts->ts_scan_type = TMON_SCAN_ONCE;

	if (do_log) {
		if (ts->ts_log_fp != NULL) {
			fputs("Logging scan already scheduled.\n", os);
			goto out;
		}

		ts->ts_log_fp = os;
		ts->ts_log_complete = false;

		rtos_mutex_release(ts->ts_mutex);

		fputs("Logging scan scheduled. Please wait...\n", os);
		fflush(os);
		timer_timeout_start(&to, 10000);

		while (ts->ts_log_complete == false &&
		    !timer_timeout_expired(&to)) {
			rtos_task_sleep(300);
		}

		rtos_mutex_acquire(ts->ts_mutex);

		fputs("Rescan complete", os);
		if (ts->ts_log_fp != NULL) {
			fputs("; nothing found", os);
			ts->ts_log_fp = NULL;
		}
		fputs(".\n", os);
	} else {
		fputs("Scanning...\n", os);
		fflush(os);
		rtos_mutex_release(ts->ts_mutex);

		timer_timeout_start(&to, 10000);
		while (ts->ts_scan_type == TMON_SCAN_ONCE &&
		    !timer_timeout_expired(&to)) {
			rtos_task_sleep(300);
		}

		rtos_mutex_acquire(ts->ts_mutex);

		if (ts->ts_scan_type == TMON_SCAN_ONCE)
			fputs("Scan failed.\n", os);
	}

	if (ts->ts_num_targets == 0)
		fputs("No targets detected.\n", os);

	for (unsigned int i = 0; i < ts->ts_num_targets; i++) {
		struct tmon_get_target_name tn;
		target_t t = ts->ts_targets[i];


		if (tmon_get_target_name_impl(t, &tn) != TMON_QN_OK) {
			fprintf(os, "#%u: Unknown\n", i + 1);
		} else {
			if (i == 0) {
				if (tn.tn_vendor_name != NULL)
					fprintf(os, "%s", tn.tn_vendor_name);
				if (tn.tn_soc_name != NULL)
					fprintf(os, " %s", tn.tn_soc_name);
				if ((t->t_flags & TARGET_FLAG_IS_LOCKED) != 0)
					fputs(" ** LOCKED **", os);
				fputc('\n', os);
			}

			/* Locked targets cannot be debugged. */
			if ((t->t_flags & TARGET_FLAG_IS_LOCKED) == 0) {
				assert(tn.tn_core_name != NULL);
				fprintf(os, " #%u: %s\n", i + 1,
				    tn.tn_core_name);
			}
		}
	}

 out:
	rtos_mutex_release(ts->ts_mutex);
}

static const char tmon_cmd_protect_usage[] =
"usage:\n"
"protect [level [permanent]]\n"
"\tWith no parameters, this command displays the target's current\n"
"\tprotection level along with a description of the possible target-\n"
"\tspecific values for 'level'.\n"
"\tIf specified, 'level' is used to change the current protection\n"
"\tlevel of the target. Note that this command can only increase the\n"
"\tprotection level. Use the 'unlock' command to remove protection.\n"
"\tif possible. Specify '1' or 'set' to enable protection. Other non-\n"
"\tzero values for 'level' are interpreted in a target-specific way,\n"
"\tas described when the current protection level is displayed.\n\n"
"Note that changing the protection level may require the target to be\n"
"power-cycled for the change to be effective. If 'level' is followed by\n"
"the word \"permanent\" then, for some targets, the combination may\n"
"render the target irreversibly locked, so be sure to understand the\n"
"consequences of your actions!\n";

static const char tmon_prot_text_powercycle[] = "TARGET MUST BE POWER-"
						"CYCLED.";
static const char tmon_prot_text_reset[] = "TARGET MUST BE RESET.";
static const char tmon_prot_text_unchanged[] = "Level unchanged.";
static const char tmon_prot_text_perm_flag[] = "Level unchanged; 'permanent'"
					       " flag must be specified.";
static const char tmon_prot_text_use_unlock[] = "Cannot lower level. Use"
						"the 'unlock' command.";

static void
tmon_cmd_protect(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	struct target_protection_info pi;
	int rv, do_set = 0;
	u_int new_level;
	target_t t;

	rtos_mutex_acquire(ts->ts_mutex);

	if (ts->ts_targets == NULL || (t = ts->ts_targets[0]) == NULL) {
		rtos_mutex_release(ts->ts_mutex);
		fputs("Target disappeared.\n", os);
		return;
	}

	pi.pi_out_file = os;
	pi.pi_text = NULL;
	rv = tmon_target_protection_query_impl(t, &pi);
	if (rv < 0) {
		rtos_mutex_release(ts->ts_mutex);
		fputs("Failed to query max protection level\n", os);
		return;
	}

	if (argc > 2) {
 usage:
 		fputs(tmon_cmd_protect_usage, os);
 failmaxlevel:
		rtos_mutex_release(ts->ts_mutex);
		fprintf(os, "Valid range for level is 0-%u\n", pi.pi_max_level);
		return;
	}

	new_level = pi.pi_level;

	if (argc) {
		size_t len = strlen(argv[0]);
		if ((len == 1 && argv[0][0] == '1') ||
		    strncasecmp("set", argv[0], len) == 0) {
			new_level = 1;
			argc--;
		} else
		if ((len == 1 && argv[0][0] == '0') ||
		    strncasecmp("clear", argv[0], len) == 0) {
			new_level = 0;
			argc--;
		} else
		if (sscanf(argv[0], "%u", &new_level) != 1) {
			goto usage;
		} else
		if (new_level > pi.pi_max_level) {
			goto failmaxlevel;
		} else {
			argc--;
		}

		pi.pi_permanent = 0;
		if (argc) {
			if (strcasecmp("permanent", argv[1]) == 0)
				pi.pi_permanent = 1;
			else
				goto usage;

			if ((t->t_flags & TARGET_FLAG_SUPPORTS_PERM_PROT) == 0){
				fputs("Ignoring 'permanent' flag; unsupported "
				    "by this target.\n", os);
				pi.pi_permanent = 0;
			}
		}

		if (new_level < pi.pi_level)
			fputs(tmon_prot_text_use_unlock, os);

		do_set = new_level > pi.pi_level;
		if (do_set)
			pi.pi_level = new_level;
	}

	pi.pi_out_file = os;
	pi.pi_text = NULL;
	if (do_set)
		rv = tmon_target_protection_set_impl(t, &pi);
	else
		rv = tmon_target_protection_query_impl(t, &pi);

	rtos_mutex_release(ts->ts_mutex);

	if (rv < 0) {
		fprintf(os, "Failed to %s the protection level\n",
		    do_set ? "set" : "query");
		return;
	}

	if (do_set) {
		const char *msg;
		switch (rv) {
		default:
			msg = "";
			break;
		case TARGET_PI_OK_POWER_CYCLE:
			msg = tmon_prot_text_powercycle;
			break;
		case TARGET_PI_OK_RESET:
			msg = tmon_prot_text_reset;
			break;
		case TARGET_PI_UNCHANGED:
			msg = tmon_prot_text_unchanged;
			break;
		case TARGET_PI_NEED_PERM_FLAG:
			msg = tmon_prot_text_perm_flag;
			break;
		case TARGET_PI_USE_UNLOCK:
			msg = tmon_prot_text_use_unlock;
			break;
		}
		fprintf(os, "New level: %u\n%s\n", pi.pi_level, msg);
	} else {
		assert(pi.pi_text != NULL);
		fprintf(os, "%sCurrent level: %u\n", pi.pi_text, pi.pi_level);
	}
}

static const char tmon_cmd_unlock_usage[] =
"usage:\n"
"unlock\n"
"\tThis command will perform the necessary steps to unlock a device which\n"
"\thas been protected from debugger access. In effect, it undos the results\n"
"\tof the 'protect' command. In most cases, this will cause the device's\n"
"\tFlash and volatile memories to be completely erased, and is usually\n"
"\tfollowed up with a device reset. However, if the device has been locked\n"
"\tpermanently then this command will have no effect.\n";

static void
tmon_cmd_unlock(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	target_t t;
	int rv;

	(void) argv;

	if (argc >= 2) {
		fputs(tmon_cmd_unlock_usage, os);
		return;
	}


	rtos_mutex_acquire(ts->ts_mutex);

	if (ts->ts_targets == NULL || (t = ts->ts_targets[0]) == NULL) {
		rtos_mutex_release(ts->ts_mutex);
		fputs("Target disappeared.\n", os);
		return;
	}

	fputs("Attempting unlock...\n", os);

	rv = tmon_target_unlock_impl(t, os);

	rtos_mutex_release(ts->ts_mutex);

	if (rv < 0) {
		fputs("Unlock failed.\n", os);
	} else {
		const char *msg;

		switch (rv) {
		case TARGET_PI_OK:
			msg = "Unlock succeeded.";
			break;
		case TARGET_PI_OK_POWER_CYCLE:
			msg = tmon_prot_text_powercycle;
			break;
		case TARGET_PI_OK_RESET:
			msg = tmon_prot_text_reset;
			break;
		case TARGET_PI_UNCHANGED:
			msg = tmon_prot_text_unchanged;
			break;
		default:
			msg = "";
			assert(0);
			break;
		}
		fprintf(os, "%s\n", msg);
	}
}

bool
tmon_flash_verify_enabled(void)
{
	uint8_t v;

#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_uint8(CF_KEY_UINT8_FLASH_VERIFY, &v) == 0)
		v = 1;
#else
	v = glob.glob_flash_verify;
#endif

	return v != 0;
}

static const char tmon_cmd_flash_usage[] =
"usage:\n"
"flash verify <on|off>\n"
"\tControls whether or not " PRODUCT_NAME " will verify writes to\n"
"\tFlash memory. Disabling verification can improve write performance\n"
"\tat the risk of write-failures going unnoticed. Verification is on\n"
"\tby default.\n";

static void
tmon_cmd_flash(FILE *os, uint8_t argc, const char * const *argv)
{
	bool verify_current, verify_new;
	size_t arglen;

	verify_current = verify_new = tmon_flash_verify_enabled();

	if (argc) {
		arglen = strlen(argv[0]);

		if (argc != 2 ||
		    strncasecmp(argv[0], "verify", arglen) != 0) {
 usage:
			fputs(tmon_cmd_flash_usage, os);
			return;
		}

		arglen = strlen(argv[1]);
		if (strcasecmp(argv[1], "on") == 0 ||
		    strncasecmp(argv[1], "enable", arglen) == 0 ||
		    (argv[1][0] == '1' && arglen == 1)) {
			verify_new = true;
		} else
		if (strcasecmp(argv[1], "off") == 0 ||
		    strncasecmp(argv[1], "disable", arglen) == 0 ||
		    (argv[1][0] == '0' && arglen == 1)) {
			verify_new = false;
		} else
			goto usage;

		if (verify_new != verify_current) {
			uint8_t v = verify_new ? 1 : 0;
#ifdef CONFIG_USE_CONFIGDB
			cf_set_uint8(CF_KEY_UINT8_FLASH_VERIFY, v);
			configdb_persist();
#else
			glob.glob_flash_verify = v;
			glob_save();
#endif
		}
	}

	fprintf(os, "Flash verify: %s\n", verify_new ? "Enabled" : "Disabled");
}

#if (RELEASE_BUILD == 0)
static int
tmon_cmd_memory_write(target_t t, target_addr_t addr, int size, uint32_t value)
{
	int rv;

	if (size == sizeof(uint32_t))
		rv = tmon_mem_write_impl(t, addr, size, &value);
	else
	if (size == sizeof(uint16_t)) {
		uint16_t v16 = (uint16_t)value;
		rv = tmon_mem_write_impl(t, addr, size, &v16);
	} else {
		uint8_t v8 = (uint8_t)value;
		rv = tmon_mem_write_impl(t, addr, size, &v8);
	}

	return rv == TMON_MW_OK;
}

static int
tmon_cmd_memory_read(FILE *os, target_t t, target_addr_t addr, int size,
    uint32_t count)
{
	union {
		uint32_t v32[4];
		uint16_t v16[8];
		uint8_t v8[16];
	} u;
	u_int columns, curcol;
	uint8_t *p;
	size_t total;
	u_int i;
	int rv = 0;

	if (count == 0)
		count = 1;

	total = (u_int)size * count;

	if (size == sizeof(uint32_t))
		columns = 4;
	else
	if (size == sizeof(uint16_t))
		columns = 8;
	else
		columns = 16;
	curcol = 0;
	p = u.v8;

	while (total) {
		total -= size;

		if (curcol++ == 0)
			fprintf(os, "%08" PRIxTADDR ":", addr);

		rv = tmon_mem_read_impl(t, addr, size, p);
		if (rv != TMON_MR_OK)
			break;

		addr += size;
		p += size;

		if (size == sizeof(uint32_t))
			fprintf(os, " %08" PRIx32, u.v32[curcol - 1]);
		else
		if (size == sizeof(uint16_t))
			fprintf(os, " %04" PRIx16, u.v16[curcol - 1]);
		else {
			/* XXX: newlib nano-printf doesn't grok %hh */
			fprintf(os, " %02x", (unsigned int)u.v8[curcol - 1]);
		}

		if (curcol == columns) {
			fputc(' ', os);
			for (i = 0; i < sizeof(u.v8); i++) {
				if (u.v8[i] >= ' ' && u.v8[i] < 0x7fu)
					fputc((char)u.v8[i], os);
				else
					fputc('.', os);
			}
			fputc('\n', os);
			curcol = 0;
			p = u.v8;
		}
	}

	if (rv != TMON_MR_OK)
		fprintf(os, " Read failed\n");
	else
	if (curcol < columns) {
		fputc(' ', os);
		for (i = 0; i < (curcol * size); i++) {
			if (u.v8[i] >= ' ' && u.v8[i] < 0x7fu)
				fputc((char)u.v8[i], os);
			else
				fputc('.', os);
		}
		fputc('\n', os);
	}

	return rv == TMON_MR_OK;
}

static const char tmon_cmd_memory_usage[] =
"usage:\n"
"memory [-124] [-w] <[address [count]] | address value>\n"
;

static void
tmon_cmd_memory(FILE *os, uint8_t argc, const char * const *argv, void *ctx)
{
	struct tmon_state *ts = ctx;
	target_addr_t addr;
	uint32_t value;
	int rv, ch, wflag, size;
	target_t t;

	wflag = 0;
	size = ts->ts_last_size;

	if (argc) {
		getopt_data getopt_state;

		argc++;
		argv--;
		memset(&getopt_state, 0, sizeof(getopt_state));
		while ((ch = getopt_r(argc, (char * const *)(uintptr_t)argv,
		    "124w", &getopt_state)) != EOF) {
			switch (ch) {
			case '1':
				size = sizeof(uint8_t);
				break;
			case '2':
				size = sizeof(uint16_t);
				break;
			case '4':
				size = sizeof(uint32_t);
				break;
			case 'w':
				wflag = 1;
				break;
			default:
				goto usage;
			}
		}

		argc -= getopt_state.optind;
		argv += getopt_state.optind;
	}

	if (wflag && argc != 2) {
		fprintf(os, "-w requires address and value.\n");
		return;
	}

	addr = ts->ts_last_address;
	value = wflag ? ts->ts_last_wr_value : ts->ts_last_rd_value;

	if (argc) {
		const char *pa = argv[0];
		if (strncasecmp(pa, "0x", 2) == 0)
			pa += 2;
		if (sscanf(pa, "%" SCNxTADDR, &addr) != 1) {
			fprintf(os, "Bad address '%s'\n", argv[0]);
			return;
		}
		if (argc == 2) {
			pa = argv[1];
			if (strncasecmp(pa, "0x", 2) == 0)
				pa += 2;
			if (sscanf(pa, "%" SCNx32, &value) != 1) {
				fprintf(os, "Bad value '%s'\n", argv[1]);
				return;
			}
		} else
		if (argc > 2) {
 usage:
			fprintf(os, "%s", tmon_cmd_memory_usage);
			return;
		}
	}

	rtos_mutex_acquire(ts->ts_mutex);

	if (ts->ts_targets == NULL || (t = ts->ts_targets[0]) == NULL) {
		rtos_mutex_release(ts->ts_mutex);
		fputs("Target disappeared.\n", os);
		return;
	}

	if (wflag) {
		rv = tmon_cmd_memory_write(t, addr, size, value);
		if (rv)
			addr += size;
	} else {
		rv = tmon_cmd_memory_read(os, t, addr, size, value);
		if (rv)
			addr += value * size;
	}

	if (rv == 0) {
		uint32_t link_err;
		tmon_get_link_error_impl(t, &link_err);
		fprintf(os, "Link error: %08" PRIx32 "\n", link_err);
	}

	ts->ts_last_size = size;
	ts->ts_last_address = addr;
	if (wflag)
		ts->ts_last_wr_value = value;
	else
		ts->ts_last_rd_value = value;

	rtos_mutex_release(ts->ts_mutex);
}
#endif /* (RELEASE_BUILD == 0) */

static const char tmon_cmd_attach_usage[] =
"usage:\n"
"gdb_attach reset\n"
"\tWhen GDB attaches, reset the target and halt it on the reset vector.\n"
"\tThis is useful in conjunction with GDB's \"load\" command to download\n"
"\tand run/debug a new firmware image. This is the default setting.\n"
"gdb_attach halt\n"
"\tWhen GDB attaches, forcibly halt the target without performing a reset.\n"
"\tThis is useful when using GDB to \"break\" into a running programm to\n"
"\tdetermine its current state.\n"
#ifdef CONFIG_USE_CONFIGDB
"gdb_attach auto\n"
"\tWhen GDB connects to " PRODUCT_NAME ", a connection to the target will\n"
"\ttake place automatically, without the need for \"attach 1\" at the GDB\n"
"\tprompt. This is the default setting.\n"
"gdb_attach manual\n"
"\tWhen GDB connects to " PRODUCT_NAME ", you will then need to issue an\n"
"\t\"attach N\" command at the GDB prompt, where \"N\" is the CPU core\n"
"\tnumber plus 1. For example \"attach 1\" will attach to CPU core #0.\n"
#endif
"\nThe settings are persistant; it will be preserved across " PRODUCT_NAME 
" reboots.\n";

static void
tmon_cmd_attach(FILE *os, uint8_t argc, const char * const *argv)
{
	size_t len;
	uint8_t attach_reset;
#ifdef CONFIG_USE_CONFIGDB
	uint8_t attach_auto_current, attach_auto;
	bool changed;
#endif

	if (argc > 1) {
 usage:
 		fputs(tmon_cmd_attach_usage, os);
		return;
	}

#ifdef CONFIG_USE_CONFIGDB
	if (cf_get_uint8(CF_KEY_UINT8_ATTACH_AUTO, &attach_auto_current) == 0)
		attach_auto_current = 1;
	attach_auto = attach_auto_current;
#endif

	if (argc) {
		len = strlen(argv[0]);
		attach_reset = (uint8_t)tmon_attach_reset;
		if (strncasecmp("reset", argv[0], len) == 0)
			attach_reset = 1;
		else
		if (strncasecmp("halt", argv[0], len) == 0)
			attach_reset = 0;
		else
#ifdef CONFIG_USE_CONFIGDB
		if (strncasecmp("auto", argv[0], len) == 0)
			attach_auto = 1;
		else
		if (strncasecmp("manual", argv[0], len) == 0)
			attach_auto = 0;
		else
#endif
			goto usage;

#ifndef CONFIG_USE_CONFIGDB
		if (attach_reset != tmon_attach_reset) {
			tmon_attach_reset = (int) attach_reset;
			glob_save();
		}
#else
		changed = false;
		if (attach_reset != tmon_attach_reset) {
			tmon_attach_reset = attach_reset;
			cf_set_uint8(CF_KEY_UINT8_ATTACH_RESET, attach_reset);
			changed = true;
		}

		if (attach_auto != attach_auto_current) {
			cf_set_uint8(CF_KEY_UINT8_ATTACH_AUTO, attach_auto);
			changed = true;
		}

		if (changed)
			configdb_persist();
#endif
	}

#ifndef CONFIG_USE_CONFIGDB
	fprintf(os, "GDB Attach behaviour: %s\n", tmon_attach_reset ?
	    "Reset" : "Halt");
#else
	fprintf(os, "GDB Attach behaviour: %s with %s\n",
	    attach_auto ? "Automatic" : "Manual",
	    tmon_attach_reset ? "Reset" : "Halt");
#endif
}
