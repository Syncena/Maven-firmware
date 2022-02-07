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

#define __need_getopt_newlib
#include <assert.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "target.h"
#include "tmon.h"
#include "gdb_server.h"
#include "gdb_rtos.h"
#include "gdb_rtos_freertos.h"
#include "gdb_rtos_nuttx.h"
#include "configdb.h"
#include "shell.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

static const struct gdb_rtos_backend * const gdb_rtos_backends[] = {
	[TARGET_RTOS_FREERTOS] = &gdb_rtos_backend_freertos,
	[TARGET_RTOS_NUTTX] = &gdb_rtos_backend_nuttx,
};
#define	GDB_RTOS_BACKEND_COUNT	(sizeof(gdb_rtos_backends) / \
				 sizeof(gdb_rtos_backends[0]))

SHELL_CMD_DECL(rtos, gdb_rtos_cmd, "View/modify RTOS configuration");

struct gdb_rtos_state {
	tmon_cookie_t rs_tmon;
	unsigned int rs_instance;
	target_rtos_type_t rs_rtos_type;
	bool rs_rtos_auto;
	const struct gdb_rtos_backend *rs_backend;
	void *rs_backend_ctx;
	gdb_rtos_params_t rs_backend_params;
	struct gdb_rtos_symbol_value *rs_sym_values;
	struct gdb_rtos_thread rs_dummy;
	struct gdb_rtos_thread *rs_threads;
	int rs_nthreads;
	target_addr_t rs_current_thread;

	/* Used while probing symbols for each back-end. */
	unsigned int rs_backend_number;
	unsigned int rs_symbol_number;
};

static rtos_mutex_t gdb_rtos_state_mutex;
static struct gdb_rtos_state *gdb_rtos_current_states[CONFIG_GDB_SERVERS];
static gdb_rtos_params_t gdb_rtos_custom_params[TARGET_RTOS_COUNT];

void
gdb_rtos_init(void)
{

	gdb_rtos_state_mutex = rtos_mutex_create();
	assert(gdb_rtos_state_mutex);

	SHELL_CMD_ADD(rtos);
}

static bool
gdb_rtos_supported(target_rtos_type_t *prt)
{
	uint8_t t;

	if (cf_get_uint8(CF_KEY_UINT8_RTOS_TYPE, &t) != 0) {
		if (t > TARGET_RTOS_NONE)
			t = TARGET_RTOS_NONE;

		/* Return now if user disabled RTOS support. */
		if (t == TARGET_RTOS_NONE)
			return false;
	} else {
		/* Force auto detection. */
		t = TARGET_RTOS_NONE;
	}

	*prt = (target_rtos_type_t)t;
	return true;
}

gdb_rtos_state_t
gdb_rtos_new(tmon_cookie_t tc, unsigned int gdb_instance)
{
	gdb_rtos_state_t rs;
	target_rtos_type_t rt;

	DBFPRINTF("New state for instance %u\n", gdb_instance);

	if (gdb_rtos_supported(&rt) == false) {
		DBFPRINTF("RTOS support disabled.\n");
		return NULL;
	}

	if ((rs = zone_calloc(1, sizeof(*rs))) == NULL) {
		DBFPRINTF("Out of memory!\n");
		return NULL;
	}

	rtos_mutex_acquire(gdb_rtos_state_mutex);

	assert(gdb_instance < CONFIG_GDB_SERVERS);
	assert(gdb_rtos_current_states[gdb_instance] == NULL);

	if (rt != TARGET_RTOS_NONE) {
		rs->rs_rtos_type = rt;
		rs->rs_rtos_auto = false;
		DBFPRINTF("Fixed RTOS: %u.\n", (unsigned int)rt);
	} else {
		rs->rs_rtos_auto = true;
		DBFPRINTF("Auto detect.\n");
	}

	rs->rs_tmon = tc;
	rs->rs_instance = gdb_instance;

	rs->rs_dummy.rt_tid = 1;
	strcpy(rs->rs_dummy.rt_info, "Bare metal");

	gdb_rtos_current_states[gdb_instance] = rs;

	rtos_mutex_release(gdb_rtos_state_mutex);

	return rs;
}

static void
gdb_rtos_free_symbols(gdb_rtos_state_t rs)
{

	if (rs->rs_sym_values != NULL) {
		zone_free(rs->rs_sym_values);
		rs->rs_sym_values = NULL;
	}
}

static void
gdb_rtos_free_threads(gdb_rtos_state_t rs)
{

	if (rs->rs_threads == NULL)
		return;

	zone_free(rs->rs_threads);
	rs->rs_threads = NULL;
	rs->rs_nthreads = 0;
}

void
gdb_rtos_free(gdb_rtos_state_t rs)
{

	DBFPRINTF("Freeing RTOS instance %u\n", rs->rs_instance);

	rtos_mutex_acquire(gdb_rtos_state_mutex);

	assert(gdb_rtos_current_states[rs->rs_instance] == rs);
	gdb_rtos_current_states[rs->rs_instance] = NULL;

	rtos_mutex_release(gdb_rtos_state_mutex);

	if (rs->rs_backend != NULL)
		rs->rs_backend->rb_free(rs->rs_backend_ctx);

	if (rs->rs_backend_params != NULL)
		zone_free(rs->rs_backend_params);

	gdb_rtos_free_threads(rs);

	gdb_rtos_free_symbols(rs);
	zone_free(rs);
}

bool
gdb_rtos_detected(gdb_rtos_state_t rs)
{

	return rs->rs_backend != NULL;
}

static const char *
gdb_rtos_init_symbols(gdb_rtos_state_t rs, const struct gdb_rtos_backend *rb)
{

	gdb_rtos_free_symbols(rs);

	DBFPRINTF("Trying RTOS '%s'.\n", rb->rb_name);

	rs->rs_sym_values = zone_calloc(rb->rb_nsymbols,
	    sizeof(rs->rs_sym_values[0]));
	if (rs->rs_sym_values == NULL) {
		DBFPRINTF("Failed to alloc rs_sym_values\n");
		return NULL;
	}

	rs->rs_symbol_number = 0;

	DBFPRINTF("First symbol: '%s'\n", rb->rb_symbols[0].sym_name);

	return rb->rb_symbols[0].sym_name;
}

static unsigned int
gdb_rtos_find_backend_number_by_type(target_rtos_type_t rt)
{

	for (unsigned int i = 0; i < GDB_RTOS_BACKEND_COUNT; i++) {
		if (gdb_rtos_backends[i]->rb_type == rt)
			return i;
	}

	/* This should never happen. */
	return GDB_RTOS_BACKEND_COUNT;
}

static const struct gdb_rtos_backend *
gdb_rtos_find_backend_by_name(const char *name)
{

	for (unsigned int i = 0; i < GDB_RTOS_BACKEND_COUNT; i++) {
		if (strcasecmp(gdb_rtos_backends[i]->rb_name, name) == 0)
			return gdb_rtos_backends[i];
	}

	return NULL;
}

const char *
gdb_rtos_qSymbol_first(gdb_rtos_state_t rs)
{
	unsigned int i;

	if (rs->rs_backend != NULL) {
		DBFPRINTF("RTOS unavailable.\n");
		return NULL;
	}

	if (rs->rs_rtos_auto == false) {
		i = gdb_rtos_find_backend_number_by_type(rs->rs_rtos_type);
		assert(i < GDB_RTOS_BACKEND_COUNT);
	} else {
		i = 0;
	}

	rs->rs_backend_number = i;

	return gdb_rtos_init_symbols(rs, gdb_rtos_backends[i]);
}

static gdb_rtos_params_t
gdb_rtos_get_back_end_params(const struct gdb_rtos_backend *rb)
{
	gdb_rtos_params_t rp;

	rtos_mutex_acquire(gdb_rtos_state_mutex);

	if ((rp = gdb_rtos_custom_params[rb->rb_type]) == NULL &&
	    (rp = zone_malloc(rb->rb_params_size)) != NULL) {
		cf_len_t pl;

		gdb_rtos_custom_params[rb->rb_type] = rp;

		/* Does this RTOS have custom params? */
		pl = cf_get_data(CF_KEY_DATA_RTOS_PARAMS(rb->rb_type), 0, NULL);

		if (pl == rb->rb_params_size) {
			cf_get_data(CF_KEY_DATA_RTOS_PARAMS(rb->rb_type), pl,
			    rp);
		} else {
			memcpy(rp, rb->rb_params_default, rb->rb_params_size);
		}
	}

	rtos_mutex_release(gdb_rtos_state_mutex);

	return rp;
}

const char *
gdb_rtos_qSymbol_next(gdb_rtos_state_t rs, bool found, target_addr_t value)
{
	const struct gdb_rtos_backend *rb;
	const struct gdb_rtos_symbol *sym;

	assert(rs->rs_backend == NULL);
	assert(rs->rs_backend_number < GDB_RTOS_BACKEND_COUNT);
	rb = gdb_rtos_backends[rs->rs_backend_number];
	assert(rs->rs_symbol_number < rb->rb_nsymbols);
	sym = &rb->rb_symbols[rs->rs_symbol_number];

	if (!found && (sym->sym_flags & GDB_RTOS_SYM_FLAGS_OPTIONAL) == 0) {
		/*
		 * Failed to find a mandatory symbol. Move to the next
		 * back-end if we're in auto-detect mode.
		 */
		DBFPRINTF("Mandatory symbol '%s' not found.\n", sym->sym_name);
		rs->rs_backend_number++;
		if (rs->rs_rtos_auto == false ||
		    rs->rs_backend_number == GDB_RTOS_BACKEND_COUNT) {
			/* Finished. No RTOS support. */
			gdb_rtos_free_symbols(rs);
			DBFPRINTF("Giving up.\n");
			return NULL;
		}

		rb = gdb_rtos_backends[rs->rs_backend_number];
		return gdb_rtos_init_symbols(rs, rb);
	}

	if (found) {
		DBFPRINTF("Symbol '%s' found. Value %" PRIxTADDR "\n",
		    sym->sym_name, (target_addr_t)value);
	} else {
		DBFPRINTF("Symbol '%s' not found.\n", sym->sym_name);
	}

	/* Record the symbol's value and found status. */
	rs->rs_sym_values[rs->rs_symbol_number].sv_value = value;
	rs->rs_sym_values[rs->rs_symbol_number].sv_found = found;

	rs->rs_symbol_number++;
	if (rs->rs_symbol_number < rb->rb_nsymbols) {
		/* Backend has more symbols to resolve. */
		sym++;
		DBFPRINTF("Next symbol '%s'.\n", sym->sym_name);
		return sym->sym_name;
	}

	/*
	 * All symbols in this backend have been resolved.
	 */
	rs->rs_backend = rb;
	DBFPRINTF("Detected RTOS '%s'.\n", rb->rb_name);

	rs->rs_backend_params = NULL;
	if (rb->rb_nparams) {
		gdb_rtos_params_t rp;

		assert(rb->rb_params_size != 0);

		rs->rs_backend_params = zone_malloc(rb->rb_params_size);
		if (rs->rs_backend_params == NULL) {
			DBFPRINTF("Out of memory for instance params\n");
			return NULL;
		}

		rp = gdb_rtos_get_back_end_params(rb);
		if (rp == NULL) {
			DBFPRINTF("Out of memory for back-end params\n");
			return NULL;
		}

		memcpy(rs->rs_backend_params, rp, rb->rb_params_size);
	}

	/* The RTOS backend can now safely attach. */
	rs->rs_backend_ctx = rb->rb_attach(rs->rs_tmon, rs->rs_sym_values,
	    rs->rs_backend_params);

	if (rs->rs_backend_ctx == NULL)
		rs->rs_backend = NULL;
	else {
		/*
		 * By this time, GDB is attached to the target. If the target
		 * was reset at attach time, we cannot depend on the contents
		 * of volatile memory such as SRAM. Go initialise it, if
		 * necessary.
		 */
		if (tmon_attach_will_reset())
			gdb_rtos_reset_symbols(rs);
	}

	return NULL;
}

static bool
gdb_rtos_do_init_syms(void)
{
	uint8_t v;

	if (cf_get_uint8(CF_KEY_UINT8_RTOS_INIT_SYMBOLS, &v) == 0)
		return true;

	return v != 0;
}

void
gdb_rtos_reset_symbols(gdb_rtos_state_t rs)
{
	const struct gdb_rtos_backend *rb;
	const struct gdb_rtos_symbol *sym;
	uint64_t zero;

	if ((rb = rs->rs_backend) == NULL || !gdb_rtos_do_init_syms())
		return;

	sym = rb->rb_symbols;
	zero = 0;

	for (unsigned int i = 0; i < rb->rb_nsymbols; i++, sym++) {
		if (rs->rs_sym_values[i].sv_found == false ||
		    (sym->sym_flags & GDB_RTOS_SYM_FLAGS_VOLATILE) == 0)
			continue;

		DBFPRINTF("Zapping symbol '%s' at 0x%" PRIxTADDR ".\n",
		    sym->sym_name, rs->rs_sym_values[i].sv_value);

		/*
		 * XXX: This only clears the first 64-bits of the symbol.
		 * The back-end data structure needs a hint as to the
		 * symbol's size.
		 */
		tmon_mem_write(rs->rs_tmon, rs->rs_sym_values[i].sv_value,
		    sizeof(uint64_t), &zero);
	}

	if (rs->rs_backend_ctx != NULL && rb->rb_reset != NULL)
		rb->rb_reset(rs->rs_backend_ctx);
}

void
gdb_rtos_update_threads(gdb_rtos_state_t rs)
{
	const struct gdb_rtos_backend *rb = rs->rs_backend;

	if (rb == NULL)
		return;

	gdb_rtos_free_threads(rs);

	rs->rs_nthreads = rb->rb_threads(rs->rs_backend_ctx, &rs->rs_threads,
	    &rs->rs_current_thread);

	if (rs->rs_nthreads < 0) {
		DBFPRINTF("Failed to update threads.\n");
		rs->rs_nthreads = 0;
	}

	if (rs->rs_nthreads == 0) {
		DBFPRINTF("Using dummy thread.\n");
		rs->rs_current_thread = rs->rs_dummy.rt_tid;
	}

	gdb_rtos_select_thread(rs, 0);
}

void
gdb_rtos_select_thread(gdb_rtos_state_t rs, target_addr_t tid)
{
	const struct gdb_rtos_backend *rb = rs->rs_backend;
	target_addr_t frame;
	void *param;

	DBFPRINTF("tid 0x%" PRIxTADDR "\n", tid);

	if (tid == 0 || rs->rs_nthreads == 0)
		tid = rs->rs_current_thread;

	if (rb == NULL || tid == rs->rs_current_thread ||
	    (frame = rb->rb_get_frame(rs->rs_backend_ctx, tid, &param)) == 0) {
		DBFPRINTF("Will use live registers.\n");
		tmon_rtos_set_frame(rs->rs_tmon, TARGET_RTOS_NONE, 0, NULL);
	} else {
		DBFPRINTF("Will use registers from thread frame at 0x%"
		    PRIxTADDR "\n", frame);
		tmon_rtos_set_frame(rs->rs_tmon, rb->rb_type, frame, param);
	}
}

const struct gdb_rtos_thread *
gdb_rtos_thread_info(gdb_rtos_state_t rs, target_addr_t tid)
{
	struct gdb_rtos_thread *rt = rs->rs_threads;

	for (int i = 0; i < rs->rs_nthreads; i++, rt++) {
		if (rt->rt_tid == tid)
			return rt;
	}

	if (tid == rs->rs_dummy.rt_tid)
		return &rs->rs_dummy;

	return NULL;
}

target_addr_t
gdb_rtos_current_thread(gdb_rtos_state_t rs)
{

	return rs->rs_current_thread;
}

stringio_t
gdb_rtos_thread_list(gdb_rtos_state_t rs)
{
	struct gdb_rtos_thread *rt = rs->rs_threads;
	int nthreads = rs->rs_nthreads;
	stringio_t tl;

	if (rt == NULL || nthreads == 0) {
		rt = &rs->rs_dummy;
		nthreads = 1;
	}

	if ((tl = stringio_new()) == NULL)
		return NULL;

	while (nthreads--) {
		stringio_printf(tl, "%" PRIxTADDR "%s", rt->rt_tid,
		    (nthreads > 0) ? "," : "");
		rt++;
	}

	return tl;
}

static const char gdb_rtos_cmd_usage[] =
"usage:\n"
"rtos\n"
"\tShow current RTOS configuration.\n"
"rtos support <auto|off|name>\n"
"\tConfigures RTOS detection.\n"
"\t\tauto: Target will be probed for all supported RTOSes. (default)\n"
"\t\toff:  RTOS support is disabled.\n"
"\t\tname: Only the named RTOS will be probed.\n"
"\tNote that changes to this will not affect current GDB sessions.\n"
"rtos params [-s] [-r <name>] [<new-params>]\n"
"\tShows parameters for the RTOS specified by <name>. If the target's\n"
"\tRTOS requires slightly different parameters, to handle structure\n"
"\tpadding, for example, then these can be specified with [new-params].\n"
"\tThis is a comma-separated list of integers pairs in the form I:V,\n"
"\twhere 'I' specifies the parameter index, and 'V' is the parameter\n"
"\tvalue. Valid indices per RTOS are shown by 'rtos params <name>'.\n"
"\tIf '-s' is specified, the new params are saved to non-volatile\n"
"\tstorage so need only be specified once. Otherwise, they will affect\n"
"\tthe current GDB session only. The default RTOS parameters can be\n"
"\trestored by specifying 'default' in place of <new-params>\n"
"rtos init-syms <on|off>\n"
"\tIf an RTOS is detected, this flag indicates whether or not\n"
"\tvolatile RTOS data should be initialised when 'gdb_attach' mode\n"
"\tis configured for 'reset'. The default is 'on', and will ensure\n"
"\tstale/invalid thread state is not used. Changes to this will not\n"
"\taffect current GDB sessions.\n\n"
"Supported RTOSes:\n";

static void
gdb_rtos_show_usage(FILE *os)
{

	fputs(gdb_rtos_cmd_usage, os);

	for (unsigned int i = 0; i < GDB_RTOS_BACKEND_COUNT; i++)
		fprintf(os, "\t%s\n", gdb_rtos_backends[i]->rb_name);
}

static void
gdb_rtos_show_params(FILE *os, const struct gdb_rtos_backend *rb,
    const gdb_rtos_params_t rp)
{
	static const char *source[] = {
		[GDB_RTOS_PARAM_DEFAULT] = "Default",
		[GDB_RTOS_PARAM_PROBED] = "Auto-probed",
		[GDB_RTOS_PARAM_CUSTOM] = "Customised",
		[GDB_RTOS_PARAM_SAVED_CUSTOM] = "Saved custom",
	};

	fprintf(os, "Parameters for \"%s\" (%s):\n", rb->rb_name,
	    source[rp->rp_source]);

	fputs("Index  Value  Description\n", os);
	for (unsigned int i = 0; i < rb->rb_nparams; i++) {
		const char *ptype, *pend;

		if ((rb->rb_params_desc[i].pd_flags &
		    GDB_RTOS_PARAM_TYPE_SIZEOF) != 0) {
			ptype = "sizeof(";
			pend = ")";
		} else
		if ((rb->rb_params_desc[i].pd_flags &
		    GDB_RTOS_PARAM_TYPE_OFFSETOF) != 0) {
			ptype = "offsetof(";
			pend = ")";
		} else
		if ((rb->rb_params_desc[i].pd_flags &
		    GDB_RTOS_PARAM_TYPE_BOOLEAN) != 0) {
			ptype = "boolean ";
			pend = "";
		} else {
			ptype = "";
			pend = "";
		}

		fprintf(os, "%5u  %5" PRIuRTPARAM "  %s%s%s\n", i,
		    rp->rp_val[i], ptype, rb->rb_params_desc[i].pd_name, pend);
	}
}

static void
gdb_rtos_show_symbols(FILE *os, const struct gdb_rtos_backend *rb,
    const struct gdb_rtos_symbol_value *sv)
{
	const struct gdb_rtos_symbol *sym = rb->rb_symbols;

	if (rb->rb_nsymbols == 0)
		return;

	fprintf(os, "%s symbol status:\n", rb->rb_name);

	for (unsigned int i = 0; i < rb->rb_nsymbols; i++, sym++, sv++) {
		bool opt, vol;

		opt = (sym->sym_flags & GDB_RTOS_SYM_FLAGS_OPTIONAL) != 0;
		vol = (sym->sym_flags & GDB_RTOS_SYM_FLAGS_VOLATILE) != 0;

		fprintf(os, "Symbol '%s'%s", sym->sym_name,
		    (opt || vol) ? " (" : "");
		if (opt)
			fprintf(os, "optional%s", vol ? ", " : ")");
		if (vol)
			fprintf(os, "volatile)");

		if (sv->sv_found == false)
			fprintf(os, ": Not found\n");
		else
			fprintf(os, ": 0x%" PRIxTADDR "\n", sv->sv_value);
	}

}

static void
gdb_rtos_cmd_support(FILE *os, const char *arg)
{
	target_rtos_type_t current_type, new_type;
	bool current_state, new_state, changed;
	size_t arglen;
	uint8_t x;

	current_state = gdb_rtos_supported(&current_type);
	new_state = current_state;
	new_type = current_type;

	arglen = strlen(arg);
	if (strncasecmp(arg, "auto", arglen) == 0) {
		new_state = true;
	} else
	if (strncasecmp(arg, "off", arglen) == 0) {
		new_state = false;
		new_type = TARGET_RTOS_NONE;
	} else {
		const struct gdb_rtos_backend *rb;

		if ((rb = gdb_rtos_find_backend_by_name(arg)) == NULL) {
			fprintf(os, "'%s' is not a recognised RTOS name\n",
			    arg);
			return;
		}

		new_state = true;
		new_type = rb->rb_type;
	}

	changed = false;

	if (new_state != current_state) {
		if (new_state == true) {
			if (new_type == TARGET_RTOS_NONE) {
				cf_del_uint8(CF_KEY_UINT8_RTOS_TYPE);
				changed = true;
			} else
			if (new_type != current_type) {
				x = (uint8_t)new_type;
				cf_set_uint8(CF_KEY_UINT8_RTOS_TYPE, x);
				changed = true;
			}
		} else {
			x = (uint8_t)TARGET_RTOS_NONE;
			cf_set_uint8(CF_KEY_UINT8_RTOS_TYPE, x);
			changed = true;
		}
	} else
	if (new_type != current_type) {
		x = (uint8_t)new_type;
		cf_set_uint8(CF_KEY_UINT8_RTOS_TYPE, x);
		changed = true;
	}

	if (changed)
		configdb_persist();
}

static void
gdb_rtos_cmd_init_syms(FILE *os, const char *arg)
{
	bool current_state, new_state;

	current_state = gdb_rtos_do_init_syms();
	new_state = current_state;

	if (strcasecmp(arg, "on") == 0)
		new_state = true;
	else if (strcasecmp(arg, "off") == 0)
		new_state = false;
	else {
		fputs("Expected 'on' or 'off'\n", os);
		return;
	}

	if (current_state != new_state) {
		if (new_state)
			cf_del_uint8(CF_KEY_UINT8_RTOS_INIT_SYMBOLS);
		else
			cf_set_uint8(CF_KEY_UINT8_RTOS_INIT_SYMBOLS, 0);

		configdb_persist();
	}
}

static gdb_rtos_params_t
gdb_rtos_parse_params(FILE *os, const struct gdb_rtos_backend *rb,
    gdb_rtos_params_t rp, uint8_t argc, const char * const *argv,
    bool *changed)
{
	const struct gdb_rtos_param_desc *pd;
	gdb_rtos_params_t new_rp;
	bool failed = false;

	if ((new_rp = zone_malloc(rb->rb_params_size)) == NULL) {
		fputs("Out of memory!\n", os);
		return NULL;
	}

	memcpy(new_rp, rp, rb->rb_params_size);

	for (; !failed && argc; argc--, argv++) {
		char *str, *tok, *brkt;

		if ((str = zone_strdup(*argv)) == NULL) {
			fputs("Out of memory!\n", os);
			zone_free(new_rp);
			return NULL;
		}

		for (tok = strtok_r(str, ",", &brkt);
		    tok != NULL;
		    tok = strtok_r(NULL, ",", &brkt)) {
			unsigned int idx, val;

			if (sscanf(tok, "%u:%u", &idx, &val) != 2) {
				fputs("Parameter format is Index:Value\n", os);
				failed = true;
				break;
			}

			if (idx >= rb->rb_nparams) {
				fprintf(os, "Parameter index must be in the "
				    "range 0 to %u inclusive\n",
				    rb->rb_nparams);
				failed = true;
				break;
			}

			pd = &rb->rb_params_desc[idx];
			switch (pd->pd_flags & GDB_RTOS_PARAM_TYPE_SIZEOF) {
			case GDB_RTOS_PARAM_TYPE_SIZEOF_POINTER:
				if (val != 2 && val != 4) {
					 fputs("Pointer size must be 2 or 4\n",
					     os);
					 failed = true;
				}
				break;

			case GDB_RTOS_PARAM_TYPE_SIZEOF_WORD:
				if (val != 2 && val != 4) {
					 fputs("Word size must be 2 or 4\n",
					     os);
					 failed = true;
				}
				break;

			case GDB_RTOS_PARAM_TYPE_BOOLEAN:
				if (val != 0 && val != 1) {
					 fputs("Boolean must be 0 or 1\n", os);
					 failed = true;
				}
				break;

			case GDB_RTOS_PARAM_TYPE_SIZEOF_STRUCT:
				if (val == 0) {
					 fputs("Struct size must be non-zero\n",
					     os);
					 failed = true;
					 break;
				}
				/*FALLTHROUGH*/

			default:
				if (val > GDB_RTOS_PARAM_MAX_VALUE) {
					fprintf(os, "Parameter value must be "
					    "in the range 0 to %u inclusive\n",
					    GDB_RTOS_PARAM_MAX_VALUE);
					failed = true;
				}
				break;
			}

			if (failed)
				break;

			new_rp->rp_val[idx] = (gdb_rtos_param_t)val;
		}

		zone_free(str);
	}

	if (failed) {
		zone_free(new_rp);
		return NULL;
	}

	pd = rb->rb_params_desc;
	for (unsigned int i = 0; i < rb->rb_nparams; i++) {
		bool is_config;

		is_config = (pd[i].pd_flags & GDB_RTOS_PARAM_FLAG_CONFIG) != 0;

		if (rp->rp_val[i] != new_rp->rp_val[i]) {
			*changed = true;
			if (!is_config &&
			    new_rp->rp_source < GDB_RTOS_PARAM_CUSTOM) {
				new_rp->rp_source = GDB_RTOS_PARAM_CUSTOM;
			}
		}
	}

	return new_rp;
}

static void
gdb_rtos_cmd_params(FILE *os, uint8_t argc, const char * const *argv)
{
	const struct gdb_rtos_backend *rb;
	struct gdb_rtos_state *rs;
	gdb_rtos_params_t rp, new_rp;
	bool save_flag, rtos_flag, update_custom;
	getopt_data getopt_state;
	int i;

	rb = NULL;
	rp = NULL;
	rs = NULL;

	i = gdb_server_get_instance();
	if (i >= 0) {
		/*
		 * We've been invoked via GDB's "monitor" command, so we
		 * may already have valid RTOS state. Note: because of
		 * this, we don't need to acquire gdb_rtos_state_mutex.
		 * It is not an error if state is NULL.
		 */
		assert(i < CONFIG_GDB_SERVERS);
		if ((rs = gdb_rtos_current_states[i]) != NULL) {
			rb = rs->rs_backend;
			rp = rs->rs_backend_params;
		}
	}

	save_flag = rtos_flag = false;
	memset(&getopt_state, 0, sizeof(getopt_state));
	while ((i = getopt_r(argc, (char * const *)(uintptr_t)argv, "r:s",
	    &getopt_state)) != EOF) {
		switch (i) {
		case 'r':
			/* RTOS name. */
			rb = gdb_rtos_find_backend_by_name(getopt_state.optarg);
			if (rb == NULL) {
				fprintf(os, "'%s' is not a recognised RTOS"
				    " name\n", getopt_state.optarg);
				return;
			}
			rtos_flag = true;
			break;

		case 's':
			save_flag = true;
			break;

		default:
			break;
		}
	}

	argc -= getopt_state.optind;
	argv += getopt_state.optind;

	if (argc > 1)
		fputs("Ignoring extra command line parameters\n", os);

	if (rb == NULL) {
		fputs("You must specify an RTOS using '-r <rtos-name>'\n", os);
		return;
	}

	/* Bail now if the specified RTOS does not have parameters. */
	if (rb->rb_nparams == 0) {
		fputs("This RTOS does not have cusomisable parameters.\n", os);
		return;
	}

	if (rp == NULL) {
		/*
		 * This can only happen if we're being invoked outside
		 * of a GDB task.
		 */
		assert(rs == NULL);

		/*
		 * There may be some custom/persistant params already.
		 * Otherwise, use the back-end's defaults.
		 */
		rp = gdb_rtos_get_back_end_params(rb);
		if (rp == NULL) {
			fputs("Out of memory retrieving RTOS params\n", os);
			return;
		}
	}

	/*
	 * 'rp' now points either to a saved copy of the RTOS params in
	 * gdb_rtos_custom_params[rb->rb_type], or the params used by
	 * the current GDB instance. Regardless, we're free to modify
	 * them.
	 */

	if (argc == 0) {
		/* Show current params. */
		gdb_rtos_show_params(os, rb, rp);
		return;
	}

	if (argc == 1 && strcasecmp(argv[0], "default") == 0) {
		new_rp = NULL;

		if (save_flag && rtos_flag) {
			/* Ensure there is no persistant copy. */
			cf_del_data(CF_KEY_DATA_RTOS_PARAMS(rb->rb_type));
		}
	} else {
		bool changed = false;

		new_rp = gdb_rtos_parse_params(os, rb, rp, argc, argv,
		    &changed);
		if (new_rp == NULL)
			return;

		if (!changed) {
			zone_free(new_rp);
			fputs("Unchanged.\n", os);
			return;
		}

		if (save_flag && new_rp->rp_source == GDB_RTOS_PARAM_CUSTOM)
			new_rp->rp_source = GDB_RTOS_PARAM_SAVED_CUSTOM;
	}

	rtos_mutex_acquire(gdb_rtos_state_mutex);

	if (new_rp != NULL)
		memcpy(rp, new_rp, rb->rb_params_size);
	else
		memcpy(rp, rb->rb_params_default, rb->rb_params_size);

	/*
	 * If an RTOS was supplied, ensure gdb_rtos_custom_params[]
	 * is updated, even if invoked in within GDB.
	 */
	update_custom = rtos_flag;

	if (new_rp != NULL) {
		if (save_flag) {
			cf_set_data(CF_KEY_DATA_RTOS_PARAMS(rb->rb_type),
			    rb->rb_params_size, new_rp);
			fputs("Saved.\n", os);
			update_custom = true;
		}
		zone_free(new_rp);
	} else {
		/* This implies we're restoring defaults. Show them. */
		gdb_rtos_show_params(os, rb, rp);
	}

	/* Keep gdb_rtos_custom_params[] in sync if required. */
	if (update_custom &&
	    gdb_rtos_custom_params[rb->rb_type] != NULL &&
	    gdb_rtos_custom_params[rb->rb_type] != rp) {
		memcpy(gdb_rtos_custom_params[rb->rb_type], rp,
		    rb->rb_params_size);
	}

	rtos_mutex_release(gdb_rtos_state_mutex);
}

static void
gdb_rtos_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	target_rtos_type_t rt;
	int gdb_instance;

	if (argc) {
		size_t arglen = strlen(argv[0]);

		if (strncasecmp(argv[0], "support", arglen) == 0 &&
		    argc == 2) {
			gdb_rtos_cmd_support(os, argv[1]);
		} else
		if (strncasecmp(argv[0], "init-syms", arglen) == 0 &&
		    argc == 2) {
			gdb_rtos_cmd_init_syms(os, argv[1]);
		} else
		if (strncasecmp(argv[0], "params", arglen) == 0) {
			gdb_rtos_cmd_params(os, argc, argv);
		} else {
			gdb_rtos_show_usage(os);
		}

		return;
	}

	fputs("RTOS support: ", os);
	if (gdb_rtos_supported(&rt)) {
		if (rt != TARGET_RTOS_NONE) {
			unsigned int be;
			be = gdb_rtos_find_backend_number_by_type(rt);
			assert(be < GDB_RTOS_BACKEND_COUNT);
			fprintf(os, "%s\n", gdb_rtos_backends[be]->rb_name);
		} else {
			fputs("Auto-detect\n", os);
		}
	} else {
		fputs("Off\n", os);
	}

	fprintf(os, "Initialise volatile symbols: O%s\n",
	    gdb_rtos_do_init_syms() ? "n" : "ff");

	gdb_instance = gdb_server_get_instance();
	if (gdb_instance >= 0) {
		struct gdb_rtos_state *rs;

		assert(gdb_instance < CONFIG_GDB_SERVERS);
		if ((rs = gdb_rtos_current_states[gdb_instance]) != NULL) {
			if (rs->rs_backend != NULL) {
				fprintf(os, "Current RTOS: %s\n",
				    rs->rs_backend->rb_name);
				gdb_rtos_show_params(os, rs->rs_backend,
				    rs->rs_backend_params);
				gdb_rtos_show_symbols(os, rs->rs_backend,
				    rs->rs_sym_values);
			} else {
				fputs("No RTOS support for this target\n", os);
			}
			return;
		}
	}

	fputs("List of supported RTOSes:\n", os);
	for (unsigned int i = 0; i < GDB_RTOS_BACKEND_COUNT; i++) {
		const struct gdb_rtos_backend *rb = gdb_rtos_backends[i];

		fprintf(os, "\tName: %s\n", rb->rb_name);
	}
}

void
gdb_rtos_load_defaults(void)
{

	for (unsigned int i = 0; i < GDB_RTOS_BACKEND_COUNT; i++) {
		cf_del_data(CF_KEY_DATA_RTOS_PARAMS(
		    gdb_rtos_backends[i]->rb_type));
	}

	cf_del_uint8(CF_KEY_UINT8_RTOS_TYPE);
	cf_del_uint8(CF_KEY_UINT8_RTOS_INIT_SYMBOLS);
}
