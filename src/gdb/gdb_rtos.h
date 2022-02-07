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

#ifndef GDB_RTOS_H
#define GDB_RTOS_H

#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "tmon.h"
#include "stringio.h"

/*
 * Symbols are provided by RTOS back-end code in an array of the following...
 * If all the mandatory symbols are found, the RTOS back-end is attached.
 */
struct gdb_rtos_symbol {
	const char *sym_name;
	uint32_t sym_flags;
	/* XXX: Future enhancement - sizeof(symbol) */
};
/* Don't prevent the RTOS back-end attaching if the symbol is not found. */
#define	GDB_RTOS_SYM_FLAGS_OPTIONAL	(1u << 0)
/* The symbol value references volatile memory (e.g. RAM). */
#define	GDB_RTOS_SYM_FLAGS_VOLATILE	(1u << 1)
#define	GDB_RTOS_SYMBOL_INIT(n,f)		\
	{.sym_name = n, .sym_flags = (uint32_t)(f)}

/*
 * The RTOS back-end attach function is passed a pointer to an array of
 * the following. This provides the values of all resolved symbols.
 */
struct gdb_rtos_symbol_value {
	target_addr_t sv_value;	/* Value of the symbol. */
	bool sv_found;		/* 'true' if the symbol lookup succeeded. */
};

/*
 * This is used for both sizeof and offsetof parameters, and so needs to be
 * wide enough to handle all supported RTOSes.
 * Don't make it too wide to avoid bloating the Config DB.
 */
typedef uint8_t gdb_rtos_param_t;
#define	PRIuRTPARAM			"u"
#define	PRIxRTPARAM			"x"
#define	GDB_RTOS_PARAM_MAX_VALUE	255u

/*
 * Flags for parameter lists.
 */
#define	GDB_RTOS_PARAM_TYPE_SIZEOF		(3u << 0)
#define	GDB_RTOS_PARAM_TYPE_SIZEOF_POINTER	(1u << 0)
#define	GDB_RTOS_PARAM_TYPE_SIZEOF_WORD		(2u << 0)
#define	GDB_RTOS_PARAM_TYPE_SIZEOF_STRUCT	(3u << 0)
#define	GDB_RTOS_PARAM_TYPE_OFFSETOF		(1u << 2)
#define	GDB_RTOS_PARAM_TYPE_CONSTANT		(1u << 3)
#define	GDB_RTOS_PARAM_TYPE_BOOLEAN		(1u << 4)
#define	GDB_RTOS_PARAM_FLAG_CONFIG		(1u << 5)

/*
 * RTOS parameters are described using this structure.
 */
struct gdb_rtos_param_desc {
	const char *pd_name;
	uint8_t pd_flags;
};
#define	GDB_RTOS_PARAM_DESC_INIT(n,f)	{.pd_name = n, .pd_flags = f}

enum gdb_rtos_param_source {
	GDB_RTOS_PARAM_DEFAULT,
	GDB_RTOS_PARAM_PROBED,
	GDB_RTOS_PARAM_CUSTOM,
	GDB_RTOS_PARAM_SAVED_CUSTOM,
};

/* Storage for RTOS params */
#define	GDB_RTOS_PARAMS_DEF(t,c)			\
	typedef struct {				\
		enum gdb_rtos_param_source rp_source;	\
		gdb_rtos_param_t rp_val[c];		\
	} t

/* Generic reference to the above, with zero-sized array of params. */
GDB_RTOS_PARAMS_DEF(*gdb_rtos_params_t,);

/*
 * RTOS threads are described thus
 */
struct gdb_rtos_thread {
	target_addr_t rt_tid;
	char rt_info[60];
};

#define	GDB_RTOS_MAX_THREADS		64	/* Seems a reasonable limit. */

struct gdb_rtos_backend {
	const char *rb_name;
	target_rtos_type_t rb_type;

	/* Symbols for this RTOS. */
	unsigned int rb_nsymbols;
	const struct gdb_rtos_symbol *rb_symbols;

	/* The number of RTOS parameters required. */
	unsigned int rb_nparams;
	/* Size of the RTOS-specific gdb_rtos_params_t. */
	size_t rb_params_size;
	/* Default set of parameters. */
	const gdb_rtos_params_t rb_params_default;
	/* Descriptions of each parameter. */
	const struct gdb_rtos_param_desc *rb_params_desc;

	/* Attach an RTOS back-end. */
	void *(*rb_attach)(tmon_cookie_t, const struct gdb_rtos_symbol_value *,
	    gdb_rtos_params_t);

	/* Free/detach an RTOS back-end. */
	void (*rb_free)(void *);

	/* Optional: Back-end method to clear volatile symbols. */
	void (*rb_reset)(void *);

	/* Fetch the list of target threads. */
	int (*rb_threads)(void *, struct gdb_rtos_thread **, target_addr_t *);

	/*
	 * Return the stack frame address associated with thread 'tid'.
	 * '*params' arg is passed as 'params' to tmon_rtos_set_frame().
	 */
	target_addr_t (*rb_get_frame)(void *, target_addr_t tid, void **params);
};

struct gdb_rtos_state;
typedef struct gdb_rtos_state *gdb_rtos_state_t;

extern void gdb_rtos_init(void);
extern gdb_rtos_state_t gdb_rtos_new(tmon_cookie_t, unsigned int);
extern void gdb_rtos_free(gdb_rtos_state_t);
extern bool gdb_rtos_detected(gdb_rtos_state_t);
extern void gdb_rtos_reset_symbols(gdb_rtos_state_t);

extern const char *gdb_rtos_qSymbol_first(gdb_rtos_state_t);
extern const char *gdb_rtos_qSymbol_next(gdb_rtos_state_t, bool, target_addr_t);

extern void gdb_rtos_update_threads(gdb_rtos_state_t);
extern void gdb_rtos_select_thread(gdb_rtos_state_t, target_addr_t);
extern const struct gdb_rtos_thread *gdb_rtos_thread_info(gdb_rtos_state_t,
		target_addr_t);
extern target_addr_t gdb_rtos_current_thread(gdb_rtos_state_t);
extern stringio_t gdb_rtos_thread_list(gdb_rtos_state_t);

extern bool gdb_rtos_params_changed(const gdb_rtos_params_t *pold,
		const gdb_rtos_params_t *pnew, unsigned int nparams,
		bool ignore_config);

extern void gdb_rtos_load_defaults(void);

#endif /* GDB_RTOS_H */
