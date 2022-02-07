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
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <setjmp.h>

#include "rtos.h"
#include "product.h"
#include "netusb_mux.h"
#include "gdb_server.h"
#include "gdb_signals.h"
#ifdef CONFIG_GDB_RTOS_SUPPORT
#include "gdb_rtos.h"
#endif
#include "tmon.h"
#include "target_comms.h"
#include "target.h"
#include "shell.h"
#include "timer.h"
#include "hexdump.h"
#ifdef CONFIG_USE_CONFIGDB
#include "configdb.h"
#endif
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
//#define	DEBUG_FLAG	gdb_server_debug
#include "debug.h"

#if (CONFIG_GDB_SERVERS == 0) || (CONFIG_GDB_SERVERS > 2)
#error "Invalid CONFIG_GDB_SERVERS"
#endif

/*
 * If necessary, use "set debug remote 1" in GDB to see its version of events.
 */

#define	GDB_SERVER_RINGBUFF_SIZE_TX	1500
#define	GDB_SERVER_RINGBUFF_SIZE_RX	1500

#define	GDB_SERVER_NETWORK_RX_TIMEOUT	50
#define	GDB_SERVER_NETWORK_TX_TIMEOUT	50

#define	GDB_SERVER_REQ_SIZE		1360

#define	GDB_SERVER_MAX_CMD_ARGS		16
#define	GDB_SERVER_OUTBUFF_LEN		64

/*
 * State related to our link with the host.
 */
struct gdb_server_link_state {
	/* Non-null whenever we're listening or actively communicating */
	netusb_mux_state_t gls_netusb_mux;

	/* Non-zero if we have an active connection */
	uint8_t gls_connected;

	/*
	 * Non-zero if our link to the host is no longer valid.
	 * This is set asynchronously via the target comms thread.
	 */
	volatile uint8_t gls_hangup;

	/* Ring buffers for host comms */
	ringbuff_t gls_to_host;
	ringbuff_t gls_from_host;
	ringbuff_blocking_t gls_to_host_blk;
	ringbuff_blocking_t gls_from_host_blk;
	uint8_t gls_rb_to_host[GDB_SERVER_RINGBUFF_SIZE_TX];
	uint8_t gls_rb_from_host[GDB_SERVER_RINGBUFF_SIZE_RX];
};

struct gdb_context {
	/* These fields initialised by the upper layer */
	struct gdb_server_link_state *gc_link;
	uint8_t *gc_buff;
	uint32_t gc_buff_len;
	uint32_t gc_buff_size;

	/* The remaining fields are private to gdb_server.c */
	unsigned int gc_instance;
	unsigned int gc_target_num;
	tmon_cookie_t gc_tmon;

	FILE *gc_ostream;
	u_int gc_olen;
	uint8_t gc_ocsum;
	uint8_t gc_kill_value;
	uint16_t gc_flags;
#define	GDB_FLAGS_TARGET_ATTACHED	(1u << 0)
#define	GDB_FLAGS_EXTENDED_MODE		(1u << 1)
#define	GDB_FLAGS_NO_ACK_MODE		(1u << 2)
#define	GDB_FLAGS_NO_RESPONSE		(1u << 3)
#define	GDB_FLAGS_KILLED		(1u << 4)
#define	GDB_FLAGS_EXITED		(1u << 5)
#define	GDB_FLAGS_CMD_EMPTY		(1u << 6)
#define	GDB_FLAGS_HAS_SWBREAK		(1u << 7)
#define	GDB_FLAGS_HAS_HWBREAK		(1u << 8)
#define	GDB_FLAGS_HAS_VCONT		(1u << 9)
#define	GDB_FLAGS_SEND_ACK		(1u << 10)

#ifdef CONFIG_GDB_RTOS_SUPPORT
	gdb_rtos_state_t gc_rtos;
#endif

	void *gc_reg_buff;
	u_int gc_reg_buff_len;

	void *gc_mem_buff;
	u_int gc_mem_buff_len;

	void *gc_hex_buff;
	u_int gc_hex_buff_len;

	uint8_t gc_cmd_outbuff[GDB_SERVER_OUTBUFF_LEN];
	const char *gc_cmd_argv[GDB_SERVER_MAX_CMD_ARGS];

	jmp_buf gc_jmp_buf;
	const char *gc_bail_str;

	stringio_t gc_xml_string;
};
typedef struct gdb_context *gdb_context_t;

struct gdb_server_state {
	struct gdb_server_link_state gs_link_state;
	rtos_task_t gs_task_handle;
	struct gdb_context gs_server_ctx;
	uint8_t gs_gdb_comms_buff[GDB_SERVER_REQ_SIZE];
	uint8_t gs_gdb_mem_buff[GDB_SERVER_REQ_SIZE / 2];
	uint8_t gs_gdb_hex_buff[GDB_SERVER_REQ_SIZE];
};
struct gdb_server_state gdb_server_state[CONFIG_GDB_SERVERS];

#define	GDB_SERVER_EVENT_NETUSB_MUX_POLL	(1u << 0)
#define	GDB_SERVER_EVENT_TARGET			(1u << 1)

static int gdb_server_link_write(struct gdb_server_link_state *, const void *,
	    uint32_t);
static int gdb_server_link_read(struct gdb_server_link_state *, void *,
	    uint32_t);
static bool gdb_server_link_up(struct gdb_server_link_state *);
static ringbuff_len_t gdb_server_link_poll(struct gdb_server_link_state *);

#define	GDB_CMD_CODE(x)			((x) & 0x3fu)
#define	GDB_CMD_ATTACHED		(1u << 6)
#define	GDB_CMD_FIO			(1u << 7)

#define	GDB_CMD_NOOP			GDB_CMD_CODE(0)
static void gdb_cmd_noop(gdb_context_t, uint32_t);
#define	GDB_CMD_IGNORED			GDB_CMD_CODE(1)
static void gdb_cmd_ignored(gdb_context_t, uint32_t);
#define	GDB_CMD_DETACH			GDB_CMD_CODE(2)
static void gdb_cmd_detach(gdb_context_t, uint32_t);
#define	GDB_CMD_ENABLE_EXT_MODE		GDB_CMD_CODE(3)
static void gdb_cmd_enable_ext_mode(gdb_context_t, uint32_t);
#define	GDB_CMD_QUERY_HALT		GDB_CMD_CODE(4)
static void gdb_cmd_query_halt(gdb_context_t, uint32_t);
#define	GDB_CMD_SET_ARGS		GDB_CMD_CODE(5)
static void gdb_cmd_set_args(gdb_context_t, uint32_t);
#define	GDB_CMD_CONTINUE_SIG		GDB_CMD_CODE(7)
static void gdb_cmd_continue_sig(gdb_context_t, uint32_t);
#define	GDB_CMD_FILEIO			(GDB_CMD_CODE(8) | GDB_CMD_FIO)
static void gdb_cmd_fileio(gdb_context_t, uint32_t);
#define	GDB_CMD_CONTINUE		GDB_CMD_CODE(9)
static void gdb_cmd_continue(gdb_context_t, uint32_t);
#define	GDB_CMD_READ_ALL_REGS		(GDB_CMD_CODE(11) | GDB_CMD_ATTACHED)
static void gdb_cmd_read_all_regs(gdb_context_t, uint32_t);
#define	GDB_CMD_WRITE_ALL_REGS		(GDB_CMD_CODE(12) | GDB_CMD_ATTACHED)
static void gdb_cmd_write_all_regs(gdb_context_t, uint32_t);
#define	GDB_CMD_SET_THREAD		GDB_CMD_CODE(13)
static void gdb_cmd_set_thread(gdb_context_t, uint32_t);
#define	GDB_CMD_STEP_CYCLE		GDB_CMD_CODE(14)
static void gdb_cmd_step_cycle(gdb_context_t, uint32_t);
#define	GDB_CMD_SIG_CYCLE		GDB_CMD_CODE(15)
static void gdb_cmd_sig_cycle(gdb_context_t, uint32_t);
#define	GDB_CMD_KILL_TARGET		GDB_CMD_CODE(16)
static void gdb_cmd_kill_target(gdb_context_t, uint32_t);
#define	GDB_CMD_READ_MEM		(GDB_CMD_CODE(17) | GDB_CMD_FIO | \
					 GDB_CMD_ATTACHED)
static void gdb_cmd_read_mem(gdb_context_t, uint32_t);
#define	GDB_CMD_WRITE_MEM		(GDB_CMD_CODE(18) | GDB_CMD_FIO | \
					 GDB_CMD_ATTACHED)
static void gdb_cmd_write_mem(gdb_context_t, uint32_t);
#define	GDB_CMD_READ_REG		(GDB_CMD_CODE(19) | GDB_CMD_ATTACHED)
static void gdb_cmd_read_reg(gdb_context_t, uint32_t);
#define	GDB_CMD_WRITE_REG		(GDB_CMD_CODE(20) | GDB_CMD_ATTACHED)
static void gdb_cmd_write_reg(gdb_context_t, uint32_t);
#define	GDB_CMD_QUERY			GDB_CMD_CODE(21)
static void gdb_cmd_query(gdb_context_t, uint32_t);
#define	GDB_CMD_SET			GDB_CMD_CODE(22)
static void gdb_cmd_set(gdb_context_t, uint32_t);
#define	GDB_CMD_RESTART			GDB_CMD_CODE(24)
static void gdb_cmd_restart(gdb_context_t, uint32_t);
#define	GDB_CMD_STEP			GDB_CMD_CODE(25)
static void gdb_cmd_step(gdb_context_t, uint32_t);
#define	GDB_CMD_SIG_STEP		GDB_CMD_CODE(26)
static void gdb_cmd_sig_step(gdb_context_t, uint32_t);
#define	GDB_CMD_REVERSE_SEARCH		GDB_CMD_CODE(27)
static void gdb_cmd_reverse_search(gdb_context_t, uint32_t);
#define	GDB_CMD_THREAD_STATE		GDB_CMD_CODE(28)
static void gdb_cmd_thread_state(gdb_context_t, uint32_t);
#define	GDB_CMD_V_PACKET		GDB_CMD_CODE(29)
static void gdb_cmd_v_packet(gdb_context_t, uint32_t);
#define	GDB_CMD_WRITE_BUFFER		(GDB_CMD_CODE(30) | GDB_CMD_FIO | \
					 GDB_CMD_ATTACHED)
static void gdb_cmd_write_buffer(gdb_context_t, uint32_t);
#define	GDB_CMD_Z_PACKET		(GDB_CMD_CODE(31) | GDB_CMD_ATTACHED)
static void gdb_cmd_z_packet(gdb_context_t, uint32_t);

typedef void (*gdb_cmd_handler_t)(gdb_context_t, uint32_t);

static const gdb_cmd_handler_t gdb_cmd_dispatch[] = {
	[GDB_CMD_CODE(GDB_CMD_NOOP)] = gdb_cmd_noop,
	[GDB_CMD_CODE(GDB_CMD_IGNORED)] = gdb_cmd_ignored,
	[GDB_CMD_CODE(GDB_CMD_DETACH)] = gdb_cmd_detach,
	[GDB_CMD_CODE(GDB_CMD_ENABLE_EXT_MODE)] = gdb_cmd_enable_ext_mode,
	[GDB_CMD_CODE(GDB_CMD_QUERY_HALT)] = gdb_cmd_query_halt,
	[GDB_CMD_CODE(GDB_CMD_SET_ARGS)] = gdb_cmd_set_args,
	[GDB_CMD_CODE(GDB_CMD_CONTINUE_SIG)] = gdb_cmd_continue_sig,
	[GDB_CMD_CODE(GDB_CMD_FILEIO)] = gdb_cmd_fileio,
	[GDB_CMD_CODE(GDB_CMD_CONTINUE)] = gdb_cmd_continue,
	[GDB_CMD_CODE(GDB_CMD_READ_ALL_REGS)] = gdb_cmd_read_all_regs,
	[GDB_CMD_CODE(GDB_CMD_WRITE_ALL_REGS)] = gdb_cmd_write_all_regs,
	[GDB_CMD_CODE(GDB_CMD_SET_THREAD)] = gdb_cmd_set_thread,
	[GDB_CMD_CODE(GDB_CMD_STEP_CYCLE)] = gdb_cmd_step_cycle,
	[GDB_CMD_CODE(GDB_CMD_SIG_CYCLE)] = gdb_cmd_sig_cycle,
	[GDB_CMD_CODE(GDB_CMD_KILL_TARGET)] = gdb_cmd_kill_target,
	[GDB_CMD_CODE(GDB_CMD_READ_MEM)] = gdb_cmd_read_mem,
	[GDB_CMD_CODE(GDB_CMD_WRITE_MEM)] = gdb_cmd_write_mem,
	[GDB_CMD_CODE(GDB_CMD_READ_REG)] = gdb_cmd_read_reg,
	[GDB_CMD_CODE(GDB_CMD_WRITE_REG)] = gdb_cmd_write_reg,
	[GDB_CMD_CODE(GDB_CMD_QUERY)] = gdb_cmd_query,
	[GDB_CMD_CODE(GDB_CMD_SET)] = gdb_cmd_set,
	[GDB_CMD_CODE(GDB_CMD_RESTART)] = gdb_cmd_restart,
	[GDB_CMD_CODE(GDB_CMD_STEP)] = gdb_cmd_step,
	[GDB_CMD_CODE(GDB_CMD_SIG_STEP)] = gdb_cmd_sig_step,
	[GDB_CMD_CODE(GDB_CMD_REVERSE_SEARCH)] = gdb_cmd_reverse_search,
	[GDB_CMD_CODE(GDB_CMD_THREAD_STATE)] = gdb_cmd_thread_state,
	[GDB_CMD_CODE(GDB_CMD_V_PACKET)] = gdb_cmd_v_packet,
	[GDB_CMD_CODE(GDB_CMD_WRITE_BUFFER)] = gdb_cmd_write_buffer,
	[GDB_CMD_CODE(GDB_CMD_Z_PACKET)] = gdb_cmd_z_packet,
};

static const uint8_t gdb_cmd_index[128] = {
/*00*/	GDB_CMD_NOOP,		/*01*/	GDB_CMD_NOOP,
/*02*/	GDB_CMD_NOOP,		/*03*/	GDB_CMD_NOOP,
/*04*/	GDB_CMD_DETACH,		/*05*/	GDB_CMD_NOOP,
/*06*/	GDB_CMD_NOOP,		/*07*/	GDB_CMD_NOOP,
/*08*/	GDB_CMD_NOOP,		/*09*/	GDB_CMD_NOOP,
/*0a*/	GDB_CMD_NOOP,		/*0b*/	GDB_CMD_NOOP,
/*0c*/	GDB_CMD_NOOP,		/*0d*/	GDB_CMD_NOOP,
/*0e*/	GDB_CMD_NOOP,		/*0f*/	GDB_CMD_NOOP,
/*10*/	GDB_CMD_NOOP,		/*11*/	GDB_CMD_NOOP,
/*12*/	GDB_CMD_NOOP,		/*13*/	GDB_CMD_NOOP,
/*14*/	GDB_CMD_NOOP,		/*15*/	GDB_CMD_NOOP,
/*16*/	GDB_CMD_NOOP,		/*17*/	GDB_CMD_NOOP,
/*18*/	GDB_CMD_NOOP,		/*19*/	GDB_CMD_NOOP,
/*1a*/	GDB_CMD_NOOP,		/*1b*/	GDB_CMD_NOOP,
/*1c*/	GDB_CMD_NOOP,		/*1d*/	GDB_CMD_NOOP,
/*1e*/	GDB_CMD_NOOP,		/*1f*/	GDB_CMD_NOOP,
/*   */	GDB_CMD_NOOP,		/* ! */	GDB_CMD_ENABLE_EXT_MODE,
/* " */	GDB_CMD_NOOP,		/* # */	GDB_CMD_NOOP,
/* $ */	GDB_CMD_NOOP,		/* % */	GDB_CMD_NOOP,
/* & */	GDB_CMD_NOOP,		/* ' */	GDB_CMD_NOOP,
/* ( */	GDB_CMD_NOOP,		/* ) */	GDB_CMD_NOOP,
/* * */	GDB_CMD_NOOP,		/* + */	GDB_CMD_NOOP,
/* , */	GDB_CMD_NOOP,		/* - */	GDB_CMD_NOOP,
/* . */	GDB_CMD_NOOP,		/* / */	GDB_CMD_NOOP,
/* 0 */	GDB_CMD_NOOP,		/* 1 */	GDB_CMD_NOOP,
/* 2 */	GDB_CMD_NOOP,		/* 3 */	GDB_CMD_NOOP,
/* 4 */	GDB_CMD_NOOP,		/* 5 */	GDB_CMD_NOOP,
/* 6 */	GDB_CMD_NOOP,		/* 7 */	GDB_CMD_NOOP,
/* 8 */	GDB_CMD_NOOP,		/* 9 */	GDB_CMD_NOOP,
/* : */	GDB_CMD_NOOP,		/* ; */	GDB_CMD_NOOP,
/* < */	GDB_CMD_NOOP,		/* = */	GDB_CMD_NOOP,
/* > */	GDB_CMD_NOOP,		/* ? */	GDB_CMD_QUERY_HALT,
/* @ */	GDB_CMD_NOOP,		/* A */	GDB_CMD_NOOP /*GDB_CMD_SET_ARGS*/,
/* B */	GDB_CMD_IGNORED,	/* C */	GDB_CMD_CONTINUE_SIG,
/* D */	GDB_CMD_DETACH,		/* E */	GDB_CMD_NOOP,
/* F */	GDB_CMD_FILEIO,		/* G */	GDB_CMD_WRITE_ALL_REGS,
/* H */	GDB_CMD_SET_THREAD,	/* I */	GDB_CMD_SIG_CYCLE,
/* J */	GDB_CMD_NOOP,		/* K */	GDB_CMD_NOOP,
/* L */	GDB_CMD_NOOP,		/* M */	GDB_CMD_WRITE_MEM,
/* N */	GDB_CMD_NOOP,		/* O */	GDB_CMD_NOOP,
/* P */	GDB_CMD_WRITE_REG,	/* Q */	GDB_CMD_SET,
/* R */	GDB_CMD_RESTART,	/* S */	GDB_CMD_SIG_STEP,
/* T */	GDB_CMD_THREAD_STATE,	/* U */	GDB_CMD_NOOP,
/* V */	GDB_CMD_NOOP,		/* W */	GDB_CMD_NOOP,
/* X */	GDB_CMD_WRITE_BUFFER,	/* Y */	GDB_CMD_NOOP,
/* Z */	GDB_CMD_Z_PACKET,	/* [ */	GDB_CMD_NOOP,
/* \ */	GDB_CMD_NOOP,		/* ] */	GDB_CMD_NOOP,
/* ^ */	GDB_CMD_NOOP,		/* _ */	GDB_CMD_NOOP,
/* ` */	GDB_CMD_NOOP,		/* a */	GDB_CMD_NOOP,
/* b */	GDB_CMD_IGNORED,	/* c */	GDB_CMD_CONTINUE,
/* d */	GDB_CMD_IGNORED,	/* e */	GDB_CMD_NOOP,
/* f */	GDB_CMD_NOOP,		/* g */	GDB_CMD_READ_ALL_REGS,
/* h */	GDB_CMD_NOOP,		/* i */	GDB_CMD_STEP_CYCLE,
/* j */	GDB_CMD_NOOP,		/* k */	GDB_CMD_KILL_TARGET,
/* l */	GDB_CMD_NOOP,		/* m */	GDB_CMD_READ_MEM,
/* n */	GDB_CMD_NOOP,		/* o */	GDB_CMD_NOOP,
/* p */	GDB_CMD_READ_REG,	/* q */	GDB_CMD_QUERY,
/* r */	GDB_CMD_IGNORED,	/* s */	GDB_CMD_STEP,
/* t */	GDB_CMD_REVERSE_SEARCH,	/* u */	GDB_CMD_NOOP,
/* v */	GDB_CMD_V_PACKET,	/* w */	GDB_CMD_NOOP,
/* x */	GDB_CMD_NOOP,		/* y */	GDB_CMD_NOOP,
/* z */	GDB_CMD_Z_PACKET,	/* { */	GDB_CMD_NOOP,
/* | */	GDB_CMD_NOOP,		/* } */	GDB_CMD_NOOP,
/* ~ */	GDB_CMD_NOOP,		/*del*/	GDB_CMD_NOOP,
};

/*
 * High-level response codes.
 */
#define	GDB_RESP_OK		"OK"
#define	GDB_RESP_DEAD		"EFF"

#define	GDB_RESP_EPERM		"E01"
#define	GDB_RESP_ENOENT		"E02"
#define	GDB_RESP_ESRCH		"E03"
#define	GDB_RESP_EINTR		"E04"
#define	GDB_RESP_EIO		"E05"
#define	GDB_RESP_ENXIO		"E06"
#define	GDB_RESP_E2BIG		"E07"
#define	GDB_RESP_ENOEXEC	"E08"
#define	GDB_RESP_EBADF		"E09"
#define	GDB_RESP_ECHILD		"E0A"
#define	GDB_RESP_EAGAIN		"E0B"
#define	GDB_RESP_ENOMEM		"E0C"
#define	GDB_RESP_EACCESS	"E0D"
#define	GDB_RESP_EFAULT		"E0E"
#define	GDB_RESP_ENOTBLK	"E0F"
#define	GDB_RESP_EBUSY		"E10"
#define	GDB_RESP_EEXIST		"E11"
#define	GDB_RESP_EXDEV		"E12"
#define	GDB_RESP_ENODEV		"E13"
#define	GDB_RESP_ENOTDIR	"E14"
#define	GDB_RESP_EISDIR		"E15"
#define	GDB_RESP_EINVAL		"E16"
#define	GDB_RESP_ENFILE		"E17"
#define	GDB_RESP_EMFILE		"E18"
#define	GDB_RESP_ENOTTY		"E19"
#define	GDB_RESP_ETXTBSY	"E1A"
#define	GDB_RESP_EFBIG		"E1B"
#define	GDB_RESP_ENOSPC		"E1C"
#define	GDB_RESP_ESPIPE		"E1D"
#define	GDB_RESP_EROFS		"E1E"
#define	GDB_RESP_EMLINK		"E1F"
#define	GDB_RESP_EPIPE		"E20"

struct gdb_string_cmd {
	const char *sc_string;
	uint16_t sc_string_len;
	uint16_t sc_flags;
	void (*sc_handler)(gdb_context_t, const uint8_t *, uint32_t);
};
#define	STR_CMD(s,h)	{.sc_string = (s), \
			 .sc_string_len = (uint16_t)(sizeof(s) - 1), \
			 .sc_flags = 0, \
			 .sc_handler = (h)}
#define	STR_CMD_F(s,h,f)	{.sc_string = (s), \
			 .sc_string_len = (uint16_t)(sizeof(s) - 1), \
			 .sc_flags = (f), \
			 .sc_handler = (h)}
#define	STR_CMD_END	{NULL, 0, 0, NULL}

static void gdb_server_loop(gdb_context_t, bool);

static void
bin_to_hex(const uint8_t *src, uint8_t *hex, uint32_t len)
{
	static uint8_t hexit[] = "0123456789abcdef";
	uint8_t ch;

	while (len--) {
		ch = *src++;
		*hex++ = hexit[ch >> 4];
		*hex++ = hexit[ch & 0xfu];
	}
}

static uint8_t
from_hex_digit(uint8_t ch)
{

	ch -= (uint8_t)'0';

	if (ch > 9)
		ch -= (uint8_t)'A' - (uint8_t)'0' - 10u;

	if (ch > 16)
		ch -= (uint8_t)'a' - (uint8_t)'A';

	return ch;
}

static uint32_t
hex_to_bin(const char *hex, uint8_t *dest, uint32_t len)
{
	uint32_t rv;
	uint8_t ch;

	len /= 2;
	rv = len;

	while (hex[0] != '\0' && hex[1] != '\0' && len) {
		len -= 1;
		ch = from_hex_digit((uint8_t)*hex++) << 4;
		*dest++ = ch | from_hex_digit((uint8_t)*hex++);
	}

	return rv - len;
}

#ifdef CONFIG_GDB_RTOS_SUPPORT
static const uint8_t *
hex_to_uint64(const uint8_t *hex, uint64_t *pv)
{
	uint64_t rv = 0;

	while (*hex != '\0' && isxdigit(*hex)) {
		rv <<= 4;
		rv |= from_hex_digit(*hex++);
	}

	*pv = rv;

	return hex;
}
#endif /* CONFIG_GDB_RTOS_SUPPORT */

static void
gdb_write_character(gdb_context_t gc, uint8_t ch)
{

	if (gdb_server_link_write(gc->gc_link, &ch, 1) != 1) {
		gc->gc_bail_str =  "gdb_write_character: write failed";
		longjmp(gc->gc_jmp_buf, 1);
	}
}

static uint8_t
gdb_read_character(gdb_context_t gc, bool blocking)
{
	uint8_t ch;

	if (gdb_server_link_up(gc->gc_link) == false) {
 bail:
		gc->gc_bail_str =  "gdb_read_character: failed";
		longjmp(gc->gc_jmp_buf, 1);
	}

	if (!blocking && gdb_server_link_poll(gc->gc_link) == 0)
		return 0;

	if (gdb_server_link_read(gc->gc_link, &ch, 1) != 1)
		goto bail;

	return ch;
}

static uint32_t
gdb_read_packet(gdb_context_t gc)
{
	uint8_t ch, csum, rxsum, *rd, *wr;
	uint32_t len;
	enum {
		RPS_WAIT_START,
		RPS_PAYLOAD,
		RPS_ESCAPE,
		RPS_CSUM0,
		RPS_CSUM1,
		RPS_DONE,
		RPS_CTRLD
	} read_packet_state;

	read_packet_state = RPS_WAIT_START;
	csum = rxsum = 0;
	wr = rd = gc->gc_buff;
	len = 0;

	do {
		if (len == 0) {
			if (!gdb_server_link_up(gc->gc_link)) {
				gc->gc_bail_str = "gdb_read_packet: link down";
				longjmp(gc->gc_jmp_buf, 1);
			}

			/* How many bytes are available? */
			len = gdb_server_link_poll(gc->gc_link);

			/*
			 * If none, arrange to block until at least one is
			 * available.
			 */
			if (len == 0)
				len = 1;

			rd = wr;
			if ((rd + len) > &gc->gc_buff[gc->gc_buff_size]) {
				gc->gc_bail_str = "gdb_read_packet: buffer "
				    "overflow";
				longjmp(gc->gc_jmp_buf, 1);
			}

			/* Fetch some data from the remote. */
			if (gdb_server_link_read(gc->gc_link, rd, len) < 0) {
				gc->gc_bail_str =  "gdb_read_packet: failed";
				longjmp(gc->gc_jmp_buf, 1);
			}
		}

		len -= 1;
		ch = *rd++;

		if (ch == '#') {
			read_packet_state = RPS_CSUM0;
			continue;
		}

		switch (read_packet_state) {
		case RPS_PAYLOAD:
			csum += ch;
			if (ch == '$') {
				/* Start over. */
				wr = gc->gc_buff;
				csum = 0;
			} else
			if (ch == '}') {
				read_packet_state = RPS_ESCAPE;
			} else {
				*wr++ = ch;
			}
			break;

		case RPS_ESCAPE:
			csum += ch;
			*wr++ = ch ^ 0x20u;
			read_packet_state = RPS_PAYLOAD;
			break;

		case RPS_WAIT_START:
			if (ch == '$') {
				read_packet_state = RPS_PAYLOAD;
				wr = gc->gc_buff;
				csum = 0;
			} else
			if (ch == 0x04u) {
				wr = gc->gc_buff;
				*wr++ = ch;
				/* Force the loop to exit. */
				read_packet_state = RPS_CTRLD;
			}
			break;

		case RPS_CSUM0:
			rxsum = from_hex_digit(ch) << 4;
			read_packet_state = RPS_CSUM1;
			break;

		case RPS_CSUM1:
			rxsum |= from_hex_digit(ch);
			if (csum == rxsum && len == 0) {
				read_packet_state = RPS_DONE;
			} else {
				/*
				 * Either checksums don't match, or we've
				 * received some extra data from Gdb. In
				 * either case, NACK if required, and restart
				 * from the beginning.
				 * In the case of slow transfers, we can
				 * receive a duplicate packet due to retries,
				 * so the check for 'len == 0' ensures, in
				 * most cases, that we ignore the first copy.
				 */
#if (RELEASE_BUILD == 0)
				printf("read_packet: %lu bytes left\n", len);
#endif

				if ((gc->gc_flags & GDB_FLAGS_NO_ACK_MODE) == 0)
					gdb_write_character(gc, '-');

				read_packet_state = RPS_WAIT_START;
			}
			break;

		default:
			assert(0);
			break;
		}
	} while (read_packet_state < RPS_DONE);

	if (read_packet_state != RPS_CTRLD &&
	    (gc->gc_flags & GDB_FLAGS_NO_ACK_MODE) == 0) {
		/*
		 * The ACK will be sent as the first character of our
		 * reply, if necessary.
		 */
		gc->gc_flags |= GDB_FLAGS_SEND_ACK;
	}

	/* Terminate the received packet and record its length. */
	*wr = 0;
	gc->gc_buff_len = wr - gc->gc_buff;

	return gc->gc_buff_len;
}

static int
gdb_ostream_write(void *arg, const char *buff, int buflen)
{
	gdb_context_t gc = arg;
	uint8_t ch, csum, *dest;
	int rv = buflen;
	u_int len;

	csum = gc->gc_ocsum;
	len = gc->gc_olen;
	dest = &gc->gc_buff[len];

	if (len == 0) {
		if ((gc->gc_flags & GDB_FLAGS_SEND_ACK) != 0) {
			gc->gc_flags &= ~GDB_FLAGS_SEND_ACK;
			*dest++ = '+';
			len++;
		}

		*dest++ = '$';
		len++;
	}

	while (buflen--) {
		if ((len + 2) >= gc->gc_buff_size)
			break;

		ch = (uint8_t) *buff++;
		csum += ch;

		switch (ch) {
		case '$':
		case '#':
		case '*':
		case '}':
			csum += (uint8_t)'}';
			*dest++ = '}';
			len++;
			ch ^= 0x20u;
			/*FALLTHROUGH*/

		default:
			*dest++ = ch;
			len++;
			break;
		}
	}

	gc->gc_ocsum = csum;
	gc->gc_olen = len;

	return rv;
}

static void
gdb_flush_packet(gdb_context_t gc)
{
	int retry, expect_ack;
	u_int len;
	uint8_t ch;

	fflush(gc->gc_ostream);

	if (gc->gc_flags & GDB_FLAGS_NO_RESPONSE) {
		gc->gc_flags &= ~GDB_FLAGS_NO_RESPONSE;
		goto done;
	}

	if ((len = gc->gc_olen) == 0) {
		if ((gc->gc_flags & GDB_FLAGS_SEND_ACK) != 0) {
			gc->gc_flags &= ~GDB_FLAGS_SEND_ACK;
			gc->gc_buff[len++] = (uint8_t)'+';
			gc->gc_buff[len++] = (uint8_t)'$';
		} else {
			gc->gc_buff[len++] = '$';
		}
	}

	gc->gc_buff[len++] = '#';
	bin_to_hex(&gc->gc_ocsum, &gc->gc_buff[len], 1);
	len += 2;

#ifdef DEBUG_ENABLED
	if (DEBUG_FLAG > 1)
		hexdump(NULL, "GDB-TX ", 16, gc->gc_buff, len);
#endif

	expect_ack = (gc->gc_buff[0] == (uint8_t)'+');
	if (!expect_ack)
		ch = '+';

	retry = 4;

	do {
		if (gdb_server_link_write(gc->gc_link, gc->gc_buff, len) != (int)len) {
			gc->gc_bail_str = "gdb_flush_packet: write failed";
			longjmp(gc->gc_jmp_buf, 1);
		}
		if (expect_ack)
			ch = gdb_read_character(gc, true);
	} while (ch != (uint8_t)'+' && --retry);

 done:
	gc->gc_olen = 0;
	gc->gc_ocsum = 0;
}

static void
gdb_send_hex_buff(gdb_context_t gc, const void *src, size_t len)
{

	assert((len * 2u) <= gc->gc_hex_buff_len);

	bin_to_hex(src, gc->gc_hex_buff, len);
	fwrite(gc->gc_hex_buff, 2, len, gc->gc_ostream);
}

static bool
gdb_target_in_use(gdb_context_t gc)
{
#if (CONFIG_GDB_SERVERS > 1)
	/*
	 * Don't permit multiple Gdb server instances to attach to the
	 * same target.
	 *
	 * XXX: Yes, there's a race condition here if two or more instances
	 * run through the code simultaneously, but the window is tiny.
	 */
	for (unsigned int i = 0; i < CONFIG_GDB_SERVERS; i++) {
		gdb_context_t ogc = &gdb_server_state[i].gs_server_ctx;

		if (gc == ogc)
			continue;

		if ((ogc->gc_flags & GDB_FLAGS_TARGET_ATTACHED) != 0 &&
		    gc->gc_target_num == ogc->gc_target_num) {
			return true;
		}
	}
#else
	(void) gc;
#endif

	return false;
}

static bool
gdb_should_auto_attach(gdb_context_t gc)
{

	/*
	 * Can't auto-attach if the target is already in use, usually
	 * by another Gdb server instance if supported by the probe.
	 */
	if (gdb_target_in_use(gc))
		return false;

#ifndef CONFIG_USE_CONFIGDB
	/*
	 * Don't auto-attach if there are multiple targets.
	 */
	return tmon_get_target_count() == 1;
#else
	/*
	 * Rely on a configuration parameter to control auto-attach.
	 */
	uint8_t auto_attach;

	if (cf_get_uint8(CF_KEY_UINT8_ATTACH_AUTO, &auto_attach) == 0)
		auto_attach = 1;

	return auto_attach != 0;
#endif
}

static int
gdb_attached_to_target(gdb_context_t gc, int do_attach)
{
	int rv;

	DBFPRINTF("do_attach %d\n", do_attach);

	if ((gc->gc_flags & GDB_FLAGS_TARGET_ATTACHED) == 0 &&
	    gdb_target_in_use(gc)) {
		/*
		 * Another Gdb server instance has already
		 * attached to this target.
		 */
		return TMON_QA_ERROR;
	}

	rv = tmon_query_attach(&gc->gc_tmon, gc->gc_target_num, do_attach);

	if (rv != TMON_QA_ATTACHED) {
		/*
		 * Fail. If we were already attached, mark as detached.
		 */
		if ((gc->gc_flags & GDB_FLAGS_TARGET_ATTACHED) != 0) {
			gc->gc_flags &= ~GDB_FLAGS_TARGET_ATTACHED;
			gdb_server_syscall_hook(gc->gc_instance, gc->gc_tmon,
			    NULL);
		}
	} else
	if (do_attach && rv == TMON_QA_ATTACHED &&
	    (gc->gc_flags & GDB_FLAGS_TARGET_ATTACHED) == 0) {
		gc->gc_flags |= GDB_FLAGS_TARGET_ATTACHED;
		gdb_server_syscall_hook(gc->gc_instance, gc->gc_tmon, gc);
#ifdef CONFIG_GDB_RTOS_SUPPORT
		if (gc->gc_rtos != NULL)
			gdb_rtos_update_threads(gc->gc_rtos);
#endif
	}

	return rv;
}

static void
gdb_server(gdb_context_t gc)
{

	gc->gc_ostream = fwopen(gc, gdb_ostream_write);
	if (gc->gc_ostream == NULL)
		return;
	gc->gc_ostream->_flags |= __SMBF;

	if (setjmp(gc->gc_jmp_buf) == 0)
		gdb_server_loop(gc, false);
	else
		DBFPRINTF("bailed: '%s'\n", gc->gc_bail_str);

	if (gdb_attached_to_target(gc, false) == TMON_QA_ATTACHED) {
		/*
		 * If GDB exited while programming Flash (user hit ^C)
		 * then the Flash driver needs to perform some cleanup
		 * operations.
		 *
		 * Invoking this unconditionally is fine; the drivers
		 * will ensure its a no-op if not required.
		 *
		 * XXX: For multi-core, this should be done on the
		 * primary target only.
		 */
		tmon_flash_done(gc->gc_tmon);

		/* Detach from and reset the core. */
		gdb_server_syscall_hook(gc->gc_instance, gc->gc_tmon, NULL);
		tmon_detach(gc->gc_tmon);
		tmon_reset(gc->gc_tmon, TARGET_RESET_SOFT_NO_HALT);
	}

#ifdef CONFIG_GDB_RTOS_SUPPORT
	if (gc->gc_rtos != NULL) {
		gdb_rtos_free(gc->gc_rtos);
		gc->gc_rtos = NULL;
	}
#endif /* CONFIG_GDB_RTOS_SUPPORT */

	if (gc->gc_reg_buff != NULL) {
		zone_free(gc->gc_reg_buff);
		gc->gc_reg_buff = NULL;
	}

	if (gc->gc_xml_string != NULL) {
		stringio_done(gc->gc_xml_string);
		gc->gc_xml_string = NULL;
	}

	fclose(gc->gc_ostream);
}

static void
gdb_server_loop(gdb_context_t gc, bool syscall)
{
#undef GDB_TIMED_SERVER_LOOP
#ifdef GDB_TIMED_SERVER_LOOP
	struct timeval tv_start, tv_stop, tv_diff;
	static struct timeval tv_last;
#endif
	uint32_t plen;
	uint8_t cmd;

	DBFPRINTF("%s session.\n", syscall ? "Syscall" : "Primary");

	for (;;) {
#ifdef GDB_TIMED_SERVER_LOOP
		if (!syscall) {
			timer_get_mono_time(&tv_start);
			timer_sub(&tv_start, &tv_last, &tv_diff);
			if (tv_diff.tv_sec >= 1)
				putchar('\n');
			tv_last = tv_start;
			printf("server_loop(%u.%06u %7lu): ",
			    (unsigned int)tv_diff.tv_sec,
			    (unsigned int)tv_diff.tv_usec,
			    SysTick->VAL);
			fflush(stdout);

			timer_get_mono_time(&tv_start);
		}
#endif

		plen = gdb_read_packet(gc); 

#ifdef DEBUG_ENABLED
		if (DEBUG_FLAG > 1) {
			if (gc->gc_buff[0] == 0x4) {
				DBFPRINTF("Rx packet 'CTRL-D'\n");
			} else {
				hexdump(NULL, "GDB-RX ", 16, gc->gc_buff,
				    gc->gc_buff_len);
			}
		}
#endif

		if ((cmd = gc->gc_buff[0]) >= sizeof(gdb_cmd_index))
			cmd = GDB_CMD_CODE(GDB_CMD_NOOP);
		else
			cmd = gdb_cmd_index[cmd];

		if (syscall && cmd == GDB_CMD_FILEIO)
			break;

		if (!syscall || (cmd & GDB_CMD_FIO) != 0) {
			if ((cmd & GDB_CMD_ATTACHED) != 0 &&
			    gdb_attached_to_target(gc, false) <=
			    TMON_QA_DETACHED) {
				fputs(GDB_RESP_DEAD, gc->gc_ostream);
			} else {
				(gdb_cmd_dispatch[GDB_CMD_CODE(cmd)])(gc, plen);
			}
		}

		gdb_flush_packet(gc);

#ifdef GDB_TIMED_SERVER_LOOP
		if (!syscall) {
			timer_get_mono_time(&tv_stop);
			timer_sub(&tv_stop, &tv_start, &tv_diff);
			printf("cmd %02x, %u.%06u, %7lu\n", cmd,
			    (unsigned int)tv_diff.tv_sec,
			    (unsigned int)tv_diff.tv_usec,
			    SysTick->VAL);
		}
#endif
	}
}

static void
gdb_string_command(gdb_context_t gc, uint32_t plen,
    const struct gdb_string_cmd *sc)
{

	while (sc->sc_string != NULL &&
	    strncmp((const char *)gc->gc_buff, sc->sc_string,
	    sc->sc_string_len) != 0) {
		sc++;
	}

	if (sc->sc_string != NULL) {
		if ((sc->sc_flags & GDB_CMD_ATTACHED) != 0 &&
		    gdb_attached_to_target(gc, false) <= TMON_QA_DETACHED) {
			fputs(GDB_RESP_DEAD, gc->gc_ostream);
		} else {
			(sc->sc_handler)(gc, &gc->gc_buff[sc->sc_string_len],
			    plen - sc->sc_string_len);
		}
	}
}

static void
gdb_cmd_noop(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("unsupported packet: '%s'\n", (char *)gc->gc_buff);

	/* No need to write an empty response; gdb_flush_packet() will DTRT */
}

static void
gdb_cmd_ignored(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("ignored packet: '%s'\n", (char *)gc->gc_buff);

	/* No need to write an empty response; gdb_flush_packet() will DTRT */
}

static void
gdb_cmd_detach(gdb_context_t gc, uint32_t plen)
{
	int rv;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	rv = TMON_DT_DETACHED;

	if (gdb_attached_to_target(gc, false) == TMON_QA_ATTACHED)
		rv = tmon_detach(gc->gc_tmon);

	if (rv == TMON_DT_DETACHED) {
		fputs(GDB_RESP_OK, gc->gc_ostream);
		gc->gc_flags |= GDB_FLAGS_KILLED;
		gc->gc_kill_value = 0;
	} else {
		fputs(GDB_RESP_EBUSY, gc->gc_ostream);
	}
}

static void
gdb_cmd_enable_ext_mode(gdb_context_t gc, uint32_t plen)
{

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	gc->gc_flags |= GDB_FLAGS_EXTENDED_MODE;

	/* Nothing more required */
	fputs(GDB_RESP_OK, gc->gc_ostream);
}

static int
gdb_halt_query_wait(gdb_context_t gc, struct tmon_watch *tw, bool just_query)
{
	int rv;
#ifdef DEBUG_ENABLED
	int cnt = 0;
#endif

	DBFPRINTF("%s\n", just_query ? "Just Query" : "Wait for Halt");

	/*
	 * Target is running.
	 * We wait here for one of three things to happen:
	 *  - Target halts of its own accord.
	 *  - GDB sends us a CTRL-C in order to forcibly halt the target.
	 *  - The target disappears.
	 */
	do {
		rv = tmon_halt_status(gc->gc_tmon, tw, just_query);
		if (just_query || rv != TMON_HS_RUNNING)
			break;

#ifdef DEBUG_ENABLED
		if (cnt++ >= 1000) {
			debug_print("gdb_halt_query_wait: still running\n");
			cnt = 0;
		}
#endif

		/*
		 * Don't just rely on pre-emption; let lower-priority tasks
		 * get some CPU time while we're waiting.
		 */
		rtos_task_sleep(5);

		/*
		 * Check if the client is requesting we halt the target.
		 */
		if (gdb_server_check_for_interrupt(gc)) {
			DBFPRINTF("CTRL-C\n");
			rv = tmon_halt(gc->gc_tmon);
			if (rv == TMON_HS_FORCED)
				tw->tw_type = GDB_SIGNAL_STOP;
		}
	} while (rv == TMON_HS_RUNNING);

	DBFPRINTF("rv %d\n", rv);

#ifdef CONFIG_GDB_RTOS_SUPPORT
	if (rv!= TMON_HS_RUNNING && gc->gc_rtos != NULL)
		gdb_rtos_update_threads(gc->gc_rtos);
#endif

	return rv;
}

static void
gdb_query_halt(gdb_context_t gc, bool just_query)
{
	struct tmon_watch tw;
	u_int halt, sig;
	const char *wtype;
	char reply_ch, st, tstr[20], wstr[20];
	int x;

	DBFPRINTF("just_query %u\n", just_query);

	if (gc->gc_flags & (GDB_FLAGS_KILLED | GDB_FLAGS_EXITED)) {
		char what = (gc->gc_flags & GDB_FLAGS_KILLED) ? 'X' : 'W';
		gc->gc_flags &= ~(GDB_FLAGS_KILLED | GDB_FLAGS_EXITED);
		DBFPRINTF("overriding halt status: %c%02x\n", what,
		    (unsigned int)gc->gc_kill_value);
		fprintf(gc->gc_ostream, "%c%02X", what,
		    (unsigned int)gc->gc_kill_value);
		return;
	}

	switch ((x = gdb_attached_to_target(gc, false))) {
	case TMON_QA_ATTACHED:
		DBFPRINTF("currently attached\n");
		break;

	case TMON_QA_ERROR:
		DBFPRINTF("gdb_attached_to_target() failed\n");
		fputs("W00", gc->gc_ostream);
		return;

	case TMON_QA_DETACHED:
		DBFPRINTF("re-attaching (%d)\n", x);
		if (gdb_should_auto_attach(gc) == false ||
		    gdb_attached_to_target(gc, true) != TMON_QA_ATTACHED) {
			DBFPRINTF("gdb_attached_to_target() failed\n");
			fputs("W00", gc->gc_ostream);
			return;
		}
		sig = GDB_SIGNAL_INT;
		fprintf(gc->gc_ostream, "S%02X", sig);
		DBFPRINTF("re-attach succeeded\n");
		gc->gc_flags &= ~(GDB_FLAGS_KILLED | GDB_FLAGS_EXITED);
		return;

	default:
		DBFPRINTF("gdb_attached_to_target() bogons (%d)\n", x);
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	/*
	 * Target is attached, and we should query its halt status.
	 * *
	 * At this point, 'just_query' indicates whether we perform
	 * a simple query (true), or we hang around until the target
	 * stops.
	 */
	tw.tw_addr = 0;
	tw.tw_type = TMON_WATCH_TYPE_ACCESS;
	halt = gdb_halt_query_wait(gc, &tw, just_query);

	DBFPRINTF("halt reason %u\n", halt);

	/* Default status. */
	wstr[0] = '\0';
	tstr[0] = '\0';
	st = 'S';

#ifdef CONFIG_GDB_RTOS_SUPPORT
	/*
	 * If we have an active RTOS, synchronise state.
	 */
	if (gc->gc_rtos != NULL) {
		target_addr_t tid;

		tid = gdb_rtos_current_thread(gc->gc_rtos);
		snprintf(tstr, sizeof(tstr), "thread:%" PRIxTADDR ";", tid);
		st = 'T';
	}
#endif

	switch (halt) {
	case TMON_HS_DETACHED:
		DBFPRINTF("DETATCHED\n");
		sig = GDB_SIGNAL_0;
		reply_ch = 'W';
		break;

	case TMON_HS_FAULT:
		DBFPRINTF("FAULT\n");
		sig = GDB_SIGNAL_SEGV;
		reply_ch = st;
		break;

	case TMON_HS_WATCHPOINT:
		DBFPRINTF("SIGTRAP - watchpoint\n");
		switch (tw.tw_type) {
		case TMON_WATCH_TYPE_WRITE:
			wtype = "watch";
			break;
		case TMON_WATCH_TYPE_READ:
			wtype = "rwatch";
			break;
		case TMON_WATCH_TYPE_ACCESS:
		default:
			wtype = "awatch";
			break;
		}
		snprintf(wstr, sizeof(wstr), "%s:%08" PRIxTADDR ";", wtype,
		    tw.tw_addr);
		sig = GDB_SIGNAL_TRAP;
		reply_ch = 'T';
		break;

	case TMON_HS_FORCED:
		DBFPRINTF("FORCED %" PRIu32 "\n", tw.tw_type);
		sig = tw.tw_type;
		reply_ch = st;
		break;

	case TMON_HS_RUNNING: /* This can't happen! */
	case TMON_HS_ERROR:
		DBFPRINTF("error\n");
		sig = GDB_SIGNAL_LOST;
		reply_ch = 'X';
		break;

	case TMON_HS_EXITED:
		DBFPRINTF("Exited %" PRIu32 "\n", tw.tw_type);
#if 0
		/*
		 * This is how we'd like to do things, because Gdb will
		 * correctly show that the target firmware has "exited" with
		 * the relevant exit code. However, Gdb will assume that the
		 * target "process" is completely gone, and will refuse to
		 * restart it.
		 */
		sig = tw.tw_type;
		reply_ch = 'W';
		gc->gc_flags |= GDB_FLAGS_EXITED;
		gc->gc_kill_value = (uint8_t)sig;
#else
		/*
		 * Use SIG0 to indicate firmware invoked exit().
		 */
		sig = GDB_SIGNAL_0;
		reply_ch = st;
#endif
		break;

	case TMON_HS_BREAKPOINT:
		DBFPRINTF("%s BREAKPOINT @ 0x%" PRIxTADDR "\n",
		    (tw.tw_type == TMON_WATCH_TYPE_BSOFT) ? "SOFT" : "HARD",
		    tw.tw_addr);

		if (tw.tw_type == TMON_WATCH_TYPE_BSOFT &&
		    (gc->gc_flags & GDB_FLAGS_HAS_SWBREAK) != 0) {
			strcpy(wstr, "swbreak:;");
			reply_ch = 'T';
		} else
		if (tw.tw_type == TMON_WATCH_TYPE_BHARD &&
		    (gc->gc_flags & GDB_FLAGS_HAS_HWBREAK) != 0) {
			strcpy(wstr, "hwbreak:;");
			reply_ch = 'T';
		} else {
			/* Fallback to bare SIGTRAP */
			reply_ch = st;
		}
		sig = GDB_SIGNAL_TRAP;
		break;

	default:
		DBFPRINTF("SIGTRAP (default)\n");
		sig = GDB_SIGNAL_TRAP;
		reply_ch = st;
		break;
	}

	DBFPRINTF("reply: %c%02X%s%s\n", reply_ch, sig, tstr, wstr);

	fprintf(gc->gc_ostream, "%c%02X%s%s", reply_ch, sig, tstr, wstr);
}

static void
gdb_cmd_query_halt(gdb_context_t gc, uint32_t plen)
{

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	gdb_query_halt(gc, true);
}

static void
gdb_cmd_set_args(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);
}

static void
gdb_cmd_continue_sig(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr, *pa;
	unsigned int sig;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	pa = &addr;
	switch (sscanf((const char *)gc->gc_buff, "C%u;%" SCNxTADDR, &sig, pa)){
	case 1:
		pa = NULL;
		break;
	case 2:
		break;
	default:
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	switch (tmon_resume(gc->gc_tmon, false, pa)) {
	case TMON_RS_OK:
		gdb_query_halt(gc, false);
		break;

	default:
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}
}

static void
gdb_cmd_fileio(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);
}

static void
gdb_cmd_continue(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr, *pa;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	pa = &addr;
	if (sscanf((const char *)gc->gc_buff, "c%" SCNxTADDR, pa) != 1)
		pa = NULL;

	switch (tmon_resume(gc->gc_tmon, false, pa)) {
	case TMON_RS_OK:
		gdb_query_halt(gc, false);
		break;

	default:
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}
}

static int
gdb_check_regbuff(gdb_context_t gc)
{
	uint32_t rbsize;
	int rv;

	rv = tmon_read_regs(gc->gc_tmon, NULL);
	if (rv < 0)
		return rv;

	rbsize = (uint32_t) rv;

	if (gc->gc_reg_buff != NULL && gc->gc_reg_buff_len != rbsize) {
		zone_free(gc->gc_reg_buff);
		gc->gc_reg_buff = NULL;
		gc->gc_reg_buff_len = 0;
	}

	if (gc->gc_reg_buff == NULL) {
		if ((gc->gc_reg_buff = zone_malloc(rbsize)) == NULL) {
			gc->gc_bail_str = "gdb_check_regbuff: no mem";
			longjmp(gc->gc_jmp_buf, 1);
		}
		gc->gc_reg_buff_len = rbsize;
	}

	return 0;
}

static void
gdb_cmd_read_all_regs(gdb_context_t gc, uint32_t plen)
{

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gdb_check_regbuff(gc) < 0 ||
	    tmon_read_regs(gc->gc_tmon, gc->gc_reg_buff) < 0) {
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	gdb_send_hex_buff(gc, gc->gc_reg_buff, gc->gc_reg_buff_len);
}

static void
gdb_cmd_write_all_regs(gdb_context_t gc, uint32_t plen)
{
	uint32_t blen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gdb_check_regbuff(gc) < 0) {
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	plen -= 1;	/* Account for the leading 'G' */

	if (gc->gc_reg_buff_len != (plen / 2)) {
		DBFPRINTF("hex length (%" PRIu32 ") != bin length (%u)\n",
		    plen / 2, gc->gc_reg_buff_len);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	blen = hex_to_bin((const char *)&gc->gc_buff[1], gc->gc_reg_buff, plen);

	if (blen != (plen / 2)) {
		DBFPRINTF("binary length (%" PRIu32 ") != len (%lu)\n", blen,
		    plen);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	if (tmon_write_regs(gc->gc_tmon, gc->gc_reg_buff) < 0) {
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}
}

static void
gdb_cmd_set_thread(gdb_context_t gc, uint32_t plen)
{

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (plen >= 3) {
		switch ((char)gc->gc_buff[1]) {
		case 'g':
#ifdef CONFIG_GDB_RTOS_SUPPORT
			if (gc->gc_rtos != NULL) {
				uint8_t *arg = gc->gc_buff + 2;
				uint64_t tid;
				if (hex_to_uint64(arg, &tid) != arg) {
					gdb_rtos_select_thread(gc->gc_rtos,
					    (target_addr_t)tid);
					fputs(GDB_RESP_OK, gc->gc_ostream);
					break;
				}
			}
#endif
			if (strcmp((const char *)&gc->gc_buff[2], "0") == 0)
				fputs(GDB_RESP_OK, gc->gc_ostream);
			break;

		case 'c':
			if (strcmp((const char *)&gc->gc_buff[2], "-1") == 0)
				fputs(GDB_RESP_OK, gc->gc_ostream);
			break;

		default:
			break;
		}
	}
}

static void
gdb_cmd_step_cycle(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	/* No ACK required */
}

static void
gdb_cmd_sig_cycle(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	/* No ACK required */
}

static void
gdb_cmd_kill_target(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gdb_attached_to_target(gc, false) == TMON_QA_ATTACHED) {
		tmon_reset(gc->gc_tmon, TARGET_RESET_SOFT_NO_HALT);
		tmon_detach(gc->gc_tmon);
	}

	/* When not in extended mode, kill the gdb server connection */
	if ((gc->gc_flags & GDB_FLAGS_EXTENDED_MODE) == 0)
		longjmp(gc->gc_jmp_buf, 1);

	/* Don't respond */
	gc->gc_flags |= GDB_FLAGS_NO_RESPONSE | GDB_FLAGS_KILLED;
}

static void
gdb_cmd_read_mem(gdb_context_t gc, uint32_t plen)
{
#undef GDB_TIMED_MEMORY_READ
#ifdef GDB_TIMED_MEMORY_READ
	struct timeval tv_start, tv_stop, tv_diff;
	static struct timeval tv_last;
#endif
	target_addr_t addr;
	uint32_t len;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (sscanf((const char *)gc->gc_buff, "m%" SCNxTADDR ",%" SCNx32, &addr,
	    &plen) != 2) {
		DBFPRINTF("failed to parse addr/len '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

#ifdef GDB_TIMED_MEMORY_READ
	timer_get_mono_time(&tv_start);
	timer_sub(&tv_start, &tv_last, &tv_diff);
	if (tv_diff.tv_sec >= 1)
		putchar('\n');
	tv_last = tv_start;
	printf("mem_read(%u.%06u): 0x%08" PRIxTADDR " %" PRIu32 ": ",
	    (unsigned int)tv_diff.tv_sec, (unsigned int)tv_diff.tv_usec,
	    addr, plen);
	fflush(stdout);
	timer_get_mono_time(&tv_start);
#endif

	while (plen) {
		len = plen;
		if (len > gc->gc_mem_buff_len)
			len = gc->gc_mem_buff_len;


		if (tmon_mem_read(gc->gc_tmon, addr, len,
		    gc->gc_mem_buff) < 0) {
			DBFPRINTF("read mem failed\n");
			fputs(GDB_RESP_EFAULT, gc->gc_ostream);
#ifdef GDB_TIMED_MEMORY_READ
			printf("failed\n");
#endif
			return;
		}

		gdb_send_hex_buff(gc, gc->gc_mem_buff, len);

		addr += len;
		plen -= len;
	}

#ifdef GDB_TIMED_MEMORY_READ
	timer_get_mono_time(&tv_stop);
	timer_sub(&tv_stop, &tv_start, &tv_diff);
	printf("%u.%06u\n", (unsigned int)tv_diff.tv_sec,
	    (unsigned int)tv_diff.tv_usec);
#endif
}

static void
gdb_cmd_write_mem(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr;
	uint32_t len, blen;
	int hoff;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (sscanf((const char *)gc->gc_buff, "M%" SCNxTADDR ",%" SCNx32 ":%n",
	    &addr, &len, &hoff) != 2) {
		DBFPRINTF("failed to parse addr/len '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	if (len > gc->gc_mem_buff_len) {
		DBFPRINTF("length(%" PRIu32 ") > buffer size (%u)\n", len,
		    gc->gc_mem_buff_len);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	blen = hex_to_bin((const char *)&gc->gc_buff[hoff], gc->gc_mem_buff,
	    len * 2);
	if (blen != len) {
		DBFPRINTF("binary length (%" PRIu32 ") != len (%" PRIu32 ")\n",
		    blen, len);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	if (tmon_mem_write(gc->gc_tmon, addr, blen, gc->gc_mem_buff) < 0) {
		DBFPRINTF("failed\n");
		fputs(GDB_RESP_EFAULT, gc->gc_ostream);
	} else
		fputs(GDB_RESP_OK, gc->gc_ostream);
}

static void
gdb_cmd_read_reg(gdb_context_t gc, uint32_t plen)
{
	unsigned int reg;
	int rv;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (sscanf((const char *)gc->gc_buff, "p%x", &reg) != 1) {
		DBFPRINTF("failed to parse regnum '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	if (gdb_check_regbuff(gc) < 0 ||
	    (rv = tmon_read_one_reg(gc->gc_tmon, reg, gc->gc_reg_buff)) < 0) {
		DBFPRINTF("failed to read reg '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	gdb_send_hex_buff(gc, gc->gc_reg_buff, rv);
}

static void
gdb_cmd_write_reg(gdb_context_t gc, uint32_t plen)
{
	unsigned int reg, hoff;
	int shoff;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (sscanf((const char *)gc->gc_buff, "P%x=%n", &reg, &shoff) != 1 ||
	    shoff < 0) {
		DBFPRINTF("failed to parse regnum '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	hoff = (unsigned int)shoff;

	if (gdb_check_regbuff(gc) < 0) {
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	if (plen < hoff || ((plen - hoff) / 2) > gc->gc_reg_buff_len) {
		DBFPRINTF("length (%" PRIu32 ") > regbuff (%u)\n",
		    (plen - hoff) / 2, gc->gc_reg_buff_len);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	(void) hex_to_bin((const char *)&gc->gc_buff[hoff], gc->gc_reg_buff,
	    plen - hoff);

	if (tmon_write_one_reg(gc->gc_tmon, reg, gc->gc_reg_buff) < 0) {
		DBFPRINTF("failed to write reg '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	fputs(GDB_RESP_OK, gc->gc_ostream);
}

static int
gdb_cmdstream_write(void *arg, const char *buff, int buflen)
{
	gdb_context_t gc = arg;
	uint32_t in_bytes, out_bytes;
	int rv = buflen;
	char O = 'O';

	if (buflen)
		gc->gc_flags &= ~GDB_FLAGS_CMD_EMPTY;

	while (buflen) {
		in_bytes = (uint32_t) buflen;
		out_bytes = in_bytes * 2;
		if (out_bytes > GDB_SERVER_OUTBUFF_LEN) {
			out_bytes = GDB_SERVER_OUTBUFF_LEN;
			in_bytes = GDB_SERVER_OUTBUFF_LEN / 2;
		}

		if (out_bytes >= ((gc->gc_buff_size - gc->gc_olen) - 4))
			gdb_flush_packet(gc);

		if (gc->gc_olen == 0)
			gdb_ostream_write(arg, &O, 1);

		bin_to_hex((const uint8_t *)buff,
		    (uint8_t *)gc->gc_cmd_outbuff, in_bytes);
		gdb_ostream_write(gc, (const char *)gc->gc_cmd_outbuff,
		    out_bytes);

		buflen -= in_bytes;
		buff += in_bytes;
	}

	return rv;
}

static void
gdb_cmd_query_command(gdb_context_t gc, const uint8_t *args, uint32_t len)
{
	uint8_t argc;

	if (len < 2) {
		fputs(GDB_RESP_OK, gc->gc_ostream);
		return;
	}

	/*
	 * We can re-use the mem buffer for the command line. Just make sure
	 * the length is appropriate.
	 */
	if ((len / 2) >= gc->gc_mem_buff_len)
		len = (gc->gc_mem_buff_len - 1) * 2;

	len = hex_to_bin((const char *) args, gc->gc_mem_buff, len);
	((char *)gc->gc_mem_buff)[len] = '\0';

	argc = shell_tokenise(gc->gc_cmd_argv, GDB_SERVER_MAX_CMD_ARGS,
	    gc->gc_mem_buff);

	if (argc) {
		FILE *os;

		os = fwopen(gc, gdb_cmdstream_write);
		if (os == NULL) {
			DBFPRINTF("failed to open ostream\n");
			fprintf(gc->gc_ostream, "E%02X", errno);
			return;
		}
		os->_flags |= __SMBF;

		/* Ensure output handler is ready to rumble */
		gc->gc_olen = 0;
		gc->gc_ocsum = 0;
		gc->gc_flags |= GDB_FLAGS_CMD_EMPTY;

		shell_execute_cmd(NULL, gc->gc_cmd_argv, argc, os, NULL);

		fflush(os);
		fclose(os);

		if ((gc->gc_flags & GDB_FLAGS_CMD_EMPTY) == 0 && gc->gc_olen)
			gdb_flush_packet(gc);
	}

	fputs(GDB_RESP_OK, gc->gc_ostream);
}

static void
gdb_cmd_query_qSupported(gdb_context_t gc, const uint8_t *args, uint32_t len)
{

	(void) len;

	gc->gc_flags &= ~(GDB_FLAGS_HAS_SWBREAK | GDB_FLAGS_HAS_HWBREAK |
	    GDB_FLAGS_HAS_VCONT);

	if (strstr((const char *)args, "swbreak+") != NULL)
		gc->gc_flags |= GDB_FLAGS_HAS_SWBREAK;
	if (strstr((const char *)args, "hwbreak+") != NULL)
		gc->gc_flags |= GDB_FLAGS_HAS_HWBREAK;
	if (strstr((const char *)args, "vContSupported+") != NULL)
		gc->gc_flags |= GDB_FLAGS_HAS_VCONT;

	fprintf(gc->gc_ostream, "PacketSize=0x%" PRIx32, gc->gc_buff_size - 1);
	fputs(";qXfer:features:read+", gc->gc_ostream);
	fputs(";qXfer:memory-map:read+", gc->gc_ostream);
	fputs(";QStartNoAckMode+", gc->gc_ostream);
	if (gc->gc_flags & GDB_FLAGS_HAS_SWBREAK)
		fputs(";swbreak+", gc->gc_ostream);
	if (gc->gc_flags & GDB_FLAGS_HAS_HWBREAK)
		fputs(";hwbreak+", gc->gc_ostream);
}

static bool
gdb_query_offset_length(gdb_context_t gc, const uint8_t *args, stringio_t s)
{
	uint32_t offset, len, str_len;
	char resp;

	if (sscanf((const char *)args, "%" SCNx32 ",%" SCNx32, &offset,
	    &len) != 2) {
		DBFPRINTF("failed to parse offset/len '%s'\n", args);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return true;
	}

	DBFPRINTF("offset %" PRIu32 ", len %" PRIu32 "\n", offset, len);

	str_len = (uint32_t)stringio_get_len(s);

	if (offset >= str_len) {
		if (offset > str_len) {
			DBFPRINTF("offset > str_len (%" PRIu32 ")\n", str_len);
			fputs(GDB_RESP_ESPIPE, gc->gc_ostream);
		}
		return true;
	}

	if ((offset + len) > str_len) {
		len = str_len - offset;
		resp = 'l';
	} else {
		resp = 'm';
	}

	fputc(resp, gc->gc_ostream);

	if (len) {
		const char *str = stringio_get_buff(s);

		assert(str != NULL);

		fwrite(&str[offset], 1, len, gc->gc_ostream);
	}

	return resp == 'l';
}

static void
gdb_cmd_query_target_xml(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	bool do_attach;
	stringio_t s;

	(void) plen;

	do_attach = gdb_should_auto_attach(gc);

	if (gdb_attached_to_target(gc, do_attach) <= TMON_QA_DETACHED) {
		DBFPRINTF("Not attached to target.\n");
		fputs(GDB_RESP_EPERM, gc->gc_ostream);
		return;
	}

	if ((s = gc->gc_xml_string) == NULL) {
		int rv;

		if ((s = stringio_new()) == NULL ||
		    (rv = tmon_get_xml_regs(gc->gc_tmon, s)) < 0 ||
		    stringio_get_len(s) == 0) {
			DBFPRINTF("no registers XML\n");
			fputs(GDB_RESP_EIO, gc->gc_ostream);
			return;
		}

		gc->gc_xml_string = s;
	}

	if (gdb_query_offset_length(gc, args, s)) {
		stringio_done(s);
		gc->gc_xml_string = NULL;
	}
}

static void
gdb_cmd_query_mem_map(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	bool do_attach;
	stringio_t s;

	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

	do_attach = gdb_should_auto_attach(gc);

	if (gdb_attached_to_target(gc, do_attach) <= TMON_QA_DETACHED) {
		DBFPRINTF("Not attached to target\n");
		fputs(GDB_RESP_EPERM, gc->gc_ostream);
		return;
	}

	if ((s = gc->gc_xml_string) == NULL) {
		int rv;

		if ((s = stringio_new()) == NULL ||
		    (rv = tmon_get_xml_memory(gc->gc_tmon, s)) < 0 ||
		    stringio_get_len(s) == 0) {
			DBFPRINTF("no memory map XML\n");
			fputs(GDB_RESP_EIO, gc->gc_ostream);
			return;
		}

		gc->gc_xml_string = s;
	}

	if (gdb_query_offset_length(gc, args, s)) {
		stringio_done(s);
		gc->gc_xml_string = NULL;
	}
}

static void
gdb_cmd_query_attached(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	bool do_attach;

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

	do_attach = gdb_should_auto_attach(gc);

	if (gdb_attached_to_target(gc, do_attach) <= TMON_QA_DETACHED) {
		DBFPRINTF("attach failed\n");
		fputs(GDB_RESP_EPERM, gc->gc_ostream);
		return;
	}

	fputs("1", gc->gc_ostream);
}

static void
gdb_cmd_query_offsets(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

	fputs("Text=0;Data=0;Bss=0", gc->gc_ostream);
}

static void
gdb_cmd_query_thread_info_first(gdb_context_t gc, const uint8_t *args,
    uint32_t plen)
{

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

#ifdef CONFIG_GDB_RTOS_SUPPORT
	stringio_t tl;
	if (gc->gc_rtos != NULL &&
	    (tl = gdb_rtos_thread_list(gc->gc_rtos)) != NULL) {
		fprintf(gc->gc_ostream, "m%s", stringio_get_buff(tl));
		stringio_done(tl);
	} else
#endif

	fputs("l", gc->gc_ostream);
}

static void
gdb_cmd_query_thread_info_next(gdb_context_t gc, const uint8_t *args,
    uint32_t plen)
{

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

	fputs("l", gc->gc_ostream);
}

static void
gdb_cmd_query_thread_id(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	target_addr_t tid;

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

#ifdef CONFIG_GDB_RTOS_SUPPORT
	if (gc->gc_rtos != NULL)
		tid = gdb_rtos_current_thread(gc->gc_rtos);
	else
#endif
		tid = 0;

	DBFPRINTF("Reply: QC%" PRIxTADDR "\n", tid);

	fprintf(gc->gc_ostream, "QC%" PRIxTADDR, tid);
}

#ifdef CONFIG_GDB_RTOS_SUPPORT
static void
gdb_cmd_query_thread_extra_info(gdb_context_t gc, const uint8_t *args,
    uint32_t plen)
{
	const struct gdb_rtos_thread *rt;
	uint64_t tid;

	(void) plen;

	if (gc->gc_rtos == NULL)
		return;

	if (args[0] == '\0' || hex_to_uint64(args, &tid) == args)
		tid = 0;

	rt = gdb_rtos_thread_info(gc->gc_rtos, (target_addr_t)tid);
	if (rt == NULL)
		fputs(GDB_RESP_EPERM, gc->gc_ostream);
	else {
		size_t len = strlen(rt->rt_info);

		assert(len > 0);

		gdb_send_hex_buff(gc, rt->rt_info, len);
	}
}

static void
gdb_cmd_query_symbol(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	const char *next_symbol;
	uint64_t value;

	DBFPRINTF("'%s'\n", (const char *)args);

	if (plen < 1 || (gc->gc_rtos == NULL &&
	    (gc->gc_rtos = gdb_rtos_new(gc->gc_tmon,
	    gc->gc_instance)) == NULL)) {
		/* Ignore if query string is malformed or RTOS init failed. */
		fputs(GDB_RESP_OK, gc->gc_ostream);
		return;
	}

	if (args[0] == ':' && plen == 1) {
		/* Gdb is inviting us to request symbol lookups. */
		next_symbol = gdb_rtos_qSymbol_first(gc->gc_rtos);
		if (next_symbol != NULL)
			DBFPRINTF("First symbol: %s\n", next_symbol);
	} else {
		/* Symbol lookup response. Extract its value. */
		bool found = hex_to_uint64(args, &value) != args;
		DBFPRINTF("Symbol value: %" PRIxTADDR "\n",
		    (target_addr_t)value);
		next_symbol = gdb_rtos_qSymbol_next(gc->gc_rtos, found,
		    (target_addr_t)value);
		if (next_symbol != NULL)
			DBFPRINTF("Next symbol: %s\n", next_symbol);
	}

	if (next_symbol == NULL) {
		DBFPRINTF("No more symbols.\n");
		if (gdb_rtos_detected(gc->gc_rtos) == false) {
			DBFPRINTF("No RTOS detected.\n");
			gdb_rtos_free(gc->gc_rtos);
			gc->gc_rtos = NULL;
		} else {
			DBFPRINTF("RTOS detected.\n");
			gdb_rtos_update_threads(gc->gc_rtos);
		}
		fputs(GDB_RESP_OK, gc->gc_ostream);
	} else {
		size_t l = strlen(next_symbol);
		char *buff = gc->gc_hex_buff;

		bin_to_hex((const uint8_t *)next_symbol, gc->gc_hex_buff, l);
		buff[l * 2] = '\0';

		fprintf(gc->gc_ostream, "qSymbol:%s", buff);
	}
}
#endif /* CONFIG_GDB_RTOS_SUPPORT */

static void
gdb_cmd_query(gdb_context_t gc, uint32_t plen)
{
	static const struct gdb_string_cmd general_query_cmds[] = {
		STR_CMD("qSupported:", gdb_cmd_query_qSupported),
		STR_CMD("qRcmd,", gdb_cmd_query_command),
		STR_CMD("qXfer:features:read:target.xml:",
		    gdb_cmd_query_target_xml),
		STR_CMD("qXfer:memory-map:read::", gdb_cmd_query_mem_map),
		STR_CMD("qAttached", gdb_cmd_query_attached),
		STR_CMD("qOffsets", gdb_cmd_query_offsets),
		STR_CMD("qfThreadInfo", gdb_cmd_query_thread_info_first),
		STR_CMD("qsThreadInfo", gdb_cmd_query_thread_info_next),
		STR_CMD("qC", gdb_cmd_query_thread_id),
#ifdef CONFIG_GDB_RTOS_SUPPORT
		STR_CMD("qThreadExtraInfo,", gdb_cmd_query_thread_extra_info),
		STR_CMD("qSymbol:", gdb_cmd_query_symbol),
#endif
		STR_CMD_END
	};

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	gdb_string_command(gc, plen, general_query_cmds);
}

static void
gdb_cmd_set_no_ack_mode(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{

	(void) args;
	(void) plen;

	DBFPRINTF("'%s'\n", (const char *)args);

	gc->gc_flags |= GDB_FLAGS_NO_ACK_MODE;
	fputs(GDB_RESP_OK, gc->gc_ostream);
}

static void
gdb_cmd_set(gdb_context_t gc, uint32_t plen)
{
	static const struct gdb_string_cmd general_set_cmds[] = {
		STR_CMD("QStartNoAckMode", gdb_cmd_set_no_ack_mode),
		STR_CMD_END
	};

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	gdb_string_command(gc, plen, general_set_cmds);
}

static void
gdb_cmd_restart(gdb_context_t gc, uint32_t plen)
{

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gc->gc_flags & GDB_FLAGS_EXTENDED_MODE) {
		if (gdb_attached_to_target(gc, true) == TMON_QA_ATTACHED)
			(void) tmon_reset(gc->gc_tmon, TARGET_RESET_SOFT_HALT);
	}

	/* Do not respond */
	gc->gc_flags |= GDB_FLAGS_NO_RESPONSE;
}

static void
gdb_cmd_step(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr, *pa;
	int rv;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gdb_attached_to_target(gc, false) <= TMON_QA_DETACHED) {
		DBFPRINTF("not attached\n");
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	pa = &addr;
	if (sscanf((const char *)gc->gc_buff, "s%" SCNxTADDR, pa) != 1)
		pa = NULL;

	if ((rv = tmon_resume(gc->gc_tmon, true, pa)) != TMON_RS_OK) {
		DBFPRINTF("resume rv %d\n", rv);
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	DBFPRINTF("resume succeeded. checking halt.\n");
	gdb_query_halt(gc, false);
}

static void
gdb_cmd_sig_step(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr, *pa;
	unsigned int sig;
	int rv;

	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (gdb_attached_to_target(gc, false) <= TMON_QA_DETACHED) {
		DBFPRINTF("not attached\n");
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	pa = &addr;
	if (sscanf((const char *)gc->gc_buff, "S%x;%" SCNxTADDR, &sig, pa) != 2)
		pa = NULL;

	if ((rv = tmon_resume(gc->gc_tmon, true, pa)) != TMON_RS_OK) {
		DBFPRINTF("resume rv %d\n", rv);
		fprintf(gc->gc_ostream, "X%02X", GDB_SIGNAL_LOST);
		return;
	}

	DBFPRINTF("resume succeeded. checking halt.\n");
	gdb_query_halt(gc, true);
}

static void
gdb_cmd_reverse_search(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	/* No ACK required */
}

static void
gdb_cmd_thread_state(gdb_context_t gc, uint32_t plen)
{

	(void) gc;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

#ifdef CONFIG_GDB_RTOS_SUPPORT
	if (gc->gc_rtos != NULL) {
		uint8_t *arg = gc->gc_buff + 1;
		uint64_t tid;
		bool found = hex_to_uint64(arg, &tid) != arg;

		if (found && gdb_rtos_thread_info(gc->gc_rtos,
		    (target_addr_t)tid) != NULL) {
			fputs(GDB_RESP_OK, gc->gc_ostream);
			DBFPRINTF("Exists.\n");
		} else {
			fputs(GDB_RESP_EPERM, gc->gc_ostream);
			DBFPRINTF("Does not exist.\n");
		}
	}
#endif
}

static void
gdb_cmd_attach(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	unsigned int pid;

	(void) plen;

	DBFPRINTF("%s\n", (const char *)args);

	if (sscanf((const char *)args, "%x", &pid) != 1 || pid == 0) {
		DBPRINTF("Bad param\n");
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	pid -= 1;

	/*
	 * User can request we attach to a specific core...
	 */
	if (pid >= tmon_get_target_count()) {
		DBPRINTF("Invalid target: %u\n", pid);
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	if (gc->gc_target_num != pid &&
	    gdb_attached_to_target(gc, false) == TMON_QA_ATTACHED) {
		/*
		 * Need to detach.
		 */
		tmon_flash_done(gc->gc_tmon);
		gdb_server_syscall_hook(gc->gc_instance, gc->gc_tmon, NULL);
		tmon_detach(gc->gc_tmon);
	}
	gc->gc_target_num = pid;

	DBPRINTF("Attempting to attach to target: %u\n", gc->gc_target_num);

	if (gdb_attached_to_target(gc, true) <= TMON_QA_DETACHED) {
		DBPRINTF("Attach failed\n");
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	DBPRINTF("Attach succeeded. Querying target status.\n");

	/* We've attached. Return target status. */
	gdb_query_halt(gc, false);
}

static void
gdb_cmd_flash_erase(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
	target_addr_t addr;
	uint32_t len;

	(void) plen;

	if (sscanf((const char *)args, "%" SCNxTADDR ",%" SCNx32, &addr,
	    &len) != 2) {
		DBFPRINTF("failed to parse addr/len '%s'\n", args);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	DBFPRINTF("addr %08" PRIxTADDR " len %08" PRIx32 "\n", addr, len);

	switch (tmon_flash_erase(gc->gc_tmon, addr, len)) {
	case TMON_FL_OK:
		fputs(GDB_RESP_OK, gc->gc_ostream);
		break;
	case TMON_FL_NOT_FLASH:
		fputs(GDB_RESP_EFAULT, gc->gc_ostream);
		break;
	default:
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		break;
	}
}

static void
gdb_cmd_flash_write(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{
#undef GDB_TIMED_FLASH_WRITE
#ifdef GDB_TIMED_FLASH_WRITE
	struct timeval tv_start, tv_stop, tv_diff;
	static struct timeval tv_last;
#endif
	target_addr_t addr;
	int rv, off;

	if (sscanf((const char *)args, "%" SCNxTADDR ":%n", &addr, &off) != 1) {
		DBFPRINTF("failed to parse addr '%s'\n", args);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	DBFPRINTF("addr %08" PRIxTADDR " (plen %" PRIu32 ")\n", addr, plen);

#ifdef GDB_TIMED_FLASH_WRITE
	timer_get_mono_time(&tv_start);
	timer_sub(&tv_start, &tv_last, &tv_diff);
	if (tv_diff.tv_sec >= 1)
		putchar('\n');
	tv_last = tv_start;
	printf("flash_write(%u.%06u): 0x%08" PRIxTADDR " %" PRIu32 ": ",
	    (unsigned int)tv_diff.tv_sec, (unsigned int)tv_diff.tv_usec,
	    addr, plen - off);
	fflush(stdout);
	timer_get_mono_time(&tv_start);
#endif

	rv = tmon_flash_write(gc->gc_tmon, addr, plen - off, &args[off]);

#ifdef GDB_TIMED_FLASH_WRITE
	timer_get_mono_time(&tv_stop);
	timer_sub(&tv_stop, &tv_start, &tv_diff);
	printf("%u.%06u\n", (unsigned int)tv_diff.tv_sec,
	    (unsigned int)tv_diff.tv_usec);
#endif

	DBFPRINTF("rv == %d\n", rv);

	switch (rv) {
	case TMON_FL_OK:
		fputs(GDB_RESP_OK, gc->gc_ostream);
		break;
	case TMON_FL_NOT_FLASH:
		fputs(GDB_RESP_EFAULT, gc->gc_ostream);
		break;
	default:
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		break;
	}
}

static void
gdb_cmd_flash_done(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{

	(void) args;
	(void) plen;

	DBFPRINTF("\n");

	switch (tmon_flash_done(gc->gc_tmon)) {
	case TMON_FL_OK:
		fputs(GDB_RESP_OK, gc->gc_ostream);
		break;
	case TMON_FL_NOT_FLASH:
		fputs(GDB_RESP_EFAULT, gc->gc_ostream);
		break;
	default:
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		break;
	}
}

static void
gdb_cmd_empty(gdb_context_t gc, const uint8_t *args, uint32_t plen)
{

	(void) gc;
	(void) args;
	(void) plen;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);
}

static void
gdb_cmd_v_packet(gdb_context_t gc, uint32_t plen)
{
	static const struct gdb_string_cmd v_commands[] = {
		STR_CMD("vAttach;", gdb_cmd_attach),
		STR_CMD_F("vFlashErase:", gdb_cmd_flash_erase, GDB_CMD_ATTACHED),
		STR_CMD_F("vFlashWrite:", gdb_cmd_flash_write, GDB_CMD_ATTACHED),
		STR_CMD_F("vFlashDone", gdb_cmd_flash_done, GDB_CMD_ATTACHED),
		STR_CMD("vMustReplyEmpty", gdb_cmd_empty),
		STR_CMD_END
	};

	gdb_string_command(gc, plen, v_commands);
}

static void
gdb_cmd_write_buffer(gdb_context_t gc, uint32_t plen)
{

	target_addr_t addr;
	uint32_t len;
	int off;

	DBFPRINTF("%s\n", (char *)gc->gc_buff);

	if (sscanf((const char *)gc->gc_buff, "X%" SCNxTADDR ",%" SCNx32 ":%n",
	    &addr, &len, &off) != 2 || plen < (uint32_t)off) {
		DBFPRINTF("failed to parse addr/len '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	plen -= (uint32_t)off;
	if (plen != len) {
		DBFPRINTF("remaining plen (%" PRIu32 ") != len (%" PRIu32 ")\n",
		    plen, len);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	DBFPRINTF("addr 0x%08" PRIxTADDR ", len 0x%" PRIx32 "\n", addr, len);

	if (len != 0 && tmon_mem_write(gc->gc_tmon, addr, len,
	    &gc->gc_buff[off]) < 0) {
		DBFPRINTF("failed\n");
		fputs(GDB_RESP_EFAULT, gc->gc_ostream);
	} else {
		fputs(GDB_RESP_OK, gc->gc_ostream);
	}
}

static void
gdb_cmd_z_packet(gdb_context_t gc, uint32_t plen)
{
	target_addr_t addr;
	struct tmon_breakwatch tb;
	unsigned int type, kind;
	int rv;

	DBFPRINTF("%" PRIu32 ": %s\n", plen, (char *)gc->gc_buff);

	if (plen < 6) {
		fputs(GDB_RESP_DEAD, gc->gc_ostream);
		return;
	}

	if (sscanf((const char *)&gc->gc_buff[1], "%u,%" SCNxTADDR ",%x", &type,
	    &addr, &kind) != 3 || type > TARGET_BKWP_TYPE_ACCESS_WATCH) {
		DBFPRINTF("failed to parse type/addr/kind '%s'\n", gc->gc_buff);
		fputs(GDB_RESP_EINVAL, gc->gc_ostream);
		return;
	}

	tb.tb_address = addr;
	tb.tb_type = type;
	tb.tb_kind = kind;

	rv = tmon_breakwatch(gc->gc_tmon, &tb,
	    (gc->gc_buff[0] == (uint8_t)'Z'));
	 
	switch (rv) {
	case TMON_BW_OK:
		fputs(GDB_RESP_OK, gc->gc_ostream);
		break;
	case TMON_BW_UNSUPPORTED:
		/* Caller will return empty string */
		break;
	default:
		fputs(GDB_RESP_ENOENT, gc->gc_ostream);
		break;
	}
}

static void
gdb_server_link_set_events(struct gdb_server_link_state *gls, uint32_t ev)
{
	struct gdb_server_state *gs = (struct gdb_server_state *)(uintptr_t)gls;

	rtos_notify_any(gs->gs_task_handle, ev, RTOS_NOTIFY_ACTION_SET_BITS);
}

static uint32_t
gdb_server_link_get_events(struct gdb_server_link_state *gls)
{
	uint32_t bits;

	(void) gls;

	while (rtos_notify_wait(0, 0xfful, &bits, RTOS_MAX_SLEEP_MS) == pdFALSE)
		;

	return bits;
}

/*
 * Invoked by the netusb_mux code whenever it requires a subsequent
 * call to its poll method.
 * This function may be invoked from a hardware interrupt handler.
 */
static void
gdb_server_netusb_mux_poll_cb(void *arg)
{
	struct gdb_server_link_state *gls = arg;

	gdb_server_link_set_events(gls, GDB_SERVER_EVENT_NETUSB_MUX_POLL);
}

/*
 * Invoked by the netusb_mux code to give advance warning of a link
 * hangup event. We use this to break out of a blocking rinbuffer
 * read/write.
 * This function may be invoked from a hardware interrupt handler.
 */
static void
gdb_server_netusb_mux_hangup_cb(void *arg)
{
	struct gdb_server_link_state *gls = arg;

	gls->gls_hangup = 1;
}

/*
 * Establish a netusb mux context and wait around until we have
 * a connection.
 */
static void
gdb_server_link_open(struct gdb_server_link_state *gls)
{
	struct gdb_server_state *gs = (struct gdb_server_state *)(uintptr_t)gls;
	struct netusb_mux_attach_args aa;
	struct target_comms_config tcc;
	target_comms_port_t port;
	uint32_t events;

	(void) gs;

	DBFPRINTF("%u: link open\n", gs->gs_server_ctx.gc_instance);

	/*
	 * Allocate and initialise the ring buffers. This should never
	 * fail.
	 */
	gls->gls_to_host = ringbuff_alloc(gls->gls_rb_to_host,
	    sizeof(gls->gls_rb_to_host));
	assert(gls->gls_to_host != NULL);

	gls->gls_from_host = ringbuff_alloc(gls->gls_rb_from_host,
	    sizeof(gls->gls_rb_from_host));
	assert(gls->gls_from_host != NULL);

	gls->gls_from_host_blk = ringbuff_blocking_consumer(gls->gls_from_host,
	    GDB_SERVER_NETWORK_RX_TIMEOUT);
	assert(gls->gls_from_host_blk != NULL);

	gls->gls_to_host_blk = ringbuff_blocking_producer(gls->gls_to_host,
	    GDB_SERVER_NETWORK_TX_TIMEOUT);
	assert(gls->gls_to_host_blk != NULL);

	gls->gls_hangup = 0;
	gls->gls_connected = 0;

#if (CONFIG_GDB_SERVERS == 2)
	if (gs->gs_server_ctx.gc_instance == 0)
		port = TARGET_COMMS_PORT_GDB_SERVER0;
	else
		port = TARGET_COMMS_PORT_GDB_SERVER1;
#else
	port = TARGET_COMMS_PORT_GDB_SERVER0;
#endif
	if (target_comms_get_config(port, &tcc) == false ||
	    (tcc.tcc_usb_port == TARGET_COMMS_USB_PORT_UNUSED &&
	     tcc.tcc_tcp_port == TARGET_COMMS_TCP_PORT_UNUSED)) {
		return;
	}

	aa.aa_poll_cb = gdb_server_netusb_mux_poll_cb;
	aa.aa_hangup_cb = gdb_server_netusb_mux_hangup_cb;
	aa.aa_cb_arg = gls;
	aa.aa_rb_to_host = gls->gls_to_host;
	aa.aa_rb_from_host = gls->gls_from_host;
#ifdef CONFIG_USE_NETWORK
	aa.aa_tcp_port = tcc.tcc_tcp_port;
	aa.aa_do_telnet = false;
	aa.aa_do_nagle = !!(tcc.tcc_flags & TARGET_COMMS_FLAGS_DO_NAGLE);
#endif
	aa.aa_usb_port = tcc.tcc_usb_port;
	aa.aa_usb_cfg = 0;

	gls->gls_netusb_mux = netusb_mux_attach(&aa);
	assert(gls->gls_netusb_mux != NULL);

	/* netusb_mux requires at least one poll on startup. */
	events = GDB_SERVER_EVENT_NETUSB_MUX_POLL;

	/*
	 * Wait around until the host connects
	 */
	while (gls->gls_connected == 0) {
		DBFPRINTF("link: events %02" PRIx32 "\n", events);

		if (events & GDB_SERVER_EVENT_NETUSB_MUX_POLL) {
			netusb_mux_event_t ev;

			while ((ev = netusb_mux_poll(gls->gls_netusb_mux,
			    NULL)) != NETUSB_MUX_EVENT_NONE) {

				DBFPRINTF("link: mux %u\n", ev);
				if (ev == NETUSB_MUX_EVENT_CONNECT_NET ||
				    ev == NETUSB_MUX_EVENT_CONNECT_USB) {
					DBFPRINTF("connected\n");
					gls->gls_connected = 1;
				} else
				if (ev == NETUSB_MUX_EVENT_DISCONNECT)
					gls->gls_connected = 0;
                        }
		}

		if (events & GDB_SERVER_EVENT_TARGET) {
			DBFPRINTF("%u: target event\n",
			    gs->gs_server_ctx.gc_instance);
			gls->gls_connected = 0;
			break;
		}

		if (gls->gls_connected == 0)
			events = gdb_server_link_get_events(gls);
	}

	/* Connection state indicated in gls->gls_connected */
}

static void
gdb_server_link_close(struct gdb_server_link_state *gls)
{
	ringbuff_blocking_t rbl;

	/*
	 * Tear down the mux connection, if it's valid.
	 */
	if (gls->gls_netusb_mux != NULL) {
		netusb_mux_state_t ms = gls->gls_netusb_mux;
		gls->gls_netusb_mux = NULL;
		netusb_mux_detach(ms);
	}

	gls->gls_connected = 0;
	gls->gls_hangup = 1;

	/*
	 * Close the ring buffers.
	 */
	rbl = gls->gls_from_host_blk;
	gls->gls_from_host_blk = NULL;
	if (rbl != NULL)
		ringbuff_blocking_close(rbl);
	if (gls->gls_from_host != NULL) {
		ringbuff_free(gls->gls_from_host);
		gls->gls_from_host = NULL;
	}

	rbl = gls->gls_to_host_blk;
	gls->gls_to_host_blk = NULL;
	if (rbl != NULL)
		ringbuff_blocking_close(rbl);
	if (gls->gls_to_host != NULL) {
		ringbuff_free(gls->gls_to_host);
		gls->gls_to_host = NULL;
	}
}

static int
gdb_server_link_write(struct gdb_server_link_state *gls, const void *src,
    uint32_t len)
{
	int rv;

	if (gls->gls_to_host_blk == NULL || gls->gls_hangup)
		return -1;

	do {
		rv = ringbuff_blocking_write(gls->gls_to_host_blk, src,
		    (ringbuff_len_t)len);
		if (rv > 0)
			ringbuff_produce_done(gls->gls_to_host);
	} while (rv <= 0 && gls->gls_hangup == 0);

	return rv;
}

static int
gdb_server_link_read(struct gdb_server_link_state *gls, void *dst, uint32_t len)
{
	int rv;

	if (gls->gls_from_host_blk == NULL || gls->gls_hangup)
		return -1;

	do {
		rv = (ringbuff_blocking_read(gls->gls_from_host_blk, dst,
		    (ringbuff_len_t)len));
	} while (rv <= 0 && gls->gls_hangup == 0);

	return rv;
}

static bool
gdb_server_link_up(struct gdb_server_link_state *gls)
{

	return !(gls->gls_hangup || gls->gls_from_host == NULL ||
	    gls->gls_from_host_blk == NULL);
}

static ringbuff_len_t
gdb_server_link_poll(struct gdb_server_link_state *gls)
{

	return ringbuff_get_count(gls->gls_from_host);
}

static void
gdb_server_task(void *arg)
{
	struct gdb_server_state *gs = arg;
	struct gdb_context *gc = &gs->gs_server_ctx;
	unsigned int instance;

	rtos_task_sleep(1000u);

	instance = gc->gc_instance;

	for (;;) {
		gdb_server_link_open(&gs->gs_link_state);

		if (gs->gs_link_state.gls_connected) {
			memset(gc, 0, sizeof(*gc));
			gc->gc_instance = instance;
			gc->gc_target_num = instance;
			gc->gc_link = &gs->gs_link_state;
			gc->gc_buff = gs->gs_gdb_comms_buff;
			gc->gc_buff_size = sizeof(gs->gs_gdb_comms_buff);
			gc->gc_mem_buff = gs->gs_gdb_mem_buff;
			gc->gc_mem_buff_len = sizeof(gs->gs_gdb_mem_buff);
			gc->gc_hex_buff = gs->gs_gdb_hex_buff;
			gc->gc_hex_buff_len = sizeof(gs->gs_gdb_hex_buff);

			gdb_server(gc);
		}

		gdb_server_link_close(&gs->gs_link_state);
	}
}

int
gdb_server_get_instance(void)
{
	rtos_task_t t = rtos_task_current();

	for (int i = 0; i < CONFIG_GDB_SERVERS; i++) {
		if (gdb_server_state[i].gs_task_handle == t)
			return i;
	}

	return -1;
}

void
gdb_server_init(void)
{
	struct gdb_server_state *gs = gdb_server_state;
	char name[configMAX_TASK_NAME_LEN];
	unsigned int i;

#ifdef CONFIG_GDB_RTOS_SUPPORT
	gdb_rtos_init();
#endif

	for (i = 0; i < CONFIG_GDB_SERVERS; i++, gs++) {
		gs->gs_server_ctx.gc_instance = i;
		snprintf(name, sizeof(name), "GdbServer%u", i);
		gs->gs_task_handle = rtos_task_create(name, gdb_server_task, gs,
		    RTOS_TASK_PRIORITY_MED, RTOS_STACK_SIZE(1800));
		assert(gs->gs_task_handle != NULL);
	}
}

bool
gdb_server_check_for_interrupt(void *handle)
{
	gdb_context_t gc = handle;
	bool rv;

	switch (gdb_read_character(gc, false)) {
	case 0x03u:
	case 0x04u:
		rv = true;
		break;
	default:
		rv = false;
		break;
	}

	return rv;
}

int
gdb_server_fio_request(void *handle, const char *reqstr,
    struct gdb_server_fio_result *result)
{
	gdb_context_t gc = handle;
	unsigned int ctrlc;
	int rv;

	assert(gc != NULL);
	assert(reqstr != NULL);
	assert(result != NULL);
	assert(strlen(reqstr) > 1);
	assert(reqstr[0] == 'F');

	DBFPRINTF("%s\n", reqstr);

	/* Send the request string to the host. */
	fputs(reqstr, gc->gc_ostream);
	gdb_flush_packet(gc);

	/*
	 * Back to the server loop to process any memory read/write
	 * requests in the usual way.
	 */
	gdb_server_loop(gc, true);

	DBFPRINTF("reply %s\n", (const char *)gc->gc_buff);

	rv = TMON_HS_RUNNING;
	ctrlc = 0;

	/*
	 * The loop returns only when an 'F' reply is received.
	 */
	switch (sscanf((const char *)gc->gc_buff, "F%x,%x,%x",
	    (unsigned int *)&result->retcode,
	    (unsigned int *)&result->gdb_errno, &ctrlc)) {
	case 1:
		/* A single result code. */
		result->gdb_errno = 0;
		break;

	case 2:
		/* A result and errno. */
		break;

	case 3:
		/* A result, but CTRL-C pressed? */
		if (ctrlc == 0x0cu)
			rv = TMON_HS_FORCED;
		break;

	default:
		/*
		 * An unexpected response. We can't reply to the host,
		 * but we can fake a result for the target.
		 */
		result->retcode = -1;
		result->gdb_errno = 22;	/* EINVAL */
		break;
	}

	return rv;
}

void
gdb_server_load_defaults(void)
{

#ifdef CONFIG_GDB_RTOS_SUPPORT
	gdb_rtos_load_defaults();
#endif
}
