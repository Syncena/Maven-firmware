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

/*
 * Common shell code available options:
 *
 * SHELL_CMDLINE_SIZE
 *	Specify command line buffer size (defaults to 64 if not specified).
 * SHELL_MAX_ARGS
 *	Specify max number of parameters per command (default 8)
 * SHELL_RX_RING_SIZE
 *	Ring buffer size for shell input. Defaults to 32 bytes.
 * SHELL_TX_RING_SIZE
 *	Ring buffer size for shell output. Defaults to 128 bytes.
 */

#ifndef SHELL_H
#define	SHELL_H

#include "rtos.h"
#include "ringbuff.h"

#ifndef SHELL_CMDLINE_SIZE
#define	SHELL_CMDLINE_SIZE	128
#endif
#ifndef SHELL_MAX_ARGS
#define	SHELL_MAX_ARGS		8
#endif
#ifndef SHELL_RX_RING_SIZE
#define	SHELL_RX_RING_SIZE	16
#endif
#ifndef SHELL_TX_RING_SIZE
#define	SHELL_TX_RING_SIZE	128
#endif

#define	SHELL_STATE_DETACHED		0
#define	SHELL_STATE_ATTACHED		1
#define	SHELL_STATE_CONNECTED		2
#define	SHELL_STATE_SHOW_PROMPT		3
#define	SHELL_STATE_RUNNING		4

typedef enum {
	SHELL_EVENT_CONNECT = 0,
	SHELL_EVENT_CONNECTED,
	SHELL_EVENT_DISCONNECTED,
	SHELL_EVENT_INPUT_READY,
} shell_event_t;

struct shell_state {
	/* Application initialises this before calling shell_add() */
	const char *ss_name;
	FILE *ss_ostream;
	uint8_t ss_flags;
#define	SHELL_FLAGS_NO_ECHO			(1u << 0)
#define	SHELL_FLAGS_SUPPRESS_WELCOME		(1u << 1)
#define	SHELL_FLAGS_SUPPRESS_FIRST_PROMPT	(1u << 2)

	const char *ss_str_prompt;
	const char *ss_str_error;
	uint8_t (*ss_attach)(struct shell_state *);
	void (*ss_event)(struct shell_state *, shell_event_t);

	/* These can be referenced during attach */
	ringbuff_t ss_input;
	ringbuff_t ss_output;

	/* Touch nothing below */
#ifdef RINGBUFF_BLOCKING
	ringbuff_blocking_t ss_output_blocking;
#endif
	uint8_t ss_state;
	uint8_t ss_cmdline_len;
	rtos_task_t ss_thread;
	const char *ss_args[SHELL_MAX_ARGS];
	char ss_cmdline[SHELL_CMDLINE_SIZE];
	uint8_t ss_rx_ring[SHELL_RX_RING_SIZE];
        uint8_t ss_tx_ring[SHELL_TX_RING_SIZE];
};

struct shell_command_def {
	const char *sc_cmd_string;
	const char *sc_cmd_help;
	union {
		void (*sc_cmd_func)(FILE *, uint8_t argc, const char * const *argv);
		void (*sc_cmd_ctx_func)(FILE *, uint8_t argc, const char * const *argv,
					void *);
	} sc_cmd_u;
};

struct shell_command {
	const struct shell_command_def *sc_def;
	struct shell_command *_sc_next;
	void *_sc_ctx;
};

#define	SHELL_CMD_DECL(name, func, help)				\
	static void func (FILE *, uint8_t, const char * const *);	\
	static const char _shell_cmd_help_##name[] = help;		\
	static const struct shell_command_def _shell_cmd_##name = {	\
		.sc_cmd_string = #name,					\
		.sc_cmd_help = _shell_cmd_help_##name,			\
		.sc_cmd_u.sc_cmd_func = func				\
	};								\
	static struct shell_command _shell_command_##name = {		\
		.sc_def = &_shell_cmd_##name				\
	}

#define	SHELL_CMD_DECL_CTX(name, func, help)				\
	static void func (FILE *, uint8_t, const char * const *, void *); \
	static const char _shell_cmd_help_##name[] = help;		\
	static const struct shell_command_def _shell_cmd_##name = {	\
		.sc_cmd_string = #name,					\
		.sc_cmd_help = _shell_cmd_help_##name,			\
		.sc_cmd_u.sc_cmd_ctx_func = func			\
	};								\
	static struct shell_command _shell_command_##name = {		\
		.sc_def = &_shell_cmd_##name				\
	}

#define	SHELL_CMD_ADD(name)						\
	do {								\
		_shell_command_##name . _sc_ctx = NULL;			\
		shell_add_command(&_shell_command_##name, NULL);	\
	} while (0)
#define	SHELL_CMD_ADD_CUST(name, list)					\
	do {								\
		_shell_command_##name . _sc_ctx = NULL;			\
		shell_add_command(&_shell_command_##name, &(list));	\
	} while (0)

#define	SHELL_CMD_ADD_CTX(name, ctx)					\
	do {								\
		_shell_command_##name . _sc_ctx = (ctx);		\
		shell_add_command(&_shell_command_##name, NULL);	\
	} while (0)
#define	SHELL_CMD_ADD_CTX_CUST(name, ctx, list)				\
	do {								\
		_shell_command_##name . _sc_ctx = (ctx);		\
		shell_add_command(&_shell_command_##name, &(list));	\
	} while (0)

#define	SHELL_CMD_DEL(name)	shell_del_command(&_shell_command_##name)

extern void	shell_init(ringbuff_t, void (*reboot_fn)(int, int));
extern void	shell_attach(struct shell_state *);
extern void	shell_event(struct shell_state *, shell_event_t);
extern void	shell_add_command(struct shell_command *,
				  struct shell_command **);
extern void	shell_del_command(struct shell_command *);

extern uint8_t	shell_tokenise(const char **argv, uint8_t max_args, char *s);
extern void	shell_execute_cmd(const struct shell_command *cmd_list,
				  const char **argv, uint8_t argc, FILE *os,
				  const char *err_str);
extern void	shell_puts_wrap(FILE *, unsigned int cols, const char *str,
				const char *prefix);

#define	SHELL_WELCOME	app_shell_welcome
extern void	app_shell_init(void);
extern void	app_shell_welcome(FILE *);
extern int	shell_platform_new_file(struct shell_state *ss);

#endif /* SHELL_H */
