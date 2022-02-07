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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "rtos.h"
#include "shell.h"
#include "ringbuff.h"
#include "zone_alloc.h"

#ifndef SHELL_WELCOME
#define	SHELL_WELCOME(x)	/* Nothing */
#endif

SHELL_CMD_DECL(help, shell_cmd_help, "Displays this help");
SHELL_CMD_DECL(reboot, shell_cmd_reboot, "Reboots this device");

static struct shell_command *shell_commands;
static rtos_mutex_static_t shell_cmd_mutex_store;
static rtos_mutex_t shell_cmd_mutex;

static void (*shell_reboot_fn)(int, int);

static void
shell_cmd_lock(void)
{
	rtos_saved_ipl_t ipl;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	if (shell_cmd_mutex == NULL) {
		shell_cmd_mutex = rtos_mutex_create_static(
		    &shell_cmd_mutex_store);
	}
	rtos_ipl_restore(ipl);

	if (rtos_scheduler_running())
		rtos_mutex_acquire(shell_cmd_mutex);
}

static void
shell_cmd_unlock(void)
{

	if (rtos_scheduler_running())
		rtos_mutex_release(shell_cmd_mutex);
}

static void
shell_cmd_help(FILE *os, uint8_t argc, const char * const *argv)
{
	struct shell_command_def def;
	struct shell_command *sc;

	(void) argc;
	(void) argv;

	shell_cmd_lock();

	for (sc = shell_commands; sc != NULL; sc = sc->_sc_next) {
		memcpy(&def, sc->sc_def, sizeof(def));
		if (def.sc_cmd_help) {
			fprintf(os, "%-8s %s\n", def.sc_cmd_string,
			    def.sc_cmd_help);
		}
	}

	shell_cmd_unlock();
}

static void
shell_cmd_reboot(FILE *os, uint8_t argc, const char * const *argv)
{
	int to_bootloader;
	int to_defaults;

	to_bootloader = (argc == 1 && strcasecmp(argv[0], "bl") == 0);
	to_defaults = (argc ==2 && strcasecmp(argv[0], "defaults") == 0 &&
	    strcasecmp(argv[1], "yesplease") == 0);

	(void) os;

	if (shell_reboot_fn != NULL)
		(shell_reboot_fn)(to_defaults, to_bootloader);

	(void) rtos_ipl_raise(HW_IPL_MAX);
	for (;;);
}

static char *
shell_skip_space(char *s)
{

	while (*s != '\0' && isspace((unsigned char)(*s)))
		s++;

	return s;
}

uint8_t
shell_tokenise(const char **argv, uint8_t max_args, char *src)
{
	uint8_t nargs, escaped, in_quote;
	char *dest, ch;

	/* Skip any leading white space */
	dest = src = shell_skip_space(src);
	if (*src == '\0')
		return 0;

	/* Prime the first arg */
	nargs = 0;
	argv[0] = dest;

	escaped = 0;
	in_quote = 0;

	do {
		ch = *src++;

		if (escaped && ch != '\0') {
			escaped = 0;
			*dest++ = ch;
			continue;
		}

		switch (ch) {
		case '\0':
			*dest = '\0';
			nargs++;
			break;

		case '\\':
			escaped = 1;
			break;

		case '"':
			in_quote = 1 - in_quote;
			break;

		case ' ':
			if (in_quote == 0) {
				*dest++ = '\0';
				src = shell_skip_space(src);
				if (*src != '\0') {
					nargs++;
					if (nargs == max_args)
						return nargs;
					argv[nargs] = dest;
				}
				break;
			}
			/*FALLTHROUGH*/

		default:
			*dest++ = ch;
			break;
		}
	} while (ch != '\0');

	return nargs;
}

void
shell_execute_cmd(const struct shell_command *cmd_list, const char **argv,
    uint8_t argc, FILE *os, const char *err_str)
{
	const struct shell_command *match = NULL;
	uint8_t ambig = 0;
	size_t cmdlen;

	cmdlen = strlen(argv[0]);

	if (cmd_list == NULL)
		cmd_list = shell_commands;

	shell_cmd_lock();

	for (; cmd_list != NULL; cmd_list = cmd_list->_sc_next) {
		if (strncasecmp(argv[0], cmd_list->sc_def->sc_cmd_string,
		    cmdlen) != 0) {
			continue;
		}

		if (match == NULL) {
			match = cmd_list;
			if (strlen(cmd_list->sc_def->sc_cmd_string) == cmdlen) {
				/* Exact match */
				break;
			}
		} else {
			if (ambig == 0) {
				fprintf(os, "Ambiguous command:\n\t%s\n",
				    match->sc_def->sc_cmd_string);
			}

			ambig++;
			fprintf(os, "\t%s\n", cmd_list->sc_def->sc_cmd_string);
		}
	}

	shell_cmd_unlock();

	if (ambig == 0 && match != NULL) {
		if (match->_sc_ctx != NULL) {
			void (*func)(FILE *, uint8_t, const char * const *, void *);
			func = match->sc_def->sc_cmd_u.sc_cmd_ctx_func;
			(*func)(os, argc - 1, &argv[1], match->_sc_ctx);
		} else {
			void (*func)(FILE *, uint8_t, const char * const *);
			func = match->sc_def->sc_cmd_u.sc_cmd_func;
			(*func)(os, argc - 1, &argv[1]);
		}
	} else
	if (ambig == 0) {
		if (err_str == NULL)
			err_str = "Unknown command. Have you tried 'help'?\n";
		fputs(err_str, os);
	}
}

static void
shell_do(struct shell_state *ss)
{
	uint8_t len, got;
	char ch;

	got = 0;

	while (!got && ringbuff_get_count(ss->ss_input)) {
		ch = ringbuff_consume(ss->ss_input);
		ringbuff_consume_done(ss->ss_input);

		if (ch == '\n' || ch == '\r') {
			ss->ss_cmdline[ss->ss_cmdline_len++] = '\0';
			got = 1;
		} else
		if (ch == '\b' || ch == 0x7f) {
			if (ss->ss_cmdline_len > 0) {
				ss->ss_cmdline_len -= 1;
				if ((ss->ss_flags & SHELL_FLAGS_NO_ECHO) == 0) {
					fputc('\b', ss->ss_ostream);
					fputc(' ', ss->ss_ostream);
					fputc('\b', ss->ss_ostream);
				}
			}
		} else
		if (ch >= ' ' && ch <= 0x7e &&
		    ss->ss_cmdline_len < (SHELL_CMDLINE_SIZE - 1)) {
			ss->ss_cmdline[ss->ss_cmdline_len++] = ch;
			if ((ss->ss_flags & SHELL_FLAGS_NO_ECHO) == 0)
				fputc(ch, ss->ss_ostream);
		}
		fflush(ss->ss_ostream);
	}

	if (!got || ss->ss_cmdline_len == 0)
		return;

	if ((ss->ss_flags & SHELL_FLAGS_NO_ECHO) == 0)
		fputc('\n', ss->ss_ostream);

	len = ss->ss_cmdline_len - 1;
	ss->ss_cmdline_len = 0;
	if (len) {
		uint8_t argc;

		argc = shell_tokenise(ss->ss_args, SHELL_MAX_ARGS,
		    ss->ss_cmdline);
		if (argc) {
			shell_execute_cmd(NULL, ss->ss_args, argc,
			    ss->ss_ostream, ss->ss_str_error);
		}
	}

	fprintf(ss->ss_ostream, "%s> ", ss->ss_str_prompt ?
	    ss->ss_str_prompt : ss->ss_name);
	fflush(ss->ss_ostream);
}

static void
shell_init_ringbuffs(struct shell_state *ss)
{

	ringbuff_init(ss->ss_input);

#ifdef RINGBUFF_BLOCKING
	if (ss->ss_output_blocking != NULL) {
		ringbuff_blocking_close(ss->ss_output_blocking);
		ss->ss_output_blocking = NULL;
	}
#endif

	ringbuff_init(ss->ss_output);

#ifdef RINGBUFF_BLOCKING
	ss->ss_output_blocking = ringbuff_blocking_producer(ss->ss_output, 0);
#endif
}

static void
shell_input_ready_cb(ringbuff_t rb, void *arg)
{
	struct shell_state *ss = arg;

	(void) rb;

	if (ss->ss_state > SHELL_STATE_ATTACHED) {
		rtos_notify_any(ss->ss_thread, 1u << SHELL_EVENT_INPUT_READY,
		    RTOS_NOTIFY_ACTION_SET_BITS);
	}
}

static void
shell_worker_impl(struct shell_state *ss)
{

	switch (ss->ss_state) {
	case SHELL_STATE_DETACHED:
again:
		if (ss->ss_attach(ss)) {
			if (ss->ss_state == SHELL_STATE_DETACHED)
				ss->ss_state = SHELL_STATE_ATTACHED;
		} else {
			rtos_task_sleep(1000);
			goto again;
		}
		break;

	case SHELL_STATE_ATTACHED:
		/* This state just hangs around until connection established */
		return;

	case SHELL_STATE_CONNECTED:
		if ((ss->ss_flags & SHELL_FLAGS_SUPPRESS_WELCOME) == 0) {
			SHELL_WELCOME(ss->ss_ostream);
		}
		if ((ss->ss_flags & SHELL_FLAGS_SUPPRESS_FIRST_PROMPT) != 0) {
			ss->ss_state = SHELL_STATE_RUNNING;
			break;
		}
		ss->ss_state = SHELL_STATE_SHOW_PROMPT;
		/*FALLTHROUGH*/

	case SHELL_STATE_SHOW_PROMPT:
		ss->ss_state = SHELL_STATE_RUNNING;
		fprintf(ss->ss_ostream, "\n%s> ", ss->ss_name);
		fflush(ss->ss_ostream);
		break;

	default:
		break;
	}

	do {
		shell_do(ss);
		if (ss->ss_output)
			ringbuff_produce_done(ss->ss_output);
	} while (ringbuff_get_count(ss->ss_input) != 0);
}

static void
shell_ready(struct shell_state *ss)
{

	ss->ss_state = SHELL_STATE_CONNECTED;
	ss->ss_cmdline_len = 0;
	ringbuff_consumer_init(ss->ss_input, shell_input_ready_cb, ss);
}

static void
shell_detach(struct shell_state *ss)
{

	ss->ss_state = SHELL_STATE_DETACHED;
	shell_init_ringbuffs(ss);
}

static void
shell_worker(void *arg)
{
	struct shell_state *ss = arg;
	uint32_t event;

	rtos_task_sleep(500);

	for (;;) {
		shell_worker_impl(ss);

		while (!rtos_notify_wait(0, 0xfful, &event, RTOS_MAX_SLEEP_MS))
			;

		if (event & (1u << SHELL_EVENT_CONNECT)) {
			if (ss->ss_event != NULL)
				(ss->ss_event)(ss, SHELL_EVENT_CONNECT);
		}

		if (event & (1u << SHELL_EVENT_CONNECTED)) {
			if (ss->ss_event != NULL)
				(ss->ss_event)(ss, SHELL_EVENT_CONNECTED);
			shell_ready(ss);
		}

		if (event & (1u << SHELL_EVENT_DISCONNECTED)) {
			if (ss->ss_event != NULL)
				(ss->ss_event)(ss, SHELL_EVENT_DISCONNECTED);
			shell_detach(ss);
		}
	}
}

void
shell_event(struct shell_state *ss, shell_event_t ev)
{

	rtos_notify(ss->ss_thread, 1u << ev, RTOS_NOTIFY_ACTION_SET_BITS);
}

void
shell_add_command(struct shell_command *new_sc, struct shell_command **psc)
{
	const char *cmd;

	assert(new_sc->_sc_next == NULL);

	shell_cmd_lock();

	cmd = new_sc->sc_def->sc_cmd_string;

	if (psc == NULL)
		psc = &shell_commands;

	for (; *psc != NULL; psc = &(*psc)->_sc_next) {
		/* Ensure command is not already on the list. */
		assert(*psc != new_sc);

		if (strcasecmp(cmd, (*psc)->sc_def->sc_cmd_string) < 0)
			break;
	}

	new_sc->_sc_next = *psc;
	*psc = new_sc;

	shell_cmd_unlock();
}

void
shell_del_command(struct shell_command *old_sc)
{
	struct shell_command **psc;

	shell_cmd_lock();

	for (psc = &shell_commands; *psc != NULL; psc = &(*psc)->_sc_next) {
		if (*psc == old_sc) {
			*psc = old_sc->_sc_next;
			old_sc->_sc_next = NULL;
			break;
		}
	}

	shell_cmd_unlock();
}

void
shell_attach(struct shell_state *ss)
{
	int rv;

	if (ss->ss_ostream != stdout) {
		rv = shell_platform_new_file(ss);
		assert(rv >= 0);
		(void) rv;

		ss->ss_input = ringbuff_alloc(
#ifndef RINGBUFF_SIZE
		    ss->ss_rx_ring, SHELL_RX_RING_SIZE
#endif
		    );
		assert(ss->ss_input != NULL);

		ss->ss_output = ringbuff_alloc(
#ifndef RINGBUFF_SIZE
		    ss->ss_tx_ring, SHELL_TX_RING_SIZE
#endif
		    );
		assert(ss->ss_output != NULL);

		shell_init_ringbuffs(ss);
	} else {
		ss->ss_output = NULL;
	}

	ss->ss_state = SHELL_STATE_DETACHED;
	ss->ss_cmdline_len = 0;

	ss->ss_thread = rtos_task_create(ss->ss_name, shell_worker, ss,
	    RTOS_TASK_PRIORITY_MED, RTOS_STACK_SIZE(1200));
	assert(ss->ss_thread != NULL);
}

static uint8_t
shell_console_attach(struct shell_state *ss)
{

	shell_event(ss, SHELL_EVENT_CONNECTED);
	return 1;
}

void
shell_puts_wrap(FILE *fp, unsigned int cols, const char *str,
    const char *prefix)
{
	size_t cnt, prefix_len;
	char *tok, *ctx, *scopy;

	if ((scopy = zone_strdup(str)) == NULL)
		return;

	if (prefix != NULL) {
		prefix_len = strlen(prefix);
		fputs(prefix, fp);
	} else {
		prefix_len = 0;
	}

	cnt = prefix_len;

	for (tok = strtok_r(scopy, " \t\n", &ctx); tok != NULL;
	    tok = strtok_r(NULL, " \t\n", &ctx)) {
		size_t tok_len = strlen(tok);

		if (cnt != prefix_len)
			tok_len += 1;

		if ((tok_len + cnt) > cols) {
			fputc('\n', fp);
			if (prefix != NULL)
				fputs(prefix, fp);
			cnt = prefix_len;
		}

		if (cnt != prefix_len)
			fputc(' ', fp);
		cnt += tok_len;
		fputs(tok, fp);
	}

	fputc('\n', fp);
	zone_free(scopy);
}

void
shell_init(ringbuff_t cons_input_rb, void (*reboot_fn)(int, int))
{

	shell_reboot_fn = reboot_fn;

	if (cons_input_rb != NULL) {
		struct shell_state *ss;
		static struct shell_state shell_console_state;

		ss = &shell_console_state;
		ss->ss_ostream = stdout;
		ss->ss_name = "ConShell";
		ss->ss_flags = 0;
		ss->ss_input = cons_input_rb;
		ss->ss_attach = shell_console_attach;
		shell_attach(ss);
	}

	SHELL_CMD_ADD(help);
	SHELL_CMD_ADD(reboot);
}
