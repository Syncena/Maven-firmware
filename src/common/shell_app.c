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
#include <stdio.h>
#include <string.h>
#if (RELEASE_BUILD == 0)
#include <ctype.h>
#endif

#ifdef CONFIG_USE_NETWORK
#include "network.h"
#endif
#include "version.h"
#include "platform.h"
#include "product.h"
#include "serial_number.h"
#include "shell.h"

struct shell_state;

static int
shell_platform_write(void *arg, const char *buf, int buflen)
{
	struct shell_state *ss = arg;
	ringbuff_len_t remlen, thislen;
	int rv = buflen;
	char *p;

	if (ss->ss_output == NULL || ss->ss_state <= SHELL_STATE_ATTACHED)
		return buflen;

	remlen = (ringbuff_len_t)buflen;

	while (remlen) {
		p = memchr(buf, '\n', remlen);
		if (p == NULL) {
			/* No LF. Send the remainder. */
			thislen = remlen;
		} else {
			/* Found LF. Output the preceding data */
			thislen = (ringbuff_len_t)((uintptr_t)p -
			    (uintptr_t)buf);
		}

		if (thislen) {
			if (ringbuff_blocking_write(ss->ss_output_blocking,
			    buf, thislen) < 0) {
				return -1;
			}
		}

		if (p) {
			/* Print the CRLF */
			if (ringbuff_blocking_write(ss->ss_output_blocking,
			    "\r\n", 2) < 0) {
				return -1;
			}
			thislen += 1;
		}

		remlen -= thislen;
		buf += thislen;
	}

	return rv;
}

int
shell_platform_new_file(struct shell_state *ss)
{

	if (ss->ss_ostream == stdout)
		return 0;

	assert(ss->ss_ostream == NULL);

	ss->ss_ostream = fwopen(ss, shell_platform_write);
	assert(ss->ss_ostream != NULL);

	return 0;
}

struct app_shell_state {
	struct shell_state ass_shell;
	void *ass_handle;
};

#ifdef CONFIG_USE_NETWORK
static struct app_shell_state app_shell_network;
#endif

SHELL_CMD_DECL(serial, serial_cmd, "Show this device's serial number");

static void
serial_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	const serial_number_t *sn;

	(void) argc;
	(void) argv;

	sn = serial_number_fetch();
	assert(sn != NULL);

	fprintf(os, "Serial number: %s\n", sn->cstr);
}

#ifdef CONFIG_USE_NETWORK
static void
app_shell_network_status_cb(network_sock_status_t st, void *arg)
{
	struct app_shell_state *ass = arg;

	switch (st) {
	case NETWORK_STATUS_CONNECT:
		shell_event(&ass->ass_shell, SHELL_EVENT_CONNECT);
		break;

	case NETWORK_STATUS_DISCONNECT:
	case NETWORK_STATUS_TIMEOUT:
	case NETWORK_STATUS_LINK_DOWN:
	case NETWORK_STATUS_UNREACHABLE:
		shell_event(&ass->ass_shell, SHELL_EVENT_DISCONNECTED);
		break;

	case NETWORK_STATUS_TELNET_READY:
		shell_event(&ass->ass_shell, SHELL_EVENT_CONNECTED);
		break;

	default:
		break;
	}
}

static void
app_shell_network_event(struct shell_state *ss, shell_event_t ev)
{
	struct app_shell_state *ass = (struct app_shell_state *)ss;

	switch (ev) {
	case SHELL_EVENT_CONNECT:
		if (ass->ass_handle != NULL &&
		    network_establish(ass->ass_handle,
		    ass->ass_shell.ss_output,
		    ass->ass_shell.ss_input, 1, NULL) == 0) {
			break;
		}
		/*FALLTHROUGH*/

	case SHELL_EVENT_DISCONNECTED:
		if (ass->ass_handle) {
			network_close(ass->ass_handle);
			ass->ass_handle = NULL;
		}
		break;

	default:
		break;
	}
}

static uint8_t
app_shell_network_attach(struct shell_state *ss)
{
	struct app_shell_state *ass = (void *)ss;

	ass->ass_handle = network_socket_listen(23, app_shell_network_status_cb,
	    ass);

	return ass->ass_handle != NULL;
}
#endif /* CONFIG_USE_NETWORK */

void
app_shell_welcome(FILE *fp)
{

	fprintf(fp, "%s", product_copyright);
	fprintf(fp, "Firmware Version: " PRODUCT_VERSION_MAJOR "."
            PRODUCT_VERSION_MINOR "." PRODUCT_VERSION_TINY "\n");
	fprintf(fp, "Firmware Build Date: " PRODUCT_BUILD_DATE ", "
            PRODUCT_BUILD_TIME "\n");
}

void
app_shell_init(void)
{

	SHELL_CMD_ADD(serial);

#ifdef CONFIG_USE_NETWORK
	struct app_shell_state *ass;

	ass = &app_shell_network;
	ass->ass_shell.ss_name = PRODUCT_NAME;
	ass->ass_shell.ss_flags = 0;
	ass->ass_shell.ss_ostream = NULL;
	ass->ass_shell.ss_attach = app_shell_network_attach;
	ass->ass_shell.ss_event = app_shell_network_event;
	shell_attach(&ass->ass_shell);
#endif
}
