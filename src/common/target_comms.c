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
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "target_comms.h"
#include "target_console.h"
#include "ringbuff.h"
#include "platform.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif
#include "linked-lists.h"
#include "product.h"
#include "shell.h"
#include "usb_common.h"
#include "uart.h"
#include "zone_alloc.h"
#ifdef CONFIG_USE_NETWORK
#include "network.h"
#ifdef NETWORK_OPT_HTTPD
#include "fsdata_custom.h"
#include "network_httpd.h"
#include "stringio.h"
#include "json_utils.h"
#endif
#endif /* CONFIG_USE_NETWORK */

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

/*
 * All registered comms sources are assigned one of the following
 * data structures.
 */
#define	TCSS_SETTINGS_USB	0
#define	TCSS_SETTINGS_TCP	1
#define	TCSS_SETTINGS_CNT	2
struct target_comms_source_state {
	/* Pointer to the comms source. */
	struct target_comms_source *css_source;

	/* The Port Daemon instance assigned to this source. */
	port_daemon_instance_t css_port_daemon[TCSS_SETTINGS_CNT];

	/*
	 * Port configuration is stored here. We maintain two copies
	 * in case USB and TCP need to be split.
	 */
	struct port_daemon_settings css_settings[TCSS_SETTINGS_CNT];
};

/*
 * Comms sources which share the same USB and/or TCP port are tracked
 * using one of the following structure.
 */
struct target_comms_shared {
	/* Points to the ops for the currently connected source. */
	port_daemon_ops_t sh_ops;
	/* Parameters for the current ops. */
	target_comms_arg_t sh_ops_arg;

	/* The number of comms sources sharing this port. */
	unsigned int sh_num_sources;

	/* Array of comms sources sharing this port. */
	struct target_comms_source_state *sh_css[TARGET_COMMS_PORT__COUNT];

	/* Array of Port Daemon 'chooser' strings for this port. */
	const char *sh_options[TARGET_COMMS_PORT__COUNT];

	/* The Port Daemon instance assigned to the shared sources. */
	port_daemon_instance_t sh_port_daemon;

	/* Common port configuration for the shared sources. */
	struct port_daemon_settings sh_settings;

	TAILQ_ENTRY(target_comms_shared) sh_qent;
};
TAILQ_HEAD(target_comms_shared_qhead, target_comms_shared);

/*
 * Overall Target Comms state.
 */
struct target_comms_state {
	/* Mutex. Used during registration. */
	rtos_mutex_t tcs_mutex;

	/* Non-zero if target config changed since reboot. */
	bool tcs_changed;

	/* Count and list of all registered sources. */
	unsigned int tcs_num_sources;
	struct target_comms_source_state tcs_sources[TARGET_COMMS_PORT__COUNT];

	/* List of shared sources. */
	struct target_comms_shared_qhead tcs_shared;
};
static struct target_comms_state target_comms_state;

/*
 * Default source/port configuration and mapping. Consulted if there is
 * no persistent configuration for a given port..
 */
static const struct target_comms_config
	tc_default_configs[TARGET_COMMS_PORT__COUNT] = {
    [TARGET_COMMS_PORT_TARGET_CONSOLE] = PLATFORM_TC_DEFAULT_CFG_TARGET_CONSOLE,
#ifdef CONFIG_TARGET_CORTEXM
    [TARGET_COMMS_PORT_GDB_SERVER0]    = PLATFORM_TC_DEFAULT_CFG_GDB_SERVER0,
#if (CONFIG_GDB_SERVERS > 1)
    [TARGET_COMMS_PORT_GDB_SERVER1]    = PLATFORM_TC_DEFAULT_CFG_GDB_SERVER1,
#endif
    [TARGET_COMMS_PORT_SWO]            = PLATFORM_TC_DEFAULT_CFG_SWO,
    [TARGET_COMMS_PORT_SEMIHOST0]      = PLATFORM_TC_DEFAULT_CFG_SEMIHOST0,
#if (CONFIG_GDB_SERVERS > 1)
    [TARGET_COMMS_PORT_SEMIHOST1]      = PLATFORM_TC_DEFAULT_CFG_SEMIHOST1,
#endif
#else
    [TARGET_COMMS_PORT_AVR_PROGRAMMER] = PLATFORM_TC_DEFAULT_CFG_AVR_PROGRAMMER,
#endif
#ifdef PLATFORM_WILC_UART
    [TARGET_COMMS_PORT_WIFI_UART]      = PLATFORM_TC_DEFAULT_CFG_WIFI_UART,
#endif
};
static_assert((sizeof(tc_default_configs)/sizeof(tc_default_configs[0])) ==
    TARGET_COMMS_PORT__COUNT, "tc_default_configs[] size bogons");

/* Names for the target comms sources. */
struct target_comms_name {
	const char *cn_cmd;
	const char *cn_desc;
	const char *cn_menu;
};
static const struct target_comms_name
	target_comms_names[TARGET_COMMS_PORT__COUNT] = {
    [TARGET_COMMS_PORT_TARGET_CONSOLE] = {"serial", "Target Serial Port",
					  "Connect to target serial port."},
#ifdef CONFIG_TARGET_CORTEXM
#if (CONFIG_GDB_SERVERS == 1)
    [TARGET_COMMS_PORT_GDB_SERVER0]    = {"gdb", "Gdb Server", NULL},
#else
    [TARGET_COMMS_PORT_GDB_SERVER0]    = {"gdb0", "CPU core #0 Gdb Server",
					  NULL},
    [TARGET_COMMS_PORT_GDB_SERVER1]    = {"gdb1", "CPU core #1 Gdb Server",
					  NULL},
#endif
    [TARGET_COMMS_PORT_SWO]            = {"swo", "Target SWO",
					  "Connect to target SWO port."},
#if (CONFIG_GDB_SERVERS == 1)
    [TARGET_COMMS_PORT_SEMIHOST0]      = {"semihost", "Semihosted Console I/O",
					  "Connect to Semihost console."},
#else
    [TARGET_COMMS_PORT_SEMIHOST0]      = {"semihost0", "Semihost Console #0",
					  "Connect to Semihost console #0."},
    [TARGET_COMMS_PORT_SEMIHOST1]      = {"semihost1", "Semihost Console #1",
					  "Connect to Semihost console #1."},
#endif
#else  /* CONFIG_TARGET_CORTEXM */
    [TARGET_COMMS_PORT_AVR_PROGRAMMER] = {"avr", "AVR Programmer", NULL},
#endif /* CONFIG_TARGET_CORTEXM */
#ifdef PLATFORM_WILC_UART
    [TARGET_COMMS_PORT_WIFI_UART]      = {"wifi", "WiFi UART",
					  "Connect to WiFi UART port."},
#endif
};
static_assert((sizeof(target_comms_names)/sizeof(target_comms_names[0])) ==
    TARGET_COMMS_PORT__COUNT, "target_comms_names[] size bogons");

#ifdef CONFIG_USE_CONFIGDB
/*
 * Future-proof the configdb entry by versioning this structure.
 */
struct target_comms_config_versioned {
	uint8_t version;
	struct target_comms_config tcc;
};
#define	TC_CONFIG_VERSION	0
#endif /* CONFIG_USE_CONFIGDB */

/* Declare the shell command to manipulate Target Comms configuration. */
SHELL_CMD_DECL(tc, tc_cmd, "Target communications.");

#define TCPRINTF(fp, fmt, ...)					\
	do {							\
		if ((fp) != NULL)				\
			fprintf(fp, fmt, ##__VA_ARGS__);	\
	} while (0)

/*
 * The following functions are used to front-end shared sources.
 */
static bool
tc_op_connect(void *arg, port_daemon_instance_t si, ringbuff_t to_host,
    ringbuff_t from_host, unsigned int choice)
{
	struct target_comms_shared *sh = arg;
	struct target_comms_source_state *css;
	struct target_comms_source *cs;
	bool rv;

	assert(sh->sh_port_daemon == si);

	/*
	 * Validate the user's choice of source, and ensure no other source
	 * is currently active.
	 */
	if (choice >= sh->sh_num_sources || sh->sh_ops != NULL)
		return false;

	/* Locate the chosen source. */
	css = sh->sh_css[choice];
	cs = css->css_source;

	/* Invoke its connect method. */
	rv = cs->cs_ops->op_connect(&cs->cs_arg, si, to_host, from_host, 0);
	if (rv) {
		/* Success. This source is now active. */
		sh->sh_ops = cs->cs_ops;
		sh->sh_ops_arg = &cs->cs_arg;
	}

	return rv;
}

static void
tc_op_disconnect(void *arg)
{
	struct target_comms_shared *sh = arg;

	/* Invoke the source's disconnect method, then we're idle. */
	sh->sh_ops->op_disconnect(sh->sh_ops_arg);
	sh->sh_ops = NULL;
}

static void
tc_op_worker(void *arg)
{
	struct target_comms_shared *sh = arg;

	/* Bounce straight through to the active source's worker method. */
	sh->sh_ops->op_worker(sh->sh_ops_arg);
}

static int
tc_op_ctrl(void *arg, port_daemon_ctrl_t op, const void *oparg)
{
	struct target_comms_shared *sh = arg;

	if (sh->sh_ops->op_ctrl != NULL)
		return sh->sh_ops->op_ctrl(sh->sh_ops_arg, op, oparg);
	return 0;
}

static void
tc_op_hangup(void *arg)
{
	struct target_comms_shared *sh = arg;

	if (sh->sh_ops->op_hangup != NULL)
		sh->sh_ops->op_hangup(sh->sh_ops_arg);
}

/*
 * Fetch the configuration for a specific port.
 */
bool
target_comms_get_config(target_comms_port_t port,
    struct target_comms_config *tcc)
{
	const struct target_comms_config *p;

	assert(port < TARGET_COMMS_PORT__COUNT);

	if (!TARGET_COMMS_PORT_VALID(&tc_default_configs[port]))
		return false;

#ifndef CONFIG_USE_CONFIGDB
	/* The old-skool method. */
	p = &glob.glob_target_comms[port];
#else
	struct target_comms_config_versioned tcc_v;
	cf_len_t l;

	/*
	 * Check the existence and size of a saved config for this port.
	 */
	if ((l = cf_get_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(port), 0,
	    NULL)) != sizeof(tcc_v)) {
		/*
		 * At the moment, we only have a single version. So a size
		 * mismatch here indicates there is no saved config. Use
		 * the default.
		 */
		p = &tc_default_configs[port];
	} else {
		/*
		 * Fetch the saved config.
		 */
		cf_get_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(port), (size_t)l,
		    &tcc_v);

		/* Not strictly required yet, but check version. */
		if (tcc_v.version != TC_CONFIG_VERSION)
			p = &tc_default_configs[port];
		else
			p = &tcc_v.tcc;
	}
#endif /* CONFIG_USE_CONFIGDB */

	/* Copy config to caller's buffer. */
	*tcc = *p;

	return true;
}

/*
 * Save port configuration to persistent storage.
 */
bool
target_comms_set_config(target_comms_port_t port,
    const struct target_comms_config *tcc, bool persist)
{
	bool changed;

	assert(port < TARGET_COMMS_PORT__COUNT);
	assert(tcc->tcc_usb_port < PLATFORM_USB_CDC_INSTANCE_NB);
	assert(tcc->tcc_tcp_port < 65536u);

#ifndef CONFIG_USE_CONFIGDB
	/* Old-skool is easy. */
	changed = memcmp(tcc, &glob.glob_target_comms[port], sizeof(*tcc)) != 0;

	if (changed) {
		glob.glob_target_comms[port] = *tcc;

		if (persist)
			glob_save();
	}
#else
	struct target_comms_config_versioned tcc_v;

	memset(&tcc_v, 0, sizeof(tcc_v));

	if (cf_get_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(port), sizeof(tcc_v),
	    &tcc_v) == 0) {
		tcc_v.tcc = tc_default_configs[port];
	}

	changed = memcmp(tcc, &tcc_v.tcc, sizeof(*tcc)) != 0;

	if (changed) {
		tcc_v.version = TC_CONFIG_VERSION;
		tcc_v.tcc = *tcc;

		cf_set_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(port),
		    sizeof(tcc_v), &tcc_v);

		if (persist)
			configdb_persist();
	}
#endif /* CONFIG_USE_CONFIGDB */

	return changed;
}

static void
tc_show_config_entry(FILE *os, bool header, target_comms_port_t port,
    const struct target_comms_config *tcc)
{

#ifdef CONFIG_USE_NETWORK
#if (RELEASE_BUILD == 0)
	if (header) {
		fprintf(os, "%-10s%-4s%-6s%-7s%-13s%-13s%s\n", "Service", "USB",
		    "TCP", "Nagle", "Mode", "UART Config", "Description");
		for (unsigned int i = 0; i < 64; i++)
			fputc('=', os);
		fputc('\n', os);
	}
#else
	if (header) {
		fprintf(os, "%-10s%-4s%-6s%-13s%-13s%s\n", "Service", "USB",
		    "TCP", "Mode", "UART Config", "Description");
		for (unsigned int i = 0; i < 57; i++)
			fputc('=', os);
		fputc('\n', os);
	}
#endif
#else /* CONFIG_USE_NETWORK */
#if (RELEASE_BUILD == 0)
	if (header) {
		fprintf(os, "%-10s%-4s%-13s%-13s%s\n", "Service", "USB",
		    "Mode", "UART Config", "Description");
		for (unsigned int i = 0; i < 64; i++)
			fputc('=', os);
		fputc('\n', os);
	}
#else
	if (header) {
		fprintf(os, "%-10s%-4s%-13s%-13s%s\n", "Service", "USB",
		    "Mode", "UART Config", "Description");
		for (unsigned int i = 0; i < 57; i++)
			fputc('=', os);
		fputc('\n', os);
	}
#endif
#endif /* CONFIG_USE_NETWORK */

	fprintf(os, "%-10s", target_comms_names[port].cn_cmd);

	if (tcc->tcc_usb_port != TARGET_COMMS_USB_PORT_UNUSED)
		fprintf(os, "%-4d", (int)tcc->tcc_usb_port);
	else
		fprintf(os, "%-4s", "");

#ifdef CONFIG_USE_NETWORK
	if (tcc->tcc_tcp_port != TARGET_COMMS_TCP_PORT_UNUSED)
		fprintf(os, "%-6" PRIu16, tcc->tcc_tcp_port);
	else
		fprintf(os, "%-6s", "");

#if (RELEASE_BUILD == 0)
	fprintf(os, "%-7s", (tcc->tcc_flags & TARGET_COMMS_FLAGS_DO_NAGLE) ?
	    "yes" : "no");
#endif
#endif /* CONFIG_USE_NETWORK */

	const char *m = "";
	if (tcc->tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_MODE) {
		if (TC_FLAGS_IS_RAW(tcc->tcc_flags)) {
			m = "Raw";
		} else
#ifdef CONFIG_USE_NETWORK
		if (TC_FLAGS_DO_CRLF(tcc->tcc_flags) &&
		    TC_FLAGS_DO_TELNET(tcc->tcc_flags)) {
			m = "CRLF,Telnet";
		} else
		if (TC_FLAGS_DO_CRLF(tcc->tcc_flags)) {
			m = "CRLF";
		} else
		if (TC_FLAGS_DO_TELNET(tcc->tcc_flags)) {
			m = "Telnet";
		}
#else
			m = "CRLF";
#endif
	}
	fprintf(os, "%-13s", m);

	if (tcc->tcc_flags & TARGET_COMMS_FLAGS_UART) {
		char cfg[4], baud[16];
		uart_config_strings(tcc->tcc_uart_cfg, tcc->tcc_uart_baud,
		    cfg, baud);
		fprintf(os, "%-3s,%-9s", cfg, baud);
	} else {
		fprintf(os, "%-13s", "");
	}

	fprintf(os, "%s\n", target_comms_names[port].cn_desc);
}

typedef enum {
	TC_CONFIG_NEED_REBOOT,
	TC_CONFIG_CHANGED,
	TC_CONFIG_NO_CHANGE,
	TC_CONFIG_ERROR,
} tc_config_result_t;

static bool
tc_is_shared(target_comms_port_t port, int8_t usb_port, uint16_t tcp_port)
{
	struct target_comms_config tcc;
	target_comms_port_t p;

	for (p = TARGET_COMMS_PORT_TARGET_CONSOLE; p < TARGET_COMMS_PORT__COUNT;
	    p++) {
		if (p == port)
			continue;

		if (target_comms_get_config(p, &tcc) == false)
			continue;

		if (usb_port != TARGET_COMMS_USB_PORT_UNUSED &&
		    usb_port == tcc.tcc_usb_port) {
			return true;
		}

		if (tcp_port != TARGET_COMMS_TCP_PORT_UNUSED &&
		    tcp_port == tcc.tcc_tcp_port) {
			return true;
		}
	}

	return false;
}

static bool
tc_can_share(target_comms_port_t port, int8_t usb_port, uint16_t tcp_port)
{
	struct target_comms_config tcc;
	target_comms_port_t p;

	for (p = TARGET_COMMS_PORT_TARGET_CONSOLE; p < TARGET_COMMS_PORT__COUNT;
	    p++) {
		if (p == port)
			continue;

		if (target_comms_get_config(p, &tcc) == false)
			continue;

		if ((tcc.tcc_flags & TARGET_COMMS_FLAGS_CAN_SHARE) != 0)
			continue;

		if (usb_port != TARGET_COMMS_USB_PORT_UNUSED &&
		    usb_port == tcc.tcc_usb_port) {
			return false;
		}

		if (tcp_port != TARGET_COMMS_TCP_PORT_UNUSED &&
		    tcp_port == tcc.tcc_tcp_port) {
			return false;
		}
	}

	return true;
}

static tc_config_result_t
tc_cmd_do_usb_config(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	static char port_msg[64];
	int x;

	if (arg[0] == '-' && arg[1] == '\0') {
		x = TARGET_COMMS_USB_PORT_UNUSED;
	} else
	if (sscanf(arg, "%d", &x) != 1 ||
	    x < 0 || x >= PLATFORM_USB_CDC_INSTANCE_NB) {
		if (!web) {
			snprintf(port_msg, sizeof(port_msg),
			    "USB port number must be between 0 and %u or \"-\" "
			    "to disable.", PLATFORM_USB_CDC_INSTANCE_NB - 1);
			*msg = port_msg;
		} else {
			*msg = "Invalid USB port number.";
		}
		return TC_CONFIG_ERROR;
	}

	if ((int8_t)x == tcc->tcc_usb_port)
		return TC_CONFIG_NO_CHANGE;

	if (x != TARGET_COMMS_USB_PORT_UNUSED) {
		if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_CAN_SHARE) == 0 &&
		    tc_is_shared(port, x, TARGET_COMMS_TCP_PORT_UNUSED)) {
			if (!web) {
				*msg = "This service cannot share USB "
				    "ports with any other service.";
			} else {
				*msg = "USB port sharing conflict.";
			}
			return TC_CONFIG_ERROR;
		}

		if (!tc_can_share(port, x, TARGET_COMMS_TCP_PORT_UNUSED)) {
			if (!web) {
				*msg = "This USB port is being used by another "
				    "service which doesn't support sharing.";
			} else {
				*msg = "USB port sharing conflict.";
			}
			return TC_CONFIG_ERROR;
		}
	}

	tcc->tcc_usb_port = (int8_t)x;
	return TC_CONFIG_NEED_REBOOT;
}

static struct target_comms_source_state *
tc_find_comms_source_state(target_comms_port_t port)
{
	struct target_comms_state *tcs = &target_comms_state;

	for (unsigned int i = 0; i < tcs->tcs_num_sources; i++) {
		if (tcs->tcs_sources[i].css_source->cs_port == port)
			return &tcs->tcs_sources[i];
	}

	return NULL;
}

#ifdef CONFIG_USE_NETWORK
static tc_config_result_t
tc_cmd_do_tcp_config(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	unsigned int x;

	if (arg[0] == '-' && arg[1] == '\0') {
		x = TARGET_COMMS_TCP_PORT_UNUSED;
	} else
	if (sscanf(arg, "%u", &x) != 1 || x == 0u || x > 65535u) {
		if (!web) {
			*msg = "TCP port number must be between 1 and "
			    "65535, or \"-\" to disable.";
		} else {
			*msg = "TCP port number must be between 0 and 65535.";
		}
		return TC_CONFIG_ERROR;
	}

	if ((uint16_t)x == tcc->tcc_tcp_port)
		return TC_CONFIG_NO_CHANGE;

	if (x != TARGET_COMMS_TCP_PORT_UNUSED) {
		if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_CAN_SHARE) == 0 &&
		    tc_is_shared(port, TARGET_COMMS_USB_PORT_UNUSED, x)) {
			if (!web) {
				*msg = "This service cannot share TCP "
				    "ports with any other service.";
			} else {
				*msg = "TCP port sharing conflict.";
			}
			return TC_CONFIG_ERROR;
		}

		if (!tc_can_share(port, TARGET_COMMS_USB_PORT_UNUSED, x)) {
			if (!web) {
				*msg = "This TCP port is being used by another "
				    "service which doesn't support sharing.";
			} else {
				*msg = "TCP port sharing conflict.";
			}
			return TC_CONFIG_ERROR;
		}
	}

	tcc->tcc_tcp_port = (uint16_t)x;
	return TC_CONFIG_NEED_REBOOT;
}

static void
tc_update_telnet_nagle(target_comms_port_t port,
    target_comms_config_flags_t new_flags)
{
	struct target_comms_state *tcs = &target_comms_state;
	struct target_comms_source_state *css;
	struct target_comms_shared *sh;
	port_daemon_instance_t si;
	uint8_t pd_flags;

	if ((css = tc_find_comms_source_state(port)) == NULL)
		return;

	/* If this source is not using TCP, we're done. */
	if (css->css_settings[TCSS_SETTINGS_TCP].s_tcp_port ==
	    TARGET_COMMS_TCP_PORT_UNUSED) {
		return;
	}

	pd_flags = 0;
	if (new_flags & TARGET_COMMS_FLAGS_TELNET)
		pd_flags |= PORT_DAEMON_SOURCE_FLAG_DO_TELNET;
	if (new_flags & TARGET_COMMS_FLAGS_DO_NAGLE)
		pd_flags |= PORT_DAEMON_SOURCE_FLAG_DO_NAGLE;

	if ((si = css->css_port_daemon[TCSS_SETTINGS_TCP]) != NULL) {
		/*
		 * Unshared source on a TCP port. Update it.
		 */
		port_daemon_change_flags(si, pd_flags);
		return;
	}

	/*
	 * Scan all shares to see if the source is registered with one.
	 */
	TAILQ_FOREACH(sh, &tcs->tcs_shared, sh_qent) {
		unsigned int i;

		for (i = 0; i < sh->sh_num_sources; i++) {
			if (sh->sh_css[i] == css &&
			    sh->sh_settings.s_tcp_port !=
			    TARGET_COMMS_TCP_PORT_UNUSED &&
			    (si = sh->sh_port_daemon) != NULL) {
				/*
				 * Found one. Change its flags.
				 */
				port_daemon_change_flags(si,
				    pd_flags & sh->sh_settings.s_flags);
			}
		}
	}
}
#endif /* CONFIG_USE_NETWORK */

static tc_config_result_t
tc_cmd_do_mode_config(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	target_comms_config_flags_t new_flags = tcc->tcc_flags;
	tc_config_result_t rv;
	char *tok, *ctx, *str;

	if ((new_flags & TARGET_COMMS_FLAGS_MUTABLE_MODE) == 0) {
		if (!web)
			*msg = "The mode for this service cannot be changed.";
		return TC_CONFIG_NO_CHANGE;
	}

	str = zone_strdup(arg);

	for (tok = strtok_r(str, ",", &ctx); tok != NULL;
	    tok = strtok_r(NULL, ",", &ctx)) {
		bool clear = *tok == '-';
		uint8_t bit;
		char *p;

		p = tok;
		if (*tok == '-' || *tok == '+')
			p++;

		if (strcasecmp(p, "crlf") == 0)
			bit = TARGET_COMMS_FLAGS_CRLF;
		else
#ifdef CONFIG_USE_NETWORK
		if (strcasecmp(p, "telnet") == 0)
			bit = TARGET_COMMS_FLAGS_TELNET;
		else
#endif
		if (strcasecmp(p, "raw") == 0)
			bit = 0;
		else {
			zone_free(str);
			if (!web) {
				*msg = "Unrecognised mode string.";
				return TC_CONFIG_ERROR;
			}
			return TC_CONFIG_NO_CHANGE;
		}

		if (bit) {
			if (clear)
				new_flags &= ~bit;
			else
				new_flags |= bit;
		} else {
			if (clear) {
				new_flags |= TARGET_COMMS_FLAGS_CRLF |
				    TARGET_COMMS_FLAGS_TELNET;
			} else {
				new_flags &= ~(TARGET_COMMS_FLAGS_CRLF |
				    TARGET_COMMS_FLAGS_TELNET);
			}
		}
	}

	zone_free(str);

	if (new_flags == tcc->tcc_flags)
		return TC_CONFIG_NO_CHANGE;

	rv = TC_CONFIG_CHANGED;
	if ((tcc->tcc_flags ^ new_flags) & TARGET_COMMS_FLAGS_CRLF) {
		struct target_comms_source_state *css;
		/*
		 * CRLF configuration has changed. Update the source.
		 */
		css = tc_find_comms_source_state(port);
		if (css != NULL) {
			target_comms_config_flags_t f;

			f = css->css_source->cs_arg.ca_flags;
			f &= ~TARGET_COMMS_FLAGS_CRLF;
			f |= new_flags & TARGET_COMMS_FLAGS_CRLF;
			css->css_source->cs_arg.ca_flags = f;
		}
	}

#ifdef CONFIG_USE_NETWORK
	if ((tcc->tcc_flags ^ new_flags) & TARGET_COMMS_FLAGS_TELNET)
		tc_update_telnet_nagle(port, new_flags);
#endif

	tcc->tcc_flags = new_flags;
	return rv;
}

static tc_config_result_t
tc_cmd_do_uart_fmt(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	uint16_t cfg;

	(void) port;

	if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_UART) == 0) {
		if (!web)
			*msg = "This service does not have a UART.";
		return TC_CONFIG_NO_CHANGE;
	}

	if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_CFG) == 0) {
		if (!web) {
			*msg = "The UART character format for this service "
			    "cannot be changed.";
		}
		return TC_CONFIG_NO_CHANGE;
	}

	if (strlen(arg) != 3) {
		if (!web) {
			*msg = "The UART character format must be three "
			    "characters. For example 8N1.";
		}
		return TC_CONFIG_NO_CHANGE;
	}

	cfg = 0;
	switch (arg[0]) {
#ifndef UART_OPT_FIXED_DATABITS
	case '5':
		cfg = UART_CHARSIZE_5;
		break;
	case '6':
		cfg = UART_CHARSIZE_6;
		break;
	case '7':
		cfg = UART_CHARSIZE_7;
		break;
#endif /* UART_OPT_FIXED_DATABITS */
	case '8':
		cfg = UART_CHARSIZE_8;
		break;
	default:
		if (!web) {
#ifndef UART_OPT_FIXED_DATABITS
			*msg = "Supported character sizes are: 5, 6, 7, or 8.";
#else
			*msg = "Supported character sizes are: 8.";
#endif
		}
		return TC_CONFIG_NO_CHANGE;
	}

	switch (arg[1]) {
	case 'n':
	case 'N':
		cfg |= UART_PARITY_NONE;
		break;
	case 'o':
	case 'O':
		cfg |= UART_PARITY_ODD;
		break;
	case 'e':
	case 'E':
		cfg |= UART_PARITY_EVEN;
		break;
	default:
		if (!web)
			*msg = "Supported parity values are: N, O, or E.";
		return TC_CONFIG_NO_CHANGE;
	}

	switch (arg[2]) {
	case '1':
		cfg |= UART_STOPBITS_1;
		break;
#ifndef UART_OPT_FIXED_STOPBITS
	case '2':
		cfg |= UART_STOPBITS_2;
		break;
#endif
	default:
		if (!web)
			*msg = "Supported stop bit values are: 1, or 2.";
		return TC_CONFIG_NO_CHANGE;
	}

#define	CFG_MASK (UART_CHARSIZE_MASK | UART_PARITY_MASK | UART_STOPBITS_MASK)
	if (cfg != (tcc->tcc_uart_cfg & ~CFG_MASK)) {
		tcc->tcc_uart_cfg = cfg | (tcc->tcc_uart_cfg & ~CFG_MASK);
		/* XXX: Update active config */
		return TC_CONFIG_NEED_REBOOT;
	}

	return TC_CONFIG_NO_CHANGE;
}

static tc_config_result_t
tc_cmd_do_uart_baud(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	uint32_t baud;
	uint16_t cfg;

	(void) port;

	if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_UART) == 0) {
		if (!web)
			*msg = "This service does not have a UART.";
		return TC_CONFIG_NO_CHANGE;
	}

	if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_BAUD) == 0) {
		if (!web) {
			*msg = "The UART baud rate for this service "
			    "cannot be changed.";
		}
		return TC_CONFIG_NO_CHANGE;
	}

	if (sscanf(arg, "%" SCNu32, &baud) != 1 ||
	    baud < 150 ||
	    (tcc->tcc_uart_max_baud && baud > tcc->tcc_uart_max_baud)) {
		if (baud < 150 ||
		    (tcc->tcc_uart_max_baud && baud > tcc->tcc_uart_max_baud)) {
			static char baud_msg[64];
			snprintf(baud_msg, sizeof(baud_msg),
			    "Baud rate must be between 150 and %" PRIu32
			    " inclusive.", tcc->tcc_uart_max_baud);
			*msg = baud_msg;
		} else {
			*msg = "Invalid or out-of-range baud rate.";
		}
		if (web)
			return TC_CONFIG_NO_CHANGE;
		return TC_CONFIG_ERROR;
	}

	cfg = uart_baud_to_cfg((uint32_t)baud);

	if (cfg != UART_BAUDRATE_CUSTOM)
		baud = TARGET_COMMS_UART_BAUD_STANDARD;

	cfg |= tcc->tcc_uart_cfg & ~UART_BAUDRATE_MASK;

	if (cfg != tcc->tcc_uart_cfg || baud != tcc->tcc_uart_baud) {
		struct target_comms_source_state *css;

		tcc->tcc_uart_cfg = cfg;
		tcc->tcc_uart_baud = baud;

		css = tc_find_comms_source_state(port);
		if (css != NULL && css->css_source->cs_ops->op_ctrl != NULL) {
			struct target_comms_source *cs;
			netusb_mux_event_data_t ed;

			ed.ed_usb_cfg.mc_cfg = cfg;
			ed.ed_usb_cfg.mc_baud = baud;

			cs = css->css_source;
			if (cs->cs_ops->op_ctrl(&cs->cs_arg,
			    PORT_DAEMON_CTRL_UART_CONFIG, &ed) < 0) {
				*msg = "Maven cannot generate this "
				    "baud rate with enough accuracy.";
				if (web)
					return TC_CONFIG_ERROR;
				return TC_CONFIG_NO_CHANGE;
			}
		}

		return TC_CONFIG_CHANGED;
	}

	return TC_CONFIG_NO_CHANGE;
}

#if defined(CONFIG_USE_NETWORK) && (RELEASE_BUILD == 0)
static tc_config_result_t
tc_cmd_do_nagle_config(const char **msg, target_comms_port_t port,
    struct target_comms_config *tcc, const char *arg, bool web)
{
	bool on, old, valid;

	(void) port;

	if ((tcc->tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_NAGLE) == 0) {
		if (!web) {
			*msg = "The Nagle setting for this service "
			    "cannot be changed.";
		}
		return TC_CONFIG_NO_CHANGE;
	}

	valid = false;
	on = false;

	if (arg[1] == '\0') {
		switch (arg[0]) {
		case '1':
		case 'Y':
		case 'y':
			valid = true;
			on = true;
			break;
		case '0':
		case 'N':
		case 'n':
			valid = true;
			on = false;
			break;
		default:
			break;
		}
	} else {
		if (strcasecmp(arg, "on") == 0 ||
		    strcasecmp(arg, "yes") == 0 ||
		    strcasecmp(arg, "true") == 0) {
			valid = true;
			on = true;
		} else
		if (strcasecmp(arg, "off") == 0 ||
		    strcasecmp(arg, "no") == 0 ||
		    strcasecmp(arg, "false") == 0) {
			valid = true;
			on = false;
		}
	}

	if (!valid) {
		*msg = "Expected yes/no, or true/false, or 1/0.";
		return TC_CONFIG_ERROR;
	}

	old = !!(tcc->tcc_flags & TARGET_COMMS_FLAGS_DO_NAGLE);
	if (old != on) {
		if (on)
			tcc->tcc_flags |= TARGET_COMMS_FLAGS_DO_NAGLE;
		else
			tcc->tcc_flags &= ~TARGET_COMMS_FLAGS_DO_NAGLE;

		tc_update_telnet_nagle(port, tcc->tcc_flags);
		return TC_CONFIG_CHANGED;
	}

	return TC_CONFIG_NO_CHANGE;
}
#endif /* defined(CONFIG_USE_NETWORK) && (RELEASE_BUILD == 0) */

#if defined(CONFIG_USE_NETWORK) && defined(NETWORK_OPT_HTTPD)
static bool
tc_get_port_config(target_comms_port_t port, struct target_comms_config *tcc)
{
	struct target_comms_config_versioned tcc_v;

	if (!TARGET_COMMS_PORT_VALID(&tc_default_configs[port]))
		return false;

	if (cf_get_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(port),
	    sizeof(tcc_v), &tcc_v) == 0) {
		*tcc = tc_default_configs[port];
	} else {
		*tcc = tcc_v.tcc;
	}

	return true;
}

static void
target_comms_output_json(void)
{
	struct target_comms_config tcc;
	target_comms_port_t port;
	const char *nm, *json;
	uint16_t usb_vendor, usb_device;
	char ch, strbuff[10];
	stringio_t ss;

	if ((ss = stringio_new()) == NULL)
		return;

	stringio_printf(ss, JSON_OBJECT_START);

	for (port = TARGET_COMMS_PORT_TARGET_CONSOLE;
	    port < TARGET_COMMS_PORT__COUNT; port++) {
#ifdef PLATFORM_WILC_UART
		if (port == TARGET_COMMS_PORT_WIFI_UART)
			continue;
#endif
#ifdef CONFIG_TARGET_AVR
		if (port == TARGET_COMMS_PORT_AVR_PROGRAMMER)
			continue;
#endif

		if (tc_get_port_config(port, &tcc) == false)
			continue;

#if (CONFIG_GDB_SERVERS == 1)
		if (port == TARGET_COMMS_PORT_GDB_SERVER0)
			nm = "gdb0";
		else
		if (port == TARGET_COMMS_PORT_SEMIHOST0)
			nm = "semihost0";
		else
#endif
			nm = target_comms_names[port].cn_cmd;

		stringio_printf(ss, JSON_OBJECT_NAMED_V(nm));

		if (tcc.tcc_usb_port == TARGET_COMMS_USB_PORT_UNUSED)
			ch = '-';
		else
			ch = '0' + (char)tcc.tcc_usb_port;
		stringio_printf(ss, JSON_STRING_CH("usb", ch));
		stringio_printf(ss, JSON_SEPARATOR);

		stringio_printf(ss, JSON_NUMBER("tcp", tcc.tcc_tcp_port));

		if (tcc.tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_MODE) {
			int n;

			if (TC_FLAGS_IS_RAW(tcc.tcc_flags)) {
				n = 0;
			} else
			if (TC_FLAGS_DO_CRLF(tcc.tcc_flags) &&
			    TC_FLAGS_DO_TELNET(tcc.tcc_flags)) {
				n = 3;
			} else
			if (TC_FLAGS_DO_CRLF(tcc.tcc_flags)) {
				n = 1;
			} else {
				n = 2;
			}

			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_NUMBER("mode", n));
		}

		if (tcc.tcc_flags & TARGET_COMMS_FLAGS_UART) {
			char cfg[4];

			uart_config_strings(tcc.tcc_uart_cfg,
			    tcc.tcc_uart_baud, cfg, strbuff);

			stringio_printf(ss, JSON_SEPARATOR);
			stringio_printf(ss, JSON_STRING("baud", strbuff));

			if (tcc.tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_CFG) {
				stringio_printf(ss, JSON_SEPARATOR);
				stringio_printf(ss, JSON_STRING_CH("bits",
				    cfg[0]));

				stringio_printf(ss, JSON_SEPARATOR);
				stringio_printf(ss, JSON_STRING_CH("parity",
				    cfg[1]));

				stringio_printf(ss, JSON_SEPARATOR);
				stringio_printf(ss, JSON_STRING_CH("stop",
				    cfg[2]));
			}
		}

		stringio_printf(ss, JSON_OBJECT_END);
		stringio_printf(ss, JSON_SEPARATOR);
	}

	if (usb_get_config(&usb_vendor, &usb_device) ==
	    USB_PERSONALITY_DISABLED) {
		usb_vendor = usb_device = 0;
	}

	stringio_printf(ss, JSON_OBJECT_NAMED("usb"));
	snprintf(strbuff, sizeof(strbuff), "0x%04" PRIx16, usb_vendor);
	stringio_printf(ss, JSON_STRING("vendor", strbuff));
	stringio_printf(ss, JSON_SEPARATOR);
	snprintf(strbuff, sizeof(strbuff), "0x%04" PRIx16, usb_device);
	stringio_printf(ss, JSON_STRING("device", strbuff));
	stringio_printf(ss, JSON_OBJECT_END);

	stringio_printf(ss, JSON_OBJECT_END);

	if ((json = stringio_get_buff(ss)) != NULL) {
		fsdata_create("/targetcomms.json", json, strlen(json), true);
		network_httpd_update_ping(NETWORK_HTTPD_PING_COMMS);
	}

	stringio_done(ss);
}

static int
tc_parse_usb_id(const char *str, uint16_t *pid)
{
	unsigned long v;
	char *p;

	v = strtoul(str, &p, 0);
	if (p == NULL || *p != '\0')
		return -1;

	if (v <= 0xffffu) {
		*pid = (uint16_t)v;
		return 0;
	}

	return -1;
}

static const char *
target_comms_post_furl(void *cookie, int nkeys,
    const struct network_http_furl_key_value *kv)
{
	struct target_comms_config *tcc;
	uint16_t usb_vendor, usb_device;
	uint16_t new_vendor, new_device;
	target_comms_port_t port;
	tc_config_result_t res;
	bool changed;
	const char *rv;

	(void) cookie;

	if ((tcc = zone_calloc(1, sizeof(tc_default_configs))) == NULL)
		return "/error.txt";

	for (port = TARGET_COMMS_PORT_TARGET_CONSOLE;
	    port < TARGET_COMMS_PORT__COUNT; port++) {
		tc_get_port_config(port, &tcc[port]);
	}

	if (usb_get_config(&usb_vendor, &usb_device) ==
	    USB_PERSONALITY_DISABLED) {
		usb_vendor = usb_device = 0;
	}

	new_vendor = usb_vendor;
	new_device = usb_device;
	res = TC_CONFIG_NO_CHANGE;

	for (changed = false, rv = NULL; rv == NULL && nkeys; nkeys--, kv++) {
		struct target_comms_config *t;
		const char *k;

		if (strncmp(kv->kv_key, "tc-serial-", 10) == 0) {
			k = &kv->kv_key[10];
			port = TARGET_COMMS_PORT_TARGET_CONSOLE;
		} else
#ifdef CONFIG_TARGET_CORTEXM
		if (strncmp(kv->kv_key, "tc-swo-", 7) == 0) {
			k = &kv->kv_key[7];
			port = TARGET_COMMS_PORT_SWO;
		} else
		if (strncmp(kv->kv_key, "tc-semi-", 8) == 0) {
			k = &kv->kv_key[8];
			port = TARGET_COMMS_PORT_SEMIHOST0;
		} else
		if (strncmp(kv->kv_key, "tc-gdb-", 7) == 0) {
			k = &kv->kv_key[7];
			port = TARGET_COMMS_PORT_GDB_SERVER0;
		} else
#endif
		if (strcmp(kv->kv_key, "tc-usb-vendor") == 0) {
			if (tc_parse_usb_id(kv->kv_value, &new_vendor) < 0)
				rv = "Invalid USB vendor ID.";
			continue;
		} else
		if (strcmp(kv->kv_key, "tc-usb-device") == 0) {
			if (tc_parse_usb_id(kv->kv_value, &new_device) < 0)
				rv = "Invalid USB product ID.";
			continue;
		} else {
			continue;
		}

		t = &tcc[port];
		if (!TARGET_COMMS_PORT_VALID(t))
			continue;

		if (strcmp(k, "usb") == 0) {
			res = tc_cmd_do_usb_config(&rv, port, t, kv->kv_value,
			    true);
		} else
		if (strcmp(k, "tcp") == 0) {
			res = tc_cmd_do_tcp_config(&rv, port, t, kv->kv_value,
			    true);
		} else
		if (strcmp(k, "mode") == 0) {
			static const char *mstr[4] = {
				"raw", "crlf", "telnet", "telnet,crlf"
			};
			unsigned int mode = (unsigned int)(kv->kv_value[0]-'0');

			if ((t->tcc_flags & TARGET_COMMS_FLAGS_MUTABLE_MODE) &&
			    kv->kv_value[1] == '\0' && mode < 4) {
				t->tcc_flags &= ~(TARGET_COMMS_FLAGS_CRLF |
				    TARGET_COMMS_FLAGS_TELNET);
				res = tc_cmd_do_mode_config(&rv, port, t,
				    mstr[mode], true);
			} else {
				res = TC_CONFIG_NO_CHANGE;
			}
		} else
		if (strcmp(k, "baud") == 0) {
			res = tc_cmd_do_uart_baud(&rv, port, t, kv->kv_value,
			    true);
		} else
		if (strcmp(k, "data-bits") == 0 && strlen(kv->kv_value) == 1) {
			char cfg[4];
			uart_config_strings(t->tcc_uart_cfg, 0, cfg, NULL);
			cfg[0] = kv->kv_value[0];
			res = tc_cmd_do_uart_fmt(&rv, port, t, cfg, true);
		} else
		if (strcmp(k, "parity") == 0 && strlen(kv->kv_value) == 1) {
			char cfg[4];
			uart_config_strings(t->tcc_uart_cfg, 0, cfg, NULL);
			cfg[1] = kv->kv_value[0];
			res = tc_cmd_do_uart_fmt(&rv, port, t, cfg, true);
		} else
		if (strcmp(k, "stop-bits") == 0 && strlen(kv->kv_value) == 1) {
			char cfg[4];
			uart_config_strings(t->tcc_uart_cfg, 0, cfg, NULL);
			cfg[2] = kv->kv_value[0];
			res = tc_cmd_do_uart_fmt(&rv, port, t, cfg, true);
		} else {
			rv = "Invalid UART data format.";
			res = TC_CONFIG_ERROR;
		}

#if (RELEASE_BUILD == 0)
		if (res == TC_CONFIG_ERROR && rv != NULL) {
			debug_print("TC port %u %s:%s %s\n", port, kv->kv_key,
			    kv->kv_value, rv);
		}
#endif
		if (res == TC_CONFIG_NEED_REBOOT || res == TC_CONFIG_CHANGED)
			changed = true;
		else
		if (res == TC_CONFIG_NO_CHANGE)
			rv = NULL;
	}

	if (rv == NULL) {
		bool persist = false;

		if (new_vendor == 0xffffu ||
		    new_device == 0x0000u || new_device == 0xffffu) {
			new_vendor = 0;
		}
		if (new_vendor == 0)
			new_device = 0;

		if (new_vendor != usb_vendor || new_device != usb_device) {
			cf_set_uint16(CF_KEY_UINT16_USB_VID, new_vendor);
			cf_set_uint16(CF_KEY_UINT16_USB_PID, new_device);
			persist = true;

			if (new_vendor != usb_vendor &&
			    (new_vendor == 0 || usb_vendor == 0)) {
				res = TC_CONFIG_NEED_REBOOT;
			}
		}

		if (changed) {
			for (port = TARGET_COMMS_PORT_TARGET_CONSOLE;
			    port < TARGET_COMMS_PORT__COUNT; port++) {
				if (target_comms_set_config(port,
				    &tcc[port], false)) {
					persist = true;
				}
			}
		}

		if (persist) {
			configdb_persist();
			target_comms_output_json();
		}

		if (res == TC_CONFIG_NEED_REBOOT) {
			rv = "Maven must be rebooted for this change to "
			    "take effect.";
		}
	}

	zone_free(tcc);

	if (rv == NULL)
		return "/saved.txt";

	return fsdata_create_one_time("txt", rv, strlen(rv), false);
}
#endif /* defined(CONFIG_USE_NETWORK) && defined(NETWORK_OPT_HTTPD) */

static const char tc_cmd_usage[] =
"usage:\n"
"tc\n"
"\tShow current USB"
#ifdef CONFIG_USE_NETWORK
"/TCP"
#endif
" port assignments and settings for each service.\n"
"tc <service> \"usb\" <usb-port>\n"
"\tAssign the numbered USB CDC port instance <usb-port> to the specified\n"
"\tservice. Valid values are 0 - %u inclusive, or '-' to deassign USB.\n"
"\tIt is possible to share certain services on a single USB instance.\n"
"\tUpon connection, you will be asked which service you wish to connect\n"
"\tto. Note that the "
#ifdef CONFIG_TARGET_CORTEXM
"GDB server"
#else
"AVR programmer"
#endif
  " service cannot be shared in this way.\n"
"\tChanges to this setting require a reboot to take effect.\n"
#ifdef CONFIG_USE_NETWORK
"tc <service> \"tcp\" <tcp-port>\n"
"\tAssign TCP port <tcp-port> to the specified service. Valid ports\n"
"\tare numbered between 1 and 65535, and must not match an existing\n"
"\tport number, including port 23 - reserved for the shell server.\n"
"\tSharing TCP ports is possible in the same way as described for\n"
"\tUSB CDC instances.\n"
"\tChanges to this setting require a reboot to take effect.\n"
#if (RELEASE_BUILD == 0)
"tc <service> \"nagle\" <\"yes\" | \"no\">\n"
"\tEnable or disable the TCP Nagle algorithm on the specified port.\n"
#endif
#endif /* CONFIG_USE_NETWORK */
"tc <service> \"mode\" <mode-string>\n"
"\tConfigure the specified service's data mode. Supported modes are:\n"
"\t - \"crlf\" Convert an LF character into a CRLF pair.\n"
#ifdef CONFIG_USE_NETWORK
"\t - \"telnet\" Perform Telnet negotiation when a connection is\n"
"\t   established. This will instruct the host to switch from line-\n"
"\t   mode to character-mode.\n"
#endif
"\t - \"raw\" Disable both CRLF and Telnet. Data is passed as-is to\n"
"\t   the host.\n"
"\tMode strings can be prefixed with an optional \"+\" or \"-\" character\n"
"\tto imply addition or removal of the mode from the service.\n"
"\tNote that services sharing a USB "
#ifdef CONFIG_USE_NETWORK
"and/or TCP "
#endif
"port will use the lowest\n"
"\tcommon mode. So if one of the services is in \"raw\" mode, all shared\n"
"\tservices on the same port will be in \"raw\" mode.\n"
"\tChanges to these settings will not affect an established connection.\n"
"tc <service> \"fmt\" <uart-character-format>\n"
"\tFor services provided by a UART, this command configures the UART\n"
"\tcharacter format (number of data bits, parity, and stop bits). For\n"
"\texample, \"8N1\" specifies 8 bits, no parity, 1 stop bit. \"5E2\" is\n"
"\t5 bits, even parity and 2 stop bits.\n"
#ifdef WAVR_BOARD
"\tUnfortunately, this version of " PRODUCT_NAME " only supports 8 data\n"
"\tbits and 1 stop bit, so those fields are fixed.\n"
#endif
#ifdef CONFIG_TARGET_CORTEXM
"\tNote that the character format for the \"swo\" service is fixed at\n"
"\t\"8N1\" as per the standard.\n"
#endif
"tc <service> \"baud\" <uart-baudrate>\n"
"\tConfigure UART baud rate. Valid rates are > 0 and <= a probe-specific\n"
"\tmaximum value (usually around 2500000). If " PRODUCT_NAME "\n"
"\tcannot get to within +/- 1.5%% of the required rate then an error\n"
"\twill be displayed. Rates above 2 Mbaud are at higher risk of\n"
"\treceiver overrun errors. See the \"uart\" command for the actual baud\n"
"\trate used, together with other useful statistics.\n";

static void
tc_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	struct target_comms_config tcc;
	target_comms_port_t port;
	tc_config_result_t rv;
	const char *msg;
	bool usb_used;

	if (argc) {
		if (strcasecmp(argv[0], "help") == 0) {
 usage:
			fprintf(os, tc_cmd_usage,
			    (unsigned int)(PLATFORM_USB_CDC_INSTANCE_NB - 1));
			return;
		}

		for (port = TARGET_COMMS_PORT_TARGET_CONSOLE;
		    port < TARGET_COMMS_PORT__COUNT; port++) {
			if (target_comms_get_config(port, &tcc) == false)
				continue;
			if (strcasecmp(argv[0],
			    target_comms_names[port].cn_cmd) == 0) {
				break;
			}
		}

		if (port == TARGET_COMMS_PORT__COUNT) {
			fprintf(os, "'%s' is not a valid service name\n",
			    argv[0]);
			return;
		}

		if (argc != 3)
			goto usage;

		msg = NULL;

		if (strcasecmp(argv[1], "usb") == 0) {
			rv = tc_cmd_do_usb_config(&msg, port, &tcc, argv[2],
			    false);
		} else
#ifdef CONFIG_USE_NETWORK
		if (strcasecmp(argv[1], "tcp") == 0) {
			rv = tc_cmd_do_tcp_config(&msg, port, &tcc, argv[2],
			    false);
		} else
#endif
		if (strcasecmp(argv[1], "mode") == 0) {
			rv = tc_cmd_do_mode_config(&msg, port, &tcc, argv[2],
			    false);
		} else
		if (strcasecmp(argv[1], "fmt") == 0) {
			rv = tc_cmd_do_uart_fmt(&msg, port, &tcc, argv[2],
			    false);
		} else
		if (strcasecmp(argv[1], "baud") == 0) {
			rv = tc_cmd_do_uart_baud(&msg, port, &tcc, argv[2],
			    false);
#if defined(CONFIG_USE_NETWORK) && (RELEASE_BUILD == 0)
		} else
		if (strcasecmp(argv[1], "nagle") == 0) {
			rv = tc_cmd_do_nagle_config(&msg, port, &tcc, argv[2],
			    false);
#endif
		} else {
			msg = "Invalid service name.";
			rv = TC_CONFIG_ERROR;
		}

		if (msg != NULL)
			fprintf(os, "%s\n", msg);

		if (rv == TC_CONFIG_ERROR)
			goto usage;

		if (rv == TC_CONFIG_CHANGED || rv == TC_CONFIG_NEED_REBOOT) {
			target_comms_set_config(port, &tcc, true);
			tc_show_config_entry(os, false, port, &tcc);
			if (rv == TC_CONFIG_NEED_REBOOT) {
				fprintf(os, "Change will take effect after a "
				    "reboot\n");
				target_comms_state.tcs_changed = true;
			}
#ifdef NETWORK_OPT_HTTPD
			target_comms_output_json();
#endif
		} else {
			fprintf(os, "Configuration unchanged.\n");
		}
		return;
	}

	usb_used = false;
	for (port = TARGET_COMMS_PORT_TARGET_CONSOLE;
	    port < TARGET_COMMS_PORT__COUNT; port++) {
		if (target_comms_get_config(port, &tcc)) {
			tc_show_config_entry(os,
			    port == TARGET_COMMS_PORT_TARGET_CONSOLE, port,
			    &tcc);
			if (tcc.tcc_usb_port != TARGET_COMMS_USB_PORT_UNUSED)
				usb_used = true;
		}
	}

	if (target_comms_state.tcs_changed)
		fprintf(os, "These settings will take effect after a reboot\n");

	if (usb_used && usb_cdc_enabled() == false) {
		fprintf(os, "Note: USB is disabled. Please refer to the 'usb' "
		    "command.\n");
	}
}

/*
 * Helper functions to copy data from one ring-buffer to another
 * according to port_daemon_settings.s_flags settings.
 */
static void
target_comms_raw(ringbuff_t from_target, ringbuff_t to_host)
{
	ringbuff_len_t srclen, dstspc;
	const uint8_t *src;
	uint8_t *dst;

	/*
	 * For ports in raw mode, blast as fast as we can using the
	 * 'contiguous' buffer feature from the ringbuff API, and
	 * memcpy.
	 */
	while ((srclen = ringbuff_consume_contig_size(from_target)) != 0 &&
	    (dstspc = ringbuff_produce_contig_size(to_host)) != 0) {
		/* Keep a lid on the number of bytes we transfer. */
		if (srclen > dstspc)
			srclen = dstspc;

		/* Grab source & destination ring buffer pointers. */
		src = ringbuff_consume_current_buff_pointer(from_target);
		dst = ringbuff_produce_current_buff_pointer(to_host);

		/* Move the data over. */
		memcpy(dst, src, srclen);

		/* Mark it as consumed. */
		ringbuff_consume_contig(from_target, srclen);
		ringbuff_consume_done(from_target);

		/* Update the destination ringbuff. */
		ringbuff_produce_contig(to_host, srclen);
		ringbuff_produce_done(to_host);
	}
}

static void
target_comms_cooked(ringbuff_t from_target, ringbuff_t to_host, bool crlf)
{
	ringbuff_len_t srclen, dstspc, dstmin;
	uint8_t ch;
	bool work;

	/*
	 * We need at least two characters available in case we need to
	 * perform CRLF translation.
	 */
	dstmin = crlf ? 2 : 1;

	/* How many Rx bytes from the target UART are waiting? */
	if ((srclen = ringbuff_get_count(from_target)) == 0)
		return;

	/* How much space is available in the destination? */
	dstspc = ringbuff_get_space(to_host);
	if (dstspc < dstmin)
		return;

	/* Only process what the destination can handle */
	if (srclen > dstspc)
		srclen = dstspc;

	/*
	 */
	work = false;
	while (srclen-- && dstspc >= dstmin) {
		bool printable;
		work = true;
		ch = ringbuff_consume(from_target);

		/* Determine if each character is printable. */
		switch (ch) {
		case '\n':	/* New line */
			if (crlf) {
				ringbuff_produce(to_host, '\r');
				dstspc--;
			}
			/*FALLTHROUGH*/

		case 0x04u:	/* EOT */
		case 0x07u:	/* BEL */
		case '\b':	/* Back space */
		case '\t':	/* Tab */
		case '\r':	/* Carraige return */
		case 0x1bu:	/* ESC */
			printable = 1;
			break;

		default:
			printable = (ch >= 0x20u && ch <= 0x7f);
			break;
		}

		if (printable) {
			ringbuff_produce(to_host, ch);
			dstspc--;
		}
	}

	if (work) {
		ringbuff_consume_done(from_target);
		ringbuff_produce_done(to_host);
	}
}

void
target_comms_generic_worker(ringbuff_t target, ringbuff_t host, uint8_t flags)
{

	/*
	 * Raw or cooked - convenience function for consumers of the Target
	 * Comms API.
	 */
	if (TC_FLAGS_IS_RAW(flags))
		target_comms_raw(target, host);
	else
		target_comms_cooked(target, host, TC_FLAGS_DO_CRLF(flags));
}

/*
 * We start non-shared comms sources here.
 */
static void
tc_start_source(struct target_comms_source_state *css, unsigned int idx)
{
	struct target_comms_source *cs = css->css_source;
	port_daemon_instance_t si;
	port_daemon_settings_t s;

	if (idx == TCSS_SETTINGS_CNT)
		s = css->css_settings;
	else
		s = &css->css_settings[idx];

	/* Couldn't be easier. */
	si = port_daemon_register_source(s, cs->cs_ops, &cs->cs_arg, NULL, 0);
	assert(si != NULL);

	if (idx == TCSS_SETTINGS_CNT) {
		css->css_port_daemon[TCSS_SETTINGS_USB] = si;
		css->css_port_daemon[TCSS_SETTINGS_TCP] = si;
	} else {
		css->css_port_daemon[idx] = si;
	}
}

/*
 * Shared comms sources take a bit more effort.
 * The list of shared sources is supplied in 'css' as an array of pointers.
 * The number of elements is supplied in 'cnt'.
 */
static void
tc_start_shared(struct target_comms_state *tcs, unsigned int idx,
    unsigned int cnt, struct target_comms_source_state **css)
{
	static const struct port_daemon_ops tc_chooser_ops = {
		.op_connect = tc_op_connect,
		.op_disconnect = tc_op_disconnect,
		.op_worker = tc_op_worker,
		.op_ctrl = tc_op_ctrl,
		.op_hangup = tc_op_hangup
	};
	struct target_comms_shared *sh;
	port_daemon_settings_t s;
	uint16_t to_rb_max, from_rb_max;
	uint8_t flags;

	/* Allocate a structure to track shared sources. */
	sh = zone_malloc(sizeof(*sh));
	assert(sh != NULL);

	sh->sh_num_sources = cnt;
	sh->sh_ops = NULL;
	sh->sh_ops_arg = NULL;

	/*
	 * The port settings are a mix of those from the shared port.
	 * In the case of s_flags, we perform a logical 'AND' of all
	 * the sources. For ring buffer sizes, we pick the max value
	 * of them all.
	 */
	flags = css[0]->css_settings[idx].s_flags;	/* XXX: All bits set? */
	to_rb_max = from_rb_max = 0;

	for (unsigned int i = 0; i < cnt; i++) {
		/* Grab the source pointer. */
		sh->sh_css[i] = css[i];

		/* Point to the required settings structure. */
		s = &sh->sh_css[i]->css_settings[idx];

		/*
		 * Incorporate into the shared s_flags and ring buffer sizes.
		 */
		flags &= s->s_flags;
		if (s->s_rb_size_to_host > to_rb_max)
			to_rb_max = s->s_rb_size_to_host;
		if (s->s_rb_size_from_host > from_rb_max)
			from_rb_max = s->s_rb_size_from_host;

		/* Add to the 'chooser' options array. */
		sh->sh_options[i] =
		    target_comms_names[css[i]->css_source->cs_port].cn_menu;
	}

	/* Finalise the settings. */
	sh->sh_settings.s_flags = flags;
	sh->sh_settings.s_usb_port = css[0]->css_settings[idx].s_usb_port;
	sh->sh_settings.s_tcp_port = css[0]->css_settings[idx].s_tcp_port;
	sh->sh_settings.s_rb_size_to_host = to_rb_max;
	sh->sh_settings.s_rb_size_from_host = from_rb_max;

	/* Add to list of shared ports. */
	TAILQ_INSERT_TAIL(&tcs->tcs_shared, sh, sh_qent);

	/* Pass ownership to the Port Daemon. */
	sh->sh_port_daemon = port_daemon_register_source(&sh->sh_settings,
	    &tc_chooser_ops, sh, sh->sh_options, cnt);
	assert(sh->sh_port_daemon != NULL);
}

/*
 * This function returns 'true' if the supplied USB and/or TCP ports
 * are used by any registered comms source, except for the source at
 * tcs_sources[idx].
 */
static bool
target_comms_shares_ports(struct target_comms_state *tcs,
    unsigned int idx, int8_t usb_port, uint16_t tcp_port)
{
	struct port_daemon_settings *s;

	/* Compare against all registered sources. */
	for (unsigned int i = 0; i < tcs->tcs_num_sources; i++) {
		/* Except for the source in 'idx'. */
		if (i == idx)
			continue;

		/* Check all settings for this source. */
		for (unsigned int j = 0; j < TCSS_SETTINGS_CNT; j++) {
			s = &tcs->tcs_sources[i].css_settings[j];

			if (usb_port != TARGET_COMMS_USB_PORT_UNUSED &&
			    usb_port == s->s_usb_port) {
				/* Shares a USB instance. */
				return true;
			}

			if (tcp_port != TARGET_COMMS_TCP_PORT_UNUSED &&
			    tcp_port == s->s_tcp_port) {
				/* Shares a network port. */
				return true;
			}
		}
	}

	/* No other sources are using the port(s). */
	return false;
}

/*
 * qsort(3) callback, to compare the USB ports of two comms sources.
 */
static int
tc_usb_sort(const void *v1, const void *v2)
{
	const struct target_comms_source_state * const *css1, * const *css2;

	css1 = v1;
	css2 = v2;

	assert((*css1)->css_settings[TCSS_SETTINGS_USB].s_usb_port !=
	    TARGET_COMMS_USB_PORT_UNUSED);
	assert((*css2)->css_settings[TCSS_SETTINGS_USB].s_usb_port !=
	    TARGET_COMMS_USB_PORT_UNUSED);

	if ((*css1)->css_settings[TCSS_SETTINGS_USB].s_usb_port ==
	    (*css2)->css_settings[TCSS_SETTINGS_USB].s_usb_port) {
		return 0;
	} else
	if ((*css1)->css_settings[TCSS_SETTINGS_USB].s_usb_port <
	    (*css2)->css_settings[TCSS_SETTINGS_USB].s_usb_port) {
		return -1;
	}

	return 1;
}

/*
 * qsort(3) callback, to compare the TCP ports of two comms sources.
 */
static int
tc_tcp_sort(const void *v1, const void *v2)
{
	const struct target_comms_source_state * const *css1, * const *css2;

	css1 = v1;
	css2 = v2;

	assert((*css1)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port !=
	    TARGET_COMMS_TCP_PORT_UNUSED);
	assert((*css2)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port !=
	    TARGET_COMMS_TCP_PORT_UNUSED);

	if ((*css1)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port ==
	    (*css2)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port) {
		return 0;
	} else
	if ((*css1)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port <
	    (*css2)->css_settings[TCSS_SETTINGS_TCP].s_tcp_port) {
		return -1;
	}

	return 1;
}

/*
 * The grand sorting ceremony.
 *
 * To take full advantage of the port daemon's "Chooser" feature, we
 * need to sort through the list of sources to see which ones are
 * using matching ports. Ideally, sources will share both USB and
 * TCP ports. However, this is not always possible, but we can still
 * split them and build sets of matching USB and TCP sources.
 * This function performs that task. Buckle up - it's quite the ride.
 */
void
target_comms_start(void)
{
	/* List of sources where both USB/TCP ports are used and unshared. */
	struct target_comms_source_state *src_both[TARGET_COMMS_PORT__COUNT];
	/* List of sources where just the USB port is used, and maybe shared. */
	struct target_comms_source_state *src_usb[TARGET_COMMS_PORT__COUNT * 2];
	/* List of sources where just the TCP port is used, and maybe shared. */
	struct target_comms_source_state *src_tcp[TARGET_COMMS_PORT__COUNT * 2];
	struct target_comms_source_state *css;
	struct target_comms_state *tcs;
	struct target_comms_source *cs;
	struct port_daemon_settings *s, *susb, *stcp;
	unsigned int cnt_both, cnt_usb, cnt_tcp, i, j;

	tcs = &target_comms_state;

	/* Empty the arrays. */
	memset(src_both, 0, sizeof(src_both));
	memset(src_usb, 0, sizeof(src_usb));
	memset(src_tcp, 0, sizeof(src_tcp));
	cnt_both = cnt_usb = cnt_tcp = 0;

	/*
	 * The first loop through all registered sources performs the following:
	 *
	 *  1. Identify any sources which are unused (no resources were assigned
	 *     to their USB/TCP ports). These can be discarded.
	 *
	 *  2. Identify sources where both USB and TCP ports are valid:
	 *     - Where the USB/TCP port pair does not clash with any other
	 *       source, these can be started immediately and removed from
	 *       further consideration.
	 *     - In case of a clash, determine if one of USB/TCP is unique and
	 *       so can be started immediately, leaving just the other port
	 *       for further consideration.
	 *     - Otherwise, add the source to the 'src_both[]' array.
	 *
	 *  3. Identify sources with one active port. Because of the check we
	 *     did in step 1, we know these are shared by some other source.
	 *     Note that step 2 may have created one of these.
	 *     Add the sources to the appropriate list: src_usb[] or src_tcp[].
	 */
	for (i = 0; i < tcs->tcs_num_sources; i++) {
		s = tcs->tcs_sources[i].css_settings;
		cs = tcs->tcs_sources[i].css_source;
		assert(cs != NULL);
		(void)cs;

#ifndef CONFIG_USE_NETWORK
		assert(s->s_tcp_port == TARGET_COMMS_TCP_PORT_UNUSED);
#endif

		/*
		 * Filter out a source for which no resource is assigned.
		 */
		if (s->s_usb_port == TARGET_COMMS_USB_PORT_UNUSED &&
		    s->s_tcp_port == TARGET_COMMS_TCP_PORT_UNUSED) {
			DBPRINTF("Source %p (%s): No active ports.\n",
			    (void *)cs,
			    target_comms_names[cs->cs_port].cn_desc);
			continue;
		}

		/*
		 * Compare this source with other sources to check if ports
		 * are shared.
		 */
		if (!target_comms_shares_ports(tcs, i, s->s_usb_port,
		    s->s_tcp_port)) {
			/*
			 * This source shares no ports with any other
			 * source, so it can be started directly.
			 */
			DBPRINTF("Source %p (%s): Completely unshared.\n",
			    (void *)cs,
			    target_comms_names[cs->cs_port].cn_desc);
			tc_start_source(&tcs->tcs_sources[i],
			    TCSS_SETTINGS_CNT);
			continue;
		}

		/*
		 * Perhaps the source shares only one of the USB/TCP port pair.
		 * If so, the unshared port can be started immediately.
		 */
		if (s->s_usb_port != TARGET_COMMS_USB_PORT_UNUSED &&
		    s->s_tcp_port != TARGET_COMMS_TCP_PORT_UNUSED) {
			if (!target_comms_shares_ports(tcs, i,
			    s->s_usb_port, TARGET_COMMS_TCP_PORT_UNUSED)) {
				/* USB on its own will work. */
				DBPRINTF("Source %p (%s): Split and start "
				    "USB\n", (void *)cs,
				    target_comms_names[cs->cs_port].cn_desc);
				s[TCSS_SETTINGS_USB].s_tcp_port =
				    TARGET_COMMS_TCP_PORT_UNUSED;
				s[TCSS_SETTINGS_TCP].s_usb_port =
				    TARGET_COMMS_USB_PORT_UNUSED;
				tc_start_source(&tcs->tcs_sources[i],
				    TCSS_SETTINGS_USB);

				/* Only TCP remains. */
				DBPRINTF("Source %p (%s): TCP remains.\n",
				    (void *)cs,
				    target_comms_names[cs->cs_port].cn_desc);
				assert(cnt_tcp < (TARGET_COMMS_PORT__COUNT*2));
				src_tcp[cnt_tcp++] = &tcs->tcs_sources[i];
			} else
			if (!target_comms_shares_ports(tcs, i,
			    TARGET_COMMS_USB_PORT_UNUSED, s->s_tcp_port)) {
				/* TCP on its own will work. */
				DBPRINTF("Source %p (%s): Split and start "
				    "TCP\n", (void *)cs,
				    target_comms_names[cs->cs_port].cn_desc);
				s[TCSS_SETTINGS_USB].s_tcp_port =
				    TARGET_COMMS_TCP_PORT_UNUSED;
				s[TCSS_SETTINGS_TCP].s_usb_port =
				    TARGET_COMMS_USB_PORT_UNUSED;
				tc_start_source(&tcs->tcs_sources[i],
				    TCSS_SETTINGS_TCP);

				/* Only USB remains. */
				DBPRINTF("Source %p (%s): USB remains.\n",
				    (void *)cs,
				    target_comms_names[cs->cs_port].cn_desc);
				assert(cnt_usb < (TARGET_COMMS_PORT__COUNT*2));
				src_usb[cnt_usb++] = &tcs->tcs_sources[i];
			} else {
				/* Can't split yet, both active. */
				DBPRINTF("Source %p (%s): Both active.\n",
				    (void *)cs,
				    target_comms_names[cs->cs_port].cn_desc);
				assert(cnt_both < TARGET_COMMS_PORT__COUNT);
				src_both[cnt_both++] = &tcs->tcs_sources[i];
			}
		} else
		if (s[TCSS_SETTINGS_USB].s_usb_port !=
		    TARGET_COMMS_USB_PORT_UNUSED) {
			/* Only USB port is active. */
			DBPRINTF("Source %p (%s): USB active only.\n",
			    (void *)cs,
			    target_comms_names[cs->cs_port].cn_desc);
			assert(cnt_usb < (TARGET_COMMS_PORT__COUNT * 2));
			src_usb[cnt_usb++] = &tcs->tcs_sources[i];
		} else {
			/* Only TCP port is active. */
			assert(s[TCSS_SETTINGS_TCP].s_tcp_port !=
			    TARGET_COMMS_TCP_PORT_UNUSED);
			DBPRINTF("Source %p (%s): TCP active only.\n",
			    (void *)cs,
			    target_comms_names[cs->cs_port].cn_desc);
			assert(cnt_tcp < (TARGET_COMMS_PORT__COUNT * 2));
			src_tcp[cnt_tcp++] = &tcs->tcs_sources[i];
		}
	}

	/*
	 * Now do a pass through the 'src_both[]' list to find pairs which
	 * must be split because they match other sources.
	 */
	for (i = 0; i < cnt_both; i++) {
		uint16_t tcp_port;
		int8_t usb_port;

		css = src_both[i];
		assert(css != NULL);

		/* Fetch the USB/TCP ports for this source. */
		s = css->css_settings;
		tcp_port = s[TCSS_SETTINGS_TCP].s_tcp_port;
		usb_port = s[TCSS_SETTINGS_USB].s_usb_port;

		/* They must both be valid. */
		assert(usb_port != TARGET_COMMS_USB_PORT_UNUSED);
		assert(tcp_port != TARGET_COMMS_TCP_PORT_UNUSED);

		DBPRINTF("Source %p (%s): Both active. Checking for imperfect "
		    "match.\n", (void *)css->css_source,
		    target_comms_names[css->css_source->cs_port].cn_desc);

		/* Check other paired sources first. */
		for (j = i + 1; j < cnt_both; j++) {
			assert(src_both[j] != NULL);

			susb = &src_both[j]->css_settings[TCSS_SETTINGS_USB];
			stcp = &src_both[j]->css_settings[TCSS_SETTINGS_USB];

			/* Ignore if neither port matches. */
			if (usb_port != susb->s_usb_port &&
			    tcp_port != stcp->s_tcp_port) {
				continue;
			}

			/*
			 * Ignore if we find a perfect match; we may be able
			 * to pair these sources later.
			 */
			if (usb_port == susb->s_usb_port &&
			    tcp_port == stcp->s_tcp_port) {
				DBPRINTF("Source %p (%s): Perfect match - "
				    "ignoring.\n", (void *)css->css_source,
				    target_comms_names[css->css_source->cs_port].cn_desc);
				continue;
			}

			DBPRINTF("Source %p (%s): Imperfect match - will "
			    "split.\n", (void *)css->css_source,
			    target_comms_names[css->css_source->cs_port].cn_desc);

			/* This source needs to be split. */
			goto split_it;
		}

		/* Scan for a USB port match in src_usb[] */
		DBPRINTF("Source %p (%s): Both active. Checking for USB "
		    "match.\n", (void *)css->css_source,
		    target_comms_names[css->css_source->cs_port].cn_desc);

		for (j = 0; j < cnt_usb; j++) {
			susb = &src_usb[j]->css_settings[TCSS_SETTINGS_USB];
			if (usb_port != susb->s_usb_port)
				continue;

			DBPRINTF("Source %p (%s): USB match - will split.\n",
			    (void *)css->css_source,
			    target_comms_names[css->css_source->cs_port].cn_desc);

			/* This source needs to be split. */
			goto split_it;
		}

		/* Scan for a TCP port match in src_tcp[] */
		DBPRINTF("Source %p (%s): Both active. Checking for TCP "
		    "match.\n", (void *)css->css_source,
		    target_comms_names[css->css_source->cs_port].cn_desc);

		for (j = 0; j < cnt_tcp; j++) {
			stcp = &src_tcp[j]->css_settings[TCSS_SETTINGS_TCP];
			if (tcp_port != stcp->s_tcp_port)
				continue;

			DBPRINTF("Source %p (%s): TCP match - will split.\n",
			    (void *)css->css_source,
			    target_comms_names[css->css_source->cs_port].cn_desc);

			/* This source needs to be split. */
			goto split_it;
		}

		/*
		 * We get here for one of two reasons:
		 *  1. Neither of the source's ports match any other source.
		 *  2. The source exactly matches another source in src_both[].
		 * In both cases, we leave the source where it is.
		 */
		continue;

 split_it:
		/*
		 * Due to a match with another source, this source must be
		 * removed from src_both[], and split onto src_usb[] and
		 * src_tcp[].
		 */
		/* Mark the unused USB/TCP ports. */
		s[TCSS_SETTINGS_USB].s_tcp_port = TARGET_COMMS_TCP_PORT_UNUSED;
		s[TCSS_SETTINGS_TCP].s_usb_port = TARGET_COMMS_USB_PORT_UNUSED;

		/* Add to the src_usb[] list. */
		assert(cnt_usb < (TARGET_COMMS_PORT__COUNT * 2));
		src_usb[cnt_usb++] = css;

		/* Add to the src_tcp[] list. */
		assert(cnt_tcp < (TARGET_COMMS_PORT__COUNT * 2));
		src_tcp[cnt_tcp++] = css;

		/* Remove from src_both[] */
		src_both[i] = NULL;
	}

	DBPRINTF("Can now start split sources. USB %u, TCP %u\n", cnt_usb,
	    cnt_tcp);

	/*
	 * We can now start all the services in src_usb[] and src_net[].
	 * Since we want to take advantage of port daemon's "chooser", we
	 * group together sources with matching port numbers. This is made
	 * easier by sorting the lists first.
	 */
	if (cnt_usb) {
		/* Sort the list by USB port number. */
		qsort(src_usb, cnt_usb, sizeof(src_usb[0]), tc_usb_sort);

		/* Isolate and start the matching groups. */
		for (i = 0; i < cnt_usb; ) {
			s = &src_usb[i]->css_settings[TCSS_SETTINGS_USB];

			/* Find the end of this group */
			for (j = i + 1; j < cnt_usb; j++) {
				susb = &src_usb[j]->css_settings[
				    TCSS_SETTINGS_USB];
				if (s->s_usb_port != susb->s_usb_port)
					break;
			}

			/* Start it. */
			tc_start_shared(tcs, TCSS_SETTINGS_USB, j - i, &src_usb[i]);
			i = j;
		}
	}

	if (cnt_tcp) {
		/* Sort the list by TCP port number. */
		qsort(src_tcp, cnt_tcp, sizeof(src_tcp[0]), tc_tcp_sort);

		/* Isolate and start the matching groups. */
		for (i = 0; i < cnt_tcp; ) {
			s = &src_tcp[i]->css_settings[TCSS_SETTINGS_TCP];

			/* Find the end of this group */
			for (j = i + 1; j < cnt_tcp; j++) {
				stcp = &src_tcp[j]->css_settings[TCSS_SETTINGS_TCP];
				if (s->s_tcp_port != stcp->s_tcp_port)
					break;
			}

			/* Start it. */
			tc_start_shared(tcs, TCSS_SETTINGS_TCP, j - i, &src_tcp[i]);
			i = j;
		}
	}

	/*
	 * There may still be entries in src_both[] to deal with. Because of
	 * all the checks above, each entry is guaranteed to be a perfect
	 * match with one or more other src_both[] entries.
	 * If we removed some entries from src_both[] by writing NULL, we need
	 * to rebuild the array by shifting the non-NULL entries down and
	 * recomputing cnt_both.
	 */
	j = 0;
	for (i = 0; i < cnt_both; i++) {
		if (src_both[i] != NULL)
			src_both[j++] = src_both[i];
	}
	cnt_both = j;
	DBPRINTF("Remaining unsplit sources: %u\n", cnt_both);

	/* Now start the perfect match groups. */
	if (cnt_both) {
		/*
		 * Sort them by TCP port number. We could use USB port
		 * instead; it makes no difference.
		 */
		qsort(src_both, cnt_both, sizeof(src_both[0]), tc_tcp_sort);

		/* Isolate and start the matching groups. */
		for (i = 0; i < cnt_both; ) {
			s = src_both[i]->css_settings;

			/* Find the end of this group */
			for (j = i + 1; j < cnt_both; j++) {
				stcp = src_both[j]->css_settings;
				if (s->s_tcp_port != stcp->s_tcp_port)
					break;

				assert(s->s_usb_port == stcp->s_usb_port);
			}

			/* Start it. */
			tc_start_shared(tcs, 0, j - i, &src_both[i]);
			i = j;
		}
	}

	/* All registered sources are now active. */
	SHELL_CMD_ADD(tc);

#ifdef NETWORK_OPT_HTTPD
	network_httpd_add_post_handler_furl(target_comms_post_furl, NULL,
            "/targetcomms.cgi");
	target_comms_output_json();
#endif
}

/*
 * Register a comms source. The source will be started when
 * target_comms_start() is invoked.
 */
void
target_comms_register(struct target_comms_source *cs)
{
	struct target_comms_state *tcs = &target_comms_state;
	struct target_comms_source_state *css;
	struct target_comms_config tcc;
	struct port_daemon_settings *s;

	assert((tcs->tcs_num_sources + 1) < TARGET_COMMS_PORT__COUNT);
	assert(tcs->tcs_sources[tcs->tcs_num_sources].css_source == NULL);

	/*
	 * Since this can be called from various thread contexts during
	 * initialisation, we need to serialise access to our global state.
	 */
	rtos_mutex_acquire(tcs->tcs_mutex);

	/* Fetch configuration for this source. */
	if (target_comms_get_config(cs->cs_port, &tcc)) {
		/* Make sure target_comms_names[] also has an entry... */
		assert(target_comms_names[cs->cs_port].cn_cmd != NULL);

		/* Point to the first available entry, and its settings. */
		css = &tcs->tcs_sources[tcs->tcs_num_sources];
		s = &css->css_settings[TCSS_SETTINGS_USB];

		/* Initialise settings. */
		cs->cs_arg.ca_flags = tcc.tcc_flags;
		s->s_usb_port = tcc.tcc_usb_port;
		s->s_tcp_port = tcc.tcc_tcp_port;
		s->s_rb_size_to_host = tcc.tcc_rb_size_to_host;
		s->s_rb_size_from_host = tcc.tcc_rb_size_from_host;
		s->s_flags = 0;

#ifdef CONFIG_USE_NETWORK
		if (TC_FLAGS_DO_TELNET(tcc.tcc_flags))
			s->s_flags |= PORT_DAEMON_SOURCE_FLAG_DO_TELNET;

		if (tcc.tcc_flags & TARGET_COMMS_FLAGS_DO_NAGLE)
			s->s_flags |= PORT_DAEMON_SOURCE_FLAG_DO_NAGLE;
#endif /* CONFIG_USE_NETWORK */

		/* TCP version needs an identical copy. */
		css->css_settings[TCSS_SETTINGS_TCP] = *s;

		/* Record the source. */
		css->css_source = cs;
		tcs->tcs_num_sources++;
	}

	rtos_mutex_release(tcs->tcs_mutex);
}

void
target_comms_init(void)
{

	target_comms_state.tcs_mutex = rtos_mutex_create();
	assert(target_comms_state.tcs_mutex != NULL);
	TAILQ_INIT(&target_comms_state.tcs_shared);

	target_console_init();
}

void
target_comms_load_defaults(void)
{
#ifndef CONFIG_USE_CONFIGDB
	memcpy(glob.glob_target_comms, tc_default_configs,
	    sizeof(glob.glob_target_comms));
#else
	for (target_comms_port_t p = TARGET_COMMS_PORT_TARGET_CONSOLE;
	    p < TARGET_COMMS_PORT__COUNT; p++) {
		cf_del_data(CF_KEY_DATA_TARGET_COMMS_CONFIG(p));
	}
#endif
}
