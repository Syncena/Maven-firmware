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
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/errno.h>

#include "target.h"
#include "rtos.h"
#include "ringbuff.h"
#include "target_comms.h"
#include "semihosting.h"
#include "tmon.h"
#include "timer.h"
#include "target.h"
#include "gdb_server.h"
#include "gdb_signals.h"
#include "bitmap.h"

//#define DEBUG_FLAG_INIT 1
#include "debug.h"

/*
 * Semihosting System Calls
 */
#define	SH_SYS_OPEN			0x01
	struct sh_sys_open {
		uint32_t name;
		uint32_t mode;
		uint32_t namelen;
	};
#define	SH_SYS_CLOSE			0x02
#define	SH_SYS_WRITEC			0x03
#define	SH_SYS_WRITE0			0x04
#define	SH_SYS_WRITE			0x05
#define	SH_SYS_READ			0x06
	struct sh_sys_read_write {
		uint32_t handle;
		uint32_t data;
		uint32_t datalen;
	};
#define	SH_SYS_READC			0x07
#define	SH_SYS_ISERROR			0x08
#define	SH_SYS_ISTTY			0x09
#define	SH_SYS_SEEK			0x0a
	struct sh_sys_seek {
		uint32_t handle;
		uint32_t offset;
	};
#define	SH_SYS_FLEN			0x0c
#define	SH_SYS_TMPNAM			0x0d
#define	SH_SYS_REMOVE			0x0e
	struct sh_sys_remove {
		uint32_t name;
		uint32_t namelen;
	};
#define	SH_SYS_RENAME			0x0f
	struct sh_sys_rename {
		uint32_t oldname;
		uint32_t oldnamelen;
		uint32_t newname;
		uint32_t newnamelen;
	};
#define	SH_SYS_CLOCK			0x10
#define	SH_SYS_TIME			0x11
#define	SH_SYS_SYSTEM			0x12
	struct sh_sys_system {
		uint32_t command;
		uint32_t commandlen;
	};
#define	SH_SYS_ERRNO			0x13
#define	SH_SYS_GET_CMDLINE		0x15
#define	SH_SYS_HEAPINFO			0x16
	struct sh_sys_heapinfo {
		uint32_t heap_base;
		uint32_t heap_limit;
		uint32_t stack_base;
		uint32_t stack_limit;
	};
#define	SH_SYS_ELAPSED			0x30
#define	SH_SYS_TICKFREQ			0x31

#define	SH_angel_SWIreason_EnterSVC		0x17
#define	SH_angel_SWIreason_ReportException 	0x18
#define	SH_angel_SWIreason_ReportExceptionExtended 	0x20
	struct sh_angel_exception_ex {
		uint32_t reason;
		uint32_t sig;
	};
	/* Values for the above 'reason' member. */
#define	 SH_ADP_Stopped_BranchThroughZero	0x20000
#define	 SH_ADP_Stopped_UndefinedInstr		0x20001
#define	 SH_ADP_Stopped_SoftwareInterrupt	0x20002
#define	 SH_ADP_Stopped_PrefetchAbort		0x20003
#define	 SH_ADP_Stopped_DataAbort		0x20004
#define	 SH_ADP_Stopped_AddressException	0x20005
#define	 SH_ADP_Stopped_IRQ			0x20006
#define	 SH_ADP_Stopped_FIQ			0x20007
#define	 SH_ADP_Stopped_BreakPoint		0x20020
#define	 SH_ADP_Stopped_WatchPoint		0x20021
#define	 SH_ADP_Stopped_StepComplete		0x20022
#define	 SH_ADP_Stopped_RunTimeErrorUnknown	0x20023
#define	 SH_ADP_Stopped_InternalError		0x20024
#define	 SH_ADP_Stopped_UserInterruption	0x20025
#define	 SH_ADP_Stopped_ApplicationExit		0x20026

#define	SH_STATE_SUCCESS		0u
#define	SH_STATE_FAILURE		(~(uint32_t)0u)

#define	SH_MAX_GDB_HANDLES		256

struct semihost_state {
	/* Semihosting state. */
	rtos_mutex_t ss_mutex;
	int ss_errno;
	int ss_stdio_fd;
	unsigned int ss_feature_off;
	void *ss_gdb_fio_handle;
	void *ss_tmon_syscall_handle;

	volatile port_daemon_instance_t ss_port_daemon;
	volatile bool ss_hangup;

	/* Ring buffer for target -> host data */
	ringbuff_t ss_rb_target2host;
	ringbuff_t ss_rb_bounce;

	/*
	 * Ring buffer for host -> target data
	 * This needs to be blocking.
	 */
	ringbuff_blocking_t ss_rb_host2target_blocking;

	struct target_comms_source ss_cs;
	BITMAP_DECLARE(ss_gdb_handles, SH_MAX_GDB_HANDLES);
	char ss_semi_buff[64];
	uint8_t ss_bounce_ring[256];
};
static struct semihost_state semihost_state[CONFIG_GDB_SERVERS];

#define	SH_EXT_EXIT_EXTENDED_BITNUM	(1u << 0)
#define	SH_EXT_STDOUT_STDERR_BITNUM	(1u << 1)
#define	SHFB_MAGIC_0			0x53
#define	SHFB_MAGIC_1			0x48
#define	SHFB_MAGIC_2			0x46
#define	SHFB_MAGIC_3			0x42
static const uint8_t semihost_features[] = {
	SHFB_MAGIC_0, SHFB_MAGIC_1, SHFB_MAGIC_2, SHFB_MAGIC_3,
	SH_EXT_EXIT_EXTENDED_BITNUM | SH_EXT_STDOUT_STDERR_BITNUM
};

#define	SH_FD_STDIN	SH_MAX_GDB_HANDLES
#define	SH_FD_STDOUT	(SH_FD_STDIN + 1)
#define	SH_FD_STDERR	(SH_FD_STDOUT + 1)
#define	SH_FD_FEATURES	(SH_FD_STDERR + 1)

#define	SH_FD_IS_STDIO(fd)	(((unsigned int)(fd) >= SH_FD_STDIN && \
				 (unsigned int)(fd) <= SH_FD_STDERR) || \
				 ((unsigned int)(fd) <= 2))
#define	SH_FD_IS_OPEN(ss, fd)	(SH_FD_IS_STDIO(fd) || ((ss)->ss_stdio_fd & \
				 (1u << ((fd) - SH_FD_STDIN))) == 0)
#define	SH_OPEN_FD(ss, fd)	(ss)->ss_stdio_fd &= ~(1u << ((fd) - SH_FD_STDIN))
#define	SH_CLOSE_FD(ss, fd)	(ss)->ss_stdio_fd |= (1u << ((fd) - SH_FD_STDIN))


static int
semihost_mem_read(tmon_cookie_t tc, uint32_t addr, size_t len, void *dest)
{

	return (tmon_mem_read(tc, addr, len, dest) == TMON_MR_OK) ? 0 : -1;
}

static int
semihost_mem_write(tmon_cookie_t tc, uint32_t addr, size_t len, const void *src)
{

	return (tmon_mem_write(tc, addr, len, src) == TMON_MR_OK) ? 0 : -1;
}

static bool
semihost_is_open_gdb_handle(struct semihost_state *ss, uint32_t fd)
{

	if (fd >= SH_MAX_GDB_HANDLES)
		return false;

	return bitmap_get_state(ss->ss_gdb_handles, (unsigned int)fd) != 0;
}

static int
semihost_set_gdb_handle(struct semihost_state *ss, uint32_t fd)
{

	if (fd >= SH_MAX_GDB_HANDLES)
		return -1;

	bitmap_set_state(ss->ss_gdb_handles, (unsigned int)fd, 1);
	return 0;
}

static void
semihost_clear_gdb_handle(struct semihost_state *ss, uint32_t fd)
{

	assert(fd < SH_MAX_GDB_HANDLES);

	bitmap_set_state(ss->ss_gdb_handles, (unsigned int)fd, 0);
}

static void
semihosting_rb_callback(ringbuff_t rb, void *arg)
{
	struct semihost_state *ss = arg;

	(void) rb;

	if (ss->ss_port_daemon)
		port_daemon_attention(ss->ss_port_daemon);
}

static bool
semihosting_connect(void *arg, port_daemon_instance_t si, ringbuff_t to_host,
    ringbuff_t from_host, unsigned int unused)
{
        target_comms_arg_t ca = arg;
	struct semihost_state *ss = ca->ca_ops_arg;

	(void) unused;

	if (ss->ss_port_daemon != NULL) {
		DBFPRINTF("busy\n");
		return false;
	}

	rtos_mutex_acquire(ss->ss_mutex);

	ringbuff_init(ss->ss_rb_bounce);
	ringbuff_consumer_init(ss->ss_rb_bounce, semihosting_rb_callback, ss);
	ringbuff_producer_init(to_host, semihosting_rb_callback, ss);

	/* Host -> Target is blocking, with 10mS timeout */
	ss->ss_rb_host2target_blocking = ringbuff_blocking_consumer(from_host,
	    10);
	if (ss->ss_rb_host2target_blocking == NULL) {
		DBFPRINTF("blocking rb alloc failed\n");
		rtos_mutex_release(ss->ss_mutex);
		return false;
	}

	/* Target -> Host is non-blocking */
	ss->ss_rb_target2host = to_host;
	ss->ss_port_daemon = si;
	ss->ss_hangup = false;

	rtos_mutex_release(ss->ss_mutex);

	return ss;
}

static void
semihosting_disconnect(void *arg)
{
        target_comms_arg_t ca = arg;
	struct semihost_state *ss = ca->ca_ops_arg;
	ringbuff_blocking_t rb;

	if (ss->ss_port_daemon == NULL) {
		DBFPRINTF("already closed\n");
		return;
	}

	rtos_mutex_acquire(ss->ss_mutex);

	ss->ss_port_daemon = NULL;

	/* Target -> Host is easy */
	ss->ss_rb_target2host = NULL;

	/* The other direction needs some more work */
	rb = ss->ss_rb_host2target_blocking;
	ss->ss_rb_host2target_blocking = NULL;
	ss->ss_hangup = true;
	ringbuff_blocking_close(rb);

	rtos_mutex_release(ss->ss_mutex);
}

static void
semihosting_hangup(void *arg)
{
        target_comms_arg_t ca = arg;
	struct semihost_state *ss = ca->ca_ops_arg;

	ss->ss_hangup = true;
}

static void
semihosting_worker(void *arg)
{
        target_comms_arg_t ca = arg;
	struct semihost_state *ss = ca->ca_ops_arg;

	target_comms_generic_worker(ss->ss_rb_bounce, ss->ss_rb_target2host,
	    ca->ca_flags);
}

/*
 * Read data in the host -> target direction.
 * This will block until either the file handle is closed or
 * the host has disconnected.
 */
static int
semihost_console_read(struct semihost_state *ss, uint8_t *data, uint32_t *lp)
{
	uint32_t len, read_len = 0;
	int rv, i = 0;

	len = *lp;
	rv = TMON_HS_RUNNING;

	while (i >= 0 && len && ss->ss_hangup == false) {
		rtos_mutex_acquire(ss->ss_mutex);

		if (ss->ss_rb_host2target_blocking != NULL) {
			/*
			 * This can return 0 on timeout, -1 on error (the ring
			 * buffer was closed), or the number of bytes read.
			 */
			i = ringbuff_blocking_read(
			    ss->ss_rb_host2target_blocking, data,
			    (ringbuff_len_t)len);
		} else {
			i = -1;
		}

		rtos_mutex_release(ss->ss_mutex);

		if (i > 0) {
			data += i;
			len -= i;
			read_len += i;
		}

		if (ss->ss_gdb_fio_handle &&
		    gdb_server_check_for_interrupt(ss->ss_gdb_fio_handle)) {
			rv = TMON_HS_FORCED;
			break;
		}
	}

	*lp = read_len;

	return rv;
}

static int
semihost_console_write(struct semihost_state *ss, const uint8_t *data,
    uint32_t *lp)
{
	uint32_t write_len, len = *lp;
	int rv, i = 0;

	write_len = 0;
	rv = TMON_HS_RUNNING;

	while (i >= 0 && len && ss->ss_hangup == false) {
		rtos_mutex_acquire(ss->ss_mutex);

		if (ss->ss_port_daemon != NULL) {
			ringbuff_len_t l;
			void *p;

			l = ringbuff_produce_contig_size(ss->ss_rb_bounce);
			if (l) {
				if (l > len)
					l = (ringbuff_len_t)len;
				p = ringbuff_produce_current_buff_pointer(
				    ss->ss_rb_bounce);
				memcpy(p, data, (size_t)l);
				data += l;
				len -= l;
				write_len += l;
				ringbuff_produce_contig(ss->ss_rb_bounce, l);
				ringbuff_produce_done(ss->ss_rb_bounce);
				i = 1;
			} else {
				i = 0;
			}
		} else {
			i = -1;
		}

		rtos_mutex_release(ss->ss_mutex);

		if (ss->ss_gdb_fio_handle &&
		    gdb_server_check_for_interrupt(ss->ss_gdb_fio_handle)) {
			rv = TMON_HS_FORCED;
			break;
		}

		if (i == 0) {
			/* No space. Sleep for a short time */
			rtos_task_sleep(5);
		}
	}

	*lp = write_len;

	return rv;
}

static int
semihost_gdb_fio(struct semihost_state *ss, uint32_t *rvp, const char *fmt, ...)
{
	struct gdb_server_fio_result res;
	va_list ap;
	int rv;

	va_start(ap, fmt);
	ss->ss_semi_buff[0] = 'F';
	rv = vsnprintf(&ss->ss_semi_buff[1], sizeof(ss->ss_semi_buff) - 1, fmt,
	    ap);
	va_end(ap);
	assert(rv > 0 && (unsigned int)rv < (sizeof(ss->ss_semi_buff) - 1));

	DBFPRINTF("%s\n", (const char *)ss->ss_semi_buff);

	rv = gdb_server_fio_request(ss->ss_gdb_fio_handle,
	    ss->ss_semi_buff, &res);

	if (rv != TMON_HS_RUNNING) {
		DBFPRINTF("STOP target. Code %d\n", rv);
		/* Return EINTR (4) or EFAULT (14) according to the status. */
		ss->ss_errno = (rv == TMON_HS_FAULT) ? EINTR : EFAULT;
		*rvp = SH_STATE_FAILURE;
		return rv;
	}

	*rvp = (uint32_t)res.retcode;
	ss->ss_errno = (res.retcode < 0) ? res.gdb_errno : 0;

	DBFPRINTF("retcode %d, err %d\n", res.retcode, ss->ss_errno);

	return rv;
}

#define	GDB_FIO_O_RDONLY	0x000
#define	GDB_FIO_O_WRONLY	0x001
#define	GDB_FIO_O_RDWR		0x002
#define	GDB_FIO_O_APPEND	0x008
#define	GDB_FIO_O_CREAT		0x200
#define	GDB_FIO_O_TRUNC		0x400
#define	GDB_FIO_O_EXCL		0x800

static int
semihost_gdb_sys_open(struct semihost_state *ss, const struct sh_sys_open *args,
    uint32_t *rvp)
{
	static const uint16_t oflags[] = {
		/* r   */ GDB_FIO_O_RDONLY,
		/* rb  */ GDB_FIO_O_RDONLY,
		/* r+  */ GDB_FIO_O_RDWR,
		/* r+b */ GDB_FIO_O_RDWR,
		/* w   */ GDB_FIO_O_WRONLY | GDB_FIO_O_CREAT,
		/* wb  */ GDB_FIO_O_WRONLY | GDB_FIO_O_CREAT,
		/* w+  */ GDB_FIO_O_RDWR | GDB_FIO_O_CREAT,
		/* w+b */ GDB_FIO_O_RDWR | GDB_FIO_O_CREAT,
		/* a   */ GDB_FIO_O_WRONLY | GDB_FIO_O_APPEND | GDB_FIO_O_CREAT,
		/* ab  */ GDB_FIO_O_WRONLY | GDB_FIO_O_APPEND | GDB_FIO_O_CREAT,
		/* a+  */ GDB_FIO_O_RDWR | GDB_FIO_O_APPEND | GDB_FIO_O_CREAT,
		/* a+b */ GDB_FIO_O_RDWR | GDB_FIO_O_APPEND | GDB_FIO_O_CREAT,
	};
	int rv;

	if (args->mode >= (sizeof(oflags) / sizeof(oflags[0]))) {
		ss->ss_errno = EINVAL;	/* EINVAL */
		*rvp = SH_STATE_FAILURE;
		return TMON_HS_RUNNING;
	}

	/* The '1b6' here == 0666 in octal. */
	rv = semihost_gdb_fio(ss, rvp, "open,%" PRIx32 "/%" PRIx32 ",%" PRIx16
	    ",1b6", args->name, args->namelen + 1, oflags[args->mode]);

	if (rv == TMON_HS_RUNNING && (int)*rvp >= 0) {
		if (semihost_set_gdb_handle(ss, *rvp) < 0) {
			/* Oops! Exceeded SH_MAX_GDB_HANDLES! */
			rv = semihost_gdb_fio(ss, rvp, "close,%" PRIx32, *rvp);
			if (rv == TMON_HS_RUNNING) {
				ss->ss_errno = ENFILE;	/* ENFILE */
				*rvp = SH_STATE_FAILURE;
			}
		}
	}

	return rv;
}

static int
semihost_sys_open(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_open args;

	DBPRINTF("\nSYS_OPEN(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read params\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("name %08" PRIx32 ", mode %" PRIu32 ", namelen %" PRIu32 ": ",
	    args.name, args.mode, args.namelen);

	/*
	 * We need to read the filename to catch the special files
	 * ":tt" and ":semi..."
	 */
	if (args.namelen >= 1 && args.namelen < sizeof(ss->ss_semi_buff)) {
		/* Fetch the filename. */
		if (semihost_mem_read(tc, args.name, args.namelen,
		    ss->ss_semi_buff)) {
			DBPRINTF("Failed to read name\n");
			return TMON_HS_FAULT;
		}

		ss->ss_semi_buff[args.namelen] = '\0';

		DBPRINTF("%s, ", (const char *)ss->ss_semi_buff);

		if (strcasecmp((const char *)ss->ss_semi_buff, ":tt") == 0) {
			uint32_t fd;

			switch (args.mode) {
			case 0:
				fd = SH_FD_STDIN;
				break;
			case 4:
				fd = SH_FD_STDOUT;
				break;
			case 8:
				fd = SH_FD_STDERR;
				break;
			default:
				fd = SH_STATE_FAILURE;
				break;
			}

			*rvp = (uint32_t)fd;

			if (fd != SH_STATE_FAILURE) {
				SH_OPEN_FD(ss, fd);
				DBPRINTF("fd %" PRIx32 "\n", fd);
			} else {
				ss->ss_errno = ENFILE;	/* ENFILE */
				DBPRINTF("ENFILE\n");
			}
			return TMON_HS_RUNNING;
		} else
		if (strcasecmp((const char *)ss->ss_semi_buff,
		    ":semihosting-features") == 0) {
			/* Special fd */
			if (args.mode < 4) {
				*rvp = SH_FD_FEATURES;
				ss->ss_feature_off = 0;
				DBPRINTF("fd %u\n", SH_FD_FEATURES);
			} else {
				ss->ss_errno = EPERM;	/* EPERM */
				*rvp = SH_STATE_FAILURE;
				DBPRINTF("EPERM\n");
			}
			return TMON_HS_RUNNING;
		}
	}

	/* GDB might have more luck. */
	if (ss->ss_gdb_fio_handle) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_sys_open(ss, &args, rvp);
	}

	DBPRINTF("ENFILE\n");

	ss->ss_errno = ENFILE;	/* ENFILE */
	*rvp = SH_STATE_FAILURE;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_close(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	uint32_t fd;

	DBPRINTF("\nSYS_CLOSE(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(fd), &fd) < 0) {
		DBPRINTF("Failed to read params\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("fd = %" PRIu32 "\n", fd);

	if (semihost_is_open_gdb_handle(ss, fd)) {
		int rv;

		rv = semihost_gdb_fio(ss, rvp, "close,%" PRIx32, fd);

		if (rv == TMON_HS_RUNNING && *rvp == SH_STATE_SUCCESS) {
			semihost_clear_gdb_handle(ss, fd);

			/*
			 * XXX: Do we need to track stdio handles locally?
			 */
		}

		return rv;
	}

	if (fd == SH_FD_FEATURES) {
		*rvp = SH_STATE_SUCCESS;
	} else
	if (SH_FD_IS_OPEN(ss, fd)) {
		if (fd >= SH_FD_STDIN)
			SH_CLOSE_FD(ss, fd);
		/* XXX: Do we need to close the Gdb equivalent too? */
		*rvp = SH_STATE_SUCCESS;
	} else {
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = SH_STATE_FAILURE;
	}

	return TMON_HS_RUNNING;
}

static int
semihost_gdb_sys_read_write(struct semihost_state *ss,
    const struct sh_sys_read_write *args, uint32_t *rvp, bool is_write)
{
	uint32_t fd = args->handle;
	int rv;

	if (fd >= SH_FD_STDIN)
		fd -= SH_FD_STDIN;

	rv = semihost_gdb_fio(ss, rvp, "%s,%" PRIx32 ",%" PRIx32 ",%" PRIx32,
	    is_write ? "write" : "read", fd, args->data, args->datalen);

	if (rv != TMON_HS_RUNNING)
		return rv;

	if ((int)*rvp < 0)
		*rvp = args->datalen;
	else
		*rvp = args->datalen - *rvp;

	return rv;
}

static int
semihost_sys_writec(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	uint32_t len;
	char ch;
	int rv;

	DBPRINTF("\nSYS_WRITEC(%08" PRIx32 "): ", param);

	if (ss->ss_gdb_fio_handle != NULL && ss->ss_port_daemon == NULL) {
		struct sh_sys_read_write args;

		DBPRINTF("Handing to Gdb\n");

		args.handle = SH_FD_STDOUT;
		args.data = param;
		args.datalen = 1;

		return semihost_gdb_sys_read_write(ss, &args, rvp, true);
	}

	if (semihost_mem_read(tc, param, sizeof(ch), &ch) < 0) {
		DBPRINTF("Failed to read character\n");
		return TMON_HS_FAULT; /* XXX: Spec says no error return! */
	}

	DBPRINTF("ch: %02x\n", ch);

	len = sizeof(ch);
	rv = semihost_console_write(ss, (const uint8_t *)&ch, &len);

	if (rv == TMON_HS_RUNNING) {
		*rvp = SH_STATE_SUCCESS;
	} else {
		*rvp = SH_STATE_FAILURE;
		ss->ss_errno = EINTR;	/* EINTR */
	}

	return rv;
}

static int
semihost_sys_write0(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	uint32_t len;
	char ch;
	int rv;

	DBPRINTF("\nSYS_WRITE0(%08" PRIx32 "):\n", param);

	if (ss->ss_gdb_fio_handle != NULL && ss->ss_port_daemon == NULL) {
		struct sh_sys_read_write args;

		DBPRINTF("Handing to Gdb\n");

		args.handle = SH_FD_STDOUT;
		args.data = param;
		args.datalen = 0;

		/*
		 * We need to find the end of the string to make this
		 * syscall suitable for Gdb FIO.
		 */
		do {
			if (semihost_mem_read(tc, param++, sizeof(ch),
			    &ch) < 0) {
				DBPRINTF("Failed to read character\n");
				return TMON_HS_FAULT;
			}

			if (ch != '\0')
				args.datalen++;
		} while (ch != '\0');

		/* Let Gdb handle it. */
		return semihost_gdb_sys_read_write(ss, &args, rvp, true);
	}

	/* Read the string, one char at a time. */
	rv = TMON_HS_RUNNING;
	do {
		if (semihost_mem_read(tc, param++, sizeof(ch), &ch) < 0) {
			DBPRINTF("Failed to read character\n");
			return TMON_HS_FAULT;
		}

		len = sizeof(ch);
		if (ch != '\0') {
			rv = semihost_console_write(ss, (const uint8_t *)&ch,
			    &len);
		}
	} while (ch != '\0' && len != 0 && rv == TMON_HS_RUNNING);

	if (rv == TMON_HS_RUNNING) {
		*rvp = SH_STATE_SUCCESS;
	} else {
		*rvp = SH_STATE_FAILURE;
		ss->ss_errno = EINTR;	/* EINTR */
	}

	return rv;
}

static int
semihost_sys_write(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_read_write args;
	uint32_t thislen;
	int rv = TMON_HS_RUNNING;

	DBPRINTF("\nSYS_WRITE(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read params\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("handle %" PRIu32 ", data %08" PRIx32 ", datalen %" PRIu32
	    "\n", args.handle, args.data, args.datalen);

	if (semihost_is_open_gdb_handle(ss, args.handle) ||
	   (SH_FD_IS_STDIO(args.handle) && ss->ss_port_daemon == NULL)) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_sys_read_write(ss, &args, rvp, true);
	}

	if (args.handle == SH_FD_FEATURES) {
		ss->ss_errno = EPERM;	/* EPERM */
		*rvp = args.datalen;
		return rv;
	} else
	if (!SH_FD_IS_OPEN(ss, args.handle) || args.handle == SH_FD_STDIN) {
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = args.datalen;
		return rv;
	}

	while (ss->ss_port_daemon != NULL && args.datalen) {
		if (args.datalen > sizeof(ss->ss_semi_buff))
			thislen = sizeof(ss->ss_semi_buff);
		else
			thislen = args.datalen;

		if (semihost_mem_read(tc, args.data, thislen,
		    ss->ss_semi_buff) < 0) {
			DBPRINTF("Failed to read data\n");
			return TMON_HS_FAULT;
		}

		rv = semihost_console_write(ss,
		    (const uint8_t *)ss->ss_semi_buff, &thislen);
		if (thislen == 0 || rv != TMON_HS_RUNNING) {
			DBPRINTF("Console write failed.\n");
			break;
		}

		args.data += thislen;
		args.datalen -= thislen;
	}

	if (rv == TMON_HS_RUNNING)
		*rvp = args.datalen;
	else
		*rvp = GDB_SIGNAL_INT;

	return rv;
}

static int
semihost_sys_read_semihosting_features(struct semihost_state *ss,
    tmon_cookie_t tc, const struct sh_sys_read_write *args, uint32_t *rvp)
{
	uint32_t remaining = sizeof(semihost_features) - ss->ss_feature_off;

	if (remaining == 0) {
		*rvp = args->datalen;	/* EOF */
		return TMON_HS_RUNNING;
	}

	if (remaining > args->datalen)
		remaining = args->datalen;

	DBFPRINTF("offset %u, len %" PRIu32 "\n", ss->ss_feature_off,
	    remaining);

	if (semihost_mem_write(tc, args->data, remaining,
	    &semihost_features[ss->ss_feature_off]) < 0) {
		DBPRINTF("Failed to write data\n");
		return TMON_HS_FAULT;
	}

	ss->ss_feature_off += remaining;
	*rvp = args->datalen - remaining;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_read(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_read_write args;
	uint32_t thislen;
	int rv = TMON_HS_RUNNING;

	DBPRINTF("\nSYS_READ(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read params\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("handle %" PRIu32 ", data %08" PRIx32 ", datalen %" PRIu32
	    "\n", args.handle, args.data, args.datalen);

	if (semihost_is_open_gdb_handle(ss, args.handle) ||
	   (SH_FD_IS_STDIO(args.handle) && ss->ss_port_daemon == NULL)) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_sys_read_write(ss, &args, rvp, false);
	}

	if (args.handle == SH_FD_FEATURES) {
		DBPRINTF("Reading semihosting-features\n");
		return semihost_sys_read_semihosting_features(ss, tc, &args,
		    rvp);
	} else
	if (!SH_FD_IS_OPEN(ss, args.handle) || args.handle != SH_FD_STDIN) {
		DBPRINTF("EBADF\n");
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = args.datalen;	/* Return datalen, as per spec. */
		return rv;
	}

	DBPRINTF("Reading from console\n");

	while (ss->ss_port_daemon != NULL && args.datalen) {
		if (args.datalen > sizeof(ss->ss_semi_buff))
			thislen = sizeof(ss->ss_semi_buff);
		else
			thislen = args.datalen;

		rv = semihost_console_read(ss, (uint8_t *)ss->ss_semi_buff,
		    &thislen);
		if (thislen == 0 || rv != TMON_HS_RUNNING)
			break;	/* EOF or host CTRL-C */

		if (semihost_mem_write(tc, args.data, thislen,
		    ss->ss_semi_buff) < 0) {
			DBPRINTF("Failed to write data\n");
			return TMON_HS_FAULT;
		}

		args.data += thislen;
		args.datalen -= thislen;
	}

	if (rv == TMON_HS_RUNNING)
		*rvp = args.datalen;
	else
		*rvp = GDB_SIGNAL_INT;

	DBPRINTF("Returning %u, value %" PRIu32 "\n", rv, *rvp);

	return rv;
}

static int
semihost_sys_readc(struct semihost_state *ss, tmon_cookie_t tc, uint32_t sp,
    uint32_t *rvp)
{
	uint8_t ch;
	int rv = TMON_HS_RUNNING;

	DBPRINTF("\nSYS_READC()\n");

	if (ss->ss_gdb_fio_handle != NULL && ss->ss_port_daemon == NULL) {
		struct sh_sys_read_write args;
		uint32_t ch32;

		/*
		 * To handle this, we need some memory on the target.
		 * Open up enough space on the target's stack to hold a
		 * byte, rounded up to 32-bits to keep things aligned.
		 */
		sp = (sp & ~3u) - sizeof(uint32_t);

		args.handle = SH_FD_STDIN;
		args.data = sp;
		args.datalen = 1;

		DBPRINTF("Handing to Gdb\n");

		rv = semihost_gdb_sys_read_write(ss, &args, rvp, true);
		if (rv != TMON_HS_RUNNING)
			return rv;

		if (*rvp == 0) {
			/* Recover the value from the target's stack. */
			if (semihost_mem_read(tc, sp, sizeof(ch32), &ch32) < 0)
				return TMON_HS_FAULT;

			*rvp = ch32 & 0xffu;
		} else {
			ss->ss_errno = EBADF;	/* EBADF */
			*rvp = SH_STATE_FAILURE;
		}

		return rv;
	}

	uint32_t l = sizeof(ch);
	rv = semihost_console_read(ss, &ch, &l);
	if (l == 0)
		ch = 0;

	if (rv == TMON_HS_RUNNING) {
		*rvp = (uint32_t)ch;
	} else {
		*rvp = SH_STATE_FAILURE;
		ss->ss_errno = EINTR;	/* EINTR */
	}

	return rv;
}

static int
semihost_sys_iserr(tmon_cookie_t tc, uint32_t param, uint32_t *rvp)
{
	uint32_t errcode;

	DBPRINTF("\nSYS_ISERROR(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(errcode), &errcode) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("code 0x%" PRIx32 "\n", errcode);

	*rvp = ((int)errcode < 0) ? 1 : 0;
	return TMON_HS_RUNNING;
}

static int
semihost_gdb_sys_lseek(struct semihost_state *ss, uint32_t fd,
    uint32_t offset, int whence, uint32_t *rvp)
{

	return semihost_gdb_fio(ss, rvp, "lseek,%" PRIx32 ",%" PRIx32 ",%x", fd,
	    offset, whence);
}
#define	GDB_SEEK_SET	0
#define	GDB_SEEK_CUR	1
#define	GDB_SEEK_END	2

static int
semihost_sys_seek(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_seek args;

	DBPRINTF("\nSYS_SEEK(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("fd %" PRIu32 ", offset %" PRIu32 "\n", args.handle,
	    args.offset);

	if (semihost_is_open_gdb_handle(ss, args.handle)) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_sys_lseek(ss, args.handle, args.offset,
		    GDB_SEEK_SET, rvp);
	}

	if (args.handle == SH_FD_FEATURES) {
		if (args.offset > sizeof(semihost_features)) {
			ss->ss_errno = ESPIPE;	/* ESPIPE */
			*rvp = SH_STATE_FAILURE;
			return TMON_HS_RUNNING;
		}
		ss->ss_feature_off = args.offset;
	} else
	if (!SH_FD_IS_OPEN(ss, args.handle)) {
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = SH_STATE_FAILURE;
		return TMON_HS_RUNNING;
	}

	*rvp = SH_STATE_SUCCESS;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_flen(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	uint32_t fd, flen;

	DBPRINTF("\nSYS_FLEN(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(fd), &fd) < 0) {
		DBPRINTF("Failed to read handle\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("fd %" PRIu32 "\n", fd);

	if (semihost_is_open_gdb_handle(ss, fd)) {
		uint32_t pos;
		int rv;

		DBPRINTF("Handing to Gdb\n");

		/* Fetch the current file position. */
		rv = semihost_gdb_sys_lseek(ss, fd, 0, GDB_SEEK_CUR, &pos);
		if (rv != TMON_HS_RUNNING)
			return rv;

		DBPRINTF("Current fpos: 0x%" PRIx32 "\n", pos);

		/* Seek to EOF. */
		rv = semihost_gdb_sys_lseek(ss, fd, 0, GDB_SEEK_END, rvp);
		if (rv != TMON_HS_RUNNING)
			return rv;

		DBPRINTF("End fpos: 0x%" PRIx32 "\n", *rvp);

		/* Seek back to the previous position. */
		rv = semihost_gdb_sys_lseek(ss, fd, pos, GDB_SEEK_SET, &pos);

		DBPRINTF("Original fpos: 0x%" PRIx32 "\n", pos);
		return rv;
	}

	flen = 0;
	if (fd == SH_FD_FEATURES) {
		flen = sizeof(semihost_features);
	} else
	if (!SH_FD_IS_OPEN(ss, fd)) {
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = SH_STATE_FAILURE;
		return TMON_HS_RUNNING;
	}

	DBPRINTF("File length %" PRIu32 "\n", flen);
	*rvp = flen;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_remove(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_remove args;

	DBPRINTF("\nSYS_REMOVE(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	if (ss->ss_gdb_fio_handle) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_fio(ss, rvp, "unlink,%" PRIx32 "/%" PRIx32,
		    args.name, args.namelen + 1);
	}

	DBPRINTF("ENOENT\n");
	ss->ss_errno = ENOENT;	/* ENOENT */
	*rvp = SH_STATE_FAILURE;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_rename(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_rename args;

	DBPRINTF("\nSYS_RENAME(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	if (ss->ss_gdb_fio_handle) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_fio(ss, rvp, "rename,%" PRIx32 "/%" PRIx32
		    ",%" PRIx32 "/%" PRIx32, args.oldname, args.oldnamelen + 1,
		    args.newname, args.newnamelen + 1);
	}

	DBPRINTF("ENOENT\n");
	ss->ss_errno = ENOENT;	/* ENOENT */
	*rvp = SH_STATE_FAILURE;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_clock(uint32_t *rvp)
{
	struct timeval tv;

	DBPRINTF("\nSYS_CLOCK()\n");

	timer_get_mono_time(&tv);

	*rvp = (uint32_t)((tv.tv_sec * 100u) + (tv.tv_usec / 10000u));

	return TMON_HS_RUNNING;
}

static int
semihost_sys_elapsed(tmon_cookie_t tc, uint32_t param, uint32_t *rvp)
{
	struct timeval tv;
	uint64_t rv;

	DBPRINTF("\nSYS_ELAPSED(%" PRIu32 ")\n", param);

	timer_get_mono_time(&tv);

	rv = (uint64_t)tv.tv_sec;
	rv *= 1000u;
	rv += tv.tv_usec / 1000u;

	if (semihost_mem_write(tc, param, sizeof(rv), &rv) < 0)
		return TMON_HS_FAULT;

	*rvp = SH_STATE_SUCCESS;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_angel_exception(uint32_t param, uint32_t *rvp)
{

	DBPRINTF("\nSYS_ANGEL_EXCEPTION(0x%" PRIx32 ")\n", param);

	switch (param) {
	case SH_ADP_Stopped_ApplicationExit:
		*rvp = GDB_SIGNAL_0;
		return TMON_HS_EXITED;

	case SH_ADP_Stopped_RunTimeErrorUnknown:
		*rvp = GDB_SIGNAL_ABRT;
		break;

	default:
		*rvp = GDB_SIGNAL_SYS;
		break;
	}

	return TMON_HS_FORCED;
}

static int
semihost_sys_angel_exception_ex(tmon_cookie_t tc, uint32_t param, uint32_t *rvp)
{
	struct sh_angel_exception_ex args;

	DBPRINTF("\nSYS_ANGEL_EXCEPTIONEXTENDED(0x%" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("error reading params\n");
		return TMON_HS_FAULT;
	}

	DBPRINTF("reason 0x%" PRIx32 ", sig %" PRIu32 "\n", args.reason,
	    args.sig);

	if (args.reason == SH_ADP_Stopped_ApplicationExit) {
		*rvp = args.sig;
		return TMON_HS_EXITED;
	}

	*rvp = args.sig;
	return TMON_HS_FORCED;
}

static int
semihost_sys_tickfreq(uint32_t *rvp)
{

	DBPRINTF("\nSYS_TICKFREQ()\n");

	*rvp = TIMER_HZ;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_heapinfo(tmon_cookie_t tc, uint32_t param, uint32_t *rvp)
{
	struct sh_sys_heapinfo arg;

	DBPRINTF("\nSYS_HEAPINFO(%" PRIu32 ")\n", param);

	memset(&arg, 0, sizeof(arg));

	if (semihost_mem_write(tc, param, sizeof(arg), &arg) < 0)
		return TMON_HS_FAULT;

	*rvp = SH_STATE_SUCCESS;
	return TMON_HS_RUNNING;
}

static int
semihost_sys_istty(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	uint32_t fd;

	DBPRINTF("\nSYS_ISTTY(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(fd), &fd) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	if (semihost_is_open_gdb_handle(ss, fd)) {
		DBPRINTF("Handing to Gdb\n");
		return semihost_gdb_fio(ss, rvp, "isatty,%" PRIx32 "", fd);
	}

	if (fd == SH_FD_FEATURES) {
		DBPRINTF("0\n");
		*rvp = 0;
	} else
	if (!SH_FD_IS_OPEN(ss, fd)) {
		DBPRINTF("EBADF\n");
		ss->ss_errno = EBADF;	/* EBADF */
		*rvp = SH_STATE_FAILURE;
	} else {
		DBPRINTF("1\n");
		*rvp = 1;
	}

	return TMON_HS_RUNNING;
}

static int
semihost_sys_time(struct semihost_state *ss, tmon_cookie_t tc, uint32_t sp,
    uint32_t *rvp)
{
	uint32_t the_time = SH_STATE_FAILURE;
	int rv;

	DBPRINTF("\nSYS_TIME(%08" PRIx32 "): ", sp);

	if (ss->ss_gdb_fio_handle != NULL) {
		/*
		 * We can use Gdb's gettimeofday() call for this, but
		 * first we must open up some space on the target's stack
		 * in which Gdb will write the struct timeval, defined in
		 * the Gdb docs to be two 32-bit words.
		 */
		sp = (sp & ~3u) - (sizeof(uint32_t) * 2);

		DBPRINTF("Using Gdb gettimeofday\n");

		rv = semihost_gdb_fio(ss, rvp, "gettimeofday,%" PRIx32 ",0",
		    sp);
		if (rv != TMON_HS_RUNNING)
			return rv;

		/* Recover the first element (tv_sec) of the structure. */
		if (*rvp == 0) {
			if (semihost_mem_read(tc, sp, sizeof(the_time),
			    &the_time) < 0) {
				return TMON_HS_FAULT;
			}

			DBPRINTF("Gdb gettimeofday returned 0x%08" PRIx32 ".",
			    the_time);

			/* The value is big-endian. Swap it. */
			the_time = __builtin_bswap32(the_time);

			DBPRINTF("After swap 0x%08" PRIx32 "\n", the_time);
		} else {
			DBPRINTF("Gdb gettimeofday failed 0x%08" PRIx32 "\n",
			    *rvp);
		}
	}

	if (the_time == SH_STATE_FAILURE) {
		struct timeval tv;

		/* Just return our up-time. */

		DBPRINTF("Faking gettimeofday()\n");

		timer_get_mono_time(&tv);
		the_time = (uint32_t)tv.tv_sec;
	}

	*rvp = the_time;

	return TMON_HS_RUNNING;
}
static int
semihost_sys_system(struct semihost_state *ss, tmon_cookie_t tc, uint32_t param,
    uint32_t *rvp)
{
	struct sh_sys_system args;

	DBPRINTF("\nSYS_SYSTEM(%08" PRIx32 "): ", param);

	if (semihost_mem_read(tc, param, sizeof(args), &args) < 0) {
		DBPRINTF("Failed to read param\n");
		return TMON_HS_FAULT;
	}

	if (ss->ss_gdb_fio_handle != NULL) {
		DBPRINTF("Handing to Gdb\n");

		/* commandlen == 0 is special. */
		if (args.commandlen)
			args.commandlen++;

		return semihost_gdb_fio(ss, rvp, "system,%" PRIx32 "/%" PRIx32,
		    args.command, args.commandlen);
	}

	ss->ss_errno = EPERM;	/* EPERM */
	*rvp = SH_STATE_FAILURE;

	return TMON_HS_RUNNING;
}

static void
semihost_syscall_init(struct semihost_state *ss)
{
	unsigned int i;
	uint32_t rv;

	ss->ss_errno = 0;
	ss->ss_feature_off = 0;
	ss->ss_stdio_fd = 0;
	SH_CLOSE_FD(ss, SH_FD_STDIN);
	SH_CLOSE_FD(ss, SH_FD_STDOUT);
	SH_CLOSE_FD(ss, SH_FD_STDERR);

	/*
	 * Close all open Gdb handles.
	 * Note: Bitmap entries for stdin/out/err will never be set.
	 */
	for (i = 0; i < SH_MAX_GDB_HANDLES; i++) {
		if (semihost_is_open_gdb_handle(ss, i))
			(void) semihost_gdb_fio(ss, &rv, "close,%x", i);
	}

	bitmap_init(ss->ss_gdb_handles, SH_MAX_GDB_HANDLES);
}

/*
 * Service an ARM Semi-Hosting system call.
 */
static int
semihosting_call(void *arg, tmon_cookie_t tc, struct target_syscall *ts,
    uint32_t *rvp)
{
	struct semihost_state *ss = arg;
	uint32_t op, param;
	int rv;

	if (ts == NULL) {
		/*
		 * This indicates that the target went away.
		 */
		semihost_syscall_init(ss);
		return 0;
	}

	op = ts->ts_args[0];
	param = ts->ts_args[1];

	switch (op) {
	case SH_SYS_OPEN:
		rv = semihost_sys_open(ss, tc, param, rvp);
		break;

	case SH_SYS_CLOSE:
		rv = semihost_sys_close(ss, tc, param, rvp);
		break;

	case SH_SYS_WRITEC:
		rv = semihost_sys_writec(ss, tc, param, rvp);
		break;

	case SH_SYS_WRITE0:
		rv = semihost_sys_write0(ss, tc, param, rvp);
		break;

	case SH_SYS_WRITE:
		rv = semihost_sys_write(ss, tc, param, rvp);
		break;

	case SH_SYS_READ:
		rv = semihost_sys_read(ss, tc, param, rvp);
		break;

	case SH_SYS_READC:
		rv = semihost_sys_readc(ss, tc, ts->ts_sp, rvp);
		break;

	case SH_SYS_ISERROR:
		rv = semihost_sys_iserr(tc, param, rvp);
		break;

	case SH_SYS_SEEK:
		rv = semihost_sys_seek(ss, tc, param, rvp);
		break;

	case SH_SYS_FLEN:
		rv = semihost_sys_flen(ss, tc, param, rvp);
		break;

	case SH_SYS_REMOVE:
		rv = semihost_sys_remove(ss, tc, param, rvp);
		break;

	case SH_SYS_RENAME:
		rv = semihost_sys_rename(ss, tc, param, rvp);
		break;

	case SH_SYS_CLOCK:
		rv = semihost_sys_clock(rvp);
		break;

	case SH_SYS_HEAPINFO:
		rv = semihost_sys_heapinfo(tc, param, rvp);
		break;

	case SH_SYS_ISTTY:
		rv = semihost_sys_istty(ss, tc, param, rvp);
		break;

	case SH_SYS_TIME:
		rv = semihost_sys_time(ss, tc, ts->ts_sp, rvp);
		break;

	case SH_SYS_SYSTEM:
		rv = semihost_sys_system(ss, tc, param, rvp);
		break;

	case SH_SYS_ERRNO:
		DBPRINTF("\nSH_SYS_ERRNO. %d\n", ss->ss_errno);
		*rvp = ss->ss_errno;
		rv = TMON_HS_RUNNING;
		break;

	case SH_SYS_ELAPSED:
		rv = semihost_sys_elapsed(tc, param, rvp);
		break;

	case SH_SYS_TICKFREQ:
		rv = semihost_sys_tickfreq(rvp);
		break;

	case SH_angel_SWIreason_ReportException:
		rv = semihost_sys_angel_exception(param, rvp);
		break;

	case SH_angel_SWIreason_ReportExceptionExtended:
		rv = semihost_sys_angel_exception_ex(tc, param, rvp);
		break;

	case SH_SYS_GET_CMDLINE:
	case SH_SYS_TMPNAM:
		*rvp = SH_STATE_FAILURE;
		ss->ss_errno = ENOSYS;	/* ENOSYS */
		rv = TMON_HS_RUNNING;
		break;

	default:
		DBFPRINTF("unsupported op 0x%" PRIu32 "\n", op);
		*rvp = GDB_SIGNAL_SYS;
		rv = TMON_HS_FORCED;
		break;
	}

	if (rv == TMON_HS_RUNNING) {
		/* Jump over the BKPT instruction. */
		ts->ts_pc += 2;
	}

	return rv;
}

void
semihosting_init()
{
	static const struct port_daemon_ops semihost_ops = {
		.op_connect = semihosting_connect,
		.op_disconnect = semihosting_disconnect,
		.op_worker = semihosting_worker,
		.op_ctrl = NULL,
		.op_hangup = semihosting_hangup,
	};
	struct semihost_state *ss;

	for (int i = 0; i < CONFIG_GDB_SERVERS; i++) {
		ss = &semihost_state[i];

		ss->ss_mutex = rtos_mutex_create();
		assert(ss->ss_mutex != NULL);

		semihost_syscall_init(ss);

		ss->ss_gdb_fio_handle = NULL;
		ss->ss_port_daemon = NULL;

#if (CONFIG_GDB_SERVERS == 1)
		ss->ss_cs.cs_port = TARGET_COMMS_PORT_SEMIHOST0;
#else
		if (i == 0)
			ss->ss_cs.cs_port = TARGET_COMMS_PORT_SEMIHOST0;
		else
			ss->ss_cs.cs_port = TARGET_COMMS_PORT_SEMIHOST1;
#endif
		ss->ss_cs.cs_ops = &semihost_ops;
		ss->ss_cs.cs_arg.ca_ops_arg = ss;

		ss->ss_rb_bounce = ringbuff_alloc(ss->ss_bounce_ring,
		    sizeof(ss->ss_bounce_ring));

		if (ss->ss_rb_bounce != NULL)
			target_comms_register(&ss->ss_cs);
	}
}

void
gdb_server_syscall_hook(unsigned int gdb_instance, tmon_cookie_t tc,
    void *handle)
{
	struct semihost_state *ss;

	assert(gdb_instance < CONFIG_GDB_SERVERS);

	ss = &semihost_state[gdb_instance];

	if (handle != NULL) {
		assert(ss->ss_gdb_fio_handle == NULL ||
		    handle == ss->ss_gdb_fio_handle);

		if (ss->ss_tmon_syscall_handle == NULL) {
			ss->ss_tmon_syscall_handle =
			    tmon_register_syscall_handler(tc, semihosting_call,
			    ss);
			if (ss->ss_tmon_syscall_handle != NULL) {
				semihost_syscall_init(ss);
				ss->ss_gdb_fio_handle = handle;
			}
		}
	} else
	if (ss->ss_tmon_syscall_handle != NULL) {
		assert(ss->ss_gdb_fio_handle != NULL);

		tmon_delete_syscall_handler(ss->ss_tmon_syscall_handle);
		ss->ss_tmon_syscall_handle = NULL;
		ss->ss_gdb_fio_handle = NULL;
	}
}
