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

#ifndef GDB_SERVER_H
#define GDB_SERVER_H

#ifndef CONFIG_GDB_SERVERS
#define	CONFIG_GDB_SERVERS	1
#endif

struct gdb_server_fio_result {
	int retcode;
	int gdb_errno;
};

extern void gdb_server_init(void);

/*
 * Send a Gdb File I/O request to the client. This will block until
 * a reply has been received and placed in 'result'.
 * The return value is one of the TMON_HS_* codes, suitable for
 * passing back to tmon on syscall exit. CTRL-C is handled internally.
 *
 * NOTE! DANGER, WILL ROBINSON!
 * If the host disconnects, this function will *NOT* return. There are
 * longjmp(3)s deep in the host I/O code which effectively force-quit
 * the Gdb server. Therefore you must ensure you are not holding on to
 * a mutex or suchlike. The server will invoke
 * gdb_server_syscall_hook(instance, tnum, NULL) shortly after the longjmp().
 * You should take that opportunity to clean up.
 */
extern int gdb_server_fio_request(void *handle, const char *reqstr,
		struct gdb_server_fio_result *result);

/*
 * While processing a file I/O request locally, as in, console I/O to/from
 * Maven's terminal server, the following function should be called
 * periodically to check if the user has attempted to stop the target
 * using CTRL-C. The function returns 'true' in this case.
 *
 * NOTE! DANGER, WILL ROBINSON!
 * If the host disconnects, this function will *NOT* return. There are
 * longjmp(3)s deep in the host I/O code which effectively force-quit
 * the Gdb server. Therefore you must ensure you are not holding on to
 * a mutex or suchlike. The server will invoke
 * gdb_server_syscall_hook(instance, NULL) shortly after the longjmp().
 * You should take that opportunity to clean up.
 */
extern bool gdb_server_check_for_interrupt(void *handle);

/*
 * This function must be provided by CPU-specific support code.
 * See semihosting.c for an example of how this is implemented
 * for ARM Cortex-M devces.
 *
 * First parameter is the Gdb server instance number (usually 0, but can
 * be non-zero if the probe configuration supports multiple Gdb server
 * instance.
 *
 * Second parameter is the target cookie associated with the CPU core
 * with which the Gdb server is communicating.
 *
 * Third parameter is an opaque handle which must be passed as
 * the first parameter of gdb_server_fio_request().
 *
 * The function is invoked when a host connects to a Gdb server
 * instance, with a non-NULL 'handle'.
 * It is also invoked upon host disconnection, where 'handle' will
 * be NULL.
 */
extern void gdb_server_syscall_hook(unsigned int gdb_instance,
		tmon_cookie_t tc, void *handle);

/*
 * If called in the context of a GDB server task, this will
 * return the instance number associated with that server.
 * Otherwise it will return -1.
 */
extern int gdb_server_get_instance(void);

extern void gdb_server_load_defaults(void);

#endif /* GDB_SERVER_H */
