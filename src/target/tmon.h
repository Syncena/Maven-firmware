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

#ifndef TMON_H
#define TMON_H

#include <stdbool.h>
#include "rtos.h"
#include "target.h"
#include "stringio.h"

typedef unsigned int tmon_cookie_t;

/*
 * Fetches the name of the target SoC and CPU.
 */
struct tmon_get_target_name {
	const char *tn_core_name;
	const char *tn_vendor_name;
	const char *tn_soc_name;
	const char *tn_soc_extra[2];
};
extern int	tmon_get_target_name(tmon_cookie_t,
				     struct tmon_get_target_name *);
#define	TMON_QN_ERROR		(-1)
#define	TMON_QN_OK		0

/*
 * Queries if there is an active debug session with the current target.
 * Returns status, or -1 if there is no target.
 */
extern int	tmon_query_attach(tmon_cookie_t *, unsigned int target_idx,
				  int do_attach);
#define	TMON_QA_ERROR		(-1)
#define	TMON_QA_TIMEOUT		(-2)
#define	TMON_QA_DETACHED	0
#define	TMON_QA_ATTACHED	1

/*
 * Detaches from the current target.
 * Returns status, or -1 if there is no target.
 */
extern int	tmon_detach(tmon_cookie_t);
#define	TMON_DT_ERROR		(-1)
#define	TMON_DT_DETACHED	0

/*
 * Requests the halt status of the current target.
 * Returns status, or -1 if there is no target.
 */
struct tmon_watch {
	uint32_t tw_type;
#define	TMON_WATCH_TYPE_BSOFT	0
#define	TMON_WATCH_TYPE_BHARD	1
#define	TMON_WATCH_TYPE_WRITE	2
#define	TMON_WATCH_TYPE_READ	3
#define	TMON_WATCH_TYPE_ACCESS	4
	target_addr_t tw_addr;
};
extern int	tmon_halt_status(tmon_cookie_t, struct tmon_watch *,
				 int just_query);
#define	TMON_HS_ERROR			(-1)
#define	TMON_HS_DETACHED		0
#define	TMON_HS_FAULT			1
#define	TMON_HS_BREAKPOINT		2
#define	TMON_HS_WATCHPOINT		3
#define	TMON_HS_FORCED			4
#define	TMON_HS_SINGLE_STEP		5
#define	TMON_HS_RUNNING			6
#define	TMON_HS_SYSCALL			7
#define	TMON_HS_EXITED			8

/*
 * Requests the current target CPU is forcibly halted.
 * Returns the same status codes as tmon_halt_status(). Normally
 * this would be TMON_HS_FORCED but if the target ignore the request
 * because it's stuck on a WFI instruction then it will return
 * TMON_HS_RUNNING. On error, -1 is returned.
 */
extern int	tmon_halt(tmon_cookie_t);

/*
 * Requests that the target resume execution. Behaviour depends on the
 * parameters:
 *  - step
 *    If this is true, the CPU will execute a single instruction and
 *    stop. Otherwise it will run until halted (fault, breakpoint, or
 *    halt request).
 *  - addr
 *    If this is non-NULL, it points to a value which will be loaded
 *    into the target PC before resuming.
 *
 * Returns status, or -1 if there is no target.
 */
extern int	tmon_resume(tmon_cookie_t, bool step,const target_addr_t *addr);
#define	TMON_RS_ERROR			(-1)
#define	TMON_RS_DETACHED		0
#define	TMON_RS_OK			1

/*
 * Reset the target.
 */
extern int	tmon_reset(tmon_cookie_t, int and_halt);
#define	TMON_RT_ERROR			(-1)
#define	TMON_RT_OK			0
#define	TMON_RT_RESCAN			1

/*
 * If 'regs' is non-NULL, fill the buffer pointed to by 'regs' with the
 * current values of all the target registers.
 *
 * Returns the number of bytes copied (even if 'regs' is NULL), or -1 on error.
 */
extern int	tmon_read_regs(tmon_cookie_t, void *);
#define	TMON_RR_ERROR			(-1)

/*
 * Update all target registers with the values stored in the buffer
 * pointed to by 'regs'.
 *
 * Returns -1 on error.
 */
extern int	tmon_write_regs(tmon_cookie_t, void *);
#define	TMON_WR_ERROR			(-1)
#define	TMON_WR_OK			0

/*
 * If 'regbuff' is non-NULL, fill the buffer pointed to by 'regbuff' with the
 * current value of the target register 'reg'.
 *
 * Returns the number of bytes copied (even if 'regbuff' is NULL),
 * or -1 on error.
 */
extern int	tmon_read_one_reg(tmon_cookie_t, int reg, void *regbuff);
#define	TMON_RO_ERROR			(-1)

/*
 * Write the target register 'reg' with the value stored in the buffer
 * pointed to by 'regbuff'.
 *
 * Returns -1 on error.
 */
extern int	tmon_write_one_reg(tmon_cookie_t, int reg, void *regbuff);
#define	TMON_WO_ERROR			(-1)
#define	TMON_WO_OK			0

/*
 * Set or clear a breakpoint or watchpoint. The breakpoint/watchpoint is
 * described by 'bwp'. It is set if 'set' is true, otherwise an existing
 * breakpoint/watchpoint is cleared.
 *
 * Returns -1 on error.
 */
struct tmon_breakwatch {
	target_addr_t tb_address;
	uint16_t tb_type;
	uint16_t tb_kind;		/* Note: CPU specific. See GDB docs */
};
#define	TMON_BW_TYPE_BREAK_SOFT		0
#define	TMON_BW_TYPE_BREAK_HARD		1
#define	TMON_BW_TYPE_WATCH_WRITE	2
#define	TMON_BW_TYPE_WATCH_READ		3
#define	TMON_BW_TYPE_WATCH_ANY		4
extern int	tmon_breakwatch(tmon_cookie_t,
				const struct tmon_breakwatch *bwp, int set);
#define	TMON_BW_ERROR			(-1)
#define	TMON_BW_OK			0
#define	TMON_BW_UNSUPPORTED		1

/*
 * Read 'len' bytes of data from target memory starting at address 'addr'
 * into the buffer 'dest'. There are no special alignment constraints.
 *
 * Returns -1 on error, else zero.
 */
extern int	tmon_mem_read(tmon_cookie_t, target_addr_t addr,
			      uint32_t len, void *dest);
#define	TMON_MR_ERROR			(-1)
#define	TMON_MR_OK			0

/*
 * Write 'len' bytes of data to target memory starting at address 'addr'
 * from the buffer 'src'. There are no special alignment constraints.
 *
 * Returns -1 on error, else zero.
 */
extern int	tmon_mem_write(tmon_cookie_t, target_addr_t addr,
			       uint32_t len, const void *src);
#define	TMON_MW_ERROR			(-1)
#define	TMON_MW_OK			0

/*
 * Writes the XML description of the target's registers, as required by GDB,
 * into the stringio_t buffer 's'.
 */
extern int	tmon_get_xml_regs(tmon_cookie_t, stringio_t s);
#define	TMON_XR_ERROR			(-1)
#define	TMON_XR_OK			0

/*
 * Copies the XML description of the target's memory into the stringio_t
 * object 's'.
 */
extern int	tmon_get_xml_memory(tmon_cookie_t, stringio_t s);
#define	TMON_XM_ERROR			(-1)
#define	TMON_XM_OK			0

/*
 * If an RTOS is detected, this sets the address of the saved register
 * state for a thread. Subsequent calls to tmon_get_xml_regs, tmon_read_regs,
 * tmon_read_one_reg, tmon_write_regs, tmon_write_one_reg will use the
 * saved register values rather than the live values (where necessary).
 * The layout of 'params' is RTOS-dependent.
 * 
 * The RTOS in use is specified by 'rtos'. Set to TARGET_RTOS_NONE to
 * always use the live register values. This is the default.
 */
extern int	tmon_rtos_set_frame(tmon_cookie_t, target_rtos_type_t rtos,
				    target_addr_t frame, void *params);
#define	TMON_RSF_ERROR			(-1)
#define	TMON_RSF_OK			0

/*
 * For cross-platform RTOS support, the target-specific back-end may
 * provide details of things like pointer size, word size, and RTOS
 * structure offsets. The following call will supply this information.
 * The layout of 'params' is RTOS-dependent.
 */
extern int	tmon_rtos_load_params(tmon_cookie_t, target_rtos_type_t rtos,
				      void *params);
#define	TMON_RLP_ERROR			(-1)
#define	TMON_RLP_OK			0

/*
 * Flash memory operations
 */
extern int	tmon_flash_erase(tmon_cookie_t, target_addr_t, uint32_t);
extern int	tmon_flash_write(tmon_cookie_t, target_addr_t, uint32_t,
				 const void *src);
extern int	tmon_flash_done(tmon_cookie_t);
#define	TMON_FL_ERROR			(-1)
#define	TMON_FL_OK			0
#define	TMON_FL_NOT_FLASH		1

/*
 * Target info
 */
extern int	tmon_target_info(tmon_cookie_t, struct target_info *);
#define	TMON_TI_ERROR			(-1)
#define	TMON_TI_OK			0

/*
 * DP Error
 */
extern int	tmon_get_link_error(tmon_cookie_t, uint32_t *);
#define	TMON_LINK_ERROR	(-1)
#define	TMON_LINK_OK	0

/*
 * Forget the current target and re-scan the SWD/JTAG
 */
extern int	tmon_rescan(void);
#define	TMON_TR_OK			0

/*
 * Invoke the supplied callback with the target locked. Useful
 * for when shell commands need target access.
 */
extern uintptr_t tmon_locked_callback(uintptr_t (*)(void *), void *);

/*
 * Returns the number of targets, if any.
 */
extern unsigned int tmon_get_target_count(void);

/*
 * Returns true if the target will be reset upon attach.
 */
extern bool tmon_attach_will_reset(void);

/*
 * Returns true if Flash verification is enabled.
 */
extern bool tmon_flash_verify_enabled(void);

/*
 * Target system calls (e.g. ARM Semihosting)
 */
typedef int (*tmon_target_syscall_handler_t)(void *, tmon_cookie_t,
		struct target_syscall *, uint32_t *);
extern void *tmon_register_syscall_handler(tmon_cookie_t,
		tmon_target_syscall_handler_t, void *);
extern void tmon_delete_syscall_handler(void *);

/*
 * Product specific stuff
 */
extern void	tmon_init(void);
extern void	tmon_log(const char *, ...);
extern FILE	*tmon_log_fp(void);
extern void	tmon_load_defaults(void);
extern void	tmon_task(rtos_task_t);
extern const char *tmon_status(uint32_t *);

#ifndef CONFIG_USE_CONFIGDB
#define	TMON_GLOBAL_VARS					\
	uint32_t	glob_db_type;				\
	int		glob_attach_reset;			\
	u_int		glob_swo_mode; /* XXX: Unused */	\
	uint8_t		glob_scan_type;				\
	uint8_t		glob_flash_verify;
#endif

#define TMON_DP_TYPE_JTAGDP	0x4a544147	/* "JTAG" */
#define TMON_DP_TYPE_SWDP	0x53574450	/* "SWDP */
#define TMON_DP_TYPE_SWJDP	0x53574a54	/* "SWJT */

#endif /* TMON_H */
