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

#ifndef TARGET_H
#define TARGET_H

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include "linked-lists.h"
#include "platform.h"
#ifdef NETWORK_OPT_HTTPD
#include "network_httpd.h"
#include "stringio.h"
#endif

#ifndef CONFIG_MAX_TARGETS
#define	CONFIG_MAX_TARGETS	8	/* Support for up to 8 cores. */
#endif

struct target;
typedef struct target *target_t;

struct system;
typedef struct system *system_t;

struct target_memory;
typedef struct target_memory *target_mem_t;
TAILQ_HEAD(target_mem_qhead, target_memory);

struct target_flash;
typedef struct target_flash *target_flash_t;

#ifdef TARGET_64BIT_SUPPORT
typedef uint64_t target_addr_t;
#define PRIxTADDR	PRIx64
#define SCNxTADDR	SCNx64
#else
typedef uint32_t target_addr_t;
#define PRIxTADDR	PRIx32
#define SCNxTADDR	SCNx32
#endif	/* TARGET_64BIT_SUPPORT */

struct target_flash_sector {
	/* Size of this sector, in bytes. */
	uint32_t fs_size;

	/*
	 * Number of write-sized pages in sector. Note that all pages
	 * within the sector must be the same size, which implies
	 * that fs_pages * write-size == fs_size.
	 */
	uint32_t fs_pages;

	/* Logical sector number */
	uint32_t fs_number;

	/* For driver use */
	uintptr_t fs_cookie;
};

struct target_flash_sector_cache_entry {
	target_addr_t ce_base_offset;
	struct target_flash_sector ce_sector;
};

typedef int (*target_flash_sector_desc_t)(target_flash_t, target_addr_t *,
		struct target_flash_sector *);
typedef int (*target_flash_erase_sector_t)(target_flash_t,
		const struct target_flash_sector *);
typedef int (*target_flash_write_page_t)(target_flash_t,
		const struct target_flash_sector *, uint32_t);
typedef int (*target_flash_finish_t)(target_flash_t);
typedef void (*target_flash_free_t)(target_flash_t);

#define	TARGET_FLASH_RESULT_ERROR	(-1)
#define	TARGET_FLASH_RESULT_OK		0
#define	TARGET_FLASH_RESULT_VERIFIED	1

struct target_flash {
	/* Must be initialised by SoC code before calling target_add_flash() */
	target_flash_sector_desc_t tf_sector_desc;
	target_flash_erase_sector_t tf_erase_sector;
	target_flash_write_page_t tf_write_page;
	target_flash_finish_t tf_finish;
	target_flash_free_t tf_free;

	uint32_t tf_size;		/* Total size of Flash */
	uint32_t tf_max_page_size;	/* Max write-unit size */
	uint8_t tf_erase_byte;

	/*
	 * This will be initialised by target_add_flash() to point to a
	 * buffer of size "tf_max_page_size" bytes.
	 */
	uint8_t *tf_buffer;

	/* No user-servicable parts below */
	int tf_buffer_dirty;
	uint32_t tf_buffer_page;
	target_addr_t tf_page_addr;
	struct target_flash_sector tf_buffer_sector;

	struct target_flash_sector_cache_entry tf_sector_cache;
};

/*
 * The following structure is supplied by the 'link' layer for the target
 * architecture. On ARM, for example, this will be the 'adiv5' module.
 */
struct target_link_ops {
	/*
	 * Read/Write a single 32-bit value at the specified target address.
	 * Returns zero on success, else -1.
	 * These methods are optimised for single 32-bit quantities.
	 * 'addr' must be 32-bit aligned.
	 */
	int (*lo_reg_read32)(void *, target_addr_t addr, uint32_t *result);
	int (*lo_reg_write32)(void *, target_addr_t addr, uint32_t data);

	/*
	 * Read/Write a single 16-bit value at the specified target address.
	 * Returns zero on success, else -1.
	 * These methods are optimised for single 16-bit quantities.
	 * 'addr' must be 16-bit aligned.
	 */
	int (*lo_reg_read16)(void *, target_addr_t addr, uint16_t *result);
	int (*lo_reg_write16)(void *, target_addr_t addr, uint16_t data);

	/*
	 * Read/Write a chunk of memory to/from the target at the specified
	 * address. Returns zero on success, else -1.
	 * These methods are optimised for reading/writing buffers and
	 * can handle any alignment.
	 */
	int (*lo_mem_read)(void *, target_addr_t addr, uint32_t len, void *p);
	int (*lo_mem_write)(void *, target_addr_t addr, uint32_t len,
			    const void *p);

	/*
	 * General 'ioctl' interface.
	 */
	int (*lo_ioctl)(void *, int cmd, void *arg);

	/*
	 * Returns non-zero if a link error has been flagged since the last
	 * invocation. The return value is the actual error status as
	 * reported by the underlying debug architecture.
	 */
	uint32_t (*lo_link_error)(void *);

	/*
	 * Call this periodically to check if the link is still up.
	 * Returns 0 on success, else -1.
	 */
	int (*lo_ping)(void *);

	/*
	 * Call this when you're finished with the device.
	 */
	void (*lo_free)(void *);
};
typedef const struct target_link_ops * const target_link_ops_t;

struct target {
	u_int t_index;
	void *t_core;
	int (*t_ctl)(target_t, uint32_t *, void *);
	void (*t_free)(target_t);
	u_int t_flags;
	struct target_mem_qhead t_private_mem;
	struct target_link_ops t_link_ops;
	void *t_link_ops_arg;
	uint32_t t_link_error;
	uint32_t t_flash_max_page_size;
	void *t_flash_verify_buffer;
	bool t_flash_verify_enabled;
};
#define	TARGET_FLAG_SUPPORTS_PROTECTION	(1u << 0)
#define	TARGET_FLAG_SUPPORTS_PERM_PROT	(1u << 1)
#define	TARGET_FLAG_IS_LOCKED		(1u << 2)
#define	TARGET_FLAG_FLASH_RDONLY	(1u << 3)

/*
 * Return codes from target match routine
 */
#define	TARGET_ATTACH_NO_MATCH	(-1)
#define	TARGET_ATTACH_OK	0
#define	TARGET_ATTACH_RESCAN	1

/*
 * Efficiently read/write target hardware register space.
 */
#define	TARGET_REG_READ32(t,a,p)	(((t)->t_link_ops.lo_reg_read32) \
					 ((t)->t_link_ops_arg,(a),(p)))
#define	TARGET_REG_WRITE32(t,a,v)	(((t)->t_link_ops.lo_reg_write32) \
					 ((t)->t_link_ops_arg,(a),(v)))
#define	TARGET_REG_READ16(t,a,p)	(((t)->t_link_ops.lo_reg_read16) \
					 ((t)->t_link_ops_arg,(a),(p)))
#define	TARGET_REG_WRITE16(t,a,v)	(((t)->t_link_ops.lo_reg_write16) \
					 ((t)->t_link_ops_arg,(a),(v)))

/*
 * Read/Write target memory.
 *
 * Note: Do No Use Directly unless you are absolutely sure the target
 * memory in question is non-cached. Usee TARGET_CTL_MEM_READWRITE
 * instead to ensure cache is dealt with.
 */
#define	TARGET_MEM_READ(t,a,l,b)	(((t)->t_link_ops.lo_mem_read) \
					 ((t)->t_link_ops_arg,(a),(l),(b)))
#define	TARGET_MEM_WRITE(t,a,l,b)	(((t)->t_link_ops.lo_mem_write) \
					 ((t)->t_link_ops_arg,(a),(l),(b)))

/*
 * Link 'ioctl'.
 */
#define	TARGET_LINK_IOCTL(t,c,a)	(((t)->t_link_ops.lo_ioctl) \
					 ((t)->t_link_ops_arg,(c),(a)))

/*
 * Check for link errors since the last invocation.
 */
#define	TARGET_LINK_ERROR(t)		(((t)->t_link_ops.lo_link_error) \
					 ((t)->t_link_ops_arg))

/*
 * Check if the target is still responding.
 */
#define	TARGET_LINK_PING(t)		target_ping(t)

/*
 * Commands for target_ctl()
 */
/* Does nothing, as the name would suggest */
#define	TARGET_CTL_NOP			0

/* 'arg' is a pointer to a 'struct target_ctl_get_target_name' */
#define	TARGET_CTL_GET_TARGET_NAME	1
struct target_ctl_get_target_name {
	const char *tn_core_name;
	const char *tn_vendor_name;
	const char *tn_soc_name;
	const char *tn_soc_extra[2];
};

/* 'arg' is a pointer to an int */
#define	TARGET_CTL_QUERY_ATTACH		2
#define	TARGET_CTL_ATTACH		3

/* 'arg' is NULL */
#define	TARGET_CTL_DETACH		4

/*
 * 'arg' points to a variable of type target_addr_t.
 * Returns TARGET_HALT_STATE_* or -1 on error.
 */
struct target_halt {
	uint32_t th_type;	/* TARGET_BKWP_STATUS_* */
	target_addr_t th_addr;	/* Break/Watchpoint address */
};
struct target_syscall {
	target_addr_t ts_pc;
	target_addr_t ts_sp;
	uint32_t ts_args[2];
};
union target_halt_state {
	struct target_halt ths_halt;
	struct target_syscall ths_syscall;
};
#define	TARGET_CTL_HALT_POLL		5
#define	TARGET_CTL_HALT_QUERY		6
#define	 TARGET_HALT_STATE_NOT_ATTACHED		0
#define	 TARGET_HALT_STATE_FAULT		1
#define	 TARGET_HALT_STATE_BREAKPOINT		2	/* ths_halt */
#define	 TARGET_HALT_STATE_WATCHPOINT		3	/* ths_halt */
#define	 TARGET_HALT_STATE_FORCED		4
#define	 TARGET_HALT_STATE_SINGLE_STEP		5
#define	 TARGET_HALT_STATE_RUNNING		6
#define	 TARGET_HALT_STATE_SYSCALL		7	/* ths_syscall */
#define	 TARGET_HALT_STATE_ERROR		8

/* 'arg' (optional) points to a variable of type struct target_watchbreak */
#define	TARGET_CTL_HALT			7

/*
 * 'arg' points to a 'struct target_resume'.
 */
#define	TARGET_CTL_RESUME		8
struct target_resume {
	int tr_flags;
	target_addr_t tr_addr;
	uint32_t tr_rv;		/* Syscall result value. */
};
#define	 TARGET_RESUME_FLAG_SINGLE_STEP	(1u << 0)
#define	 TARGET_RESUME_FLAG_ADDR_VALID	(1u << 1)
#define	 TARGET_RESUME_FLAG_RV_VALID	(1u << 2)

/*
 * 'arg' points to a variable of type 'int' which has one of the defined
 * values.
 */
#define	TARGET_CTL_RESET		9
#define	 TARGET_RESET_SOFT_NO_HALT	0	/* Soft reset no CPU halt */
#define	 TARGET_RESET_SOFT_HALT		1	/* Soft reset and halt CPU */
#define	 TARGET_RESET_HARD		2	/* Hard reset. */

/*
 * 'arg' points to memory buffer containing register values, or NULL if
 * caller only wants to query the required buffer size.
 * Return value is the buffer size or -1 on error.
 */
#define	TARGET_CTL_READ_REGS		10

/* 'arg' points to memory buffer containing register values */
#define	TARGET_CTL_WRITE_REGS		11

/*
 * 'arg' points to a struct target_rdwr_one_reg, or NULL if
 * caller only wants to query the required buffer size.
 * Return value is the buffer size or -1 on error.
 */
#define	TARGET_CTL_READ_ONE_REG		12
struct target_rdwr_one_reg {
	unsigned int or_reg;	/* Register number */
	void *or_regbuff;	/* Buffer with register's value */
};

/*
 * 'arg' points to a struct target_rdwr_one_reg. Return value is zero
 * on success or -1 on failure.
 */
#define	TARGET_CTL_WRITE_ONE_REG	13

/*
 * 'arg' points to a struct target_watchbreak. Return value is one of the
 * TARGET_BKWP_STATUS_* values, below.
 */
struct target_watchbreak {
	int tw_set;		/* Non-zero to set, else clear */
	target_addr_t tw_addr;	/* Break/Watchpoint address */
	uint32_t tw_type;	/* One of TARGET_BKWP_TYPE_*, below */
	uint32_t tw_kind;	/* CPU specific, from GDB */
};
#define	TARGET_CTL_WATCHBREAK		14
/* XXX: Keep this in sync with the TMON_BW_* versions */
#define	 TARGET_BKWP_TYPE_SOFT_BREAK	0
#define	 TARGET_BKWP_TYPE_HARD_BREAK	1
#define	 TARGET_BKWP_TYPE_WRITE_WATCH	2
#define	 TARGET_BKWP_TYPE_READ_WATCH	3
#define	 TARGET_BKWP_TYPE_ACCESS_WATCH	4
#define	 TARGET_BKWP_STATUS_OK		0
#define	 TARGET_BKWP_STATUS_UNSUPPORTED	1
#define	 TARGET_BKWP_STATUS_ERROR	2

/* 'arg' points to a struct target_mem_readwrite */
#define	TARGET_CTL_MEM_READWRITE	15
#define	TARGET_CTL_MEM_POST_READWRITE	16
struct target_mem_readwrite {
	bool mr_write;			/* False for read request, else write */
	target_addr_t mr_target_addr;	/* Target address to read/write */
	uint32_t mr_length;		/* # of bytes to read/write */
	union {
		void *mru_dest;		/* destination buffer for reading */
		const void *mru_src;	/* source buffer for writing */
	} mr_u;
#define	mr_src	mr_u.mru_src
#define	mr_dest	mr_u.mru_dest
};

/*
 * 'arg' points to a buffer into which the XML register description is written.
 * Pass a NULL pointer to query the length of buffer required.
 * Return value is the size of the required buffer.
 */
#define	TARGET_CTL_GET_XML_REGS		17

/*
 * 'arg' points to a buffer into which the XML memory description is written.
 */
#define	TARGET_CTL_GET_XML_MEMORY	18

/*
 * 'arg' points to a struct target_mem_readwrite. Only mr_target_addr and
 * mr_length are used. Return value is zero on success, or -1 on error.
 */
#define	TARGET_CTL_FLASH_ERASE		19

/*
 * 'arg' points to a struct target_mem_readwrite. Only mr_target_addr,
 * mr_length and mr_src are used. Return value is zero on success, or -1
 * on error.
 */
#define	TARGET_CTL_FLASH_WRITE		20

/* 'arg' is NULL */
#define	TARGET_CTL_FLASH_DONE		21

/* 'arg' points to a struct target_info */
#define	TARGET_CTL_TARGET_INFO		22
struct target_info {
	FILE *ti_os;
#ifdef NETWORK_OPT_HTTPD
	stringio_t ti_sio;
#endif
};

/* 'arg' points to a struct target_protection_query */
#define	TARGET_CTL_PROTECTION_QUERY	23
#define	TARGET_CTL_PROTECTION_SET	24
struct target_protection_info {
	const char *pi_text;
	u_int pi_max_level;
	u_int pi_level;
	u_int pi_permanent;
	FILE *pi_out_file;
};
#define	TARGET_PI_OK			0
#define	TARGET_PI_OK_POWER_CYCLE	1
#define	TARGET_PI_OK_RESET		2
#define	TARGET_PI_UNCHANGED		3
#define	TARGET_PI_NEED_PERM_FLAG	4
#define	TARGET_PI_USE_UNLOCK		5

/* 'arg' is a FILE * to whcih the output should be written */
#define	TARGET_CTL_UNLOCK		25

/* 'arg' points to a uint32_t */
#define	TARGET_CTL_GET_LINK_ERROR	26

/* 'arg' points to a struct target_run_applet */
#define	TARGET_CTL_APPLET_RUN		27
struct target_run_applet_arg {
	unsigned int aa_reg;
	uint32_t aa_val;
};
struct target_run_applet {
	target_addr_t ra_applet_address;
	target_addr_t ra_applet_sp;
	unsigned int ra_nargs;
	const struct target_run_applet_arg *ra_args;
};
/* 'arg' is pointer to uint32_t, or NULL */
#define	TARGET_CTL_APPLET_STOP		28

/* 'arg' points to a struct target_rtos, or NULL for no RTOS */
#define	TARGET_CTL_SET_RTOS_FRAME	29
typedef enum {
	TARGET_RTOS_FREERTOS,
	TARGET_RTOS_NUTTX,
	TARGET_RTOS_NONE,	/* Keep this one at the end. */
} target_rtos_type_t;
#define	TARGET_RTOS_COUNT	TARGET_RTOS_NONE
struct target_rtos {
	target_rtos_type_t tr_rtos;
	target_addr_t tr_frame;
	void *tr_params;
};

/* 'arg' points to a struct target_rtos_params */
#define	TARGET_CTL_LOAD_RTOS_PARAMS	30
struct target_rtos_params {
	target_rtos_type_t rp_rtos;
	void *rp_params;
};

extern void target_init(void);
extern target_t *target_probe(unsigned int *);
extern void target_cleanup(void);
extern int target_ping(target_t);

/*
 * The following constant is derived from the maximum number of characters
 * per line on the display (21).
 */
#define	TARGET_NAME_LEN_MAX	22
extern void target_register_name(const char *vendor, const char *soc,
				 const char *extra1, const char *extra2);

#define	TARGET_MEM_FLASH	(0u << 0)
#define	TARGET_MEM_ROM		(1u << 0)
#define	TARGET_MEM_RAM		(2u << 0)
#define	TARGET_MEM_TYPE_MASK	(3u << 0)
#define	TARGET_MEM_EXCLUSIVE	(1u << 2)
#define	TARGET_MEM_BEST_GUESS	(1u << 3)
#define	TARGET_MEM_OPTIONS	(1u << 4)
#define	TARGET_MEM_EEPROM	(1u << 5)
#define	TARGET_MEM_FLAGS_MASK	(TARGET_MEM_EXCLUSIVE | \
				 TARGET_MEM_BEST_GUESS | \
				 TARGET_MEM_OPTIONS | \
				 TARGET_MEM_EEPROM)
#define	TARGET_MEM_FILLER	(1u << 6)
#define	TARGET_MEM_NO_APPLET	(1u << 7)
extern int target_add_memory(target_t, unsigned int, target_addr_t, uint32_t,
			     const char *);
extern int target_degap_memory(target_addr_t, target_addr_t);
extern int target_get_memory_type(target_t, target_addr_t, unsigned int *);
extern int target_alloc_ram(target_t, unsigned int, target_addr_t *);

extern int target_add_flash(target_t, target_flash_t, target_addr_t, uint32_t,
			    const char *, unsigned int);

extern int target_ctl(target_t, uint32_t, void *);
extern void target_info_all(FILE *);
extern void target_load_defaults(void);
#ifdef NETWORK_OPT_HTTPD
extern void target_json_all(stringio_t);
extern void target_debug_update(stringio_t);
extern int target_debug_update_settings(int,
		const struct network_http_furl_key_value *);
#endif

/*
 * Invoked by the architecture-specific code to allocate/free a target
 * instance.
 */
extern target_t target_alloc(target_link_ops_t lo, void *arg);
extern void target_free(target_t);

/*
 * Supplied by the architecture-specific code.
 */
struct target_arch {
	void * (*ta_init)(void);
	bool (*ta_probe)(void *);
	int (*ta_attach)(void *);
	void (*ta_detach)(void *);
	void (*ta_load_defaults)(void);
#ifdef NETWORK_OPT_HTTPD
	void (*ta_debug_update)(void *, stringio_t);
	int (*ta_debug_update_settings)(void *, int,
	    const struct network_http_furl_key_value *);
#endif /* NETWORK_OPT_HTTPD */
};

#define	TARGET_ARCH_FUNC(arch, func)	__CONCAT(target_,__CONCAT(arch, func))
#ifdef NETWORK_OPT_HTTPD
#define	TARGET_ARCH_HTTP_INIT(arch)					\
	.ta_debug_update = TARGET_ARCH_FUNC(arch, _debug_update),	\
	.ta_debug_update_settings = TARGET_ARCH_FUNC(arch,_debug_update_settings)
#else
#define	TARGET_ARCH_HTTP_INIT(arch)		/* Nothing */
#endif /* NETWORK_OPT_HTTPD */

#define	TARGET_ARCH_DECLARE(arch)					\
	const struct target_arch target_arch = {			\
		.ta_init = TARGET_ARCH_FUNC(arch, _init),		\
		.ta_probe = TARGET_ARCH_FUNC(arch, _probe),		\
		.ta_attach = TARGET_ARCH_FUNC(arch, _attach),		\
		.ta_detach = TARGET_ARCH_FUNC(arch, _detach),		\
		.ta_load_defaults = TARGET_ARCH_FUNC(arch, _load_defaults), \
		TARGET_ARCH_HTTP_INIT(arch)				\
	}

#endif /* TARGET_H */
