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

#ifndef GDB_RTOS_FREERTOS_H
#define GDB_RTOS_FREERTOS_H

#include "gdb_rtos.h"

enum {
#define	GDB_RTOS_PARAM(e,f,desc,tt4,tt2,tt4_int,tt2_int)	e,
#include "gdb_rtos_freertos.defs"
#undef GDB_RTOS_PARAM
	GDB_RTOS_FREERTOS_NPARAMS
};

/*
 * 'params' argument to tmon_rtos_get_params(). Passed by reference to
 * target-specific code via TARGET_CTL_GET_RTOS_PARAMS ctl.
 */
struct gdb_rtos_freertos_target_params {
	/* Hint: configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES might be enabled. */
	bool tp_integrity;

	/* Hint: sizeof(TickType_t) is likely to equal sizeof(uint16_t). */
	bool tp_short_ticktype;

	/* Hint: Target may be using MPU wrappers. pcTaskName may have moved. */
	bool tp_mpu_wrappers;

	/* Current params, to be adjusted as necessary. */
	gdb_rtos_params_t tp_params;
};

extern const struct gdb_rtos_backend gdb_rtos_backend_freertos;

GDB_RTOS_PARAMS_DEF(gdb_rtos_freertos_params_t, GDB_RTOS_FREERTOS_NPARAMS);
extern const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt4;
extern const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt2;
extern const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt4_int;
extern const gdb_rtos_freertos_params_t gdb_freertos_params_ilp32_tt2_int;

#endif /* GDB_RTOS_FREERTOS_H */
