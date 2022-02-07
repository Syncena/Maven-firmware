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

#ifndef JTAG_IF_H
#define JTAG_IF_H

#include <stdbool.h>

/*
 * Low-level API
 */
struct jtag_hw_state;
typedef struct jtag_hw_state *jtag_hw_state_t;

struct jtag_hw_if {
	/* All done with the hardware interface. */
	void (*hi_done)(jtag_hw_state_t);

	/* Toggle TRST if available. Can be NULL if TRST not supported. */
	void (*hi_reset)(jtag_hw_state_t);

	/* Clock up to 32-bits of TMS data. TDI unchanged, TDO ignored. */
	void (*hi_clock_tms)(jtag_hw_state_t, unsigned int, uint32_t);

	/* Clock up to 32-bits of TDO/TDI data. */
	uint32_t (*hi_clock_tdo_tdi)(jtag_hw_state_t, unsigned int, uint32_t,
				     bool);
};
typedef const struct jtag_hw_if *jtag_hw_if_t;

/*
 * High-level API
 */
struct jtag_state;
typedef struct jtag_state *jtag_state_t;

struct jtag_if {
	void (*ji_done)(jtag_state_t);
	void (*ji_reset)(jtag_state_t);
	int (*ji_probe)(jtag_state_t);
	uint32_t (*ji_get_idcode)(jtag_state_t, int);
	void (*ji_select_device)(jtag_state_t, int);
	void (*ji_write_ir)(jtag_state_t, uint32_t);
	void (*ji_rdwr_dr)(jtag_state_t, const uint32_t *, uint32_t *,
			   unsigned int);
};
typedef const struct jtag_if *jtag_if_t;

extern jtag_state_t jtag_init(const jtag_hw_if_t, jtag_hw_state_t,
			      jtag_if_t *, unsigned int);

#endif /* JTAG_IF_H */
