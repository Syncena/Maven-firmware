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

#ifndef CORTEXM_TXML_H
#define CORTEXM_TXML_H

#include "stringio.h"

#define CORTEXM_TXML_DEFINITIONS
#include "cortexm_txml.inc"
#undef  CORTEXM_TXML_DEFINITIONS

enum cortexm_txml_reg_enum {
#define	CORTEXM_TXML_CORE(n,e,r,fx)	e,
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_CORE

#define	CORTEXM_TXML_V6M(n,e,r,fx)	e,
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V6M

#define	CORTEXM_TXML_V78M(n,e,r,fx)	e,
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V78M

#define	CORTEXM_TXML_V8M_NS(n,e,r,fx)	e,
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V8M_NS

#define	CORTEXM_TXML_V8M_S(n,e,r,fx)	e,
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V8M_S

	CORTEXM_REG_MAX
};

struct cortexm_txml_regmap {
	const char *rm_name;
	uint8_t rm_enum;
	uint8_t rm_regsel;
	uint16_t rm_flags;
};

extern const struct cortexm_txml_regmap cortexm_txml_regmap_v6m[];
extern const struct cortexm_txml_regmap cortexm_txml_regmap_v78m[];
extern const struct cortexm_txml_regmap cortexm_txml_regmap_v8m_ns[];
extern const struct cortexm_txml_regmap cortexm_txml_regmap_v8m_s[];

extern int cortexm_txml_generate(cortexm_t, stringio_t);
extern int cortexm_txml_one_reg_read(cortexm_t, struct target_rdwr_one_reg *);
extern int cortexm_txml_one_reg_write(cortexm_t, struct target_rdwr_one_reg *);
extern int cortexm_txml_read_regs(cortexm_t, uint8_t *);
extern int cortexm_txml_write_regs(cortexm_t, const uint8_t *);

struct cortexm_txml_regframe {
	enum cortexm_txml_reg_enum rf_reg;
	uint16_t rf_offset;
};

#endif /* CORTEXM_TXML_H */
