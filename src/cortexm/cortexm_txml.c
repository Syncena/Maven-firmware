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

#include "cortexm.h"
#include "cortexm_regs.h"
#include "cortexm_txml.h"

static const struct cortexm_txml_regmap cortexm_txml_regmap_core[] = {
#define	CORTEXM_TXML_CORE(n,e,r,fx)	{ n, (uint8_t)(e), (r), (fx) },
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_CORE
	{ NULL, 0, 0, 0 }
};
#define	CORTEXM_REG_NCORE	(sizeof(cortexm_txml_regmap_core) / \
				 sizeof(cortexm_txml_regmap_core[0]))

const struct cortexm_txml_regmap cortexm_txml_regmap_v6m[] = {
#define	CORTEXM_TXML_V6M(n,e,r,fx)	{ n, (uint8_t)(e), (r), (fx) },
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V6M
	{ NULL, 0, 0, 0 }
};

const struct cortexm_txml_regmap cortexm_txml_regmap_v78m[] = {
#define	CORTEXM_TXML_V78M(n,e,r,fx)	{ n, (uint8_t)(e), (r), (fx) },
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V78M
	{ NULL, 0, 0, 0 }
};

const struct cortexm_txml_regmap cortexm_txml_regmap_v8m_ns[] = {
#define	CORTEXM_TXML_V8M_NS(n,e,r,fx)	{ n, (uint8_t)(e), (r), (fx) },
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V8M_NS
	{ NULL, 0, 0, 0 }
};

const struct cortexm_txml_regmap cortexm_txml_regmap_v8m_s[] = {
#define	CORTEXM_TXML_V8M_S(n,e,r,fx)	{ n, (uint8_t)(e), (r), (fx) },
#include "cortexm_txml.inc"
#undef	CORTEXM_TXML_V8M_S
	{ NULL, 0, 0, 0 }
};

static uint8_t *
cortexm_reg_pack(uint8_t *rp, uint32_t reg, u_int bytes)
{

	/* XXX: Assumes little-endian. */
	while (bytes--) {
		*rp++ = (uint8_t)reg;
		reg >>= 8;
	}

	return rp;
}

static const uint8_t *
cortexm_reg_unpack(const uint8_t *rp, uint32_t *regp, u_int bytes)
{
	unsigned int shift = 0;
	uint32_t reg = 0;

	/* XXX: Assumes little-endian. */
	while (bytes--) {
		reg |= ((uint32_t)*rp++) << shift;
		shift += 8;
	}

	*regp = reg;

	return rp;
}

static const struct cortexm_txml_regframe *
cortexm_rtos_regframe(cortexm_t cm, const struct cortexm_txml_regmap *rm)
{

	/*
	 * This linear search is not time-critical, and avoids sparse arrays.
	 */
	for (const struct cortexm_txml_regframe *rf = cm->cm_rtos_frame;
	    rf != NULL && rf < &cm->cm_rtos_frame[cm->cm_rtos_frame_count];
	    rf++) {
		if (rf->rf_reg == rm->rm_enum)
			return rf;
	}

	return NULL;
}

static uint32_t
cortexm_register_fetch(cortexm_t cm, const struct cortexm_txml_regmap *rm,
    bool is_64)
{
	const struct cortexm_txml_regframe *rf;

	if ((rf = cortexm_rtos_regframe(cm, rm)) != NULL) {
		struct target_mem_readwrite mr;
		target_addr_t sp_addr;
		uint32_t rv;

		if (rf->rf_reg == CORTEXM_REG_SP) {
			const struct cortexm_txml_regmap *rm_xpsr;

			/*
			 * Stack pointer is special. We need to return its
			 * value *before* the exception took place (GDB gets
			 * to see the task's state right at the moment it was
			 * interrupted).
			 *
			 * This is normally:
			 *
			 *     cm->cm_rtos_frame_base + rf->rf_offset
			 *
			 * But we need to check this frame's xPSR register
			 * in order to determine if the core realigned SP on
			 * exception entry.
			 */
			sp_addr = cm->cm_rtos_frame_base + rf->rf_offset;

			/*
			 * Fetch the regframe associated with xPSR.
			 * It *must* exist.
			 */
			rm_xpsr = &cortexm_txml_regmap_core[CORTEXM_REG_XPSR];
			rf = cortexm_rtos_regframe(cm, rm_xpsr);
			assert(rf != NULL);
		} else {
			sp_addr = 0;
		}

		mr.mr_target_addr = cm->cm_rtos_frame_base + rf->rf_offset;
		if (is_64)	/* XXX: Assumes little-endian */
			mr.mr_target_addr += sizeof(uint32_t);
		mr.mr_write = false;
		mr.mr_length = sizeof(uint32_t);
		mr.mr_dest = &rv;

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE,
		    &mr) < 0) {
			printf("Failed to read frame register.\n");
			return 0;
		}

		if (sp_addr != 0) {
			/*
			 * Caller wants to know the SP value.
			 *
			 * 'rv' holds this frame's xPSR. Check bit#9 to
			 * determine if SP was realigned.
			 */
			if ((rv & (1u << 9)) != 0)
				sp_addr += 4;

			rv = (uint32_t) sp_addr;
		}

		return rv;
	}

	/*
	 * Register does not exist in the RTOS frame. Read directly
	 * from the CPU's register file.
	 */
	return cortexm_core_reg_read(cm, rm->rm_regsel + (is_64 ? 1 : 0));
}

static void
cortexm_register_write(cortexm_t cm, const struct cortexm_txml_regmap *rm,
    bool is_64, uint32_t v)
{
	const struct cortexm_txml_regframe *rf;

	if ((rf = cortexm_rtos_regframe(cm, rm)) != NULL) {
		struct target_mem_readwrite mr;

		/* Stack pointer is special. Don't permit modifications. */
		if (rf->rf_reg == CORTEXM_REG_SP)
			return;

		mr.mr_write = true;
		mr.mr_target_addr = cm->cm_rtos_frame_base + rf->rf_offset;
		if (is_64)	/* XXX: Assumes little-endian */
			mr.mr_target_addr += sizeof(uint32_t);
		mr.mr_length = sizeof(uint32_t);
		mr.mr_src = &v;

		if (target_ctl(cm->cm_target, TARGET_CTL_MEM_READWRITE,
		    &mr) < 0) {
			printf("Failed to write frame register.\n");
		}

		return;
	}

	/*
	 * Register does not exist in the RTOS frame. Write directly
	 * to the CPU's register file.
	 */
	cortexm_core_reg_write(cm, rm->rm_regsel + (is_64 ? 1 : 0), v);
}

static u_int
cortexm_read_reg_single(cortexm_t cm, uint8_t *rp,
    const struct cortexm_txml_regmap *rm)
{
	uint32_t reg, reg2 = 0;
	u_int rv;

	/* Ignore if the register doesn't exist on this target. */
	if ((cm->cm_features & rm->rm_flags) == 0)
		return 0;

	rv = (CORTEXM_XML_TYPE(rm->rm_flags) ==
	    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_INT8)) ? 1 : 4;

	if (rp != NULL) {
		/* Fetch the register's current value. */
		reg = cortexm_register_fetch(cm, rm, false);

		/* Apply the appropriate shift value. (n/a to 64-bits) */
		reg >>= CORTEXM_XML_SHIFT(rm->rm_flags) * 8u;

		/* Now apply the appropriate mask. */
		switch (CORTEXM_XML_BITS(rm->rm_flags)) {
		case CORTEXM_XML_BITS(CORTEXM_XML_BITS_1):
			reg &= 0x1;
			break;
		case CORTEXM_XML_BITS(CORTEXM_XML_BITS_2):
			reg &= 0x3u;
			break;
		case CORTEXM_XML_BITS(CORTEXM_XML_BITS_8):
			reg &= 0xffu;
			break;
		case CORTEXM_XML_BITS(CORTEXM_XML_BITS_32):
			break;
		case CORTEXM_XML_BITS(CORTEXM_XML_BITS_64):
			assert(CORTEXM_XML_TYPE(rm->rm_flags) ==
			    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_DOUBLE));
			assert((cm->cm_features & rm->rm_flags) &
			    CORTEXM_FEATURE_VFP);
			reg2 = cortexm_register_fetch(cm, rm, true);
			break;
		default:
			assert(CORTEXM_XML_BITS(rm->rm_flags) ==
			    CORTEXM_XML_BITS(CORTEXM_XML_BITS_CUSTOM));
			assert(rm->rm_enum == CORTEXM_REG_V78M_CONTROL);
			uint32_t mask = 0x3u;
			if (cm->cm_features & CORTEXM_FEATURE_VFP) {
				mask |= 0x4u;
				if (cm->cm_features & CORTEXM_FEATURE_SECURITY)
					mask |= 0x8u;
			}
			reg &= mask;
			break;
		}

		/* Pack the result */
		rp = cortexm_reg_pack(rp, reg, rv);
	}

	/* Handle the 64-bit case, VFP only. */
	if (((cm->cm_features & rm->rm_flags) & CORTEXM_FEATURE_VFP) != 0 &&
	    CORTEXM_XML_TYPE(rm->rm_flags) ==
	    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_DOUBLE)) {
		rv = 8;
		if (rp != NULL)
			rp = cortexm_reg_pack(rp, reg2, 4);
	}

	return rv;
}

static const uint8_t *
cortexm_write_reg_single(cortexm_t cm, const uint8_t *rp,
    const struct cortexm_txml_regmap *rm)
{
	uint32_t reg, reg2;

	/* Ignore if the register doesn't exist on this target. */
	if ((cm->cm_features & rm->rm_flags) == 0)
		return rp;

	/* Unpack the value */
	rp = cortexm_reg_unpack(rp, &reg, (CORTEXM_XML_TYPE(rm->rm_flags) ==
	    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_INT8)) ? 1 : 4);
	reg2 = 0;

	/* Apply the appropriate mask. */
	switch (CORTEXM_XML_BITS(rm->rm_flags)) {
	case CORTEXM_XML_BITS(CORTEXM_XML_BITS_1):
		reg &= 0x1;
		break;
	case CORTEXM_XML_BITS(CORTEXM_XML_BITS_2):
		reg &= 0x3u;
		break;
	case CORTEXM_XML_BITS(CORTEXM_XML_BITS_8):
		reg &= 0xffu;
		break;
	case CORTEXM_XML_BITS(CORTEXM_XML_BITS_32):
		break;
	case CORTEXM_XML_BITS(CORTEXM_XML_BITS_64):
		assert(CORTEXM_XML_TYPE(rm->rm_flags) ==
		    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_DOUBLE));
		assert((cm->cm_features & rm->rm_flags) & CORTEXM_FEATURE_VFP);
		rp = cortexm_reg_unpack(rp, &reg2, 4);
		break;
	default:
		assert(CORTEXM_XML_BITS(rm->rm_flags) ==
		    CORTEXM_XML_BITS(CORTEXM_XML_BITS_CUSTOM));
		assert(rm->rm_enum == CORTEXM_REG_V78M_CONTROL);
		uint32_t mask = 0x3u;
		if (cm->cm_features & CORTEXM_FEATURE_VFP) {
			mask |= 0x4u;
			if (cm->cm_features & CORTEXM_FEATURE_SECURITY)
				mask |= 0x8u;
		}
		reg &= mask;
		break;
	}

	/* Apply the appropriate shift value. (n/a to 64-bits) */
	reg <<= CORTEXM_XML_SHIFT(rm->rm_flags) * 8u;

	/* Control register needs special treatment... */
	if (rm->rm_regsel == CMSCW_DCRSR_REG_CFBP) {
		uint32_t mask = 0xffu << CORTEXM_XML_SHIFT(rm->rm_flags) * 8u;
		uint32_t cfbp = cortexm_register_fetch(cm, rm, false);
		cfbp &= ~mask;
		cfbp |= reg;
	}

	/* Update the register. */
	cortexm_register_write(cm, rm, false, reg);

	if (CORTEXM_XML_BITS(rm->rm_flags) ==
	    CORTEXM_XML_BITS(CORTEXM_XML_BITS_64)) {
		cortexm_register_write(cm, rm, true, reg2);
	}

	return rp;
}

static u_int
cortexm_read_reg_array(cortexm_t cm, uint8_t *rp,
    const struct cortexm_txml_regmap *rm)
{
	u_int rv = 0;

	while (rm->rm_name != NULL) {
		u_int bytes;

		bytes = cortexm_read_reg_single(cm, rp, rm);
		rv += bytes;
		if (rp != NULL)
			rp += bytes;
		rm++;
	}

	return rv;
}

static const uint8_t *
cortexm_write_reg_array(cortexm_t cm, const uint8_t *rp,
    const struct cortexm_txml_regmap *rm)
{

	while (rm->rm_name != NULL) {
		rp = cortexm_write_reg_single(cm, rp, rm);
		rm++;
	}

	return rp;
}

int
cortexm_txml_read_regs(cortexm_t cm, uint8_t *rp)
{
	u_int bytes, rv;

	/* Core/VFP Registers */
	rv = bytes = cortexm_read_reg_array(cm, rp, cortexm_txml_regmap_core);
	if (rp != NULL)
		rp += bytes;

	/* System Registers */
	bytes = cortexm_read_reg_array(cm, rp, cm->cm_regmap_arch);
	rv += bytes;
	if (rp != NULL)
		rp += bytes;

	/* Optional security registers */
	if (cm->cm_regmap_security != NULL) {
		bytes = cortexm_read_reg_array(cm, rp, cm->cm_regmap_security);
		rv += bytes;
		if (rp != NULL)
			rp += bytes;
	}

	return (rp != NULL && TARGET_LINK_ERROR(cm->cm_target) != 0) ? -1 :
	    (int)rv;
}

int
cortexm_txml_write_regs(cortexm_t cm, const uint8_t *rp)
{

	/* Core registers */
	rp = cortexm_write_reg_array(cm, rp, cortexm_txml_regmap_core);

	/* System Registers */
	rp = cortexm_write_reg_array(cm, rp, cm->cm_regmap_arch);

	/* System Registers: Security */
	/* Optional security registers */
	if (cm->cm_regmap_security != NULL) {
		rp = cortexm_write_reg_array(cm, rp,
		    cm->cm_regmap_security);
	}

	return (TARGET_LINK_ERROR(cm->cm_target) != 0) ? -1 : 0;
}

static const struct cortexm_txml_regmap *
cortexm_find_regmap(cortexm_t cm, u_int rn)
{
	const struct cortexm_txml_regmap *rm;

	if (rn < CORTEXM_REG_NCORE) {
		assert(cortexm_txml_regmap_core[rn].rm_enum == rn);
		return &cortexm_txml_regmap_core[rn];
	}

	for (rm = cm->cm_regmap_arch; rm->rm_name != NULL; rm++) {
		if (rm->rm_enum == rn)
			return rm;
	}

	if (cm->cm_regmap_security != NULL) {
		for (rm = cm->cm_regmap_security; rm->rm_name != NULL; rm++) {
			if (rm->rm_enum == rn)
				return rm;
		}
	}

	return NULL;
}

int
cortexm_txml_one_reg_read(cortexm_t cm, struct target_rdwr_one_reg *or)
{
	const struct cortexm_txml_regmap *rm;
	u_int rv;

	rm = cortexm_find_regmap(cm, or->or_reg);
	if (rm == NULL)
		return -1;

	rv = cortexm_read_reg_single(cm, or->or_regbuff, rm);

	if (TARGET_LINK_ERROR(cm->cm_target) != 0)
		return -1;

	return (int) rv;
}

int
cortexm_txml_one_reg_write(cortexm_t cm, struct target_rdwr_one_reg *or)
{
	const struct cortexm_txml_regmap *rm;

	rm = cortexm_find_regmap(cm, or->or_reg);
	if (rm == NULL)
		return -1;

	(void) cortexm_write_reg_single(cm, or->or_regbuff, rm);

	return TARGET_LINK_ERROR(cm->cm_target) ? -1 : 0;
}

static void
cortexm_target_xml_emit(cortexm_t cm, stringio_t s, u_int xml_class,
    const struct cortexm_txml_regmap *rm)
{
	static const uint8_t cortexm_reg_bitsize[] = {
		1u, 2u, 8u, 32u, 64u, 0u, 0u, 0u
	};
	u_int bits;

	if (CORTEXM_XML_CLASS(rm->rm_flags) != CORTEXM_XML_CLASS(xml_class))
		return;

	if ((cm->cm_features & rm->rm_flags) == 0)
		return;

	if (CORTEXM_XML_BITS(rm->rm_flags) ==
	    CORTEXM_XML_BITS(CORTEXM_XML_BITS_CUSTOM)) {
		assert(rm->rm_enum == CORTEXM_REG_V78M_CONTROL);
		bits = 2u;
		if (cm->cm_features & CORTEXM_FEATURE_VFP) {
			bits = 3u;
			if (cm->cm_features & CORTEXM_FEATURE_SECURITY)
				bits = 4u;
		}
	} else {
		bits = cortexm_reg_bitsize[CORTEXM_XML_BITS(rm->rm_flags)];
	}

	stringio_printf(s, "<reg name=\"%s\" bitsize=\"%u\" regnum=\"%u\"",
	    rm->rm_name, bits, (unsigned int)rm->rm_enum);

	if (CORTEXM_XML_TYPE(rm->rm_flags) <
	    CORTEXM_XML_TYPE(CORTEXM_XML_TYPE_NONE)) {
		static const char *type_name[] = {
			"data_ptr", "code_ptr", "int8", "ieee_double"
		};
		stringio_printf(s, " type=\"%s\"",
		    type_name[CORTEXM_XML_TYPE(rm->rm_flags)]);
	} else
	if (rm->rm_enum == CORTEXM_REG_VFP_FPSCR)
		stringio_cat(s, " group=\"float\"");

	stringio_cat(s, "/>");
}

int
cortexm_txml_generate(cortexm_t cm, stringio_t s)
{
	const struct cortexm_txml_regmap *rm;

	static const char target_xml_prologue[] =
	    "<?xml version=\"1.0\"?><!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
            "<target><architecture>arm</architecture>";
	stringio_cat(s, target_xml_prologue);

	static const char target_xml_fmt_feature_prologue[] =
	    "<feature name=\"org.gnu.gdb.arm.%s\">";
	static const char target_xml_fmt_feature_epilogue[] =
	    "</feature>";
	stringio_printf(s, target_xml_fmt_feature_prologue, "m-profile");
	for (rm = cortexm_txml_regmap_core; rm->rm_name != NULL; rm++)
		cortexm_target_xml_emit(cm, s, CORTEXM_XML_CLASS_PROFILE, rm);
	stringio_cat(s, target_xml_fmt_feature_epilogue);

	if (cm->cm_features & CORTEXM_FEATURE_VFP) {
		stringio_printf(s, target_xml_fmt_feature_prologue, "vfp");
		for (rm = cortexm_txml_regmap_core; rm->rm_name != NULL; rm++) {
			cortexm_target_xml_emit(cm, s, CORTEXM_XML_CLASS_VFP,
			    rm);
		}
		stringio_cat(s, target_xml_fmt_feature_epilogue);
	}

	stringio_printf(s, target_xml_fmt_feature_prologue, "m-system");
	for (rm = cortexm_txml_regmap_core; rm->rm_name != NULL; rm++)
		cortexm_target_xml_emit(cm, s, CORTEXM_XML_CLASS_SYSTEM, rm);

	for (rm = cm->cm_regmap_arch; rm->rm_name != NULL; rm++)
		cortexm_target_xml_emit(cm, s, CORTEXM_XML_CLASS_SYSTEM, rm);

	if (cm->cm_regmap_security != NULL) {
		for (rm = cm->cm_regmap_security; rm->rm_name != NULL; rm++) {
			cortexm_target_xml_emit(cm, s, CORTEXM_XML_CLASS_SYSTEM,
			    rm);
		}
	}

	stringio_cat(s, target_xml_fmt_feature_epilogue);

	static const char target_xml_epilogue[] =
	    "</target>";
	stringio_cat(s, target_xml_epilogue);

	return 0;
}
