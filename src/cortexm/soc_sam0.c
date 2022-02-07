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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cortexm.h"
#include "cortexm_txml.h"
#include "soc_sam0.h"
#include "microchip_dsu.h"
#include "hardware.h"
#include "flash_sam_nvmctrl.h"
#include "zone_alloc.h"

#define	SAM0_PROCESSOR_M0_PLUS		0x1u
#define	SAM0_PROCESSOR_M23		0x2u
#define	SAM0_FAMILY_D			0x0u
#define	SAM0_FAMILY_L			0x1u
#define	SAM0_FAMILY_C			0x2u
#define	SAM0_SERIES_09			0x4u
#define	SAM0_SERIES_10			0x2u
#define	SAM0_SERIES_11			0x3u
#define	SAM0_SERIES_20			0x0u
#define	SAM0_SERIES_21			0x1u
#define	SAM0_SERIES_22			0x2u

#define	SAM0_SOC_NAME_LEN		11
#define	SAM0_PART_VENDOR		"Microchip"
#define	SAM0_PART_PREFIX		"ATSAM"

#define	SAM0_SRAM_BASE			0x20000000u
#define	SAM0_LP_SRAM_BASE		0x30000000u
#define	SAM0_FLASH_BASE			0x00000000u
#define	SAM0_RWWEEP_BASE		0x00400000u
#define	SAM0_USER_BASE			0x00804000u
#define	SAM0_NVMCTRL_REGS		0x41004000u
#define	SAM0_NVMCTRL_SEC_REGS		0x41005000u

struct sam0_id {
	const char si_soc_name[SAM0_SOC_NAME_LEN];
	uint8_t si_devsel;
	uint8_t si_sram_size;
	uint8_t si_flags;
	const uint64_t *si_nvm_default;
};
#define	SAM0_FLAGS_HAS_RWWEEP		(1u << 0)
#define	SAM0_FLAGS_BOOTROM		(1u << 1)
#define	SAM0_FLAGS_TRUSTZONE		(1u << 2)

#define	SAM0_NVM_BOOTPROT(x)		(((uint64_t)(x) & 0x07ull) << 0)
#define	SAM0_NVM_RESVD3(x)		(((uint64_t)(x) & 0x01ull) << 3)
#define	SAM0_NVM_EEPROM(x)		(((uint64_t)(x) & 0x07ull) << 4)
#define	SAM0_NVM_RESVD7(x)		(((uint64_t)(x) & 0x01ull) << 7)
#define	SAM0_NVM_BOD_LEVEL(x)		(((uint64_t)(x) & 0x3full) << 8)
#define	SAM0_NVM_BOD_ENABLE(x)		(((uint64_t)(x) & 0x01ull) << 14)
#define	SAM0_NVM_BOD_ACTION(x)		(((uint64_t)(x) & 0x03ull) << 15)
#define	SAM0_NVM_RESVD42(x)		(((uint64_t)(x) & 0x3full) << 42)
#define	SAM0_NVM_WDT_LOCK(x)		(((uint64_t)(x) & 0xffffull) << 48)

/* SAMDx */
#define	SAM0D_NVM_RESVD17(x)		(((uint64_t)(x) & 0xffull) << 17)
#define	SAM0D_NVM_WDT_ENABLE(x)		(((uint64_t)(x) & 0x01ull) << 25)
#define	SAM0D_NVM_WDT_ALWAYS_ON(x)	(((uint64_t)(x) & 0x01ull) << 26)
#define	SAM0D_NVM_WDT_PERIOD(x)		(((uint64_t)(x) & 0x0full) << 27)
#define	SAM0D_NVM_WDT_WINDOW(x)		(((uint64_t)(x) & 0x0full) << 31)
#define	SAM0D_NVM_WDT_EWOFFSET(x)	(((uint64_t)(x) & 0x0full) << 35)
#define	SAM0D_NVM_WDT_WEN(x)		(((uint64_t)(x) & 0x01ull) << 39)
#define	SAM0D_NVM_BOD_HYSTERESIS(x)	(((uint64_t)(x) & 0x01ull) << 40)
#define	SAM0D_NVM_RESVD41(x)		(((uint64_t)(x) & 0x01ull) << 41)
#define	SAM0D_NVM_RESVD_MASK		(SAM0_NVM_RESVD3(0x1) | \
					 SAM0_NVM_RESVD7(0x1) | \
					 SAM0D_NVM_RESVD17(0xff) | \
					 SAM0D_NVM_RESVD41(0x1) | \
					 SAM0_NVM_RESVD42(0x3f))

/* SAML2x and SAMC2x shift a bunch of stuff up by 1 bit. */
#define	SAM0LC_NVM_RESVD17(x)		(((uint64_t)(x) & 0x1ffull) << 17)
#define	SAM0LC_NVM_WDT_ENABLE(x)	(((uint64_t)(x) & 0x01ull) << 26)
#define	SAM0LC_NVM_WDT_ALWAYS_ON(x)	(((uint64_t)(x) & 0x01ull) << 27)
#define	SAM0LC_NVM_WDT_PERIOD(x)	(((uint64_t)(x) & 0x0full) << 28)
#define	SAM0LC_NVM_WDT_WINDOW(x)	(((uint64_t)(x) & 0x0full) << 32)
#define	SAM0LC_NVM_WDT_EWOFFSET(x)	(((uint64_t)(x) & 0x0full) << 36)
#define	SAM0LC_NVM_WDT_WEN(x)		(((uint64_t)(x) & 0x01ull) << 40)
#define	SAM0LC_NVM_BOD_HYSTERESIS(x)	(((uint64_t)(x) & 0x01ull) << 41)
#define	SAM0L_NVM_RESVD_MASK		(SAM0_NVM_RESVD3(0x1) | \
					 SAM0_NVM_RESVD7(0x1) | \
					 SAM0LC_NVM_RESVD17(0x1ff) | \
					 SAM0_NVM_RESVD42(0x3f))

/* SAMC2x has a minor change in the final reserved fields. */
#define	SAM0C_NVM_RESVD42(x)		(((uint64_t)(x) & 0x01ull) << 42)
#define	SAM0C_NVM_RESVD43(x)		(((uint64_t)(x) & 0x1full) << 43)
#define	SAM0C_NVM_RESVD_MASK		(SAM0_NVM_RESVD3(0x1) | \
					 SAM0_NVM_RESVD7(0x1) | \
					 SAM0LC_NVM_RESVD17(0x1ff) | \
					 SAM0C_NVM_RESVD42(0x01) | \
					 SAM0C_NVM_RESVD43(0x1f))

static const uint64_t samd09_10_11_nvm_default =
	       SAM0_NVM_BOOTPROT(0x07)	|
		 SAM0_NVM_EEPROM(0x07)	|
	      SAM0_NVM_BOD_LEVEL(0x07)	|
	     SAM0_NVM_BOD_ENABLE(0x01)	|
	     SAM0_NVM_BOD_ACTION(0x01)	|
	    SAM0D_NVM_WDT_ENABLE(0x00)	|
	 SAM0D_NVM_WDT_ALWAYS_ON(0x00)	|
	    SAM0D_NVM_WDT_PERIOD(0x0b)	|
	    SAM0D_NVM_WDT_WINDOW(0x0b)	|
	  SAM0D_NVM_WDT_EWOFFSET(0x0b)	|
	       SAM0D_NVM_WDT_WEN(0x00)	|
	SAM0D_NVM_BOD_HYSTERESIS(0x00)	|
	       SAM0_NVM_WDT_LOCK(0xffff);
static const struct sam0_id samd09_ids[] = {
	{"D09D14A-M",  0x0, 4, 0, &samd09_10_11_nvm_default},
	{"D09C13A-SS", 0x7, 4, 0, &samd09_10_11_nvm_default},
};

static const struct sam0_id samd10_ids[] = {
	{"D10D14A-M", 0x00, 4, 0, &samd09_10_11_nvm_default},
	{"D10D13A-M", 0x01, 4, 0, &samd09_10_11_nvm_default},
	{"D10D12A-M", 0x02, 4, 0, &samd09_10_11_nvm_default},
	{"D10D14A-S", 0x03, 4, 0, &samd09_10_11_nvm_default},
	{"D10D13A-S", 0x04, 4, 0, &samd09_10_11_nvm_default},
	{"D10D12A-S", 0x05, 4, 0, &samd09_10_11_nvm_default},
	{"D10D14A",   0x06, 4, 0, &samd09_10_11_nvm_default},
	{"D10D13A",   0x07, 4, 0, &samd09_10_11_nvm_default},
	{"D10D12A",   0x08, 4, 0, &samd09_10_11_nvm_default},
	{"D10D14A-U", 0x09, 4, 0, &samd09_10_11_nvm_default},
};

static const struct sam0_id samd11_ids[] = {
	{"D11D14A-M",  0x00, 4, 0, &samd09_10_11_nvm_default},
	{"D11D13A-M",  0x01, 4, 0, &samd09_10_11_nvm_default},
	{"D11D12A-M",  0x02, 4, 0, &samd09_10_11_nvm_default},
	{"D11D14A-SS", 0x03, 4, 0, &samd09_10_11_nvm_default},
	{"D11D13A-SS", 0x04, 4, 0, &samd09_10_11_nvm_default},
	{"D11D12A-SS", 0x05, 4, 0, &samd09_10_11_nvm_default},
	{"D11C14A",    0x06, 4, 0, &samd09_10_11_nvm_default},
	{"D11C13A",    0x07, 4, 0, &samd09_10_11_nvm_default},
	{"D11C12A",    0x08, 4, 0, &samd09_10_11_nvm_default},
	{"D11D14A-U",  0x09, 4, 0, &samd09_10_11_nvm_default},
};

static const uint64_t samd2x_nvm_default =
	       SAM0_NVM_BOOTPROT(0x07)	|
		 SAM0_NVM_EEPROM(0x07)	|
	      SAM0_NVM_BOD_LEVEL(0x07)	| /* XXX D21 AEC-Q100 value is 0x22 */
	     SAM0_NVM_BOD_ENABLE(0x01)	|
	     SAM0_NVM_BOD_ACTION(0x01)	|
	    SAM0D_NVM_WDT_ENABLE(0x00)	|
	 SAM0D_NVM_WDT_ALWAYS_ON(0x00)	|
	    SAM0D_NVM_WDT_PERIOD(0x0b)	|
	    SAM0D_NVM_WDT_WINDOW(0x05)	|
	  SAM0D_NVM_WDT_EWOFFSET(0x0b)	|
	       SAM0D_NVM_WDT_WEN(0x00)	|
	SAM0D_NVM_BOD_HYSTERESIS(0x01)	|
	       SAM0_NVM_WDT_LOCK(0xffff);
static const uint64_t samd2x_wlcsp_nvm_default =
	       SAM0_NVM_BOOTPROT(0x03)	|
	         SAM0_NVM_EEPROM(0x07)	|
	      SAM0_NVM_BOD_LEVEL(0x07)	| /* XXX D21 AEC-Q100 value is 0x22 */
	     SAM0_NVM_BOD_ENABLE(0x01)	|
	     SAM0_NVM_BOD_ACTION(0x01)	|
	    SAM0D_NVM_WDT_ENABLE(0x00)	|
	 SAM0D_NVM_WDT_ALWAYS_ON(0x00)	|
	    SAM0D_NVM_WDT_PERIOD(0x0b)	|
	    SAM0D_NVM_WDT_WINDOW(0x05)	|
	  SAM0D_NVM_WDT_EWOFFSET(0x0b)	|
	       SAM0D_NVM_WDT_WEN(0x00)	|
	SAM0D_NVM_BOD_HYSTERESIS(0x01)	|
	       SAM0_NVM_WDT_LOCK(0xffff);
static const struct sam0_id samd20_ids[] = {
	{"D20J18A", 0x00, 32, 0, &samd2x_nvm_default},
	{"D20J17A", 0x01, 16, 0, &samd2x_nvm_default},
	{"D20J16A", 0x02,  8, 0, &samd2x_nvm_default},
	{"D20J15A", 0x03,  4, 0, &samd2x_nvm_default},
	{"D20J14A", 0x04,  2, 0, &samd2x_nvm_default},

	{"D20G18A", 0x05, 32, 0, &samd2x_nvm_default},
	{"D20G17A", 0x06, 16, 0, &samd2x_nvm_default},
	{"D20G16A", 0x07,  8, 0, &samd2x_nvm_default},
	{"D20G15A", 0x08,  4, 0, &samd2x_nvm_default},
	{"D20G14A", 0x09,  2, 0, &samd2x_nvm_default},

	{"D20E18A", 0x0a, 32, 0, &samd2x_nvm_default},
	{"D20E17A", 0x0b, 16, 0, &samd2x_nvm_default},
	{"D20E16A", 0x0c,  8, 0, &samd2x_nvm_default},
	{"D20E15A", 0x0d,  4, 0, &samd2x_nvm_default},
	{"D20E14A", 0x0e,  2, 0, &samd2x_nvm_default},

	{"D20G18U", 0x10, 32, 0, &samd2x_wlcsp_nvm_default},
	{"D20G17U", 0x11, 16, 0, &samd2x_wlcsp_nvm_default},
};

static const struct sam0_id samd21_ids[] = {
	{"D21J18A", 0x00, 32, 0, &samd2x_nvm_default},
	{"D21J17A", 0x01, 16, 0, &samd2x_nvm_default},
	{"D21J16A", 0x02,  8, 0, &samd2x_nvm_default},
	{"D21J15A", 0x03,  4, 0, &samd2x_nvm_default},
	{"D21J14A", 0x04,  2, 0, &samd2x_nvm_default},	/* Obsolete? */
	{"D21G18A", 0x05, 32, 0, &samd2x_nvm_default},
	{"D21G17A", 0x06, 16, 0, &samd2x_nvm_default},
	{"D21G16A", 0x07,  8, 0, &samd2x_nvm_default},
	{"D21G15A", 0x08,  4, 0, &samd2x_nvm_default},
	{"D21G14A", 0x09,  2, 0, &samd2x_nvm_default},	/* Obsolete? */
	{"D21E18A", 0x0a, 32, 0, &samd2x_nvm_default},
	{"D21E17A", 0x0b, 16, 0, &samd2x_nvm_default},
	{"D21E16A", 0x0c,  8, 0, &samd2x_nvm_default},
	{"D21E15A", 0x0d,  4, 0, &samd2x_nvm_default},
	{"D21E14A", 0x0e,  2, 0, &samd2x_nvm_default},	/* Obsolete? */
	{"D21G18AU",0x0f, 32, 0, &samd2x_wlcsp_nvm_default},
	{"D21G17AU",0x10, 16, 0, &samd2x_wlcsp_nvm_default},
	{"D21J16B", 0x20,  8, 0, &samd2x_nvm_default},
	{"D21J15B", 0x21,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21G16B", 0x23,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21G15B", 0x24,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E16B", 0x26,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E15B", 0x27,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E16L", 0x3e,  8, 0, &samd2x_nvm_default},
	{"D21E15L", 0x3f,  4, 0, &samd2x_nvm_default},
	{"D21E16BU",0x55,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_wlcsp_nvm_default},
	{"D21E15BU",0x56,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_wlcsp_nvm_default},
	{"D21G16L", 0x57,  8, 0, &samd2x_wlcsp_nvm_default},
	{"D21E16CU",0x62,  8, 0, &samd2x_wlcsp_nvm_default},
	{"D21E15CU",0x63,  4, 0, &samd2x_wlcsp_nvm_default},
	{"D21G16L", 0x87,  8, 0, &samd2x_wlcsp_nvm_default},/*DEVSEL confusion*/
	{"D21J17D", 0x92, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21G17D", 0x93, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E17D", 0x94, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E17DU",0x95, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_wlcsp_nvm_default},
	{"D21G17L", 0x96, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"D21E17L", 0x97, 16, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},

	/* SAMR21 range seems to fit in with the SAMD21 range. */
	{"R21G19A", 0x18, 32, 0, &samd2x_nvm_default},
	{"R21G18A", 0x19, 32, 0, &samd2x_nvm_default},
	{"R21G17A", 0x1a, 32, 0, &samd2x_nvm_default},
	{"R21G16A", 0x1b, 16, 0, &samd2x_nvm_default},
	{"R21E18A", 0x1c, 32, 0, &samd2x_nvm_default},
	{"R21E17A", 0x1d, 32, 0, &samd2x_nvm_default},
	{"R21E16A", 0x1e, 16, 0, &samd2x_nvm_default},

	/* SAMDA1 range also fits here. */
	{"DA1J16A", 0x29,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1J15A", 0x2a,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1J14A", 0x2b,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G16A", 0x2c,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G15A", 0x2d,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G14A", 0x2e,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E16A", 0x2f,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E15A", 0x30,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E14A", 0x31,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},

	{"DA1J16B", 0x64,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1J15B", 0x65,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1J14B", 0x66,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G16B", 0x67,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G15B", 0x68,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1G14B", 0x69,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E16B", 0x6a,  8, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E15B", 0x6b,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
	{"DA1E14B", 0x6c,  4, SAM0_FLAGS_HAS_RWWEEP, &samd2x_nvm_default},
};

static const uint64_t saml2x_nvm_default =
		SAM0_NVM_BOOTPROT(0x07)	|
		  SAM0_NVM_EEPROM(0x07)	|
	       SAM0_NVM_BOD_LEVEL(0x06)	|
	      SAM0_NVM_BOD_ENABLE(0x00)	| /* Note: BOD Disable */
	      SAM0_NVM_BOD_ACTION(0x01)	|
	    SAM0LC_NVM_WDT_ENABLE(0x00)	|
	 SAM0LC_NVM_WDT_ALWAYS_ON(0x00)	|
	    SAM0LC_NVM_WDT_PERIOD(0x0b)	|
	    SAM0LC_NVM_WDT_WINDOW(0x0b)	|
	  SAM0LC_NVM_WDT_EWOFFSET(0x0b)	|
	       SAM0LC_NVM_WDT_WEN(0x00)	|
	SAM0LC_NVM_BOD_HYSTERESIS(0x00)	|
	      SAM0_NVM_WDT_LOCK(0xffff);
static const struct sam0_id saml21_ids[] = {
	{"L21J18A", 0x00, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21J17A", 0x01, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21J16A", 0x02,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G18A", 0x05, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G17A", 0x06, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G16A", 0x07,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E18A", 0x0a, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E17A", 0x0b, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E16A", 0x0c,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E15A", 0x0d,  4, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},

	{"L21J18B", 0x0f, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21J17B", 0x10, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21J16B", 0x11,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G18B", 0x14, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G17B", 0x15, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21G16B", 0x16,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E18B", 0x19, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E17B", 0x1a, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E16B", 0x1b,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L21E15B", 0x1c,  4, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},

	/* SAMR30 ranges seems to fit in with the SAML21 range. */
	{"R30G18A", 0x1e, 32, 0, &saml2x_nvm_default},
	{"R30E18A", 0x1f, 32, 0, &saml2x_nvm_default},

	/* Ditto SAMR34/35 */
	{"R34J18",  0x28, 32, 0, &saml2x_nvm_default},
};

static const struct sam0_id saml22_ids[] = {
	{"L22N18A", 0x00, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22N17A", 0x01, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22N16A", 0x02,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22J18A", 0x05, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22J17A", 0x06, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22J16A", 0x07,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22G18A", 0x0a, 32, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22G17A", 0x0b, 16, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
	{"L22G16A", 0x0c,  8, SAM0_FLAGS_HAS_RWWEEP, &saml2x_nvm_default},
};

static const uint64_t samc2x_nvm_default =
		SAM0_NVM_BOOTPROT(0x07)	|
		  SAM0_NVM_EEPROM(0x07)	|
	       SAM0_NVM_BOD_LEVEL(0x08)	|
	      SAM0_NVM_BOD_ENABLE(0x00)	| /* Note: BOD Disable */
	      SAM0_NVM_BOD_ACTION(0x01)	|
	    SAM0LC_NVM_WDT_ENABLE(0x00)	|
	 SAM0LC_NVM_WDT_ALWAYS_ON(0x00)	|
	    SAM0LC_NVM_WDT_PERIOD(0x0b)	|
	    SAM0LC_NVM_WDT_WINDOW(0x0b)	|
	  SAM0LC_NVM_WDT_EWOFFSET(0x0b)	|
	       SAM0LC_NVM_WDT_WEN(0x00)	|
	SAM0LC_NVM_BOD_HYSTERESIS(0x00)	|
	        SAM0_NVM_WDT_LOCK(0xffff);
static const struct sam0_id samc20_ids[] = {
	{"C20J18A", 0x00, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20J17A", 0x01, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20J16A", 0x02,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20J15A", 0x03,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},

	{"C20G18A", 0x05, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20G17A", 0x06, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20G16A", 0x07,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20G15A", 0x08,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},

	{"C20E18A", 0x0a, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20E17A", 0x0b, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20E16A", 0x0c,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20E15A", 0x0d,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},

	{"C20N18A", 0x20, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C20N17A", 0x21, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
};

static const struct sam0_id samc21_ids[] = {
	{"C21J18A", 0x0, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21J17A", 0x1, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21J16A", 0x2,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21J15A", 0x3,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},

	{"C21G18A", 0x5, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21G17A", 0x6, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21G16A", 0x7,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21G15A", 0x8,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},

	{"C21E18A", 0xa, 32, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21E17A", 0xb, 16, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21E16A", 0xc,  8, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
	{"C21E15A", 0xd,  4, SAM0_FLAGS_HAS_RWWEEP, &samc2x_nvm_default},
};

struct sam0_soc {
	uint8_t soc_family;
	uint8_t soc_series;
	uint8_t soc_part_count;
	const struct sam0_id *soc_parts;
	const uint64_t *soc_resvd_mask;
};

#define	SAM0_SOC(f,s,p,m)	{.soc_family = (f),			     \
				 .soc_series = (s),			     \
				 .soc_part_count = sizeof(p) / sizeof((p)[0]),\
				 .soc_parts = (p),			     \
				 .soc_resvd_mask = (m)}

static const uint64_t sam0c_nvm_resvd_mask = SAM0C_NVM_RESVD_MASK;
static const uint64_t sam0d_nvm_resvd_mask = SAM0D_NVM_RESVD_MASK;
static const uint64_t sam0l_nvm_resvd_mask = SAM0L_NVM_RESVD_MASK;

static const struct sam0_soc sam0_socs_m0plus[] = {
  SAM0_SOC(SAM0_FAMILY_C, SAM0_SERIES_20, samc20_ids, &sam0c_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_C, SAM0_SERIES_21, samc21_ids, &sam0c_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_D, SAM0_SERIES_09, samd09_ids, &sam0d_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_D, SAM0_SERIES_10, samd10_ids, &sam0d_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_D, SAM0_SERIES_11, samd11_ids, &sam0d_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_D, SAM0_SERIES_20, samd20_ids, &sam0d_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_D, SAM0_SERIES_21, samd21_ids, &sam0d_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_L, SAM0_SERIES_21, saml21_ids, &sam0l_nvm_resvd_mask),
  SAM0_SOC(SAM0_FAMILY_L, SAM0_SERIES_22, saml22_ids, &sam0l_nvm_resvd_mask),
};
#define	SAM0_NSOCS_M0PLUS	(sizeof(sam0_socs_m0plus) / \
				 sizeof(sam0_socs_m0plus[0]))

/* SAML10/SAML11 do things differently. */
#define	SAML10_NVM_RESVD0(x)		(((uint64_t)(x) & 0x07ull) << 0)
#define	SAML11_NVM_SULCK(x)		(((uint64_t)(x) & 0x07ull) << 0)
#define	SAML1x_NVM_NSULCK(x)		(((uint64_t)(x) & 0x07ull) << 3)
#define	SAML1x_NVM_RESVD6(x)		(((uint64_t)(x) & 0x01ull) << 6)
#define	SAML1x_NVM_BOD33_LEVEL(x)	(((uint64_t)(x) & 0x3full) << 7)
#define	SAML1x_NVM_BOD33_DIS(x)		(((uint64_t)(x) & 0x01ull) << 13)
#define	SAML1x_NVM_BOD33_ACTION(x)	(((uint64_t)(x) & 0x03ull) << 14)
#define	SAML1x_NVM_BOD12_CALIB(x)	(((uint64_t)(x) & 0x1ffull) << 16)
#define	SAML1x_NVM_WDT_RUNSTDBY(x)	(((uint64_t)(x) & 0x01ull) << 25)
#define	SAML1x_NVM_WDT_ENABLE(x)	(((uint64_t)(x) & 0x01ull) << 26)
#define	SAML1x_NVM_WDT_ALWAYSON(x)	(((uint64_t)(x) & 0x01ull) << 27)
#define	SAML1x_NVM_WDT_PER(x)		(((uint64_t)(x) & 0x0full) << 28)
#define	SAML1x_NVM_WDT_WINDOW(x)	(((uint64_t)(x) & 0x0full) << 32)
#define	SAML1x_NVM_WDT_EWOFFSET(x)	(((uint64_t)(x) & 0x0full) << 36)
#define	SAML1x_NVM_WDT_WEN(x)		(((uint64_t)(x) & 0x01ull) << 40)
#define	SAML1x_NVM_BOD33_HYST(x)	(((uint64_t)(x) & 0x01ull) << 41)
#define	SAML10_NVM_RESVD42(x)		(((uint64_t)(x) & 0x3fffffull) << 42)
#define	SAML11_NVM_RESVD42(x)		(((uint64_t)(x) & 0x01ull) << 42)
#define	SAML11_NVM_RXN(x)		(((uint64_t)(x) & 0x01ull) << 43)
#define	SAML11_NVM_DXN(x)		(((uint64_t)(x) & 0x01ull) << 44)
#define	SAML11_NVM_RESVD44(x)		(((uint64_t)(x) & 0x7ffffull) << 45)
#define	SAML11_NVM_AS(x)		(((uint64_t)(x) & 0xffull) << 0)
#define	SAML11_NVM_ANSC(x)		(((uint64_t)(x) & 0x3full) << 8)
#define	SAML11_NVM_RESVD78(x)		(((uint64_t)(x) & 0x03ull) << 14)
#define	SAML11_NVM_DS(x)		(((uint64_t)(x) & 0x0full) << 16)
#define	SAML11_NVM_RESVD84(x)		(((uint64_t)(x) & 0x0full) << 20)
#define	SAML11_NVM_RS(x)		(((uint64_t)(x) & 0x7full) << 24)
#define	SAML11_NVM_RESVD95(x)		(((uint64_t)(x) & 0x01ull) << 31)
#define	SAML11_NVM_URWEN(x)		(((uint64_t)(x) & 0x01ull) << 32)
#define	SAML11_NVM_RESVD97(x)		(((uint64_t)(x) & 0x7fffffffull) << 33)
#define	SAML11_NVM_NONSECA(x)		(((uint64_t)(x) & 0xffffffffull) << 0)
#define	SAML11_NVM_NONSECB(x)		(((uint64_t)(x) & 0xffffffffull) << 32)
#define	SAML11_NVM_NONSECC(x)		(((uint64_t)(x) & 0xffffffffull) << 0)
#define	SAML11_NVM_USERCRC(x)		(((uint64_t)(x) & 0xffffffffull) << 32)

#define	SAML10_NVM_RESVD_MASK0		(SAML10_NVM_RESVD0(0x7) | \
					 SAML1x_NVM_RESVD6(0x1) | \
					 SAML1x_NVM_BOD12_CALIB(0x1ff) | \
					 SAML10_NVM_RESVD42(0xfffffc))
#define	SAML10_NVM_RESVD_MASK1		0xffffffffffffffffull
#define	SAML10_NVM_RESVD_MASK2		0xffffffffffffffffull
#define	SAML10_NVM_RESVD_MASK3		0xffffffffffffffffull
#define	SAML11_NVM_RESVD_MASK0		(SAML1x_NVM_RESVD6(0x1) | \
					 SAML1x_NVM_BOD12_CALIB(0x1ff) | \
					 SAML11_NVM_RESVD42(0x1) | \
					 SAML11_NVM_RESVD44(0x7ffff))
#define	SAML11_NVM_RESVD_MASK1		(SAML11_NVM_RESVD78(0x3) | \
					 SAML11_NVM_RESVD84(0xf) | \
					 SAML11_NVM_RESVD95(0x1) | \
					 SAML11_NVM_RESVD97(0x7fffffff))
#define	SAML11_NVM_RESVD_MASK2		0xffffffffffffffffull
#define	SAML11_NVM_RESVD_MASK3		0xffffffffffffffffull


static const uint64_t saml10_nvm_default =
		SAML1x_NVM_NSULCK(0x07)		|
	       SAML1x_NVM_BOD33_LEVEL(0x06)	|
	      SAML1x_NVM_BOD33_DIS(0x00)	|
	      SAML1x_NVM_BOD33_ACTION(0x01)	|
	    SAML1x_NVM_WDT_RUNSTDBY(0x00)	|
		SAML1x_NVM_WDT_ENABLE(0x00)	|
		SAML1x_NVM_WDT_ALWAYSON(0x00)	|
		SAML1x_NVM_WDT_PER(0x0b)	|
		SAML1x_NVM_WDT_WINDOW(0x0b)	|
		SAML1x_NVM_WDT_EWOFFSET(0x0b)	|
		SAML1x_NVM_WDT_WEN(0x00)	|
		SAML1x_NVM_BOD33_HYST(0x00);
static const struct sam0_id saml10_ids[] = {
	{"L10E16A", 0x0, 16, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
	{"L10E15A", 0x1,  8, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
	{"L10E14A", 0x2,  4, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
	{"L10D16A", 0x3, 16, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
	{"L10D15A", 0x4,  8, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
	{"L10D14A", 0x5,  4, SAM0_FLAGS_BOOTROM, &saml10_nvm_default},
};

static const uint64_t saml11_nvm_default[] = {
		SAML11_NVM_SULCK(0x07)		|
		SAML1x_NVM_NSULCK(0x07)		|
	       SAML1x_NVM_BOD33_LEVEL(0x06)	|
	      SAML1x_NVM_BOD33_DIS(0x00)	|
	      SAML1x_NVM_BOD33_ACTION(0x01)	|
	    SAML1x_NVM_WDT_RUNSTDBY(0x00)	|
		SAML1x_NVM_WDT_ENABLE(0x00)	|
		SAML1x_NVM_WDT_ALWAYSON(0x00)	|
		SAML1x_NVM_WDT_PER(0x0b)	|
		SAML1x_NVM_WDT_WINDOW(0x0b)	|
		SAML1x_NVM_WDT_EWOFFSET(0x0b)	|
		SAML1x_NVM_WDT_WEN(0x00)	|
		SAML1x_NVM_BOD33_HYST(0x00)	|
		SAML11_NVM_RXN(0x01)		|
		SAML11_NVM_DXN(0x01),

		SAML11_NVM_AS(0xff)		|
		SAML11_NVM_ANSC(0x00)		|
		SAML11_NVM_DS(0x08)		|
		SAML11_NVM_RS(0x7f)		|
		SAML11_NVM_URWEN(0x01),

		SAML11_NVM_NONSECA(0x00)	|
		SAML11_NVM_NONSECB(0x00),

		SAML11_NVM_NONSECC(0x00)	|
		SAML11_NVM_USERCRC(0x8433651e)
};
static const struct sam0_id saml11_ids[] = {
	{"L11E16A", 0x0, 16, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
	{"L11E15A", 0x1,  8, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
	{"L11E14A", 0x2,  8, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
	{"L11D16A", 0x3, 16, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
	{"L11D15A", 0x4,  8, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
	{"L11D14A", 0x5,  8, SAM0_FLAGS_BOOTROM | SAM0_FLAGS_TRUSTZONE,
	 saml11_nvm_default},
};

static const uint64_t saml10_nvm_resvd_mask[] = {
	SAML10_NVM_RESVD_MASK0, SAML10_NVM_RESVD_MASK1,
	SAML10_NVM_RESVD_MASK2, SAML10_NVM_RESVD_MASK3
};
static const uint64_t saml11_nvm_resvd_mask[] = {
	SAML11_NVM_RESVD_MASK0, SAML11_NVM_RESVD_MASK1,
	SAML11_NVM_RESVD_MASK2, SAML11_NVM_RESVD_MASK3
};

static const struct sam0_soc sam0_socs_m23[] = {
	SAM0_SOC(SAM0_FAMILY_L, SAM0_SERIES_09, saml10_ids,
		 saml10_nvm_resvd_mask),
	SAM0_SOC(SAM0_FAMILY_L, SAM0_SERIES_11, saml11_ids,
		saml11_nvm_resvd_mask),
};
#define	SAM0_NSOCS_M23		(sizeof(sam0_socs_m23) / \
				 sizeof(sam0_socs_m23[0]))

struct sam0_state {
	void *ss_nvmctrl;
	void *ss_dsu;
	uint32_t ss_did;
	uint32_t ss_flash_type;
	uint32_t ss_dsu_flags;
	uint32_t ss_user_size;
	const uint64_t *ss_resvd_mask;
	const struct sam0_id *ss_si;
	microchip_dsu_protection_t ss_prot;
};

static int
soc_sam0_ctl(cortexm_t cm, uint32_t *cmd, void *arg)
{
	struct sam0_state *ss = cm->cm_soc;
	int rv = 0;


	switch (*cmd) {
	case TARGET_CTL_PROTECTION_QUERY:
	case TARGET_CTL_UNLOCK:
		/*
		 * DSU handles TARGET_CTL_PROTECTION_QUERY and TARGET_CTL_UNLOCK
		 */
		rv = microchip_dsu_ctl(ss->ss_dsu, cmd, arg);
		break;

	case TARGET_CTL_PROTECTION_SET:
		*cmd = TARGET_CTL_NOP;
		if (ss->ss_flash_type == FLASH_SAM_NVMCTRL_TYPE_1) {
			rv = flash_sam_nvmctrl1_prot_set(ss->ss_nvmctrl, arg,
			    ss->ss_prot);
		} else {
			rv = flash_sam_nvmctrl2_prot_set(ss->ss_nvmctrl, arg,
			    ss->ss_prot);
		}
		break;

	default:
		break;
	}

	return rv;
}

static void
soc_sam0_free(cortexm_t cm)
{
	struct sam0_state *ss = cm->cm_soc;

	microchip_dsu_done(ss->ss_dsu);
	cm->cm_soc = NULL;
	zone_free(ss);
}

static const struct sam0_id *
soc_sam0_match(uint32_t did, uint16_t *flash_type, uint32_t *user_size,
    const uint64_t **resvd_mask, const struct sam0_soc **ssp,
    uint32_t *dsu_flags)
{
	const struct sam0_soc *soc, *esoc;
	const struct sam0_id *si;
	uint32_t usize;
	uint16_t ft;

	switch (MCP_DSU_DID_PROCESSOR(did)) {
	case SAM0_PROCESSOR_M0_PLUS:
		soc = sam0_socs_m0plus;
		esoc = &sam0_socs_m0plus[SAM0_NSOCS_M0PLUS];
		ft = FLASH_SAM_NVMCTRL_TYPE_1;
		*dsu_flags = MCP_DSU_FLAGS_TYPE_M0;
		usize = 1;
		break;

	case SAM0_PROCESSOR_M23:
		soc = sam0_socs_m23;
		esoc = &sam0_socs_m23[SAM0_NSOCS_M23];
		ft = FLASH_SAM_NVMCTRL_TYPE_2;
		*dsu_flags = 0;
		usize = 4;
		break;

	default:
		return NULL;
	}

	/* Find the parts list for this family/series */
	while (soc < esoc) {
		if (MCP_DSU_DID_FAMILY(did) == soc->soc_family &&
		    MCP_DSU_DID_SERIES(did) == soc->soc_series) {
			break;
		}
		soc++;
	}

	if (soc == esoc)
		return NULL;

	/* Scan the parts list for a matching devsel ID */
	for (si = soc->soc_parts;
	    si < &soc->soc_parts[soc->soc_part_count]; si++) {
		if (MCP_DSU_DID_DEVSEL(did) == si->si_devsel)
			break;
	}

	if (si == &soc->soc_parts[soc->soc_part_count])
		return NULL;

	/* Found one */
	if (ssp != NULL)
		*ssp = soc;

	if (flash_type != NULL)
		*flash_type = ft;

	if (user_size != NULL)
		*user_size = usize;

	if (resvd_mask != NULL)
		*resvd_mask = soc->soc_resvd_mask;

	if (si->si_flags & SAM0_FLAGS_BOOTROM)
		*dsu_flags |= MCP_DSU_FLAGS_TYPE_ROM;

	if (si->si_flags & SAM0_FLAGS_TRUSTZONE)
		*dsu_flags |= MCP_DSU_FLAGS_TRUSTZONE;

	return si;
}

static void
soc_sam0_register(uint32_t did, const struct sam0_id *si)
{
	char soc_name[TARGET_NAME_LEN_MAX];
	char did_string[TARGET_NAME_LEN_MAX];

	snprintf(soc_name, sizeof(soc_name), SAM0_PART_PREFIX "%s",
	    si->si_soc_name);
	snprintf(did_string, sizeof(did_string), "DID 0x%08" PRIx32, did);
	target_register_name(SAM0_PART_VENDOR, soc_name, did_string, NULL);
}

static int
soc_sam0_user_erase(void *cookie, uint8_t *nvm, uint32_t len)
{
	struct sam0_state *ss = cookie;
	unsigned int i;
	uint64_t copy;

	/*
	 * 'nvm' points to a copy of the existing User Page.
	 * We must erase it back to defaults, while preserving the
	 * 'reserved' fields.
	 */

	assert(len == ss->ss_user_size);

	for (i = 0; i < (len / sizeof(uint64_t)); i++) {
		/* Grab a local/aligned copy of the existing uint64_t. */
		memcpy(&copy, nvm, sizeof(copy));

		/* Keep only this SoC's reserved bitfields. */
		copy &= ss->ss_resvd_mask[i];

		/* Merge with the default values for this SoC. */
		copy |= ss->ss_si->si_nvm_default[i] & ~(ss->ss_resvd_mask[i]);

		/* Update the NVMCTRL driver's copy. */
		memcpy(nvm, &copy, len);

		nvm += sizeof(uint64_t);
	}

	return TARGET_FLASH_RESULT_OK;
}

static int
soc_sam0_user_check(void *cookie, uint8_t *nvm, uint32_t pagenum, uint8_t *page)
{
	struct sam0_state *ss = cookie;
	uint64_t resvd, new_bits;
	unsigned int i;

	/*
	 * 'nvm' points to the relevant page within the copy of the existing
	 * User Page. Chances are, we erased it earlier but this is not
	 * guaranteed.
	 * 'pagenum' is the page number with the user row. For SAM0, this must
	 * be 0.
	 * 'page' points to the data supplied by the user.
	 * We accept all 64 bits of the 'page', even if the user only wrote
	 * one or two of the bytes therein. The Flash framework will have
	 * padded the remaining bytes with 0xff. This isn't ideal, but we
	 * have no real way (yet?) to enforce page-size write.
	 * So, all we need do is ensure the reserved values are preserved.
	 */

	(void) pagenum;
	assert(pagenum == 0);

	for (i = 0; i < (ss->ss_user_size / sizeof(uint64_t)); i++) {
		/* Grab a local/aligned copy of the existing page. */
		memcpy(&resvd, nvm, sizeof(resvd));

		/* Keep only this SoC's reserved bitfields. */
		resvd &= ss->ss_resvd_mask[i];

		/* Grab the new data we're about to write. */
		memcpy(&new_bits, page, sizeof(new_bits));

		/* Keep all but this SoC's reserved bitfields. */
		new_bits &= ~(ss->ss_resvd_mask[i]);

		/* Merge with the existing reserved bitfields. */
		new_bits |= resvd;

		/* Update the user's buffer. */
		memcpy(page, &new_bits, sizeof(new_bits));

		nvm += sizeof(uint64_t);
	}

	return 0;
}

int
soc_sam0_attach(target_t t)
{
	const struct sam0_id *si;
	const struct sam0_soc *soc;
	struct sam0_state *ss;
	uint16_t flash_type;
	cortexm_t cm;
	uint32_t did, dsu_flags, user_size;
	const uint64_t *resvd_mask;
	void *dsu;
	microchip_dsu_protection_t prot;

	/*
	 * These devices have a DSU. Go probe for it...
	 */
	if ((dsu = microchip_dsu_core_probe_did(t, &did)) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	/* Do we support this device? */
	if ((si = soc_sam0_match(did, &flash_type, &user_size, &resvd_mask,
	    &soc, &dsu_flags)) == NULL) {
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	if ((ss = zone_malloc(sizeof(*ss))) == NULL)
		return CORTEXM_SOC_ATTACH_NO_MATCH;

	ss->ss_si = si;
	ss->ss_did = did;
	ss->ss_dsu_flags = dsu_flags;
	ss->ss_user_size = user_size * sizeof(uint64_t);
	ss->ss_resvd_mask = resvd_mask;

	/* Device is supported. Grab protection status from DSU. */
	if (microchip_dsu_protection_status(dsu, ss->ss_dsu_flags, &prot) < 0) {
		zone_free(ss);
		return CORTEXM_SOC_ATTACH_NO_MATCH;
	}

	cm = (cortexm_t)t->t_core;
	ss->ss_dsu = dsu;
	ss->ss_prot = prot;
	ss->ss_flash_type = flash_type;

	if ((si->si_flags & SAM0_FLAGS_TRUSTZONE) == 0)
		cm->cm_flags &= ~CORTEXM_FLAG_V8M_EXT_SECURITY;

	if (prot <= MCP_DSU_PROT_PARTIAL) {
		struct flash_sam_nvmctrl_attach_args aa;

		memset(&aa, 0, sizeof(aa));

		target_add_memory(t, TARGET_MEM_RAM, SAM0_SRAM_BASE,
		    (uint32_t)si->si_sram_size * 1024u, NULL);

		if (soc->soc_family == SAM0_FAMILY_L &&
		    soc->soc_series == SAM0_SERIES_21) {
			/*
			 * These devices have an additional low-power SRAM
			 * region at 0x30000000. Size is dependent on the
			 * designation, which we can figure out based on
			 * the size of regular SRAM.
			 */
			uint32_t lp_sram_size;

			if (si->si_sram_size == 4)
				lp_sram_size = 2;
			else
			if (si->si_sram_size == 8)
				lp_sram_size = 4;
			else
				lp_sram_size = 8;

			target_add_memory(t, TARGET_MEM_RAM |
			    TARGET_MEM_NO_APPLET, SAM0_LP_SRAM_BASE,
			    lp_sram_size * 1024u, "Low-power SRAM");
		}

		aa.aa_target = t;
		if (si->si_flags & SAM0_FLAGS_TRUSTZONE)
			aa.aa_nvmctrl_regs = SAM0_NVMCTRL_SEC_REGS;
		else
			aa.aa_nvmctrl_regs = SAM0_NVMCTRL_REGS;
		aa.aa_flash_base = SAM0_FLASH_BASE;
		aa.aa_user_base = SAM0_USER_BASE;
		aa.aa_user_size = ss->ss_user_size;
		aa.aa_user_erase = soc_sam0_user_erase;
		aa.aa_user_check = soc_sam0_user_check;
		aa.aa_user_cookie = ss;
		aa.aa_type_ws = FLASH_SAM_NVMCTRL_TYPE(flash_type, 1);
		aa.aa_flags = FLASH_SAM_NVMCTRL_FLAG_EEP;

		if (si->si_flags & SAM0_FLAGS_HAS_RWWEEP) {
			aa.aa_flags |= FLASH_SAM_NVMCTRL_FLAG_RWWEEP;
			aa.aa_rwweep_base = SAM0_RWWEEP_BASE;
		}

		ss->ss_nvmctrl = flash_sam_nvmctrl_attach(&aa);
		if (ss->ss_nvmctrl == NULL) {
			microchip_dsu_done(ss->ss_dsu);
			zone_free(ss);
			return CORTEXM_SOC_ATTACH_NO_MATCH;
		}

		switch (MCP_DSU_DID_PROCESSOR(did)) {
		case SAM0_PROCESSOR_M0_PLUS:
			break;

		case SAM0_PROCESSOR_M23:
			if (ss->ss_dsu_flags & MCP_DSU_FLAGS_TRUSTZONE) {
				cm->cm_features |= CORTEXM_FEATURE_SECURITY;
				cm->cm_flags |= CORTEXM_FLAG_V8M_EXT_SECURITY;
				if (prot == MCP_DSU_PROT_PARTIAL) {
					cm->cm_flags |=
					    CORTEXM_FLAG_UNTRUSTED_ONLY;
				}
				cm->cm_regmap_security =
				    cortexm_txml_regmap_v8m_s;
			} else {
				cm->cm_regmap_security =
				    cortexm_txml_regmap_v8m_ns;
			}
			break;

		default:
			break;
		}
	} else {
		t->t_flags |= TARGET_FLAG_IS_LOCKED;
		ss->ss_nvmctrl = NULL;
	}

	t->t_flags |= TARGET_FLAG_SUPPORTS_PROTECTION;

	cm->cm_soc_ctl = soc_sam0_ctl;
	cm->cm_soc_free = soc_sam0_free;
	cm->cm_soc = ss;

	soc_sam0_register(did, si);

	return CORTEXM_SOC_ATTACH_OK;
}

void *
soc_sam0_dsu_probe(uint32_t did, uint32_t *pflags)
{

	return (void *)(uintptr_t)soc_sam0_match(did, NULL, NULL, NULL, NULL,
	    pflags);
}

uint32_t
soc_sam0_dsu_register(uint32_t did, void *arg)
{
	const struct sam0_id *si = arg;

	soc_sam0_register(did, si);

	/*
	 * Worst case erase time: 64 KB, 1K per page, 6 mS erase time/page.
	 * Round it up to one second.
	 */

	return 1000u;
}
