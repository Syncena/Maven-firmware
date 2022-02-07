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

#ifndef CORTEXM_REGS_H
#define CORTEXM_REGS_H

/*
 * Registers within the Cortex-M? System Control Space.
 * These offsets are relative to the start of the component,
 * as described by the Memory Access Component's ROM table.
 */
#define	CMSCS_REG_ICTR			0x004u
#define	CMSCS_REG_ACTLR			0x008u
#define	CMSCS_REG_STCSR			0x010u
#define	CMSCS_REG_STRVR			0x014u
#define	CMSCS_REG_STCVR			0x018u
#define	CMSCS_REG_STCR			0x01cu

#define	CMSCS_REG_CPUID			0xd00u
#define	CMSCS_REG_ICSR			0xd04u
#define	CMSCS_REG_VTOR			0xd08u
#define	CMSCS_REG_AIRCR			0xd0cu
#define	 CMSCS_AIRCR_VECTKEY		(0x05fau << 16)
#define	 CMSCS_AIRCR_ENDIANNESS		(1u << 15)
#define	 CMSCS_AIRCR_PRIGROUP_SHIFT	11
#define	 CMSCS_AIRCR_PRIGROUP_MASK	0x0fu
#define	 CMSCS_AIRCR_SYSRESETREQ	((1u << 2) | CMSCS_AIRCR_VECTKEY)
#define	 CMSCS_AIRCR_VECTCLRACTIVE	((1u << 1) | CMSCS_AIRCR_VECTKEY)
#define	 CMSCS_AIRCR_VECTRESET		((1u << 0) | CMSCS_AIRCR_VECTKEY)
#define	CMSCS_REG_SCR			0xd10u
#define	CMSCS_REG_CCR			0xd14u
#define  CMSCS_CCR_NONBASETHRDENA	(1u << 0)
#define  CMSCS_CCR_USERSETMPEND		(1u << 1)
#define  CMSCS_CCR_UNALIGN_TRP		(1u << 3)
#define  CMSCS_CCR_DIV_0_TRP		(1u << 4)
#define  CMSCS_CCR_BFHFNMIGN		(1u << 8)
#define  CMSCS_CCR_STKALIGN		(1u << 9)
#define  CMSCS_CCR_DC			(1u << 16)
#define  CMSCS_CCR_IC			(1u << 17)
#define  CMSCS_CCR_BP			(1u << 18)
#define	CMSCS_REG_SHPR1			0xd18u
#define	CMSCS_REG_SHPR2			0xd1cu
#define	CMSCS_REG_SHPR3			0xd20u
#define	CMSCS_REG_SHCSR			0xd24u
#define	CMSCS_REG_CFSR			0xd28u
#define	 CMSCS_CFSR_MEMMANAGE_MASK	0x0000003bu
#define	  CMSCS_MMFSR_IACCVIOL		(1u << 0)
#define	  CMSCS_MMFSR_DACCVIOL		(1u << 1)
#define	  CMSCS_MMFSR_MUNSTKERR		(1u << 3)
#define	  CMSCS_MMFSR_MSTKERR		(1u << 4)
#define	  CMSCS_MMFSR_MLSPERR		(1u << 5)
#define	  CMSCS_MMFSR_MMARVALID		(1u << 7)
#define	 CMSCS_CFSR_BUSFAULT_MASK	0x00003f00u
#define	  CMSCS_BFSR_IBUSERR		(1u << 8)
#define	  CMSCS_BFSR_PRECISERR		(1u << 9)
#define	  CMSCS_BFSR_IMPRECISERR	(1u << 10)
#define	  CMSCS_BFSR_UNSTKERR		(1u << 11)
#define	  CMSCS_BFSR_STKERR		(1u << 12)
#define	  CMSCS_BFSR_LSPERR		(1u << 13)
#define	  CMSCS_BFSR_BFARVALID		(1u << 15)
#define	 CMSCS_CFSR_USAGEFAULT_MASK	0x030f0000u
#define	  CMSCS_UFSR_UNDEFINSTR		(1u << 16)
#define	  CMSCS_UFSR_INVSTATE		(1u << 17)
#define	  CMSCS_UFSR_INVPC		(1u << 18)
#define	  CMSCS_UFSR_NOCP		(1u << 19)
#define	  CMSCS_UFSR_UNALIGNED		(1u << 24)
#define	  CMSCS_UFSR_DIVBYZERO		(1u << 25)
#define	 CMSCS_CFSR_FAULT_MASK		(CMSCS_CFSR_MEMMANAGE_MASK | \
					 CMSCS_CFSR_BUSFAULT_MASK | \
					 CMSCS_CFSR_USAGEFAULT_MASK)
#define	CMSCS_REG_HFSR			0xd2cu
#define  CMSCS_HFSR_DEBUGEVT		(1u << 31)
#define  CMSCS_HFSR_FORCED		(1u << 30)
#define  CMSCS_HFSR_VECTTBL		(1u << 1)
#define	CMSCS_REG_DFSR			0xd30u
#define  CMSCS_DFSR_HALTED		(1u << 0)
#define  CMSCS_DFSR_BKPT		(1u << 1)
#define  CMSCS_DFSR_DWTTRAP		(1u << 2)
#define  CMSCS_DFSR_VCATCH		(1u << 3)
#define  CMSCS_DFSR_EXTERNAL		(1u << 4)
#define	CMSCS_REG_MMFAR			0xd34u
#define	CMSCS_REG_BFAR			0xd38u
#define	CMSCS_REG_AFSR			0xd3cu
#define	CMSCS_REG_ID_PFR0		0xd40u
#define	CMSCS_REG_ID_PFR1		0xd44u
#define	 CMSCS_ID_PFR1_SECURITY(x)	(((x) >> 4) & 0xfu)
#define	 CMSCS_ID_PFR1_SECURITY_NONE	0
#define	 CMSCS_ID_PFR1_SECURITY_AVAIL	1
#define	CMSCS_REG_ID_DFR0		0xd48u
#define	CMSCS_REG_ID_AFR0		0xd4cu
#define	CMSCS_REG_ID_MMFR0		0xd50u
#define	CMSCS_REG_ID_MMFR1		0xd54u
#define	CMSCS_REG_ID_MMFR2		0xd58u
#define	CMSCS_REG_ID_MMFR3		0xd5cu
#define	CMSCS_REG_ID_ISAR0		0xd60u
#define	CMSCS_REG_ID_ISAR1		0xd64u
#define	CMSCS_REG_ID_ISAR2		0xd68u
#define	CMSCS_REG_ID_ISAR3		0xd6cu
#define	CMSCS_REG_ID_ISAR4		0xd70u
#define	CMSCS_REG_ID_ISAR5		0xd74u	/* Reserved, RAZ */

#define	CMSCS_REG_CLIDR			0xd78u
#define	 CMSCS_CLIDR_CTYPE(c,l)		(((c) >> ((l) * 3)) & 0x7u)
#define	 CMSCS_CLIDR_CTYPE_NONE		0u
#define	 CMSCS_CLIDR_CTYPE_I		1u
#define	 CMSCS_CLIDR_CTYPE_D		2u
#define	 CMSCS_CLIDR_CTYPE_ID		3u
#define	 CMSCS_CLIDR_CTYPE_UNIFIED	4u
#define	CMSCS_REG_CTR			0xd7cu
#define	 CMSCS_CTR_FORMAT_ARM(ctr)	(((ctr) & 0xf000c000u) == 0x8000c000u)
#define	 CMSCS_CTR_IMINLINE(ctr)	((ctr) & 0xfu)
#define	 CMSCS_CTR_DMINLINE(ctr)	(((ctr) >> 16) & 0xfu)
#define	 CMSCS_CTR_ERG(ctr)		(((ctr) >> 20) & 0xfu)
#define	 CMSCS_CTR_CWG(ctr)		(((ctr) >> 24) & 0xfu)
#define	CMSCS_REG_CCSIDR		0xd80u
#define	 CMSCS_CCSIDR_LINESIZE_BYTES(x)	(4u << (((x) & 0x7u) + 2u))
#define	 CMSCS_CCSIDR_ASSOCIATIVITY(x)	((((x) >> 3) & 0x3ffu) + 1u)
#define	 CMSCS_CCSIDR_NUM_SETS(x)	((((x) >> 13) & 0x7fffu) + 1u)
#define	 CMSCS_CCSIDR_WA		(1u << 28)
#define	 CMSCS_CCSIDR_RA		(1u << 29)
#define	 CMSCS_CCSIDR_WB		(1u << 30)
#define	 CMSCS_CCSIDR_WT		(1u << 31)
#define	CMSCS_REG_CSSELR		0xd84u
#define	CMSCS_REG_CPACR			0xd88u

#define	CMSCS_REG_MPU_TYPE		0xd90u
#define	 CMSCS_MPU_TYPE_IREGION(t)	(((t) >> 16) & 0xffu)
#define	 CMSCS_MPU_TYPE_DREGION(t)	(((t) >> 8) & 0xffu)
#define	 CMSCS_MPU_TYPE_SEPARATE(t)	((t) & 1u)
#define	CMSCS_REG_MPU_CTRL		0xd94u
#define	 CMSCS_MPU_CTRL_ENABLE		(1u << 0)
#define	 CMSCS_MPU_CTRL_HFNMIENA	(1u << 1)
#define	 CMSCS_MPU_CTRL_PRIVDEFENA	(1u << 2)
#define	CMSCS_REG_MPU_RNR		0xd98u
#define	CMSCS_REG_MPU_RBAR		0xd9cu
#define	CMSCS_REG_MPU_RASR		0xda0u

#define	CMSCS_REG_DHCSR			0xdf0u
#define	 CMSCS_DHCSR_S_RESET		(1u << 25)
#define	 CMSCS_DHCSR_S_RETIRE_ST	(1u << 24)
#define	 CMSCS_DHCSR_S_SDE		(1u << 20)
#define	 CMSCS_DHCSR_S_LOCKUP		(1u << 19)
#define	 CMSCS_DHCSR_S_SLEEP		(1u << 18)
#define	 CMSCS_DHCSR_S_HALT		(1u << 17)
#define	 CMSCS_DHCSR_S_REGRDY		(1u << 16)
#define	 CMSCS_DHCSR_C_SNAPSTALL	(1u << 5)
#define	 CMSCS_DHCSR_C_MASKINTS		(1u << 3)
#define	 CMSCS_DHCSR_C_STEP		(1u << 2)
#define	 CMSCS_DHCSR_C_HALT		(1u << 1)
#define	 CMSCS_DHCSR_C_DEBUGEN		(1u << 0)
#define	 CMSCS_DHCSR_DBGKEY		(0xa05f0000u)
#define	CMSCS_REG_DCRSR			0xdf4u
#define	 CMSCS_DCRSR_REGSEL_RD(r)	((r) & 0x7fu)
#define	 CMSCS_DCRSR_REGSEL_WR(r)	(((r) & 0x7fu) | (1u << 16))
#define	 CMSCW_DCRSR_REG_GPR(n)		(n)
#define	 CMSCW_DCRSR_REG_SP		13
#define	 CMSCW_DCRSR_REG_LR		14
#define	 CMSCW_DCRSR_REG_PC		15
#define	 CMSCW_DCRSR_REG_xPSR		16
#define	 CMSCW_DCRSR_REG_MSP		17
#define	 CMSCW_DCRSR_REG_PSP		18
#define	 CMSCW_DCRSR_REG_CFBP		20
#define	 CMSCW_DCRSR_CFBP_PRIMASK_SHIFT	0
#define	 CMSCW_DCRSR_CFBP_PRIMASK_MASK	0x01u
#define	 CMSCW_DCRSR_CFBP_PRIMASK(x)	\
		(((x) >> CMSCW_DCRSR_CFBP_PRIMASK_SHIFT) & \
		 CMSCW_DCRSR_CFBP_PRIMASK_MASK)
#define	 CMSCW_DCRSR_CFBP_BASEPRI_SHIFT	8
#define	 CMSCW_DCRSR_CFBP_BASEPRI_MASK	0xffu
#define	 CMSCW_DCRSR_CFBP_BASEPRI(x)	\
		(((x) >> CMSCW_DCRSR_CFBP_BASEPRI_SHIFT) & \
		 CMSCW_DCRSR_CFBP_BASEPRI_MASK)
#define	 CMSCW_DCRSR_CFBP_FAULTMASK_SHIFT	16
#define	 CMSCW_DCRSR_CFBP_FAULTMASK_MASK	0x1u
#define	 CMSCW_DCRSR_CFBP_FAULTMASK(x)	\
		(((x) >> CMSCW_DCRSR_CFBP_FAULTMASK_SHIFT) & \
		 CMSCW_DCRSR_CFBP_FAULTMASK_MASK)
#define	 CMSCW_DCRSR_CFBP_CONTROL_SHIFT	24
#define	 CMSCW_DCRSR_CFBP_CONTROL_MASK	0x7u
#define	 CMSCW_DCRSR_CFBP_CONTROL(x)	\
		(((x) >> CMSCW_DCRSR_CFBP_CONTROL_SHIFT) & \
		 CMSCW_DCRSR_CFBP_CONTROL_MASK)
#define	 CMSCW_DCRSR_REG_MSP_NS		24
#define	 CMSCW_DCRSR_REG_PSP_NS		25
#define	 CMSCW_DCRSR_REG_MSP_S		26	/* Only when DHCSR.S_SDE == 1 */
#define	 CMSCW_DCRSR_REG_PSP_S		27	/* Only when DHCSR.S_SDE == 1 */
#define	 CMSCW_DCRSR_REG_MSPLIM_S	28	/* Only when DHCSR.S_SDE == 1 */
#define	 CMSCW_DCRSR_REG_PSPLIM_S	29	/* Only when DHCSR.S_SDE == 1 */
#define	 CMSCW_DCRSR_REG_MSPLIM_NS	30
#define	 CMSCW_DCRSR_REG_PSPLIM_NS	31
#define	 CMSCW_DCRSR_REG_FPSCR		33
#define	 CMSCW_DCRSR_REG_FPR(n)		(64 + (n))
#define	 CMSCW_DCRSR_PSEUDO_REG_SECURE	255u
#define	CMSCS_REG_DCRDR			0xdf8u
#define	CMSCS_REG_DEMCR			0xdfcu
#define	 CMSCS_DEMCR_TRCENA		(1u << 24)
#define	 CMSCS_DEMCR_DWTENA		CMSCS_DEMCR_TRCENA   /* M0+ synonym */
#define	 CMSCS_DEMCR_MON_REQ		(1u << 19)
#define	 CMSCS_DEMCR_MON_STEP		(1u << 18)
#define	 CMSCS_DEMCR_MON_PEND		(1u << 17)
#define	 CMSCS_DEMCR_MON_EN		(1u << 16)
#define	 CMSCS_DEMCR_VC_SFERR		(1u << 11)
#define	 CMSCS_DEMCR_VC_HARDERR		(1u << 10)
#define	 CMSCS_DEMCR_VC_INTERR		(1u << 9)
#define	 CMSCS_DEMCR_VC_BUSERR		(1u << 8)
#define	 CMSCS_DEMCR_VC_STATERR		(1u << 7)
#define	 CMSCS_DEMCR_VC_CHKERR		(1u << 6)
#define	 CMSCS_DEMCR_VC_NOCPERR		(1u << 5)
#define	 CMSCS_DEMCR_VC_MMERR		(1u << 4)
#define	 CMSCS_DEMCR_VC_CORERESET	(1u << 0)
#define	 CMSCS_DEMCR_VC_CFG_V6M		CMSCS_DEMCR_VC_HARDERR
#define	 CMSCS_DEMCR_VC_CFG_V7M		(CMSCS_DEMCR_VC_HARDERR |	\
					 CMSCS_DEMCR_VC_INTERR |	\
					 CMSCS_DEMCR_VC_BUSERR |	\
					 CMSCS_DEMCR_VC_STATERR |	\
					 CMSCS_DEMCR_VC_CHKERR |	\
					 CMSCS_DEMCR_VC_NOCPERR |	\
					 CMSCS_DEMCR_VC_MMERR)
#define	 CMSCS_DEMCR_VC_CFG_V8M		(CMSCS_DEMCR_VC_CFG_V7M |	\
					 CMSCS_DEMCR_VC_SFERR)

#define	CMSCS_REG_DSCSR			0xe08u	/* ARMv8M, with Sec only */
#define	 CMSCS_DSCSR_CDSKEY		(1u << 17)
#define	 CMSCS_DSCSR_CDS		(1u << 16)
#define	 CMSCS_DSCSR_SBRSEL		(1u << 1)
#define	 CMSCS_DSCSR_SBRSELEN		(1u << 0)

#define	CMSCS_REG_STIR			0xf00u
#define	CMSCS_REG_FPCCR			0xf34u
#define	 CMSCS_FPCCR_ASPEN		(1u << 31)
#define	 CMSCS_FPCCR_LSPEN		(1u << 30)
#define	 CMSCS_FPCCR_MONRDY		(1u << 8)
#define	 CMSCS_FPCCR_BFRDY		(1u << 6)
#define	 CMSCS_FPCCR_MMRDY		(1u << 5)
#define	 CMSCS_FPCCR_HFRDY		(1u << 4)
#define	 CMSCS_FPCCR_THREAD		(1u << 3)
#define	 CMSCS_FPCCR_USER		(1u << 1)
#define	 CMSCS_FPCCR_LSPACT		(1u << 0)
#define	CMSCS_REG_FPCAR			0xf38u
#define	CMSCS_REG_FPDSCR		0xf3cu
#define	CMSCS_REG_MVFR0			0xf40u
#define	 CMSCS_MVFR0_SP_SUPPORTED(r)	((((r) >> 4) & 0xfu) == 0x2u)
#define	 CMSCS_MVFR0_DP_SUPPORTED(r)	((((r) >> 8) & 0xfu) == 0x2u)
#define	CMSCS_REG_MVFR1			0xf44u
#define	CMSCS_REG_MVFR2			0xf48u

#define	CMSCS_REG_ICIALLU		0xf50u
#define	CMSCS_REG_ICIMVAU		0xf58u
#define	CMSCS_REG_DCIMVAC		0xf5cu
#define	CMSCS_REG_DCISW			0xf60u
#define	CMSCS_REG_DCCMVAU		0xf64u
#define	CMSCS_REG_DCCMVAC		0xf68u
#define	CMSCS_REG_DCCSW			0xf6cu
#define	CMSCS_REG_DCCIMVAC		0xf70u
#define	CMSCS_REG_DCCISW		0xf74u
#define	CMSCS_REG_BPIALL		0xf78u

#define	CMSCS_REG_PID4			0xfd0u
#define	CMSCS_REG_PID5			0xfd4u
#define	CMSCS_REG_PID6			0xfd8u
#define	CMSCS_REG_PID7			0xfdcu
#define	CMSCS_REG_PID0			0xfe0u
#define	CMSCS_REG_PID1			0xfe4u
#define	CMSCS_REG_PID2			0xfe8u
#define	CMSCS_REG_PID3			0xfecu
#define	CMSCS_REG_CID0			0xff0u
#define	CMSCS_REG_CID1			0xff4u
#define	CMSCS_REG_CID2			0xff8u
#define	CMSCS_REG_CID3			0xffcu

#define	CMSCS_CPUID_IMPLEMENTER(c)	((unsigned int)(((c) >> 24) & 0xffu))
#define	CMSCS_CPUID_VARIANT(c)		((unsigned int)(((c) >> 20) & 0x0fu))
#define	CMSCS_CPUID_ARCHITECTURE(c)	((unsigned int)(((c) >> 16) & 0x0fu))
#define	CMSCS_CPUID_PARTNO(c)		((unsigned int)(((c) >> 4) & 0xfffu))
#define	CMSCS_CPUID_REVISION(c)		((unsigned int)(((c) >> 0) & 0x0fu))

#define	CPUID_IMPLEMENTER_ARM		0x41u
#define	CPUID_PARTNO_CM0		0xc20u
#define	CPUID_PARTNO_CM0_PLUS		0xc60u
#define	CPUID_PARTNO_CM3		0xc23u
#define	CPUID_PARTNO_CM4		0xc24u
#define	CPUID_PARTNO_CM7		0xc27u
#define	CPUID_PARTNO_CM23		0xd20u
#define	CPUID_PARTNO_CM33		0xd21u
#define	CPUID_ARCH_ARMV6M		0x0cu
#define	CPUID_ARCH_ARMV7M		0x0fu
#define	CPUID_ARCH_ARMV8M_WITH_MAIN	0x0fu
#define	CPUID_ARCH_ARMV8M_WITHOUT_MAIN	0x0cu

/*
 * DWT Registers (base is usually 0xe0001000)
 */
#define	CMDWT_REG_CTRL			0x00u
#define	 CMDWT_CTRL_NUMCOMP(c)		(((c) >> 28) & 0x0fu)
#define	CMDWT_REG_CYCCNT		0x04u
#define	CMDWT_REG_CPICNT		0x08u
#define	CMDWT_REG_EXCCNT		0x0cu
#define	CMDWT_REG_SLEEPCNT		0x10u
#define	CMDWT_REG_LSUCNT		0x14u
#define	CMDWT_REG_FOLDCNT		0x18u
#define	CMDWT_REG_PCSR			0x1cu
#define	CMDWT_REG_COMP(n)		(0x20u + (0x10u * (n)))
#define	CMDWT_REG_MASK(n)		(0x24u + (0x10u * (n)))
#define	CMDWT_MASK_MASK			0x1fu
#define	CMDWT_REG_FUNCTION(n)		(0x28u + (0x10u * (n)))
#define	 CMDWT_FUNC_MATCHED		(1u << 24)
#define	 CMDWT_FUNC_ACTION_MASK		0x0fu
#define	 CMDWT_FUNC_ACTION_READ		5u
#define	 CMDWT_FUNC_ACTION_WRITE	6u
#define	 CMDWT_FUNC_ACTION_ACCESS	7u
#define	 CMDWT_FUNC_ACTION_MASK_V8	0x3fu
#define	 CMDWT_FUNC_ACTION_ACCESS_V8	0x14u
#define	 CMDWT_FUNC_ACTION_WRITE_V8	0x15u
#define	 CMDWT_FUNC_ACTION_READ_V8	0x16u

/*
 * FPB Registers
 */
#define	CMFPB_REG_FP_CTRL		0x00u
#define	 CMFPB_FP_CTRL_ENABLE		(1u << 0)
#define	 CMFPB_FP_CTRL_KEY		(1u << 1)
#define	 CMFPB_FP_CTRL_NUM_CODE(c)	((((c) >> 4) & 0x0fu) | \
					 (((c) >> 8) & 0x70u))
#define	 CMFPB_FP_CTRL_NUM_CODEv6(c)	(((c) >> 4) & 0x0fu)
#define	 CMFPB_FP_CTRL_NUM_LIT(c)	(((c) >> 8) & 0x0fu)
#define	 CMFPB_FP_CTRL_REV(c)		(((c) >> 28) & 0x0fu)
#define	CMFPB_REG_FP_REMAP		0x04u
#define	 CMFPB_FP_REMAP_BASE(r)		((r) & 0x1ffffff0u)
#define	 CMFPB_FP_REMAP_RMPSPT		(1u << 29)
#define	CMFPB_REG_FP_COMP(n)		(0x08u + ((n) * 4))
#define	 CMFPB_FP_COMP_ENABLE		(1u << 0)
#define	 CMFPB_FP_COMP_ADDR_MASK	0x1ffffffcu
#define	 CMFPB_FP_COMP_ADDR(v)		((v) & CMFPB_FP_COMP_ADDR_MASK)
#define	 CMFPB_FP_COMP_REPLACE(v)	(((v) >> 30) & 0x03u)
#define	 CMFPB_FP_COMP_REPLACESHIFT	30
#define	 CMFPB_FP_COMP_REMAP		0u
#define	 CMFPB_FP_COMP_COMP0		1u
#define	 CMFPB_FP_COMP_COMP1		2u
#define	 CMFPB_FP_COMP_COMP_BOTH	3u

/*
 * PC value indicating core lockup
 */
#define	CMPC_LOCKUP_VALUE		0xfffffffeu

#endif /* CORTEXM_REGS_H */
