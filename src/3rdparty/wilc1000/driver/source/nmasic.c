/**
 *
 * \file
 *
 * \brief This module contains WILC ASIC specific internal APIs.
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "driver/source/nmbus.h"
#include "bsp/include/nm_bsp.h"
#include "driver/source/nmasic.h"

#ifndef CONF_WILC_FW_IN_FLASH
#ifdef CONF_WILC_COMPRESSED_FIRMWARE
#include <stdlib.h>
#endif
#endif

#define NMI_GLB_RESET_0 (NMI_PERIPH_REG_BASE + 0x400)
#define NMI_GLB_RESET_1 (NMI_PERIPH_REG_BASE + 0x404)
#define NMI_INTR_REG_BASE (NMI_PERIPH_REG_BASE+0xa00)
#define NMI_PIN_MUX_0 (NMI_PERIPH_REG_BASE + 0x408)
#define NMI_INTR_ENABLE (NMI_INTR_REG_BASE)
#define GET_UINT32(X,Y) (X[0+Y] + ((uint32)X[1+Y]<<8) + ((uint32)X[2+Y]<<16) +((uint32)X[3+Y]<<24))

#define TIMEOUT				(2000)
#define M2M_DISABLE_PS        0xD0UL
#define WAKUP_TRAILS_TIMEOUT		(10000)

void chip_idle(void)
{
	uint32 reg =0;
	nm_read_reg_with_ret(WILC_WAKEUP_REG, &reg);
	if(reg & WILC_WAKEUP_BIT)
	{
		reg &=~WILC_WAKEUP_BIT;
		nm_write_reg(WILC_WAKEUP_REG, reg);
	}
}

sint8 enable_interrupts(void) 
{
	uint32 reg;
	sint8 ret;
	/**
	interrupt pin mux select
	**/
	ret = nm_read_reg_with_ret(NMI_PIN_MUX_0, &reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	reg |= ((uint32) 1 << 8);
	ret = nm_write_reg(NMI_PIN_MUX_0, reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	/**
	interrupt enable
	**/
	ret = nm_read_reg_with_ret(NMI_INTR_ENABLE, &reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	reg |= ((uint32) 1 << 16);
	ret = nm_write_reg(NMI_INTR_ENABLE, reg);
	if (M2M_SUCCESS != ret) {
		return M2M_ERR_BUS_FAIL;
	}
	return M2M_SUCCESS;
}

sint8 cpu_start(void) {
	sint8 ret = M2M_SUCCESS;
	
#ifndef CONF_WILC_FW_IN_FLASH
	uint32 reg;
	/**
	reset regs 
	*/
	/*disable boot rom*/
	nm_write_reg(0xc0000, 0x71);
	nm_write_reg(NMI_STATE_REG,0);
	nm_write_reg(NMI_REV_REG,0);
	
	/**
	Go...
	**/
	ret = nm_read_reg_with_ret(0x1118, &reg);
	if (M2M_SUCCESS != ret) {
		ret = M2M_ERR_BUS_FAIL;
		M2M_ERR("[nmi start]: fail read reg 0x1118 ...\n");
	}
	reg |= (1 << 0);
	ret = nm_write_reg(0x1118, reg);
#if defined CONF_WILC_USE_SPI
	nm_write_reg(NMI_VMM_CORE_CFG, 1);
#elif defined CONF_WILC_USE_SDIO
	nm_write_reg(NMI_VMM_CORE_CFG, (1 << 3));
#endif
	ret += nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if ((reg & (1ul << 10)) == (1ul << 10)) {
		reg &= ~(1ul << 10);
		ret += nm_write_reg(NMI_GLB_RESET_0, reg);
	}

	reg |= (1ul << 10);
	ret += nm_write_reg(NMI_GLB_RESET_0, reg);
	nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
#else
	
	#if defined CONF_WILC_USE_SPI
	nm_write_reg(NMI_VMM_CORE_CFG, 1);
	#elif defined CONF_WILC_USE_SDIO
	nm_write_reg(NMI_VMM_CORE_CFG, (1 << 3));
	#endif

#endif /* CONF_WILC_FW_IN_FLASH */
	return ret;
}

#if (defined CONF_WILC_USE_3000_REV_A && !defined WILC_SERIAL_BRIDGE_INTERFACE)
sint8 cpu_start_bt(void) {
	uint32 reg;
	sint8 ret = M2M_SUCCESS;

#ifdef CONF_WILC_FW_IN_FLASH
	/* Set Mux Select for BT CPU Reset */
	ret = nm_read_reg_with_ret(0x3b0090, &reg);
	reg |= 1;
	ret += nm_write_reg(0x3b0090, reg);
	
	/* Boot from IRAM, not ROM*/
	nm_write_reg(0x4F0000, 0x71);
#endif /* CONF_WILC_FW_IN_FLASH */
	nm_write_reg(0x4F000c, 0x10add09e);
	
	/**
	Go...
	**/
	ret += nm_read_reg_with_ret(0x3B0400, &reg);
	if((reg & ((1ul << 2) | (1ul << 3))) != 0)
	{
		reg &= ~((1ul << 2) | (1ul << 3));
		nm_write_reg(0x3B0400, reg);
		nm_bsp_sleep(100);
	}
	reg |= ((1ul << 2) | (1ul << 3));
	nm_write_reg(0x3B0400, reg);
	
	nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
	return ret;
}
#endif

#ifdef CONF_WILC_USE_1000_REV_B
uint32 nmi_get_chipid(void)
{
	static uint32 chipid = 0;
	if (chipid == 0) {
		/*
		 * SDIO can't read into global variables
		 * Use this variable as a temp, then copy to the global
		 */
		uint32 temp_chipid = 0;
		uint32 rfrevid;

		if((nm_read_reg_with_ret(0x1000, &temp_chipid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
		}
		//if((ret = nm_read_reg_with_ret(0x11fc, &revid)) != M2M_SUCCESS) {
		//	return 0;
		//}
		if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
		}
		chipid = temp_chipid;

		if (chipid == 0x1002a0)  {
			if (rfrevid == 0x1) { /* 1002A0 */
			} else /* if (rfrevid == 0x2) */ { /* 1002A1 */
				chipid = 0x1002a1;
			}
		} else if(chipid == 0x1002b0) {
			if(rfrevid == 3) { /* 1002B0 */
			} else if(rfrevid == 4) { /* 1002B1 */
				chipid = 0x1002b1;
			} else /* if(rfrevid == 5) */ { /* 1002B2 */
				chipid = 0x1002b2;
			}
		} else {
		}
//#define PROBE_FLASH
#ifdef PROBE_FLASH
		if(chipid) {
			UWORD32 flashid;

			flashid = probe_spi_flash();
			if(flashid == 0x1230ef) {
				chipid &= ~(0x0f0000);
				chipid |= 0x050000;
			}
			if(flashid == 0xc21320c2) {
				chipid &= ~(0x0f0000);
				chipid |= 0x050000;
			}
		}
#else
		/*M2M is by default have SPI flash*/
		chipid &= ~(0x0f0000);
		chipid |= 0x050000;
#endif /* PROBE_FLASH */
	}
	return chipid;
}
#elif defined CONF_WILC_USE_3000_REV_A
uint32 nmi_get_chipid(void)
{
	static uint32 chipid = 0;

	if (chipid == 0) {
		/*
		 * SDIO can't read into global variables
		 * Use this variable as a temp, then copy to the global
		 */
		uint32 temp_chipid = 0;
		uint32 rfrevid;

		if((nm_read_reg_with_ret(0x3b0000, &temp_chipid)) != M2M_SUCCESS) {
			chipid = 0;
			return 0;
		}
		chipid = temp_chipid;
		//if((ret = nm_read_reg_with_ret(0x11fc, &revid)) != M2M_SUCCESS) {
		//	return 0;
		//}
		if (chipid == 0x3000D0)  {
			if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
				chipid = 0;
				return 0;
			}

			if (rfrevid == 0x6) { /* 1002A0 */
				chipid |= 0xD1;
			}
		
		}
//#define PROBE_FLASH
#ifdef PROBE_FLASH
		if(chipid) {
			UWORD32 flashid;

			flashid = probe_spi_flash();
			if((flashid != 0xffffffff) && (flashid != 0x0)) {
				chipid &= ~(0x0f0000);
				chipid |= 0x040000;
			}
		}
#else
		/*M2M is by default have SPI flash*/
		//chipid &= ~(0x0f0000);
		//chipid |= 0x050000;
#endif /* PROBE_FLASH */
	}
	return chipid;
}
#endif
uint32 nmi_get_rfrevid(void)
{
    uint32 rfrevid;
    if((nm_read_reg_with_ret(0x13f4, &rfrevid)) != M2M_SUCCESS) {
        rfrevid = 0;
        return 0;
    }
    return rfrevid;
}

void restore_pmu_settings_after_global_reset(void)
{
	/* 
	* Must restore PMU register value after 
	* global reset if PMU toggle is done at 
	* least once since the last hard reset.
	*/
#ifdef CONF_WILC_USE_1000_REV_B
	nm_write_reg(0x1e48, 0xb78469ce);
#endif
}

void nmi_update_pll(void)
{
	uint32 pll;

	pll = nm_read_reg(0x1428);
	pll &= ~0x1ul;
	nm_write_reg(0x1428, pll);
	pll |= 0x1ul;
	nm_write_reg(0x1428, pll);

}
void nmi_set_sys_clk_src_to_xo(void) 
{
#ifdef CONF_WILC_USE_1000_REV_B
	uint32 val32;

	/* Switch system clock source to XO. This will take effect after nmi_update_pll(). */
	val32 = nm_read_reg(0x141c);
	val32 |= (1 << 2);
	nm_write_reg(0x141c, val32);
	
	/* Do PLL update */
	nmi_update_pll();
#endif
}

sint8 chip_sleep(void)
{
	uint32 reg;
	sint8 ret = M2M_SUCCESS;
	uint32 trials = 10;
	
#ifdef CONF_WILC_USE_1000_REV_B
	while(trials)
	{
		trials--;
		ret = nm_read_reg_with_ret(WILC_TO_INTERFACE_FROM_WF_REG,&reg);
		if(ret != M2M_SUCCESS) goto ERR1;
		if((reg & WILC_TO_INTERFACE_FROM_WF_BIT) == 0) break;
	}
	if (!trials)
		M2M_ERR("FW not responding\n");
#endif
	/* Clear bit 1 */
	ret = nm_read_reg_with_ret(WILC_WAKEUP_REG, &reg);
	if(ret != M2M_SUCCESS)goto ERR1;
	if(reg & WILC_WAKEUP_BIT)
	{
		reg &=~WILC_WAKEUP_BIT;
		ret = nm_write_reg(WILC_WAKEUP_REG, reg);
		if(ret != M2M_SUCCESS)goto ERR1;
	}
#ifdef CONF_WILC_USE_1000_REV_B
	ret = nm_read_reg_with_ret(WILC_FROM_INTERFACE_TO_WF_REG, &reg);
	if(ret != M2M_SUCCESS)goto ERR1;
	if(reg & WILC_FROM_INTERFACE_TO_WF_BIT)
	{
		reg &= ~WILC_FROM_INTERFACE_TO_WF_BIT;
		ret = nm_write_reg(WILC_FROM_INTERFACE_TO_WF_REG, reg);
		if(ret != M2M_SUCCESS)goto ERR1;
	}
#endif
ERR1:
	return ret;
}

sint8 chip_wake(void)
{
	sint8 ret;
	uint32 clk_status_reg = 0,trials = 0;

#ifdef CONF_WILC_USE_1000_REV_B

	/*USE bit 0 to indicate host wakeup*/
	ret = nm_write_reg(WILC_FROM_INTERFACE_TO_WF_REG, WILC_FROM_INTERFACE_TO_WF_BIT);
	if(ret != M2M_SUCCESS)goto _WAKE_EXIT;

	ret = nm_write_reg(WILC_WAKEUP_REG, WILC_WAKEUP_BIT);
	if(ret != M2M_SUCCESS) goto _WAKE_EXIT;

#else
	ret = nm_read_reg_with_ret(WILC_WAKEUP_REG, (uint32*)&reg);
	if(ret != M2M_SUCCESS)goto _WAKE_EXIT;
	/* Set bit 1 */
	if(!(reg & WILC_WAKEUP_BIT))
	{
		ret = nm_write_reg(WILC_WAKEUP_REG, reg | WILC_WAKEUP_BIT);
		if(ret != M2M_SUCCESS) goto _WAKE_EXIT;	
	}
#endif
	do
	{
		ret = nm_read_reg_with_ret(WILC_CLK_STATUS_REG, (uint32*)&clk_status_reg);
		if(ret != M2M_SUCCESS) {
			M2M_ERR("Bus error (5).%d %lx\n",ret,clk_status_reg);
			goto _WAKE_EXIT;
		}
		if(clk_status_reg & WILC_CLK_STATUS_BIT) {
			break;
		}
		//nm_bsp_sleep(2);
		trials++;
		if(trials > WAKUP_TRAILS_TIMEOUT)
		{
			M2M_ERR("Failed to wakup the chip\n");
			ret = M2M_ERR_TIME_OUT;
			goto _WAKE_EXIT;
		}
	}while(1);
	
	/*workaround sometimes spi fail to read clock regs after reading/writing clockless registers*/
	nm_bus_reset();
	
_WAKE_EXIT:
	return ret;
}

sint8 chip_reset(void)
{
	sint8 ret = M2M_SUCCESS;
	
	nmi_set_sys_clk_src_to_xo();
#ifdef CONF_WILC_USE_SPI
	ret += nm_write_reg(NMI_GLB_RESET_0, 0);
#elif defined CONF_WILC_USE_SDIO
 /* Don't reset the SDIO modules*/
	ret += nm_write_reg(NMI_GLB_RESET_0, 0x80000301);
	ret += nm_write_reg(NMI_GLB_RESET_1, 0xFFFFFFFF);
#endif
	nm_bsp_sleep(50);
	restore_pmu_settings_after_global_reset();

	return ret;
}

#ifdef __KERNEL__ 
#include "linux/string.h"
void* linux_wlan_malloc(uint32_t sz);
#define LINUX_DMA_COHERENT_MEM
#include <linux/kernel.h>
#include <linux/slab.h> 
#endif

#ifndef CONF_WILC_FW_IN_FLASH
#ifndef CONF_WILC_COMPRESSED_FIRMWARE
#ifdef CONF_WILC_USE_1000_REV_B
#include "driver/include/wifi_firmware_1000b.h"
#elif defined CONF_WILC_USE_3000_REV_A
#include "driver/include/wifi_firmware_3000.h"
#if (defined CONF_WILC_USE_3000_REV_A && !defined WILC_SERIAL_BRIDGE_INTERFACE)
#include "driver/include/ble_firmware_3000.h"
#endif
#endif

sint8 firmware_download(void)
{
	sint8 s8Ret;
	uint32 u32SecSize, u32SecAddress, u32Val;
	uint8_t* pu8FirmwareBuffer;
	sint32 BuffIndex = 0, CurrentSecSize = 0;
	uint8_t u8TransferChunk[32], ChunkSize = 32;
	
	/* Set Mux Select for Wifi CPU Reset */
	s8Ret = nm_read_reg_with_ret(0x1118, &u32Val);
	u32Val |= 1;
	s8Ret += nm_write_reg(0x1118, u32Val);
	
	/* Assert CPU reset */
	s8Ret += nm_read_reg_with_ret(0x1400, &u32Val);
	u32Val &= ~(1ul << 10);
	s8Ret += nm_write_reg(0x1400, u32Val);

	/* Boot from IRAM, not ROM*/
	s8Ret += nm_write_reg(0xc0000,0x71);

	if(s8Ret != M2M_SUCCESS)
		goto ERR;

	pu8FirmwareBuffer = (uint8_t *)firmware;
	M2M_DBG("firmware size = %d\n",sizeof(firmware));

	while((uint32_t)pu8FirmwareBuffer < (uint32_t)(firmware+sizeof(firmware)))
	{		
		/*get text section address and size*/	
		u32SecAddress 	= (((uint32_t)(pu8FirmwareBuffer[3]))<<24)|(((uint32_t)(pu8FirmwareBuffer[2]))<<16)|
		(((uint32_t)(pu8FirmwareBuffer[1]))<<8)|(((uint32_t)(pu8FirmwareBuffer[0]))<<0);
		u32SecSize 	= (((uint32_t)(pu8FirmwareBuffer[7]))<<24)|(((uint32_t)(pu8FirmwareBuffer[6]))<<16)|
		(((uint32_t)(pu8FirmwareBuffer[5]))<<8)|(((uint32_t)(pu8FirmwareBuffer[4]))<<0);
		M2M_DBG("write sec %lx size %lu\n",u32SecAddress,u32SecSize);
		CurrentSecSize = u32SecSize;		
		ChunkSize = 32;
		BuffIndex = 8;
		while(CurrentSecSize>0)
		{
			if(CurrentSecSize < ChunkSize)
				ChunkSize = CurrentSecSize;
			
			m2m_memcpy(u8TransferChunk,pu8FirmwareBuffer+BuffIndex,ChunkSize);
			nm_write_block(u32SecAddress,u8TransferChunk,ChunkSize);
			u32SecAddress += ChunkSize;
			BuffIndex += ChunkSize;
			CurrentSecSize -= ChunkSize;
		}		
		pu8FirmwareBuffer += BuffIndex;
		
	}	
	
ERR:
	return s8Ret;
}

#else /* CONF_WILC_COMPRESSED_FIRMWARE */

extern int platform_wilc_fw_decompress(int (*)(void *, const unsigned char *,
				       unsigned long), void *, unsigned long);

#define	FW_CHUNK_SIZE	2048
struct fw_chunk_state {
	uint8_t cs_chunkbuff[FW_CHUNK_SIZE];
	uint32_t cs_chunklen;
	enum {
		FW_CHUNK_STATE_SECADDRESS,
		FW_CHUNK_STATE_SECSIZE,
		FW_CHUNK_STATE_DATA
	} cs_state;
	uint32_t cs_section_address;
	uint32_t cs_section_size;
};

static uint32_t
firmware_chunk_state_machine(struct fw_chunk_state *cs)
{
	uint32_t rv = 0;

	switch (cs->cs_state) {
	case FW_CHUNK_STATE_SECADDRESS:
		if (cs->cs_chunklen >= sizeof(uint32_t)) {
			memcpy(&cs->cs_section_address,
			    cs->cs_chunkbuff, sizeof(uint32_t));
			cs->cs_state = FW_CHUNK_STATE_SECSIZE;
			rv = sizeof(uint32_t);
		}
		break;

	case FW_CHUNK_STATE_SECSIZE:
		if (cs->cs_chunklen >= sizeof(uint32_t)) {
			memcpy(&cs->cs_section_size,
			    cs->cs_chunkbuff, sizeof(uint32_t));
			cs->cs_state = FW_CHUNK_STATE_DATA;
			rv = sizeof(uint32_t);
//			printf("FW Section @ 0x%08lx, size 0x%lx\n", cs->cs_section_address,
//			    cs->cs_section_size);
		}
		break;

	default:
		rv = cs->cs_section_size;
		if (rv > cs->cs_chunklen)
			rv = cs->cs_chunklen;
		
		nm_write_block(cs->cs_section_address, cs->cs_chunkbuff, rv);

		cs->cs_section_address += rv;
		cs->cs_section_size -= rv;
		if (cs->cs_section_size == 0) {
			cs->cs_state = FW_CHUNK_STATE_SECADDRESS;
//			printf("FW Section done.\n");
		}
		break;
	}

	return rv;
}

static int
firmware_chunk_cb(void *cookie, const unsigned char *chunk,
    unsigned long chunk_len)
{
	struct fw_chunk_state *cs = cookie;
	uint32_t copylen;

#if 0
	static uint32_t addr;
	const unsigned char *c = chunk;
	printf("%05lx: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		addr, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
	    c[8], c[9], c[10], c[11], c[12], c[13], c[14], c[15]);
	addr += 16;
	c += 16;
	printf("%05lx: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		addr, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
	    c[8], c[9], c[10], c[11], c[12], c[13], c[14], c[15]);
	addr += 16;
#endif

	while (chunk_len) {
		copylen = FW_CHUNK_SIZE - cs->cs_chunklen;
		if (copylen > chunk_len)
			copylen = chunk_len;
		memcpy(&cs->cs_chunkbuff[cs->cs_chunklen], chunk, copylen);
		cs->cs_chunklen += copylen;
		chunk_len -= copylen;

		while (cs->cs_chunklen &&
		    (copylen = firmware_chunk_state_machine(cs)) != 0) {
			if (copylen < cs->cs_chunklen) {
				memmove(cs->cs_chunkbuff,
				    &cs->cs_chunkbuff[copylen],
				    cs->cs_chunklen - copylen);
				cs->cs_chunklen -= copylen;
			} else {
				cs->cs_chunklen = 0;
			}
		}
	}

	return 0;
}

sint8
firmware_download(void)
{
	struct fw_chunk_state *cs;
	sint8 s8Ret;
	uint32 u32Val;
	int rv;

	if ((cs = malloc(sizeof(*cs))) == NULL)
		return M2M_ERR_FAIL;

	/* Set Mux Select for Wifi CPU Reset */
	s8Ret = nm_read_reg_with_ret(0x1118, &u32Val);
	u32Val |= 1;
	s8Ret += nm_write_reg(0x1118, u32Val);
	
	/* Assert CPU reset */
	s8Ret += nm_read_reg_with_ret(0x1400, &u32Val);
	u32Val &= ~(1ul << 10);
	s8Ret += nm_write_reg(0x1400, u32Val);

	/* Boot from IRAM, not ROM*/
	s8Ret += nm_write_reg(0xc0000,0x71);

	if(s8Ret != M2M_SUCCESS)
		return s8Ret;

	cs->cs_chunklen = 0;
	cs->cs_state = FW_CHUNK_STATE_SECADDRESS;
	rv = platform_wilc_fw_decompress(firmware_chunk_cb, cs, FW_CHUNK_SIZE);
	s8Ret = (rv >= 0 && cs->cs_chunklen == 0) ? M2M_SUCCESS : M2M_ERR_FAIL;

	free(cs);

	return s8Ret;
}
#endif /* CONF_WILC_COMPRESSED_FIRMWARE */

#if (defined CONF_WILC_USE_3000_REV_A && !defined WILC_SERIAL_BRIDGE_INTERFACE)
sint8 firmware_download_bt(void)
{
	sint8 s8Ret;
	uint32 u32SecSize, u32SecAddress, u32Val;
	uint8_t* pu8FirmwareBuffer;
	sint32 BuffIndex = 0, CurrentSecSize = 0;
	uint8_t u8TransferChunk[32], ChunkSize = 32;

	/* Set Mux Select for BT CPU Reset */
	s8Ret = nm_read_reg_with_ret(0x3b0090, &u32Val);
	u32Val |= 1;
	s8Ret += nm_write_reg(0x3b0090, u32Val);
	
	/* Assert CPU reset */
	s8Ret += nm_read_reg_with_ret(0x3b0400, &u32Val);
	u32Val &= ~(1ul << 2);
	s8Ret += nm_write_reg(0x3b0400, u32Val);

	/* Boot from IRAM, not ROM*/
	s8Ret += nm_write_reg(0x4f0000,0x71);

	pu8FirmwareBuffer = (uint8_t *)firmware_ble;
	u32SecSize = sizeof(firmware_ble);
	
	M2M_DBG("BT firmware size = %d\n", u32SecSize);
	ChunkSize = 32;
	u32SecAddress = 0x400000;
	CurrentSecSize = u32SecSize;
	while(CurrentSecSize>0)
	{
		if(CurrentSecSize < ChunkSize)
			ChunkSize = CurrentSecSize;
		
		m2m_memcpy(u8TransferChunk,pu8FirmwareBuffer+BuffIndex,ChunkSize);
		nm_write_block(u32SecAddress,u8TransferChunk,ChunkSize);
		u32SecAddress += ChunkSize;
		BuffIndex += ChunkSize;
		CurrentSecSize -= ChunkSize;
	}		
	pu8FirmwareBuffer += BuffIndex;

	return s8Ret;
}
#endif
#else

sint8 wait_for_bootrom(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;
	
	reg = 0;
	while(1) {
		reg = nm_read_reg(0x1014);	/* wait for efuse loading done */
		if (reg & 0x80000000) {
			break;
		}
		nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
	}
	reg = nm_read_reg(M2M_WAIT_FOR_HOST_REG);
	reg &= 0x1;

	/* check if waiting for the host will be skipped or not */
	if(reg == 0)
	{
		reg = 0;
		while(reg != M2M_FINISH_BOOT_ROM)
		{
			nm_bsp_sleep(1);
			reg = nm_read_reg(BOOTROM_REG);

			if(++cnt > TIMEOUT)
			{
				M2M_DBG("failed to load firmware from flash.\n");
				ret = M2M_ERR_INIT;
				goto ERR2;
			}
		}
	}

	nm_write_reg(BOOTROM_REG,M2M_START_FIRMWARE);

ERR2:
	return ret;
}
#endif /*CONF_WILC_FW_IN_FLASH */
sint8 wait_for_firmware_start(void)
{
	sint8 ret = M2M_SUCCESS;
	uint32 reg = 0, cnt = 0;;
	
	while (reg != M2M_FINISH_INIT_STATE)
	{
		nm_bsp_sleep(1); /* TODO: Why bus error if this delay is not here. */
		M2M_DBG("%x %x %x\n",(unsigned int)nm_read_reg(0x108c),(unsigned int)nm_read_reg(0x108c),(unsigned int)nm_read_reg(0x14A0));
		reg = nm_read_reg(NMI_STATE_REG);
		if(++cnt > TIMEOUT)
		{
			M2M_DBG("Time out for wait firmware Run\n");
			ret = M2M_ERR_INIT;
			goto ERR;
		}
	}
	nm_write_reg(NMI_STATE_REG,0);
ERR:
	return ret;
}

sint8 chip_deinit(void)
{
	uint32 reg = 0;
	sint8 ret;
	uint8 timeout = 10;

	/**
	stop the firmware, need a re-download
	**/
	ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
	if (ret != M2M_SUCCESS) {
		M2M_ERR("failed to de-initialize\n");
	}
	reg &= ~(1 << 10);
	ret = nm_write_reg(NMI_GLB_RESET_0, reg);

	if (ret != M2M_SUCCESS) {
		M2M_ERR("Error while writing reg\n");
		return ret;
	}

	do {
		ret = nm_read_reg_with_ret(NMI_GLB_RESET_0, &reg);
		if (ret != M2M_SUCCESS) {
			M2M_ERR("Error while reading reg\n");
			return ret;
		}
		/*Workaround to ensure that the chip is actually reset*/
		if ((reg & (1 << 10))) {
			M2M_DBG("Bit 10 not reset retry %d\n", timeout);
			reg &= ~(1 << 10);
			ret = nm_write_reg(NMI_GLB_RESET_0, reg);
			timeout--;
		} else {
			break;
		}

	} while (timeout);
	
	return ret;		
}

sint8 set_gpio_dir(uint8 gpio, uint8 dir)
{
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20108, &val32);
	if(ret != M2M_SUCCESS) goto _EXIT;
	
	if(dir) {
		val32 |= (1ul << gpio);
	} else {
		val32 &= ~(1ul << gpio);
	}

	ret = nm_write_reg(0x20108, val32);

_EXIT:
	return ret;
}
sint8 set_gpio_val(uint8 gpio, uint8 val)
{
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20100, &val32);
	if(ret != M2M_SUCCESS) goto _EXIT;
	
	if(val) {
		val32 |= (1ul << gpio);
	} else {
		val32 &= ~(1ul << gpio);
	}

	ret = nm_write_reg(0x20100, val32);

_EXIT:
	return ret;
}

sint8 get_gpio_val(uint8 gpio, uint8* val)
{
	uint32 val32;
	sint8 ret;

	ret = nm_read_reg_with_ret(0x20104, &val32);
	if(ret != M2M_SUCCESS) goto _EXIT;
	
	*val = (uint8)((val32 >> gpio) & 0x01);

_EXIT:
	return ret;
}

sint8 pullup_ctrl(uint32 pinmask, uint8 enable)
{
	sint8 s8Ret;
	uint32 val32;
	s8Ret = nm_read_reg_with_ret(0x142c, &val32);
	if(s8Ret != M2M_SUCCESS) {
		M2M_ERR("[pullup_ctrl]: failed to read\n");
		goto _EXIT;
	}
	if(enable) {
		val32 &= ~pinmask;
		} else {
		val32 |= pinmask;
	}
	s8Ret = nm_write_reg(0x142c, val32);
	if(s8Ret  != M2M_SUCCESS) {
		M2M_ERR("[pullup_ctrl]: failed to write\n");
		goto _EXIT;
	}
_EXIT:
	return s8Ret;
}

sint8 nmi_get_otp_mac_address(uint8 *pu8MacAddr,  uint8 * pu8IsValid)
{
	sint8 ret;
	uint32	u32RegValue;
	
	ret = nm_read_reg_with_ret(rNMI_GP_REG_0, &u32RegValue);
	if(ret != M2M_SUCCESS) goto _EXIT_ERR;
	
	if(!EFUSED_MAC(u32RegValue)) {
		M2M_DBG("Default MAC\n");
		m2m_memset(pu8MacAddr, 0, 6);
		goto _EXIT_ERR;
	}
	
	M2M_DBG("OTP MAC\n");
	u32RegValue >>=16;
	nm_read_block(u32RegValue|0x30000, pu8MacAddr, 6);
	if(pu8IsValid) *pu8IsValid = 1;	
	return ret;

_EXIT_ERR:
	if(pu8IsValid) *pu8IsValid = 0;
	return ret;
}

sint8 nmi_get_mac_address(uint8 *pu8MacAddr)
{
	sint8 ret;
	uint32	u32RegValue;
	
	ret = nm_read_reg_with_ret(rNMI_GP_REG_0, &u32RegValue);
	if(ret != M2M_SUCCESS) goto _EXIT_ERR;

	/*WILCBARESW-69*/
	u32RegValue &=0x0000ffff;
	nm_read_block(u32RegValue|0x30000, pu8MacAddr, 12);
	
	return ret;
	
_EXIT_ERR:
	return ret;
}

sint8 is_valid_gpio(uint8 gpio)
{
#if	defined(CONF_WILC_USE_1000_REV_B)
	return (gpio == 0 || gpio == 1 || gpio == 3 || gpio == 4 || gpio == 6);
#elif defined(CONF_WILC_USE_3000_REV_A)
	return (gpio == 0 || gpio == 3 || gpio == 4 ||gpio == 6 ||
		(gpio >= 17 && gpio <= 20));
#endif
	
	return 0;
}
