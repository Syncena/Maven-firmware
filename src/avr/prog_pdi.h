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

#ifndef PROG_PDI_H
#define PROG_PDI_H

struct prog_pdi_erase_arg {
	uint8_t pea_mem_type;
	uint32_t pea_address;
};
#define PROG_PDI_ERASE_DEVICE			0x00u
#define PROG_PDI_ERASE_APPLICATION		0x01u
#define PROG_PDI_ERASE_BOOTLOADER		0x02u
#define PROG_PDI_ERASE_EEPROM			0x03u
#define PROG_PDI_ERASE_APPLICATION_PAGE		0x04u
#define PROG_PDI_ERASE_BOOTLOADER_PAGE		0x05u
#define PROG_PDI_ERASE_EEPROM_PAGE		0x06u
#define PROG_PDI_ERASE_USER_SIGNATURE		0x07u
#define PROG_PDI_ERASE_CONFIG			0x08u

struct prog_pdi_program_arg {
	uint8_t ppa_mem_type;
	uint8_t ppa_page_mode;
	uint16_t ppa_length;
	uint32_t ppa_address;
	const void *ppa_data;
};
#define PROG_PDI_MEM_TYPE_APPLICATION		0x00u
#define PROG_PDI_MEM_TYPE_BOOTLOADER		0x01u
#define PROG_PDI_MEM_TYPE_EEPROM		0x02u
#define PROG_PDI_MEM_TYPE_FUSE			0x03u
#define PROG_PDI_MEM_TYPE_LOCKBITS		0x04u
#define PROG_PDI_MEM_TYPE_USER_SIGNATURE	0x05u
#define PROG_PDI_MEM_TYPE_FACTORY_CALIB		0x06u

#define	PROG_PDI_PAGEMODE_ERASE			(1u << 0)
#define	PROG_PDI_PAGEMODE_WRITE			(1u << 1)

struct prog_pdi_read_arg {
	uint8_t pra_mem_type;
	uint16_t pra_length;
	uint32_t pra_address;
	void *pra_buffer;
};

struct prog_pdi_crc_arg {
	uint8_t pca_crc_type;
};
#define PROG_PDI_CRC_APPLICATION		0x00u
#define PROG_PDI_CRC_BOOTLOADER			0x01u
#define PROG_PDI_CRC_ALL_FLASH			0x02u

union prog_pdi_param_arg {
	uint32_t ppa_nvm_base;
	uint16_t ppa_ee_page_size;
	uint8_t ppa_nvm_cmd_addr;
	uint8_t ppa_nvm_sr_addr;
};
#define	PROG_PDI_PARAM_NVMBASE			0x00u
#define	PROG_PDI_PARAM_EEPAGESIZE		0x01u
#define	PROG_PDI_PARAM_NVMCMD_REG		0x02u
#define	PROG_PDI_PARAM_NVMCSR_REG		0x03u

#define PROG_PDI_STATUS_OK			0x00u
#define PROG_PDI_STATUS_FAILURE			0x01u
#define PROG_PDI_STATUS_COLLISION		0x02u
#define PROG_PDI_STATUS_TIMEDOUT		0x03u

#ifndef CONFIG_USE_CONFIGDB
#define	PROG_PDI_GLOBAL_VARS		\
	uint32_t glob_pdi_clock;
#endif

extern void *prog_pdi_init(void *cookie);
extern void prog_pdi_quiesce(void *);
extern void prog_pdi_reset_comms(void *);
extern void prog_pdi_load_defaults(void);

extern int	prog_pdi_set_param(void *, int,
			const union prog_pdi_param_arg *);
extern int	prog_pdi_enter_prog_mode(void *);
extern int	prog_pdi_leave_prog_mode(void *);
extern int	prog_pdi_erase(void *, const struct prog_pdi_erase_arg *);
extern int	prog_pdi_program(void *, const struct prog_pdi_program_arg *);
extern int	prog_pdi_read(void *, const struct prog_pdi_read_arg *);
//extern int	prog_pdi_crc(void *, const struct prog_pdi_crc_arg *);

#endif /* PROG_PDI_H */
