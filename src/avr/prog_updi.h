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

#ifndef PROG_UPDI_H
#define PROG_UPDI_H

struct prog_updi_erase_arg {
	uint8_t uea_mem_type;
	uint16_t uea_address;
};
#define PROG_UPDI_ERASE_DEVICE			0x00
#define PROG_UPDI_ERASE_EEPROM			0x01
#define PROG_UPDI_ERASE_USERSIG			0x02
#define PROG_UPDI_ERASE_CONFIG			0x03

struct prog_updi_program_arg {
	uint8_t upa_mem_type;
	uint8_t upa_mode;
	uint16_t upa_length;
	uint16_t upa_address;
	const void *upa_data;
};
#define PROG_UPDI_MEM_TYPE_FLASH		0x00
#define PROG_UPDI_MEM_TYPE_EEPROM		0x01
#define PROG_UPDI_MEM_TYPE_FUSE			0x02
#define PROG_UPDI_MEM_TYPE_LOCKBITS		0x03
#define PROG_UPDI_MEM_TYPE_USERROW		0x04
#define PROG_UPDI_MEM_TYPE_SIGROW		0x05
#define PROG_UPDI_MEM_TYPE_SIB			0x06

#define	PROG_UPDI_MODE_ERASE_FIRST		(1u << 0)
#define	PROG_UPDI_MODE_WORDS			(1u << 1)

struct prog_updi_read_arg {
	uint8_t ura_mem_type;
	uint16_t ura_length;
	uint16_t ura_address;
	void *ura_buffer;
};

struct prog_updi_param_arg {
	uint16_t upa_flash_base;
	uint16_t upa_nvm_addr;
	uint16_t upa_ocd_addr;
	uint8_t upa_flash_page_size;
	uint8_t upa_ee_page_size;
};

#define PROG_UPDI_STATUS_OK			0x00
#define PROG_UPDI_STATUS_FAILURE		0x01
#define PROG_UPDI_STATUS_COLLISION		0x02
#define PROG_UPDI_STATUS_TIMEDOUT		0x03

#ifndef CONFIG_USE_CONFIGDB
#define	PROG_UPDI_GLOBAL_VARS		\
	uint32_t glob_updi_baud;
#endif

extern void *prog_updi_init(void *cookie);
extern void prog_updi_quiesce(void *);
extern void prog_updi_reset_comms(void *);
extern void prog_updi_load_defaults(void);

extern int	prog_updi_get_sync(void *);
extern int	prog_updi_set_param(void *, const struct prog_updi_param_arg *);
extern int	prog_updi_enter_prog_mode(void *);
extern int	prog_updi_leave_prog_mode(void *);
extern int	prog_updi_erase(void *, const struct prog_updi_erase_arg *);
extern int	prog_updi_program(void *, const struct prog_updi_program_arg *);
extern int	prog_updi_read(void *, const struct prog_updi_read_arg *);

#endif /* PROG_UPDI_H */
