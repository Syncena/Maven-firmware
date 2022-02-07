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

#ifndef PROG_ISP_H
#define PROG_ISP_H

struct prog_isp_start_params {
	float isp_sck_duration;		/* In micro-seconds */
	uint32_t isp_clock;
	uint8_t isp_timeout;
	uint8_t isp_stab_delay;
	uint8_t isp_cmd_delay;
	uint8_t isp_sync_loops;
	uint8_t isp_byte_delay;
	uint8_t isp_poll_val;
	uint8_t isp_poll_idx;
	uint8_t isp_command[4];
};

struct prog_isp_finish_params {
	uint8_t ifp_delay_pre;
	uint8_t ifp_delay_post;
};

struct prog_isp_chip_erase_params {
	uint8_t ice_delay;
	uint8_t ice_poll;
	uint8_t ice_command[4];
};

struct prog_isp_memory_program_params {
	const void *imp_data;
	uint32_t imp_address;
	uint16_t imp_len;
	uint8_t imp_extended_address;
	uint8_t imp_is_eeprom;
	uint8_t imp_mode;
	uint8_t imp_delay;
	uint8_t imp_command[3];
	uint8_t imp_poll[2];
};

struct prog_isp_memory_read_params {
	void *imr_data;
	uint32_t imr_address;
	uint16_t imr_len;
	uint8_t imr_extended_address;
	uint8_t imr_is_eeprom;
	uint8_t imr_command;
};

struct prog_isp_fuse_program_params {
	uint8_t ifp_is_lock;
	uint8_t ifp_command[4];
};

struct prog_isp_fuse_read_params {
	uint8_t ifr_addr;
	uint8_t ifr_command[4];
};

struct prog_isp_multi_spi_params {
	const uint8_t *ims_txbuff;
	uint8_t *ims_rxbuff;
	uint8_t ims_num_tx;
	uint8_t ims_num_rx;
	uint8_t ims_start_addr;
};

#define PROG_ISP_STATUS_OK		0x00
#define PROG_ISP_STATUS_TIMEOUT		0x01
#define PROG_ISP_STATUS_RDY_BSY_TIMEOUT	0x02
#define PROG_ISP_STATUS_FAILED		0x03
#define PROG_ISP_STATUS_NOT_ACTIVE	0x04
#define PROG_ISP_STATUS_ALREADY_ACTIVE	0x05

extern void	*prog_isp_init(void *cookie);
extern void	prog_isp_quiesce(void *);
extern void	prog_isp_update_sck(void *, float);
extern void	prog_isp_load_defaults(void);

extern int	prog_isp_start(void *, const struct prog_isp_start_params *);
extern int	prog_isp_finish(void *, const struct prog_isp_finish_params *);
extern int	prog_isp_chip_erase(void *,
			const struct prog_isp_chip_erase_params *);
extern int	prog_isp_memory_program(void *,
			const struct prog_isp_memory_program_params *);
extern int	prog_isp_memory_read(void *,
			const struct prog_isp_memory_read_params *);
extern int	prog_isp_fuse_program(void *,
			const struct prog_isp_fuse_program_params *);
extern int	prog_isp_fuse_read(void *,
			const struct prog_isp_fuse_read_params *, uint8_t *);
extern int	prog_isp_spi_multi(void *,
			const struct prog_isp_multi_spi_params *);

#ifndef CONFIG_USE_CONFIGDB
#define PROG_ISP_GLOBAL_VARS		\
        uint32_t glob_isp_clock;
#endif

#endif /* PROG_ISP_H */
