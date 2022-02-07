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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtos.h"
#include "timer.h"
#include "prog_isp.h"
#include "avr_ll.h"
#include "hardware.h"
#include "shell.h"
#include "programmer.h"
#ifndef CONFIG_USE_CONFIGDB
#include "globals.h"
#else
#include "configdb.h"
#endif

#define	PROG_ISP_DEBUG_OFF	0u
#define	PROG_ISP_DEBUG_ON	1u
#define	PROG_ISP_DEBUG_DUMP	2u

//#define	DEBUG_FLAG_INIT	0
//#define	DEBUG_FLAG	prog_isp_debug
#include "debug.h"

struct isp_state {
	bool is_active;
	bool is_started;
	bool is_slow_clock;
	volatile bool is_timer_expired;
	uint32_t is_clock_delay;
	float is_sck_duration;
	bool is_sck_duration_set;
	rtos_timer_t is_timer;
};
static struct isp_state isp_state;

SHELL_CMD_DECL(isp, isp_shell_cmd, "Configure ISP programmer");

/*
 * Used during flash/eeprom programing
 */
#define	PROG_ISP_MODE_PAGED		(1u << 0)
#define	PROG_ISP_MODE_TIMED_DELAY	(1u << 1)
#define	PROG_ISP_MODE_VALUE_POLLING	(1u << 2)
#define	PROG_ISP_MODE_RDY_BSY_POLLING	(1u << 3)
#define	PROG_ISP_MODE_COMMIT		(1u << 7)
#define	PROG_ISP_MODE_COMPLETION_MASK	(PROG_ISP_MODE_TIMED_DELAY |	\
					 PROG_ISP_MODE_VALUE_POLLING |	\
					 PROG_ISP_MODE_RDY_BSY_POLLING)

#define	PROG_ISP_HIGH_BYTE_CMD_BIT	(1u << 3)

#define	ISP_CMD_LOAD_EXTENDED_ADDRESS	0x4du

static void
prog_isp_ticks_delay(uint32_t ticks)
{

	if (ticks)
		timer_buzz_delay_us(ticks);
}

static void
prog_isp_timer_callback(rtos_timer_t handle)
{
	struct isp_state *is;

	is = rtos_timer_get_id(handle);

	is->is_timer_expired = true;
}

static uint8_t
prog_isp_transfer(struct isp_state *is, const uint8_t *p, uint8_t len)
{
	rtos_saved_ipl_t ipl;
	uint8_t rv = 0;

	ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
	while (len--)
		rv = avr_ll_isp_xfer(*p++, is->is_clock_delay);
	timer_delay_cycles(is->is_clock_delay);
	rtos_ipl_restore(ipl);

	return rv;
}

static void
prog_isp_timer_start(struct isp_state *is, uint8_t ms)
{

	rtos_timer_stop(is->is_timer);
	rtos_timer_change_wait_period(is->is_timer, ms);
	is->is_timer_expired = false;
	rtos_timer_start(is->is_timer);
}

static void
prog_isp_timer_stop(struct isp_state *is)
{

	rtos_timer_stop(is->is_timer);
	is->is_timer_expired = false;
}

static uint8_t
prog_isp_wait_until_ready(struct isp_state *is, uint8_t timeout)
{
	uint8_t status, rv;
	uint8_t cmd_buff[4];

	if (timeout == 0)
		timeout = 1;

	rv = PROG_ISP_STATUS_TIMEOUT;

	prog_isp_timer_start(is, timeout);

	do {
		cmd_buff[0] = 0xf0u;
		cmd_buff[1] = 0x00u;
		cmd_buff[2] = 0x00u;
		cmd_buff[3] = 0x00u;

		status = prog_isp_transfer(is, cmd_buff, sizeof(cmd_buff));

		if ((status & 1) == 0)
			rv = PROG_ISP_STATUS_OK;
	} while (rv == PROG_ISP_STATUS_TIMEOUT && !is->is_timer_expired);

	prog_isp_timer_stop(is);

	return rv;
}

static void
prog_isp_detach(struct isp_state *is, uint8_t rst_delay)
{
	rtos_saved_ipl_t ipl;

	if (is->is_active) {
		DBFPRINTF("detaching\n");

		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		avr_ll_isp_sck_low();
		avr_ll_isp_mosi_low();
		avr_ll_isp_rst_low();
		rtos_ipl_restore(ipl);

		if (rst_delay)
			rtos_task_sleep((rtos_tick_t) rst_delay);

		ipl = rtos_ipl_raise(HW_IPL_SCHEDULER);
		avr_ll_isp_clk_stop();
		avr_ll_isp_stop();

		is->is_active = false;
		rtos_ipl_restore(ipl);
	}
}

static uint32_t
prog_isp_sck_to_delay(struct isp_state *is, float sck)
{
	uint32_t rv;

	/*
	 * 'sck' specifies the desired clock period, in microseconds.
	 */

	if (is->is_slow_clock) {
		/* Override: Clock < 8KHz */
		sck = (1.0 / 8000.0) * 1000000.0;
	}

	/*
	 * Work out the number of buzz-loops per SCK phase in avr_ll_isp_xfer().
	 *
	 * XXX: Note, this is very closely tied to the loop in hwp_isp_xfer()
	 * The "0.217" accounts for loop overhead.
	 *
	 * While we allow very small values to be configured, it's worth
	 * noting that 'is_clock_delay' of 1 will result in an SPI clock
	 * of about 4.4 MHz. This is unlikely to work with any degree of
	 * success...
	 */
	sck = (21500.0F * (sck - 0.217F)) / 1000.0F;
	sck /= 2.0F;

	rv = (uint32_t)sck;

	/*
	 * Constrain it some something reasonable (about 1 MHz) on 120 MHz M4.
	 */
	if (rv < 7)
		rv = 7;

	return rv;
}

void
prog_isp_update_sck(void *arg, float sck_duration)
{
	struct isp_state *is = arg;

	DBFPRINTF("was %lu, now %lu\n", is->is_clock_delay,
	    prog_isp_sck_to_delay(is, sck_duration));

	is->is_sck_duration = sck_duration;
	is->is_sck_duration_set = true;
}

int
prog_isp_start(void *arg, const struct prog_isp_start_params *isp)
{
	struct isp_state *is = arg;
	rtos_saved_ipl_t ipl;
	uint8_t i, rv = 0;
	uint32_t isp_clock;
	float sck;

#ifndef CONFIG_USE_CONFIGDB
	isp_clock = glob.glob_isp_clock;
#else
	if (cf_get_uint32(CF_KEY_UINT32_ISP_CLOCK, &isp_clock) == 0)
		isp_clock = 0;
#endif

	if (is->is_active)
		return PROG_ISP_STATUS_ALREADY_ACTIVE;

	if (isp_clock == 0) {
		if (is->is_sck_duration_set == false)
			sck = isp->isp_sck_duration;
		else
			sck = is->is_sck_duration;
	} else {
		sck = (1.0F / (float)isp_clock) * 1000000.0F;
	}

	is->is_clock_delay = prog_isp_sck_to_delay(is, sck);

	programmer_curr_clock_ticks = 300;	/* 30 seconds */
	programmer_curr_clock_name = "ISPsck";
	programmer_curr_clock_rate = (uint32_t)(1000000.0F / sck);

	/* Configure hardware for ISP mode */
	avr_ll_isp_start();

	/* Start CLK */
	if (isp->isp_clock)
		avr_ll_isp_clk_start(isp->isp_clock);

	if (isp->isp_stab_delay == 0)
		timer_delay_cycles(is->is_clock_delay);
	else
		rtos_task_sleep((rtos_tick_t) isp->isp_stab_delay);

	is->is_active = true;

	DBFPRINTF("pulsing RESET\n");

	/* Send +ve RST pulse. */
	ipl = rtos_ipl_raise(HW_IPL_MAX);
	avr_ll_isp_rst_high();
	timer_delay_cycles(is->is_clock_delay);
	avr_ll_isp_rst_low();
	rtos_ipl_restore(ipl);

	rtos_task_sleep((rtos_tick_t) isp->isp_cmd_delay);

	DBFPRINTF("sending command %02x.%02x.%02x.%02x idx %u\n",
	    isp->isp_command[0], isp->isp_command[1], isp->isp_command[2],
	    isp->isp_command[3], isp->isp_poll_idx);

	for (i = (isp->isp_sync_loops ? isp->isp_sync_loops : 32); i; i--) {
		rv = prog_isp_transfer(is, isp->isp_command, isp->isp_poll_idx);

		if (isp->isp_poll_idx < 4) {
			prog_isp_transfer(is,
			    isp->isp_command + isp->isp_poll_idx,
			    sizeof(isp->isp_command) - isp->isp_poll_idx);
		}

		if (rv == isp->isp_poll_val) {
			DBFPRINTF("success\n");
			is->is_started = true;
			return PROG_ISP_STATUS_OK;
		}

		ipl = rtos_ipl_raise(HW_IPL_MAX);
		avr_ll_isp_sck_high();
		prog_isp_ticks_delay(300);
		avr_ll_isp_sck_low();
		prog_isp_ticks_delay(300);
		rtos_ipl_restore(ipl);
	}

	DBFPRINTF("got %02x, expecting %02x\n", rv, isp->isp_poll_val);

	prog_isp_detach(is, 0);

	DBFPRINTF("failed\n");

	return PROG_ISP_STATUS_FAILED;
}

int
prog_isp_finish(void *arg, const struct prog_isp_finish_params *ifp)
{
	struct isp_state *is = arg;

	if (!is->is_active)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	DBFPRINTF("clearing up\n");
	prog_isp_detach(is, ifp->ifp_delay_pre);
	rtos_task_sleep((rtos_tick_t) ifp->ifp_delay_post);

	is->is_started = false;

	return PROG_ISP_STATUS_OK;
}

int
prog_isp_chip_erase(void *arg, const struct prog_isp_chip_erase_params *ice)
{
	struct isp_state *is = arg;
	int rv;

	if (!is->is_active)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	DBFPRINTF("issuing erase command\n");
	prog_isp_transfer(is, ice->ice_command, sizeof(ice->ice_command));

	if (ice->ice_poll) {
		DBFPRINTF("polling for completion\n");
		rv = prog_isp_wait_until_ready(is,
		    (ice->ice_delay >= 100) ? ice->ice_delay : 100);
	} else {
		DBFPRINTF("sleeping for completion (%u uS)\n", ice->ice_delay);
		rtos_task_sleep((rtos_tick_t) ice->ice_delay);
		rv = PROG_ISP_STATUS_OK;
	}

	DBFPRINTF("done\n");
	return rv;
}

static int
prog_isp_wait_for_complete(struct isp_state *is, uint8_t mode,
    uint8_t *xfer_buff, uint8_t poll_value, uint8_t delay)
{
	int rv = PROG_ISP_STATUS_OK;

	switch (mode & PROG_ISP_MODE_COMPLETION_MASK) {
	case PROG_ISP_MODE_TIMED_DELAY:
		rtos_task_sleep((rtos_tick_t) delay);
		break;

	case PROG_ISP_MODE_VALUE_POLLING:
		if (delay <= 100)
			delay = 100;

		prog_isp_timer_start(is, delay);

		while (prog_isp_transfer(is, xfer_buff, 4) != poll_value) {
			if (is->is_timer_expired) {
				rv = PROG_ISP_STATUS_TIMEOUT;
				break;
			}
		}

		prog_isp_timer_stop(is);
		break;

	case PROG_ISP_MODE_RDY_BSY_POLLING:
		rv = prog_isp_wait_until_ready(is, delay);
	}

	return rv;
}

int
prog_isp_memory_program(void *arg,
    const struct prog_isp_memory_program_params *imp)
{
	struct isp_state *is = arg;
	uint32_t addr;
	uint16_t len, i, poll_addr;
	const uint8_t *data;
	bool load_extended;
	uint8_t mode, commands[3], poll_value;
	uint8_t xfer_buff[4];
	int rv;

	if (!is->is_started)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	rv = PROG_ISP_STATUS_OK;

	data = imp->imp_data;
	len = imp->imp_len;
	addr = imp->imp_address;
	load_extended = imp->imp_extended_address && !imp->imp_is_eeprom;
	commands[0] = imp->imp_command[0];
	commands[1] = imp->imp_command[1];
	commands[2] = imp->imp_command[2];

	poll_value = imp->imp_is_eeprom ? imp->imp_poll[1] : imp->imp_poll[0];
	poll_addr = 0;

	/*
	 * Page/word programming modes have similar bits; they're just
	 * in different places in the mode byte. Normalise...
	 */
	if (imp->imp_mode & PROG_ISP_MODE_PAGED) {
		/* Page mode */
		mode = ((imp->imp_mode >> 3) & PROG_ISP_MODE_COMPLETION_MASK);
		mode |= (imp->imp_mode & PROG_ISP_MODE_COMMIT);
		mode |= PROG_ISP_MODE_PAGED;
	} else {
		mode = imp->imp_mode & PROG_ISP_MODE_COMPLETION_MASK;
	}

	DBFPRINTF("len %u, imp_mode %02x, addr %08lx\n", len, imp->imp_mode,
	    addr);
	DBFPRINTF("mode %02x, cmd %02x %02x %02x\n", mode, imp->imp_command[0],
	    imp->imp_command[1], imp->imp_command[2]);
	DBMFPRINTF(PROG_ISP_DEBUG_DUMP, "Data:");

	for (i = 0; i < len; i++) {
		/*
		 * For extended addresses, ensure bits 16-23 are loaded into
		 * the target.
		 */
		if (load_extended) {
			load_extended = false;
			xfer_buff[0] = ISP_CMD_LOAD_EXTENDED_ADDRESS;
			xfer_buff[1] = 0x00;
			xfer_buff[2] = (addr >> 16) & 0xffu;
			xfer_buff[3] = 0x00;
			prog_isp_transfer(is, xfer_buff, sizeof(xfer_buff));
		}

		/*
		 * Send the current byte
		 */
		xfer_buff[0] = commands[0];
		xfer_buff[1] = (addr >> 8) & 0xffu;
		xfer_buff[2] = (addr >> 0) & 0xffu;
		xfer_buff[3] = data[i];
		prog_isp_transfer(is, xfer_buff, sizeof(xfer_buff));

		DBMPRINTF(PROG_ISP_DEBUG_DUMP, " %02x", data[i]);

		/*
		 * For Flash, modify the write command to indicate hi/lo byte
		 */
		if (!imp->imp_is_eeprom)
			commands[0] ^= PROG_ISP_HIGH_BYTE_CMD_BIT;

		/*
		 * Modify the Flash read command similarly
		 */
		if (poll_addr == 0 && data[i] != poll_value) {
			if ((i & 1) != 0 && !imp->imp_is_eeprom)
				commands[2] |= PROG_ISP_HIGH_BYTE_CMD_BIT;
			else
				commands[2] &= ~PROG_ISP_HIGH_BYTE_CMD_BIT;

			poll_addr = addr & 0xffffu;
		}

		/*
		 * For non-paged writes, we need to poll for
		 * completion on each and every byte sent.
		 */
		if ((mode & PROG_ISP_MODE_PAGED) == 0) {
			/* Might need to switch to time delay polling */
			if (poll_addr == 0 &&
			    (mode & PROG_ISP_MODE_RDY_BSY_POLLING) == 0) {
				mode &= ~PROG_ISP_MODE_VALUE_POLLING;
				mode |= PROG_ISP_MODE_TIMED_DELAY;
			}

			xfer_buff[0] = commands[2];
			xfer_buff[1] = (poll_addr >> 8) & 0xffu;
			xfer_buff[2] = (poll_addr >> 0) & 0xffu;

			rv = prog_isp_wait_for_complete(is, mode, xfer_buff,
			    poll_value, imp->imp_delay);
			if (rv != PROG_ISP_STATUS_OK)
				break;

			poll_addr = 0;
		}

		/*
		 * Bump the address. For Flash, this is every 2nd byte
		 * since it's word-addressed. For EEPROM, do it every time.
		 */
		if ((i & 1) != 0 || imp->imp_is_eeprom) {
			addr += 1;

			/*
			 * For Flash check if we've crossed the 64K
			 * boundary, in which case send an extended
			 * address command next time around.
			 */
			if (!imp->imp_is_eeprom && (addr & 0xffff) == 0)
				load_extended = true;
		}
	}

	DBMPRINTF(PROG_ISP_DEBUG_DUMP, "\n");

	/*
	 * Check if the host wants us to commit the page to memory
	 */
	if ((mode & PROG_ISP_MODE_COMMIT) != 0) {
		addr = imp->imp_address;

		/*
		 * Send the command/address to commit the page
		 */
		xfer_buff[0] = commands[1];
		xfer_buff[1] = (addr >> 8) & 0xffu;
		xfer_buff[2] = (addr >> 0) & 0xffu;
		xfer_buff[3] = 0;

		prog_isp_transfer(is, xfer_buff, sizeof(xfer_buff));

		/*
		 * If the host wants us to poll for a value but we have
		 * no polling address, switch to timed mode.
		 */
		if ((mode & PROG_ISP_MODE_VALUE_POLLING) != 0 &&
		    poll_addr == 0) {
			mode = (mode & ~PROG_ISP_MODE_VALUE_POLLING) |
			    PROG_ISP_MODE_TIMED_DELAY;
		}

		rv = prog_isp_wait_for_complete(is, mode, xfer_buff,
		    poll_value, imp->imp_delay);
	}

	return rv;
}

int
prog_isp_memory_read(void *arg, const struct prog_isp_memory_read_params *imr)
{
	struct isp_state *is = arg;
	uint32_t addr;
	uint16_t len, i;
	uint8_t *buff;
	uint8_t cmd_buff[4];
	uint8_t v;

	if (!is->is_started)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	buff = imr->imr_data;
	addr = imr->imr_address;
	len = imr->imr_len;
	cmd_buff[3] = 0x00;

	DBMFPRINTF(PROG_ISP_DEBUG_DUMP, "len %u, addr %08lx:", len,addr);

	if (!imr->imr_is_eeprom && imr->imr_extended_address) {
		cmd_buff[0] = ISP_CMD_LOAD_EXTENDED_ADDRESS;
		cmd_buff[1] = 0x00;
		cmd_buff[2] = (addr >> 16) & 0xffu;
		prog_isp_transfer(is, cmd_buff, sizeof(cmd_buff));
	}

	for (i = 0; i < len; i++) {
		cmd_buff[0] = imr->imr_command & ~0x08u;
		cmd_buff[1] = (addr >> 8) & 0xffu;
		cmd_buff[2] = (addr >> 0) & 0xffu;

		if (!imr->imr_is_eeprom) {
			if (i & 1) {
				cmd_buff[0] |= 0x08u;
				addr++;
			}
		} else {
			addr++;
		}

		v = prog_isp_transfer(is, cmd_buff, sizeof(cmd_buff));
		*buff++ = v;
		DBMPRINTF(PROG_ISP_DEBUG_DUMP, " %02x", v);
	}

	DBMPRINTF(PROG_ISP_DEBUG_DUMP, "\n");

	return PROG_ISP_STATUS_OK;
}

int
prog_isp_fuse_program(void *arg, const struct prog_isp_fuse_program_params *ifp)
{
	struct isp_state *is = arg;

	if (!is->is_started)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	prog_isp_transfer(is, ifp->ifp_command, sizeof(ifp->ifp_command));

	/* XXX: Should we wait until ready here? */
	/* XXX: What about ifp->ifp_is_lock ? */

	return PROG_ISP_STATUS_OK;
}

int
prog_isp_fuse_read(void *arg, const struct prog_isp_fuse_read_params *ifr,
    uint8_t *value)
{
	struct isp_state *is = arg;
	uint8_t rv;

	if (!is->is_started)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	DBFPRINTF("addr %02x, cmd %02x.%02x.%02x.%02x\n", ifr->ifr_addr,
	    ifr->ifr_command[0], ifr->ifr_command[1],
	    ifr->ifr_command[2], ifr->ifr_command[3]);

	rv = prog_isp_transfer(is, ifr->ifr_command, ifr->ifr_addr);
	if (ifr->ifr_addr < 4) {
		DBFPRINTF("extra read\n");
		prog_isp_transfer(is, ifr->ifr_command + ifr->ifr_addr,
		    sizeof(ifr->ifr_command) - ifr->ifr_addr);
	}

	DBFPRINTF("rv %u\n", rv);

	*value = rv;

	return PROG_ISP_STATUS_OK;
}

int
prog_isp_spi_multi(void *arg, const struct prog_isp_multi_spi_params *pims)
{
	struct isp_state *is = arg;
	uint16_t txlen, rxlen;
    	const uint8_t *txbuff;
    	uint8_t *rxbuff;

	if (!is->is_started)
		return PROG_ISP_STATUS_NOT_ACTIVE;

	txbuff = pims->ims_txbuff;
	rxbuff = pims->ims_rxbuff;
	txlen = pims->ims_num_tx;
	rxlen = pims->ims_num_rx;

	if (txlen > pims->ims_start_addr)
		txlen = pims->ims_start_addr;

	prog_isp_transfer(is, txbuff, txlen);

	txbuff += txlen;
	txlen = pims->ims_num_tx - txlen;

	while (txlen--) {
		uint8_t v;

		v = prog_isp_transfer(is, txbuff++, 1);

		if (rxlen) {
			rxlen -= 1;
			*rxbuff++ = v;
		}
	}

	while (rxlen--) {
		uint8_t dummy = 0;
		*rxbuff++ = prog_isp_transfer(is, &dummy, 1);
	}

	return PROG_ISP_STATUS_OK;
}

void
prog_isp_quiesce(void *arg)
{
	struct isp_state *is = arg;

	prog_isp_detach(is, 0);
	is->is_started = false;
}

static const char prog_isp_usage[] = {
	"usage:\n"
	"isp\n"
	"\tDisplays current ISP programming clock rate\n"
	"isp clock <rate>\n"
	"\tSets the ISP programming clock to <rate>. This is the bit clock\n"
	"\tused when programming ISP devices and a non-zero value overrides\n"
	"\tany bit clock supplied by the programmer (such as avrdude). The\n"
	"\tdefault is 0, which means use a safe default value or whatever\n"
	"\tvalue is specified by the programmer. Non-zero values must lie\n"
	"\tbetween 1000 and 1000000 inclusive. Take note of the maximum\n"
	"\tsafe value specified in the target device's datasheet in case\n"
	"\tyou exceed the limit it is capable of supporting. Typically this\n"
	"\tis CPU clock / 4 for most ISP-capable targets.\n"
};

static void
isp_shell_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	uint32_t isp_clock, new_clock;

#ifndef CONFIG_USE_CONFIGDB
	isp_clock = glob.glob_isp_clock;
#else
	if (cf_get_uint32(CF_KEY_UINT32_ISP_CLOCK, &isp_clock) == 0)
		isp_clock = 0;
#endif

	if (argc == 2 && strcasecmp(argv[0], "clock") == 0) {
		char *p;

		if (strcasecmp(argv[1], "auto") == 0)
			new_clock = 0;
		else {
			new_clock = strtoul(argv[1], &p, 10);

			if (p == NULL || *p != '\0' || (new_clock != 0 &&
			    (new_clock < 1000 || new_clock > 1000000))) {
				fprintf(os, "Invalid ISP clock. Clock must be "
				    "between 1000 and 1000000\n");
				new_clock = isp_clock;
			}
		}

		if (isp_clock != new_clock) {
#ifndef CONFIG_USE_CONFIGDB
			glob.glob_isp_clock = new_clock;
			glob_save();
#else
			cf_set_uint32(CF_KEY_UINT32_ISP_CLOCK, new_clock);
			configdb_persist();
#endif
			isp_clock = new_clock;
		}
	} else
#ifdef DEBUG_ENABLED
	if (argc >= 1 && strcasecmp(argv[0], "debug") == 0) {
		if (argc >= 2) {
			if (strcasecmp(argv[1], "on") == 0)
				DEBUG_FLAG = PROG_ISP_DEBUG_ON;
			else
			if (strcasecmp(argv[1], "off") == 0)
				DEBUG_FLAG = PROG_ISP_DEBUG_OFF;
			else
			if (strcasecmp(argv[1], "dump") == 0)
				DEBUG_FLAG = PROG_ISP_DEBUG_DUMP;
		}
	} else
#endif /* DEBUG_ENABLED */
	if (argc != 0) {
		fprintf(os, "%s", prog_isp_usage);
		return;
	}

	if (isp_clock == 0)
		fprintf(os, "ISP Clock: Set by programmer. Default 125 KHz\n");
	else
		fprintf(os, "ISP Clock: %ld\n", isp_clock);

#ifdef DEBUG_ENABLED
	fprintf(os, "ISP debug state: %s\n", DEBUG_FLAG ? "on" : "off");
#endif /* DEBUG_ENABLED */

}

void *
prog_isp_init(void *cookie)
{
	char pname[16];

	(void) cookie;	/* XXX: Can probably ditch this */

	strcpy(pname, "ISPTimer");
	isp_state.is_timer = rtos_timer_create(pname, 100, 0,
	    prog_isp_timer_callback, &isp_state);
	assert(isp_state.is_timer != NULL);

	SHELL_CMD_ADD(isp);

	return &isp_state;
}

void
prog_isp_load_defaults(void)
{

	/* Zero means automatic clock selection */
#ifndef CONFIG_USE_CONFIGDB
	glob.glob_isp_clock = 0;
#else
	cf_set_uint32(CF_KEY_UINT32_ISP_CLOCK, 0);
#endif
}
