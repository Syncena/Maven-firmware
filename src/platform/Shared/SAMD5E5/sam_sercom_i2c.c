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
#include <stdint.h>
#include <stdio.h>

#include "rtos.h"
#include "platform.h"
#include "timer.h"
#include "zone_alloc.h"
#include "sam_sercom_i2c.h"

#undef SAM_I2C_IRQ_MODE		/* Interrupt mode doesn't work :( */

#define	SAM_I2C_DBG_EXEC	(1u << 0)
#define	SAM_I2C_DBG_BEGIN	(1u << 1)
#define	SAM_I2C_DBG_COMMAND	(1u << 2)
#define	SAM_I2C_DBG_READ	(1u << 3)
#define	SAM_I2C_DBG_WRITE	(1u << 4)
#define	SAM_I2C_DBG_INTERRUPT	(1u << 5)
#define	SAM_I2C_DBG_ERROR	(1u << 6)
#define	SAM_I2C_DBG_INIT	(1u << 7)
#define	SAM_I2C_DBG_ISSUE_CMD	(1u << 8)

#if 0
#define	SAM_I2C_DEBUG	(SAM_I2C_DBG_EXEC |		\
			 SAM_I2C_DBG_BEGIN |		\
			 SAM_I2C_DBG_COMMAND |		\
			 SAM_I2C_DBG_READ |		\
			 SAM_I2C_DBG_WRITE |		\
			 SAM_I2C_DBG_INTERRUPT |	\
			 SAM_I2C_DBG_ERROR |		\
			 SAM_I2C_DBG_ISSUE_CMD)
#endif

#ifdef SAM_I2C_DEBUG
static unsigned int sam_i2c_debug = SAM_I2C_DEBUG;
#define DBPRINTF(lvl,fmt, ...)					\
	do {							\
		if ((lvl) & sam_i2c_debug) {			\
			printf(fmt, ##__VA_ARGS__);		\
		}						\
	} while (0)
#else
#define DBPRINTF(x, ...)	do { } while (0)
#endif

struct sam_i2c_state {
	struct i2c_controller is_driver;
	SercomI2cm *is_regs;
	uint32_t is_baud;
	void (*is_ctl)(sam_sercom_i2c_ctl_t, bool);
	rtos_mutex_t is_mutex;
#ifdef SAM_I2C_IRQ_MODE
	rtos_sema_t is_sema;
#endif
	uint32_t is_expected_bus_state;
	bool is_do_reset;
	uint8_t is_intflag;

	/* Current I2C transaction. */
	i2c_op_t is_op;
	const uint8_t *is_cmd;
	size_t is_cmdlen;
	uint8_t *is_buf;
	size_t is_buflen;
	void (*is_state_fn)(struct sam_i2c_state *, uint16_t);
	uint32_t is_ctrlb;
	int is_retries;
	int is_result;
};

#define	SAM_I2C_BUS_STATE(i)		(((i)->STATUS.reg & \
					 SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> \
					 SERCOM_I2CM_STATUS_BUSSTATE_Pos)
#define	SAM_I2C_BUS_STATE_UNKNOWN	0u
#define	SAM_I2C_BUS_STATE_IDLE		1u
#define	SAM_I2C_BUS_STATE_OWNED		2u
#define	SAM_I2C_BUS_STATE_BUSY		3u

#define	SAM_I2C_COMMAND_NOP		0u
#define	SAM_I2C_COMMAND_REPEAT_START	1u
#define	SAM_I2C_COMMAND_BYTE_READ	2u
#define	SAM_I2C_COMMAND_STOP		3u

static void
sam_i2c_syncbusy(SercomI2cm *i, uint32_t mask)
{

	while ((i->SYNCBUSY.reg & mask) != 0)
		;
}

static void
hw_cycle_delay(uint32_t cycles)
{

	if (cycles) {
		__asm __volatile(
			"1:     subs    %0, %0, #1      \n"
			"       bne.n   1b              \n"
			: "+l" (cycles) : : "cc");
	}
}

static __inline void
sam_ie2_ctl_scl(struct sam_i2c_state *is, bool state, uint32_t cycles)
{

	is->is_ctl(SAM_SERCOM_I2C_CTL_SCL, state);
	hw_cycle_delay(cycles);
}

static __inline void
sam_ie2_ctl_sda(struct sam_i2c_state *is, bool state, uint32_t cycles)
{

	is->is_ctl(SAM_SERCOM_I2C_CTL_SDA, state);
	hw_cycle_delay(cycles);
}

static void
sam_i2c_reset_sequence(struct sam_i2c_state *is)
{

	hw_cycle_delay(400);

	/* Send start */
	sam_ie2_ctl_scl(is, true, 1);			/* SCL high */
	sam_ie2_ctl_sda(is, false, 3);			/* SDA low */
	sam_ie2_ctl_scl(is, false, 1);			/* SCL low */
	sam_ie2_ctl_sda(is, true, 3);			/* SDA high */

	/* Send 9 clocks */
	for (unsigned int i = 0; i < 9; i++) {
		sam_ie2_ctl_scl(is, true, 4);		/* SCL high */
		sam_ie2_ctl_scl(is, false, 4);		/* SCL high */
	}

	/* Send start */
	sam_ie2_ctl_scl(is, true, 1);			/* SCL high */
	sam_ie2_ctl_sda(is, false, 3);			/* SDA low */
	sam_ie2_ctl_scl(is, false, 4);			/* SCL low */

	/* Send stop */
	sam_ie2_ctl_scl(is, true, 1);			/* SCL high */
	sam_ie2_ctl_sda(is, true, 3);			/* SDA high */
	sam_ie2_ctl_scl(is, false, 4);			/* SCL low */

	sam_ie2_ctl_scl(is, true, 4);			/* SCL high */
}

static int
sam_i2c_wait_for_idle_bus(struct sam_i2c_state *is)
{
	timer_timeout_t to;
	int rv = -1;

	/* Wait for the bus to be idle. */
	timer_timeout_start(&to, 500);
	while (!timer_timeout_expired(&to)) {
		sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);

		if (SAM_I2C_BUS_STATE(is->is_regs) == SAM_I2C_BUS_STATE_IDLE) {
			rv = 0;
			break;
		}

		rtos_yield();
	}

	if (rv < 0) {
		DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_wait_for_idle_bus: "
		    "Timeout.\n");
	}

	return rv;
}

static void
sam_i2c_reset(struct sam_i2c_state *is)
{
	SercomI2cm *i;

	is->is_expected_bus_state = SAM_I2C_BUS_STATE_IDLE;
	is->is_do_reset = false;

	DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: relinquish control of "
	    "pins.\n");

	/* Relinquish control of the pins. */
	is->is_ctl(SAM_SERCOM_I2C_CTL_CONTROL, false);

	i = is->is_regs;

	if (i->CTRLA.reg & SERCOM_I2CM_CTRLA_ENABLE) {
		/* Device is already enabled. Disable it. */
		DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: disabling device\n");
		i->CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
		sam_i2c_syncbusy(i, SERCOM_I2CM_SYNCBUSY_ENABLE);
	}

	/* Issue a software reset */
	DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: resetting device.\n");
	i->CTRLA.reg = SERCOM_I2CM_CTRLA_SWRST;
	sam_i2c_syncbusy(i, SERCOM_I2CM_SYNCBUSY_SWRST);

	/* Send the I2C reset sequence on the pins. */
	DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: sending I2C reset "
	    "sequence.\n");
	sam_i2c_reset_sequence(is);

	/* Now initialise the device in I2C master mode */
	i->CTRLA.reg =
	    SERCOM_I2CM_CTRLA_MODE(5) |		/* I2C master mode */
	    SERCOM_I2CM_CTRLA_SCLSM |		/* Clock stretch enabled. */
	    SERCOM_I2CM_CTRLA_SPEED(0) |	/* Normal speed (100 KHz) */
	    SERCOM_I2CM_CTRLA_SDAHOLD(2) |	/* 100-250 nS SDA hold time */
	    SERCOM_I2CM_CTRLA_RUNSTDBY;
	i->CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
	sam_i2c_syncbusy(i, SERCOM_I2CM_SYNCBUSY_SYSOP);
	i->CTRLC.reg = 0;
	i->BAUD.reg = SERCOM_I2CM_BAUD_BAUD(is->is_baud);
	i->CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
	sam_i2c_syncbusy(i, SERCOM_I2CM_SYNCBUSY_ENABLE);

	/* Regain control of the pins. */
	DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: regaining control of "
	    "pins.\n");
	is->is_ctl(SAM_SERCOM_I2C_CTL_CONTROL, true);

	if (sam_i2c_wait_for_idle_bus(is) < 0) {
		DBPRINTF(SAM_I2C_DBG_INIT, "sam_i2c_reset: forcing IDLE.\n");
		i->STATUS.reg = SERCOM_I2CM_STATUS_BUSERR |
		    SERCOM_I2CM_STATUS_ARBLOST | SERCOM_I2CM_STATUS_LOWTOUT |
		    SERCOM_I2CM_STATUS_BUSSTATE(SAM_I2C_BUS_STATE_IDLE) |
		    SERCOM_I2CM_STATUS_MEXTTOUT | SERCOM_I2CM_STATUS_SEXTTOUT |
		    SERCOM_I2CM_STATUS_LENERR;
		sam_i2c_syncbusy(i, SERCOM_I2CM_SYNCBUSY_SYSOP);
	}
}

static int
sam_i2c_acquire_bus(void *cookie)
{
	struct sam_i2c_state *is = cookie;

	rtos_mutex_acquire(is->is_mutex);

	return 0;
}

static void
sam_i2c_release_bus(void *cookie)
{
	struct sam_i2c_state *is = cookie;

	assert(is->is_expected_bus_state == SAM_I2C_BUS_STATE_IDLE);

	rtos_mutex_release(is->is_mutex);
}

static void
sam_i2c_issue_command(struct sam_i2c_state *is, uint32_t cmd)
{
#ifdef SAM_I2C_DEBUG
	static const char *cmd_strings[] = {
		"NOP", "REPEAT_START", "READ_BYTE", "STOP"
	};
#endif

	DBPRINTF(SAM_I2C_DBG_ISSUE_CMD, "sam_i2c_issue_command: %s%s%s%s, "
	    "SYNCBUSY %02x\n", cmd_strings[cmd],
	    (is->is_ctrlb & SERCOM_I2CM_CTRLB_QCEN) ? "/QCEN" : "",
	    (is->is_ctrlb & SERCOM_I2CM_CTRLB_SMEN) ? "/SMEM" : "",
	    (is->is_ctrlb & SERCOM_I2CM_CTRLB_ACKACT) ? "/ACKACT" : "",
	    (unsigned int)is->is_regs->SYNCBUSY.reg);

	if (cmd != SAM_I2C_COMMAND_NOP)
		sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);

	is->is_regs->CTRLB.reg = SERCOM_I2CM_CTRLB_CMD(cmd) | is->is_ctrlb;
}

static void
sam_i2c_state_read(struct sam_i2c_state *is, uint16_t status)
{
	SercomI2cm *i = is->is_regs;

	(void) status;

	/*
	 * 'DATA' already contains a received byte, and the controller has
	 * sent its ACK or NACK according to the previous condition of ACKACT.
	 */

	if (is->is_buflen == 2) {
		/*
		 * For the final byte, flag NACK.
		 */
		DBPRINTF(SAM_I2C_DBG_READ, "sam_i2c_state_read: ACKACT for "
		    "final byte.\n");
		is->is_ctrlb |= SERCOM_I2CM_CTRLB_ACKACT;
		sam_i2c_issue_command(is, SAM_I2C_COMMAND_NOP);
	} else
	if (is->is_buflen == 1) {
		DBPRINTF(SAM_I2C_DBG_READ, "sam_i2c_state_read: Final.\n");
		if (I2C_OP_STOP_P(is->is_op))
			sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		is->is_result = 0;
		is->is_state_fn = NULL;
	}

	sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);
	*(is->is_buf) = (uint8_t) i->DATA.reg;

	DBPRINTF(SAM_I2C_DBG_READ, "sam_i2c_state_read: Received byte %02x"
	    ", %u remaining.\n", (unsigned int)*(is->is_buf),
	    (unsigned int) is->is_buflen - 1);

	is->is_buf++;
	is->is_buflen--;
}

static void
sam_i2c_state_write(struct sam_i2c_state *is, uint16_t status)
{
	SercomI2cm *i = is->is_regs;

	if (status & SERCOM_I2CM_STATUS_RXNACK) {
		DBPRINTF(SAM_I2C_DBG_WRITE | SAM_I2C_DBG_ERROR,
		    "sam_i2c_state_write: NACK received.\n");
		is->is_ctrlb = 0;
		sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		is->is_result = -1;
		is->is_state_fn = NULL;
		return;
	}

	if (is->is_buflen == 0) {
		if (I2C_OP_STOP_P(is->is_op)) {
			DBPRINTF(SAM_I2C_DBG_WRITE, "sam_i2c_state_write: "
			    "Done, with STOP.\n");
			is->is_ctrlb = 0;
			sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		} else {
			DBPRINTF(SAM_I2C_DBG_WRITE, "sam_i2c_state_write: "
			    "Done, no STOP.\n");
		}
		is->is_state_fn = NULL;
		is->is_result = 0;
		return;
	}

	DBPRINTF(SAM_I2C_DBG_WRITE, "sam_i2c_state_write: Sending byte "
	    "%02x, %u remaining.\n",
	    (unsigned int)*(is->is_buf), (unsigned int) is->is_buflen - 1);

	i->DATA.reg = *(is->is_buf);
	is->is_buf++;
	is->is_buflen--;
}

static void
sam_i2c_state_command(struct sam_i2c_state *is, uint16_t status)
{
	SercomI2cm *i = is->is_regs;

	if (status & SERCOM_I2CM_STATUS_RXNACK) {
		DBPRINTF(SAM_I2C_DBG_COMMAND | SAM_I2C_DBG_ERROR,
		    "sam_i2c_state_command: NACK received.\n");
		is->is_ctrlb = 0;
		sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		is->is_result = -1;
		is->is_state_fn = NULL;
		return;
	}


	if (is->is_cmdlen != 0) {
		DBPRINTF(SAM_I2C_DBG_COMMAND, "sam_i2c_state_command: Sending "
		    "byte %02x, %u remaining.\n", (unsigned int)*(is->is_cmd),
		    (unsigned int) is->is_cmdlen - 1);
		i->DATA.reg = (uint32_t) *(is->is_cmd);
		sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);
		is->is_cmd++;
		is->is_cmdlen--;
		return;
	}

	if (is->is_buflen == 0) {
		is->is_result = 0;
		is->is_state_fn = NULL;
		if (I2C_OP_STOP_P(is->is_op)) {
			DBPRINTF(SAM_I2C_DBG_COMMAND, "sam_i2c_state_command: "
			    "Done, with STOP.\n");
			sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		} else {
			DBPRINTF(SAM_I2C_DBG_COMMAND, "sam_i2c_state_command: "
			    "Done, no STOP.\n");
		}
		return;
	}

	if (I2C_OP_READ_P(is->is_op)) {
		DBPRINTF(SAM_I2C_DBG_COMMAND, "sam_i2c_state_command: "
		    "Transition to READ, len %u.\n",
		    (unsigned int)is->is_buflen);
		is->is_ctrlb = 0;
		if (is->is_buflen <= 1)
			is->is_ctrlb |= SERCOM_I2CM_CTRLB_ACKACT;
		if (is->is_buflen >= 1)
			is->is_ctrlb |= SERCOM_I2CM_CTRLB_SMEN;
		sam_i2c_issue_command(is, SAM_I2C_COMMAND_NOP);
		is->is_state_fn = sam_i2c_state_read;
		is->is_intflag = SERCOM_I2CM_INTFLAG_SB |
		    SERCOM_I2CM_INTFLAG_MB |
		    SERCOM_I2CM_INTFLAG_ERROR;
		i->ADDR.reg |= 1u;
		sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);
	} else {
		DBPRINTF(SAM_I2C_DBG_COMMAND, "sam_i2c_state_command: "
		    "Continue to WRITE.\n");
		is->is_state_fn = sam_i2c_state_write;
		sam_i2c_state_write(is, status);
	}
}

static void
sam_i2c_state_begin(struct sam_i2c_state *is, uint16_t status)
{

	/*
	 * Address has been sent. Check if we received an ACK.
	 */
	if (status & SERCOM_I2CM_STATUS_RXNACK) {
		/* We did not receive an ACK. Retry if necessary. */
		uint32_t cmd;

		if (is->is_retries--) {
			/* Repeated start. */
			DBPRINTF(SAM_I2C_DBG_BEGIN | SAM_I2C_DBG_ERROR,
			    "sam_i2c_state_begin: No ACK for address. "
			    "Retrying.\n");
			cmd = SAM_I2C_COMMAND_REPEAT_START;
		} else {
			/* Give up. Issue a stop. */
			DBPRINTF(SAM_I2C_DBG_BEGIN | SAM_I2C_DBG_ERROR,
			    "sam_i2c_state_begin: No ACK for address. "
			    "Giving up.\n");
			is->is_result = -1;
			is->is_state_fn = NULL;
			if (I2C_OP_STOP_P(is->is_op))
				cmd = SAM_I2C_COMMAND_STOP;
			else
				cmd = SAM_I2C_COMMAND_NOP;
		}

		if (cmd)
			sam_i2c_issue_command(is, cmd);
		return;
	}

	/* ACK received. */
	if (is->is_cmdlen != 0) {
		/* There's a command to write. */
		DBPRINTF(SAM_I2C_DBG_BEGIN, "sam_i2c_state_begin: Got ACK. "
		    "Sending command.\n");
		is->is_state_fn = sam_i2c_state_command;
		sam_i2c_state_command(is, status);
		return;
	} else
	if (is->is_buflen == 0) {
		/*
		 * This was just a bus probe, with neither cmd nor data.
		 */
		is->is_result = 0;
		is->is_state_fn = NULL;
		if (I2C_OP_STOP_P(is->is_op)) {
			DBPRINTF(SAM_I2C_DBG_BEGIN, "sam_i2c_state_begin: "
			    "Bus probe, with STOP.\n");
			sam_i2c_issue_command(is, SAM_I2C_COMMAND_STOP);
		} else {
			DBPRINTF(SAM_I2C_DBG_BEGIN, "sam_i2c_state_begin: "
			    "Bus probe, no STOP.\n");
		}
		return;
	}


	if (I2C_OP_READ_P(is->is_op)) {
		DBPRINTF(SAM_I2C_DBG_BEGIN, "sam_i2c_state_begin: Start "
		    "READ.\n");
		is->is_state_fn = sam_i2c_state_read;
	} else {
		DBPRINTF(SAM_I2C_DBG_BEGIN, "sam_i2c_state_begin: Start "
		    "WRITE.\n");
		is->is_state_fn = sam_i2c_state_write;
	}

	is->is_state_fn(is, status);
}

static void
sam_i2c_interrupt(void *arg)
{
	struct sam_i2c_state *is = arg;
	SercomI2cm *i = is->is_regs;
	uint16_t status;
	uint8_t intflag;

	status = i->STATUS.reg;
	intflag = i->INTFLAG.reg;
	i->INTFLAG.reg = intflag;

	if (is->is_state_fn == NULL) {
		DBPRINTF(SAM_I2C_DBG_INTERRUPT, "sam_i2c_interrupt: we "
		    "should be idle.\n");
		i->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
		return;
	}

	do {
		DBPRINTF(SAM_I2C_DBG_INTERRUPT, "sam_i2c_interrupt: STATUS "
		    "%04" PRIx16 ", INTFLAG %02x, INTMASK %02" PRIx16
		    "\n", status, (unsigned int)intflag, i->INTENSET.reg);

		if (status &
		    (SERCOM_I2CM_STATUS_ARBLOST | SERCOM_I2CM_STATUS_BUSERR)) {
			/* The bus has gone awry. */
			DBPRINTF(SAM_I2C_DBG_INTERRUPT | SAM_I2C_DBG_ERROR,
			    "sam_i2c_interrupt: bus is screwed.\n");
			is->is_do_reset = true;
			is->is_result = -1;
			is->is_state_fn = NULL;
			break;
		} else
		if ((intflag & is->is_intflag) == 0) {
			DBPRINTF(SAM_I2C_DBG_INTERRUPT | SAM_I2C_DBG_ERROR,
			    "sam_i2c_interrupt: unexpected bus state. Wanted "
			    "%02x, got %02x.\n", (unsigned int)is->is_intflag,
			    (unsigned int)intflag);
			is->is_do_reset = true;
			is->is_result = -1;
			is->is_state_fn = NULL;
			break;
		}

		is->is_state_fn(is, status);

		status = i->STATUS.reg;
		intflag = i->INTFLAG.reg;
		i->INTFLAG.reg = intflag;
	} while (intflag);

#ifdef SAM_I2C_IRQ_MODE
	if (is->is_state_fn == NULL) {
		DBPRINTF(SAM_I2C_DBG_INTERRUPT, "sam_i2c_interrupt: Notifying "
		    "mainline.\n");
		rtos_sema_give_isr(is->is_sema);
	}
#endif

	DBPRINTF(SAM_I2C_DBG_INTERRUPT, "sam_i2c_interrupt: Done. FLAG "
	    "%02x, STATUS %04" PRIx16 ".\n", (unsigned int)i->INTFLAG.reg,
	    i->STATUS.reg);
}

static int
sam_i2c_exec(void *cookie, i2c_op_t op, i2c_addr_t addr, const void *cmd,
    size_t cmdlen, void *buf, size_t buflen)
{
	struct sam_i2c_state *is = cookie;
	SercomI2cm *i = is->is_regs;
	uint32_t a;

#ifdef SAM_I2C_DEBUG
	const char *opstr;
	if (op == I2C_OP_READ)
		opstr = "READ";
	else
	if (op == I2C_OP_READ_WITH_STOP)
		opstr = "READ-WITH-STOP";
	else
	if (op == I2C_OP_WRITE)
		opstr = "WRITE";
	else
	if (op == I2C_OP_WRITE_WITH_STOP)
		opstr = "WRITE-WITH-STOP";
	else
		opstr = "CRAP";
	DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: %s, addr %02x, cmdlen"
	    "%u, buflen %u\n", opstr, (unsigned int)addr, (unsigned int)cmdlen,
	    (unsigned int)buflen);
#endif

	/* We only support 8-bit addressing. */
	assert(addr < 0x80u);
	assert((addr & 0x78u) != 0x78u);

	if (is->is_do_reset) {
		DBPRINTF(SAM_I2C_DBG_EXEC | SAM_I2C_DBG_ERROR, "sam_i2c_exec: "
		    "reset required.\n");
		sam_i2c_reset(is);
	}

	if (SAM_I2C_BUS_STATE(i) != is->is_expected_bus_state) {
		switch (SAM_I2C_BUS_STATE(i)) {
		case SAM_I2C_BUS_STATE_IDLE:
			DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: bus is idle\n");
			break;
		case SAM_I2C_BUS_STATE_OWNED:
			DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: bus is owned. addr "
			    "%02x, %s\n", (unsigned int)i->ADDR.reg >> 1,
			    (i->ADDR.reg & 1u) ? "READ" : "WRITE");

			if (sam_i2c_wait_for_idle_bus(is) < 0) {
				DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: Timeout "
				    "waiting for idle\n");
				sam_i2c_reset(is);
			}
			break;
		case SAM_I2C_BUS_STATE_UNKNOWN:
			DBPRINTF(SAM_I2C_DBG_EXEC | SAM_I2C_DBG_ERROR, "sam_i2c_exec: "
			    "bus is unknown\n");
			return -1;
		default:
			DBPRINTF(SAM_I2C_DBG_EXEC | SAM_I2C_DBG_ERROR, "sam_i2c_exec: "
			    "bus is weird (%04" PRIx16 ")\n", SAM_I2C_BUS_STATE(i));
			return -1;
		}
	}

	is->is_op = op;
	is->is_cmd = cmd;
	is->is_cmdlen = cmdlen;
	is->is_buf = buf;
	is->is_buflen = buflen;
	is->is_state_fn = sam_i2c_state_begin;
	is->is_retries = 4;
	is->is_ctrlb = 0;	//SERCOM_I2CM_CTRLB_QCEN;

	a = SERCOM_I2CM_ADDR_ADDR(addr << 1);
	if (I2C_OP_READ_P(op) && cmdlen == 0) {
		a |= 1u;
		DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: READ with no "
		    "command.\n");
		if (is->is_buflen <= 1)
			is->is_ctrlb |= SERCOM_I2CM_CTRLB_ACKACT;
		if (is->is_buflen >= 1)
			is->is_ctrlb |= SERCOM_I2CM_CTRLB_SMEN;
		is->is_intflag = SERCOM_I2CM_INTFLAG_MB |
		    SERCOM_I2CM_INTFLAG_SB |
		    SERCOM_I2CM_INTFLAG_ERROR;
	} else {
		is->is_intflag = SERCOM_I2CM_INTFLAG_MB |
		    SERCOM_I2CM_INTFLAG_SB |
		    SERCOM_I2CM_INTFLAG_ERROR;
	}

	sam_i2c_issue_command(is, SAM_I2C_COMMAND_NOP);

	/* Initiate the transfer. */
	DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: initiating transfer.\n");

#ifdef SAM_I2C_IRQ_MODE
	i->INTENSET.reg = is->is_intflag;
	i->ADDR.reg = a;
	sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);

	if (rtos_sema_take_timed(is->is_sema, 500) == 0 ||
	    is->is_state_fn != NULL) {
		DBPRINTF(SAM_I2C_DBG_EXEC | SAM_I2C_DBG_ERROR, "sam_i2c_exec: "
		    "timed out. MASK %02x, FLAG %02x, STATUS %04" PRIx16
		    ", CTRLB %08" PRIx16 ".\n", (unsigned int)i->INTENSET.reg,
		    (unsigned int)i->INTFLAG.reg, i->STATUS.reg, i->CTRLB.reg);
		i->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
		is->is_state_fn = NULL;
		is->is_result = -1;
		is->is_do_reset = true;
	}
#else
	i->ADDR.reg = a;
	sam_i2c_syncbusy(is->is_regs, SERCOM_I2CM_SYNCBUSY_SYSOP);
	do {
		timer_timeout_t to;

		timer_timeout_start(&to, 500);
		while ((i->INTFLAG.reg & is->is_intflag) == 0) {
			if (timer_timeout_expired(&to))
				break;
			rtos_yield();
		}

		if ((i->INTFLAG.reg & is->is_intflag) == 0) {
			DBPRINTF(SAM_I2C_DBG_EXEC | SAM_I2C_DBG_ERROR,
			    "sam_i2c_exec: Timed out.\n");
			is->is_state_fn = NULL;
			is->is_result = -1;
			is->is_do_reset = true;
		} else {
			sam_i2c_interrupt(is);
		}
	} while (is->is_state_fn != NULL);
#endif	/* SAM_I2C_IRQ_MODE */

	is->is_expected_bus_state = I2C_OP_STOP_P(op) ? SAM_I2C_BUS_STATE_IDLE :
	    SAM_I2C_BUS_STATE_OWNED;

	DBPRINTF(SAM_I2C_DBG_EXEC, "sam_i2c_exec: Done. Result is %d\n",
	    is->is_result);

	return is->is_result;
}

i2c_tag_t
sam_sercom_i2c_attach(const sam_sercom_i2c_attach_args_t *aa)
{
	struct sam_i2c_state *is;

	if ((is = zone_malloc(sizeof(*is))) == NULL)
		return NULL;

	DBPRINTF(SAM_I2C_DBG_INIT, "sam_sercom_i2c_attach: hi there.\n");

	is->is_regs = aa->aa_regs;
	is->is_ctl = aa->aa_ctl;
	is->is_mutex = rtos_mutex_create();
	assert(is->is_mutex != NULL);
#ifdef SAM_I2C_IRQ_MODE
	is->is_sema = rtos_sema_create_binary();
	assert(is->is_sema != NULL);
#endif
	is->is_baud = (aa->aa_clock / (200000u)) - 1;
	assert(is->is_baud <= 0xffu);

	is->is_driver.ic_cookie = is;
	is->is_driver.ic_acquire_bus = sam_i2c_acquire_bus;
	is->is_driver.ic_release_bus = sam_i2c_release_bus;
	is->is_driver.ic_exec = sam_i2c_exec;
	is->is_driver.ic_send_start = NULL;
	is->is_driver.ic_send_stop = NULL;
	is->is_driver.ic_initiate_xfer = NULL;
	is->is_driver.ic_read_byte = NULL;
	is->is_driver.ic_write_byte = NULL;

	sam_i2c_reset(is);

#ifdef SAM_I2C_IRQ_MODE
	rtos_irq_register_named(aa->aa_name, aa->aa_vector, aa->aa_ipl,
	    sam_i2c_interrupt, is);
#endif

	DBPRINTF(SAM_I2C_DBG_INIT, "sam_sercom_i2c_attach: all good.\n");

	return &is->is_driver;
}
