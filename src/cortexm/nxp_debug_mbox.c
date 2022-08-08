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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "rtos.h"
#include "nxp_debug_mbox.h"
#include "timer.h"
#include "zone_alloc.h"

//#define	DEBUG_FLAG_INIT	1
#include "debug.h"

#define	NXP_MBREG_CSW		0x000u
#define	 NXP_CSW_RESYNCH_REQ	(1u << 0)
#define	 NXP_CSW_REQ_PENDING	(1u << 1)
#define	 NXP_CSW_DBG_OR_ERR	(1u << 2)
#define	 NXP_CSW_AHB_OR_ERR	(1u << 3)
#define	 NXP_CSW_SOFT_RESET	(1u << 4)
#define	 NXP_CSW_CHIP_RESET_REQ	(1u << 5)
#define	NXP_MBREG_REQUEST	0x004u
#define	NXP_MBREG_RETURN	0x008u
#define	NXP_MBREG_ID		0x0fcu
#define	 NXP_ID_LPC55S6X	NXP_LPC55S6X_IDR

/*
 * Mailbox request/response/ack structure, as per the datasheet.
 * Mailbox is little-endian.
 */
union dm_hdr {
	uint16_t v16[2];
	uint32_t v32;
};
#define	commandID	v16[0]
#define	commandStatus	v16[0]
#define	token		v16[0]
#define	dataWordCount	v16[1]
#define	remainCount	v16[1]

/* Support mailbox commands. */
#define	NXP_CMD_START_DM_AP			0x01u	/* Legacy command. */
#define	NXP_CMD_BULK_ERASE			0x02u	/* Legacy command. */
#define	NXP_CMD_EXIT_DM_AP			0x04u	/* Legacy command. */
#define	NXP_CMD_ENTER_ISP_MODE			0x05u
#define	NXP_CMD_SET_FA_MODE			0x06u
#define	NXP_CMD_START_DEBUG_SESSION		0x07u

/* Data Word for NXP_CMD_ENTER_ISP_MODE */
#define	NXP_ISP_MODE_AUTODETECT			0xffffffffu
#define	NXP_ISP_MODE_UART			0x1u
#define	NXP_ISP_MODE_I2C			0x2u
#define	NXP_ISP_MODE_SPI			0x4u
#define	NXP_ISP_MODE_USB_HID			0x10u

/* Mailbox response status codes. */
#define	NXP_RESP_STATUS_SUCCESS			0x0u
#define	NXP_RESP_STATUS_DEBUG_MODE_NOT_ENTERED	0x1u
#define	NXP_RESP_STATUS_BAD_COMMAND		0x2u
#define	NXP_RESP_STATUS_COMMAND_FAILED		0x3u
#define	NXP_RESP_NEW_PROTOCOL(r)	(((r)->hdr.dataWordCount&0x8000u) != 0)
#define	NXP_RESP_DATA_WORD_COUNT(r)	((r)->hdr.dataWordCount&0x7fffu)

/* Mailbox ACK token structure, as per the datasheet. */
#define	NXP_ACK_TOKEN	0xa5a5u

struct nxp_debug_mbox_state {
	const struct adiv5_ap_interface *ms_ap;
	void *ms_ap_arg;
};

static bool
nxp_reg_read(struct nxp_debug_mbox_state *ms, uint32_t reg, uint32_t *vp)
{

	*vp = ms->ms_ap->ai_read(ms->ms_ap_arg, reg);

	return ms->ms_ap->ai_dp_error(ms->ms_ap_arg) == 0;
}

static bool
nxp_reg_write(struct nxp_debug_mbox_state *ms, uint32_t reg, uint32_t v)
{
	int rv;

	rv = ms->ms_ap->ai_write(ms->ms_ap_arg, reg, v, true);

	return rv == 0 && ms->ms_ap->ai_dp_error(ms->ms_ap_arg) == 0;
}

static bool
nxp_issue_reset(struct nxp_debug_mbox_state *ms, bool resync)
{
	timer_timeout_t to;
	uint32_t reg;

	reg = resync ? NXP_CSW_RESYNCH_REQ : 0;

	/*
	 * Issue Chip Reset, and Resync Request.
	 */
	if (!nxp_reg_write(ms, NXP_MBREG_CSW, reg | NXP_CSW_CHIP_RESET_REQ)) {
		DBFPRINTF("Chip Reset/Resync failed\n");
		return false;
	}

	/*
	 * Wait for the ROM to acknowledge
	 */
	DBFPRINTF("Wait for ROM ack\n");
	timer_timeout_start(&to, 2000u);
	do {
		if (nxp_reg_read(ms, NXP_MBREG_CSW, &reg) == false)
			return false;
	} while (reg != 0 && !timer_timeout_expired(&to));

	if (reg != 0) {
		DBFPRINTF("No acknowledge for Reset/Resync command "
		    "(%08" PRIx32 "\n", reg);
		return false;
	}

	rtos_task_sleep(5);
	return true;
}

static bool
nxp_request_debug_mode(struct nxp_debug_mbox_state *ms)
{
	union dm_hdr hdr;
	uint32_t reg;

	/*
	 * Issue Chip Reset, and Resync Request.
	 */
	if (nxp_issue_reset(ms, true) == false) {
		DBFPRINTF("Chip reset failed.\n");
		return false;
	}

	/*
	 * Erratum ROM.6: ROM fails to respond to debug session request on
	 * version 0A parts.
	 *
	 * Implement the sugested work-around here.
	 */
	DBFPRINTF("Probing for ROM version (erratum ROM.6)\n");
	hdr.commandID = NXP_CMD_START_DM_AP;
	hdr.dataWordCount = 0;
	if (nxp_reg_write(ms, NXP_MBREG_REQUEST, hdr.v32) == false) {
		DBFPRINTF("START_DM_AP command failed\n");
		return false;
	}

	/*
	 * Send command to start debug session, without checking RETURN.
	 */
	DBFPRINTF("Issueing First Start Debug Session command.\n");
	hdr.commandID = NXP_CMD_START_DEBUG_SESSION;
	hdr.dataWordCount = 0;
	if (nxp_reg_write(ms, NXP_MBREG_REQUEST, hdr.v32) == false) {
		DBFPRINTF("Start Debug Session command failed\n");
		return false;
	}

	/* Errata document suggests 10mS sleep at this point. */
	rtos_task_sleep(10);

	DBFPRINTF("Read back CSW.\n");
	if (nxp_reg_read(ms, NXP_MBREG_CSW, &reg) == false) {
		DBFPRINTF("Failed to read CSW\n");
		return false;
	}

	/*
	 * Check for an error. 1B silicon will raise an overrun error,
	 * otherwise we're dealing with 0A silicon.
	 */
	if ((reg & NXP_CSW_AHB_OR_ERR) != 0) {
		DBFPRINTF("Silicon revision is 1B.\n");

		/* Repeat reset to clean overrun. */
		if (nxp_issue_reset(ms, true) == false) {
			DBFPRINTF("Second Chip reset failed.\n");
			return false;
		}

		/*
		 * Need to issue a second Start Debug Session command.
		 */
		DBFPRINTF("Issueing Second Start Debug Session command.\n");
		hdr.commandID = NXP_CMD_START_DEBUG_SESSION;
		hdr.dataWordCount = 0;
		if (nxp_reg_write(ms, NXP_MBREG_REQUEST, hdr.v32) == false) {
			DBFPRINTF("Start Debug Session command failed\n");
			return false;
		}

		/*
		 * Fetch the return status.
		 */
		if (nxp_reg_read(ms, NXP_MBREG_RETURN, &hdr.v32) == false)
			DBFPRINTF("No acknowledge for Reset/Resync command\n");
		else {
			DBFPRINTF("Start Debug Session acknowledged (%08"
			    PRIx16 ")\n", hdr.commandStatus);
		}
	} else {
		DBFPRINTF("Silicon revision is 0A.\n");
	}


	return true;
}

int
nxp_debug_mailbox_probe(const struct adiv5_ap_interface *api, void *arg)
{
	struct nxp_debug_mbox_state ms;
	uint32_t reg;

	ms.ms_ap = api;
	ms.ms_ap_arg = arg;

	/*
	 * Validate the ID register is what we expect.
	 */
	reg = 0;
	if (!nxp_reg_read(&ms, NXP_MBREG_ID, &reg) || reg != NXP_ID_LPC55S6X) {
		DBFPRINTF("NXP_MBREG_ID failed, or no match (%08" PRIx32 "\n",
		    reg);
		return TARGET_ATTACH_NO_MATCH;
	}

	/*
	 * If the Mem-APs have been found already, we don't need to establish
	 * a debug session.
	 */
	if (adiv5_ap_exists(0) && adiv5_ap_exists(1)) {
		DBFPRINTF("Mem-APs already exist.\n");
		return TARGET_ATTACH_NO_MATCH;
	}

	DBFPRINTF("Issue Reset/Resync command\n");

	/*
	 * Reset the chip, and request debug access.
	 */
	(void) nxp_request_debug_mode(&ms);

	/*
	 * Re-enumerate the device.
	 */
	return TARGET_ATTACH_RESCAN;
}

void *
nxp_debug_mailbox_attach(void)
{
	struct nxp_debug_mbox_state *ms;

	if ((ms = zone_malloc(sizeof(*ms))) == NULL)
		return NULL;

	if ((ms->ms_ap_arg = adiv5_other_ap_match(NXP_LPC55S6X_IDR,
	    NXP_LPC55S6X_IDR_MASK, &ms->ms_ap)) == NULL) {
		zone_free(ms);
		ms = NULL;
	}

	return ms;
}

void
nxp_debug_mailbox_detach(void *arg)
{
	struct nxp_debug_mbox_state *ms = arg;

	if (ms->ms_ap->ai_free != NULL)
		ms->ms_ap->ai_free(ms->ms_ap_arg);
	zone_free(ms);
}

bool
nxp_debug_mailbox_reset(void *arg)
{
	struct nxp_debug_mbox_state *ms = arg;

	if (nxp_issue_reset(ms, false) == false) {
		DBFPRINTF("nxp_issue_reset() failed\n");
		return false;
	}

	rtos_task_sleep(100);

	/* Request debug mode */
	return nxp_request_debug_mode(ms);
}
