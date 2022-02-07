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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "serial_number.h"

/*
 * Copy of the device's serial number.
 *
 * This structure looks a little odd, with the trailing 'pad' character.
 * The original wAVR/Maven board used the SAM4S silicon unique ID to
 * fabricate a serial number. As luck would have it, this ID was a 14-
 * character ASCII string preceded by a single 0x00 byte, so with the
 * addition of a trailing NUL, it could be printed directly.
 *
 * So for backwards compat (new firmware builds still work on old devices
 * already in the field) we need to keep the leading 0x00 and add an
 * additional 'pad' for the trailing NUL. Since our serial number will
 * fail to validate on legacy SAM4S devices, platform_fabricate_serial()
 * will be used to populate the structure with the original silicon
 * unique ID.
 *
 * The trailing 'pad' NUL byte is unused on all new devices.
 */
static struct {
	serial_number_t sn;
	char pad;		/* NUL pad for legacy devices. */
} serial_number;

#ifdef PLATFORM_SERIAL_NUMBER
static bool
bad_digits(const char *s)
{
	char ch;

	/*
	 * Serial number is comprised of digits 0-9 and letters A-Z.
	 */
	while ((ch = *s++) != '\0') {
		if (ch >= '0' && ch <= '9')
			continue;
		if (ch >= 'A' && ch <= 'Z')
			continue;

		return true;
	}

	return false;
}
#endif /* PLATFORM_SERIAL_NUMBER */

void
serial_number_init(void)
{
#ifdef PLATFORM_SERIAL_NUMBER
	const serial_number_t *fsn;
	serial_number_t *csn;
	unsigned int i;
	uint8_t chk;

	/*
	 * Point to the in-flash serial number for this device.
	 */
	fsn = PLATFORM_SERIAL_NUMBER;
	csn = &serial_number.sn;

	/*
	 * Copy the serial number, calculating the checksum as we go.
	 */
	for (chk = 0, i = 0; i < (sizeof(*csn) - 1); i++)
		chk += csn->u8[i] = fsn->u8[i];

	/* Initialise the remaining fields. */
	csn->checksum = 0x00u;
	serial_number.pad = '\0';

	if (/* Computed checksum must match stored checksum. */
	    fsn->checksum != chk ||

	    /* The leading 0x00 must be present. */
	    csn->zero != 0x00u ||

	    /* The serial number string's trailing NUL must be present. */
	    csn->cstr[sizeof(csn->cstr) - 1] != '\0' ||

	    /* The serial number string must be the correct length. */
	    strlen(csn->cstr) != (sizeof(csn->cstr) - 1) ||

	    /* The serial number must contain valid digits. */
	    bad_digits(csn->cstr)) {
		/*
		 * Serial number is bad. Call on platform code to fabricate
		 * something.
		 */
		platform_fabricate_serial(&serial_number.sn);
	}
#else
	platform_fabricate_serial(&serial_number.sn);
#endif /* PLATFORM_SERIAL_NUMBER */
}

const serial_number_t *
serial_number_fetch(void)
{

	return &serial_number.sn;
}
