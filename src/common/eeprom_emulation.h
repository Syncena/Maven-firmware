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

#ifndef EEPROM_EMULATION_H
#define EEPROM_EMULATION_H

#include <stdbool.h>
#include <stdint.h>

/*
 * This driver emulates EEPROM using a persistant backing store, usually
 * Flash memory, on behalf of the configdb subsystem.
 * To do this, a region of Flash memory comprising multiple erase units
 * is allocated for use by the emulator. The size of the EEPROM is (usually)
 * equivalent to the size of a Flash erase unit, or some multiple thereof.
 */

struct eeprom_emulation_driver {
	/* Size, in bytes, of an erase unit (page). */
	size_t ed_page_size;

	/* Number of pages in the backing store. */
	unsigned int ed_pages;

	bool (*ed_page_read)(void *, unsigned int page, void *data);
	bool (*ed_page_program)(void *, unsigned int page, const void *data);
};

extern int eeprom_emulation_attach(const struct eeprom_emulation_driver *,
		void *);

#endif /* EEPROM_EMULATION_H */
