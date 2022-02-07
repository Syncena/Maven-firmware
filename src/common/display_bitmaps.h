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

#ifndef DISPLAY_BITMAPS_H
#define	DISPLAY_BITMAPS_H

struct display_bitmap {
	uint8_t b_width;
	uint8_t b_height;
	const uint8_t *b_bits;
};

extern uint8_t display_bitmap_get(uint8_t bitmap, struct display_bitmap *pb);

#define	DISPLAY_BITMAP_WIFI_SIGNAL_NONE		0
#define	DISPLAY_BITMAP_WIFI_SIGNAL_25		1
#define	DISPLAY_BITMAP_WIFI_SIGNAL_50		2
#define	DISPLAY_BITMAP_WIFI_SIGNAL_75		3
#define	DISPLAY_BITMAP_WIFI_SIGNAL_100		4
#define	DISPLAY_BITMAP_ACTIVITY_NET_AND_USB	5
#define	DISPLAY_BITMAP_ACTIVITY_NET_ONLY	6
#define	DISPLAY_BITMAP_ACTIVITY_USB_ONLY	7

#endif /* DISPLAY_BITMAPS_H */
