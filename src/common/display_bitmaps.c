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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "display_bitmaps.h"

/*
 * The following neat trick for pseudo-binary encoded literals was
 * lifted shamelessly from Renato Chandelier's response in this thread:
 * http://stackoverflow.com/questions/2611764/can-i-use-a-binary-literal-in-c-or-c
 */
/* <shameless-copy> */
#define B_0000	0
#define B_0001	8
#define B_0010	4
#define B_0011	c
#define B_0100	2
#define B_0101	a
#define B_0110	6
#define B_0111	e
#define B_1000	1
#define B_1001	9
#define B_1010	5
#define B_1011	d
#define B_1100	3
#define B_1101	b
#define B_1110	7
#define B_1111	f

#define _B2H(bits)  B_##bits
#define B2H(bits)   _B2H(bits)
#define _HEX(n)     0x##n
#define HEX(n)      _HEX(n)
#define _CCAT(a,b)  a##b
#define CCAT(a,b)   _CCAT(a,b)

#define BYTE(a,b)       ((uint8_t)HEX( CCAT(B2H(b),B2H(a)) ))
/* </shameless-copy> */


static const uint8_t bm_wifi_signal_none_bits[] = {
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0010,1010),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0001,0100),
	BYTE(0000,0000), BYTE(0000,1010), BYTE(1010,1010),
	BYTE(0000,0000), BYTE(0000,0101), BYTE(0001,0100),
	BYTE(0000,0010), BYTE(1010,1010), BYTE(1010,1010),
	BYTE(0000,0001), BYTE(0100,0101), BYTE(0001,0100),
	BYTE(1010,1010), BYTE(1010,1010), BYTE(1010,1010),
	BYTE(0101,0001), BYTE(0100,0101), BYTE(0001,0100),
};
static const struct display_bitmap bm_wifi_signal_none = {
	24, 8, bm_wifi_signal_none_bits
};

static const uint8_t bm_wifi_signal_25_bits[] = {
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0010,1010),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0001,0100),
	BYTE(0000,0000), BYTE(0000,1010), BYTE(1010,1010),
	BYTE(0000,0000), BYTE(0000,0101), BYTE(0001,0100),
	BYTE(0000,0010), BYTE(1010,1010), BYTE(1010,1010),
	BYTE(0000,0001), BYTE(0100,0101), BYTE(0001,0100),
	BYTE(1111,1010), BYTE(1010,1010), BYTE(1010,1010),
	BYTE(1111,1001), BYTE(0100,0101), BYTE(0001,0100),
};
static const struct display_bitmap bm_wifi_signal_25 = {
	24, 8, bm_wifi_signal_25_bits
};

static const uint8_t bm_wifi_signal_50_bits[] = {
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0010,1010),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0001,0100),
	BYTE(0000,0000), BYTE(0000,1010), BYTE(1010,1010),
	BYTE(0000,0000), BYTE(0000,0101), BYTE(0001,0100),
	BYTE(0000,0011), BYTE(1110,1010), BYTE(1010,1010),
	BYTE(0000,0011), BYTE(1110,0101), BYTE(0001,0100),
	BYTE(1111,1011), BYTE(1110,1010), BYTE(1010,1010),
	BYTE(1111,1011), BYTE(1110,0101), BYTE(0001,0100),
};
static const struct display_bitmap bm_wifi_signal_50 = {
	24, 8, bm_wifi_signal_50_bits
};

static const uint8_t bm_wifi_signal_75_bits[] = {
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0010,1010),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0001,0100),
	BYTE(0000,0000), BYTE(0000,1111), BYTE(1010,1010),
	BYTE(0000,0000), BYTE(0000,1111), BYTE(1001,0100),
	BYTE(0000,0011), BYTE(1110,1111), BYTE(1010,1010),
	BYTE(0000,0011), BYTE(1110,1111), BYTE(1001,0100),
	BYTE(1111,1011), BYTE(1110,1111), BYTE(1010,1010),
	BYTE(1111,1011), BYTE(1110,1111), BYTE(1001,0100),
};
static const struct display_bitmap bm_wifi_signal_75 = {
	24, 8, bm_wifi_signal_75_bits
};

static const uint8_t bm_wifi_signal_100_bits[] = {
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0011,1110),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0011,1110),
	BYTE(0000,0000), BYTE(0000,1111), BYTE(1011,1110),
	BYTE(0000,0000), BYTE(0000,1111), BYTE(1011,1110),
	BYTE(0000,0011), BYTE(1110,1111), BYTE(1011,1110),
	BYTE(0000,0011), BYTE(1110,1111), BYTE(1011,1110),
	BYTE(1111,1011), BYTE(1110,1111), BYTE(1011,1110),
	BYTE(1111,1011), BYTE(1110,1111), BYTE(1011,1110),
};
static const struct display_bitmap bm_wifi_signal_100 = {
	24, 8, bm_wifi_signal_100_bits
};

static const uint8_t bm_activity_net_and_usb_bits[] = {
	BYTE(1001,0000), BYTE(0000,0010), BYTE(0100,0000), BYTE(0000,0000),
	BYTE(1001,0111), BYTE(0000,0011), BYTE(0101,1100), BYTE(0000,0000),
	BYTE(1001,0100), BYTE(0111,0011), BYTE(1101,0001), BYTE(1100,0000),
	BYTE(1001,0111), BYTE(0100,1010), BYTE(1101,1000), BYTE(1000,0000),
	BYTE(0110,0001), BYTE(0111,0010), BYTE(0101,0000), BYTE(1000,0000),
	BYTE(0000,0111), BYTE(0100,1000), BYTE(0001,1100), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0111,0000), BYTE(0000,0000), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000),
};
static const struct display_bitmap bm_activity_net_and_usb = {
	26, 8, bm_activity_net_and_usb_bits
};

static const uint8_t bm_activity_net_only_bits[] = {
	BYTE(0000,0000), BYTE(0000,0010), BYTE(0100,0000), BYTE(0000,0000),
	BYTE(0000,0000), BYTE(0000,0011), BYTE(0101,1100), BYTE(0000,0000),
	BYTE(0000,0000), BYTE(0000,0011), BYTE(1101,0001), BYTE(1100,0000),
	BYTE(0000,0000), BYTE(0000,0010), BYTE(1101,1000), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0000,0010), BYTE(0101,0000), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0001,1100), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000), BYTE(1000,0000),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000),
};
static const struct display_bitmap bm_activity_net_only = {
	26, 8, bm_activity_net_only_bits
};

static const uint8_t bm_activity_usb_only_bits[] = {
	BYTE(1001,0000), BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(1001,0111), BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(1001,0100), BYTE(0111,0000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(1001,0111), BYTE(0100,1000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(0110,0001), BYTE(0111,0000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(0000,0111), BYTE(0100,1000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(0000,0000), BYTE(0111,0000), BYTE(0000,0000), BYTE(0000,0000),
	BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000), BYTE(0000,0000),
};
static const struct display_bitmap bm_activity_usb_only = {
	26, 8, bm_activity_usb_only_bits
};

static const struct display_bitmap * const display_bitmaps[] = {
	[DISPLAY_BITMAP_WIFI_SIGNAL_NONE]=	&bm_wifi_signal_none,
	[DISPLAY_BITMAP_WIFI_SIGNAL_25]=	&bm_wifi_signal_25,
	[DISPLAY_BITMAP_WIFI_SIGNAL_50]=	&bm_wifi_signal_50,
	[DISPLAY_BITMAP_WIFI_SIGNAL_75]=	&bm_wifi_signal_75,
	[DISPLAY_BITMAP_WIFI_SIGNAL_100]=	&bm_wifi_signal_100,
	[DISPLAY_BITMAP_ACTIVITY_NET_AND_USB]=	&bm_activity_net_and_usb,
	[DISPLAY_BITMAP_ACTIVITY_NET_ONLY]=	&bm_activity_net_only,
	[DISPLAY_BITMAP_ACTIVITY_USB_ONLY]=	&bm_activity_usb_only,
};

uint8_t
display_bitmap_get(uint8_t bm, struct display_bitmap *pb)
{
	const struct display_bitmap *b;

	if (bm > (sizeof(display_bitmaps) / sizeof(display_bitmaps[0])))
		return 0;

	b = display_bitmaps[bm];

	pb->b_width = b->b_width;
	pb->b_height = b->b_height;
	pb->b_bits = b->b_bits;

	return 1;
}
