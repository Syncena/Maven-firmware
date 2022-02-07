##############################################################################
#
# Copyright (c) 2022, Steve C. Woodford.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################

# Add applets to the list, below
APPLETS	:=	applet-sam-nvmctrl-t0.applet
APPLETS	+=	applet-sam-nvmctrl-t1.applet
APPLETS	+=	applet-sam-efc.applet
APPLETS	+=	applet-stm-fpec.applet
APPLETS	+=	applet-stm-efmi.applet
APPLETS	+=	applet-nxp-lpc55.applet

# The applets are basically an array of 8-bit values for direct
# inclusion in C source. hexdump(1) can be used to generate them.
HD = hexdump -v -e "/1 \"0x%02x,\""

# No user-servicable parts below.

# Cortex-M0 is the lowest common denominator. Some of the applets may be
# relevant for any Cortex-M flavour, but we want to avoid duplication.
CFLAGS = -Os -mcpu=cortex-m0 -mthumb -fpic -mpic-data-is-text-relative
LDFLAGS += -Wl,--script=applet.ld -fpic -nostartfiles
TOOLCHAIN = arm-none-eabi
CC = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
CDEFS = -DAPPLET_VOLATILE=volatile

.PHONY:	all clean

ifeq ($(KEEP_ELF),1)
.PRECIOUS:	$(APPLETS:.applet=.elf)
endif

all:	$(APPLETS)

clean:
	-rm -f $(APPLETS) *.o *.bin *.elf

%.o:	%.c
	$(CC) $(CFLAGS) $(CDEFS) -c $<

%.elf:	%.o
	$(CC) $(LDFLAGS) $< -o $@

%.bin:	%.elf
	$(OBJCOPY) --output-format=binary $< $@

%.applet:	%.bin
	$(HD) $< > $@
