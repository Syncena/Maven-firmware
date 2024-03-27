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

NAME				:= mculink
CPU_ARCH			:= arm
ARM_CPU_OPTION			:= cortex-m33
PLATFORM			:= MCULink

CFLAGS				+= -fno-exceptions
CFLAGS				+= -fno-unwind-tables

MK_INCLUDE_ARMV8M		:= 1
MK_INCLUDE_CONFIGDB		:= 1
MK_INCLUDE_EEPROM_EMULATION	:= 1
MK_INCLUDE_TARGET_COMMS		:= 1
MK_INCLUDE_SHELL		:= 1
MK_INCLUDE_TINY_USB		:= 1
MK_INCLUDE_TOMCRYPT		:= 1

TINYUSB_DRIVER			:= nxp/lpc_ip3511/dcd_lpc_ip3511.c

#
# NXP's "blhost" command works best with S-record images. Note that converting
# the .elf direct to S-record triggers a bug in "blhost" which causes Flash
# programming to fail (I suspect "blhost" objects to gaps in S-record files).
# So we create $(PRODUCT).srec from the .bin file.
#
POSTREQUISITES			= $(PRODUCT).srec
$(PRODUCT).srec:	$(PRODUCT).bin
	$(OBJCOPY) -O srec $< $@

include $(TOPDIR)/../build/Makefile.inc
