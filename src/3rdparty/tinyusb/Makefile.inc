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

DEFINES			+= -DCONFIG_USE_USB=1
DEFINES			+= -DCONFIG_USE_TINYUSB=1
EXTRAINCDIRS		+= $(TINYUSB_PATH)
TINYUSB_OBJDIR		:= $(OBJDIR)/usb
TINYUSB_IF_SRCS		:=		\
	$(TINYUSB_PATH)/usb_cdc.c	\
	$(TINYUSB_PATH)/usb_common.c

TINYUSB_DIST		:= $(TINYUSB_PATH)/dist/src
EXTRAINCDIRS		+= $(TINYUSB_DIST)
TINYUSB_DIST_SRCS	:=				\
	$(TINYUSB_DIST)/common/tusb_fifo.c		\
	$(TINYUSB_DIST)/class/cdc/cdc_device.c		\
	$(TINYUSB_DIST)/device/usbd.c			\
	$(TINYUSB_DIST)/device/usbd_control.c		\
	$(TINYUSB_DIST)/tusb.c				\
	$(TINYUSB_DIST)/portable/$(TINYUSB_DRIVER)

TINYUSB_SRCS		:= $(TINYUSB_IF_SRCS) $(TINYUSB_DIST_SRCS)

# Upstream source needs the following compiler warnings disabled.
TINYUSB_CFLAGS		:=		\
	-Wno-cast-qual			\
	-Wno-format-nonliteral		\
	-Wno-pedantic			\
	-Wno-missing-prototypes		\
	-Wno-missing-declarations

$(foreach t,$(TINYUSB_SRCS),$(eval $(call make-rule-c-flags,$(TINYUSB_OBJDIR),$(t),$(TINYUSB_CFLAGS))))
