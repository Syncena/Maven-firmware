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

EXTRAINCDIRS		+= $(PLATFORM_PATH)
EXTRAINCDIRS		+= $(TOPDIR)/3rdparty/samd51
DEFINES			+= -DSAMD51=1 -D__SAMD51__
ifeq ($(PLATFORM_VERSION),2)
DEFINES			+= -D__SAMD51G19A__
else
DEFINES			+= -D__SAMD51J20A__
endif
DEFINES			+= -DMAVEN_BOARD=$(PLATFORM_VERSION)
PLATFORM_LDSCRIPT	?= $(PLATFORM_PATH)/Maven$(PLATFORM_VERSION)-$(PLATFORM_TARGET).ld

HW_ID			:= 0x2020000$(PLATFORM_VERSION)

PLATFORM_OBJDIR		:= $(OBJDIR)/platform

PLATFORM_SRCS		:=			\
	$(PLATFORM_PATH)/maven_button.c		\
	$(PLATFORM_PATH)/maven_display.c	\
	$(PLATFORM_PATH)/maven_flash.c		\
	$(PLATFORM_PATH)/maven_platform.c	\
	$(PLATFORM_PATH)/maven_power.c		\
	$(PLATFORM_PATH)/maven_uart.c		\
	$(PLATFORM_PATH)/maven_wifi.c

ifdef MK_INCLUDE_ATMEL_USB
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/maven_usb.c
endif
ifdef MK_INCLUDE_TINY_USB
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/maven_usb.c
endif

ifdef MK_INCLUDE_AVR_PROGRAMMER
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/maven_avr_hw.c
endif

PLATFORM_SOC_PATH	:= $(TOPDIR)/platform/Shared/SAMD5E5
include $(PLATFORM_SOC_PATH)/Makefile.inc

$(foreach t,$(PLATFORM_SRCS),$(eval $(call make-rule-c,$(PLATFORM_OBJDIR),$(t))))
