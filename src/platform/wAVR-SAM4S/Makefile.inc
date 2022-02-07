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
EXTRAINCDIRS		+= $(TOPDIR)/3rdparty/sam4s
DEFINES			+= -D__SAM4S4A__
DEFINES			+= -DDONT_USE_CMSIS_INIT=1
PLATFORM_LDSCRIPT	?= $(PLATFORM_PATH)/wAVR-SAM4S-$(PLATFORM_TARGET).ld

HW_ID			:= 0x20180001

PLATFORM_OBJDIR		:= $(OBJDIR)/platform

PLATFORM_SRCS		:=			\
	$(PLATFORM_PATH)/globals.c		\
	$(PLATFORM_PATH)/sam-efc.c		\
	$(PLATFORM_PATH)/sam-usart.c		\
	$(PLATFORM_PATH)/sam4s4_platform.c	\
	$(PLATFORM_PATH)/sam4s4_power.c		\
	$(PLATFORM_PATH)/spi-devs.c

ifdef MK_INCLUDE_ATMEL_USB
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/sam4s4_usb.c		\
	${PLATFORM_PATH}/sam-udp.c
endif

ifdef MK_INCLUDE_WINC1500_NATIVE
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/sam4s4_wifi.c
endif

ifdef MK_INCLUDE_WIZNET_NATIVE
PLATFORM_SRCS		+=			\
	$(PLATFORM_PATH)/sam4s4_wizchip.c
endif

$(foreach t,$(PLATFORM_SRCS),$(eval $(call make-rule-c,$(PLATFORM_OBJDIR),$(t))))
