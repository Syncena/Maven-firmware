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

TOOLCHAIN		:= arm-none-eabi
CPU_ARCH_GCC_FLAGS	:= -mcpu=$(ARM_CPU_OPTION)

EXTRAINCDIRS		+= $(CPU_ARCH_PATH)
EXTRAINCDIRS		+= $(TOPDIR)/3rdparty/CMSIS

CPU_ARCH_OBJDIR		:= $(OBJDIR)/platform/core
CPU_ARCH_C_SRCS		:=		\
	$(CPU_ARCH_PATH)/rtos_cortexm.c	\
	$(CPU_ARCH_PATH)/swo_uart.c
CPU_ARCH_S_SRCS		:=		\
	$(CPU_ARCH_PATH)/memcpy-armv7m.S

ifdef MK_INCLUDE_ARMV7M
CPU_ARCH_C_SRCS		+=		\
	$(CPU_ARCH_PATH)/mpu_armv7m.c
endif

ifdef MK_INCLUDE_ARMV8M
CPU_ARCH_C_SRCS		+=		\
	$(CPU_ARCH_PATH)/mpu_armv8m.c
endif

$(foreach t,$(CPU_ARCH_C_SRCS),$(eval $(call make-rule-c,$(CPU_ARCH_OBJDIR),$(t))))
$(foreach t,$(CPU_ARCH_S_SRCS),$(eval $(call make-rule-S,$(CPU_ARCH_OBJDIR),$(t))))
