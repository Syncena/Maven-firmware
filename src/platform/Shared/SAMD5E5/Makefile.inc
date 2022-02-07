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

EXTRAINCDIRS			+= $(PLATFORM_SOC_PATH)

ifndef MK_INCLUDE_PREBOOT
PLATFORM_SOC_PATH_OBJDIR	:= $(OBJDIR)/platform/soc
PLATFORM_SOC_PATH_SRCS		:=		\
	$(PLATFORM_SOC_PATH)/sam_aes.c		\
	$(PLATFORM_SOC_PATH)/sam_clocks.c	\
	$(PLATFORM_SOC_PATH)/sam_cmcc.c		\
	$(PLATFORM_SOC_PATH)/sam_crc32.c	\
	$(PLATFORM_SOC_PATH)/sam_dmac.c		\
	$(PLATFORM_SOC_PATH)/sam_eic.c		\
	$(PLATFORM_SOC_PATH)/sam_gpio.c		\
	$(PLATFORM_SOC_PATH)/sam_sercom_spi.c	\
	$(PLATFORM_SOC_PATH)/sam_sercom_usart.c	\
	$(PLATFORM_SOC_PATH)/sam_trng.c

ifdef MK_INCLUDE_SAME5X_GMAC
PLATFORM_SOC_PATH_SRCS		+=		\
	$(PLATFORM_SOC_PATH)/sam_gmac.c
endif

ifdef MK_INCLUDE_SAME5X_I2C
PLATFORM_SOC_PATH_SRCS		+=		\
	$(PLATFORM_SOC_PATH)/sam_sercom_i2c.c
endif

ifdef MK_INCLUDE_ATMEL_USB
PLATFORM_SOC_PATH_SRCS		+=		\
	$(PLATFORM_SOC_PATH)/sam_usb.c		\
	$(PLATFORM_SOC_PATH)/sam_usb_udd.c
endif

$(foreach t,$(PLATFORM_SOC_PATH_SRCS),$(eval $(call make-rule-c,$(PLATFORM_SOC_PATH_OBJDIR),$(t))))
endif
