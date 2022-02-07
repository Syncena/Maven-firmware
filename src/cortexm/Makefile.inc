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

EXTRAINCDIRS	+= $(CORTEXM_PATH)
DEFINES		+= -DCONFIG_TARGET_CORTEXM=1
CORTEXM_OBJDIR	:= $(OBJDIR)/cortexm
CORTEXM_SRCS	:=				\
	$(CORTEXM_PATH)/adiv5.c			\
	$(CORTEXM_PATH)/adiv5_jtagdp.c		\
	$(CORTEXM_PATH)/adiv5_ll.c		\
	$(CORTEXM_PATH)/adiv5_swdp.c		\
	$(CORTEXM_PATH)/cortexm.c		\
	$(CORTEXM_PATH)/cortexm_target.c	\
	$(CORTEXM_PATH)/cortexm_txml.c		\
	$(CORTEXM_PATH)/flash_nxp_lpc5x.c	\
	$(CORTEXM_PATH)/flash_sam_calw.c	\
	$(CORTEXM_PATH)/flash_sam_efc.c		\
	$(CORTEXM_PATH)/flash_sam_nvmctrl.c	\
	$(CORTEXM_PATH)/flash_stm_efmi.c	\
	$(CORTEXM_PATH)/flash_stm_efmi_h7.c	\
	$(CORTEXM_PATH)/flash_stm_fpec.c	\
	$(CORTEXM_PATH)/microchip_dsu.c		\
	$(CORTEXM_PATH)/nxp_debug_mbox.c	\
	$(CORTEXM_PATH)/semihosting.c		\
	$(CORTEXM_PATH)/soc_lpc55s6x.c		\
	$(CORTEXM_PATH)/soc_rp2040.c		\
	$(CORTEXM_PATH)/soc_sam0.c		\
	$(CORTEXM_PATH)/soc_sam34x.c		\
	$(CORTEXM_PATH)/soc_samd5x.c		\
	$(CORTEXM_PATH)/soc_samg5x.c		\
	$(CORTEXM_PATH)/soc_samx7x.c		\
	$(CORTEXM_PATH)/soc_stm32f0.c		\
	$(CORTEXM_PATH)/soc_stm32f1.c		\
	$(CORTEXM_PATH)/soc_stm32f4.c		\
	$(CORTEXM_PATH)/soc_stm32f7.c		\
	$(CORTEXM_PATH)/soc_stm32h7.c		\
	$(CORTEXM_PATH)/swo_serve.c

ifdef MK_INCLUDE_GDB_RTOS
CORTEXM_SRCS	+=				\
	$(CORTEXM_PATH)/cortexm_rtos.c		\
	$(CORTEXM_PATH)/cortexm_rtos_freertos.c	\
	$(CORTEXM_PATH)/cortexm_rtos_nuttx.c
endif

$(foreach t,$(CORTEXM_SRCS),$(eval $(call make-rule-c,$(CORTEXM_OBJDIR),$(t))))
