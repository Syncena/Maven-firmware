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

EXTRAINCDIRS	+= $(WILC1000_PATH)
WILC1000_OBJDIR	:= $(OBJDIR)/wilc1000

ifndef MK_INCLUDE_PREBOOT
WILC1000_SRCS	:=					\
	${WILC1000_PATH}/driver/source/m2m_hif.c	\
	${WILC1000_PATH}/driver/source/m2m_periph.c	\
	${WILC1000_PATH}/driver/source/m2m_wifi.c	\
	${WILC1000_PATH}/driver/source/nmasic.c		\
	${WILC1000_PATH}/driver/source/nmbus.c		\
	${WILC1000_PATH}/driver/source/nmdrv.c		\
	${WILC1000_PATH}/driver/source/nmspi.c		\
	${WILC1000_PATH}/common/source/nm_common.c

$(foreach t,$(WILC1000_SRCS),$(eval $(call make-rule-c,$(WILC1000_OBJDIR),$(t))))
endif
