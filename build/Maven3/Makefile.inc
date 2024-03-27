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

NAME			:= maven
CPU_ARCH		:= arm
ARM_CPU_OPTION		:= cortex-m4
PLATFORM		:= Maven-SAMD51
PLATFORM_VERSION	?= 3

MK_INCLUDE_ARMV7M	:= 1
MK_INCLUDE_DISPLAY	:= 1
MK_INCLUDE_U8GLIB	:= 1
MK_INCLUDE_NETWORK	:= 1
MK_INCLUDE_LWIP		:= 1
MK_INCLUDE_LWIP_HTTPD	:= 1
MK_INCLUDE_CONFIGDB	:= 1
MK_INCLUDE_TOMCRYPT	:= 1
MK_INCLUDE_PUFF		:= 1

# Pick one of the folowing according to the WiFi controller used on the
# device. Note that WINC1500 is preferred due to bugs in the WILC1000's
# firmware, but also note that WiFi provisioning mode is not supported
# for WINC1500 in 'raw Ethernet' mode.
#MK_INCLUDE_WILC1000	:= 1
MK_INCLUDE_WINC1500_LWIP	:= 1

include $(TOPDIR)/../build/Makefile.inc
