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

EXTRAINCDIRS	+= $(NETWORK_PATH)
NETWORK_OBJDIR	:= $(OBJDIR)/network
NETWORK_SRCS	:=				\
	$(NETWORK_PATH)/network.c		\
	$(NETWORK_PATH)/telnet.c

ifdef MK_INCLUDE_WILC1000
MK_INCLUDE_LWIP	:= 1
DEFINES		+= -DNETWORK_DEV_WILC=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/dns_hijack.c		\
	$(NETWORK_PATH)/tinydhcpserver.c	\
	$(NETWORK_PATH)/tinyservices.c		\
	$(NETWORK_PATH)/wifi_driver.c		\
	$(NETWORK_PATH)/wifi_provision.c	\
	$(NETWORK_PATH)/wilc_wifi.c
WILC1000_PATH	:= $(TOPDIR)/3rdparty/wilc1000
include $(WILC1000_PATH)/Makefile.inc
else ifdef MK_INCLUDE_WINC1500_NATIVE
DEFINES		+= -DNETWORK_DEV_WINC=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/wifi_driver.c		\
	$(NETWORK_PATH)/winc_wifi.c
WINC1500_PATH	:= $(TOPDIR)/3rdparty/winc1500
include $(WINC1500_PATH)/Makefile.inc
else ifdef MK_INCLUDE_WINC1500_LWIP
MK_INCLUDE_LWIP	:= 1
DEFINES		+= -DNETWORK_DEV_WINC=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/native_dhcp.c		\
	$(NETWORK_PATH)/wifi_driver.c		\
	$(NETWORK_PATH)/wifi_provision.c	\
	$(NETWORK_PATH)/winc_wifi.c
WINC1500_PATH	:= $(TOPDIR)/3rdparty/winc1500
include $(WINC1500_PATH)/Makefile.inc
else ifdef MK_INCLUDE_WIZNET_NATIVE
DEFINES		+= -DNETWORK_DEV_WIZCHIP=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/native_dhcp.c		\
	$(NETWORK_PATH)/wiz5x00.c		\
	$(NETWORK_PATH)/wizchip_driver.c
else ifdef MK_INCLUDE_WIZNET_LWIP
MK_INCLUDE_LWIP	:= 1
DEFINES		+= -DNETWORK_DEV_WIZCHIP=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/wiz5x00.c		\
	$(NETWORK_PATH)/wizchip_driver.c
endif

ifdef MK_INCLUDE_LWIP
DEFINES		+= -DNETWORK_LWIP=1
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/network_lwip.c

ifdef MK_INCLUDE_LWIP_HTTPD
DEFINES		+= -DNETWORK_OPT_HTTPD=1
EXTRAINCDIRS	+= $(NETWORK_PATH)/http
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/network_httpd.c		\
	$(NETWORK_PATH)/http/fsdata_provision.c
ifdef MK_INCLUDE_LWIP_HTTPD_CUSTOM_FSDATA
NETWORK_SRCS	+=				\
	$(NETWORK_PATH)/http/fsdata_app.c	\
	$(NETWORK_PATH)/http/fsdata_assets.c	\
	$(NETWORK_PATH)/http/fsdata_custom.c	\
	$(NETWORK_PATH)/http/fsdata_custom_data.c
endif
endif

LWIP_PATH	:= $(TOPDIR)/3rdparty/lwip
include $(LWIP_PATH)/Makefile.inc
endif

$(foreach t,$(NETWORK_SRCS),$(eval $(call make-rule-c,$(NETWORK_OBJDIR),$(t))))
