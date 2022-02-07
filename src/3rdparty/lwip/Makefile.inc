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

LWIP_SRC_PATH	:= $(LWIP_PATH)/lwip-2.1.2/src
EXTRAINCDIRS	+= $(LWIP_SRC_PATH)/include
LWIP_OBJDIR	:= $(OBJDIR)/lwip
LWIP_SRCS	:=				\
	$(LWIP_SRC_PATH)/api/api_lib.c		\
	$(LWIP_SRC_PATH)/api/api_msg.c		\
	$(LWIP_SRC_PATH)/api/err.c		\
	$(LWIP_SRC_PATH)/api/if_api.c		\
	$(LWIP_SRC_PATH)/api/netbuf.c		\
	$(LWIP_SRC_PATH)/api/netdb.c		\
	$(LWIP_SRC_PATH)/api/netifapi.c		\
	$(LWIP_SRC_PATH)/api/sockets.c		\
	$(LWIP_SRC_PATH)/api/tcpip.c		\
	$(LWIP_SRC_PATH)/netif/ethernet.c	\
	$(LWIP_SRC_PATH)/core/altcp.c		\
	$(LWIP_SRC_PATH)/core/altcp_alloc.c	\
	$(LWIP_SRC_PATH)/core/altcp_tcp.c	\
	$(LWIP_SRC_PATH)/core/def.c		\
	$(LWIP_SRC_PATH)/core/dns.c		\
	$(LWIP_SRC_PATH)/core/inet_chksum.c	\
	$(LWIP_SRC_PATH)/core/init.c		\
	$(LWIP_SRC_PATH)/core/ip.c		\
	$(LWIP_SRC_PATH)/core/mem.c		\
	$(LWIP_SRC_PATH)/core/memp.c		\
	$(LWIP_SRC_PATH)/core/netif.c		\
	$(LWIP_SRC_PATH)/core/pbuf.c		\
	$(LWIP_SRC_PATH)/core/raw.c		\
	$(LWIP_SRC_PATH)/core/stats.c		\
	$(LWIP_SRC_PATH)/core/sys.c		\
	$(LWIP_SRC_PATH)/core/tcp.c		\
	$(LWIP_SRC_PATH)/core/tcp_in.c		\
	$(LWIP_SRC_PATH)/core/tcp_out.c		\
	$(LWIP_SRC_PATH)/core/timeouts.c	\
	$(LWIP_SRC_PATH)/core/udp.c		\
	$(LWIP_SRC_PATH)/core/ipv4/autoip.c	\
	$(LWIP_SRC_PATH)/core/ipv4/dhcp.c	\
	$(LWIP_SRC_PATH)/core/ipv4/etharp.c	\
	$(LWIP_SRC_PATH)/core/ipv4/icmp.c	\
	$(LWIP_SRC_PATH)/core/ipv4/igmp.c	\
	$(LWIP_SRC_PATH)/core/ipv4/ip4.c	\
	$(LWIP_SRC_PATH)/core/ipv4/ip4_addr.c	\
	$(LWIP_SRC_PATH)/core/ipv4/ip4_frag.c	\
	$(LWIP_SRC_PATH)/core/ipv6/dhcp6.c	\
	$(LWIP_SRC_PATH)/core/ipv6/ethip6.c	\
	$(LWIP_SRC_PATH)/core/ipv6/icmp6.c	\
	$(LWIP_SRC_PATH)/core/ipv6/ip6.c	\
	$(LWIP_SRC_PATH)/core/ipv6/ip6_addr.c	\
	$(LWIP_SRC_PATH)/core/ipv6/ip6_frag.c	\
	$(LWIP_SRC_PATH)/core/ipv6/mld6.c	\
	$(LWIP_SRC_PATH)/core/ipv6/nd6.c

ifdef MK_INCLUDE_LWIP_HTTPD
LWIP_SRCS	+=				\
	$(LWIP_SRC_PATH)/apps/http/httpd.c	\
	$(LWIP_SRC_PATH)/apps/http/fs.c
endif

LWIP_CFLAGS	:= -Wno-unused-variable

$(foreach t,$(LWIP_SRCS),$(eval $(call make-rule-c-flags,$(LWIP_OBJDIR),$(t),$(LWIP_CFLAGS))))
