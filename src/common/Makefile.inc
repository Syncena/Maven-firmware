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

EXTRAINCDIRS	+= $(COMMON_PATH)

COMMON_OBJDIR	:= $(OBJDIR)/common

COMMON_SRCS	:=				\
	$(COMMON_PATH)/bitmap.c			\
	$(COMMON_PATH)/console.c		\
	$(COMMON_PATH)/debug.c			\
	$(COMMON_PATH)/hardware_common.c	\
	$(COMMON_PATH)/hexdump.c		\
	$(COMMON_PATH)/jtag_if.c		\
	$(COMMON_PATH)/jtag_bitbang.c		\
	$(COMMON_PATH)/main.c			\
	$(COMMON_PATH)/newlib_stubs.c		\
	$(COMMON_PATH)/ringbuff.c		\
	$(COMMON_PATH)/rtos.c			\
	$(COMMON_PATH)/serial_number.c		\
	$(COMMON_PATH)/stringio.c		\
	$(COMMON_PATH)/sys_event.c		\
	$(COMMON_PATH)/timer.c			\
	$(COMMON_PATH)/twiddle.c		\
	$(COMMON_PATH)/uart.c			\
	$(COMMON_PATH)/zone_alloc.c

ifdef MK_INCLUDE_CONFIGDB
DEFINES		+= -DCONFIG_USE_CONFIGDB=1
COMMON_SRCS	+=				\
	$(COMMON_PATH)/configdb.c
ifdef MK_INCLUDE_EEPROM_EMULATION
COMMON_SRCS	+=				\
	$(COMMON_PATH)/eeprom_emulation.c
endif
endif

ifdef MK_INCLUDE_DISPLAY
DEFINES		+= -DCONFIG_USE_DISPLAY=1
COMMON_SRCS	+=				\
	$(COMMON_PATH)/display.c		\
	$(COMMON_PATH)/display_bitmaps.c
endif

ifdef MK_INCLUDE_SHELL
DEFINES		+= -DCONFIG_SHELL=1
COMMON_SRCS	+=				\
	$(COMMON_PATH)/shell.c			\
	$(COMMON_PATH)/shell_app.c
endif

ifdef MK_INCLUDE_TARGET_COMMS
COMMON_SRCS	+=				\
	$(COMMON_PATH)/netusb_mux.c		\
	$(COMMON_PATH)/port_daemon.c		\
	$(COMMON_PATH)/target_comms.c		\
	$(COMMON_PATH)/target_console.c
endif

ifdef MK_INCLUDE_TOMCRYPT
COMMON_SRCS	+=				\
	$(COMMON_PATH)/crypto_crc32.c
endif

# We need to replace some newlib functions/variables with our own thread-safe
# versions. We do this using the Gnu linker's "--wrap" feature. Call out all
# the wrapped symbols here...
WRAPSYMBOLS	+= _malloc_r _calloc_r _realloc_r _free_r _mstats_r
WRAPSYMBOLS	+= __retarget_lock_init
WRAPSYMBOLS	+= __retarget_lock_init_recursive
WRAPSYMBOLS	+= __retarget_lock_close
WRAPSYMBOLS	+= __retarget_lock_close_recursive
WRAPSYMBOLS	+= __retarget_lock_acquire
WRAPSYMBOLS	+= __retarget_lock_acquire_recursive
WRAPSYMBOLS	+= __retarget_lock_try_acquire
WRAPSYMBOLS	+= __retarget_lock_try_acquire_recursive
WRAPSYMBOLS	+= __retarget_lock_release
WRAPSYMBOLS	+= __retarget_lock_release_recursive
WRAPSYMBOLS	+= __lock___sinit_recursive_mutex
WRAPSYMBOLS	+= __lock___sfp_recursive_mutex
WRAPSYMBOLS	+= __lock___env_recursive_mutex
#WRAPSYMBOLS	+= __lock___tz_mutex

$(foreach t,$(COMMON_SRCS),$(eval $(call make-rule-c,$(COMMON_OBJDIR),$(t))))
