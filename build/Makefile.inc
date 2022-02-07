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

RELEASE_BUILD	?= 0
ifeq ($(RELEASE_BUILD),1)
DEFINES		+= -DRELEASE_BUILD=1
DEFINES		+= -DNDEBUG=1
COPT		?= -Os
OBJDIR		?= obj-release
else
COPT		?= -Og
GFLAG		?= -g
OBJDIR		?= obj-debug
endif
PRODUCT		?= $(OBJDIR)/$(NAME)
DEFINES		+= -DMAVERICK_HARDWARE_ID=$(HW_ID)
WADL		?= wadl
WADL_ARGS	?= -p /dev/cu.usbmodemMAV2ZJ45UFGF81
HW_SERIAL	?= ""
CLEARTEXT_IMAGE ?= ""

.PHONY:	all binaries image clean size program

all:	binaries size

ifeq ($(VERBOSE),1)
V	:=
else
V	:= @
endif

#
# Macro to generate a rule to compile a source file to an object, and to
# create the a dependency file while we're at it.
#
# usage: $(call make-rule-?,object-directory,src-file)
#
%.o : %.c
define make-rule-c
$(1)/$(basename $(notdir $(2))).o:	$(2)
ifneq ($(VERBOSE),1)
	@echo "Compiling $$(subst $(TOPDIR)/,,$$<)"
endif
	$$(V)$$(CC) $$(CFLAGS) -c $$< -o $$@
OBJS += $(1)/$(basename $(notdir $(2))).o
-include $(1)/$(basename $(notdir $(2))).d
endef

#
# As above, but with additional compiler flags - useful suppressing
# warnings when compiling 3rd party source code.
#
define make-rule-c-flags
$(1)/$(basename $(notdir $(2))).o:	$(2)
ifneq ($(VERBOSE),1)
	@echo "Compiling $$(subst $(TOPDIR)/,,$$<)"
endif
	$$(V)$$(CC) $$(CFLAGS) $(3) -c $$< -o $$@
OBJS += $(1)/$(basename $(notdir $(2))).o
-include $(1)/$(basename $(notdir $(2))).d
endef

%.o : %.S
define make-rule-S
$(1)/$(basename $(notdir $(2))).o:	$(2)
ifneq ($(VERBOSE),1)
	@echo "Assembling $$(subst $(TOPDIR)/,,$$<)"
endif
	$(V)$$(CC) $$(ASMFLAGS) -c $$< -o $$@
OBJS += $(1)/$(basename $(notdir $(2))).o
-include $(1)/$(basename $(notdir $(2))).d
endef

#
# ==== Common source ====
#
COMMON_PATH	:= $(TOPDIR)/common
include $(COMMON_PATH)/Makefile.inc

ifdef MK_INCLUDE_NETWORK
#
# ==== Network ====
#
DEFINES		+= -DCONFIG_USE_NETWORK=1
NETWORK_PATH	:= $(TOPDIR)/network
include $(NETWORK_PATH)/Makefile.inc
endif

#
# ==== FreeRTOS ====
#
FREERTOS_PATH	:= $(TOPDIR)/3rdparty/FreeRTOS
include $(FREERTOS_PATH)/Makefile.inc

#
# ==== fixmath ====
#
LIBFIXMATH_PATH	:= $(TOPDIR)/3rdparty/libfixmath
include $(LIBFIXMATH_PATH)/Makefile.inc

#
# ==== Platform source ====
#
PLATFORM_PATH	:= $(TOPDIR)/platform/$(PLATFORM)
include $(PLATFORM_PATH)/Makefile.inc
CPU_ARCH_PATH	:= $(TOPDIR)/platform/$(CPU_ARCH)
include $(CPU_ARCH_PATH)/Makefile.inc

#
# ==== GDB ====
#
ifdef MK_INCLUDE_GDB
GDB_PATH	:= $(TOPDIR)/gdb
include $(GDB_PATH)/Makefile.inc
# For the moment, target code is lumped with GDB.
TARGET_PATH	:= $(TOPDIR)/target
include $(TARGET_PATH)/Makefile.inc
endif

#
# ==== Cortex-M support  ====
#
ifdef MK_INCLUDE_CORTEXM
CORTEXM_PATH	:= $(TOPDIR)/cortexm
include $(CORTEXM_PATH)/Makefile.inc
endif

#
# ==== AVR support  ====
#
ifdef MK_INCLUDE_AVR_PROGRAMMER
AVR_PROG_PATH	:= $(TOPDIR)/avr
include $(AVR_PROG_PATH)/Makefile.inc
endif

#
# ==== puff ====
#
ifdef MK_INCLUDE_PUFF
PUFF_PATH	:= $(TOPDIR)/3rdparty/puff
include $(PUFF_PATH)/Makefile.inc
endif

#
# ==== TomCrypt ====
#
ifdef MK_INCLUDE_TOMCRYPT
TOMCRYPT_PATH	:= $(TOPDIR)/3rdparty/TomCrypt
include $(TOMCRYPT_PATH)/Makefile.inc
endif

#
# ==== Atmel/Microchip USB ====
#
ifdef MK_INCLUDE_ATMEL_USB
ATMEL_USB_PATH	:= $(TOPDIR)/3rdparty/atmel-usb
include $(ATMEL_USB_PATH)/Makefile.inc
endif

#
# ==== Tiny USB ====
#
ifdef MK_INCLUDE_TINY_USB
TINYUSB_PATH	:= $(TOPDIR)/3rdparty/tinyusb
include $(TINYUSB_PATH)/Makefile.inc
endif

#
# ==== u8glib ====
#
ifdef MK_INCLUDE_U8GLIB
U8GLIB_PATH	:= $(TOPDIR)/3rdparty/u8glib
include $(U8GLIB_PATH)/Makefile.inc
endif


ifneq ($(HW_SERIAL),"")
WAVRIMG_S_OPT	:= -s $(HW_SERIAL)
endif
ifneq ($(CLEARTEXT_IMAGE),"")
WAVRIMG_C_OPT	:= -c $(CLEARTEXT_IMAGE)
endif

#=============================
AR	:= $(TOOLCHAIN)-ar
CC	:= $(TOOLCHAIN)-gcc
NM	:= $(TOOLCHAIN)-nm
OBJCOPY	:= $(TOOLCHAIN)-objcopy
OBJDUMP	:= $(TOOLCHAIN)-objdump
SIZE	:= $(TOOLCHAIN)-size

#=============================
SPECS		?= --specs=nano.specs
LTO		?= # empty
COMMON_FLAGS	+= $(COPT)
COMMON_FLAGS	+= $(GFLAG)
COMMON_FLAGS	+= $(CPU_ARCH_GCC_FLAGS)
COMMON_FLAGS	+= -ffunction-sections -fdata-sections
COMMON_FLAGS	+= $(SPECS)
COMMON_FLAGS	+= $(LTO)

#=============================
SOURCE_FLAGS	+= $(COMMON_FLAGS)
SOURCE_FLAGS	+= -include build_options.h
#SOURCE_FLAGS	+= -MT $(<) -MMD -MP -MD -MF $(basename $(@)).d
SOURCE_FLAGS	+= -MMD
SOURCE_FLAGS	+= -I. $(patsubst %,-I%,$(EXTRAINCDIRS))
SOURCE_FLAGS	+= $(DEFINES)

#=============================
CFLAGS		+= --std=gnu11
CFLAGS		+= -funsigned-char -funsigned-bitfields
CFLAGS		+= -Werror -Wall -Wextra -Wpedantic
CFLAGS		+= -Wstrict-prototypes -Wmissing-prototypes -Wunused
CFLAGS		+= -Wformat=2
CFLAGS		+= -Wcast-align -Wcast-qual -Wpointer-arith
CFLAGS		+= -Wmissing-declarations
CFLAGS		+= -Wwrite-strings
CFLAGS		+= -Wno-expansion-to-defined
CFLAGS		+= -Wno-attributes
CFLAGS		+= -Wdouble-promotion
CFLAGS		+= -Wmultistatement-macros
CFLAGS		+= -Wshadow
CFLAGS		+= $(SOURCE_FLAGS)

#=============================
ASMFLAGS	+= $(SOURCE_FLAGS)

#=============================
LDFLAGS		+= -Wl,-Map=$(PRODUCT).map
LDFLAGS		+= -Wl,--gc-sections -Wl,--relax
LDFLAGS		+= -Wl,--script=$(PLATFORM_LDSCRIPT)
LDFLAGS		+= $(COMMON_FLAGS)
LIBS		+= $(patsubst %,-L%,$(EXTRALIBDIRS)) $(EXTRALIBS)

binaries:	$(PREREQUISITES) \
		$(OBJDIR) $(PRODUCT).bin $(PRODUCT).lss $(PRODUCT).sym	\
		$(POSTREQUISITES)

image:	binaries
	$(V)mavimage -h $(HW_ID) -f $(FW_ID) $(WAVRIMG_S_OPT) $(WAVRIMG_C_OPT) \
	    $(PRODUCT).bin $(PRODUCT).img

$(OBJDIR):
ifneq ($(VERBOSE),1)
	@echo "Creating build directories"
endif
	$(V)mkdir -p $(sort $(dir $(OBJS)))

%.bin:	%.elf
ifneq ($(VERBOSE),1)
	@echo "Creating $@"
endif
	$(V)$(OBJCOPY) -O binary $< $@

%.lss:	%.elf
ifneq ($(VERBOSE),1)
	@echo "Creating $@"
endif
	$(V)$(OBJDUMP) -h -S $< > $@

%.sym:	%.elf
ifneq ($(VERBOSE),1)
	@echo "Creating $@"
endif
	$(V)$(NM) -n $< > $@

%.srec:	%.elf
ifneq ($(VERBOSE),1)
	@echo "Creating $@"
endif
	$(V)$(OBJCOPY) -O srec $< $(@).tmp && sed '/^S8/d' $(@).tmp > $@ && rm -f $(@).tmp

$(PRODUCT).elf:	$(OBJS)
ifneq ($(VERBOSE),1)
	@echo "Linking $@"
endif
	$(V)$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

size:	$(PRODUCT).elf
	@$(SIZE) $(PRODUCT).elf

clean:
	-rm -rf $(OBJDIR)

program:	$(PRODUCT).elf
	$(WADL) ${WADL_ARGS} $<
