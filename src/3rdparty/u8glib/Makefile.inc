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

U8GLIB_DIR	:= $(U8GLIB_PATH)/src
EXTRAINCDIRS	+= $(U8GLIB_DIR)
U8GLIB_OBJDIR	:= $(OBJDIR)/u8glib
U8GLIB_SRCS	:=				\
	$(U8GLIB_DIR)/u8g_bitmap.c		\
	$(U8GLIB_DIR)/u8g_circle.c		\
	$(U8GLIB_DIR)/u8g_clip.c		\
	$(U8GLIB_DIR)/u8g_com_api.c		\
	$(U8GLIB_DIR)/u8g_com_io.c		\
	$(U8GLIB_DIR)/u8g_com_null.c		\
	$(U8GLIB_DIR)/u8g_cursor.c		\
	$(U8GLIB_DIR)/u8g_delay.c		\
	$(U8GLIB_DIR)/u8g_dev_ssd1306_128x64.c	\
	$(U8GLIB_DIR)/u8g_dev_ssd1309_128x64.c	\
	$(U8GLIB_DIR)/u8g_ellipse.c		\
	$(U8GLIB_DIR)/u8g_font.c		\
	$(U8GLIB_DIR)/u8g_font_data.c		\
	$(U8GLIB_DIR)/u8g_line.c		\
	$(U8GLIB_DIR)/u8g_ll_api.c		\
	$(U8GLIB_DIR)/u8g_page.c		\
	$(U8GLIB_DIR)/u8g_pb.c			\
	$(U8GLIB_DIR)/u8g_pb14v1.c		\
	$(U8GLIB_DIR)/u8g_pb16h1.c		\
	$(U8GLIB_DIR)/u8g_pb16h2.c		\
	$(U8GLIB_DIR)/u8g_pb16v1.c		\
	$(U8GLIB_DIR)/u8g_pb16v2.c		\
	$(U8GLIB_DIR)/u8g_pb32h1.c		\
	$(U8GLIB_DIR)/u8g_pb8h1.c		\
	$(U8GLIB_DIR)/u8g_pb8h1f.c		\
	$(U8GLIB_DIR)/u8g_pb8h2.c		\
	$(U8GLIB_DIR)/u8g_pb8h8.c		\
	$(U8GLIB_DIR)/u8g_pb8v1.c		\
	$(U8GLIB_DIR)/u8g_pb8v2.c		\
	$(U8GLIB_DIR)/u8g_pbxh16.c		\
	$(U8GLIB_DIR)/u8g_pbxh24.c		\
	$(U8GLIB_DIR)/u8g_polygon.c		\
	$(U8GLIB_DIR)/u8g_rect.c		\
	$(U8GLIB_DIR)/u8g_rot.c			\
	$(U8GLIB_DIR)/u8g_scale.c		\
	$(U8GLIB_DIR)/u8g_state.c		\
	$(U8GLIB_DIR)/u8g_u16toa.c		\
	$(U8GLIB_DIR)/u8g_u8toa.c

$(foreach t,$(U8GLIB_SRCS),$(eval $(call make-rule-c,$(U8GLIB_OBJDIR),$(t))))
