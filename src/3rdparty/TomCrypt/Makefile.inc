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

TOMCRYPT_DIR		:= $(TOMCRYPT_PATH)/libtomcrypt/src
EXTRAINCDIRS		+= $(TOMCRYPT_DIR)/headers
TOMCRYPT_OBJDIR		:= $(OBJDIR)/libtomcrypt

ifdef MK_INCLUDE_TOMCRYPT_CRYPTO
TOMCRYPT_CRYPTO_SRCS	:=					\
	$(TOMCRYPT_DIR)/ciphers/aes/aes.c			\
	$(TOMCRYPT_DIR)/hashes/helper/hash_memory.c		\
	$(TOMCRYPT_DIR)/hashes/md5.c				\
	$(TOMCRYPT_DIR)/misc/pk_get_oid.c			\
	$(TOMCRYPT_DIR)/misc/error_to_string.c			\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_unregister_prng.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_unregister_cipher.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt.c			\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_cipher_any.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_prng_descriptor.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_constants.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_prng_is_valid.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_prng_rng_descriptor.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_cipher_is_valid.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_hash_oid.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_hash_descriptor.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_all_ciphers.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_prng.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_cipher.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_hash.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_inits.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_cipher_descriptor.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_all_hashes.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_cipher.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_hash_id.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_ltc_mp_descriptor.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_cipher_id.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_hash.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_hash_is_valid.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_prng.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_argchk.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_sizes.c		\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_find_hash_any.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_register_all_prngs.c	\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_fsa.c			\
	$(TOMCRYPT_DIR)/misc/crypt/crypt_unregister_hash.c	\
	$(TOMCRYPT_DIR)/misc/pkcs5/pkcs_5_1.c			\
	$(TOMCRYPT_DIR)/misc/pkcs5/pkcs_5_2.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_setiv.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_decrypt.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_done.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_getiv.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_start.c			\
	$(TOMCRYPT_DIR)/modes/cbc/cbc_encrypt.c
endif

TOMCRYPT_SRCS	:=						\
	$(TOMCRYPT_DIR)/misc/crc32.c				\
	$(TOMCRYPT_DIR)/misc/zeromem.c				\
	$(TOMCRYPT_CRYPTO_SRCS)

LTC_FLAGS	=				\
	$(CFLAGS)				\
	-Wno-cast-qual				\
	-DLTC_SMALL_CODE			\
	-DLTC_SOURCE				\
	-DARGTYPE=3				\
	-DLTC_NOTHING=1				\
	-DLTC_MD5=1				\
	-DLTC_RIJNDAEL=1			\
	-DLTC_CBC_MODE=1			\
	-DLTC_HMAC=1				\
	-DLTC_HASH_HELPERS=1			\
	-DLTC_PKCS_5=1				\
	-DLTC_CRC32=1				\
	-DLTC_NO_FILE=1				\
	-DLTC_SMALL=1				\
	-DLTC_NO_TEST=1

define make-rule-ltc
$(1)/$(basename $(notdir $(2))).o:	$(2)
ifneq ($(VERBOSE),1)
	@echo "Compiling $$(subst $(TOPDIR)/,,$$<)"
endif
	$$(V)$$(CC) $$(LTC_FLAGS) -c $$< -o $$@
OBJS += $(1)/$(basename $(notdir $(2))).o
-include $(1)/$(basename $(notdir $(2))).d
endef

$(foreach t,$(TOMCRYPT_SRCS),$(eval $(call make-rule-ltc,$(TOMCRYPT_OBJDIR),$(t))))
