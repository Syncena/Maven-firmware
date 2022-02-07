libtomcrypt was used to encrypt/decrypt wAVR/Maven firmware images
when they were commercial products with a fancy bootloader.

The firmware is now open-source, so encrypted update images are no
longer necessary. However, the firmware still uses the CRC32 routine
provided by libtomcrypt, so it remains here.
