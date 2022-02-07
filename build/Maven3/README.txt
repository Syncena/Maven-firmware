Maven3 was going to be the successor to the original wAVR hardware. It
is based on a faster SoC with more on-chip Flash/SRAM, together with a
different WiFi controller (Microchip WILC1000). The latter requires
a TCP/IP stack to be built into the firmware, which freed us from the
limitations of the original WINC1500 WiFi controller's TCP/IP stack.

Unfortunately there are bugs in the WILC1000's internal firmware which
make it unreliable on busy WiFi networks. Thus the product was never
put into production. Firmware for the device is quite mature however.

Designs for the hardware will be released in due course.
