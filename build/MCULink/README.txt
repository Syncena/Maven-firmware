This is the build framework for Maven on the NXP MCU-Link debug adapter.

To install the Maven firmware, put the MCU-Link into ISP mode by
installing the jumper marked "Short to enable firmware update" and
reconnecting to the host USB port.

The MCU-Link will show up as a USB Composite Device with vendor/product
IDs set to 0x1fc9/0x0021.

Go to the following URL on nxp.com:

https://www.nxp.com/design/development-boards/lpcxpresso-boards/mcu-link-debug-probe:MCU-LINK

Scroll to the bottom of the page and select the Development Software
appropriate for your host operating system (Windows, MacOS, or Linux).
This download contains a 'blhost' tool for uploading a firmware image
onto the MCU-Link, as well as a copy of the device's latest NXP firmware.
You can use this to revert back to the MCU-Link's original behaviour if
you so desire.

The following instructions assume your host system is running MacOS, and
you have a shell prompt in a Terminal.app window.

Unpack the above software package and copy the "blhost" program contained
within it to somewhere on your shell's $PATH. Then run it with the following
options to verify connectivity:

 $ blhost -u 0x1fc9,0x0021 get-property 1
 Inject command 'get-property'
 Response status = 0 (0x0) Success.
 Response word 1 = 1258487808 (0x4b030000)
 Current Version = K3.0.0

You should see something similar to the above output. You can now Flash
the Maven image contained in the build directory:

 $ blhost -u 0x1fc9,0x0021 flash-image cortexm/obj-debug/mculink.srec erase
 Inject command 'flash-image'
 Successful generic response to command 'flash-erase-region'
 Wrote 147024 bytes to address 0
 Successful generic response to command 'write-memory'
 (1/1)100% Completed!
 Successful generic response to command 'write-memory'
 Response status = 0 (0x0) Success.

Disconnect MCU-Link from the USB port and remove the ISP jumper.

On reconnecting USB, MCU-Link will show up as two USB serial ports.
The first port is running a Gdb Server, while the second gives
access to MCU-Link's target serial port pins.

Connect GDB to the MCU-Link as you would for a Blackmagic probe.
The "mon help" command will provide a list of available commands.

Maven firmware supports most Microchip SAM SoCs, and many STM32 SoCs,
as well as NXP's LPC55S69 on the MCU-Link itself.

Note that this is beta-quality firmware at the moment; Maven was
designed to run on a probe with TCP/IP network connectivity, so the
MCU-Link is less than ideal. There is also a known issue with USB
on some MacOS platforms, indicated by Gdb timeouts and protocol errors.
This is a host-side problem, specifically with the AppleUSBXHCIPPT driver
and USB3 host ports. The only known work-around is to switch to a USB2
port which uses one of the AppleUSBEHCI* drivers.

Please direct any questions to @maverick on the 1BitSquared Discord Server.
