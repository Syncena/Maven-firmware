# Maven-firmware

Maven/wAVR WiFi and USB Debug Probe Firmware

The Maven Programmer/Debugger is a small WiFi-enabled in-system programming
and debugging device for ARM Cortex-M micro-controllers originally designed
by Maverick Embedded Technology Ltd. In early 2022, the decision was made to
Open-Source the firmware for the device. The hardware design(s) will also
released in due course.

Numerous significant features set Maven apart from other ARM programmers/debuggers:
- Built-in GDB server accessible over USB or WiFi means Maven works with any host system which can run GDB for ARM.
- Does not require middleware running on the host, such as OpenOCD, and the associated configuration file(s).
- Supports Cortex-M TRACESWO serial data in NRZ mode at up to 3MHz.
- Communicates with your target using RS232 on a UART or bit-banged I/O pin. Maven will make the UART data available over WiFi using the telnet command on your host. Both RxD and TxD are supported at all the common baud rates.
- Maven firmware supports an OLED display to keep you informed of both its status and various target parameters.
- The optional USB interface provides CDC-compatible RS232 interfaces. One of those is dedicated to TRACVESWO data. Another other provides access to the same target UART interface mentioned above. Note: the number of CDC interfaces is hardware-dependent.

# GDB Server Support
Maven’s primary purpose is to act as a GDB server on behalf of the target
micro-controller. In this role, the Gnu Debugger (GDB), or IDE supporting
the GDB remote protocol, running on a host computer can control and inspect
the state of the target’s CPU core and its connected peripherals using the
GDB ‘extended-remote’ target protocol. Maven translates instructions received
from the debugger into the appropriate register and memory read/write
requests over the ARM CPU’s ADIv5 interface known as the Serial Wire Debug
Port (SWD-DP).

Maven supports breakpoints both in RAM and in Flash memory. The latter
utilising Cortex-M hardware breakpoints where available - most Cortex-M cores
have around 6 hardware breakpoints. Maven supports up to 128 software
breakpoints for code running in RAM. In most cases GDB will choose the
appropriate type of breakpoint automatically, since Maven informs GDB of
the address/size/type of memory regions for supported targets.

Hardware watchpoints are also supported, where available on the target
Cortex-M core. These can be used to halt execution when firmware accesses
specific memory addresses, and are an invaluable tool to help track down
memory corruption and/or use-after-free type bugs.

# Building Maven firmware from source
You'll need the [ARM toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
together with a reasonably recent version of Gnu make to build the Maven firmware.

You'll also need to build `tools/makefsdata` if you update the html source
in `src/network/http/html`.

Makefiles are provided to build Maven firmware for a number of targets
and configurations. For example:

```
$ cd build/wAVR/cortexm
$ make
```

The above will create a debug firmware image for the original wAVR/Maven
hardware. Alternatively:

```
$ cd build/MCULink/cortexm
$ make
```

The above will create a debug firmware image for the NXP MCULink
debug probe.

Note that the author's development environment is MacOS. Your mileage may
vary when building on another platform.

# Maven on MCU-Link
The MCU-Link hardware from NXP is a small USB debug probe whose firmware
implements the usual CMSIS-DAP protocol. This is probably the best
device with which to start playing with Maven firmware as it is very
easy to replace its default firmware image.

See `build/MCULink/README.txt` for information on how to replace the stock
firmware with Maven.

# AVR Programming
Maven started life as a WiFi programmer for AVR devices, working in
conjunction with [avrdude](https://github.com/avrdudes/avrdude). This
version of the firmware can be built using the Makefiles in `build/wAVR/avr`
and `build/Maven3/avr`.
