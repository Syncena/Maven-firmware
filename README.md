# Maven-firmware

Maven WiFi/USB GDB Server Firmware

The Maven Programmer/Debugger was a small WiFi-enabled in-system programming
and debugging device for ARM Cortex-M micro-controllers originally designed
by Maverick Embedded Technology Ltd. In early 2022, the decision was made to
Open-Source the firmware for the device. The hardware design(s) will also
release soon.

Numerous significant features set Maven apart from other ARM programmers/debuggers:
- Built-in GDB server accessible over USB or WiFi means Maven works with any host system which can run GDB for ARM.
- Does not require middleware running on the host, such as OpenOCD, and the associated configuration file(s).
- Supports Cortex-M TRACESWO serial data in NRZ mode at up to 3MHz.
- Communicates with your target using RS232 on a UART or bit-banged I/O pin. Maven will make the UART data available over WiFi using the telnet command on your host. Both RxD and TxD are supported at all the common baud rates.
- Maven firmware supports an OLED display to keep you informed of both its status and various target parameters. It can also be configured to show the RS232 data received from the target.
- The optional USB interface provides CDC-compatible RS232 interfaces. One of those is dedicated to TRACVESWO data. Another other provides access to the same target UART interface mentioned above.

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


