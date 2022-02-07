Flash Applets
=============

Small code fragments downloaded into target RAM to accelerate Flash
programming.

Programming Flash memory can require many read/write operations to SoC
registers for each small chunk of Flash programmed (usually referred to
as a page of around 8-64 bytes). This can add a significant amount of
time to Flash programming operations due to round-trip delays over the
target's debug interface. Indeed, some Flash memory controllers can be
highly inefficient when programmed this way (looking at you STMicro).

By offloading the low-level register twiddling associated with Flash
programming to code running on the target itself, all Maven needs to
do is stream the contents of Flash memory to a RAM-based ring-buffer,
managed by a small shared data structure.

Erasing Flash memory is still performed by a Maven Flash driver as it
is not so time critical.

Applets must implement a single function, flash_applet_md_write_page(),
before #include'ing common.c.

The shared data structure, 'struct flash_applet_descriptor', is defined
in applet-shared.h, along with some useful inline functions.

Applets must not declare global variables, or rely on the standard C
runtime library. They must be entirely self-contained.

Applets are assigned a 32-byte stack by default, but target-specific
code can set the stack size to some other value if required.

The Makefile ensures Applets are compiled and linked to be position
independent as they can be loaded to arbitrary addresses in the target's
SRAM.

The results of 'make' are a bunch of .applet files comprised of a
comma-separated list of 8-bit hex values. These must be in #include'd
in the SoC-specific Flash driver source file as follows:

	static const uint8_t fancy_soc_flash_applet[] = {
		#include "flash-applets/applet-fancy-soc.applet"
	};

Flash drivers must then register the Applet at attach time by populating
a struct flash_applet_params with the relevant info before calling
flash_applet_attach() to acquire an opaque cookie by which the Flash
Applet API calls can then be invoked.
