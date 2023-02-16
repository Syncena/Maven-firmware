/* Entry Point */
ENTRY(start)

/* Specify the memory areas */
MEMORY {
	/* Don't forget the 16KB assigned to SmartEEPROM */
	flash (rx)     : ORIGIN = 0x00000000, LENGTH = 512K-16K
	sram  (rw)     : ORIGIN = 0x20000000, LENGTH = 192K
}

__top_flash = ORIGIN(flash) + LENGTH(flash);
__top_ram = ORIGIN(sram) + LENGTH(sram);

_Interrupt_Stack_Size = 0x600;

/* Define output sections */
SECTIONS {
	.text : ALIGN(4) {
		/*
		 * First 4K contains a stripped-down vector table.
		 * The remainder of the 4K region will be disabled in
		 * the MPU to detect NULL pointer dereferences.
		 */
		FILL(0xff)
		KEEP(*(.resetvectors))
		. = ALIGN(4K);

		KEEP(*(.realvectors))
		. = ALIGN(8);
		*(.text .text.* .gnu.linkonce.t.*)
		KEEP(*(.resetcrt))
		*(.rodata .rodata* .gnu.linkonce.r.*)
		*(.ARM.extab* .gnu.linkonce.armextab.*)

		. = ALIGN(4);
		KEEP (*(.init))
		. = ALIGN(4);

		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		. = ALIGN(4);

		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
		. = ALIGN(4);

		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))

		KEEP (*(.fini))
		. = ALIGN(4);

		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))

		. = ALIGN(4);
	} > flash

	/* .ARM.exidx is sorted, so has to go in its own output section.  */
	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx : {
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > flash
	PROVIDE_HIDDEN (__exidx_end = .);

	. = ALIGN(4);
	_etext = .;

	/* RAM functions */
	_siramfunc = LOADADDR(.ramfunc);
	.ramfunc : {
		. = ALIGN(4);
		_sramfunc = .;
		*(.ramfunc .ramfunc.*);
		. = ALIGN(4);
		_eramfunc = .;
	} >sram AT> flash

	/* All other initialised data. */
	_sidata = LOADADDR(.data);
	.data : {
		. = ALIGN(4);
		_data = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} >sram AT> flash

	/* Uninitialized data section */
	.bss (NOLOAD) : {
		/* This is used by the startup in order to initialize the .bss secion */
		. = ALIGN(4);
		_sbss = .;
		_bss = .;
		*(.bss .bss.*)
		*(COMMON)
		_ebss = .;
	} >sram

	.stack (NOLOAD) : {
		. = ALIGN(8);
		. = . + _Interrupt_Stack_Size;
		. = ALIGN(8);
		_stack_top = .;
	} >sram

	. = ALIGN(8);
	_end = .;

	PROVIDE ( _heap_end = __top_ram - 0 );
	PROVIDE ( _top_flash = __top_flash );
}
