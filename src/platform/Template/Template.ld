/*
 * Linker script for the application.
 */

/* Entry Point */
ENTRY(start)

/* Specify the memory areas */
MEMORY {
	flash (rx)	: ORIGIN = 0x00000000, LENGTH = 256K
	sram (rw)	: ORIGIN = 0x20000000, LENGTH = 256K
}

__top_ram = ORIGIN(sram) + LENGTH(sram);

_Interrupt_Stack_Size = 0x600;

/* Define output sections */
SECTIONS {
	/*
	 * The main read-only firmware section.
	 */
	.text : ALIGN(4) {
		FILL(0xff)
		KEEP (*(.resetvectors));
		KEEP(*(.resetcrt))
		/*
		 * Pad to 4 KB. The region 0x000 -> 0xfff will be disabled
		 * in the MPU to catch NULL pointer dereferences.
		 */
		. = ALIGN(4096);
		*(.text)
		*(.text*)
		. = ALIGN(8);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} > flash

	/* newlib's preinit, init, and finish arrays are next. */
	.fini_array : {
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);

		/* Library init routines. */
		KEEP (*(.init))
		. = ALIGN(8);
		KEEP (*(.init*))
		. = ALIGN(4);
		KEEP (*(.fini))
		. = ALIGN(4);

		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >flash

	. = ALIGN(4);
	_etext = .;

	/*
	 * RAM code
	 */
	_siramfunc = LOADADDR(.ramfunc);
	.ramfunc : {
		_sramfunc = .;
		KEEP (*(.realvectors));
		*(.ramfunc)
		*(.ramfunc*)
		. = ALIGN(4);
		_eramfunc = .;
	} >sram AT> flash

	/*
	 * All other initialised data.
	 */
	_sidata = LOADADDR(.data);
	.data : {
		_data = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} >sram AT> flash

	/* Uninitialized data section */
	. = ALIGN(4);
	.bss : {
		_bss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(8);
		_ebss = .;
		_stack_bottom = .;
		. = . + _Interrupt_Stack_Size;
		. = ALIGN(8);
		PROVIDE ( _stack_top = . );
	} >sram

	PROVIDE ( _heap_end = __top_ram - 0 );

	/* Remove information from the standard libraries */
	/DISCARD/ : {
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
	}
}
