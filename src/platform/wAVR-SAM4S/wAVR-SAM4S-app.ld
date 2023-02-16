/* Entry Point */
ENTRY(start)

/* Specify the memory areas */
MEMORY {
	flash (rx)      : ORIGIN = 0x00400000, LENGTH = 256K
	xram (xr)       : ORIGIN = 0x20000000, LENGTH = 512
	sram (xrw)      : ORIGIN = 0x20000200, LENGTH = 64K-512
}

__top_flash = ORIGIN(flash) + LENGTH(flash);
__top_ram = ORIGIN(sram) + LENGTH(sram);

_Interrupt_Stack_Size = 0x600;

/* Define output sections */
SECTIONS {
	.text : ALIGN(4) {
		FILL(0xff)
		KEEP(*(.realvectors))
		KEEP(*(.resetcrt))
		*(.text)
		*(.text*)
		. = ALIGN(8);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(8);
		KEEP (*(.init))
		. = ALIGN(8);
		KEEP (*(.fini))
		. = ALIGN(4);
	} > flash

	.preinit_array : {
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >flash

	.init_array : {
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >flash

	.fini_array : {
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >flash

	. = ALIGN(4);
	_etext = .;

	/*
	 * Initialised data for the RAM vector table and .ramfunc section.
	 */
	_siramfunc = LOADADDR(.ramfunc);
	.ramfunc : {
		. = ALIGN(4);
		_sramfunc = .;
		*(.ramvectors .ramvectors.*);
		*(.ramfunc .ramfunc.*);
		. = ALIGN(4);
		_eramfunc = .;
	} >xram AT> flash

	/*
	 * All other initialised data.
	 */
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
	. = ALIGN(4);
	.bss : {
		/* This is used by the startup in order to initialize the .bss secion */
		_bss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(8);
		. = . + _Interrupt_Stack_Size;
		. = ALIGN(8);
		PROVIDE ( _stack_top = . );
	} >sram

	PROVIDE ( _heap_end = __top_ram - 0 );
}
