/*
 * Linker script for Flash applets
 */

/* Entry Point */
ENTRY(flash_applet)

/*
 * The Flash applets are position independent, so this is arbitrary.
 */
MEMORY {
	sram (rw)      : ORIGIN = 0x20000000, LENGTH = 4K
}

/* Define output sections */
SECTIONS {
	.text : ALIGN(4) {
		KEEP (*(.entry_point));
		*(.text)
		*(.text*)
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} > sram

	.data : {
		*(.data)
		*(.data*)
		. = ALIGN(4);
	} > sram

	.bss : {
		*(.bss)
		*(.bss*)
		. = ALIGN(8);
	} > sram
}
