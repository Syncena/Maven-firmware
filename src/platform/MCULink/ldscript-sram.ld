/*
 * Linker script for the application.
 */

/* Entry Point */
ENTRY(start)

_EEPROM_Reserved = 0x4000;

/* Specify the memory areas */
MEMORY {
	flash (rx)	: ORIGIN = 0x00000000, LENGTH = 640K - (_EEPROM_Reserved + (17 * 512))
	ramx (rx)	: ORIGIN = 0x04000000, LENGTH = 32K
	sram (rw)	: ORIGIN = 0x20000000, LENGTH = 256K
	usbram (rw)	: ORIGIN = 0x40100000, LENGTH = 16K
	/* RAM4 (16KB @ 0x20040000 is used for RTOS objects/stacks. */
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
		PROVIDE_HIDDEN (__exidx_start = .);
		PROVIDE_HIDDEN (__exidx_end = .);

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
	 * RAM-X code
	 */
	_siramxfunc = LOADADDR(.ramxfunc);
	.ramxfunc : {
		_sramxfunc = .;
		KEEP (*(.realvectors));
		*(.ramxfunc)
		*(.ramxfunc*)

		/* Selected misc functions. */
		*(.text.memcpy*)
		*(.text.target_comms_generic_worker*)
		*(.text.timer_*)
		*(.text.gdb_server*)

		/* Selected ADIv5 functions. */
		*(.text.dp_swdp_xfer*)
		*(.text.adiv5_dp_read*)
		*(.text.adiv5_dp_write*)
		*(.text.adiv5_dp_select*)
		*(.text.adiv5_dp_target_select*)
		*(.text.adiv5_dp_clear_error*)
		*(.text.adiv5_ap_read*)
		*(.text.adiv5_ap_write*)
		*(.text.adiv5_mem_ap_align*)
		*(.text.adiv5_mem_ap_sysmem_*)
		*(.text.adiv5_mem_ap_reg16*)
		*(.text.adiv5_mem_ap_reg32*)
		*(.text.adiv5_mem_ap_read16*)
		*(.text.adiv5_mem_ap_read32*)
		*(.text.adiv5_mem_ap_write16*)
		*(.text.adiv5_mem_ap_write32*)
		*(.text.adiv5_mem_ap_setup_address*)

		/* Selected ring buffer functions. */
		*(.text.ringbuff_produce*)
		*(.text.ringbuff_consume*)
		*(.text.ringbuff_blocking_read*)
		*(.text.ringbuff_blocking_getc*)
		*(.text.ringbuff_blocking_write*)
		*(.text.ringbuff_blocking_putc*)

		/* TinyUSB */
		*(.text.cdcd_*)
		*(.text.usb_cdc*)
		*(.text.usbd*)
		*(.text.advance_pointer*)
		*(.text.backward_pointer*)
		*(.text.get_relative_pointer*)
		*(.text.invoke_class_control*)
		*(.text._usb*)
		*(.text._ff_*)
		*(.text._prep_*)
		*(.text.prepare_ep_xfer*)
		*(.text.get_driver*)
		*(.text.osal_*)
		*(.text._data_*)
		*(.text._status_*)
		*(.text._invoke_class_*)
		*(.text.tu_*)
		*(.text._tu_*)
		*(.text._tusb_*)
		*(.text.tud_*)
		*(.text.dcd*)

		/* FreeRTOS */
		*(.text.pcTask*)
		*(.text.prv*)
		*(.text.vQueue*)
		*(.text.xQueue*)
		*(.text.pvTask*)
		*(.text.ulTask*)
		*(.text.vTask*)
		*(.text.xTask*)
		*(.text.pvTimer*)
		*(.text.xTimer*)
		*(.text.uxList*)
		*(.text.vList*)
		*(.text.vPort*)
		*(.text.portYIELD*)
		. = ALIGN(32);
		_eramxfunc = .;
	} >ramx AT> flash

	/*
	 * Everything else in SRAM.
	 */
	_siramfunc = LOADADDR(.ramfunc);
	.ramfunc : ALIGN(32) {
		_sramfunc = .;
		*(.text)
		*(.text*)
		*(.ramfunc)
		*(.ramfunc*)
		. = ALIGN(32);
		_eramfunc = .;
	} >sram AT> flash

	/*
	 * All other initialised data.
	 */
	_sidata = LOADADDR(.data);
	.data : ALIGN(32) {
		_data = .;
		*(.data)
		*(.data*)
		. = ALIGN(32);
		_edata = .;
	} >sram AT> flash

	/*
	 * Read-only data remains in Flash.
	 */
	.rodata : {
		. = ALIGN(8);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(8);
	} >flash

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

	/*
	 * USB RAM
	 */
	_siusbram = LOADADDR(.usbram);
	.usbram : {
		_susbram = .;
		*(.usbram)
		*(.usbram*)
		. = ALIGN(32);
		_eusbram = .;
	} >usbram AT> flash

	PROVIDE ( _heap_end = __top_ram - 0 );
	PROVIDE ( _eeprom_flash_start = (ORIGIN(flash) + LENGTH(flash)) );
	PROVIDE ( _eeprom_flash_end = _eeprom_flash_start + _EEPROM_Reserved );
}
