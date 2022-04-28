// STM32MP157 linker script (based on armcm_link.lds.S )
//
// Copyright (C) 2021-2026 Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_FLASH_START

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
    RETRAM_interrupts (xrw)  : ORIGIN = 0x00000000,  LENGTH = 0x00000600
    rom (rx) : ORIGIN = CONFIG_FLASH_START , LENGTH = CONFIG_FLASH_SIZE
    ram (rwx) : ORIGIN = CONFIG_RAM_START , LENGTH = CONFIG_RAM_SIZE
}

SECTIONS
{
    .vector_table : {
        . = ALIGN(4);
        _text_vectortable_start = .;
        KEEP(*(.vector_table))      /* Startup code */
        _text_vectortable_end = .;
        . = ALIGN(4);
    } > RETRAM_interrupts

    .text : {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata*)
        . = ALIGN(4);
    } > rom

    . = ALIGN(4);
    _data_flash = .;

#if CONFIG_ARMCM_RAM_VECTORTABLE
    .ram_vectortable (NOLOAD) : {
        _ram_vectortable_start = .;
        . = . + ( _text_vectortable_end - _text_vectortable_start ) ;
        _ram_vectortable_end = .;
    } > ram
#endif

    .data : AT (_data_flash)
    {
        . = ALIGN(4);
        _data_start = .;
        *(.ramfunc .ramfunc.*);
        *(.data .data.*);
        . = ALIGN(4);
        _data_end = .;
    } > ram

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > ram

    _stack_start = CONFIG_RAM_START + CONFIG_RAM_SIZE - CONFIG_STACK_SIZE ;
    .stack _stack_start (NOLOAD) :
    {
        . = . + CONFIG_STACK_SIZE;
        _stack_end = .;
    } > ram

    /DISCARD/ : {
        // The .init/.fini sections are used by __libc_init_array(), but
        // that isn't needed so no need to include them in the binary.
        *(.init)
        *(.fini)
    }
}
