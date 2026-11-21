/*
 * Linker script for RK3568 AMP CPU3 RT-Thread (ARM32)
 *
 * RT firmware lives at 0x03000000 (48MB), a 2MB window that is:
 *   - above the running kernel image (0x00280000, ~40MB incl. BSS)
 *   - below OP-TEE secure memory (0x08400000, carved out by U-Boot BIDRAM)
 *   - outside the AMP IPC window (0x07800000)
 * Must match amp.its, rk3568-amp.dtsi, rockchip_amp.c and ota_update.c.
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_reset)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x03000000, LENGTH = 0x00200000
}

SECTIONS
{
    . = ORIGIN(RAM);

    .text :
    {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        *(.glue_7*)
        *(.glue_7t*)
    } > RAM

    .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > RAM
    .ARM.exidx   : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } > RAM

    . = ALIGN(4);

    /* RT-Thread symbol table (for FINSH) */
    . = ALIGN(4);
    RTMSymTab : {
        __rtmsymtab_start = .;
        KEEP(*(RTMSymTab))
        __rtmsymtab_end = .;
    } > RAM

    /* FINSH symbol table */
    FSymTab : {
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
    } > RAM

    .data :
    {
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    . = ALIGN(4);
    __bss_start = .;
    .bss :
    {
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
    } > RAM
    __bss_end = .;

    _end = .;

    /* Stack top */
    __stack_top = ORIGIN(RAM) + LENGTH(RAM) - 0x10000;
}
