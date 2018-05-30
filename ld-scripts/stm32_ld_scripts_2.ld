MEMORY
{
        RAM (rwx)       : ORIGIN = 0x20000000, LENGTH = 6K
        FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 16K
}

ld_stack = 0x20000000 + 6K - 1;

SECTIONS
{
        .text 0x20000200 : ALIGN(4)
        {
                ld_sisr = . ;

                KEEP(*(.vectors))

                . = ALIGN(4);

                *(.text)
                *(.text.*)
                *(.rodata)
                *(.rodata.*)

                . = ALIGN(4);
								/*		LDSCRIPT =  */
                ld_etext = . ;

        } > RAM

        .data : AT (ld_etext) ALIGN(4)
        {
                ld_sdata = . ;

                *(.data)
                *(.data.*)

                . = ALIGN(4);
                ld_edata = . ;

        } > RAM

        .bss : ALIGN(4)
        {
                ld_sbss = . ;

                *(.bss)
                *(.bss.*)
                *(COMMON)

                . = ALIGN(4);
                ld_ebss = . ;

        } > RAM
}
