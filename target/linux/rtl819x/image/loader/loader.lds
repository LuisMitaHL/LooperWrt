OUTPUT_ARCH(mips)
ENTRY(_start)
SECTIONS
{
  . = 0x85000000;
  .text : {
    *(.text.entry)
    *(.text*)
    *(.rodata*)
  }
  . = 0x85010000;
  .header : { KEEP(*(.header)) }
  . = ALIGN(16);
  .bss : {
    __bss_start = .;
    *(.bss*) *(COMMON)
    __bss_end = .;
  }
}
