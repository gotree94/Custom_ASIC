"""링커 스크립트 생성기"""


def generate_linker_script(gen) -> str:
    """링커 스크립트 (.ld) 생성"""
    mem = gen.config['memory']
    stack_size = gen.parse_size(gen.config['stack']['size'])
    heap_size = gen.parse_size(gen.config['heap']['size'])
    
    flash_start = gen.parse_size(mem['flash']['start'])
    flash_size = gen.parse_size(mem['flash']['size'])
    sram_start = gen.parse_size(mem['sram']['start'])
    sram_size = gen.parse_size(mem['sram']['size'])
    
    script = f'''\
/*
 * Linker Script for {gen.chip_name}
 * Generated: {gen.get_timestamp()}
 * 
 * Memory Map:
 *   FLASH: 0x{flash_start:08X} - 0x{flash_start + flash_size - 1:08X} ({flash_size // 1024}KB)
 *   SRAM:  0x{sram_start:08X} - 0x{sram_start + sram_size - 1:08X} ({sram_size // 1024}KB)
 */

/* Entry point */
ENTRY(Reset_Handler)

/* Stack and Heap sizes */
_Min_Heap_Size = 0x{heap_size:X};   /* {heap_size} bytes */
_Min_Stack_Size = 0x{stack_size:X}; /* {stack_size} bytes */

/* Memory regions */
MEMORY
{{
    FLASH (rx)  : ORIGIN = 0x{flash_start:08X}, LENGTH = {flash_size // 1024}K
    SRAM (rwx)  : ORIGIN = 0x{sram_start:08X}, LENGTH = {sram_size // 1024}K
}}

/* Sections */
SECTIONS
{{
    /* Vector table - must be at start of FLASH */
    .isr_vector :
    {{
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    }} >FLASH

    /* Program code */
    .text :
    {{
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)         /* ARM-Thumb interworking */
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    }} >FLASH

    /* Read-only data */
    .rodata :
    {{
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    }} >FLASH

    /* ARM exception handling */
    .ARM.extab :
    {{
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    }} >FLASH
    
    .ARM :
    {{
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    }} >FLASH

    /* C++ constructors/destructors */
    .preinit_array :
    {{
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    }} >FLASH
    
    .init_array :
    {{
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    }} >FLASH
    
    .fini_array :
    {{
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    }} >FLASH

    /* Used by startup to initialize .data */
    _sidata = LOADADDR(.data);

    /* Initialized data - loaded from FLASH, runs in SRAM */
    .data :
    {{
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    }} >SRAM AT> FLASH

    /* Uninitialized data (zero-initialized) */
    .bss :
    {{
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    }} >SRAM

    /* User heap */
    ._user_heap_stack :
    {{
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    }} >SRAM

    /* Remove debug info from output */
    /DISCARD/ :
    {{
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }}

    /* ARM attributes */
    .ARM.attributes 0 : {{ *(.ARM.attributes) }}
}}

/* Provide symbols for startup code */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);  /* End of SRAM = Initial SP */
'''
    return script
