"""Keil 스캐터 파일 (.sct) 생성기"""


def generate_scatter_file(gen) -> str:
    """Keil 링커용 스캐터 파일 생성"""
    mem = gen.config['memory']
    stack_size = gen.parse_size(gen.config['stack']['size'])
    heap_size = gen.parse_size(gen.config['heap']['size'])
    
    flash_start = gen.parse_size(mem['flash']['start'])
    flash_size = gen.parse_size(mem['flash']['size'])
    sram_start = gen.parse_size(mem['sram']['start'])
    sram_size = gen.parse_size(mem['sram']['size'])
    
    return f'''\
; *************************************************************
; Scatter File for {gen.chip_name}
; Generated: {gen.get_timestamp()}
;
; Memory Map:
;   FLASH: 0x{flash_start:08X} - 0x{flash_start + flash_size - 1:08X} ({flash_size // 1024}KB)
;   SRAM:  0x{sram_start:08X} - 0x{sram_start + sram_size - 1:08X} ({sram_size // 1024}KB)
; *************************************************************

LR_IROM1 0x{flash_start:08X} 0x{flash_size:08X}  {{    ; Load region - entire FLASH

  ER_IROM1 0x{flash_start:08X} 0x{flash_size:08X}  {{  ; Execution region for code
    *.o (RESET, +First)                ; Vector table first
    *(InRoot$$Sections)                ; ARM library sections
    .ANY (+RO)                         ; All read-only code and data
    .ANY (+XO)                         ; Execute-only code
  }}

  RW_IRAM1 0x{sram_start:08X} 0x{sram_size:08X}  {{    ; RW data in SRAM
    .ANY (+RW +ZI)                     ; All read-write and zero-init data
  }}

  ARM_LIB_HEAP  (0x{sram_start:08X} + 0x{sram_size - stack_size - heap_size:08X}) EMPTY 0x{heap_size:08X} {{
    ; Heap region
  }}

  ARM_LIB_STACK (0x{sram_start:08X} + 0x{sram_size:08X}) EMPTY -0x{stack_size:08X} {{
    ; Stack region (grows downward)
  }}
}}
'''
