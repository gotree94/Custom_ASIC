"""Keil용 스타트업 어셈블리 코드 생성기 (ARM 어셈블러 문법)"""


def generate_keil_startup(gen) -> str:
    """Keil ARM 어셈블러용 스타트업 코드 생성"""
    interrupts = gen.config.get('interrupts', {})
    stack_size = gen.parse_size(gen.config['stack']['size'])
    heap_size = gen.parse_size(gen.config['heap']['size'])
    
    max_irq = max(interrupts.keys()) if interrupts else 45
    
    startup = f'''\
;**************************************************************************
;* @file    startup_{gen.chip_name_lower}.s
;* @brief   {gen.chip_name} startup code for Keil MDK-ARM
;* @date    {gen.get_timestamp()}
;**************************************************************************

; Amount of memory (in bytes) allocated for Stack and Heap
Stack_Size      EQU     0x{stack_size:08X}
Heap_Size       EQU     0x{heap_size:08X}

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB

;**************************************************************************
; Vector Table - mapped to address 0 at Reset
;**************************************************************************
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
'''
    
    # 주변장치 인터럽트 벡터
    for i in range(max_irq + 1):
        if i in interrupts:
            name, desc = interrupts[i]
            handler = name.replace('_IRQn', '_IRQHandler')
            startup += f'                DCD     {handler:30s}; {desc}\n'
        else:
            startup += f'                DCD     IRQ{i}_Handler                 ; Reserved\n'
    
    startup += '''
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

;**************************************************************************
; Reset Handler
;**************************************************************************
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

;**************************************************************************
; Default Exception Handlers
;**************************************************************************
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP

HardFault_Handler PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP

MemManage_Handler PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP

BusFault_Handler PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP

UsageFault_Handler PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP

SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP

DebugMon_Handler PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP

PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP

SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

;**************************************************************************
; Default Peripheral Interrupt Handlers
;**************************************************************************
Default_Handler PROC
'''
    
    # 주변장치 인터럽트 핸들러 EXPORT
    for i in range(max_irq + 1):
        if i in interrupts:
            handler = interrupts[i][0].replace('_IRQn', '_IRQHandler')
        else:
            handler = f'IRQ{i}_Handler'
        startup += f'                EXPORT  {handler:30s}[WEAK]\n'
    
    startup += '\n'
    
    # 핸들러 레이블
    for i in range(max_irq + 1):
        if i in interrupts:
            handler = interrupts[i][0].replace('_IRQn', '_IRQHandler')
        else:
            handler = f'IRQ{i}_Handler'
        startup += f'{handler}\n'
    
    startup += '''                B       .
                ENDP

                ALIGN

;**************************************************************************
; User Stack and Heap initialization
;**************************************************************************
                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap
                LDR     R0, =Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, =(Heap_Mem + Heap_Size)
                LDR     R3, =Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END
'''
    
    return startup
