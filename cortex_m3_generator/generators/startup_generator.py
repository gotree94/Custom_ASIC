"""스타트업 어셈블리 코드 생성기"""


def generate_startup(gen) -> str:
    """스타트업 어셈블리 코드 생성"""
    interrupts = gen.config.get('interrupts', {})
    boot = gen.config.get('boot', {})
    
    # 최대 인터럽트 번호
    max_irq = max(interrupts.keys()) if interrupts else 45
    
    startup = f'''\
/**
 * @file    startup_{gen.chip_name_lower}.s
 * @brief   Startup code for {gen.chip_name} (Cortex-M3)
 * @date    {gen.get_timestamp()}
 *
 * Vector table, reset handler, and default exception handlers.
 */

    .syntax unified
    .cpu cortex-m3
    .fpu softvfp
    .thumb

/* Global symbols from linker script */
.global g_pfnVectors
.global Default_Handler

/* Linker script symbols */
.word _sidata       /* Start of .data initialization values in FLASH */
.word _sdata        /* Start of .data section in SRAM */
.word _edata        /* End of .data section in SRAM */
.word _sbss         /* Start of .bss section */
.word _ebss         /* End of .bss section */
.word _estack       /* Initial stack pointer (end of SRAM) */

/**
 * @brief  Reset Handler - Entry point after reset
 */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer */
    ldr sp, =_estack

'''
    # .data 섹션 초기화
    if boot.get('init_data_section', True):
        startup += '''\
    /* Copy .data section from FLASH to SRAM */
    ldr r0, =_sdata         /* Destination: start of .data in SRAM */
    ldr r1, =_edata         /* Destination end */
    ldr r2, =_sidata        /* Source: .data init values in FLASH */
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]        /* Load from FLASH */
    str r4, [r0, r3]        /* Store to SRAM */
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

'''
    
    # .bss 섹션 초기화
    if boot.get('init_bss_section', True):
        startup += '''\
    /* Zero fill .bss section */
    ldr r2, =_sbss
    ldr r4, =_ebss
    movs r3, #0
    b LoopFillZerobss

FillZerobss:
    str r3, [r2]
    adds r2, r2, #4

LoopFillZerobss:
    cmp r2, r4
    bcc FillZerobss

'''
    
    # C++ 생성자 및 main 호출
    if boot.get('call_constructors', True):
        startup += '''\
    /* Call static constructors (C++) */
    bl __libc_init_array
    
'''
    
    startup += '''\
    /* Call main() */
    bl main

    /* If main returns, loop forever */
LoopForever:
    b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  Default Handler for unused interrupts
 */
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
    b Infinite_Loop
    .size Default_Handler, .-Default_Handler

/**
 * @brief  Vector Table
 */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    /* Cortex-M3 System Exceptions */
    .word _estack                   /* 0x00: Initial Stack Pointer */
    .word Reset_Handler             /* 0x04: Reset Handler */
    .word NMI_Handler               /* 0x08: NMI Handler */
    .word HardFault_Handler         /* 0x0C: Hard Fault Handler */
    .word MemManage_Handler         /* 0x10: MPU Fault Handler */
    .word BusFault_Handler          /* 0x14: Bus Fault Handler */
    .word UsageFault_Handler        /* 0x18: Usage Fault Handler */
    .word 0                         /* 0x1C: Reserved */
    .word 0                         /* 0x20: Reserved */
    .word 0                         /* 0x24: Reserved */
    .word 0                         /* 0x28: Reserved */
    .word SVC_Handler               /* 0x2C: SVCall Handler */
    .word DebugMon_Handler          /* 0x30: Debug Monitor Handler */
    .word 0                         /* 0x34: Reserved */
    .word PendSV_Handler            /* 0x38: PendSV Handler */
    .word SysTick_Handler           /* 0x3C: SysTick Handler */

    /* External Interrupts */
'''
    
    # 주변장치 인터럽트 벡터 추가
    for i in range(max_irq + 1):
        if i in interrupts:
            name, desc = interrupts[i]
            handler_name = name.replace('_IRQn', '_IRQHandler')
            startup += f'    .word {handler_name:30s} /* IRQ{i:2d}: {desc} */\n'
        else:
            startup += f'    .word IRQ{i}_Handler               /* IRQ{i:2d}: Reserved */\n'
    
    # Weak 핸들러 정의
    startup += '''
/**
 * @brief  Weak aliases for exception handlers
 *         Override these in your application code
 */
    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

'''
    
    # 주변장치 인터럽트 weak 핸들러
    for i in range(max_irq + 1):
        if i in interrupts:
            name = interrupts[i][0].replace('_IRQn', '_IRQHandler')
        else:
            name = f'IRQ{i}_Handler'
        startup += f'''    .weak {name}
    .thumb_set {name}, Default_Handler

'''
    
    return startup
