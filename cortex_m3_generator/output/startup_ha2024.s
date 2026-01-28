/**
 * @file    startup_ha2024.s
 * @brief   Startup code for HA2024 (Cortex-M3)
 * @date    2026-01-28 13:26:30
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

    /* Call static constructors (C++) */
    bl __libc_init_array
    
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
    .word WWDG_IRQHandler                /* IRQ 0: Window Watchdog */
    .word PVD_IRQHandler                 /* IRQ 1: PVD through EXTI Line detect */
    .word TAMPER_IRQHandler              /* IRQ 2: Tamper */
    .word RTC_IRQHandler                 /* IRQ 3: RTC global */
    .word FLASH_IRQHandler               /* IRQ 4: Flash */
    .word RCC_IRQHandler                 /* IRQ 5: RCC global */
    .word EXTI0_IRQHandler               /* IRQ 6: EXTI Line 0 */
    .word EXTI1_IRQHandler               /* IRQ 7: EXTI Line 1 */
    .word EXTI2_IRQHandler               /* IRQ 8: EXTI Line 2 */
    .word EXTI3_IRQHandler               /* IRQ 9: EXTI Line 3 */
    .word EXTI4_IRQHandler               /* IRQ10: EXTI Line 4 */
    .word DMA1_Channel1_IRQHandler       /* IRQ11: DMA1 Channel 1 */
    .word DMA1_Channel2_IRQHandler       /* IRQ12: DMA1 Channel 2 */
    .word DMA1_Channel3_IRQHandler       /* IRQ13: DMA1 Channel 3 */
    .word DMA1_Channel4_IRQHandler       /* IRQ14: DMA1 Channel 4 */
    .word DMA1_Channel5_IRQHandler       /* IRQ15: DMA1 Channel 5 */
    .word DMA1_Channel6_IRQHandler       /* IRQ16: DMA1 Channel 6 */
    .word DMA1_Channel7_IRQHandler       /* IRQ17: DMA1 Channel 7 */
    .word ADC1_2_IRQHandler              /* IRQ18: ADC1 and ADC2 */
    .word USB_HP_CAN1_TX_IRQHandler      /* IRQ19: USB High Priority or CAN1 TX */
    .word USB_LP_CAN1_RX0_IRQHandler     /* IRQ20: USB Low Priority or CAN1 RX0 */
    .word CAN1_RX1_IRQHandler            /* IRQ21: CAN1 RX1 */
    .word CAN1_SCE_IRQHandler            /* IRQ22: CAN1 SCE */
    .word EXTI9_5_IRQHandler             /* IRQ23: EXTI Lines 9:5 */
    .word TIM1_BRK_IRQHandler            /* IRQ24: TIM1 Break */
    .word TIM1_UP_IRQHandler             /* IRQ25: TIM1 Update */
    .word TIM1_TRG_COM_IRQHandler        /* IRQ26: TIM1 Trigger and Commutation */
    .word TIM1_CC_IRQHandler             /* IRQ27: TIM1 Capture Compare */
    .word TIM2_IRQHandler                /* IRQ28: TIM2 global */
    .word TIM3_IRQHandler                /* IRQ29: TIM3 global */
    .word TIM4_IRQHandler                /* IRQ30: TIM4 global */
    .word I2C1_EV_IRQHandler             /* IRQ31: I2C1 Event */
    .word I2C1_ER_IRQHandler             /* IRQ32: I2C1 Error */
    .word I2C2_EV_IRQHandler             /* IRQ33: I2C2 Event */
    .word I2C2_ER_IRQHandler             /* IRQ34: I2C2 Error */
    .word SPI1_IRQHandler                /* IRQ35: SPI1 global */
    .word SPI2_IRQHandler                /* IRQ36: SPI2 global */
    .word USART1_IRQHandler              /* IRQ37: USART1 global */
    .word USART2_IRQHandler              /* IRQ38: USART2 global */
    .word USART3_IRQHandler              /* IRQ39: USART3 global */
    .word EXTI15_10_IRQHandler           /* IRQ40: EXTI Lines 15:10 */
    .word RTC_Alarm_IRQHandler           /* IRQ41: RTC Alarm through EXTI */
    .word USBWakeUp_IRQHandler           /* IRQ42: USB Wakeup from suspend */
    .word I2S_IRQHandler                 /* IRQ43: I2S global (Audio) */
    .word DSP_IRQHandler                 /* IRQ44: DSP processing complete */
    .word CODEC_IRQHandler               /* IRQ45: Audio codec */

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

    .weak WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak PVD_IRQHandler
    .thumb_set PVD_IRQHandler, Default_Handler

    .weak TAMPER_IRQHandler
    .thumb_set TAMPER_IRQHandler, Default_Handler

    .weak RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler

    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler

    .weak EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler

    .weak EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler

    .weak EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler

    .weak EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler

    .weak EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler

    .weak DMA1_Channel1_IRQHandler
    .thumb_set DMA1_Channel1_IRQHandler, Default_Handler

    .weak DMA1_Channel2_IRQHandler
    .thumb_set DMA1_Channel2_IRQHandler, Default_Handler

    .weak DMA1_Channel3_IRQHandler
    .thumb_set DMA1_Channel3_IRQHandler, Default_Handler

    .weak DMA1_Channel4_IRQHandler
    .thumb_set DMA1_Channel4_IRQHandler, Default_Handler

    .weak DMA1_Channel5_IRQHandler
    .thumb_set DMA1_Channel5_IRQHandler, Default_Handler

    .weak DMA1_Channel6_IRQHandler
    .thumb_set DMA1_Channel6_IRQHandler, Default_Handler

    .weak DMA1_Channel7_IRQHandler
    .thumb_set DMA1_Channel7_IRQHandler, Default_Handler

    .weak ADC1_2_IRQHandler
    .thumb_set ADC1_2_IRQHandler, Default_Handler

    .weak USB_HP_CAN1_TX_IRQHandler
    .thumb_set USB_HP_CAN1_TX_IRQHandler, Default_Handler

    .weak USB_LP_CAN1_RX0_IRQHandler
    .thumb_set USB_LP_CAN1_RX0_IRQHandler, Default_Handler

    .weak CAN1_RX1_IRQHandler
    .thumb_set CAN1_RX1_IRQHandler, Default_Handler

    .weak CAN1_SCE_IRQHandler
    .thumb_set CAN1_SCE_IRQHandler, Default_Handler

    .weak EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler, Default_Handler

    .weak TIM1_BRK_IRQHandler
    .thumb_set TIM1_BRK_IRQHandler, Default_Handler

    .weak TIM1_UP_IRQHandler
    .thumb_set TIM1_UP_IRQHandler, Default_Handler

    .weak TIM1_TRG_COM_IRQHandler
    .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler

    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler

    .weak TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler

    .weak I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler

    .weak I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler

    .weak I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler

    .weak I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler

    .weak SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler

    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler

    .weak EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler, Default_Handler

    .weak RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler

    .weak USBWakeUp_IRQHandler
    .thumb_set USBWakeUp_IRQHandler, Default_Handler

    .weak I2S_IRQHandler
    .thumb_set I2S_IRQHandler, Default_Handler

    .weak DSP_IRQHandler
    .thumb_set DSP_IRQHandler, Default_Handler

    .weak CODEC_IRQHandler
    .thumb_set CODEC_IRQHandler, Default_Handler

