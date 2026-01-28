/**
 * @file    ha2024_it.c
 * @brief   Interrupt handlers for HA2024
 * @date    2026-01-28 12:08:04
 *
 * Implement your interrupt handlers here.
 * Default handlers are defined as weak in startup code.
 */

#include "ha2024.h"
#include "system_ha2024.h"

/* ============================================================================
 * Cortex-M3 System Exception Handlers
 * ============================================================================ */

/**
 * @brief  Non-Maskable Interrupt handler
 */
void NMI_Handler(void)
{
    /* Add NMI handling code here */
    while (1);
}

/**
 * @brief  Hard Fault handler
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault occurs */
    while (1);
}

/**
 * @brief  Memory Management Fault handler
 */
void MemManage_Handler(void)
{
    while (1);
}

/**
 * @brief  Bus Fault handler
 */
void BusFault_Handler(void)
{
    while (1);
}

/**
 * @brief  Usage Fault handler
 */
void UsageFault_Handler(void)
{
    while (1);
}

/**
 * @brief  SVCall handler
 */
void SVC_Handler(void)
{
    /* Add SVCall handling code here */
}

/**
 * @brief  Debug Monitor handler
 */
void DebugMon_Handler(void)
{
    /* Add Debug Monitor handling code here */
}

/**
 * @brief  PendSV handler
 */
void PendSV_Handler(void)
{
    /* Used by RTOS for context switching */
}

/**
 * @brief  SysTick handler
 */
void SysTick_Handler(void)
{
    /* Add SysTick handling code here */
    /* Example: HAL_IncTick(); for HAL-based projects */
}

/* ============================================================================
 * Peripheral Interrupt Handlers
 * ============================================================================
 * Implement the handlers you need below.
 * Uncomment and modify as required.
 * ============================================================================ */

/*
 * @brief  Window Watchdog handler
 */
/*
void WWDG_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  PVD through EXTI Line detect handler
 */
/*
void PVD_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  Tamper handler
 */
/*
void TAMPER_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  RTC global handler
 */
/*
void RTC_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  Flash handler
 */
/*
void FLASH_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  RCC global handler
 */
/*
void RCC_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Line 0 handler
 */
/*
void EXTI0_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Line 1 handler
 */
/*
void EXTI1_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Line 2 handler
 */
/*
void EXTI2_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Line 3 handler
 */
/*
void EXTI3_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Line 4 handler
 */
/*
void EXTI4_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/**
 * @brief  DMA1_Channel1 interrupt handler
 */
void DMA1_Channel1_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 1))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 1);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 3))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 3);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel2 interrupt handler
 */
void DMA1_Channel2_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 5))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 5);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 7))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 7);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel3 interrupt handler
 */
void DMA1_Channel3_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 9))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 9);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 11))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 11);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel4 interrupt handler
 */
void DMA1_Channel4_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 13))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 13);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 15))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 15);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel5 interrupt handler
 */
void DMA1_Channel5_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 17))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 17);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 19))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 19);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel6 interrupt handler
 */
void DMA1_Channel6_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 21))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 21);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 23))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 23);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  DMA1_Channel7 interrupt handler
 */
void DMA1_Channel7_IRQHandler(void)
{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << 25))  /* TCIFx */
    {
        DMA1->IFCR = (1 << 25);  /* Clear flag */
        /* Handle transfer complete */
    }
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << 27))  /* TEIFx */
    {
        DMA1->IFCR = (1 << 27);  /* Clear flag */
        /* Handle transfer error */
    }
}
/**
 * @brief  ADC1/ADC2 interrupt handler
 */
void ADC1_2_IRQHandler(void)
{
    /* Check for end of conversion */
    if (ADC1->SR & (1 << 1))  /* EOC flag */
    {
        uint16_t value = ADC1->DR;
        /* Process ADC value */
        (void)value;
    }
}
/*
 * @brief  USB High Priority or CAN1 TX handler
 */
/*
void USB_HP_CAN1_TX_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  USB Low Priority or CAN1 RX0 handler
 */
/*
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  CAN1 RX1 handler
 */
/*
void CAN1_RX1_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  CAN1 SCE handler
 */
/*
void CAN1_SCE_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  EXTI Lines 9:5 handler
 */
/*
void EXTI9_5_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  TIM1 Break handler
 */
/*
void TIM1_BRK_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/**
 * @brief  TIM1 interrupt handler
 */
void TIM1_UP_IRQHandler(void)
{
    /* Check for update interrupt */
    if (TIM1->SR & 0x0001)  /* UIF flag */
    {
        TIM1->SR &= ~0x0001;  /* Clear flag */
        /* Handle timer update event */
    }
    
    /* Check for capture/compare interrupts as needed */
}
/*
 * @brief  TIM1 Trigger and Commutation handler
 */
/*
void TIM1_TRG_COM_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  TIM1 Capture Compare handler
 */
/*
void TIM1_CC_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  TIM2 global handler
 */
/*
void TIM2_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  TIM3 global handler
 */
/*
void TIM3_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  TIM4 global handler
 */
/*
void TIM4_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  I2C1 Event handler
 */
/*
void I2C1_EV_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  I2C1 Error handler
 */
/*
void I2C1_ER_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  I2C2 Event handler
 */
/*
void I2C2_EV_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  I2C2 Error handler
 */
/*
void I2C2_ER_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  SPI1 global handler
 */
/*
void SPI1_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  SPI2 global handler
 */
/*
void SPI2_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/**
 * @brief  USART1 interrupt handler
 */
void USART1_IRQHandler(void)
{
    /* Check for RX not empty */
    if (USART1->SR & (1 << 5))  /* RXNE flag */
    {
        uint8_t data = USART1->DR;
        /* Process received data */
        (void)data;
    }
    
    /* Check for TX empty */
    if (USART1->SR & (1 << 7))  /* TXE flag */
    {
        /* Send next byte or disable TX interrupt */
    }
}
/**
 * @brief  USART2 interrupt handler
 */
void USART2_IRQHandler(void)
{
    /* Check for RX not empty */
    if (USART2->SR & (1 << 5))  /* RXNE flag */
    {
        uint8_t data = USART2->DR;
        /* Process received data */
        (void)data;
    }
    
    /* Check for TX empty */
    if (USART2->SR & (1 << 7))  /* TXE flag */
    {
        /* Send next byte or disable TX interrupt */
    }
}
/**
 * @brief  USART3 interrupt handler
 */
void USART3_IRQHandler(void)
{
    /* Check for RX not empty */
    if (USART3->SR & (1 << 5))  /* RXNE flag */
    {
        uint8_t data = USART3->DR;
        /* Process received data */
        (void)data;
    }
    
    /* Check for TX empty */
    if (USART3->SR & (1 << 7))  /* TXE flag */
    {
        /* Send next byte or disable TX interrupt */
    }
}
/*
 * @brief  EXTI Lines 15:10 handler
 */
/*
void EXTI15_10_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  RTC Alarm through EXTI handler
 */
/*
void RTC_Alarm_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  USB Wakeup from suspend handler
 */
/*
void USBWakeUp_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/**
 * @brief  I2S/Audio interrupt handler
 */
void I2S_IRQHandler(void)
{
    /* Handle I2S TX/RX interrupts */
    /* Add audio processing code */
}
/*
 * @brief  DSP processing complete handler
 */
/*
void DSP_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

/*
 * @brief  Audio codec handler
 */
/*
void CODEC_IRQHandler(void)
{
    // TODO: Implement handler
}
*/

