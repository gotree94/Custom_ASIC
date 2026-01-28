"""인터럽트 핸들러 템플릿 생성기"""

import re


def generate_interrupt_handlers(gen) -> str:
    """인터럽트 핸들러 템플릿 파일 생성"""
    interrupts = gen.config.get('interrupts', {})
    
    handlers = f'''\
/**
 * @file    {gen.chip_name_lower}_it.c
 * @brief   Interrupt handlers for {gen.chip_name}
 * @date    {gen.get_timestamp()}
 *
 * Implement your interrupt handlers here.
 * Default handlers are defined as weak in startup code.
 */

#include "{gen.chip_name_lower}.h"
#include "system_{gen.chip_name_lower}.h"

/* ============================================================================
 * Cortex-M3 System Exception Handlers
 * ============================================================================ */

/**
 * @brief  Non-Maskable Interrupt handler
 */
void NMI_Handler(void)
{{
    /* Add NMI handling code here */
    while (1);
}}

/**
 * @brief  Hard Fault handler
 */
void HardFault_Handler(void)
{{
    /* Go to infinite loop when Hard Fault occurs */
    while (1);
}}

/**
 * @brief  Memory Management Fault handler
 */
void MemManage_Handler(void)
{{
    while (1);
}}

/**
 * @brief  Bus Fault handler
 */
void BusFault_Handler(void)
{{
    while (1);
}}

/**
 * @brief  Usage Fault handler
 */
void UsageFault_Handler(void)
{{
    while (1);
}}

/**
 * @brief  SVCall handler
 */
void SVC_Handler(void)
{{
    /* Add SVCall handling code here */
}}

/**
 * @brief  Debug Monitor handler
 */
void DebugMon_Handler(void)
{{
    /* Add Debug Monitor handling code here */
}}

/**
 * @brief  PendSV handler
 */
void PendSV_Handler(void)
{{
    /* Used by RTOS for context switching */
}}

/**
 * @brief  SysTick handler
 */
void SysTick_Handler(void)
{{
    /* Add SysTick handling code here */
    /* Example: HAL_IncTick(); for HAL-based projects */
}}

/* ============================================================================
 * Peripheral Interrupt Handlers
 * ============================================================================
 * Implement the handlers you need below.
 * Uncomment and modify as required.
 * ============================================================================ */

'''
    
    # 주변장치별 핸들러 템플릿
    handler_templates = {
        'USART': '''\
/**
 * @brief  {name} interrupt handler
 */
void {handler}(void)
{{
    /* Check for RX not empty */
    if ({name}->SR & (1 << 5))  /* RXNE flag */
    {{
        uint8_t data = {name}->DR;
        /* Process received data */
        (void)data;
    }}
    
    /* Check for TX empty */
    if ({name}->SR & (1 << 7))  /* TXE flag */
    {{
        /* Send next byte or disable TX interrupt */
    }}
}}
''',
        'TIM': '''\
/**
 * @brief  {name} interrupt handler
 */
void {handler}(void)
{{
    /* Check for update interrupt */
    if ({name}->SR & 0x0001)  /* UIF flag */
    {{
        {name}->SR &= ~0x0001;  /* Clear flag */
        /* Handle timer update event */
    }}
    
    /* Check for capture/compare interrupts as needed */
}}
''',
        'ADC': '''\
/**
 * @brief  ADC1/ADC2 interrupt handler
 */
void {handler}(void)
{{
    /* Check for end of conversion */
    if (ADC1->SR & (1 << 1))  /* EOC flag */
    {{
        uint16_t value = ADC1->DR;
        /* Process ADC value */
        (void)value;
    }}
}}
''',
        'DMA': '''\
/**
 * @brief  {name} interrupt handler
 */
void {handler}(void)
{{
    /* Check transfer complete flag */
    if (DMA1->ISR & (1 << {tc_bit}))  /* TCIFx */
    {{
        DMA1->IFCR = (1 << {tc_bit});  /* Clear flag */
        /* Handle transfer complete */
    }}
    
    /* Check transfer error flag */
    if (DMA1->ISR & (1 << {te_bit}))  /* TEIFx */
    {{
        DMA1->IFCR = (1 << {te_bit});  /* Clear flag */
        /* Handle transfer error */
    }}
}}
''',
        'I2S': '''\
/**
 * @brief  I2S/Audio interrupt handler
 */
void {handler}(void)
{{
    /* Handle I2S TX/RX interrupts */
    /* Add audio processing code */
}}
''',
    }
    
    # 인터럽트 핸들러 추가
    for irq_num in sorted(interrupts.keys()):
        name, desc = interrupts[irq_num]
        handler_name = name.replace('_IRQn', '_IRQHandler')
        
        # 핸들러 유형 결정
        if 'USART' in name:
            periph_name = 'USART' + name[5] if len(name) > 5 else 'USART1'
            handlers += handler_templates['USART'].format(
                name=periph_name, handler=handler_name)
        elif 'TIM' in name and 'UP' in name:
            match = re.search(r'TIM(\d+)', name)
            tim_name = f'TIM{match.group(1)}' if match else 'TIM2'
            handlers += handler_templates['TIM'].format(
                name=tim_name, handler=handler_name)
        elif 'ADC' in name:
            handlers += handler_templates['ADC'].format(handler=handler_name)
        elif 'DMA1_Channel' in name:
            match = re.search(r'Channel(\d+)', name)
            ch = int(match.group(1)) if match else 1
            handlers += handler_templates['DMA'].format(
                name=name.replace('_IRQn', ''), 
                handler=handler_name,
                tc_bit=(ch-1)*4 + 1,
                te_bit=(ch-1)*4 + 3)
        elif 'I2S' in name:
            handlers += handler_templates['I2S'].format(handler=handler_name)
        else:
            # 기본 템플릿 (주석 처리)
            handlers += f'''\
/*
 * @brief  {desc} handler
 */
/*
void {handler_name}(void)
{{
    // TODO: Implement handler
}}
*/

'''
    
    return handlers
