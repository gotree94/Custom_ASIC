"""디바이스 헤더 파일 생성기"""


def generate_device_header(gen) -> str:
    """디바이스 헤더 파일 (레지스터 맵) 생성"""
    peripherals = gen.config['peripherals']
    interrupts = gen.config.get('interrupts', {})
    chip = gen.config['chip']
    
    header = f'''\
/**
 * @file    {gen.chip_name_lower}.h
 * @brief   {gen.chip_name} Device Header File
 * @date    {gen.get_timestamp()}
 *
 * Peripheral register definitions and bit masks for {gen.chip_name}
 * Based on Cortex-M3 core with custom peripherals.
 */

#ifndef {gen.chip_name_upper}_H
#define {gen.chip_name_upper}_H

#ifdef __cplusplus
extern "C" {{
#endif

#include <stdint.h>

/* ============================================================================
 * Interrupt Number Definition
 * ============================================================================ */

typedef enum IRQn
{{
    /* Cortex-M3 System Exceptions */
    NonMaskableInt_IRQn     = -14,  /**< Non-Maskable Interrupt */
    HardFault_IRQn          = -13,  /**< Hard Fault */
    MemoryManagement_IRQn   = -12,  /**< Memory Management */
    BusFault_IRQn           = -11,  /**< Bus Fault */
    UsageFault_IRQn         = -10,  /**< Usage Fault */
    SVCall_IRQn             = -5,   /**< SV Call */
    DebugMonitor_IRQn       = -4,   /**< Debug Monitor */
    PendSV_IRQn             = -2,   /**< Pend SV */
    SysTick_IRQn            = -1,   /**< System Tick */

    /* {gen.chip_name} Peripheral Interrupts */
'''
    
    # 인터럽트 번호 추가
    for irq_num in sorted(interrupts.keys()):
        name, desc = interrupts[irq_num]
        header += f'    {name:30s} = {irq_num:3d},  /**< {desc} */\n'
    
    header += f'''\
}} IRQn_Type;

/* ============================================================================
 * Configuration of the Cortex-M3 Processor
 * ============================================================================ */

#define __CM3_REV               0x0201  /**< Cortex-M3 Core Revision */
#define __MPU_PRESENT           {1 if chip.get('mpu', False) else 0}       /**< MPU present */
#define __NVIC_PRIO_BITS        {chip.get('nvic_prio_bits', 4)}       /**< Number of priority bits */
#define __Vendor_SysTickConfig  0       /**< Use standard SysTick */

/* Include CMSIS Cortex-M3 core header */
#include "core_cm3.h"

/* ============================================================================
 * Peripheral Base Addresses
 * ============================================================================ */

'''
    
    # 주변장치 베이스 주소
    for periph_name, periph_info in peripherals.items():
        base = periph_info['base']
        desc = periph_info.get('description', '')
        header += f'#define {periph_name}_BASE       0x{base:08X}UL  /**< {desc} */\n'
    
    header += '''
/* ============================================================================
 * Peripheral Register Structures
 * ============================================================================ */

/**
 * @brief Reset and Clock Control (RCC)
 */
typedef struct
{
    __IO uint32_t CR;           /**< Clock control register */
    __IO uint32_t CFGR;         /**< Clock configuration register */
    __IO uint32_t CIR;          /**< Clock interrupt register */
    __IO uint32_t APB2RSTR;     /**< APB2 peripheral reset register */
    __IO uint32_t APB1RSTR;     /**< APB1 peripheral reset register */
    __IO uint32_t AHBENR;       /**< AHB peripheral clock enable register */
    __IO uint32_t APB2ENR;      /**< APB2 peripheral clock enable register */
    __IO uint32_t APB1ENR;      /**< APB1 peripheral clock enable register */
    __IO uint32_t BDCR;         /**< Backup domain control register */
    __IO uint32_t CSR;          /**< Control/status register */
} RCC_TypeDef;

/**
 * @brief Flash Memory Interface
 */
typedef struct
{
    __IO uint32_t ACR;          /**< Access control register */
    __IO uint32_t KEYR;         /**< Key register */
    __IO uint32_t OPTKEYR;      /**< Option key register */
    __IO uint32_t SR;           /**< Status register */
    __IO uint32_t CR;           /**< Control register */
    __IO uint32_t AR;           /**< Address register */
    __IO uint32_t RESERVED;
    __IO uint32_t OBR;          /**< Option byte register */
    __IO uint32_t WRPR;         /**< Write protection register */
} FLASH_TypeDef;

/**
 * @brief General Purpose I/O (GPIO)
 */
typedef struct
{
    __IO uint32_t CRL;          /**< Configuration register low */
    __IO uint32_t CRH;          /**< Configuration register high */
    __IO uint32_t IDR;          /**< Input data register */
    __IO uint32_t ODR;          /**< Output data register */
    __IO uint32_t BSRR;         /**< Bit set/reset register */
    __IO uint32_t BRR;          /**< Bit reset register */
    __IO uint32_t LCKR;         /**< Configuration lock register */
} GPIO_TypeDef;

/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter (USART)
 */
typedef struct
{
    __IO uint32_t SR;           /**< Status register */
    __IO uint32_t DR;           /**< Data register */
    __IO uint32_t BRR;          /**< Baud rate register */
    __IO uint32_t CR1;          /**< Control register 1 */
    __IO uint32_t CR2;          /**< Control register 2 */
    __IO uint32_t CR3;          /**< Control register 3 */
    __IO uint32_t GTPR;         /**< Guard time and prescaler register */
} USART_TypeDef;

/**
 * @brief Serial Peripheral Interface (SPI) / I2S
 */
typedef struct
{
    __IO uint32_t CR1;          /**< Control register 1 */
    __IO uint32_t CR2;          /**< Control register 2 */
    __IO uint32_t SR;           /**< Status register */
    __IO uint32_t DR;           /**< Data register */
    __IO uint32_t CRCPR;        /**< CRC polynomial register */
    __IO uint32_t RXCRCR;       /**< RX CRC register */
    __IO uint32_t TXCRCR;       /**< TX CRC register */
    __IO uint32_t I2SCFGR;      /**< I2S configuration register */
    __IO uint32_t I2SPR;        /**< I2S prescaler register */
} SPI_TypeDef;

/**
 * @brief Inter-Integrated Circuit (I2C)
 */
typedef struct
{
    __IO uint32_t CR1;          /**< Control register 1 */
    __IO uint32_t CR2;          /**< Control register 2 */
    __IO uint32_t OAR1;         /**< Own address register 1 */
    __IO uint32_t OAR2;         /**< Own address register 2 */
    __IO uint32_t DR;           /**< Data register */
    __IO uint32_t SR1;          /**< Status register 1 */
    __IO uint32_t SR2;          /**< Status register 2 */
    __IO uint32_t CCR;          /**< Clock control register */
    __IO uint32_t TRISE;        /**< TRISE register */
} I2C_TypeDef;

/**
 * @brief Timer (TIM)
 */
typedef struct
{
    __IO uint32_t CR1;          /**< Control register 1 */
    __IO uint32_t CR2;          /**< Control register 2 */
    __IO uint32_t SMCR;         /**< Slave mode control register */
    __IO uint32_t DIER;         /**< DMA/Interrupt enable register */
    __IO uint32_t SR;           /**< Status register */
    __IO uint32_t EGR;          /**< Event generation register */
    __IO uint32_t CCMR1;        /**< Capture/compare mode register 1 */
    __IO uint32_t CCMR2;        /**< Capture/compare mode register 2 */
    __IO uint32_t CCER;         /**< Capture/compare enable register */
    __IO uint32_t CNT;          /**< Counter */
    __IO uint32_t PSC;          /**< Prescaler */
    __IO uint32_t ARR;          /**< Auto-reload register */
    __IO uint32_t RCR;          /**< Repetition counter register */
    __IO uint32_t CCR1;         /**< Capture/compare register 1 */
    __IO uint32_t CCR2;         /**< Capture/compare register 2 */
    __IO uint32_t CCR3;         /**< Capture/compare register 3 */
    __IO uint32_t CCR4;         /**< Capture/compare register 4 */
    __IO uint32_t BDTR;         /**< Break and dead-time register */
    __IO uint32_t DCR;          /**< DMA control register */
    __IO uint32_t DMAR;         /**< DMA address for full transfer */
} TIM_TypeDef;

/**
 * @brief Analog to Digital Converter (ADC)
 */
typedef struct
{
    __IO uint32_t SR;           /**< Status register */
    __IO uint32_t CR1;          /**< Control register 1 */
    __IO uint32_t CR2;          /**< Control register 2 */
    __IO uint32_t SMPR1;        /**< Sample time register 1 */
    __IO uint32_t SMPR2;        /**< Sample time register 2 */
    __IO uint32_t JOFR1;        /**< Injected channel data offset register 1 */
    __IO uint32_t JOFR2;        /**< Injected channel data offset register 2 */
    __IO uint32_t JOFR3;        /**< Injected channel data offset register 3 */
    __IO uint32_t JOFR4;        /**< Injected channel data offset register 4 */
    __IO uint32_t HTR;          /**< Watchdog higher threshold register */
    __IO uint32_t LTR;          /**< Watchdog lower threshold register */
    __IO uint32_t SQR1;         /**< Regular sequence register 1 */
    __IO uint32_t SQR2;         /**< Regular sequence register 2 */
    __IO uint32_t SQR3;         /**< Regular sequence register 3 */
    __IO uint32_t JSQR;         /**< Injected sequence register */
    __IO uint32_t JDR1;         /**< Injected data register 1 */
    __IO uint32_t JDR2;         /**< Injected data register 2 */
    __IO uint32_t JDR3;         /**< Injected data register 3 */
    __IO uint32_t JDR4;         /**< Injected data register 4 */
    __IO uint32_t DR;           /**< Regular data register */
} ADC_TypeDef;

/**
 * @brief Digital to Analog Converter (DAC)
 */
typedef struct
{
    __IO uint32_t CR;           /**< Control register */
    __IO uint32_t SWTRIGR;      /**< Software trigger register */
    __IO uint32_t DHR12R1;      /**< Channel 1 12-bit right-aligned data */
    __IO uint32_t DHR12L1;      /**< Channel 1 12-bit left-aligned data */
    __IO uint32_t DHR8R1;       /**< Channel 1 8-bit right-aligned data */
    __IO uint32_t DHR12R2;      /**< Channel 2 12-bit right-aligned data */
    __IO uint32_t DHR12L2;      /**< Channel 2 12-bit left-aligned data */
    __IO uint32_t DHR8R2;       /**< Channel 2 8-bit right-aligned data */
    __IO uint32_t DHR12RD;      /**< Dual 12-bit right-aligned data */
    __IO uint32_t DHR12LD;      /**< Dual 12-bit left-aligned data */
    __IO uint32_t DHR8RD;       /**< Dual 8-bit right-aligned data */
    __IO uint32_t DOR1;         /**< Channel 1 data output register */
    __IO uint32_t DOR2;         /**< Channel 2 data output register */
} DAC_TypeDef;

/**
 * @brief DMA Channel
 */
typedef struct
{
    __IO uint32_t CCR;          /**< Configuration register */
    __IO uint32_t CNDTR;        /**< Number of data register */
    __IO uint32_t CPAR;         /**< Peripheral address register */
    __IO uint32_t CMAR;         /**< Memory address register */
} DMA_Channel_TypeDef;

/**
 * @brief DMA Controller
 */
typedef struct
{
    __IO uint32_t ISR;          /**< Interrupt status register */
    __IO uint32_t IFCR;         /**< Interrupt flag clear register */
} DMA_TypeDef;

/* ============================================================================
 * Peripheral Instances
 * ============================================================================ */

'''
    
    # 주변장치 인스턴스 매크로
    type_map = {
        'RCC': 'RCC_TypeDef',
        'FLASH_CTRL': 'FLASH_TypeDef',
        'GPIOA': 'GPIO_TypeDef', 'GPIOB': 'GPIO_TypeDef',
        'GPIOC': 'GPIO_TypeDef', 'GPIOD': 'GPIO_TypeDef',
        'USART1': 'USART_TypeDef', 'USART2': 'USART_TypeDef', 'USART3': 'USART_TypeDef',
        'SPI1': 'SPI_TypeDef', 'SPI2': 'SPI_TypeDef', 'I2S': 'SPI_TypeDef',
        'I2C1': 'I2C_TypeDef', 'I2C2': 'I2C_TypeDef',
        'TIM1': 'TIM_TypeDef', 'TIM2': 'TIM_TypeDef', 'TIM3': 'TIM_TypeDef', 'TIM4': 'TIM_TypeDef',
        'ADC1': 'ADC_TypeDef', 'ADC2': 'ADC_TypeDef',
        'DAC': 'DAC_TypeDef',
        'DMA1': 'DMA_TypeDef',
    }
    
    for periph_name in peripherals:
        typedef = type_map.get(periph_name, 'void')
        if typedef != 'void':
            header += f'#define {periph_name}    (({typedef}*){periph_name}_BASE)\n'
    
    # FLASH 별칭 추가
    header += '#define FLASH       ((FLASH_TypeDef*)FLASH_CTRL_BASE)\n'
    
    header += '''
/* ============================================================================
 * Bit Definitions
 * ============================================================================ */

/* ----- RCC_CR ----- */
#define RCC_CR_HSION_Pos        (0U)
#define RCC_CR_HSION            (0x1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSIRDY_Pos       (1U)
#define RCC_CR_HSIRDY           (0x1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_HSEON_Pos        (16U)
#define RCC_CR_HSEON            (0x1UL << RCC_CR_HSEON_Pos)
#define RCC_CR_HSERDY_Pos       (17U)
#define RCC_CR_HSERDY           (0x1UL << RCC_CR_HSERDY_Pos)
#define RCC_CR_HSEBYP_Pos       (18U)
#define RCC_CR_HSEBYP           (0x1UL << RCC_CR_HSEBYP_Pos)
#define RCC_CR_CSSON_Pos        (19U)
#define RCC_CR_CSSON            (0x1UL << RCC_CR_CSSON_Pos)
#define RCC_CR_PLLON_Pos        (24U)
#define RCC_CR_PLLON            (0x1UL << RCC_CR_PLLON_Pos)
#define RCC_CR_PLLRDY_Pos       (25U)
#define RCC_CR_PLLRDY           (0x1UL << RCC_CR_PLLRDY_Pos)

/* ----- RCC_CFGR ----- */
#define RCC_CFGR_SW_Pos         (0U)
#define RCC_CFGR_SW             (0x3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSI         (0x0UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSE         (0x1UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_PLL         (0x2UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SWS_Pos        (2U)
#define RCC_CFGR_SWS            (0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSI        (0x0UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSE        (0x1UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_PLL        (0x2UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_HPRE_Pos       (4U)
#define RCC_CFGR_HPRE           (0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_PPRE1_Pos      (8U)
#define RCC_CFGR_PPRE1          (0x7UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE2_Pos      (11U)
#define RCC_CFGR_PPRE2          (0x7UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PLLSRC_Pos     (16U)
#define RCC_CFGR_PLLSRC         (0x1UL << RCC_CFGR_PLLSRC_Pos)
#define RCC_CFGR_PLLMULL_Pos    (18U)
#define RCC_CFGR_PLLMULL        (0xFUL << RCC_CFGR_PLLMULL_Pos)

/* ----- FLASH_ACR ----- */
#define FLASH_ACR_LATENCY_Pos   (0U)
#define FLASH_ACR_LATENCY       (0x7UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_PRFTBE_Pos    (4U)
#define FLASH_ACR_PRFTBE        (0x1UL << FLASH_ACR_PRFTBE_Pos)

/* ----- RCC_APB2ENR ----- */
#define RCC_APB2ENR_AFIOEN      (0x1UL << 0)
#define RCC_APB2ENR_IOPAEN      (0x1UL << 2)
#define RCC_APB2ENR_IOPBEN      (0x1UL << 3)
#define RCC_APB2ENR_IOPCEN      (0x1UL << 4)
#define RCC_APB2ENR_IOPDEN      (0x1UL << 5)
#define RCC_APB2ENR_ADC1EN      (0x1UL << 9)
#define RCC_APB2ENR_ADC2EN      (0x1UL << 10)
#define RCC_APB2ENR_TIM1EN      (0x1UL << 11)
#define RCC_APB2ENR_SPI1EN      (0x1UL << 12)
#define RCC_APB2ENR_USART1EN    (0x1UL << 14)

/* ----- RCC_APB1ENR ----- */
#define RCC_APB1ENR_TIM2EN      (0x1UL << 0)
#define RCC_APB1ENR_TIM3EN      (0x1UL << 1)
#define RCC_APB1ENR_TIM4EN      (0x1UL << 2)
#define RCC_APB1ENR_WWDGEN      (0x1UL << 11)
#define RCC_APB1ENR_SPI2EN      (0x1UL << 14)
#define RCC_APB1ENR_USART2EN    (0x1UL << 17)
#define RCC_APB1ENR_USART3EN    (0x1UL << 18)
#define RCC_APB1ENR_I2C1EN      (0x1UL << 21)
#define RCC_APB1ENR_I2C2EN      (0x1UL << 22)
#define RCC_APB1ENR_DACEN       (0x1UL << 29)
#define RCC_APB1ENR_PWREN       (0x1UL << 28)

/* ----- RCC_AHBENR ----- */
#define RCC_AHBENR_DMA1EN       (0x1UL << 0)

'''
    
    header += f'''
#ifdef __cplusplus
}}
#endif

#endif /* {gen.chip_name_upper}_H */
'''
    
    return header
