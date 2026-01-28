/**
 * @file    ha2024.h
 * @brief   HA2024 Device Header File
 * @date    2026-01-28 12:08:04
 *
 * Peripheral register definitions and bit masks for HA2024
 * Based on Cortex-M3 core with custom peripherals.
 */

#ifndef HA2024_H
#define HA2024_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * Interrupt Number Definition
 * ============================================================================ */

typedef enum IRQn
{
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

    /* HA2024 Peripheral Interrupts */
    WWDG_IRQn                      =   0,  /**< Window Watchdog */
    PVD_IRQn                       =   1,  /**< PVD through EXTI Line detect */
    TAMPER_IRQn                    =   2,  /**< Tamper */
    RTC_IRQn                       =   3,  /**< RTC global */
    FLASH_IRQn                     =   4,  /**< Flash */
    RCC_IRQn                       =   5,  /**< RCC global */
    EXTI0_IRQn                     =   6,  /**< EXTI Line 0 */
    EXTI1_IRQn                     =   7,  /**< EXTI Line 1 */
    EXTI2_IRQn                     =   8,  /**< EXTI Line 2 */
    EXTI3_IRQn                     =   9,  /**< EXTI Line 3 */
    EXTI4_IRQn                     =  10,  /**< EXTI Line 4 */
    DMA1_Channel1_IRQn             =  11,  /**< DMA1 Channel 1 */
    DMA1_Channel2_IRQn             =  12,  /**< DMA1 Channel 2 */
    DMA1_Channel3_IRQn             =  13,  /**< DMA1 Channel 3 */
    DMA1_Channel4_IRQn             =  14,  /**< DMA1 Channel 4 */
    DMA1_Channel5_IRQn             =  15,  /**< DMA1 Channel 5 */
    DMA1_Channel6_IRQn             =  16,  /**< DMA1 Channel 6 */
    DMA1_Channel7_IRQn             =  17,  /**< DMA1 Channel 7 */
    ADC1_2_IRQn                    =  18,  /**< ADC1 and ADC2 */
    USB_HP_CAN1_TX_IRQn            =  19,  /**< USB High Priority or CAN1 TX */
    USB_LP_CAN1_RX0_IRQn           =  20,  /**< USB Low Priority or CAN1 RX0 */
    CAN1_RX1_IRQn                  =  21,  /**< CAN1 RX1 */
    CAN1_SCE_IRQn                  =  22,  /**< CAN1 SCE */
    EXTI9_5_IRQn                   =  23,  /**< EXTI Lines 9:5 */
    TIM1_BRK_IRQn                  =  24,  /**< TIM1 Break */
    TIM1_UP_IRQn                   =  25,  /**< TIM1 Update */
    TIM1_TRG_COM_IRQn              =  26,  /**< TIM1 Trigger and Commutation */
    TIM1_CC_IRQn                   =  27,  /**< TIM1 Capture Compare */
    TIM2_IRQn                      =  28,  /**< TIM2 global */
    TIM3_IRQn                      =  29,  /**< TIM3 global */
    TIM4_IRQn                      =  30,  /**< TIM4 global */
    I2C1_EV_IRQn                   =  31,  /**< I2C1 Event */
    I2C1_ER_IRQn                   =  32,  /**< I2C1 Error */
    I2C2_EV_IRQn                   =  33,  /**< I2C2 Event */
    I2C2_ER_IRQn                   =  34,  /**< I2C2 Error */
    SPI1_IRQn                      =  35,  /**< SPI1 global */
    SPI2_IRQn                      =  36,  /**< SPI2 global */
    USART1_IRQn                    =  37,  /**< USART1 global */
    USART2_IRQn                    =  38,  /**< USART2 global */
    USART3_IRQn                    =  39,  /**< USART3 global */
    EXTI15_10_IRQn                 =  40,  /**< EXTI Lines 15:10 */
    RTC_Alarm_IRQn                 =  41,  /**< RTC Alarm through EXTI */
    USBWakeUp_IRQn                 =  42,  /**< USB Wakeup from suspend */
    I2S_IRQn                       =  43,  /**< I2S global (Audio) */
    DSP_IRQn                       =  44,  /**< DSP processing complete */
    CODEC_IRQn                     =  45,  /**< Audio codec */
} IRQn_Type;

/* ============================================================================
 * Configuration of the Cortex-M3 Processor
 * ============================================================================ */

#define __CM3_REV               0x0201  /**< Cortex-M3 Core Revision */
#define __MPU_PRESENT           0       /**< MPU present */
#define __NVIC_PRIO_BITS        4       /**< Number of priority bits */
#define __Vendor_SysTickConfig  0       /**< Use standard SysTick */

/* Include CMSIS Cortex-M3 core header */
#include "core_cm3.h"

/* ============================================================================
 * Peripheral Base Addresses
 * ============================================================================ */

#define RCC_BASE       0x40021000UL  /**< Reset and Clock Control */
#define FLASH_CTRL_BASE       0x40022000UL  /**< Flash memory interface */
#define PWR_BASE       0x40007000UL  /**< Power control */
#define GPIOA_BASE       0x40010800UL  /**< General Purpose I/O Port A */
#define GPIOB_BASE       0x40010C00UL  /**< General Purpose I/O Port B */
#define GPIOC_BASE       0x40011000UL  /**< General Purpose I/O Port C */
#define GPIOD_BASE       0x40011400UL  /**< General Purpose I/O Port D */
#define TIM1_BASE       0x40012C00UL  /**< Advanced-control timer */
#define TIM2_BASE       0x40000000UL  /**< General-purpose timer */
#define TIM3_BASE       0x40000400UL  /**< General-purpose timer */
#define TIM4_BASE       0x40000800UL  /**< General-purpose timer */
#define USART1_BASE       0x40013800UL  /**< Universal synchronous asynchronous receiver transmitter */
#define USART2_BASE       0x40004400UL  /**< Universal synchronous asynchronous receiver transmitter */
#define USART3_BASE       0x40004800UL  /**< Universal synchronous asynchronous receiver transmitter */
#define SPI1_BASE       0x40013000UL  /**< Serial peripheral interface */
#define SPI2_BASE       0x40003800UL  /**< Serial peripheral interface */
#define I2C1_BASE       0x40005400UL  /**< Inter-integrated circuit */
#define I2C2_BASE       0x40005800UL  /**< Inter-integrated circuit */
#define I2S_BASE       0x40013000UL  /**< Inter-IC Sound interface */
#define ADC1_BASE       0x40012400UL  /**< Analog-to-digital converter */
#define ADC2_BASE       0x40012800UL  /**< Analog-to-digital converter */
#define DAC_BASE       0x40007400UL  /**< Digital-to-analog converter */
#define DMA1_BASE       0x40020000UL  /**< Direct memory access controller */
#define IWDG_BASE       0x40003000UL  /**< Independent watchdog */
#define WWDG_BASE       0x40002C00UL  /**< Window watchdog */
#define RTC_BASE       0x40002800UL  /**< Real-time clock */
#define EXTI_BASE       0x40010400UL  /**< External interrupt/event controller */
#define AFIO_BASE       0x40010000UL  /**< Alternate function I/O */

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

#define RCC    ((RCC_TypeDef*)RCC_BASE)
#define FLASH_CTRL    ((FLASH_TypeDef*)FLASH_CTRL_BASE)
#define GPIOA    ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB    ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC    ((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD    ((GPIO_TypeDef*)GPIOD_BASE)
#define TIM1    ((TIM_TypeDef*)TIM1_BASE)
#define TIM2    ((TIM_TypeDef*)TIM2_BASE)
#define TIM3    ((TIM_TypeDef*)TIM3_BASE)
#define TIM4    ((TIM_TypeDef*)TIM4_BASE)
#define USART1    ((USART_TypeDef*)USART1_BASE)
#define USART2    ((USART_TypeDef*)USART2_BASE)
#define USART3    ((USART_TypeDef*)USART3_BASE)
#define SPI1    ((SPI_TypeDef*)SPI1_BASE)
#define SPI2    ((SPI_TypeDef*)SPI2_BASE)
#define I2C1    ((I2C_TypeDef*)I2C1_BASE)
#define I2C2    ((I2C_TypeDef*)I2C2_BASE)
#define I2S    ((SPI_TypeDef*)I2S_BASE)
#define ADC1    ((ADC_TypeDef*)ADC1_BASE)
#define ADC2    ((ADC_TypeDef*)ADC2_BASE)
#define DAC    ((DAC_TypeDef*)DAC_BASE)
#define DMA1    ((DMA_TypeDef*)DMA1_BASE)
#define FLASH       ((FLASH_TypeDef*)FLASH_CTRL_BASE)

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


#ifdef __cplusplus
}
#endif

#endif /* HA2024_H */
