/**
 * @file    core_cm3.h
 * @brief   CMSIS Cortex-M3 Core Header (Minimal Stub)
 *
 * NOTE: For production use, download the official CMSIS headers from:
 *       https://github.com/ARM-software/CMSIS_5
 *
 * This stub provides only the minimal definitions needed for compilation.
 */

#ifndef __CORE_CM3_H
#define __CORE_CM3_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IO definitions */
#define __I     volatile const   /* Read only */
#define __O     volatile         /* Write only */
#define __IO    volatile         /* Read/Write */

/* ============================================================================
 * Core Register Structures
 * ============================================================================ */

/**
 * @brief System Control Block (SCB)
 */
typedef struct
{
    __I  uint32_t CPUID;        /* CPUID Base Register */
    __IO uint32_t ICSR;         /* Interrupt Control and State Register */
    __IO uint32_t VTOR;         /* Vector Table Offset Register */
    __IO uint32_t AIRCR;        /* Application Interrupt and Reset Control */
    __IO uint32_t SCR;          /* System Control Register */
    __IO uint32_t CCR;          /* Configuration Control Register */
    __IO uint8_t  SHP[12];      /* System Handlers Priority Registers */
    __IO uint32_t SHCSR;        /* System Handler Control and State */
    __IO uint32_t CFSR;         /* Configurable Fault Status Register */
    __IO uint32_t HFSR;         /* HardFault Status Register */
    __IO uint32_t DFSR;         /* Debug Fault Status Register */
    __IO uint32_t MMFAR;        /* MemManage Fault Address Register */
    __IO uint32_t BFAR;         /* BusFault Address Register */
    __IO uint32_t AFSR;         /* Auxiliary Fault Status Register */
} SCB_Type;

/**
 * @brief System Tick Timer (SysTick)
 */
typedef struct
{
    __IO uint32_t CTRL;         /* Control and Status Register */
    __IO uint32_t LOAD;         /* Reload Value Register */
    __IO uint32_t VAL;          /* Current Value Register */
    __I  uint32_t CALIB;        /* Calibration Register */
} SysTick_Type;

/**
 * @brief Nested Vectored Interrupt Controller (NVIC)
 */
typedef struct
{
    __IO uint32_t ISER[8];      /* Interrupt Set Enable Register */
         uint32_t RESERVED0[24];
    __IO uint32_t ICER[8];      /* Interrupt Clear Enable Register */
         uint32_t RESERVED1[24];
    __IO uint32_t ISPR[8];      /* Interrupt Set Pending Register */
         uint32_t RESERVED2[24];
    __IO uint32_t ICPR[8];      /* Interrupt Clear Pending Register */
         uint32_t RESERVED3[24];
    __IO uint32_t IABR[8];      /* Interrupt Active Bit Register */
         uint32_t RESERVED4[56];
    __IO uint8_t  IP[240];      /* Interrupt Priority Register */
         uint32_t RESERVED5[644];
    __O  uint32_t STIR;         /* Software Trigger Interrupt Register */
} NVIC_Type;

/* ============================================================================
 * Memory Map
 * ============================================================================ */

#define SCS_BASE        (0xE000E000UL)
#define SysTick_BASE    (SCS_BASE + 0x0010UL)
#define NVIC_BASE       (SCS_BASE + 0x0100UL)
#define SCB_BASE        (SCS_BASE + 0x0D00UL)

/* Core peripheral instances */
#define SCB             ((SCB_Type *)SCB_BASE)
#define SysTick         ((SysTick_Type *)SysTick_BASE)
#define NVIC            ((NVIC_Type *)NVIC_BASE)

/* ============================================================================
 * SysTick Control Register Definitions
 * ============================================================================ */

#define SysTick_CTRL_COUNTFLAG_Pos  16U
#define SysTick_CTRL_COUNTFLAG_Msk  (1UL << SysTick_CTRL_COUNTFLAG_Pos)

#define SysTick_CTRL_CLKSOURCE_Pos  2U
#define SysTick_CTRL_CLKSOURCE_Msk  (1UL << SysTick_CTRL_CLKSOURCE_Pos)

#define SysTick_CTRL_TICKINT_Pos    1U
#define SysTick_CTRL_TICKINT_Msk    (1UL << SysTick_CTRL_TICKINT_Pos)

#define SysTick_CTRL_ENABLE_Pos     0U
#define SysTick_CTRL_ENABLE_Msk     (1UL << SysTick_CTRL_ENABLE_Pos)

/* ============================================================================
 * NVIC Functions
 * ============================================================================ */

/**
 * @brief  Enable an external interrupt
 * @param  IRQn: Interrupt number
 */
static inline void NVIC_EnableIRQ(int IRQn)
{
    if (IRQn >= 0)
    {
        NVIC->ISER[((uint32_t)IRQn) >> 5] = (1UL << (((uint32_t)IRQn) & 0x1F));
    }
}

/**
 * @brief  Disable an external interrupt
 * @param  IRQn: Interrupt number
 */
static inline void NVIC_DisableIRQ(int IRQn)
{
    if (IRQn >= 0)
    {
        NVIC->ICER[((uint32_t)IRQn) >> 5] = (1UL << (((uint32_t)IRQn) & 0x1F));
    }
}

/**
 * @brief  Set interrupt priority
 * @param  IRQn: Interrupt number
 * @param  priority: Priority value
 */
static inline void NVIC_SetPriority(int IRQn, uint32_t priority)
{
    if (IRQn >= 0)
    {
        NVIC->IP[(uint32_t)IRQn] = (uint8_t)((priority << 4) & 0xFF);
    }
    else
    {
        SCB->SHP[(((uint32_t)IRQn) & 0xF) - 4] = (uint8_t)((priority << 4) & 0xFF);
    }
}

/**
 * @brief  Get interrupt priority
 * @param  IRQn: Interrupt number
 * @return Priority value
 */
static inline uint32_t NVIC_GetPriority(int IRQn)
{
    if (IRQn >= 0)
    {
        return ((uint32_t)NVIC->IP[(uint32_t)IRQn] >> 4);
    }
    else
    {
        return ((uint32_t)SCB->SHP[(((uint32_t)IRQn) & 0xF) - 4] >> 4);
    }
}

/**
 * @brief  Set pending interrupt
 * @param  IRQn: Interrupt number
 */
static inline void NVIC_SetPendingIRQ(int IRQn)
{
    if (IRQn >= 0)
    {
        NVIC->ISPR[((uint32_t)IRQn) >> 5] = (1UL << (((uint32_t)IRQn) & 0x1F));
    }
}

/**
 * @brief  Clear pending interrupt
 * @param  IRQn: Interrupt number
 */
static inline void NVIC_ClearPendingIRQ(int IRQn)
{
    if (IRQn >= 0)
    {
        NVIC->ICPR[((uint32_t)IRQn) >> 5] = (1UL << (((uint32_t)IRQn) & 0x1F));
    }
}

/**
 * @brief  System Reset
 */
static inline void NVIC_SystemReset(void)
{
    __asm volatile ("dsb 0xF" ::: "memory");
    SCB->AIRCR = (0x5FAUL << 16) | (1UL << 2);
    __asm volatile ("dsb 0xF" ::: "memory");
    while(1);
}

/**
 * @brief  Enable all interrupts
 */
static inline void __enable_irq(void)
{
    __asm volatile ("cpsie i" ::: "memory");
}

/**
 * @brief  Disable all interrupts
 */
static inline void __disable_irq(void)
{
    __asm volatile ("cpsid i" ::: "memory");
}

/**
 * @brief  No Operation
 */
static inline void __NOP(void)
{
    __asm volatile ("nop");
}

/**
 * @brief  Wait For Interrupt
 */
static inline void __WFI(void)
{
    __asm volatile ("wfi");
}

/**
 * @brief  Wait For Event
 */
static inline void __WFE(void)
{
    __asm volatile ("wfe");
}

/**
 * @brief  Data Synchronization Barrier
 */
static inline void __DSB(void)
{
    __asm volatile ("dsb 0xF" ::: "memory");
}

/**
 * @brief  Instruction Synchronization Barrier
 */
static inline void __ISB(void)
{
    __asm volatile ("isb 0xF" ::: "memory");
}

#ifdef __cplusplus
}
#endif

#endif /* __CORE_CM3_H */
