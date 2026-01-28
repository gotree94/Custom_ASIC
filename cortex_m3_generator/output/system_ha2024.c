/**
 * @file    system_ha2024.c
 * @brief   System initialization for HA2024
 * @date    2026-01-28 12:08:04
 */

#include "ha2024.h"
#include "system_ha2024.h"

/* ============================================================================
 * Global Variables
 * ============================================================================ */

/** System clock frequency (updated by SystemCoreClockUpdate) */
uint32_t SystemCoreClock = 72000000UL;

/** AHB clock frequency */
uint32_t AHBClock = 72000000UL;

/** APB1 clock frequency */
uint32_t APB1Clock = 36000000UL;

/** APB2 clock frequency */
uint32_t APB2Clock = 72000000UL;

/* AHB Prescaler table */
static const uint8_t AHBPrescTable[16] = {
    0, 0, 0, 0, 0, 0, 0, 0,  /* 0-7: /1 */
    1, 2, 3, 4, 6, 7, 8, 9   /* 8-15: /2, /4, /8, /16, /64, /128, /256, /512 */
};

/* APB Prescaler table */
static const uint8_t APBPrescTable[8] = {
    0, 0, 0, 0, 1, 2, 3, 4   /* 0-3: /1, 4-7: /2, /4, /8, /16 */
};

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief  Configure Flash latency based on system clock
 * @param  sysclk: System clock frequency in Hz
 */
static void SetFlashLatency(uint32_t sysclk)
{
    uint32_t latency;
    
    if (sysclk <= 24000000) {
        latency = 0;  /* 0 wait states */
    } else if (sysclk <= 48000000) {
        latency = 1;  /* 1 wait state */
    } else {
        latency = 2;  /* 2 wait states */
    }
    
    /* Set flash latency */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | latency;
    
    /* Enable prefetch buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/**
 * @brief  System initialization
 *         Called from startup code before main()
 *
 * Configures:
 *   - Flash latency
 *   - Clock source (HSI/HSE/PLL)
 *   - Bus prescalers (AHB, APB1, APB2)
 *   - Vector table location
 */
void SystemInit(void)
{
    /* Reset RCC configuration to default */
    RCC->CR |= RCC_CR_HSION;                    /* Enable HSI */
    RCC->CFGR = 0x00000000;                     /* Reset CFGR */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON |  /* Disable HSE, CSS, PLL */
                 RCC_CR_PLLON);
    RCC->CR &= ~RCC_CR_HSEBYP;                  /* Disable HSE bypass */
    RCC->CIR = 0x009F0000;                      /* Disable all interrupts */

    /* Configure PLL */
    /* PLL source: HSI/2 */
    /* PLL multiplier: x9 */
    
    /* HSI is already enabled, PLL source = HSI/2 */
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;

    /* Set PLL multiplier */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) | 
                (0x7UL << RCC_CFGR_PLLMULL_Pos);

    /* Configure bus prescalers */
    /* AHB prescaler: /1 */
    /* APB1 prescaler: /2 */
    /* APB2 prescaler: /1 */
    
    /* Set Flash latency for 72MHz */
    SetFlashLatency(72000000UL);
    
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (0x0UL << RCC_CFGR_HPRE_Pos);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (0x4UL << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (0x0UL << RCC_CFGR_PPRE2_Pos);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);  /* Wait for PLL ready */
    
    /* Select PLL as system clock source */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  /* Wait for switch */

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}

/**
 * @brief  Update SystemCoreClock variable
 *         Call this after changing clock configuration
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp, pllmull, pllsource;
    
    /* Get system clock source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (tmp)
    {
        case RCC_CFGR_SWS_HSI:  /* HSI as system clock */
            SystemCoreClock = 8000000UL;
            break;
            
        case RCC_CFGR_SWS_HSE:  /* HSE as system clock */
            SystemCoreClock = 8000000UL;
            break;
            
        case RCC_CFGR_SWS_PLL:  /* PLL as system clock */
            pllmull = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos) + 2;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
            
            if (pllsource == 0)
            {
                /* HSI/2 as PLL source */
                SystemCoreClock = (8000000UL >> 1) * pllmull;
            }
            else
            {
                /* HSE as PLL source */
                SystemCoreClock = 8000000UL * pllmull;
            }
            break;
            
        default:
            SystemCoreClock = 8000000UL;
            break;
    }
    
    /* Calculate AHB clock */
    tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    AHBClock = SystemCoreClock >> AHBPrescTable[tmp];
    
    /* Calculate APB1 clock */
    tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    APB1Clock = AHBClock >> APBPrescTable[tmp];
    
    /* Calculate APB2 clock */
    tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    APB2Clock = AHBClock >> APBPrescTable[tmp];
}

/**
 * @brief  Get current system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t SystemGetCoreClock(void)
{
    return SystemCoreClock;
}
