/**
 * @file    system_ha2024.h
 * @brief   System configuration header for HA2024
 * @date    2026-01-28 12:08:04
 */

#ifndef SYSTEM_HA2024_H
#define SYSTEM_HA2024_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

#define HSI_VALUE    8000000UL   /**< HSI frequency in Hz */
#define HSE_VALUE    8000000UL   /**< HSE frequency in Hz */
#define LSI_VALUE    40000UL   /**< LSI frequency in Hz */

/* Default system clock after reset (HSI) */
#define SYSCLK_DEFAULT  HSI_VALUE

/* Target system clock frequency */
#define SYSCLK_FREQ     72000000UL

/* Bus clock frequencies */
#define AHB_FREQ        72000000UL
#define APB1_FREQ       36000000UL
#define APB2_FREQ       72000000UL

/* ============================================================================
 * Global Variables
 * ============================================================================ */

extern uint32_t SystemCoreClock;    /**< Current system clock frequency */
extern uint32_t AHBClock;           /**< AHB bus clock frequency */
extern uint32_t APB1Clock;          /**< APB1 bus clock frequency */
extern uint32_t APB2Clock;          /**< APB2 bus clock frequency */

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief  System initialization (called from startup)
 */
void SystemInit(void);

/**
 * @brief  Update system clock variables
 */
void SystemCoreClockUpdate(void);

/**
 * @brief  Get current system clock frequency
 * @return Clock frequency in Hz
 */
uint32_t SystemGetCoreClock(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_HA2024_H */
