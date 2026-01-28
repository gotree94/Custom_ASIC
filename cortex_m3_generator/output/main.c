/**
 * @file    main.c
 * @brief   Main application for HA2024
 * @date    2026-01-28 13:26:30
 *
 * Template application demonstrating basic peripheral usage.
 */

#include "ha2024.h"
#include "system_ha2024.h"

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static volatile uint32_t systick_count = 0;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */

static void GPIO_Init(void);
static void SysTick_Init(void);
static void Delay_ms(uint32_t ms);

/* ============================================================================
 * Main Function
 * ============================================================================ */

int main(void)
{
    /* System is already initialized by SystemInit() in startup code */
    
    /* Initialize peripherals */
    SysTick_Init();
    GPIO_Init();
    
    /* Main loop */
    while (1)
    {
        /* Toggle LED example (assuming LED on PC13) */
        GPIOC->ODR ^= (1 << 13);
        
        Delay_ms(500);
    }
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief  Initialize GPIO for LED
 */
static void GPIO_Init(void)
{
    /* Enable GPIOC clock */
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    
    /* Configure PC13 as push-pull output, 2MHz */
    /* Clear mode bits */
    GPIOC->CRH &= ~(0xF << 20);  /* PC13 is in CRH */
    /* Set as output push-pull, 2MHz */
    GPIOC->CRH |= (0x2 << 20);   /* MODE13 = 10 (2MHz output) */
}

/**
 * @brief  Initialize SysTick for 1ms interrupt
 */
static void SysTick_Init(void)
{
    /* Configure SysTick for 1ms interrupt */
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  Blocking delay in milliseconds
 * @param  ms: Delay time in milliseconds
 */
static void Delay_ms(uint32_t ms)
{
    uint32_t start = systick_count;
    while ((systick_count - start) < ms);
}

/* ============================================================================
 * Interrupt Handlers (override weak defaults)
 * ============================================================================ */

/**
 * @brief  SysTick interrupt handler
 */
void SysTick_Handler(void)
{
    systick_count++;
}
