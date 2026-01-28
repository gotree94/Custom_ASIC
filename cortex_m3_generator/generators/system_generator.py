"""시스템 초기화 코드 생성기"""


def generate_system_c(gen) -> str:
    """시스템 초기화 C 코드 생성"""
    clocks = gen.config['clocks']
    
    sysclk_freq = clocks['sysclk']['frequency']
    ahb_freq = clocks['ahb']['frequency']
    apb1_freq = clocks['apb1']['frequency']
    apb2_freq = clocks['apb2']['frequency']
    
    system_c = f'''\
/**
 * @file    system_{gen.chip_name_lower}.c
 * @brief   System initialization for {gen.chip_name}
 * @date    {gen.get_timestamp()}
 */

#include "{gen.chip_name_lower}.h"
#include "system_{gen.chip_name_lower}.h"

/* ============================================================================
 * Global Variables
 * ============================================================================ */

/** System clock frequency (updated by SystemCoreClockUpdate) */
uint32_t SystemCoreClock = {sysclk_freq}UL;

/** AHB clock frequency */
uint32_t AHBClock = {ahb_freq}UL;

/** APB1 clock frequency */
uint32_t APB1Clock = {apb1_freq}UL;

/** APB2 clock frequency */
uint32_t APB2Clock = {apb2_freq}UL;

/* AHB Prescaler table */
static const uint8_t AHBPrescTable[16] = {{
    0, 0, 0, 0, 0, 0, 0, 0,  /* 0-7: /1 */
    1, 2, 3, 4, 6, 7, 8, 9   /* 8-15: /2, /4, /8, /16, /64, /128, /256, /512 */
}};

/* APB Prescaler table */
static const uint8_t APBPrescTable[8] = {{
    0, 0, 0, 0, 1, 2, 3, 4   /* 0-3: /1, 4-7: /2, /4, /8, /16 */
}};

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief  Configure Flash latency based on system clock
 * @param  sysclk: System clock frequency in Hz
 */
static void SetFlashLatency(uint32_t sysclk)
{{
    uint32_t latency;
    
    if (sysclk <= 24000000) {{
        latency = 0;  /* 0 wait states */
    }} else if (sysclk <= 48000000) {{
        latency = 1;  /* 1 wait state */
    }} else {{
        latency = 2;  /* 2 wait states */
    }}
    
    /* Set flash latency */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | latency;
    
    /* Enable prefetch buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
}}

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
{{
    /* Reset RCC configuration to default */
    RCC->CR |= RCC_CR_HSION;                    /* Enable HSI */
    RCC->CFGR = 0x00000000;                     /* Reset CFGR */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON |  /* Disable HSE, CSS, PLL */
                 RCC_CR_PLLON);
    RCC->CR &= ~RCC_CR_HSEBYP;                  /* Disable HSE bypass */
    RCC->CIR = 0x009F0000;                      /* Disable all interrupts */

'''
    
    # PLL 설정
    if clocks['pll']['enabled']:
        pll_src = clocks['pll']['source']
        pll_mul = clocks['pll']['multiplier']
        
        system_c += f'''\
    /* Configure PLL */
    /* PLL source: {'HSE' if pll_src == 'hse' else 'HSI/2'} */
    /* PLL multiplier: x{pll_mul} */
    
'''
        if pll_src == 'hse':
            system_c += '''\
    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);  /* Wait for HSE ready */
    
    /* Set PLL source to HSE */
    RCC->CFGR |= RCC_CFGR_PLLSRC;
'''
        else:
            system_c += '''\
    /* HSI is already enabled, PLL source = HSI/2 */
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;
'''
        
        # PLL 배수 계산 (2-16 -> 0-14)
        pll_mul_bits = pll_mul - 2 if pll_mul <= 16 else 14
        system_c += f'''
    /* Set PLL multiplier */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) | 
                (0x{pll_mul_bits:X}UL << RCC_CFGR_PLLMULL_Pos);
'''
    
    # 버스 분주비 설정
    ahb_pre = clocks['ahb']['prescaler']
    apb1_pre = clocks['apb1']['prescaler']
    apb2_pre = clocks['apb2']['prescaler']
    
    system_c += f'''
    /* Configure bus prescalers */
    /* AHB prescaler: /{ahb_pre} */
    /* APB1 prescaler: /{apb1_pre} */
    /* APB2 prescaler: /{apb2_pre} */
    
    /* Set Flash latency for {sysclk_freq // 1000000}MHz */
    SetFlashLatency({sysclk_freq}UL);
    
'''
    
    # AHB 분주비
    ahb_bits = {1: 0, 2: 8, 4: 9, 8: 10, 16: 11, 64: 12, 128: 13, 256: 14, 512: 15}.get(ahb_pre, 0)
    apb1_bits = {1: 0, 2: 4, 4: 5, 8: 6, 16: 7}.get(apb1_pre, 0)
    apb2_bits = {1: 0, 2: 4, 4: 5, 8: 6, 16: 7}.get(apb2_pre, 0)
    
    system_c += f'''\
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | (0x{ahb_bits:X}UL << RCC_CFGR_HPRE_Pos);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (0x{apb1_bits:X}UL << RCC_CFGR_PPRE1_Pos);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (0x{apb2_bits:X}UL << RCC_CFGR_PPRE2_Pos);
'''
    
    if clocks['pll']['enabled']:
        system_c += '''
    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);  /* Wait for PLL ready */
    
    /* Select PLL as system clock source */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  /* Wait for switch */
'''
    
    system_c += f'''
    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}}

/**
 * @brief  Update SystemCoreClock variable
 *         Call this after changing clock configuration
 */
void SystemCoreClockUpdate(void)
{{
    uint32_t tmp, pllmull, pllsource;
    
    /* Get system clock source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (tmp)
    {{
        case RCC_CFGR_SWS_HSI:  /* HSI as system clock */
            SystemCoreClock = {clocks['hsi']['frequency']}UL;
            break;
            
        case RCC_CFGR_SWS_HSE:  /* HSE as system clock */
            SystemCoreClock = {clocks['hse']['frequency']}UL;
            break;
            
        case RCC_CFGR_SWS_PLL:  /* PLL as system clock */
            pllmull = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> RCC_CFGR_PLLMULL_Pos) + 2;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
            
            if (pllsource == 0)
            {{
                /* HSI/2 as PLL source */
                SystemCoreClock = ({clocks['hsi']['frequency']}UL >> 1) * pllmull;
            }}
            else
            {{
                /* HSE as PLL source */
                SystemCoreClock = {clocks['hse']['frequency']}UL * pllmull;
            }}
            break;
            
        default:
            SystemCoreClock = {clocks['hsi']['frequency']}UL;
            break;
    }}
    
    /* Calculate AHB clock */
    tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;
    AHBClock = SystemCoreClock >> AHBPrescTable[tmp];
    
    /* Calculate APB1 clock */
    tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    APB1Clock = AHBClock >> APBPrescTable[tmp];
    
    /* Calculate APB2 clock */
    tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    APB2Clock = AHBClock >> APBPrescTable[tmp];
}}

/**
 * @brief  Get current system clock frequency
 * @return System clock frequency in Hz
 */
uint32_t SystemGetCoreClock(void)
{{
    return SystemCoreClock;
}}
'''
    
    return system_c


def generate_system_h(gen) -> str:
    """시스템 헤더 파일 생성"""
    clocks = gen.config['clocks']
    
    return f'''\
/**
 * @file    system_{gen.chip_name_lower}.h
 * @brief   System configuration header for {gen.chip_name}
 * @date    {gen.get_timestamp()}
 */

#ifndef SYSTEM_{gen.chip_name_upper}_H
#define SYSTEM_{gen.chip_name_upper}_H

#ifdef __cplusplus
extern "C" {{
#endif

#include <stdint.h>

/* ============================================================================
 * Clock Configuration
 * ============================================================================ */

#define HSI_VALUE    {clocks['hsi']['frequency']}UL   /**< HSI frequency in Hz */
#define HSE_VALUE    {clocks['hse']['frequency']}UL   /**< HSE frequency in Hz */
#define LSI_VALUE    {clocks['lsi']['frequency']}UL   /**< LSI frequency in Hz */

/* Default system clock after reset (HSI) */
#define SYSCLK_DEFAULT  HSI_VALUE

/* Target system clock frequency */
#define SYSCLK_FREQ     {clocks['sysclk']['frequency']}UL

/* Bus clock frequencies */
#define AHB_FREQ        {clocks['ahb']['frequency']}UL
#define APB1_FREQ       {clocks['apb1']['frequency']}UL
#define APB2_FREQ       {clocks['apb2']['frequency']}UL

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
}}
#endif

#endif /* SYSTEM_{gen.chip_name_upper}_H */
'''
