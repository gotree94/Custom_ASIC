# Cortex-M3 Firmware Generator

커스텀 Cortex-M3 기반 칩을 위한 펌웨어 기본 파일 생성기입니다.

---

## 목차

1. [Cortex-M3 부팅 시퀀스](#1-cortex-m3-부팅-시퀀스)
2. [펌웨어 구성 파일의 이해](#2-펌웨어-구성-파일의-이해)
3. [생성기 사용법](#3-생성기-사용법)
4. [설정 파일 상세 가이드](#4-설정-파일-상세-가이드)
5. [빌드 및 플래시](#5-빌드-및-플래시)
6. [커스터마이징 가이드](#6-커스터마이징-가이드)

---

## 1. Cortex-M3 부팅 시퀀스

### 1.1 리셋 직후 하드웨어 동작

MCU에 전원이 인가되거나 리셋이 발생하면, Cortex-M3 코어는 다음 동작을 **하드웨어적으로 자동 수행**합니다:

```
┌─────────────────────────────────────────────────────────────┐
│                    전원 인가 / 리셋                           │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│  1. 벡터 테이블 0x00000000에서 첫 번째 워드 읽기              │
│     → 이 값을 SP(Stack Pointer)에 로드                       │
│     → 보통 SRAM의 끝 주소 (예: 0x20010000)                   │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│  2. 벡터 테이블 0x00000004에서 두 번째 워드 읽기              │
│     → 이 값을 PC(Program Counter)에 로드                     │
│     → Reset_Handler 함수의 주소                              │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Reset_Handler로 점프하여 실행 시작                        │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 벡터 테이블 구조

벡터 테이블은 Flash 메모리의 시작점(보통 0x08000000)에 위치하며, 부팅 시 0x00000000에 매핑됩니다.

| 오프셋 | 내용 | 설명 |
|--------|------|------|
| 0x00 | Initial SP | 스택 포인터 초기값 (SRAM 끝) |
| 0x04 | Reset_Handler | 리셋 핸들러 주소 |
| 0x08 | NMI_Handler | Non-Maskable Interrupt |
| 0x0C | HardFault_Handler | 하드 폴트 |
| 0x10 | MemManage_Handler | 메모리 관리 폴트 |
| 0x14 | BusFault_Handler | 버스 폴트 |
| 0x18 | UsageFault_Handler | 사용 폴트 |
| 0x1C-0x28 | Reserved | 예약됨 |
| 0x2C | SVC_Handler | 슈퍼바이저 콜 |
| 0x30 | DebugMon_Handler | 디버그 모니터 |
| 0x34 | Reserved | 예약됨 |
| 0x38 | PendSV_Handler | Pendable 서비스 요청 |
| 0x3C | SysTick_Handler | 시스템 틱 타이머 |
| 0x40+ | IRQ0_Handler... | 외부 인터럽트 핸들러들 |

### 1.3 스타트업 코드 (Reset_Handler) 실행 흐름

Reset_Handler는 C 코드의 main() 함수가 실행되기 전에 필요한 초기화를 수행합니다:

```
Reset_Handler 시작
        │
        ▼
┌───────────────────────────────────────┐
│ ① 스택 포인터 재설정 (선택적)          │
│    ldr sp, =_estack                   │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ② .data 섹션 초기화                    │
│    - Flash에 저장된 초기값을            │
│    - SRAM의 .data 영역으로 복사         │
│    예: int global_var = 100;          │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ③ .bss 섹션 제로 초기화                │
│    - 초기값 없는 전역변수 영역을         │
│    - 0으로 채움                        │
│    예: int uninitialized_var;         │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ④ SystemInit() 호출 (선택적)          │
│    - 클럭 설정                         │
│    - PLL 구성                          │
│    - Flash latency 설정               │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ⑤ C++ 생성자 호출 (선택적)             │
│    __libc_init_array()                │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ⑥ main() 호출                         │
│    → 사용자 애플리케이션 시작           │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│ ⑦ main() 리턴 시 무한 루프             │
│    (정상적으로는 리턴하지 않음)          │
└───────────────────────────────────────┘
```

### 1.4 메모리 맵 이해

```
┌────────────────────────────────────────────────────────┐
│                    메모리 맵 예시                        │
├────────────────────────────────────────────────────────┤
│                                                        │
│  0xE000E000 ┌──────────────────┐                       │
│             │   System Control │ ← NVIC, SysTick, SCB  │
│  0xE0000000 └──────────────────┘                       │
│             │       ...        │                       │
│             │                  │                       │
│  0x40000000 ┌──────────────────┐                       │
│             │   Peripherals    │ ← GPIO, UART, SPI...  │
│             └──────────────────┘                       │
│             │       ...        │                       │
│             │                  │                       │
│  0x20010000 ┌──────────────────┐ ← _estack (SP 초기값) │
│             │      Stack ↓     │   (스택은 아래로 성장) │
│             │                  │                       │
│             │      Heap ↑      │   (힙은 위로 성장)     │
│             ├──────────────────┤                       │
│             │      .bss        │ ← 제로 초기화 데이터   │
│             ├──────────────────┤                       │
│             │      .data       │ ← 초기화된 데이터      │
│  0x20000000 └──────────────────┘ ← SRAM 시작           │
│             │       ...        │                       │
│             │                  │                       │
│  0x08040000 ┌──────────────────┐ ← Flash 끝            │
│             │                  │                       │
│             │  .rodata         │ ← 상수 데이터          │
│             │  .text           │ ← 프로그램 코드        │
│             │  .data (LMA)     │ ← .data 초기값 저장    │
│             ├──────────────────┤                       │
│             │  Vector Table    │ ← 벡터 테이블         │
│  0x08000000 └──────────────────┘ ← Flash 시작          │
│                                                        │
└────────────────────────────────────────────────────────┘

LMA (Load Memory Address): 데이터가 저장된 위치 (Flash)
VMA (Virtual Memory Address): 실행 시 데이터가 위치할 곳 (SRAM)
```

---

## 2. 펌웨어 구성 파일의 이해

커스텀 칩을 위한 펌웨어를 만들 때, 다음 파일들이 **왜 필요한지** 이해하는 것이 중요합니다.

### 2.1 스타트업 코드 (startup_xxx.s)

**목적**: 하드웨어 리셋 후 C 코드 실행 환경을 준비

**필요한 이유**:
- C 언어는 스택, 초기화된 변수, 제로 초기화된 변수가 준비되어 있다고 가정함
- 리셋 직후에는 아무것도 준비되지 않은 상태
- 어셈블리로 작성해야 하는 이유: C 런타임 자체가 아직 준비되지 않음

**주요 내용**:
```asm
; 벡터 테이블 정의
g_pfnVectors:
    .word _estack           ; 초기 스택 포인터
    .word Reset_Handler     ; 리셋 핸들러
    .word NMI_Handler       ; NMI 핸들러
    ...

; 리셋 핸들러
Reset_Handler:
    ; .data 복사 (Flash → SRAM)
    ; .bss 제로화
    ; main() 호출
```

**커스텀 칩에서 수정할 부분**:
- 스택/힙 크기
- 인터럽트 벡터 (칩에 맞는 주변장치 인터럽트 추가)

### 2.2 링커 스크립트 (xxx.ld / xxx.sct)

**목적**: 코드와 데이터를 메모리의 어디에 배치할지 정의

**필요한 이유**:
- 컴파일러는 코드를 생성하지만, 어디에 배치할지 모름
- Flash와 SRAM의 시작 주소, 크기는 칩마다 다름
- 벡터 테이블은 반드시 Flash 시작점에 위치해야 함

**GNU LD 스크립트 (.ld) 구조**:
```ld
/* 메모리 영역 정의 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 64K
}

/* 섹션 배치 */
SECTIONS
{
    .isr_vector : { *(.isr_vector) } >FLASH    /* 벡터 테이블 */
    .text       : { *(.text*) } >FLASH          /* 코드 */
    .rodata     : { *(.rodata*) } >FLASH        /* 상수 */
    .data       : { *(.data*) } >SRAM AT>FLASH  /* 초기화 데이터 */
    .bss        : { *(.bss*) } >SRAM            /* 제로 초기화 */
}
```

**Keil 스캐터 파일 (.sct) 구조**:
```
LR_IROM1 0x08000000 0x00040000 {    ; Load Region
  ER_IROM1 0x08000000 0x00040000 {  ; Execution Region (Flash)
    *.o (RESET, +First)              ; 벡터 테이블 먼저
    .ANY (+RO)                       ; 모든 읽기 전용
  }
  RW_IRAM1 0x20000000 0x00010000 {  ; RW Region (SRAM)
    .ANY (+RW +ZI)                   ; 읽기/쓰기 + 제로 초기화
  }
}
```

**커스텀 칩에서 수정할 부분**:
- Flash 시작 주소 및 크기
- SRAM 시작 주소 및 크기
- 추가 메모리 영역 (외부 SRAM, 특수 영역 등)

### 2.3 시스템 초기화 (system_xxx.c)

**목적**: 클럭 시스템 설정, 시스템 주파수 구성

**필요한 이유**:
- 리셋 후 MCU는 내부 RC 오실레이터(HSI)로 저속 동작
- 원하는 성능을 위해 PLL 설정으로 고속 클럭 생성 필요
- Flash 접근 속도에 맞는 wait state 설정 필요

**주요 함수**:
```c
void SystemInit(void)
{
    // 1. 내부 고속 오실레이터(HSI) 활성화
    RCC->CR |= RCC_CR_HSION;
    
    // 2. PLL 설정 (HSI/2 × 9 = 36MHz, 또는 HSE × 9 = 72MHz)
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    
    // 3. Flash latency 설정 (72MHz면 2 wait state)
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    
    // 4. PLL 활성화 및 시스템 클럭 소스로 선택
    RCC->CR |= RCC_CR_PLLON;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    
    // 5. 버스 클럭 분주비 설정 (AHB, APB1, APB2)
}
```

**커스텀 칩에서 수정할 부분**:
- 오실레이터 주파수 (HSI, HSE)
- PLL 배수/분주 설정
- 버스 클럭 분주비
- Flash latency 테이블

### 2.4 디바이스 헤더 (xxx.h)

**목적**: 주변장치 레지스터 주소 및 비트 정의

**필요한 이유**:
- 주변장치를 제어하려면 레지스터 주소를 알아야 함
- 매직 넘버 대신 의미 있는 이름 사용
- 구조체로 레지스터 그룹 접근 용이

**구조체 방식의 레지스터 정의**:
```c
/* GPIO 레지스터 구조체 */
typedef struct
{
    __IO uint32_t CRL;    // 0x00: Configuration Register Low
    __IO uint32_t CRH;    // 0x04: Configuration Register High
    __IO uint32_t IDR;    // 0x08: Input Data Register
    __IO uint32_t ODR;    // 0x0C: Output Data Register
    __IO uint32_t BSRR;   // 0x10: Bit Set/Reset Register
    __IO uint32_t BRR;    // 0x14: Bit Reset Register
    __IO uint32_t LCKR;   // 0x18: Lock Register
} GPIO_TypeDef;

/* 베이스 주소 정의 */
#define GPIOA_BASE    0x40010800UL
#define GPIOA         ((GPIO_TypeDef*)GPIOA_BASE)

/* 비트 정의 */
#define GPIO_ODR_OD13    (1UL << 13)

/* 사용 예 */
GPIOA->ODR |= GPIO_ODR_OD13;   // PA13 HIGH
GPIOA->ODR &= ~GPIO_ODR_OD13;  // PA13 LOW
```

**커스텀 칩에서 수정할 부분**:
- 주변장치 베이스 주소
- 레지스터 구조체 (칩 고유 레지스터)
- 비트 필드 정의
- 인터럽트 번호 열거형

### 2.5 인터럽트 핸들러 (xxx_it.c)

**목적**: 인터럽트 서비스 루틴(ISR) 구현

**필요한 이유**:
- 스타트업 코드에서 weak로 선언된 핸들러를 오버라이드
- 실제 인터럽트 발생 시 처리할 코드 작성

**구조**:
```c
/* SysTick 핸들러 - 1ms마다 호출 */
void SysTick_Handler(void)
{
    systick_count++;
}

/* UART 수신 인터럽트 */
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = USART1->DR;
        // 데이터 처리
    }
}
```

---

## 3. 생성기 사용법

### 3.1 설치

```bash
# Python 3.7 이상 필요
pip install pyyaml
```

### 3.2 파일 생성

```bash
# 모든 툴체인 (GNU GCC + Keil MDK)
python firmware_generator.py chip_config.yaml -o ./my_chip

# GNU GCC 전용
python firmware_generator.py chip_config.yaml -o ./my_chip --gcc

# Keil MDK 전용
python firmware_generator.py chip_config.yaml -o ./my_chip --keil
```

### 3.3 생성되는 파일

| 파일 | GNU GCC | Keil MDK | 설명 |
|------|:-------:|:--------:|------|
| `system_xxx.c/h` | ✓ | ✓ | 시스템 초기화 |
| `xxx.h` | ✓ | ✓ | 레지스터 맵 |
| `xxx_it.c` | ✓ | ✓ | 인터럽트 핸들러 |
| `main.c` | ✓ | ✓ | 메인 함수 예제 |
| `startup_xxx.s` | ✓ | - | GNU AS 스타트업 |
| `xxx.ld` | ✓ | - | GNU 링커 스크립트 |
| `Makefile` | ✓ | - | GNU Make |
| `startup_xxx_keil.s` | - | ✓ | ARM ASM 스타트업 |
| `xxx.sct` | - | ✓ | Keil 스캐터 파일 |
| `xxx.uvprojx` | - | ✓ | Keil 프로젝트 |
| `build_keil.bat` | - | ✓ | 빌드 스크립트 |

---

## 4. 설정 파일 상세 가이드

### 4.1 전체 구조

```yaml
# chip_config.yaml

chip:           # 칩 기본 정보
memory:         # 메모리 맵
stack:          # 스택 설정
heap:           # 힙 설정
clocks:         # 클럭 설정
peripherals:    # 주변장치 정의
interrupts:     # 인터럽트 벡터
boot:           # 부트 옵션
```

### 4.2 chip 섹션

```yaml
chip:
  name: "HA2024"              # 칩 이름 (파일명에 사용)
  vendor: "MyCompany"         # 제조사 이름
  core: "cortex-m3"           # 코어 종류
  fpu: false                  # FPU 유무 (M3는 없음)
  mpu: false                  # MPU 유무
  nvic_prio_bits: 4           # NVIC 우선순위 비트 수 (보통 3~4)
```

**nvic_prio_bits 설명**:
- 4비트 = 16단계 우선순위 (0~15)
- 3비트 = 8단계 우선순위 (0~7)
- 값이 클수록 세밀한 우선순위 제어 가능

### 4.3 memory 섹션

```yaml
memory:
  flash:
    name: "FLASH"
    start: 0x08000000         # Flash 시작 주소
    size: 256K                # 크기 (256K, 512K, 1M 등)
    page_size: 1K             # 플래시 페이지 크기 (소거 단위)
    
  sram:
    name: "SRAM"
    start: 0x20000000         # SRAM 시작 주소
    size: 64K                 # 크기
    
  # 추가 메모리 영역 (선택)
  sram2:
    name: "SRAM2"
    start: 0x20010000
    size: 16K
```

**일반적인 메모리 주소**:
| 영역 | 주소 범위 | 용도 |
|------|----------|------|
| Flash | 0x08000000 ~ | 프로그램 코드 저장 |
| SRAM | 0x20000000 ~ | 실행 시 데이터 |
| Peripherals | 0x40000000 ~ | 주변장치 레지스터 |
| Cortex-M | 0xE0000000 ~ | 코어 내부 (NVIC 등) |

### 4.4 stack/heap 섹션

```yaml
stack:
  size: 0x400                 # 1KB 스택
  
heap:
  size: 0x200                 # 512B 힙
```

**크기 결정 가이드**:
- **스택**: 함수 호출 깊이, 지역 변수 크기에 따라 결정
  - 일반적: 1KB ~ 4KB
  - RTOS 사용 시: 태스크당 별도 스택 필요
- **힙**: malloc 사용량에 따라 결정
  - 동적 할당 미사용 시: 최소값 또는 0
  - 적극 사용 시: 필요량의 1.5배 이상

### 4.5 clocks 섹션

```yaml
clocks:
  # 내부 고속 오실레이터
  hsi:
    frequency: 8000000        # 8MHz
    enabled: true
    
  # 외부 고속 오실레이터
  hse:
    frequency: 8000000        # 8MHz 크리스털
    enabled: false
    bypass: false             # 외부 클럭 직접 입력 시 true
    
  # 내부 저속 오실레이터 (RTC, IWDG용)
  lsi:
    frequency: 40000          # 40kHz
    enabled: true
    
  # PLL 설정
  pll:
    enabled: true
    source: "hsi"             # "hsi" 또는 "hse"
    multiplier: 9             # 배수 (2~16)
                              # HSI/2 × 9 = 4MHz × 9 = 36MHz
                              # HSE × 9 = 8MHz × 9 = 72MHz
    
  # 시스템 클럭
  sysclk:
    source: "pll"             # "hsi", "hse", "pll"
    frequency: 72000000       # 목표 주파수
    
  # 버스 클럭
  ahb:
    prescaler: 1              # AHB 분주비 (1,2,4,8,16,64,128,256,512)
    frequency: 72000000
    
  apb1:
    prescaler: 2              # APB1 분주비 (1,2,4,8,16)
    frequency: 36000000       # APB1 최대 36MHz 제한
    
  apb2:
    prescaler: 1              # APB2 분주비
    frequency: 72000000
```

**클럭 트리 예시**:
```
        HSI (8MHz)                    HSE (8MHz 크리스털)
            │                               │
            ▼                               ▼
        [/2 분주]                      [선택 가능]
            │                               │
            └───────────┬───────────────────┘
                        │
                        ▼
                   [PLL MUX] ──▶ [PLL ×9] ──▶ 72MHz
                        │
                        ▼
                   [SW MUX] ──▶ SYSCLK (72MHz)
                        │
            ┌───────────┼───────────┐
            ▼           ▼           ▼
         AHB (/1)   APB1 (/2)   APB2 (/1)
         72MHz      36MHz       72MHz
            │           │           │
            ▼           ▼           ▼
         DMA        TIM2~4      GPIO
         Flash      USART2,3    USART1
                    I2C, SPI2   SPI1, ADC
```

### 4.6 peripherals 섹션

```yaml
peripherals:
  # 시스템 제어
  RCC:
    base: 0x40021000
    description: "Reset and Clock Control"
    
  FLASH_CTRL:
    base: 0x40022000
    description: "Flash memory interface"
    
  # GPIO
  GPIOA:
    base: 0x40010800
    description: "General Purpose I/O Port A"
    
  GPIOB:
    base: 0x40010C00
    description: "General Purpose I/O Port B"
    
  # 통신
  USART1:
    base: 0x40013800
    description: "USART1"
    
  SPI1:
    base: 0x40013000
    description: "SPI1"
    
  I2C1:
    base: 0x40005400
    description: "I2C1"
    
  # 타이머
  TIM2:
    base: 0x40000000
    description: "General-purpose timer"
    
  # ADC/DAC (오디오)
  ADC1:
    base: 0x40012400
    description: "ADC1"
    channels: 16
    resolution: 12
    
  DAC:
    base: 0x40007400
    description: "DAC"
    channels: 2
    resolution: 12
    
  # I2S (오디오 인터페이스)
  I2S:
    base: 0x40013000          # SPI1과 공유
    description: "I2S Audio Interface"
```

**주변장치 주소 찾기**:
- 레퍼런스 칩(STM32 등)의 데이터시트 참조
- 커스텀 칩 설계 문서 확인
- 일반적인 배치:
  - APB1: 0x40000000 ~ (저속 주변장치)
  - APB2: 0x40010000 ~ (고속 주변장치)
  - AHB: 0x40020000 ~ (DMA, 메모리 컨트롤러)

### 4.7 interrupts 섹션

```yaml
interrupts:
  # 번호: [이름, 설명]
  # IRQ0부터 시작 (벡터 테이블 16번째부터)
  
  0:  [WWDG_IRQn,           "Window Watchdog"]
  1:  [PVD_IRQn,            "PVD through EXTI"]
  2:  [TAMPER_IRQn,         "Tamper"]
  3:  [RTC_IRQn,            "RTC global"]
  4:  [FLASH_IRQn,          "Flash"]
  5:  [RCC_IRQn,            "RCC global"]
  6:  [EXTI0_IRQn,          "EXTI Line 0"]
  7:  [EXTI1_IRQn,          "EXTI Line 1"]
  # ... 중략 ...
  
  37: [USART1_IRQn,         "USART1 global"]
  38: [USART2_IRQn,         "USART2 global"]
  
  # 인터럽트 추가
  43: [I2S_IRQn,            "I2S Audio"]
  44: [DSP_IRQn,            "DSP complete"]
  45: [CODEC_IRQn,          "Audio codec"]
```

**인터럽트 번호 규칙**:
- 0~15: Cortex-M3 시스템 예외 (고정)
- 16~: 칩 고유 주변장치 인터럽트
- YAML에서는 주변장치 인터럽트만 정의 (0번 = IRQ0 = 벡터 테이블 16번)

### 4.8 boot 섹션

```yaml
boot:
  vtor_remap: false           # VTOR 레지스터로 벡터 테이블 이동
  boot_from_flash: true       # Flash에서 부팅 (일반적)
  
  init_data_section: true     # .data 섹션 초기화 수행
  init_bss_section: true      # .bss 섹션 제로 초기화 수행
  call_constructors: true     # C++ 전역 생성자 호출
  
  enable_semihosting: false   # 세미호스팅 (디버깅용 printf)
```

---

## 5. 빌드 및 플래시

### 5.1 GNU GCC 빌드

```bash
cd my_chip

# 빌드
make                    # Release 빌드
make DEBUG=1            # Debug 빌드 (최적화 없음, 디버그 심볼)

# 클린
make clean

# 플래시
make flash              # OpenOCD 사용
make flash-jlink        # J-Link 사용

# 크기 확인
make size
```

### 5.2 Keil MDK 빌드

**방법 1: 명령줄 (CI/자동화)**

```batch
cd my_chip

build_keil.bat build    # 증분 빌드
build_keil.bat rebuild  # 전체 재빌드
build_keil.bat clean    # 클린
build_keil.bat flash    # 플래시 다운로드
build_keil.bat debug    # 디버그 세션 시작
```

**방법 2: Keil uVision GUI**

1. `my_chip.uvprojx` 더블클릭 → Keil 열림
2. `Project → Build Target` (F7) → 빌드
3. `Flash → Download` (F8) → 플래시 다운로드
4. `Debug → Start/Stop Debug Session` (Ctrl+F5) → 디버그

**Keil 경로 설정** (build_keil.bat 수정):
```batch
set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
```

---

## 6. 커스터마이징 가이드

### 6.1 새 주변장치 추가

**1단계: chip_config.yaml에 주변장치 추가**
```yaml
peripherals:
  MY_PERIPH:
    base: 0x40030000
    description: "My Custom Peripheral"
```

**2단계: device_header_generator.py에 구조체 추가**
```python
# generators/device_header_generator.py

# 구조체 정의 추가
"""
typedef struct
{
    __IO uint32_t CR;     // Control register
    __IO uint32_t SR;     // Status register
    __IO uint32_t DR;     // Data register
} MY_PERIPH_TypeDef;
"""

# type_map에 추가
type_map = {
    ...
    'MY_PERIPH': 'MY_PERIPH_TypeDef',
}
```

**3단계: 인터럽트 추가 (필요시)**
```yaml
interrupts:
  50: [MY_PERIPH_IRQn, "My Peripheral Interrupt"]
```

### 6.2 클럭 설정 변경

**예: 외부 크리스털 사용으로 변경**
```yaml
clocks:
  hse:
    frequency: 12000000     # 12MHz 크리스털로 변경
    enabled: true           # HSE 활성화
    
  pll:
    source: "hse"           # PLL 소스를 HSE로
    multiplier: 6           # 12MHz × 6 = 72MHz
```

### 6.3 메모리 맵 변경

**예: 더 큰 Flash/SRAM**
```yaml
memory:
  flash:
    start: 0x08000000
    size: 512K              # 512KB로 증가
    
  sram:
    start: 0x20000000
    size: 128K              # 128KB로 증가
```

### 6.4 특화 설정 예시

```yaml
chip:
  name: "HA2024_AUDIO"
  vendor: "MyAudioCorp"
  core: "cortex-m3"

memory:
  flash:
    start: 0x08000000
    size: 256K
  sram:
    start: 0x20000000
    size: 64K
  # 오디오 버퍼용 추가 SRAM
  audio_ram:
    start: 0x20010000
    size: 16K

clocks:
  sysclk:
    frequency: 72000000     # DSP 처리를 위한 고속 클럭

peripherals:
  # 오디오 입력
  ADC1:
    base: 0x40012400
    description: "Microphone ADC"
    channels: 2
    resolution: 16          # 16-bit 오디오
    
  # 오디오 출력
  DAC:
    base: 0x40007400
    description: "Speaker DAC"
    channels: 2
    resolution: 16
    
  # 디지털 오디오 인터페이스
  I2S:
    base: 0x40013000
    description: "I2S for external codec"
    
  # 사용자 인터페이스
  GPIO_BTN:
    base: 0x40010800
    description: "Button inputs"
    
  # 무선 통신 (블루투스 등)
  WIRELESS:
    base: 0x40015000
    description: "Wireless module interface"

interrupts:
  # 오디오 관련 인터럽트
  43: [I2S_IRQn,       "I2S DMA complete"]
  44: [ADC_IRQn,       "ADC conversion complete"]
  45: [DAC_IRQn,       "DAC buffer empty"]
  46: [DSP_IRQn,       "DSP processing done"]
  47: [WIRELESS_IRQn,  "Wireless event"]
```

---

## 7. 프로젝트 구조

```
cortex_m3_generator/
├── firmware_generator.py         # 메인 스크립트
├── chip_config.yaml              # 설정 파일 예제
├── README.md                     # 이 문서
├── generators/                   # 생성기 모듈
│   ├── linker_generator.py           # GNU LD 링커 스크립트
│   ├── startup_generator.py          # GNU AS 스타트업
│   ├── system_generator.py           # system_xxx.c/h
│   ├── device_header_generator.py    # xxx.h (레지스터)
│   ├── interrupt_generator.py        # xxx_it.c
│   ├── makefile_generator.py         # Makefile
│   ├── main_generator.py             # main.c
│   ├── cmsis_generator.py            # core_cm3.h
│   ├── keil_scatter_generator.py     # .sct 스캐터 파일
│   ├── keil_startup_generator.py     # Keil 스타트업
│   ├── keil_project_generator.py     # .uvprojx 프로젝트
│   └── keil_build_generator.py       # 빌드 스크립트
└── output/                       # 생성 예제
    ├── ha2024.ld
    ├── ha2024.sct
    ├── ha2024.uvprojx
    ├── startup_ha2024.s
    ├── startup_ha2024_keil.s
    ├── system_ha2024.c/h
    ├── ha2024.h
    ├── ha2024_it.c
    ├── main.c
    ├── Makefile
    ├── build_keil.bat
    └── CMSIS/Include/core_cm3.h
```

---

## 라이선스

MIT License
