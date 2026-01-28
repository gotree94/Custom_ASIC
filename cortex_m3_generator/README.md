# Cortex-M3 Firmware Generator

커스텀 Cortex-M3 기반 칩(보청기 칩 등)을 위한 펌웨어 기본 파일 생성기입니다.

## 개요

YAML 설정 파일을 기반으로 **GNU GCC**와 **Keil MDK** 양쪽 툴체인용 파일을 자동 생성합니다.

### 생성 파일

| 파일 | GNU GCC | Keil MDK | 설명 |
|------|:-------:|:--------:|------|
| `system_<chip>.c/h` | ✓ | ✓ | 시스템 초기화 (클럭, PLL) |
| `<chip>.h` | ✓ | ✓ | 레지스터 맵, 비트 정의 |
| `<chip>_it.c` | ✓ | ✓ | 인터럽트 핸들러 템플릿 |
| `main.c` | ✓ | ✓ | 메인 함수 예제 |
| `startup_<chip>.s` | ✓ | - | GNU AS 문법 스타트업 |
| `<chip>.ld` | ✓ | - | GNU LD 링커 스크립트 |
| `Makefile` | ✓ | - | GNU Make 빌드 |
| `startup_<chip>_keil.s` | - | ✓ | ARM 어셈블러 문법 스타트업 |
| `<chip>.sct` | - | ✓ | 스캐터 파일 (Keil 링커용) |
| `<chip>.uvprojx` | - | ✓ | Keil 프로젝트 파일 |
| `build_keil.bat` | - | ✓ | 명령줄 빌드 스크립트 |

## 사용법

### 1. 설정 파일 작성

`chip_config.yaml`을 수정하여 칩 사양에 맞게 설정:

```yaml
chip:
  name: "MY_CHIP"
  core: "cortex-m3"

memory:
  flash:
    start: 0x08000000
    size: 256K
  sram:
    start: 0x20000000
    size: 64K
```

### 2. 파일 생성

```bash
# 모든 툴체인 (기본값)
python firmware_generator.py chip_config.yaml -o ./my_chip

# GNU GCC 전용
python firmware_generator.py chip_config.yaml -o ./my_chip --gcc

# Keil MDK 전용
python firmware_generator.py chip_config.yaml -o ./my_chip --keil
```

### 3-A. GNU GCC 빌드

```bash
cd my_chip
make              # Release 빌드
make DEBUG=1      # Debug 빌드
make flash        # OpenOCD로 플래시
```

### 3-B. Keil MDK 빌드

**방법 1: 명령줄 (CI/자동화용)**
```batch
cd my_chip
build_keil.bat build    # 빌드
build_keil.bat rebuild  # 전체 재빌드
build_keil.bat flash    # 플래시 다운로드
build_keil.bat clean    # 클린
```

**방법 2: Keil uVision GUI**
1. `my_chip.uvprojx` 파일을 더블클릭하여 Keil 열기
2. `Project → Build Target` (F7)
3. `Flash → Download` (F8)

## Keil MDK 설정

### Keil 경로 설정 (build_keil.bat)
```batch
set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe
```
Keil 설치 경로에 맞게 수정하세요.

### 디버거 설정
프로젝트 파일에서 기본으로 ULINK2/ULINKpro가 설정되어 있습니다.
ST-Link, J-Link 등 다른 디버거 사용 시 Keil에서 설정을 변경하세요.

## 설정 파일 주요 항목

| 섹션 | 설명 |
|------|------|
| `chip` | 칩 이름, 코어 종류, NVIC 설정 |
| `memory` | Flash/SRAM 시작 주소 및 크기 |
| `stack/heap` | 스택/힙 크기 |
| `clocks` | HSI/HSE/PLL/버스 클럭 설정 |
| `peripherals` | 주변장치 베이스 주소 |
| `interrupts` | 인터럽트 벡터 정의 |

## 요구사항

**생성기 실행:**
- Python 3.7+
- PyYAML (`pip install pyyaml`)

**GNU GCC 빌드:**
- arm-none-eabi-gcc

**Keil MDK 빌드:**
- Keil MDK-ARM (uVision 5)
- ARMCC 또는 ARM Compiler 6

## 프로젝트 구조

```
cortex_m3_generator/
├── firmware_generator.py    # 메인 스크립트
├── chip_config.yaml         # 설정 파일 예제
├── generators/              # 생성기 모듈
│   ├── linker_generator.py      # GNU LD
│   ├── startup_generator.py     # GNU AS
│   ├── system_generator.py      # 시스템 초기화
│   ├── device_header_generator.py
│   ├── interrupt_generator.py
│   ├── makefile_generator.py
│   ├── main_generator.py
│   ├── cmsis_generator.py
│   ├── keil_scatter_generator.py   # Keil 스캐터
│   ├── keil_startup_generator.py   # Keil 스타트업
│   ├── keil_project_generator.py   # .uvprojx
│   └── keil_build_generator.py     # 빌드 스크립트
└── output/                  # 생성 예제
```

## 라이선스

MIT License
