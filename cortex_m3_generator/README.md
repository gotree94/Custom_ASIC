# Cortex-M3 Firmware Generator

커스텀 Cortex-M3 기반 칩(보청기 칩 등)을 위한 펌웨어 기본 파일 생성기입니다.

## 개요

YAML 설정 파일을 기반으로 다음 파일들을 자동 생성합니다:

| 파일 | 설명 |
|------|------|
| `startup_<chip>.s` | 스타트업 어셈블리 코드 (벡터 테이블, 리셋 핸들러) |
| `<chip>.ld` | 링커 스크립트 (메모리 맵, 섹션 배치) |
| `system_<chip>.c` | 시스템 초기화 (클럭 설정, PLL 구성) |
| `system_<chip>.h` | 시스템 헤더 |
| `<chip>.h` | 디바이스 헤더 (레지스터 맵, 비트 정의) |
| `<chip>_it.c` | 인터럽트 핸들러 템플릿 |
| `Makefile` | 빌드 스크립트 |
| `main.c` | 메인 함수 예제 |
| `CMSIS/Include/core_cm3.h` | CMSIS 코어 헤더 스텁 |

## 사용법

### 1. 설정 파일 작성

`chip_config.yaml`을 수정하여 칩 사양에 맞게 설정합니다:

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

clocks:
  sysclk:
    frequency: 72000000

peripherals:
  GPIOA:
    base: 0x40010800
    description: "GPIO Port A"
```

### 2. 파일 생성

```bash
python firmware_generator.py chip_config.yaml -o ./my_output
```

### 3. 빌드

```bash
cd my_output
make          # Release 빌드
make DEBUG=1  # Debug 빌드
```

### 4. 플래시

```bash
make flash          # OpenOCD 사용
make flash-jlink    # J-Link 사용
```

## 설정 파일 주요 항목

| 섹션 | 설명 |
|------|------|
| `chip` | 칩 이름, 코어 종류, NVIC 설정 |
| `memory` | Flash/SRAM 시작 주소 및 크기 |
| `stack/heap` | 스택/힙 크기 |
| `clocks` | HSI/HSE/PLL/버스 클럭 설정 |
| `peripherals` | 주변장치 베이스 주소 |
| `interrupts` | 인터럽트 벡터 정의 |
| `boot` | 부트 옵션 |

## 요구사항

- Python 3.7+
- PyYAML (`pip install pyyaml`)
- arm-none-eabi-gcc (빌드용)

## 프로젝트 구조

```
cortex_m3_generator/
├── firmware_generator.py    # 메인 스크립트
├── chip_config.yaml         # 설정 파일 예제
├── generators/              # 생성기 모듈
│   ├── linker_generator.py
│   ├── startup_generator.py
│   ├── system_generator.py
│   ├── device_header_generator.py
│   ├── interrupt_generator.py
│   ├── makefile_generator.py
│   ├── main_generator.py
│   └── cmsis_generator.py
└── output/                  # 생성된 파일 (예제)
```

## 커스터마이징

### 새 주변장치 추가
1. `chip_config.yaml`의 `peripherals` 섹션에 추가
2. `device_header_generator.py`에 레지스터 구조체 정의 추가
3. 필요시 `interrupt_generator.py`에 핸들러 템플릿 추가

### 메모리 맵 변경
`chip_config.yaml`의 `memory` 섹션 수정

## 라이선스

MIT License
