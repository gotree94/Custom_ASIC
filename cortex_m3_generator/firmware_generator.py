#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cortex-M3 Firmware Generator
=============================
커스텀 Cortex-M3 기반 칩을 위한 펌웨어 기본 파일 생성기

생성 파일 (GNU GCC):
  - startup_<chip>.s    : 스타트업 어셈블리 코드
  - <chip>.ld           : 링커 스크립트
  - system_<chip>.c     : 시스템 초기화 코드
  - system_<chip>.h     : 시스템 헤더
  - <chip>.h            : 디바이스 헤더 (레지스터 맵)
  - <chip>_it.c         : 인터럽트 핸들러 템플릿
  - Makefile            : 빌드 스크립트

생성 파일 (Keil MDK):
  - startup_<chip>.s    : Keil용 스타트업 (ARM 어셈블러)
  - <chip>.sct          : 스캐터 파일
  - <chip>.uvprojx      : Keil 프로젝트 파일
  - build_keil.bat      : 빌드 스크립트

사용법:
  python firmware_generator.py chip_config.yaml -o output_dir
  python firmware_generator.py chip_config.yaml -o output_dir --keil
  python firmware_generator.py chip_config.yaml -o output_dir --all
"""

import yaml
import argparse
import os
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, List, Tuple

# GNU GCC 생성기 모듈 임포트
from generators.linker_generator import generate_linker_script
from generators.startup_generator import generate_startup
from generators.system_generator import generate_system_c, generate_system_h
from generators.device_header_generator import generate_device_header
from generators.interrupt_generator import generate_interrupt_handlers
from generators.makefile_generator import generate_makefile
from generators.main_generator import generate_main
from generators.cmsis_generator import generate_cmsis_stub

# Keil MDK 생성기 모듈 임포트
from generators.keil_scatter_generator import generate_scatter_file
from generators.keil_startup_generator import generate_keil_startup
from generators.keil_project_generator import generate_keil_project
from generators.keil_build_generator import generate_keil_build_script, generate_keil_debug_ini


class FirmwareGenerator:
    """Cortex-M3 펌웨어 파일 생성기"""
    
    def __init__(self, config_path: str):
        """설정 파일 로드"""
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self.chip_name = self.config['chip']['name']
        self.chip_name_upper = self.chip_name.upper()
        self.chip_name_lower = self.chip_name.lower()
        
    def parse_size(self, size_str) -> int:
        """크기 문자열 파싱 (예: '256K' -> 262144)"""
        if isinstance(size_str, int):
            return size_str
        
        size_str = str(size_str).upper().strip()
        multipliers = {'K': 1024, 'M': 1024*1024, 'G': 1024*1024*1024}
        
        for suffix, mult in multipliers.items():
            if size_str.endswith(suffix):
                return int(size_str[:-1]) * mult
        
        return int(size_str, 0)  # hex 지원
    
    def get_timestamp(self) -> str:
        """현재 타임스탬프"""
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    def generate_all(self, output_dir: str, toolchain: str = 'all'):
        """모든 파일 생성
        
        Args:
            output_dir: 출력 디렉토리
            toolchain: 'gcc', 'keil', 또는 'all'
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # CMSIS 디렉토리
        cmsis_path = output_path / "CMSIS" / "Include"
        cmsis_path.mkdir(parents=True, exist_ok=True)
        
        generated_files = []
        
        # 공통 파일 (양쪽 모두 사용)
        common_files = [
            (f"system_{self.chip_name_lower}.c", generate_system_c(self)),
            (f"system_{self.chip_name_lower}.h", generate_system_h(self)),
            (f"{self.chip_name_lower}.h", generate_device_header(self)),
            (f"{self.chip_name_lower}_it.c", generate_interrupt_handlers(self)),
            ("main.c", generate_main(self)),
        ]
        
        # GNU GCC 전용 파일
        gcc_files = [
            (f"{self.chip_name_lower}.ld", generate_linker_script(self)),
            (f"startup_{self.chip_name_lower}.s", generate_startup(self)),
            ("Makefile", generate_makefile(self)),
        ]
        
        # Keil MDK 전용 파일
        keil_files = [
            (f"{self.chip_name_lower}.sct", generate_scatter_file(self)),
            (f"startup_{self.chip_name_lower}_keil.s", generate_keil_startup(self)),
            (f"{self.chip_name_lower}.uvprojx", generate_keil_project(self)),
            ("build_keil.bat", generate_keil_build_script(self)),
            ("debug.ini", generate_keil_debug_ini(self)),
        ]
        
        # 공통 파일 생성
        print("  [Common Files]")
        for filename, content in common_files:
            filepath = output_path / filename
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            generated_files.append(filepath)
            print(f"    ✓ {filename}")
        
        # GNU GCC 파일 생성
        if toolchain in ('gcc', 'all'):
            print("\n  [GNU GCC Files]")
            for filename, content in gcc_files:
                filepath = output_path / filename
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(content)
                generated_files.append(filepath)
                print(f"    ✓ {filename}")
        
        # Keil MDK 파일 생성
        if toolchain in ('keil', 'all'):
            print("\n  [Keil MDK Files]")
            for filename, content in keil_files:
                filepath = output_path / filename
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(content)
                generated_files.append(filepath)
                print(f"    ✓ {filename}")
        
        # CMSIS 스텁
        cmsis_file = cmsis_path / "core_cm3.h"
        with open(cmsis_file, 'w', encoding='utf-8') as f:
            f.write(generate_cmsis_stub(self))
        generated_files.append(cmsis_file)
        print(f"\n    ✓ CMSIS/Include/core_cm3.h")
        
        return generated_files


def main():
    """CLI 엔트리 포인트"""
    parser = argparse.ArgumentParser(
        description='Cortex-M3 펌웨어 파일 생성기',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
예제:
  python firmware_generator.py chip_config.yaml -o ./output
  python firmware_generator.py chip_config.yaml -o ./output --gcc
  python firmware_generator.py chip_config.yaml -o ./output --keil
  python firmware_generator.py chip_config.yaml -o ./output --all
        '''
    )
    
    parser.add_argument('config', help='칩 설정 YAML 파일')
    parser.add_argument('-o', '--output', default='./output',
                        help='출력 디렉토리 (기본값: ./output)')
    
    # 툴체인 선택 옵션
    toolchain_group = parser.add_mutually_exclusive_group()
    toolchain_group.add_argument('--gcc', action='store_true',
                                  help='GNU GCC 툴체인용 파일만 생성')
    toolchain_group.add_argument('--keil', action='store_true',
                                  help='Keil MDK 툴체인용 파일만 생성')
    toolchain_group.add_argument('--all', action='store_true', default=True,
                                  help='모든 툴체인용 파일 생성 (기본값)')
    
    args = parser.parse_args()
    
    # 툴체인 결정
    if args.gcc:
        toolchain = 'gcc'
    elif args.keil:
        toolchain = 'keil'
    else:
        toolchain = 'all'
    
    if not os.path.exists(args.config):
        print(f"오류: 설정 파일을 찾을 수 없습니다: {args.config}")
        return 1
    
    print(f"\n{'='*60}")
    print(f"Cortex-M3 Firmware Generator")
    print(f"{'='*60}")
    print(f"설정 파일: {args.config}")
    print(f"출력 디렉토리: {args.output}")
    print(f"툴체인: {toolchain.upper()}")
    print(f"{'='*60}\n")
    
    try:
        generator = FirmwareGenerator(args.config)
        print(f"칩 이름: {generator.chip_name}")
        print(f"\n생성 중...\n")
        
        files = generator.generate_all(args.output, toolchain)
        
        print(f"\n{'='*60}")
        print(f"완료! {len(files)}개 파일 생성됨")
        print(f"{'='*60}")
        
        # 빌드 안내
        if toolchain in ('gcc', 'all'):
            print(f"\n[GNU GCC 빌드]")
            print(f"  cd {args.output}")
            print(f"  make")
        
        if toolchain in ('keil', 'all'):
            print(f"\n[Keil MDK 빌드]")
            print(f"  cd {args.output}")
            print(f"  build_keil.bat build")
            print(f"  또는 Keil uVision에서 {generator.chip_name_lower}.uvprojx 열기")
        
        print()
        
    except Exception as e:
        print(f"오류: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
