@echo off
REM ============================================================================
REM Keil MDK-ARM Build Script for HA2024
REM Generated: 2026-01-28 13:26:30
REM ============================================================================

REM Keil UV4/UV5 설치 경로 설정 (필요시 수정)
set KEIL_PATH=C:\Keil_v5\UV4\UV4.exe

REM 프로젝트 파일
set PROJECT=ha2024.uvprojx

REM 타겟 이름
set TARGET=HA2024

REM 빌드 로그 파일
set LOG_FILE=build\build.log

REM ============================================================================
REM 빌드 디렉토리 생성
REM ============================================================================
if not exist "build" mkdir build

REM ============================================================================
REM 빌드 옵션
REM ============================================================================
REM -b : 빌드 (배치 모드)
REM -r : 리빌드 (전체 재컴파일)
REM -c : 클린
REM -f : 플래시 다운로드
REM -d : 디버그 세션 시작

if "%1"=="" goto build
if "%1"=="build" goto build
if "%1"=="rebuild" goto rebuild
if "%1"=="clean" goto clean
if "%1"=="flash" goto flash
if "%1"=="debug" goto debug
goto help

:build
echo.
echo ============================================================
echo  Building HA2024 (Incremental)
echo ============================================================
echo.
"%KEIL_PATH%" -b "%PROJECT%" -t "%TARGET%" -o "%LOG_FILE%"
if errorlevel 1 (
    echo.
    echo [ERROR] Build failed! Check %LOG_FILE% for details.
    type "%LOG_FILE%"
    exit /b 1
)
echo.
echo [SUCCESS] Build completed!
echo Output: build\ha2024_firmware.axf
echo         build\ha2024_firmware.hex
echo         build\ha2024_firmware.bin
goto end

:rebuild
echo.
echo ============================================================
echo  Rebuilding HA2024 (Full)
echo ============================================================
echo.
"%KEIL_PATH%" -r "%PROJECT%" -t "%TARGET%" -o "%LOG_FILE%"
if errorlevel 1 (
    echo [ERROR] Rebuild failed!
    type "%LOG_FILE%"
    exit /b 1
)
echo [SUCCESS] Rebuild completed!
goto end

:clean
echo.
echo ============================================================
echo  Cleaning HA2024
echo ============================================================
echo.
"%KEIL_PATH%" -c "%PROJECT%" -t "%TARGET%" -o "%LOG_FILE%"
if exist "build" rd /s /q build
echo [SUCCESS] Clean completed!
goto end

:flash
echo.
echo ============================================================
echo  Flashing HA2024
echo ============================================================
echo.
"%KEIL_PATH%" -f "%PROJECT%" -t "%TARGET%" -o "%LOG_FILE%"
if errorlevel 1 (
    echo [ERROR] Flash failed!
    exit /b 1
)
echo [SUCCESS] Flash completed!
goto end

:debug
echo.
echo ============================================================
echo  Starting Debug Session
echo ============================================================
echo.
"%KEIL_PATH%" -d "%PROJECT%" -t "%TARGET%"
goto end

:help
echo.
echo Usage: build_keil.bat [command]
echo.
echo Commands:
echo   build   - Incremental build (default)
echo   rebuild - Full rebuild
echo   clean   - Clean build files
echo   flash   - Download to target
echo   debug   - Start debug session
echo.
goto end

:end
