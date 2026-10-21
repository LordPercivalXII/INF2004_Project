@echo off
REM INF2004 Windows Build Commands
REM Batch file with common build, clean, and flash commands

setlocal enabledelayedexpansion

REM Set to your actual paths
set PICO_SDK_PATH=C:\pico\pico-sdk
set FREERTOS_KERNEL_PATH=C:\pico\FreeRTOS-Kernel

REM ============================================================================
REM HELP
REM ============================================================================

if "%1"=="" goto HELP
if "%1"=="help" goto HELP
if "%1"=="-h" goto HELP

goto PARSE_COMMAND

:HELP
echo.
echo INF2004 Windows Build System
echo.
echo Usage: build_commands.bat [COMMAND] [OPTIONS]
echo.
echo Commands:
echo   setup           Configure CMake and prepare build
echo   build           Compile the project (runs setup if needed)
echo   clean           Remove build directory
echo   rebuild         Clean and build from scratch
echo   flash           Flash robot_car.uf2 to Pico (auto-detect)
echo   configure-sim   Build with ROBOT_SIMULATION=ON (default)
echo   configure-hw    Build with ROBOT_SIMULATION=OFF for real hardware
echo   wifi-test       Build WiFi smoke test
echo   mqtt-test       Build MQTT smoke test
echo   all-targets     Build all targets (robot_car, wifi_test, mqtt_test)
echo   help            Show this help message
echo.
echo Examples:
echo   build_commands.bat setup
echo   build_commands.bat build
echo   build_commands.bat clean
echo   build_commands.bat rebuild
echo   build_commands.bat flash
echo   build_commands.bat configure-hw
echo.
exit /b 0

REM ============================================================================
REM ENVIRONMENT SETUP
REM ============================================================================

:PARSE_COMMAND

set COMMAND=%1
set BUILD_DIR=build

REM Export environment variables
set PICO_SDK_PATH=%PICO_SDK_PATH%
set FREERTOS_KERNEL_PATH=%FREERTOS_KERNEL_PATH%

echo.
echo ========================================
echo INF2004 Build System
echo ========================================
echo.

REM ============================================================================
REM CLEAN COMMAND
REM ============================================================================

if "%COMMAND%"=="clean" (
    echo Cleaning build directory...
    if exist %BUILD_DIR% (
        rmdir /s /q %BUILD_DIR%
        echo. [OK] Build directory removed
    ) else (
        echo. [INFO] Build directory does not exist
    )
    exit /b 0
)

REM ============================================================================
REM REBUILD COMMAND
REM ============================================================================

if "%COMMAND%"=="rebuild" (
    echo Performing clean rebuild...
    if exist %BUILD_DIR% (
        rmdir /s /q %BUILD_DIR%
        echo. [OK] Old build cleaned
    )
    mkdir %BUILD_DIR%
    echo. [INFO] Running CMake configuration...
    cd %BUILD_DIR%
    cmake -S .. -B . -G Ninja -DROBOT_SIMULATION=ON
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] CMake configuration failed
        exit /b 1
    )
    echo. [OK] CMake configuration complete
    echo. [INFO] Building project...
    cmake --build .
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Build failed
        exit /b 1
    )
    cd ..
    echo. [OK] Rebuild complete
    exit /b 0
)

REM ============================================================================
REM SETUP COMMAND
REM ============================================================================

if "%COMMAND%"=="setup" (
    echo. [INFO] Checking environment variables...
    if not defined PICO_SDK_PATH (
        echo. [ERROR] PICO_SDK_PATH not defined
        exit /b 1
    )
    if not defined FREERTOS_KERNEL_PATH (
        echo. [ERROR] FREERTOS_KERNEL_PATH not defined
        exit /b 1
    )
    echo. [OK] Environment variables set
    echo.   PICO_SDK_PATH=%PICO_SDK_PATH%
    echo.   FREERTOS_KERNEL_PATH=%FREERTOS_KERNEL_PATH%
    
    if not exist %BUILD_DIR% mkdir %BUILD_DIR%
    
    echo. [INFO] Running CMake configuration...
    cd %BUILD_DIR%
    cmake -S .. -B . -G Ninja -DROBOT_SIMULATION=ON
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] CMake configuration failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] Setup complete - ready to build
    exit /b 0
)

REM ============================================================================
REM BUILD COMMAND
REM ============================================================================

if "%COMMAND%"=="build" (
    if not exist %BUILD_DIR% (
        echo. [INFO] Build directory not found, running setup first...
        call %0 setup
        if !ERRORLEVEL! neq 0 exit /b 1
    )
    echo. [INFO] Building project...
    cd %BUILD_DIR%
    cmake --build .
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Build failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] Build complete
    
    REM Show output files
    echo.
    echo Build outputs:
    for /r %BUILD_DIR% %%f in (*.uf2) do echo.   - %%f
    for /r %BUILD_DIR% %%f in (*.hex) do echo.   - %%f
    exit /b 0
)

REM ============================================================================
REM CONFIGURE COMMANDS
REM ============================================================================

if "%COMMAND%"=="configure-sim" (
    echo. [INFO] Configuring for simulation mode (ROBOT_SIMULATION=ON)...
    if exist %BUILD_DIR% rmdir /s /q %BUILD_DIR%
    mkdir %BUILD_DIR%
    cd %BUILD_DIR%
    cmake -S .. -B . -G Ninja -DROBOT_SIMULATION=ON
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Configuration failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] Configured for simulation - run: build_commands.bat build
    exit /b 0
)

if "%COMMAND%"=="configure-hw" (
    echo. [INFO] Configuring for hardware mode (ROBOT_SIMULATION=OFF)...
    if exist %BUILD_DIR% rmdir /s /q %BUILD_DIR%
    mkdir %BUILD_DIR%
    cd %BUILD_DIR%
    cmake -S .. -B . -G Ninja -DROBOT_SIMULATION=OFF
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Configuration failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] Configured for hardware - run: build_commands.bat build
    exit /b 0
)

REM ============================================================================
REM INDIVIDUAL TARGET BUILDS
REM ============================================================================

if "%COMMAND%"=="wifi-test" (
    if not exist %BUILD_DIR% (
        call %0 setup
        if !ERRORLEVEL! neq 0 exit /b 1
    )
    echo. [INFO] Building WiFi smoke test...
    cd %BUILD_DIR%
    cmake --build . --target wifi_smoke_test
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Build failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] WiFi smoke test built
    echo.   Output: %BUILD_DIR%\wifi_smoke_test.uf2
    exit /b 0
)

if "%COMMAND%"=="mqtt-test" (
    if not exist %BUILD_DIR% (
        call %0 setup
        if !ERRORLEVEL! neq 0 exit /b 1
    )
    echo. [INFO] Building MQTT smoke test...
    cd %BUILD_DIR%
    cmake --build . --target mqtt_smoke_test
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Build failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] MQTT smoke test built
    echo.   Output: %BUILD_DIR%\mqtt_smoke_test.uf2
    exit /b 0
)

if "%COMMAND%"=="all-targets" (
    if not exist %BUILD_DIR% (
        call %0 setup
        if !ERRORLEVEL! neq 0 exit /b 1
    )
    echo. [INFO] Building all targets...
    cd %BUILD_DIR%
    cmake --build .
    if !ERRORLEVEL! neq 0 (
        echo. [ERROR] Build failed
        cd ..
        exit /b 1
    )
    cd ..
    echo. [OK] All targets built
    exit /b 0
)

REM ============================================================================
REM FLASH COMMAND
REM ============================================================================

if "%COMMAND%"=="flash" (
    echo. [INFO] Launching flash utility...
    echo.
    if exist flash_pico.ps1 (
        powershell -NoProfile -ExecutionPolicy Bypass -File flash_pico.ps1
        exit /b !ERRORLEVEL!
    ) else (
        echo. [ERROR] flash_pico.ps1 not found
        exit /b 1
    )
)

REM ============================================================================
REM UNKNOWN COMMAND
REM ============================================================================

echo. [ERROR] Unknown command: %COMMAND%
echo. Run 'build_commands.bat help' for usage information
exit /b 1
