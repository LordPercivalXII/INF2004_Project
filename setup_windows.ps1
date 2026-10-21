# INF2004 Windows Build Setup Script
# Simple, clean build script

Write-Host ""
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "INF2004 BUILD SYSTEM" -ForegroundColor Cyan
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host ""

# Configuration
$PICO_SDK_PATH = "C:\pico\pico-sdk"
$FREERTOS_KERNEL_PATH = "C:\pico\FreeRTOS-Kernel"
$BUILD_DIR = "build"

# Set environment variables
$env:PICO_SDK_PATH = $PICO_SDK_PATH
$env:FREERTOS_KERNEL_PATH = $FREERTOS_KERNEL_PATH

Write-Host "Checking paths..." -ForegroundColor Yellow
if (-not (Test-Path $PICO_SDK_PATH)) {
    Write-Host "❌ Pico SDK not found at $PICO_SDK_PATH" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path $FREERTOS_KERNEL_PATH)) {
    Write-Host "❌ FreeRTOS-Kernel not found at $FREERTOS_KERNEL_PATH" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Paths OK" -ForegroundColor Green
Write-Host ""

# Create build directory
if (-not (Test-Path $BUILD_DIR)) {
    New-Item -ItemType Directory -Path $BUILD_DIR | Out-Null
}

Write-Host "Running CMake configuration..." -ForegroundColor Yellow
cmake -S . -B $BUILD_DIR -G Ninja -DROBOT_SIMULATION=ON
if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ CMake configuration failed" -ForegroundColor Red
    exit 1
}
Write-Host "✅ CMake configuration done" -ForegroundColor Green
Write-Host ""

Write-Host "Building project..." -ForegroundColor Yellow
cmake --build $BUILD_DIR
if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Build failed" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Build successful!" -ForegroundColor Green
Write-Host ""
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host "✅ BUILD COMPLETE!" -ForegroundColor Green
Write-Host "═══════════════════════════════════════════════════════════════" -ForegroundColor Cyan
Write-Host ""
Write-Host "Output files:" -ForegroundColor Yellow
Get-ChildItem -Path $BUILD_DIR -Filter "*.uf2" -Recurse | ForEach-Object {
    Write-Host "  - $($_.Name)" -ForegroundColor Cyan
}
Write-Host ""
Write-Host "Next: Run .\flash_pico.ps1 to flash to Pico" -ForegroundColor Green
Write-Host ""
