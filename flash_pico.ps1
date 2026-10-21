# Raspberry Pi Pico UF2 Flash Script
# Automatically detects the Pico drive and flashes the firmware

# ============================================================================
# CONFIGURATION
# ============================================================================

$BUILD_DIR = "build"
$UF2_FILENAME = "robot_car.uf2"
$TIMEOUT_SECONDS = 30

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

function Print-Header {
    param([string]$Message)
    Write-Host ""
    Write-Host "═" * 70
    Write-Host $Message -ForegroundColor Cyan
    Write-Host "═" * 70
    Write-Host ""
}

# ============================================================================
# LOCATE UF2 FILE
# ============================================================================

Print-Header "LOCATING UF2 FILE"

$uf2Files = Get-ChildItem -Path $BUILD_DIR -Filter "*.uf2" -Recurse -ErrorAction SilentlyContinue

if (-not $uf2Files) {
    Write-Host "❌ No .uf2 files found in $BUILD_DIR" -ForegroundColor Red
    Write-Host "   Run .\setup_windows.ps1 first to build the project" -ForegroundColor Yellow
    exit 1
}

# Find the robot_car.uf2 file preferentially
$targetUf2 = $null
foreach ($file in $uf2Files) {
    if ($file.Name -eq $UF2_FILENAME) {
        $targetUf2 = $file
        break
    }
}

if (-not $targetUf2) {
    $targetUf2 = $uf2Files[0]
}

Write-Host "✅ Found: $($targetUf2.FullName)" -ForegroundColor Green

# ============================================================================
# LOCATE PICO DRIVE
# ============================================================================

Print-Header "SEARCHING FOR RASPBERRY PI PICO DRIVE"

Write-Host "Waiting for Pico in BOOTSEL mode (max $TIMEOUT_SECONDS seconds)..." -ForegroundColor Yellow
Write-Host "If your Pico is not in BOOTSEL mode:" -ForegroundColor Yellow
Write-Host "1. Disconnect the Pico from USB" -ForegroundColor White
Write-Host "2. Hold the BOOTSEL button (small button on the board)" -ForegroundColor White
Write-Host "3. While holding BOOTSEL, reconnect the USB cable" -ForegroundColor White
Write-Host "4. Release BOOTSEL" -ForegroundColor White
Write-Host ""

$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
$picoDrive = $null

while ($stopwatch.Elapsed.TotalSeconds -lt $TIMEOUT_SECONDS) {
    # Look for a drive with the name "RPI-RP2" (standard Pico BOOTSEL name)
    $drives = Get-PSDrive -PSProvider FileSystem | Where-Object { $_.Name -match "^[A-Z]$" }
    
    foreach ($drive in $drives) {
        $drivePath = "$($drive.Name):"
        # Check if the drive has the typical Pico BOOTSEL indicator
        $volumeName = (Get-Volume -DriveLetter $drive.Name -ErrorAction SilentlyContinue).FileSystemLabel
        
        if ($volumeName -eq "RPI-RP2" -or $volumeName -eq "PICO" -or (Test-Path "$drivePath\INFO_UF2.TXT")) {
            $picoDrive = $drivePath
            Write-Host "✅ Found Pico drive at: $picoDrive" -ForegroundColor Green
            break
        }
    }
    
    if ($picoDrive) { break }
    
    # Show progress
    $elapsed = [math]::Floor($stopwatch.Elapsed.TotalSeconds)
    Write-Host "   Searching... ($elapsed/$TIMEOUT_SECONDS seconds)" -ForegroundColor Gray
    Start-Sleep -Milliseconds 500
}

if (-not $picoDrive) {
    Write-Host ""
    Write-Host "❌ PICO NOT FOUND" -ForegroundColor Red
    Write-Host ""
    Write-Host "Troubleshooting:" -ForegroundColor Yellow
    Write-Host "1. Check that your Pico is connected to USB" -ForegroundColor White
    Write-Host "2. Hold BOOTSEL button and reconnect USB to enter BOOTSEL mode" -ForegroundColor White
    Write-Host "3. On Windows, look for a new drive appearing in File Explorer" -ForegroundColor White
    Write-Host "4. Run this script again" -ForegroundColor White
    exit 1
}

# ============================================================================
# FLASH FIRMWARE
# ============================================================================

Print-Header "FLASHING FIRMWARE"

Write-Host "Source file: $($targetUf2.FullName)" -ForegroundColor Cyan
Write-Host "Destination: $picoDrive" -ForegroundColor Cyan
Write-Host ""

try {
    Copy-Item -Path $targetUf2.FullName -Destination "$picoDrive\" -Force
    Write-Host "✅ Successfully copied $($targetUf2.Name) to Pico" -ForegroundColor Green
}
catch {
    Write-Host "❌ FAILED TO COPY FILE" -ForegroundColor Red
    Write-Host "Error: $_" -ForegroundColor Red
    exit 1
}

# ============================================================================
# VERIFICATION
# ============================================================================

Print-Header "VERIFICATION"

Start-Sleep -Seconds 2

# After flashing, the Pico should disconnect and reboot
Write-Host "Waiting for Pico to reboot and disconnect..." -ForegroundColor Yellow

$stopwatch.Restart()
while ($stopwatch.Elapsed.TotalSeconds -lt 5) {
    if (-not (Test-Path "$picoDrive\INFO_UF2.TXT" -ErrorAction SilentlyContinue)) {
        Write-Host "✅ Pico disconnected and rebooted successfully" -ForegroundColor Green
        Write-Host ""
        Write-Host "The firmware is now running on your Raspberry Pi Pico!" -ForegroundColor Green
        exit 0
    }
    Start-Sleep -Milliseconds 500
}

Write-Host ""
Write-Host "⚠️  Pico still connected. Manually disconnect and reconnect USB to start the firmware." -ForegroundColor Yellow
