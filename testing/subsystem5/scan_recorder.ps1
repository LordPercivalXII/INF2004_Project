# Subsystem 5 scan recorder: reads the Pico's console from a COM port, saves
# it to logs\, and serves a live page at http://localhost:8765/.
#
# Run from the project folder, with any serial monitor closed:
#   powershell -ExecutionPolicy Bypass -File testing\subsystem5\scan_recorder.ps1
# Stop with Ctrl+C.

param(
    [string] $Port = "COM5",
    [int] $HttpPort = 8765,
    [int] $Seconds = 0,         # 0 = run until Ctrl+C
    [switch] $NoBrowser
)

$page = [IO.File]::ReadAllBytes((Join-Path $PSScriptRoot "scan_recorder.html"))
$logDir = Join-Path $PSScriptRoot "logs"
$logPath = Join-Path $logDir ((Get-Date -Format "yyyy-MM-dd_HHmm") + "_scan_recording.txt")
$utf8 = New-Object Text.UTF8Encoding($false)
$url = "http://localhost:$HttpPort/"

$listener = New-Object Net.HttpListener
$listener.Prefixes.Add($url)
try {
    $listener.Start()
}
catch {
    Write-Host "Could not start: the recorder is probably already running. Stop it with Ctrl+C first."
    exit 1
}

$serial = New-Object IO.Ports.SerialPort $Port, 115200
$serial.DtrEnable = $true
$clients = New-Object Collections.ArrayList
$pending = $listener.GetContextAsync()
$buffer = ""
$lastOpenTry = [DateTime]::MinValue
$dtrRestoreAt = [DateTime]::MinValue
$end = if ($Seconds -gt 0) { (Get-Date).AddSeconds($Seconds) } else { [DateTime]::MaxValue }

function Send-Line([string] $line) {
    $bytes = $utf8.GetBytes("data: $line`n`n")
    foreach ($client in @($clients)) {
        try {
            $client.OutputStream.Write($bytes, 0, $bytes.Length)
            $client.OutputStream.Flush()
        }
        catch {
            [void] $clients.Remove($client)
        }
    }
}

Write-Host "Live page: $url"
Write-Host "Saving to: $logPath"
Write-Host "Press Ctrl+C to stop."
if (-not $NoBrowser) { Start-Process $url }

try {
    while ((Get-Date) -lt $end) {
        if ($pending.IsCompleted) {
            $context = $pending.Result
            $pending = $listener.GetContextAsync()

            if ($context.Request.Url.AbsolutePath -eq "/servo-test") {
                # A DTR pulse asks the bench firmware to run its servo test.
                if ($serial.IsOpen) {
                    $serial.DtrEnable = $false
                    $dtrRestoreAt = (Get-Date).AddMilliseconds(400)
                    $context.Response.StatusCode = 204
                }
                else {
                    $context.Response.StatusCode = 503
                }
                $context.Response.Close()
            }
            elseif ($context.Request.Url.AbsolutePath -eq "/events") {
                $context.Response.ContentType = "text/event-stream"
                $context.Response.Headers.Add("Cache-Control", "no-cache")
                $context.Response.SendChunked = $true
                [void] $clients.Add($context.Response)
            }
            else {
                $context.Response.ContentType = "text/html; charset=utf-8"
                $context.Response.ContentLength64 = $page.Length
                $context.Response.OutputStream.Write($page, 0, $page.Length)
                $context.Response.Close()
            }
        }

        try {
            if ($serial.IsOpen -and ((Get-Date) -ge $dtrRestoreAt)) {
                if (-not $serial.DtrEnable) { $serial.DtrEnable = $true }
            }

            if (-not $serial.IsOpen) {
                if (((Get-Date) - $lastOpenTry).TotalSeconds -ge 2) {
                    $lastOpenTry = Get-Date
                    $serial.Open()
                    Write-Host "Connected to $Port"
                }
            }
            elseif ($serial.BytesToRead -gt 0) {
                $buffer += $serial.ReadExisting()

                while (($newline = $buffer.IndexOf("`n")) -ge 0) {
                    $line = $buffer.Substring(0, $newline).Trim()
                    $buffer = $buffer.Substring($newline + 1)

                    if ($line) {
                        if (-not (Test-Path $logDir)) {
                            [void] (New-Item -ItemType Directory $logDir)
                        }
                        [IO.File]::AppendAllText($logPath, "$line`r`n")
                        Send-Line $line
                    }
                }
            }
        }
        catch {
            Write-Host "Waiting for $Port ($($_.Exception.Message))"
            try { $serial.Close() } catch { }
        }

        Start-Sleep -Milliseconds 20
    }
}
finally {
    foreach ($client in @($clients)) { try { $client.Close() } catch { } }
    $listener.Stop()
    if ($serial.IsOpen) { $serial.Close() }
    Write-Host "Stopped. Log: $logPath"
}
