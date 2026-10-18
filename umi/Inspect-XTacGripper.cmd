@echo off
setlocal
set "XTAC_INSPECT_DIR=%~dp0"
echo XTac read-only inspection: version, serial number, device side.
echo Close other programs using COM11 before running this check.
echo No firmware update, reset, calibration or motor control is performed.
powershell.exe -NoProfile -Command "$ErrorActionPreference='Stop'; try { & ([scriptblock]::Create((Get-Content -Raw -LiteralPath (Join-Path $env:XTAC_INSPECT_DIR 'Inspect-XTacGripper.ps1')))) } catch { Write-Host $_.Exception.Message -ForegroundColor Red; exit 1 }"
echo.
pause
endlocal
