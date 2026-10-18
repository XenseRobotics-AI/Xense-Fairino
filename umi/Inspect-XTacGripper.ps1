# Read-only XTac/TacCap MCU inspection for Windows PowerShell 5.1+.
# Protocol source: XenseRobotics-AI/TacCap-Gripper, cpp/src/bus/frame.cpp
# and cpp/include/taccap/protocol/commands.hpp (official SDK).
# Sends only GetVersion (02), GetSn (04), and GetDevType (06).
# No scan/StopStream, motor, calibration, reset, or firmware-update commands.
[CmdletBinding()]
param(
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Port = 'COM11',
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'

function Get-Crc16([byte[]]$Bytes) {
    [int]$crc = 0xFFFF
    foreach ($b in $Bytes) {
        $crc = $crc -bxor [int]$b
        for ($bit = 0; $bit -lt 8; $bit++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 0xA001 }
            else { $crc = $crc -shr 1 }
        }
    }
    return $crc
}

function New-ReadFrame([byte]$Sequence, [byte]$Command) {
    if ($Command -notin @(0x02, 0x04, 0x06)) {
        throw 'Only GetVersion, GetSn and GetDevType are allowed.'
    }
    [byte[]]$plain = @(0xAA, 0x02, $Sequence, 0x00, $Command, 0x00, 0x00)
    $crc = Get-Crc16 $plain
    $plain += [byte]($crc -band 0xFF), [byte](($crc -shr 8) -band 0xFF)
    $wire = New-Object 'System.Collections.Generic.List[byte]'
    $wire.Add(0xAA)
    for ($i = 1; $i -lt $plain.Length; $i++) {
        if ($plain[$i] -in @(0xAA, 0x55, 0x7D)) {
            $wire.Add(0x7D)
            $wire.Add([byte]($plain[$i] -bxor 0x20))
        } else { $wire.Add($plain[$i]) }
    }
    $wire.Add(0x55)
    return ,$wire.ToArray()
}

function Read-Reply($Serial, [byte]$Sequence, [byte]$Command) {
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $body = New-Object 'System.Collections.Generic.List[byte]'
    $inside = $false
    $escaped = $false
    while ($timer.ElapsedMilliseconds -lt 1500) {
        try { $b = $Serial.ReadByte() }
        catch [System.TimeoutException] { continue }
        if ($b -eq 0xAA) {
            $body.Clear(); $inside = $true; $escaped = $false
            continue
        }
        if (-not $inside) { continue }
        if ($b -eq 0x55) {
            $inside = $false
            if ($escaped -or $body.Count -lt 8) { continue }
            $length = [int]$body[4] -bor ([int]$body[5] -shl 8)
            if ($body.Count -ne ($length + 8)) { continue }
            [byte[]]$check = @(0xAA) + $body.GetRange(0, 6 + $length).ToArray()
            $expected = [int]$body[6 + $length] -bor ([int]$body[7 + $length] -shl 8)
            if ((Get-Crc16 $check) -ne $expected) { continue }
            # Firmware replies use the MCU address (02/03); some hosts use PC (01).
            # Match sequence, ACK type and command as the official SDK does.
            if ($body[0] -notin @(0x01, 0x02, 0x03) -or $body[1] -ne $Sequence -or $body[2] -ne 0x02) { continue }
            [byte[]]$data = $body.GetRange(6, $length).ToArray()
            if ($body[3] -eq 0) {
                throw ('Device returned a plain ACK/NACK: ' + [BitConverter]::ToString($data))
            }
            if ($body[3] -ne $Command) { continue }
            return ,$data
        }
        if ($escaped) { $body.Add([byte]($b -bxor 0x20)); $escaped = $false }
        elseif ($b -eq 0x7D) { $escaped = $true }
        else { $body.Add([byte]$b) }
        if ($body.Count -gt 2304) { $inside = $false; $body.Clear() }
    }
    throw ('No valid reply to read command 0x{0:X2} within 1.5 seconds.' -f $Command)
}

if ($SelfTest) {
    if ((Get-Crc16 ([Text.Encoding]::ASCII.GetBytes('123456789'))) -ne 0x4B37) {
        throw 'CRC-16/MODBUS reference-vector check failed.'
    }
    $blocked = $false
    try { $null = New-ReadFrame 1 0x70 } catch { $blocked = $true }
    if (-not $blocked) { throw 'Read-command allowlist failed.' }
    [byte[]]$frame = New-ReadFrame 1 0x02
    if ($frame[0] -ne 0xAA -or $frame[-1] -ne 0x55) { throw 'Invalid frame delimiters.' }
    Write-Output ('Offline checks passed. GetVersion frame: ' + [BitConverter]::ToString($frame))
    return
}

# Ensure the chosen port is the observed CH342/CH343 control interface (MI_02).
if ($Port -notin [IO.Ports.SerialPort]::GetPortNames()) { throw "$Port is not connected." }
$usbKey = 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB\VID_1A86&PID_55D2&MI_02'
$matching = @(Get-ChildItem -LiteralPath $usbKey -ErrorAction SilentlyContinue | Where-Object {
    $params = Get-ItemProperty -LiteralPath ($_.PSPath + '\Device Parameters') -ErrorAction SilentlyContinue
    $params.PortName -eq $Port
})
if ($matching.Count -ne 1) { throw "$Port is not a uniquely identified XTac control interface; no commands sent." }

$serial = New-Object IO.Ports.SerialPort $Port,3000000,'None',8,'One'
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
try {
    $serial.Open()
    Write-Output "Connected to $Port at 3000000 baud; read-only queries."
    $values = @{}
    [byte]$seq = 1
    foreach ($command in @(0x02, 0x04, 0x06)) {
        [byte[]]$frame = New-ReadFrame $seq $command
        $serial.Write($frame, 0, $frame.Length)
        [byte[]]$reply = Read-Reply $serial $seq $command
        $values[$command] = $reply
        Write-Output ('Read 0x{0:X2}: {1}' -f $command, [BitConverter]::ToString($reply))
        $seq++
    }
    if ($values[0x02].Length -ne 4) { throw 'Unexpected firmware version reply length; raw reply shown above.' }
    $version = ($values[0x02][0..2] -join '.')
    $sn = [Text.Encoding]::ASCII.GetString($values[0x04]).Split([char]0)[0]
    if ($sn -notmatch '^TCGU01[A-Za-z][0-9]{2}[A-Za-z][0-9]{4}[ms]$') {
        throw 'Unexpected MCU serial number; raw reply shown above. Identity not confirmed.'
    }
    if ($values[0x06].Length -ne 1) { throw 'Unexpected device-side reply length.' }
    $role = if ($sn.EndsWith('s')) { 'Follower' } else { 'Leader' }
    $side = if ($values[0x06][0] -eq 0) { 'Left' } elseif ($values[0x06][0] -eq 1) { 'Right' } else { 'Unknown' }
    [pscustomobject]@{ Port=$Port; Firmware=$version; SerialNumber=$sn; Role=$role; DeviceSide=$side }
} catch {
    throw "$($_.Exception.Message) No firmware update or motor command was sent."
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
