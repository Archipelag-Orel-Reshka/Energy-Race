<#
.SYNOPSIS
  Windows (PowerShell) deploy script for the Energy-Race mission.
  Counterpart of update_all.sh.  Remote boards still run bash over SSH.
#>

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 3

$ROOT_DIR = $PSScriptRoot

# --- Configuration (overridable via environment variables) ------------------
$UAV1_IP        = if ($env:UAV1_IP)        { $env:UAV1_IP }        else { "192.168.0.29" }
$UAV2_IP        = if ($env:UAV2_IP)        { $env:UAV2_IP }        else { "192.168.0.184" }
$UAV1_HOST      = if ($env:UAV1_HOST)      { $env:UAV1_HOST }      else { "orangepi@$UAV1_IP" }
$UAV2_HOST      = if ($env:UAV2_HOST)      { $env:UAV2_HOST }      else { "orangepi@$UAV2_IP" }
$STATION5_HOST  = if ($env:STATION5_HOST)  { $env:STATION5_HOST }  else { "pi@192.168.0.224" }
$STATION37_HOST = if ($env:STATION37_HOST) { $env:STATION37_HOST } else { "pi@192.168.0.239" }

$sshOptions = @(
    "-o", "ConnectTimeout=5",
    "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=3"
)
$scpOptions = $sshOptions

# --- Helpers ----------------------------------------------------------------

function Find-Python {
    foreach ($cmd in @("python", "python3")) {
        if (Get-Command $cmd -ErrorAction SilentlyContinue) {
            return $cmd
        }
    }
    return $null
}

# --- Pre-flight checks ------------------------------------------------------

$python = Find-Python
foreach ($cmd in @("ssh", "scp", $python)) {
    if (-not $cmd -or -not (Get-Command $cmd -ErrorAction SilentlyContinue)) {
        [Console]::Error.WriteLine("ERROR: не найдена команда: $cmd")
        exit 1
    }
}

$requiredFiles = @(
    "scripts\mission.py",
    "scripts\mission_config.json",
    "scripts\uav1.py",
    "scripts\uav2.py",
    "scripts\test_half_red_blue.py",
    "station\station.py",
    "station\calibrate.py",
    "station\field\station-5\red\config.json",
    "station\field\station-5\red\calibration.json",
    "station\field\station-37\red\config.json",
    "station\field\station-37\red\calibration.json"
)

foreach ($rel in $requiredFiles) {
    $full = Join-Path $ROOT_DIR $rel
    if (-not (Test-Path $full -PathType Leaf)) {
        [Console]::Error.WriteLine("ERROR: отсутствует файл $full")
        exit 1
    }
}

if ($env:SKIP_TESTS -ne "1") {
    Write-Host "Локальная проверка перед обновлением"
    & $python -m compileall -q scripts tests station
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $python -m json.tool (Join-Path $ROOT_DIR "scripts\mission_config.json") | Out-Null
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $python -m unittest discover -s tests
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# --- Remote helpers ---------------------------------------------------------

function Assert-RemoteIdle {
    param(
        [string]$RemoteHost,
        [string]$Pattern,
        [string]$Label
    )

    # Single-quote the regex so characters such as '(' and '|' are not parsed
    # by the remote shell before bash receives them as positional arguments.
    $remoteCommand = "bash -s -- '$Pattern' '$Label'"

    $remoteScript = @'
set -eu
pattern="$1"
label="$2"
pid=""
for candidate in $(pgrep -f "$pattern" || true); do
    if [ "$candidate" != "$$" ] && [ "$candidate" != "$PPID" ]; then
        pid="$candidate"
        break
    fi
done
if [ -n "$pid" ]; then
    echo "ERROR: [$label] работает процесс PID $pid" >&2
    echo "Сначала убедись, что БВС disarmed, затем запусти stop_mission." >&2
    exit 1
fi
mkdir -p "$HOME/scripts/.energy-race-deploy"
'@

    ($remoteScript -replace "`r`n", "`n") | & ssh @sshOptions $RemoteHost $remoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "[$Label] удалённая проверка не пройдена (код $LASTEXITCODE)."
    }
}

function Deploy-Uav {
    param(
        [string]$RemoteHost,
        [string]$Label
    )

    Write-Host "[$Label] проверка процессов"
    Assert-RemoteIdle -RemoteHost $RemoteHost `
        -Pattern 'python3.*(uav1\.py|uav2\.py|mission\.py)' -Label $Label

    Write-Host "[$Label] копирование основной миссии"
    $dest = "${RemoteHost}:~/scripts/.energy-race-deploy/"
    & scp @scpOptions `
        (Join-Path $ROOT_DIR "scripts\mission.py") `
        (Join-Path $ROOT_DIR "scripts\mission_config.json") `
        (Join-Path $ROOT_DIR "scripts\uav1.py") `
        (Join-Path $ROOT_DIR "scripts\uav2.py") `
        (Join-Path $ROOT_DIR "scripts\test_half_red_blue.py") `
        $dest
    if ($LASTEXITCODE -ne 0) { throw "[$Label] scp завершился с ошибкой (код $LASTEXITCODE)." }

    $remoteScript = @'
set -eu
stage="$HOME/scripts/.energy-race-deploy"
backup="$HOME/scripts/backups/$(date +%Y%m%d-%H%M%S)-main"
mkdir -p "$backup"
for file in mission.py mission_config.json uav1.py uav2.py test_half_red_blue.py; do
    if [ -f "$HOME/scripts/$file" ]; then
        cp -p "$HOME/scripts/$file" "$backup/$file"
    fi
done
install -m 755 "$stage/mission.py" "$HOME/scripts/mission.py"
install -m 644 "$stage/mission_config.json" "$HOME/scripts/mission_config.json"
install -m 755 "$stage/uav1.py" "$HOME/scripts/uav1.py"
install -m 755 "$stage/uav2.py" "$HOME/scripts/uav2.py"
install -m 755 "$stage/test_half_red_blue.py" "$HOME/scripts/test_half_red_blue.py"
echo "backup: $backup"
'@

    ($remoteScript -replace "`r`n", "`n") | & ssh @sshOptions $RemoteHost "bash -s"
    if ($LASTEXITCODE -ne 0) {
        throw "[$Label] удалённая установка завершилась с ошибкой (код $LASTEXITCODE)."
    }
}


function Deploy-Station {
    param(
        [string]$RemoteHost,
        [string]$StationId
    )

    $sourceDir = Join-Path $ROOT_DIR "station\field\station-$StationId\red"
    $label = "station-$StationId"

    Write-Host "[$label] проверка процессов"
    Assert-RemoteIdle -RemoteHost $RemoteHost -Pattern 'python3.*station\.py' -Label $label

    Write-Host "[$label] копирование кода и собственной калибровки"
    $dest = "${RemoteHost}:~/scripts/.energy-race-deploy/"
    & scp @scpOptions `
        (Join-Path $ROOT_DIR "station\station.py") `
        (Join-Path $ROOT_DIR "station\calibrate.py") `
        (Join-Path $sourceDir "config.json") `
        (Join-Path $sourceDir "calibration.json") `
        $dest
    if ($LASTEXITCODE -ne 0) { throw "[$label] scp завершился с ошибкой (код $LASTEXITCODE)." }

    $remoteScript = @'
set -eu
station_id="$1"
stage="$HOME/scripts/.energy-race-deploy"
actual_id="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["station_id"])' "$stage/config.json")"
if [ "$actual_id" != "$station_id" ]; then
    echo "ERROR: ожидался station_id=$station_id, получен $actual_id" >&2
    exit 1
fi
backup="$HOME/scripts/backups/$(date +%Y%m%d-%H%M%S)-station-$station_id"
mkdir -p "$backup"
for file in station.py calibrate.py config.json calibration.json; do
    if [ -f "$HOME/scripts/$file" ]; then
        cp -p "$HOME/scripts/$file" "$backup/$file"
    fi
done
install -m 755 "$stage/station.py" "$HOME/scripts/station.py"
install -m 755 "$stage/calibrate.py" "$HOME/scripts/calibrate.py"
install -m 644 "$stage/config.json" "$HOME/scripts/config.json"
install -m 644 "$stage/calibration.json" "$HOME/scripts/calibration.json"
echo "backup: $backup"
'@

    $remoteCommand = "bash -s -- '$StationId'"
    ($remoteScript -replace "`r`n", "`n") | & ssh @sshOptions $RemoteHost $remoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "[$label] удалённая установка завершилась с ошибкой (код $LASTEXITCODE)."
    }
}

# --- Deploy -----------------------------------------------------------------

Deploy-Uav -RemoteHost $UAV1_HOST -Label "uav1"
Deploy-Uav -RemoteHost $UAV2_HOST -Label "uav2"
Deploy-Station -RemoteHost $STATION5_HOST  -StationId 5
Deploy-Station -RemoteHost $STATION37_HOST -StationId 37

Write-Host ""
Write-Host "Обновление завершено. Калибровки станций не смешаны."
Write-Host "Следующий шаг: .\mission.ps1"

