<#
.SYNOPSIS
  Windows (PowerShell) stop script for the Energy-Race mission.
  Counterpart of stop_mission.sh.  Remote boards still run bash over SSH.
#>

$ErrorActionPreference = "Stop"
Set-StrictMode -Version 3

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

# --- Confirmation -----------------------------------------------------------

$answer = Read-Host "Все БВС landed/disarmed? Введи DISARMED для остановки"
if ($answer -ne "DISARMED") {
    Write-Host "Остановка отменена."
    exit 1
}

# --- Helpers ----------------------------------------------------------------

function Invoke-RemoteStop {
    param(
        [string]$RemoteHost,
        [string]$Label,
        [string]$ScriptName
    )

    $remoteCommand = "bash -s -- '$Label' '$ScriptName'"

    $remoteScript = @'
set -eu
label="$1"
script="$2"
pid_file="$HOME/scripts/.energy-race/$label.pid"

if [ ! -s "$pid_file" ]; then
    echo "[$label] PID-файл отсутствует, ничего не остановлено"
    exit 0
fi

pid="$(cat "$pid_file")"
if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    echo "[$label] уже завершён"
    exit 0
fi

cmdline="$(tr '\000' ' ' <"/proc/$pid/cmdline")"
case "$cmdline" in
    *"$script"*) ;;
    *)
        echo "ERROR: PID $pid больше не принадлежит $script; не трогаю его" >&2
        exit 1
        ;;
esac

kill -TERM "$pid"
for _ in 1 2 3 4 5; do
    if ! kill -0 "$pid" 2>/dev/null; then
        rm -f "$pid_file"
        echo "[$label] остановлен"
        exit 0
    fi
    sleep 1
done

echo "WARNING: [$label] не завершился за 5 секунд; SIGKILL не отправлялся" >&2
exit 1
'@

    ($remoteScript -replace "`r`n", "`n") | & ssh @sshOptions $RemoteHost $remoteCommand
    return $LASTEXITCODE
}

# --- Stop all remotes -------------------------------------------------------

$failures = 0

$targets = @(
    @{ Host = $UAV1_HOST;      Label = "uav1";       Script = "uav1.py" },
    @{ Host = $UAV2_HOST;      Label = "uav2";       Script = "uav2.py" },
    @{ Host = $STATION5_HOST;  Label = "station-5";  Script = "station.py" },
    @{ Host = $STATION37_HOST; Label = "station-37"; Script = "station.py" }
)

foreach ($t in $targets) {
    $code = Invoke-RemoteStop -RemoteHost $t.Host -Label $t.Label -ScriptName $t.Script
    if ($code -ne 0) {
        $failures++
    }
}

if ($failures -ne 0) {
    [Console]::Error.WriteLine("WARNING: не удалось подтвердить остановку на $failures устройствах.")
    exit 1
}

Write-Host "Все процессы, запущенные mission.ps1, остановлены или уже завершены."
