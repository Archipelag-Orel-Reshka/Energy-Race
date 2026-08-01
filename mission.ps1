<#
.SYNOPSIS
  Windows (PowerShell) launcher for the Energy-Race mission.
  Counterpart of mission.sh.  Remote boards still run bash over SSH.
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
$CONTROL_IP     = "192.168.0.90"

# ControlMaster/ControlPath are not supported by Windows OpenSSH, so only the
# timeout/keepalive subset of the original SSH options is kept here.
$sshOptions = @(
    "-o", "ConnectTimeout=5",
    "-o", "ServerAliveInterval=5",
    "-o", "ServerAliveCountMax=3"
)

# --- Helpers ----------------------------------------------------------------

function Find-Python {
    foreach ($cmd in @("python", "python3")) {
        if (Get-Command $cmd -ErrorAction SilentlyContinue) {
            return $cmd
        }
    }
    return $null
}

function Test-ControlIp {
    param([string]$Ip)

    try {
        $addresses = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop |
            Select-Object -ExpandProperty IPAddress
        if ($addresses -contains $Ip) { return $true }
    } catch {
        $output = (ipconfig 2>$null | Out-String)
        if ($output -match [regex]::Escape($Ip)) { return $true }
    }
    return $false
}

function Invoke-RemoteLaunch {
    param(
        [string]$RemoteHost,
        [string]$Label,
        [string]$ScriptName,
        [string]$AllowExisting
    )

    $remoteCommand = "bash -s -- '$Label' '$ScriptName' '$AllowExisting'"

    Write-Host "[$Label] проверка $RemoteHost"

    $remoteScript = @'
set -eu

label="$1"
script="$2"
allow_existing="$3"
scripts_dir="$HOME/scripts"
runtime_dir="$scripts_dir/.energy-race"
pid_file="$runtime_dir/$label.pid"

mkdir -p "$runtime_dir" "$scripts_dir/logs"
cd "$scripts_dir"

if [ ! -f "$script" ]; then
    echo "ERROR: $scripts_dir/$script не найден" >&2
    exit 1
fi

if [ -s "$pid_file" ]; then
    pid="$(cat "$pid_file")"
    if kill -0 "$pid" 2>/dev/null; then
        cmdline="$(tr '\000' ' ' <"/proc/$pid/cmdline")"
        case "$cmdline" in
            *"$script"*)
                if [ "$allow_existing" = "yes" ]; then
                    echo "[$label] уже запущен, PID $pid"
                    exit 0
                fi
                echo "ERROR: [$label] уже запущен, PID $pid" >&2
                exit 1
                ;;
            *)
                echo "[$label] PID $pid переиспользован другим процессом"
                rm -f "$pid_file"
                ;;
        esac
    fi
    rm -f "$pid_file"
fi

existing_pid="$(pgrep -f "python3.*$script" | head -n 1 || true)"
if [ -n "$existing_pid" ]; then
    if [ "$allow_existing" = "yes" ]; then
        echo "[$label] уже запущен вне launcher, PID $existing_pid"
        exit 0
    fi
    echo "ERROR: найден старый процесс $script, PID $existing_pid" >&2
    echo "Останови его на земле перед новой попыткой." >&2
    exit 1
fi

stamp="$(date +%Y%m%d-%H%M%S)"
log_file="$scripts_dir/logs/$label-$stamp.log"
# The board user's interactive shell config provides ROS/Python paths. Plain
# non-interactive SSH does not load it, which makes imports such as rospy fail.
printf -v launch_command 'exec python3 -u %q' "$script"
nohup bash -ic "$launch_command" >"$log_file" 2>&1 </dev/null &
pid="$!"
echo "$pid" >"$pid_file"
sleep 1

if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: [$label] завершился сразу после запуска" >&2
    tail -n 20 "$log_file" >&2 || true
    exit 1
fi

echo "[$label] запущен, PID $pid, лог $log_file"
'@

    # Normalise CRLF -> LF so the remote bash receives clean Unix line endings.
    ($remoteScript -replace "`r`n", "`n") | & ssh @sshOptions $RemoteHost $remoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "[$Label] удалённый запуск завершился с ошибкой (код $LASTEXITCODE)."
    }
}

# --- Pre-flight checks ------------------------------------------------------

$python = Find-Python
foreach ($cmd in @("ssh", $python)) {
    if (-not $cmd -or -not (Get-Command $cmd -ErrorAction SilentlyContinue)) {
        [Console]::Error.WriteLine("ERROR: не найдена команда: $cmd")
        exit 1
    }
}

if (-not (Test-ControlIp -Ip $CONTROL_IP)) {
    [Console]::Error.WriteLine("ERROR: на ноутбуке нет адреса $CONTROL_IP.")
    [Console]::Error.WriteLine("Подключись к командному роутеру и повтори запуск.")
    exit 1
}

# --- Launch sequence --------------------------------------------------------

Write-Host "Запуск станций"
Write-Host "Адреса БВС: uav1=$UAV1_IP, uav2=$UAV2_IP"
Invoke-RemoteLaunch -RemoteHost $STATION5_HOST  -Label "station-5" -ScriptName "station.py" -AllowExisting "yes"
Invoke-RemoteLaunch -RemoteHost $STATION37_HOST -Label "station-37" -ScriptName "station.py" -AllowExisting "yes"

Write-Host "Ожидание камер станций"
Start-Sleep -Seconds 2

if ($env:STATIONS_ONLY -eq "1") {
    Write-Host "Запущены только станции; процессы БВС и control.py не запускались."
    exit 0
}

Write-Host "Запуск бортовых миссий (моторы ждут START от control.py)"
Invoke-RemoteLaunch -RemoteHost $UAV1_HOST -Label "uav1" -ScriptName "uav1.py" -AllowExisting "no"
Invoke-RemoteLaunch -RemoteHost $UAV2_HOST -Label "uav2" -ScriptName "uav2.py" -AllowExisting "no"

Write-Host "Запуск контроллера на ноутбуке"
$env:ENERGY_RACE_UAV1_IP = $UAV1_IP
$env:ENERGY_RACE_UAV2_IP = $UAV2_IP
& $python (Join-Path $ROOT_DIR "scripts\control.py")
exit $LASTEXITCODE
