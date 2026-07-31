#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

UAV1_HOST="${UAV1_HOST:-orangepi@192.168.0.29}"
UAV2_HOST="${UAV2_HOST:-orangepi@192.168.0.192}"
STATION5_HOST="${STATION5_HOST:-pi@192.168.0.224}"
STATION37_HOST="${STATION37_HOST:-pi@192.168.0.239}"
CONTROL_IP="192.168.0.90"

SSH_OPTIONS=(
    -o ConnectTimeout=5
    -o ServerAliveInterval=5
    -o ServerAliveCountMax=3
    -o ControlMaster=auto
    -o ControlPersist=60
    -o ControlPath=/tmp/energy-race-ssh-%C
)

for command in ssh python3 ip; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "ERROR: не найдена команда: $command" >&2
        exit 1
    fi
done

if ! ip -4 address show | grep -q "${CONTROL_IP}/"; then
    echo "ERROR: на ноутбуке нет адреса ${CONTROL_IP}." >&2
    echo "Подключись к командному роутеру и повтори запуск." >&2
    exit 1
fi

start_remote() {
    local host="$1"
    local label="$2"
    local script="$3"
    local allow_existing="$4"

    echo "[$label] проверка $host"
    ssh "${SSH_OPTIONS[@]}" "$host" bash -s -- \
        "$label" "$script" "$allow_existing" <<'REMOTE'
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
nohup python3 -u "$script" >"$log_file" 2>&1 </dev/null &
pid="$!"
echo "$pid" >"$pid_file"
sleep 1

if ! kill -0 "$pid" 2>/dev/null; then
    echo "ERROR: [$label] завершился сразу после запуска" >&2
    tail -n 20 "$log_file" >&2 || true
    exit 1
fi

echo "[$label] запущен, PID $pid, лог $log_file"
REMOTE
}

echo "Запуск станций"
start_remote "$STATION5_HOST" "station-5" "station.py" "yes"
start_remote "$STATION37_HOST" "station-37" "station.py" "yes"

echo "Ожидание камер станций"
sleep 2

echo "Запуск бортовых миссий (моторы ждут START от control.py)"
start_remote "$UAV1_HOST" "uav1" "uav1.py" "no"
start_remote "$UAV2_HOST" "uav2" "uav2.py" "no"

echo "Запуск контроллера на ноутбуке"
exec python3 "$ROOT_DIR/scripts/control.py"
