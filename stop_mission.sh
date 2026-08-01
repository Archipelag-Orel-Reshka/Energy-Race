#!/usr/bin/env bash

set -euo pipefail

UAV1_IP="${UAV1_IP:-192.168.0.29}"
UAV2_IP="${UAV2_IP:-192.168.0.184}"
UAV1_HOST="${UAV1_HOST:-orangepi@$UAV1_IP}"
UAV2_HOST="${UAV2_HOST:-orangepi@$UAV2_IP}"
STATION5_HOST="${STATION5_HOST:-pi@192.168.0.224}"
STATION37_HOST="${STATION37_HOST:-pi@192.168.0.239}"

SSH_OPTIONS=(
    -o ConnectTimeout=5
    -o ControlMaster=auto
    -o ControlPersist=60
    -o ControlPath=/tmp/energy-race-ssh-%C
)

read -r -p "Все БВС landed/disarmed? Введи DISARMED для остановки: " answer
if [ "$answer" != "DISARMED" ]; then
    echo "Остановка отменена."
    exit 1
fi

stop_remote() {
    local host="$1"
    local label="$2"
    local script="$3"
    local remote_command

    printf -v remote_command 'bash -s -- %q %q' "$label" "$script"

    ssh "${SSH_OPTIONS[@]}" "$host" "$remote_command" <<'REMOTE'
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
REMOTE
}

failures=0
stop_remote "$UAV1_HOST" "uav1" "uav1.py" || failures=$((failures + 1))
stop_remote "$UAV2_HOST" "uav2" "uav2.py" || failures=$((failures + 1))
stop_remote "$STATION5_HOST" "station-5" "station.py" || failures=$((failures + 1))
stop_remote "$STATION37_HOST" "station-37" "station.py" || failures=$((failures + 1))

if [ "$failures" -ne 0 ]; then
    echo "WARNING: не удалось подтвердить остановку на $failures устройствах." >&2
    exit 1
fi

echo "Все процессы, запущенные mission.sh, остановлены или уже завершены."
