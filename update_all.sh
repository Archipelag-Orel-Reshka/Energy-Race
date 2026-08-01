#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

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
SCP_OPTIONS=("${SSH_OPTIONS[@]}")

for command in ssh scp python3; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "ERROR: не найдена команда: $command" >&2
        exit 1
    fi
done

required_files=(
    "$ROOT_DIR/scripts/mission.py"
    "$ROOT_DIR/scripts/mission_config.json"
    "$ROOT_DIR/scripts/uav1.py"
    "$ROOT_DIR/scripts/uav2.py"
    "$ROOT_DIR/scripts/test_half_red_blue.py"
    "$ROOT_DIR/station/station.py"
    "$ROOT_DIR/station/calibrate.py"
    "$ROOT_DIR/station/field/station-5/red/config.json"
    "$ROOT_DIR/station/field/station-5/red/calibration.json"
    "$ROOT_DIR/station/field/station-37/red/config.json"
    "$ROOT_DIR/station/field/station-37/red/calibration.json"
)

for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        echo "ERROR: отсутствует файл $file" >&2
        exit 1
    fi
done

if [ "${SKIP_TESTS:-0}" != "1" ]; then
    echo "Локальная проверка перед обновлением"
    (
        cd "$ROOT_DIR"
        python3 -m compileall -q scripts tests station
        python3 -m json.tool scripts/mission_config.json >/dev/null
        python3 -m unittest discover -s tests
    )
fi

assert_remote_idle() {
    local host="$1"
    local pattern="$2"
    local label="$3"

    ssh "${SSH_OPTIONS[@]}" "$host" bash -s -- \
        "$pattern" "$label" <<'REMOTE'
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
    echo "Сначала убедись, что БВС disarmed, затем запусти ./stop_mission.sh." >&2
    exit 1
fi
mkdir -p "$HOME/scripts/.energy-race-deploy"
REMOTE
}

deploy_uav() {
    local host="$1"
    local label="$2"

    echo "[$label] проверка процессов"
    assert_remote_idle "$host" \
        'python3.*(uav1\.py|uav2\.py|mission\.py)' "$label"

    echo "[$label] копирование основной миссии"
    scp "${SCP_OPTIONS[@]}" \
        "$ROOT_DIR/scripts/mission.py" \
        "$ROOT_DIR/scripts/mission_config.json" \
        "$ROOT_DIR/scripts/uav1.py" \
        "$ROOT_DIR/scripts/uav2.py" \
        "$ROOT_DIR/scripts/test_half_red_blue.py" \
        "$host:~/scripts/.energy-race-deploy/"

    ssh "${SSH_OPTIONS[@]}" "$host" bash -s <<'REMOTE'
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
REMOTE
}

deploy_station() {
    local host="$1"
    local station_id="$2"
    local source_dir="$ROOT_DIR/station/field/station-$station_id/red"
    local label="station-$station_id"

    echo "[$label] проверка процессов"
    assert_remote_idle "$host" 'python3.*station\.py' "$label"

    echo "[$label] копирование кода и собственной калибровки"
    scp "${SCP_OPTIONS[@]}" \
        "$ROOT_DIR/station/station.py" \
        "$ROOT_DIR/station/calibrate.py" \
        "$source_dir/config.json" \
        "$source_dir/calibration.json" \
        "$host:~/scripts/.energy-race-deploy/"

    ssh "${SSH_OPTIONS[@]}" "$host" bash -s -- "$station_id" <<'REMOTE'
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
REMOTE
}

deploy_uav "$UAV1_HOST" "uav1"
deploy_uav "$UAV2_HOST" "uav2"
deploy_station "$STATION5_HOST" 5
deploy_station "$STATION37_HOST" 37

echo
echo "Обновление завершено. Калибровки станций не смешаны."
echo "Следующий шаг: ./mission.sh"
