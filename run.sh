#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-mission}"

if [[ "$mode" == "check" ]]; then
    ENERGY_RACE_MODE=check exec python3 "$project_dir/main.py"
fi

if [[ "$mode" == "signal" ]]; then
    event="${2:?usage: ./run.sh signal EVENT IP [IP ...]}"
    shift 2
    if [[ "$#" -eq 0 ]]; then
        echo "usage: ./run.sh signal EVENT IP [IP ...]" >&2
        exit 2
    fi

    python3 - "$project_dir/config.json" "$event" "$@" <<'PY'
import json
import socket
import sys
import time

with open(sys.argv[1], encoding="utf-8") as stream:
    port = int(json.load(stream)["network"]["event_port"])

event = sys.argv[2]
message = json.dumps({"event": event}).encode()
for host in sys.argv[3:]:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _ in range(3):
            sock.sendto(message, (host, port))
            time.sleep(0.05)
    print(f"{event} -> {host}:{port}")
PY
    exit 0
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy not found: /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

for setup_file in \
    /opt/clover2/setup.bash \
    "$HOME/clover2/install/setup.bash" \
    "$HOME/clover2-dev/install/setup.bash"
do
    if [[ -f "$setup_file" ]]; then
        # shellcheck disable=SC1090
        source "$setup_file"
        break
    fi
done

export PYTHONUNBUFFERED=1

case "$mode" in
    mission)
        export ENERGY_RACE_MODE=mission
        ;;
    smoke)
        export ENERGY_RACE_MODE=smoke
        ;;
    *)
        echo "usage: ./run.sh [mission|check|smoke|signal]" >&2
        exit 2
        ;;
esac

exec python3 "$project_dir/main.py"
