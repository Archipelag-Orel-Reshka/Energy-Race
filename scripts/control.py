#!/usr/bin/env python3

import json
import socket
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CONFIG = json.loads(
    (ROOT / "mission_config.json").read_text(encoding="utf-8")
)
TEAM = CONFIG["team"]
NETWORK = CONFIG["network"]
DRONES = {
    "uav1": NETWORK["uav1_ip"],
    "uav2": NETWORK["uav2_ip"],
}
EVENT_PORT = int(NETWORK["event_port"])
CONTROL_PORT = int(NETWORK["control_port"])


def send(sock, event, target):
    payload = json.dumps({
        "team": TEAM,
        "event": event,
        "target": target,
    }).encode("utf-8")
    for _ in range(8):
        sock.sendto(payload, (DRONES[target], EVENT_PORT))
        time.sleep(0.05)
    print("{} -> {} ({})".format(event, target, DRONES[target]))


def send_start(sock):
    messages = {}
    for target in DRONES:
        messages[target] = json.dumps({
            "team": TEAM,
            "event": "START",
            "target": target,
        }).encode("utf-8")
    for _ in range(8):
        for target, payload in messages.items():
            sock.sendto(payload, (DRONES[target], EVENT_PORT))
        time.sleep(0.05)
    print("START -> uav1 и uav2")


def wait_statuses(sock):
    states = {"uav1": set(), "uav2": set()}
    deadline = time.monotonic() + 300.0
    safe_reported = False

    while time.monotonic() < deadline:
        sock.settimeout(1.0)
        try:
            payload, address = sock.recvfrom(4096)
        except socket.timeout:
            continue
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if message.get("team") != TEAM:
            continue
        if message.get("event") != "STATUS":
            continue
        role = message.get("uav")
        state = message.get("state")
        if role not in states:
            continue
        states[role].add(state)
        print("{}: {} ({})".format(role, state, address[0]))

        both_landed = (
            "STATION_LANDED" in states["uav1"]
            and "CARGO_LANDED" in states["uav2"]
        )
        if both_landed and not safe_reported:
            safe_reported = True
            print()
            print("Оба дрона landed/disarmed. Можно войти и установить груз.")
            print("После установки обязательно выйти из полётной зоны.")
            print()

        if both_landed and "CHARGE_DONE" in states["uav1"]:
            return

    raise RuntimeError("дроны не перешли в состояние готовности за 300 секунд")


with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CONTROL_PORT))

    print("Запусти mission.py на обоих дронах и дождись READY.")
    if input("Для синхронного старта введи START: ").strip() != "START":
        raise SystemExit("Старт отменён")

    send_start(sock)
    wait_statuses(sock)

    print("БВС-1 заряжен, оба дрона разоружены.")
    if input("Когда человек вышел из клетки, введи FLY: ").strip() != "FLY":
        raise SystemExit("Продолжение отменено, дроны остаются disarmed")

    send(sock, "RETURN_HOME", "uav1")
    send(sock, "CARGO_LOADED", "uav2")
    print("Продолжение миссии отправлено обоим дронам.")
