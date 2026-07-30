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
OPERATOR_TIMEOUT = float(CONFIG["timing"]["operator_timeout"])


def send_events(sock, commands):
    messages = []
    for event, target in commands:
        messages.append((
            event,
            target,
            json.dumps({
                "team": TEAM,
                "event": event,
                "target": target,
            }).encode("utf-8"),
        ))

    for _ in range(8):
        for _, target, payload in messages:
            sock.sendto(payload, (DRONES[target], EVENT_PORT))
        time.sleep(0.05)

    for event, target, _ in messages:
        print("{} -> {} ({})".format(event, target, DRONES[target]))


def receive_status(sock, states):
    sock.settimeout(1.0)
    try:
        payload, address = sock.recvfrom(4096)
    except socket.timeout:
        return

    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return
    if message.get("team") != TEAM or message.get("event") != "STATUS":
        return

    role = message.get("uav")
    state = message.get("state")
    if role not in states:
        return
    if state in states[role]:
        return
    states[role].add(state)
    print("{}: {} ({})".format(role, state, address[0]))


def wait_states(sock, states, required, description):
    deadline = time.monotonic() + OPERATOR_TIMEOUT
    while time.monotonic() < deadline:
        if all(state in states[role] for role, state in required):
            return
        receive_status(sock, states)

    raise RuntimeError("таймаут: {}".format(description))


with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", CONTROL_PORT))

    states = {"uav1": set(), "uav2": set()}
    print("Запусти uav1.py и uav2.py на соответствующих дронах.")
    print("Ожидаю автоматический READY от обоих БВС.")
    wait_states(
        sock,
        states,
        (("uav1", "READY"), ("uav2", "READY")),
        "не получен READY от обоих БВС",
    )

    print("Оба БВС готовы и мигают жёлтым.")
    if input("Для синхронного старта введи START: ").strip() != "START":
        raise SystemExit("Старт отменён")

    send_events(sock, (("START", "uav1"), ("START", "uav2")))
    wait_states(
        sock,
        states,
        (("uav1", "STATION_LANDED"), ("uav2", "CARGO_LANDED")),
        "оба БВС не сели и не разоружились",
    )

    print()
    print("Оба БВС landed/disarmed. Можно войти и установить груз.")
    print("После установки обязательно выйти из полётной зоны.")
    if input("Когда человек вышел из клетки, введи FLY: ").strip() != "FLY":
        raise SystemExit("Продолжение отменено, дроны остаются disarmed")

    send_events(sock, (("CARGO_LOADED", "uav2"),))
    wait_states(
        sock,
        states,
        (("uav1", "CHARGE_DONE"), ("uav2", "CARGO_READY")),
        "не завершена зарядка БВС-1 или захват груза БВС-2",
    )

    send_events(
        sock,
        (("RETURN_HOME", "uav1"), ("UAV2_DEPART", "uav2")),
    )
    print("БВС-1 возвращается домой, БВС-2 летит с грузом к станции.")

    wait_states(
        sock,
        states,
        (("uav1", "DONE"), ("uav2", "DONE")),
        "миссия не завершена обоими БВС",
    )
    print("Миссия завершена обоими БВС.")
