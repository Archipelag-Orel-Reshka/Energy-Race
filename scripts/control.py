#!/usr/bin/env python3

import json
import os
import socket
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = Path(os.environ.get(
    "ENERGY_RACE_CONFIG",
    str(ROOT / "mission_config.json"),
))
CONFIG = json.loads(
    CONFIG_PATH.read_text(encoding="utf-8")
)
TEAM = CONFIG["team"]
NETWORK = dict(CONFIG["network"])
NETWORK["uav1_ip"] = os.environ.get(
    "ENERGY_RACE_UAV1_IP", NETWORK["uav1_ip"]
)
NETWORK["uav2_ip"] = os.environ.get(
    "ENERGY_RACE_UAV2_IP", NETWORK["uav2_ip"]
)
DRONES = {
    "uav1": NETWORK["uav1_ip"],
    "uav2": NETWORK["uav2_ip"],
}
EVENT_PORT = int(NETWORK["event_port"])
CONTROL_IP = NETWORK["control_ip"]
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
    DRONES[role] = address[0]
    if state in ("FAILED", "INTERRUPTED"):
        detail = message.get("error") or state
        raise RuntimeError("{}: {}".format(role, detail))
    if state in states[role]:
        return
    states[role].add(state)
    print("{}: {} ({})".format(role, state, address[0]))
    if role == "uav2" and message.get("servo_ok") is False:
        if state == "READY":
            print(
                "WARNING: uav2: серво недоступно или отключено; "
                "миссия всё равно может быть запущена."
            )
        elif state == "CARGO_READY":
            print(
                "WARNING: uav2: программное закрытие серво не удалось; "
                "миссия продолжится, проверь груз физически."
            )


def wait_states(sock, states, required, description):
    deadline = time.monotonic() + OPERATOR_TIMEOUT
    while time.monotonic() < deadline:
        aborted = [
            role for role, role_states in states.items()
            if "ABORTED" in role_states
        ]
        if aborted:
            raise RuntimeError(
                "{} безопасно прервал миссию и вернулся домой".format(
                    ", ".join(aborted)
                )
            )
        if all(state in states[role] for role, state in required):
            return
        receive_status(sock, states)

    raise RuntimeError("таймаут: {}".format(description))


def run_controller():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((CONTROL_IP, CONTROL_PORT))
        except OSError:
            raise SystemExit(
                "IP {} не назначен ноутбуку. Подключись к командному "
                "роутеру и проверь: ip -4 addr show wlan0".format(
                    CONTROL_IP
                )
            )

        states = {"uav1": set(), "uav2": set()}
        print("Контроллер: {}:{}".format(CONTROL_IP, CONTROL_PORT))
        print("Запусти uav1.py и uav2.py на соответствующих дронах.")
        print("Ожидаю автоматический READY от обоих БВС.")
        wait_states(
            sock,
            states,
            (("uav1", "READY"), ("uav2", "READY")),
            "не получен READY от обоих БВС",
        )

        print("Оба БВС готовы и мигают жёлтым.")
        prompt = "Для синхронного взлёта обоих БВС введи START: "
        if input(prompt).strip() != "START":
            raise SystemExit("Старт отменён")

        send_events(sock, (("START", "uav1"), ("START", "uav2")))
        print("Оба БВС начали взлёт синхронно.")
        print("БВС-1 после набора 2 м ждёт 5 с и летит к станции 5.")
        print("БВС-2 после набора 2 м сразу летит к грузу 0.")

        wait_states(
            sock,
            states,
            (("uav1", "STATION_LANDED"), ("uav2", "CARGO_LANDED")),
            "оба БВС не сели и не разоружились",
        )

        print()
        print("Оба БВС landed/disarmed. Можно войти и установить груз.")
        print("После установки обязательно выйти из полётной зоны.")
        prompt = "Когда человек вышел из клетки, введи FLY: "
        if input(prompt).strip() != "FLY":
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
        print(
            "БВС-1 возвращается на H/48, БВС-2 несёт груз на станцию 37."
        )

        wait_states(
            sock,
            states,
            (("uav1", "DONE"), ("uav2", "DONE")),
            "миссия не завершена обоими БВС",
        )
        print("Миссия завершена обоими БВС.")


def main():
    try:
        run_controller()
        return 0
    except KeyboardInterrupt:
        print("\nКонтроллер остановлен оператором.")
        return 130
    except RuntimeError as error:
        print("ERROR: {}".format(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
