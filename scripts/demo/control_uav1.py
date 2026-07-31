#!/usr/bin/env python3

import os
import socket
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SCRIPTS_ROOT = ROOT.parent
sys.path.insert(0, str(SCRIPTS_ROOT))
os.environ["ENERGY_RACE_CONFIG"] = str(ROOT / "mission_config.json")

import control as shared  # noqa: E402


def run_controller():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((shared.CONTROL_IP, shared.CONTROL_PORT))
        except OSError:
            raise SystemExit(
                "IP {} не назначен ноутбуку. Подключись к командному "
                "роутеру и проверь: ip -4 addr show wlan0".format(
                    shared.CONTROL_IP
                )
            )

        states = {"uav1": set(), "uav2": set()}
        print("Тест БВС-1: H/48 -> станция 5 -> H/48")
        print("Ожидаю READY только от БВС-1.")
        shared.wait_states(
            sock,
            states,
            (("uav1", "READY"),),
            "не получен READY от БВС-1",
        )

        if input("Для запуска БВС-1 введи START: ").strip() != "START":
            raise SystemExit("Старт отменён")
        shared.send_events(sock, (("START", "uav1"),))

        shared.wait_states(
            sock,
            states,
            (("uav1", "STATION_LANDED"), ("uav1", "CHARGE_DONE")),
            "БВС-1 не сел на станцию 5 или не завершил зарядку",
        )
        print("БВС-1 landed/disarmed на станции 5 и завершил зарядку.")

        prompt = "Полётная зона свободна. Для возврата на H/48 введи RETURN: "
        if input(prompt).strip() != "RETURN":
            raise SystemExit("Возврат отменён, БВС-1 остаётся disarmed")
        shared.send_events(sock, (("RETURN_HOME", "uav1"),))

        shared.wait_states(
            sock,
            states,
            (("uav1", "DONE"),),
            "БВС-1 не завершил возврат на H/48",
        )
        print("Тест БВС-1 завершён: посадка на H/48, disarmed.")


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
