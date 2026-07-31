#!/usr/bin/env python3

import os
import socket
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SCRIPTS_ROOT = ROOT.parent
sys.path.insert(0, str(SCRIPTS_ROOT))
os.environ["ENERGY_RACE_CONFIG"] = str(ROOT / "mission_config.json")

import control as shared  # noqa: E402


START_DELAY = float(shared.CONFIG["timing"]["uav2_start_delay"])


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
        print("Demo-контроллер: {}:{}".format(
            shared.CONTROL_IP,
            shared.CONTROL_PORT,
        ))
        print("Ожидаю автоматический READY от обоих БВС.")
        shared.wait_states(
            sock,
            states,
            (("uav1", "READY"), ("uav2", "READY")),
            "не получен READY от обоих БВС",
        )

        print("Оба БВС готовы и мигают жёлтым.")
        prompt = "Для поэтапного старта введи START: "
        if input(prompt).strip() != "START":
            raise SystemExit("Старт отменён")

        shared.send_events(sock, (("START", "uav1"),))
        print("Жду завершения взлёта БВС-1.")
        shared.wait_states(
            sock,
            states,
            (("uav1", "TAKEOFF_DONE"),),
            "БВС-1 не завершил взлёт",
        )
        print("БВС-1 взлетел. Жду освобождения пересечения у ArUco 26.")
        shared.wait_states(
            sock,
            states,
            (("uav1", "ROUTE_CLEAR"),),
            "БВС-1 не освободил общий коридор",
        )
        print("Общий коридор свободен. Задержка {:.1f} с перед БВС-2.".format(
            START_DELAY
        ))
        time.sleep(START_DELAY)
        shared.send_events(sock, (("START", "uav2"),))

        shared.wait_states(
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

        shared.send_events(sock, (("CARGO_LOADED", "uav2"),))
        shared.wait_states(
            sock,
            states,
            (("uav1", "CHARGE_DONE"), ("uav2", "CARGO_READY")),
            "не завершена зарядка БВС-1 или захват груза БВС-2",
        )

        shared.send_events(
            sock,
            (("RETURN_HOME", "uav1"), ("UAV2_DEPART", "uav2")),
        )
        print("БВС-1 возвращается на 48, БВС-2 летит с грузом на 37.")

        shared.wait_states(
            sock,
            states,
            (("uav1", "DONE"), ("uav2", "DONE")),
            "миссия не завершена обоими БВС",
        )
        print("Demo-миссия завершена обоими БВС.")


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
