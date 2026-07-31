#!/usr/bin/env python3

import socket

import control as shared


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
        print("Тест БВС-2: 27 -> груз 0 -> станция 37 -> 27")
        print("Ожидаю READY только от БВС-2.")
        shared.wait_states(
            sock,
            states,
            (("uav2", "READY"),),
            "не получен READY от БВС-2",
        )

        if input("Для запуска БВС-2 введи START: ").strip() != "START":
            raise SystemExit("Старт отменён")
        shared.send_events(sock, (("START", "uav2"),))
        shared.wait_states(
            sock,
            states,
            (("uav2", "CARGO_LANDED"),),
            "БВС-2 не сел у груза на маркере 0",
        )

        prompt = "Установи груз, выйди из зоны и введи FLY: "
        if input(prompt).strip() != "FLY":
            raise SystemExit("Продолжение отменено, БВС-2 остаётся disarmed")
        shared.send_events(sock, (("CARGO_LOADED", "uav2"),))
        shared.wait_states(
            sock,
            states,
            (("uav2", "CARGO_READY"),),
            "БВС-2 не завершил попытку захвата груза",
        )

        prompt = (
            "Проверь фактический захват груза. Для вылета к станции 37 "
            "введи DEPART: "
        )
        if input(prompt).strip() != "DEPART":
            raise SystemExit("Вылет отменён, БВС-2 остаётся disarmed")
        shared.send_events(sock, (("UAV2_DEPART", "uav2"),))
        shared.wait_states(
            sock,
            states,
            (("uav2", "DONE"),),
            "БВС-2 не завершил маршрут через станцию 37",
        )
        print("Тест БВС-2 завершён: груз оставлен, посадка на 27, disarmed.")


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
