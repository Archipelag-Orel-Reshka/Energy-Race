#!/usr/bin/env python3

import json
import socket
import time
import uuid


TEAM = "orel-reshka"
DRONE_ID = "uav1"
DRONE_PORT = 45900
STATION_PORT = 45901
TIMEOUT = 40.0


station_ip = input("IP Raspberry Pi станции: ").strip()
station_id = int(input("Номер станции (5 или 37): ").strip())
request_id = str(uuid.uuid4())

request = {
    "team": TEAM,
    "event": "REQUEST_LAND",
    "uav": DRONE_ID,
    "station": station_id,
    "request_id": request_id,
    "reply_port": DRONE_PORT,
    "led": "green",
}

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", DRONE_PORT))
    sock.settimeout(1.0)

    payload = json.dumps(request).encode("utf-8")
    for _ in range(5):
        sock.sendto(payload, (station_ip, STATION_PORT))
        time.sleep(0.05)

    print("REQUEST_LAND отправлен. Покажи камере зелёную ленту.")
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        try:
            data, address = sock.recvfrom(4096)
        except socket.timeout:
            continue

        message = json.loads(data.decode("utf-8"))
        print("Получено от {}: {}".format(address[0], message))
        if (
            message.get("event") == "LAND_GRANTED"
            and message.get("request_id") == request_id
        ):
            break
    else:
        raise RuntimeError("LAND_GRANTED не получен")

    input("Тест успешен. Нажми Enter, чтобы освободить станцию: ")
    release = json.dumps({
        "team": TEAM,
        "event": "STATION_RELEASED",
        "uav": DRONE_ID,
        "station": station_id,
        "request_id": request_id,
    }).encode("utf-8")
    for _ in range(3):
        sock.sendto(release, (station_ip, STATION_PORT))
        time.sleep(0.05)
