#!/usr/bin/env python3
"""Verify a charging station is up and serving a correct STATION_INFO reply.

This is the laptop-side counterpart of Mission.preflight_station() from
scripts/mission.py. mission.sh / mission.ps1 run it right after launching the
stations, so a station that failed to start (most often station-37, because it
is started second and its RosCamera can crash during init if the ROS image
topic is not yet publishing) is caught BEFORE the drones and the controller are
launched.

Without this check the failure only surfaces much later, when uav2 times out
in preflight_station() waiting for STATION_INFO -- the symptom reported as
"mission.sh doesn't get station info from station 2".

Pure standard library, so it works on the Linux control laptop and on Windows.
"""

import argparse
import json
import socket
import sys
import time
import uuid


DEFAULT_TEAM = "orel-reshka"
STATION_PORT = 45901
DEFAULT_REPLY_PORT = 45920
PING_BURSTS = 3
ATTEMPTS = 10
ATTEMPT_INTERVAL = 1.0
RECV_TIMEOUT = 1.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("station_ip", help="IP address of the station host")
    parser.add_argument("station_id", type=int, help="expected station_id (5 or 37)")
    parser.add_argument("--team", default=DEFAULT_TEAM, help="team name")
    parser.add_argument(
        "--reply-port",
        type=int,
        default=DEFAULT_REPLY_PORT,
        help="local UDP port to listen on for the STATION_INFO reply",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=ATTEMPTS,
        help="number of STATION_PING bursts before giving up",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    request_id = str(uuid.uuid4())
    ping = json.dumps({
        "team": args.team,
        "event": "STATION_PING",
        "uav": "control",
        "station": "any",
        "expected_station": args.station_id,
        "request_id": request_id,
        "reply_port": args.reply_port,
    }).encode("utf-8")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", args.reply_port))
        except OSError as error:
            print(
                "ERROR: не удалось привязать локальный UDP-порт {}: {}".format(
                    args.reply_port, error
                ),
                file=sys.stderr,
            )
            return 3
        sock.settimeout(RECV_TIMEOUT)

        deadline = time.monotonic() + args.attempts * ATTEMPT_INTERVAL
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            for _ in range(PING_BURSTS):
                sock.sendto(ping, (args.station_ip, STATION_PORT))
                time.sleep(0.05)

            sub_deadline = time.monotonic() + ATTEMPT_INTERVAL
            while time.monotonic() < sub_deadline:
                try:
                    data, address = sock.recvfrom(4096)
                except socket.timeout:
                    break
                try:
                    message = json.loads(data.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if not isinstance(message, dict):
                    continue
                if message.get("team") != args.team:
                    continue
                if message.get("event") != "STATION_INFO":
                    continue
                if str(message.get("request_id")) != request_id:
                    continue

                actual = int(message.get("station", -1))
                color = message.get("target_color")
                state = message.get("station_state")
                led_ok = message.get("status_led_ok")

                print(
                    "station {} @ {} -> id={} color={} state={} led_ok={}".format(
                        args.station_id,
                        args.station_ip,
                        actual,
                        color,
                        state,
                        led_ok,
                    )
                )

                problems = []
                if actual != args.station_id:
                    problems.append(
                        "station_id: ожидалась {}, получена {}".format(
                            args.station_id, actual
                        )
                    )
                if color != "red":
                    problems.append(
                        "target_color: ожидался red, получен {!r}".format(color)
                    )
                if state != "free":
                    problems.append(
                        "station_state: ожидался free, получен {!r}".format(state)
                    )
                if led_ok is not True:
                    problems.append(
                        "status_led_ok: ожидался True, получен {!r}".format(led_ok)
                    )
                if problems:
                    print(
                        "ERROR: станция {} ответила, но: {}".format(
                            args.station_id, "; ".join(problems)
                        ),
                        file=sys.stderr,
                    )
                    return 2
                return 0

        print(
            "ERROR: нет STATION_INFO от {} (станция {}) за {} попыток. "
            "Запущен ли station.py и готова ли его камера/ROS-топик?".format(
                args.station_ip, args.station_id, attempt
            ),
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
