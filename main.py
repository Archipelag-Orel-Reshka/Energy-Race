#!/usr/bin/env python3

import datetime
import json
import math
import os
import queue
import select
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config.json"


class MissionError(RuntimeError):
    pass


def load_config():
    with CONFIG_PATH.open(encoding="utf-8") as stream:
        config = json.load(stream)
    validate_config(config)
    return config


def validate_config(config):
    role = config.get("role")
    if role not in {"uav1", "uav2"}:
        raise MissionError("config.role must be uav1 or uav2")

    profile_name = config.get("active_profile")
    profiles = config.get("profiles", {})
    if profile_name not in profiles:
        raise MissionError(f"unknown active_profile: {profile_name!r}")

    profile = profiles[profile_name]
    points = profile.get(role, {})
    required = ["home", "station"] if role == "uav1" else [
        "home", "cargo", "station"
    ]
    for name in required:
        point = points.get(name)
        if not isinstance(point, dict):
            raise MissionError(f"missing coordinate: {profile_name}.{role}.{name}")
        for axis in ("x", "y"):
            value = point.get(axis)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise MissionError(
                    f"set numeric coordinate: {profile_name}.{role}.{name}.{axis}"
                )

    navigation = config.get("navigation", {})
    altitude = navigation.get("altitude")
    speed = navigation.get("speed")
    if not isinstance(altitude, (int, float)) or not 0.8 <= altitude <= 2.5:
        raise MissionError("navigation.altitude must be between 0.8 and 2.5 m")
    if not isinstance(speed, (int, float)) or not 0.1 <= speed <= 0.8:
        raise MissionError("navigation.speed must be between 0.1 and 0.8 m/s")

    timing = config.get("timing", {})
    charge = timing.get("charge_seconds", 0)
    green = timing.get("green_before_takeoff_seconds", 0)
    if charge < 10 or not 0 < green < charge:
        raise MissionError("charging timing is invalid")

    for name, gate in config.get("gates", {}).items():
        if gate.get("mode") not in {"none", "keyboard", "udp"}:
            raise MissionError(f"unsupported gate mode: {name}")

    if profile.get("platform") == "clover5":
        if not config.get("led", {}).get("required_before_takeoff"):
            raise MissionError(
                "set led.required_before_takeoff=true for the field profile"
            )
        network = config.get("network", {})
        if not network.get("station_ip"):
            raise MissionError("set network.station_ip for the field profile")
        if role == "uav1" and not network.get("uav2_ip"):
            raise MissionError("set network.uav2_ip for UAV1 coordination")


def print_plan(config):
    role = config["role"]
    profile_name = config["active_profile"]
    profile = config["profiles"][profile_name]
    print(f"role: {role}")
    print(f"profile: {profile_name} ({profile['platform']})")
    print(f"flight_enabled: {config['flight_enabled']}")
    for name, point in profile[role].items():
        print(f"{name}: x={point['x']:.2f}, y={point['y']:.2f}")
    print("gates:")
    for name, gate in config["gates"].items():
        print(f"  {name}: {gate['mode']} -> {gate['event']}")
    if profile["platform"] == "clover5" and role == "uav2":
        gpio = config["gripper"]["gpio"]
        print(
            "gripper: gpiozero "
            f"BCM{gpio['bcm_pin']} ({gpio['pin_factory']})"
        )


class EventLog:
    def __init__(self, config):
        directory = ROOT / config["logging"]["directory"]
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y%m%dT%H%M%SZ"
        )
        self.path = directory / f"{stamp}-{config['role']}.jsonl"
        self.stream = self.path.open("a", encoding="utf-8", buffering=1)
        self.state = "INIT"

    def set_state(self, state):
        self.state = state
        self.write("state_enter")

    def write(self, event, **data):
        record = {
            "time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "monotonic": round(time.monotonic(), 3),
            "state": self.state,
            "event": event,
            **data,
        }
        print(f"[{self.state}] {event}", flush=True)
        self.stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def close(self):
        if not self.stream.closed:
            self.stream.close()


class EventGate:
    def __init__(self, config, log):
        self.config = config
        self.log = log
        self.socket = None
        self.pending = set()

    def wait(self, gate_name):
        gate = self.config["gates"][gate_name]
        mode = gate["mode"]
        event = gate["event"]
        timeout = float(gate["timeout"])
        self.log.write("gate_wait", gate=gate_name, mode=mode, expected=event)

        if mode == "none":
            return
        if mode == "keyboard":
            self._wait_keyboard(event, timeout)
            return
        self._wait_udp(event, timeout)

    def _wait_keyboard(self, event, timeout):
        print(f"Press Enter for {event}", flush=True)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            raise MissionError(f"keyboard gate timeout: {event}")
        sys.stdin.readline()
        self.log.write("gate_received", event_name=event, source="keyboard")

    def _wait_udp(self, expected, timeout):
        if expected in self.pending:
            self.pending.remove(expected)
            self.log.write("gate_received", event_name=expected, source="udp-cache")
            return

        sock = self._udp_socket()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sock.settimeout(min(1.0, max(0.05, deadline - time.monotonic())))
            try:
                payload, address = sock.recvfrom(4096)
            except socket.timeout:
                continue

            event = self._parse_event(payload)
            if not event:
                continue
            self.log.write(
                "udp_received", event_name=event, sender=address[0]
            )
            if event == expected:
                self.log.write(
                    "gate_received", event_name=expected, source=address[0]
                )
                return
            self.pending.add(event)

        raise MissionError(f"UDP gate timeout: {expected}")

    def send(self, event, host, port):
        if not host:
            self.log.write("udp_skipped", event_name=event, reason="host is empty")
            return False
        payload = json.dumps({
            "event": event,
            "role": self.config["role"],
            "time": time.time(),
        }).encode()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for _ in range(3):
                sock.sendto(payload, (host, int(port)))
                time.sleep(0.05)
        self.log.write("udp_sent", event_name=event, host=host, port=port)
        return True

    def _udp_socket(self):
        if self.socket is None:
            network = self.config["network"]
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((
                network["listen_ip"],
                int(network["event_port"]),
            ))
            self.log.write(
                "udp_listening",
                host=network["listen_ip"],
                port=network["event_port"],
            )
        return self.socket

    @staticmethod
    def _parse_event(payload):
        text = payload.decode(errors="replace").strip()
        try:
            message = json.loads(text)
        except json.JSONDecodeError:
            return text
        if isinstance(message, dict):
            return str(message.get("event", "")).strip()
        return ""

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None


class Led:
    def __init__(self, drone, config, log):
        self.drone = drone
        self.config = config["led"]
        self.log = log
        self.client = drone.led
        self.warned = False

    def preflight(self):
        if self.client is None:
            try:
                from clover2.clients import LEDClient
                self.client = LEDClient(self.drone, "/led_strip")
            except Exception as error:
                self.log.write("led_unavailable", error=str(error))

        if self.client is None and self.config["required_before_takeoff"]:
            raise MissionError("LED driver is required but /led_strip is unavailable")

    def blink(self, r, g, b):
        self._call(
            "blink",
            r,
            g,
            b,
            period=float(self.config["blink_period"]),
            brightness=float(self.config["brightness"]),
        )

    def solid(self, r, g, b):
        self._call(
            "solid_color",
            r,
            g,
            b,
            brightness=float(self.config["brightness"]),
        )

    def half_red_blue(self):
        if self.client is None:
            self._missing()
            return
        try:
            count = int(self.client.led_count)
            split = count // 2
            colors = [(255, 0, 0)] * split + [(0, 0, 255)] * (count - split)
            self.client.send_frame(colors, float(self.config["brightness"]))
            self.log.write("led_set", mode="half_red_blue", count=count)
        except Exception as error:
            self.log.write("led_failed", mode="half_red_blue", error=str(error))

    def _call(self, method, *args, **kwargs):
        if self.client is None:
            self._missing()
            return
        try:
            getattr(self.client, method)(*args, **kwargs)
            self.log.write("led_set", mode=method, rgb=list(args[:3]))
        except Exception as error:
            self.log.write("led_failed", mode=method, error=str(error))

    def _missing(self):
        if not self.warned:
            self.log.write("led_skipped", reason="LED client unavailable")
            self.warned = True


class Gripper:
    def __init__(self, profile, config, log):
        self.config = config["gripper"]
        self.timing = config["timing"]
        self.log = log
        self.platform = profile["platform"]
        self.servo = None

        if not self.config["enabled"]:
            if self.config["required"]:
                raise MissionError("gripper is required but disabled")
            return

        if self.platform == "clover5":
            self._init_gpio()

    def _init_gpio(self):
        gpio = self.config["gpio"]
        os.environ.setdefault("GPIOZERO_PIN_FACTORY", gpio["pin_factory"])
        try:
            from gpiozero import Servo
            self.servo = Servo(
                int(gpio["bcm_pin"]),
                initial_value=None,
                min_pulse_width=float(gpio["min_pulse_width"]),
                max_pulse_width=float(gpio["max_pulse_width"]),
            )
        except Exception as error:
            if self.config["required"]:
                raise MissionError(f"GPIO servo initialization failed: {error}")
            self.log.write("gripper_unavailable", error=str(error))

    def open(self):
        self._set("open")

    def close(self):
        self._set("closed")

    def _set(self, position):
        if not self.config["enabled"]:
            self.log.write("gripper_skipped", reason="disabled")
            return

        if self.platform == "simulation":
            gazebo = self.config["gazebo"]
            value = gazebo[f"{position}_position"]
            subprocess.run([
                "gz", "topic",
                "-t", gazebo["topic"],
                "-m", "gz.msgs.Double",
                "-p", f"data: {float(value)}",
            ], check=True)
        else:
            if self.servo is None:
                raise MissionError("GPIO servo is unavailable")
            gpio = self.config["gpio"]
            self.servo.value = float(gpio[f"{position}_value"])

        time.sleep(float(self.timing["gripper_settle_seconds"]))
        if self.servo is not None:
            self.servo.detach()
        self.log.write("gripper_set", position=position)

    def cleanup(self):
        if self.servo is not None:
            self.servo.detach()
            self.servo.close()
            self.servo = None


class Mission:
    def __init__(self, config):
        self.config = config
        self.role = config["role"]
        self.profile = config["profiles"][config["active_profile"]]
        self.points = self.profile[self.role]
        self.navigation = config["navigation"]
        self.timing = config["timing"]
        self.log = EventLog(config)
        self.gate = EventGate(config, self.log)
        self.drone = None
        self.led = None
        self.gripper = None
        self.tf_buffer = None
        self.tf_time = None

    def connect(self, with_gripper=True):
        from clover2 import Clover2
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener

        self.drone = Clover2(f"energy_race_{self.role}")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self.drone, spin_thread=False
        )
        self.tf_time = Time
        self.led = Led(self.drone, self.config, self.log)
        if with_gripper and self.role == "uav2":
            self.gripper = Gripper(self.profile, self.config, self.log)

    def preflight(self):
        self.log.set_state("PREFLIGHT")
        timeout = float(self.navigation["startup_timeout"])
        action_client = self.drone._offboard._navigate_async_aclient
        if not action_client.wait_for_server(timeout_sec=timeout):
            raise MissionError("/fcu_bridge/navigate_async is unavailable")

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.drone.flight_mode():
            time.sleep(0.1)
        if not self.drone.flight_mode():
            raise MissionError("/fcu_bridge/state is unavailable")

        self.led.preflight()
        self.log.write(
            "preflight_ok",
            flight_mode=self.drone.flight_mode(),
            led_available=self.led.client is not None,
        )

    def run(self):
        if not self.config["flight_enabled"]:
            raise MissionError(
                "flight is locked; set flight_enabled=true after checking config"
            )
        self.connect(with_gripper=True)
        self.preflight()
        if self.role == "uav1":
            self.run_uav1()
        else:
            self.run_uav2()

    def run_smoke(self):
        if self.profile["platform"] != "simulation":
            raise MissionError("smoke flight is allowed only for simulation profile")
        self.connect(with_gripper=False)
        self.preflight()
        self.enter("SMOKE_TAKEOFF")
        self.led.blink(255, 255, 0)
        self.takeoff()
        time.sleep(float(self.navigation["takeoff_hold"]))
        self.enter("SMOKE_LAND")
        self.land()
        self.enter("DONE")

    def run_uav1(self):
        self.enter("WAIT_START")
        self.led.blink(255, 255, 0)
        self.gate.wait("start")

        self.enter("TAKEOFF_HOME")
        self.takeoff()
        self.wait_localization()

        self.enter("SEARCH_STATION")
        self.led.solid(255, 0, 0)
        self.goto(self.points["station"])
        self.notify_station("REQUEST_LAND")

        self.enter("WAIT_STATION_INVITATION")
        self.gate.wait("station_invitation")

        self.enter("LAND_STATION")
        self.land()

        self.charge()

        self.enter("WAIT_RETURN_COMMAND")
        self.gate.wait("uav1_return")
        self.notify_peer("UAV2_DEPART")

        self.enter("TAKEOFF_RETURN")
        self.led.blink(0, 255, 0)
        self.takeoff()
        self.wait_localization()

        self.enter("RETURN_HOME")
        self.goto(self.points["home"])

        self.enter("LAND_HOME")
        self.land()
        self.enter("DONE")

    def run_uav2(self):
        self.enter("WAIT_START")
        self.led.blink(255, 255, 0)
        self.gate.wait("start")

        self.enter("TAKEOFF_HOME")
        self.takeoff()
        self.wait_localization()

        self.enter("FLY_TO_CARGO")
        self.led.blink(255, 255, 0)
        self.goto(self.points["cargo"])

        self.enter("LAND_CARGO")
        self.land()

        self.enter("CAPTURE_CARGO")
        self.led.solid(255, 0, 0)
        self.gripper.close()

        self.enter("WAIT_UAV1_RETURN")
        self.gate.wait("uav2_depart")

        self.enter("TAKEOFF_WITH_CARGO")
        self.led.solid(255, 0, 0)
        self.takeoff()
        self.wait_localization()

        self.enter("FLY_TO_STATION")
        self.goto(self.points["station"])
        self.notify_station("REQUEST_LAND")

        self.enter("WAIT_STATION_INVITATION")
        self.gate.wait("station_invitation")

        self.enter("LAND_STATION")
        self.land()

        self.charge()

        self.enter("RELEASE_CARGO")
        self.led.blink(255, 0, 0)
        self.gripper.open()

        self.enter("TAKEOFF_RETURN")
        self.led.half_red_blue()
        self.takeoff()
        self.wait_localization()

        self.enter("RETURN_HOME")
        self.goto(self.points["home"])

        self.enter("LAND_HOME")
        self.land()
        self.enter("DONE")

    def enter(self, state):
        self.log.set_state(state)

    def takeoff(self):
        altitude = float(self.navigation["altitude"])
        speed = float(self.navigation["speed"])
        self.timed_call(
            "takeoff",
            lambda: self.drone.navigate_wait(
                "base_link", z=altitude, speed=speed
            ),
        )

    def goto(self, point):
        altitude = float(self.navigation["altitude"])
        speed = float(self.navigation["speed"])
        frame_id = self.navigation["frame_id"]
        self.log.write(
            "navigate_start",
            frame_id=frame_id,
            x=point["x"],
            y=point["y"],
            z=altitude,
        )
        self.timed_call(
            "navigate",
            lambda: self.drone.navigate_wait(
                frame_id,
                x=float(point["x"]),
                y=float(point["y"]),
                z=altitude,
                speed=speed,
            ),
        )
        self.log.write("navigate_done")

    def timed_call(self, name, callback):
        result = queue.Queue(maxsize=1)

        def worker():
            try:
                result.put((True, callback()))
            except BaseException as error:
                result.put((False, error))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        thread.join(float(self.navigation["navigation_timeout"]))
        if thread.is_alive():
            self.log.write("operation_timeout", operation=name)
            if self.drone.is_armed():
                self.drone.land()
            thread.join(3.0)
            raise MissionError(f"{name} timeout")

        ok, value = result.get_nowait()
        if not ok:
            raise MissionError(f"{name} failed: {value}")
        if value is False:
            raise MissionError(f"{name} returned false")

    def wait_localization(self):
        self.log.write("localization_wait")
        deadline = time.monotonic() + float(
            self.navigation["localization_timeout"]
        )
        previous = None
        stable = 0
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.navigation["frame_id"],
                    "base_link",
                    self.tf_time(),
                )
                current = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                )
                if previous is not None:
                    distance = math.dist(previous, current)
                    stable = stable + 1 if distance < 0.25 else 0
                previous = current
                if stable >= 3:
                    self.log.write(
                        "localization_ok",
                        x=round(current[0], 3),
                        y=round(current[1], 3),
                        z=round(current[2], 3),
                    )
                    return
            except Exception:
                stable = 0
            time.sleep(0.2)

        raise MissionError("map -> base_link localization unavailable")

    def land(self):
        self.log.write("land_command")
        if not self.drone.land():
            self.log.write("land_service_returned_false")

        deadline = time.monotonic() + float(self.navigation["landing_timeout"])
        disarmed_samples = 0
        while time.monotonic() < deadline:
            if self.drone.is_armed():
                disarmed_samples = 0
            else:
                disarmed_samples += 1
                if disarmed_samples >= 5:
                    self.log.write("landed_disarmed")
                    return
            time.sleep(0.2)
        raise MissionError("landing timeout: motors are still armed")

    def charge(self):
        self.enter("CHARGING_RED")
        self.led.blink(255, 0, 0)
        total = float(self.timing["charge_seconds"])
        green = float(self.timing["green_before_takeoff_seconds"])
        deadline = time.monotonic() + total
        green_at = deadline - green
        while time.monotonic() < green_at:
            time.sleep(min(0.1, green_at - time.monotonic()))

        self.enter("CHARGING_GREEN")
        self.led.solid(0, 255, 0)
        while time.monotonic() < deadline:
            time.sleep(min(0.1, deadline - time.monotonic()))
        self.log.write("charging_done", seconds=total)

    def notify_station(self, event):
        network = self.config["network"]
        self.gate.send(event, network["station_ip"], network["station_port"])

    def notify_peer(self, event):
        network = self.config["network"]
        peer_key = "uav2_ip" if self.role == "uav1" else "uav1_ip"
        self.gate.send(event, network[peer_key], network["event_port"])

    def safe_land(self):
        if self.drone is None or not self.drone.is_armed():
            return
        self.log.write("recovery_land")
        try:
            self.drone.land()
        except Exception as error:
            self.log.write("recovery_land_failed", error=str(error))

    def close(self):
        if self.gripper is not None:
            self.gripper.cleanup()
        self.gate.close()
        self.log.close()


def main():
    config = load_config()
    mode = os.environ.get("ENERGY_RACE_MODE", "mission")
    if mode == "check":
        print_plan(config)
        return 0

    mission = Mission(config)
    try:
        if mode == "smoke":
            mission.run_smoke()
        elif mode == "mission":
            mission.run()
        else:
            raise MissionError(f"unsupported mode: {mode}")
        print(f"Mission completed. Log: {mission.log.path}")
        return 0
    except KeyboardInterrupt:
        mission.log.write("interrupted")
        mission.safe_land()
        return 130
    except Exception as error:
        mission.log.write("mission_failed", error=str(error))
        mission.safe_land()
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        mission.close()


if __name__ == "__main__":
    raise SystemExit(main())
