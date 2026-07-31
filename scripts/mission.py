#!/usr/bin/env python3

import datetime
import json
import math
import shutil
import socket
import subprocess
import time
import uuid
from pathlib import Path

import rospy
from aruco_pose.msg import MarkerArray
from std_srvs.srv import Trigger
from technic.srv import GetTelemetry, Navigate, SetLEDEffect


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "mission_config.json"


def load_config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def marker_position(marker_id):
    row, column = divmod(int(marker_id), 7)
    return float(column), float(6 - row)


class MissionLog:
    def __init__(self, role):
        self.role = role
        self.state = "INIT"
        self.path = ROOT / "mission-{}.jsonl".format(role)

    def enter(self, state):
        self.state = state
        self.write("state_enter")

    def write(self, event, **data):
        record = {
            "time": datetime.datetime.now().isoformat(),
            "monotonic": round(time.monotonic(), 3),
            "role": self.role,
            "state": self.state,
            "event": event,
        }
        record.update(data)
        line = json.dumps(record, ensure_ascii=False)
        print(line, flush=True)
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")


class UdpBus:
    def __init__(self, config, role, log):
        self.config = config
        self.role = role
        self.log = log
        self.pending = []
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("0.0.0.0", int(config["event_port"])))

    def send(self, host, port, event, **extra):
        message = {
            "team": self.config["team"],
            "event": event,
            "uav": self.role,
        }
        message.update(extra)
        encoded = json.dumps(message).encode("utf-8")
        for _ in range(5):
            self.socket.sendto(encoded, (host, int(port)))
            time.sleep(0.05)
        self.log.write("udp_sent", message=event, destination=host)

    def wait(self, event, timeout, request_id=None):
        cached = self._take_pending(event, request_id)
        if cached is not None:
            return cached

        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.socket.settimeout(
                min(1.0, max(0.05, deadline - time.monotonic()))
            )
            try:
                payload, address = self.socket.recvfrom(4096)
            except socket.timeout:
                continue

            message = self._parse(payload)
            if message is None:
                continue
            if message.get("target") not in (None, "all", self.role):
                continue
            self.log.write(
                "udp_received",
                message=message.get("event"),
                source=address[0],
            )
            if message.get("event") == "STATION_BUSY":
                raise RuntimeError("зарядная станция занята")
            if self._matches(message, event, request_id):
                return message
            self.pending.append(message)

        raise RuntimeError("timeout waiting for {}".format(event))

    def status(self, state, **extra):
        self.send(
            self.config["control_ip"],
            self.config["control_port"],
            "STATUS",
            state=state,
            **extra
        )

    def _take_pending(self, event, request_id):
        for index, message in enumerate(self.pending):
            if self._matches(message, event, request_id):
                return self.pending.pop(index)
        return None

    @staticmethod
    def _matches(message, event, request_id):
        if message.get("event") != event:
            return False
        if request_id is None:
            return True
        return str(message.get("request_id")) == str(request_id)

    def _parse(self, payload):
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        if not isinstance(message, dict):
            return None
        if message.get("team") != self.config["team"]:
            return None
        return message

    def close(self):
        self.socket.close()


class Servo:
    def __init__(self, config, log):
        self.config = config
        self.log = log
        self.enabled = bool(config["enabled"])
        self.command = str(config.get("command", "gpio"))
        self.pin = int(config["pin"])
        self.closed_angle = float(config["closed_angle"])
        self.open_angle = float(config["open_angle"])
        self.pwm_min = int(config.get("pwm_min", 50))
        self.pwm_span = int(config.get("pwm_span", 300))
        self.move_seconds = float(config["move_seconds"])
        self.initialized = False

    def preflight(self):
        if not self.enabled:
            self.log.write("servo_disabled")
            return
        if shutil.which(self.command) is None:
            raise RuntimeError(
                "gpio command not found: {}".format(self.command)
            )
        if self.pin < 0:
            raise RuntimeError("servo.pin должен быть >= 0")
        for name, angle in (
            ("closed_angle", self.closed_angle),
            ("open_angle", self.open_angle),
        ):
            if not 0.0 <= angle <= 180.0:
                raise RuntimeError("servo.{} должен быть 0..180".format(name))
        if self.pwm_min < 0 or self.pwm_span <= 0:
            raise RuntimeError("servo pwm_min/pwm_span заданы неверно")
        if self.move_seconds <= 0.0:
            raise RuntimeError("servo.move_seconds должен быть > 0")

        self._run_gpio("mode", self.pin, "pwm")
        self.initialized = True
        open_pwm = self._set_angle(self.open_angle)
        time.sleep(self.move_seconds)
        self._disable_signal()
        self.log.write(
            "servo_preflight_ok",
            pin=self.pin,
            open_angle=self.open_angle,
            open_pwm=open_pwm,
        )

    def close_grip(self):
        if not self.enabled:
            self.log.write("servo_close_skipped")
            return
        self._require_initialized()
        pwm_value = self._set_angle(self.closed_angle)
        time.sleep(self.move_seconds)
        self.log.write(
            "servo_closed",
            pin=self.pin,
            angle=self.closed_angle,
            pwm=pwm_value,
        )

    def open_grip(self):
        if not self.enabled:
            self.log.write("servo_open_skipped")
            return
        self._require_initialized()
        pwm_value = self._set_angle(self.open_angle)
        time.sleep(self.move_seconds)
        self._disable_signal()
        self.log.write(
            "servo_opened",
            pin=self.pin,
            angle=self.open_angle,
            pwm=pwm_value,
        )

    def release(self):
        if not self.enabled or not self.initialized:
            return
        try:
            self._disable_signal()
        except Exception as error:
            self.log.write("servo_release_failed", error=str(error))
        finally:
            self.initialized = False

    def angle_to_pwm(self, angle):
        actual_angle = max(0.0, min(180.0, float(angle)))
        return int(
            self.pwm_min + (actual_angle / 180.0) * self.pwm_span
        )

    def _set_angle(self, angle):
        pwm_value = self.angle_to_pwm(angle)
        self._run_gpio("pwm", self.pin, pwm_value)
        return pwm_value

    def _disable_signal(self):
        self._run_gpio("pwm", self.pin, 0)

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError("servo не прошёл preflight")

    def _run_gpio(self, *arguments):
        command = [self.command] + [str(value) for value in arguments]
        try:
            subprocess.run(
                command,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            raise RuntimeError("gpio command not found: {}".format(
                self.command
            ))
        except subprocess.CalledProcessError as error:
            detail = (error.stderr or error.stdout or "").strip()
            raise RuntimeError(
                "gpio command failed: {}{}".format(
                    " ".join(command),
                    ": {}".format(detail) if detail else "",
                )
            )


class Mission:
    def __init__(self, config, role):
        self.config = config
        if role not in config["roles"]:
            raise RuntimeError("неизвестная роль {}".format(role))
        self.role = role
        self.role_config = config["roles"][self.role]
        self.navigation = config["navigation"]
        self.timing = config["timing"]
        self.cruise_altitude = float(self.role_config.get(
            "cruise_altitude",
            self.navigation["cruise_altitude"],
        ))
        self.station_mode = str(self.role_config.get(
            "station_mode",
            "real" if self.role_config.get("station_ip") else "virtual",
        ))
        self.network = dict(config["network"])
        self.network["team"] = config["team"]
        self.log = MissionLog(self.role)
        self.bus = UdpBus(self.network, self.role, self.log)
        self.servo = Servo(config["servo"], self.log)
        self.visible_markers = set()
        self.current_marker = int(self.role_config["home_marker"])
        self.flight_active = False
        self.station_request_id = None
        self.route_clear_sent = False

        rospy.init_node(
            "energy_race_{}".format(self.role),
            disable_signals=True,
        )
        rospy.Subscriber(
            "aruco_detect/markers", MarkerArray, self._markers_callback
        )
        for service in (
            "get_telemetry", "navigate", "land", "led/set_effect"
        ):
            rospy.loginfo("waiting for %s", service)
            rospy.wait_for_service(service, timeout=10)
        self.get_telemetry = rospy.ServiceProxy(
            "get_telemetry", GetTelemetry
        )
        self.navigate = rospy.ServiceProxy("navigate", Navigate)
        self.land_service = rospy.ServiceProxy("land", Trigger)
        self.set_effect = rospy.ServiceProxy(
            "led/set_effect", SetLEDEffect
        )

    def preflight(self):
        telemetry = self.get_telemetry()
        if hasattr(telemetry, "connected") and not telemetry.connected:
            raise RuntimeError("PX4 is not connected")
        if telemetry.armed:
            raise RuntimeError("дрон уже armed до старта миссии")
        station_ip = self.role_config.get("station_ip")
        if self.station_mode not in ("real", "virtual"):
            raise RuntimeError(
                "station_mode должен быть real или virtual для {}".format(
                    self.role
                )
            )
        if self.station_mode == "real":
            if not station_ip or station_ip.startswith("SET_"):
                raise RuntimeError(
                    "укажи station_ip для {}".format(self.role)
                )
            self.preflight_station(station_ip)
        if self.role == "uav2":
            self.servo.preflight()
        self.log.write(
            "preflight_ok",
            hostname=socket.gethostname(),
            station=self.role_config["station_marker"],
            station_ip=station_ip or "virtual",
            station_mode=self.station_mode,
            cruise_altitude=self.cruise_altitude,
        )

    def run(self):
        self.preflight()
        if self.role == "uav1":
            self.run_uav1()
        else:
            self.run_uav2()

    def run_uav1(self):
        self.enter("WAIT_START")
        self.led("blink", 255, 255, 0)
        self.wait_start()

        self.enter("TAKEOFF_YELLOW")
        self.takeoff(self.cruise_altitude)
        self.wait_any_marker()
        self.bus.status("TAKEOFF_DONE")

        self.enter("SEARCH_STATION_RED")
        self.led("fill", 255, 0, 0)
        self.goto_marker(self.role_config["station_marker"])
        self.center_on_station()

        self.await_station_permission()

        self.enter("LAND_STATION")
        self.land()
        self.notify_station("LANDED")
        self.bus.status("STATION_LANDED")

        self.charge()
        self.bus.status("CHARGE_DONE")

        self.enter("WAIT_OPERATOR_FLY")
        self.bus.wait("RETURN_HOME", self.timing["operator_timeout"])

        self.enter("RETURN_HOME_GREEN")
        self.led("blink", 0, 255, 0)
        self.takeoff(self.navigation["station_departure_height"])
        self.notify_station("STATION_RELEASED")
        self.goto_marker(self.role_config["home_marker"])

        self.enter("LAND_HOME")
        self.land()
        self.bus.status("DONE")
        self.enter("DONE")

    def run_uav2(self):
        self.enter("WAIT_START")
        self.led("blink", 255, 255, 0)
        self.wait_start()

        self.enter("TAKEOFF_YELLOW")
        self.takeoff(self.cruise_altitude)
        self.wait_any_marker()

        self.enter("FLY_TO_CARGO_YELLOW")
        self.led("blink", 255, 255, 0)
        self.goto_marker(self.role_config["cargo_marker"])

        self.enter("LAND_CARGO")
        self.land()
        self.led("fill", 255, 0, 0)
        self.bus.status("CARGO_LANDED")

        self.enter("WAIT_CARGO_LOADED")
        self.bus.wait("CARGO_LOADED", self.timing["operator_timeout"])

        self.enter("CAPTURE_CARGO_RED")
        self.led("fill", 255, 0, 0)
        self.servo.close_grip()
        self.bus.status("CARGO_READY")

        self.enter("WAIT_UAV1_CHARGED")
        self.bus.wait("UAV2_DEPART", self.timing["operator_timeout"])

        self.enter("FLY_TO_STATION_RED")
        self.takeoff(self.cruise_altitude)
        self.wait_any_marker()
        self.goto_marker(self.role_config["station_marker"])
        self.center_on_station()

        self.await_station_permission()

        self.enter("LAND_STATION_WITH_CARGO")
        self.land()
        self.notify_station("LANDED")
        self.bus.status("STATION_LANDED")

        self.charge()

        self.enter("RELEASE_CARGO_RED_BLINK")
        self.led("blink", 255, 0, 0)
        self.servo.open_grip()

        self.enter("RETURN_HOME_HALF_RED_BLUE")
        self.half_red_blue()
        self.takeoff(self.navigation["station_departure_height"])
        self.notify_station("STATION_RELEASED")
        self.goto_marker(self.role_config["home_marker"])

        self.enter("LAND_HOME")
        self.land()
        self.bus.status("DONE")
        self.enter("DONE")

    def enter(self, state):
        self.log.enter(state)

    def wait_start(self):
        while not rospy.is_shutdown():
            self.bus.status("READY")
            try:
                self.bus.wait("START", 2.0)
                return
            except RuntimeError as error:
                if str(error) != "timeout waiting for START":
                    raise
        raise RuntimeError("ROS shutdown while waiting for START")

    def _markers_callback(self, message):
        self.visible_markers = {marker.id for marker in message.markers}

    def wait_any_marker(self):
        deadline = time.monotonic() + float(
            self.navigation["marker_timeout"]
        )
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.visible_markers:
                self.log.write(
                    "aruco_visible",
                    markers=sorted(self.visible_markers),
                )
                return
            rospy.sleep(0.1)
        raise RuntimeError("ArUco map is not detected")

    def led(self, effect, red, green, blue):
        response = self.set_effect(
            effect=effect, r=red, g=green, b=blue
        )
        if hasattr(response, "success") and not response.success:
            raise RuntimeError("LED rejected: {}".format(response.message))
        self.log.write(
            "led_set", effect=effect, r=red, g=green, b=blue
        )

    def half_red_blue(self):
        from led_msgs.msg import LEDState, LEDStateArray
        from led_msgs.srv import SetLEDs

        rospy.wait_for_service("led/set_leds", timeout=5)
        state = rospy.wait_for_message(
            "led/state", LEDStateArray, timeout=5
        )
        middle = len(state.leds) // 2
        colors = []
        for position, current in enumerate(state.leds):
            if position < middle:
                colors.append(LEDState(current.index, 255, 0, 0))
            else:
                colors.append(LEDState(current.index, 0, 0, 255))
        rospy.ServiceProxy("led/set_leds", SetLEDs)(colors)
        self.log.write("led_half_red_blue", count=len(colors))

    def takeoff(self, height):
        self.flight_active = True
        self.navigate_wait(
            x=0.0,
            y=0.0,
            z=float(height),
            frame_id="body",
            auto_arm=True,
        )

    def goto_marker(self, marker_id):
        marker_id = int(marker_id)
        current_x, current_y = marker_position(self.current_marker)
        target_x, target_y = marker_position(marker_id)
        self.log.write(
            "goto_marker",
            marker=marker_id,
            x=target_x,
            y=target_y,
            z=self.cruise_altitude,
        )

        while current_x != target_x or current_y != target_y:
            if current_x != target_x:
                current_x += 1.0 if target_x > current_x else -1.0
            else:
                current_y += 1.0 if target_y > current_y else -1.0

            waypoint_marker = int((6.0 - current_y) * 7 + current_x)
            self.log.write(
                "route_leg",
                marker=waypoint_marker,
                x=current_x,
                y=current_y,
            )
            self.navigate_wait(
                x=current_x,
                y=current_y,
                z=self.cruise_altitude,
                frame_id="aruco_map",
            )
            route_clear_marker = self.role_config.get("route_clear_marker")
            if (
                not self.route_clear_sent
                and route_clear_marker is not None
                and waypoint_marker == int(route_clear_marker)
            ):
                self.route_clear_sent = True
                self.bus.status("ROUTE_CLEAR")
                self.log.write("route_clear", marker=waypoint_marker)

        self.current_marker = marker_id

    def center_on_station(self):
        station_id = int(self.role_config["station_marker"])
        target_x, target_y = marker_position(station_id)
        self.enter("CENTER_ABOVE_STATION")
        self.navigate_wait(
            x=target_x,
            y=target_y,
            z=self.cruise_altitude,
            frame_id="aruco_map",
            arrival_tolerance=float(
                self.navigation["station_arrival_tolerance"]
            ),
        )

        hold_seconds = float(self.timing["station_hold_seconds"])
        deadline = time.monotonic() + hold_seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown while centering on station")
        self.log.write(
            "station_centered",
            station=station_id,
            altitude=self.cruise_altitude,
            hold_seconds=hold_seconds,
        )

    def await_station_permission(self):
        if self.station_mode == "real":
            self.enter("WAIT_STATION_RED_DETECTION")
            self.request_landing()
            return

        self.enter("VIRTUAL_STATION")
        self.log.write(
            "virtual_station_no_invitation",
            marker=int(self.role_config["station_marker"]),
        )

    def preflight_station(self, station_ip):
        expected_station = int(self.role_config["station_marker"])
        request_id = str(uuid.uuid4())
        self.bus.send(
            station_ip,
            45901,
            "STATION_PING",
            station="any",
            expected_station=expected_station,
            request_id=request_id,
            reply_port=self.network["event_port"],
        )
        response = self.bus.wait(
            "STATION_INFO",
            self.timing["station_preflight_timeout"],
            request_id=request_id,
        )
        actual_station = int(response.get("station", -1))
        if actual_station != expected_station:
            raise RuntimeError(
                "на IP {} запущена станция {}, ожидалась {}".format(
                    station_ip,
                    actual_station,
                    expected_station,
                )
            )
        if response.get("target_color") != "red":
            raise RuntimeError(
                "станция {} настроена не на red".format(actual_station)
            )
        station_state = response.get("station_state")
        if station_state != "free":
            raise RuntimeError(
                "станция {} не свободна: {}".format(
                    actual_station,
                    station_state or "unknown",
                )
            )
        self.log.write(
            "station_preflight_ok",
            station=actual_station,
            station_ip=station_ip,
            target_color=response.get("target_color"),
            station_state=station_state,
        )

    def navigate_wait(
        self,
        x,
        y,
        z,
        frame_id,
        auto_arm=False,
        arrival_tolerance=None,
    ):
        self.navigate(
            x=x,
            y=y,
            z=z,
            yaw=float("nan"),
            speed=float(self.navigation["speed"]),
            frame_id=frame_id,
            auto_arm=auto_arm,
        )
        deadline = time.monotonic() + float(
            self.navigation["navigation_timeout"]
        )
        tolerance = (
            float(self.navigation["arrival_tolerance"])
            if arrival_tolerance is None
            else float(arrival_tolerance)
        )
        while time.monotonic() < deadline and not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id="navigate_target")
            distance = math.sqrt(
                telemetry.x ** 2
                + telemetry.y ** 2
                + telemetry.z ** 2
            )
            if (
                math.isfinite(distance)
                and distance < tolerance
            ):
                self.log.write(
                    "navigate_arrived",
                    distance=round(distance, 3),
                    tolerance=tolerance,
                )
                return
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown during navigation")
        raise RuntimeError("navigation timeout")

    def land(self):
        started = time.monotonic()
        deadline = started + float(self.navigation["landing_timeout"])
        retry_seconds = float(self.navigation["landing_retry_seconds"])
        next_retry = started
        attempt = 0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            now = time.monotonic()
            if now >= next_retry:
                attempt += 1
                self.land_service()
                self.log.write(
                    "landing_command",
                    attempt=attempt,
                    elapsed=round(now - started, 1),
                )
                next_retry = now + retry_seconds

            if not self.get_telemetry().armed:
                self.flight_active = False
                self.log.write("landed_disarmed", attempts=attempt)
                return
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown during landing")
        raise RuntimeError(
            "landing timeout after {} attempts".format(attempt)
        )

    def request_landing(self):
        station_id = int(self.role_config["station_marker"])
        self.station_request_id = str(uuid.uuid4())
        self.bus.send(
            self.role_config["station_ip"],
            45901,
            "REQUEST_LAND",
            station=station_id,
            request_id=self.station_request_id,
            reply_port=self.network["event_port"],
            led="red",
        )
        invitation = self.bus.wait(
            "LAND_GRANTED",
            self.timing["station_invitation_timeout"],
            request_id=self.station_request_id,
        )
        if int(invitation.get("station", -1)) != station_id:
            raise RuntimeError("приглашение пришло от другой станции")
        if invitation.get("detected_color") != "red":
            raise RuntimeError("станция не подтвердила красную ленту")
        self.log.write(
            "landing_granted",
            station=station_id,
            detected_color=invitation.get("detected_color"),
            score=invitation.get("score"),
        )

    def notify_station(self, event):
        if self.station_mode != "real":
            self.log.write(
                "station_notification_skipped",
                message=event,
                station=int(self.role_config["station_marker"]),
            )
            return
        self.bus.send(
            self.role_config["station_ip"],
            45901,
            event,
            station=int(self.role_config["station_marker"]),
            request_id=self.station_request_id,
        )

    def charge(self):
        total = float(self.timing["charge_seconds"])
        green = float(self.timing["green_seconds"])
        red_until = time.monotonic() + total - green
        self.enter("CHARGING_RED_BLINK")
        self.led("blink", 255, 0, 0)
        while time.monotonic() < red_until:
            rospy.sleep(0.1)

        self.enter("CHARGING_GREEN")
        self.led("fill", 0, 255, 0)
        deadline = red_until + green
        while time.monotonic() < deadline:
            rospy.sleep(0.1)
        self.log.write("charging_done", seconds=total)

    def safe_land(self):
        if not self.flight_active:
            return
        self.log.write("safe_land")
        try:
            self.land()
        except Exception as error:
            self.log.write("safe_land_failed", error=str(error))

    def close(self):
        self.servo.release()
        self.bus.close()


def main(role=None):
    if role is None:
        raise SystemExit("Запусти uav1.py или uav2.py, не mission.py")
    config = load_config()
    mission = Mission(config, role)
    try:
        mission.run()
        return 0
    except KeyboardInterrupt:
        mission.log.write("interrupted")
        try:
            mission.bus.status("INTERRUPTED")
        except Exception:
            pass
        mission.safe_land()
        return 130
    except Exception as error:
        mission.log.write("mission_failed", error=str(error))
        try:
            mission.bus.status("FAILED", error=str(error))
        except Exception:
            pass
        mission.safe_land()
        print("ERROR: {}".format(error), flush=True)
        return 1
    finally:
        mission.close()


if __name__ == "__main__":
    raise SystemExit(main())
