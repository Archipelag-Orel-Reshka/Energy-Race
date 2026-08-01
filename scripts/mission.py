#!/usr/bin/env python3

import datetime
import json
import math
import signal
import shutil
import socket
import subprocess
import time
import uuid
from pathlib import Path

import rospy
from aruco_pose.msg import MarkerArray
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger
from technic.srv import GetTelemetry, Navigate, SetLEDEffect


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "mission_config.json"


class StationDetectionDenied(RuntimeError):
    pass


def load_config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def handle_termination_signal(signum, _frame):
    try:
        print(
            "WARNING: получен {}; запускаю безопасную посадку".format(
                signal.Signals(signum).name
            ),
            flush=True,
        )
    except (BrokenPipeError, OSError):
        pass
    raise KeyboardInterrupt


def install_termination_handlers():
    signal.signal(signal.SIGHUP, handle_termination_signal)
    signal.signal(signal.SIGTERM, handle_termination_signal)


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
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")
        try:
            print(line, flush=True)
        except (BrokenPipeError, OSError):
            pass


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
            if message.get("event") == "LAND_DENIED":
                denial_matches = self._matches(
                    message, "LAND_DENIED", request_id
                )
                if event == "LAND_GRANTED" and denial_matches:
                    raise StationDetectionDenied(
                        "станция не распознала LED до таймаута"
                    )
                self.log.write(
                    "udp_ignored",
                    message="LAND_DENIED",
                    source=address[0],
                    waiting_for=event,
                    reason="stale_station_reply",
                )
                continue
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
            return False
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

        self.log.write(
            "servo_preflight_ok",
            pin=self.pin,
            movement_tested=False,
        )
        return True

    def close_grip(self):
        if not self.enabled:
            self.log.write("servo_close_skipped")
            return False
        self._ensure_initialized()
        pwm_value = self._set_angle(self.closed_angle)
        time.sleep(self.move_seconds)
        self.log.write(
            "servo_closed",
            pin=self.pin,
            angle=self.closed_angle,
            pwm=pwm_value,
        )
        return True

    def open_grip(self):
        if not self.enabled:
            self.log.write("servo_open_skipped")
            return False
        self._ensure_initialized()
        pwm_value = self._set_angle(self.open_angle)
        time.sleep(self.move_seconds)
        self._disable_signal()
        self.log.write(
            "servo_opened",
            pin=self.pin,
            angle=self.open_angle,
            pwm=pwm_value,
        )
        return True

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

    def _ensure_initialized(self):
        if self.initialized:
            return
        self._run_gpio("mode", self.pin, "pwm")
        self.initialized = True
        self.log.write("servo_pwm_initialized", pin=self.pin)

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


class UnavailableServo:
    def __init__(self, log, error):
        self.log = log
        self.error = str(error)
        self.log.write(
            "servo_config_warning",
            error=self.error,
            mission_continues=True,
        )

    def preflight(self):
        self.log.write(
            "servo_unavailable",
            error=self.error,
            mission_continues=True,
        )
        return False

    def close_grip(self):
        self.log.write("servo_close_skipped", error=self.error)
        return False

    def open_grip(self):
        self.log.write("servo_open_skipped", error=self.error)
        return False

    def release(self):
        pass


def create_servo(config, log):
    try:
        return Servo(config, log)
    except Exception as error:
        return UnavailableServo(log, error)


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
        self.station_detection_altitude = float(self.navigation.get(
            "station_detection_altitude",
            self.cruise_altitude,
        ))
        if not 0.5 <= self.station_detection_altitude <= self.cruise_altitude:
            raise RuntimeError(
                "station_detection_altitude должен быть в диапазоне "
                "0.5..cruise_altitude"
            )
        self.return_altitude = float(self.navigation.get(
            "return_altitude",
            self.cruise_altitude,
        ))
        if self.return_altitude < self.cruise_altitude:
            raise RuntimeError(
                "return_altitude должен быть не ниже cruise_altitude"
            )
        self.station_mode = str(self.role_config.get(
            "station_mode",
            "real" if self.role_config.get("station_ip") else "virtual",
        ))
        self.led_count = int(config.get("led", {}).get("count", 72))
        if self.led_count <= 0 or self.led_count % 2:
            raise RuntimeError("число LED должно быть положительным и чётным")
        self.network = dict(config["network"])
        self.network["team"] = config["team"]
        self.log = MissionLog(self.role)
        self.bus = UdpBus(self.network, self.role, self.log)
        self.servo = create_servo(config.get("servo", {}), self.log)
        self.servo_available = None
        self.visible_markers = set()
        self.current_marker = int(self.role_config["home_marker"])
        self.flight_active = False
        self.station_request_id = None
        self.extended_landed_state = None
        self.extended_state_updated = None

        rospy.init_node(
            "energy_race_{}".format(self.role),
            disable_signals=True,
        )
        rospy.Subscriber(
            "aruco_detect/markers", MarkerArray, self._markers_callback
        )
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_callback,
        )
        for service in (
            "get_telemetry",
            "navigate",
            "land",
            "led/set_effect",
            "/mavros/cmd/arming",
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
        self.arming_service = rospy.ServiceProxy(
            "/mavros/cmd/arming", CommandBool
        )

    def preflight(self):
        telemetry = self.get_telemetry()
        if hasattr(telemetry, "connected") and not telemetry.connected:
            raise RuntimeError("PX4 is not connected")
        if telemetry.armed:
            raise RuntimeError("дрон уже armed до старта миссии")
        extended_state = rospy.wait_for_message(
            "/mavros/extended_state",
            ExtendedState,
            timeout=5,
        )
        self._extended_state_callback(extended_state)
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
            self.verify_role_ip(station_ip)
            self.preflight_station(station_ip)
        if self.role == "uav2":
            try:
                self.servo_available = bool(self.servo.preflight())
            except Exception as error:
                self.servo_available = False
                self.log.write(
                    "servo_preflight_warning",
                    error=str(error),
                    mission_continues=True,
                )
                self.servo.release()
        self.log.write(
            "preflight_ok",
            hostname=socket.gethostname(),
            station=self.role_config["station_marker"],
            station_ip=station_ip or "virtual",
            station_mode=self.station_mode,
            cruise_altitude=self.cruise_altitude,
            return_altitude=self.return_altitude,
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

        self.hold_before_station_route()

        self.enter("SEARCH_STATION_RED")
        self.led("fill", 255, 0, 0)
        self.goto_marker(self.role_config["station_marker"])
        self.center_on_station()

        permission_granted = self.await_station_permission()
        self.stabilize_after_land_grant(permission_granted)

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
        self.goto_marker(
            self.role_config["home_marker"],
            altitude=self.return_altitude,
        )

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
        servo_ok = self.try_servo_action("close_grip")
        self.bus.status("CARGO_READY", servo_ok=servo_ok)

        self.enter("WAIT_UAV1_CHARGED")
        self.bus.wait("UAV2_DEPART", self.timing["operator_timeout"])

        self.enter("FLY_TO_STATION_RED")
        self.takeoff(self.cruise_altitude)
        self.wait_any_marker()
        self.goto_marker(self.role_config["station_marker"])
        self.center_on_station()

        permission_granted = self.await_station_permission()
        self.stabilize_after_land_grant(permission_granted)

        self.enter("LAND_STATION_WITH_CARGO")
        self.land()
        self.notify_station("LANDED")
        self.bus.status("STATION_LANDED")

        self.charge()

        self.enter("RELEASE_CARGO_RED_BLINK")
        self.led("blink", 255, 0, 0)
        self.try_servo_action("open_grip")

        self.enter("RETURN_HOME_HALF_RED_BLUE")
        self.half_red_blue()
        self.takeoff(self.navigation["station_departure_height"])
        self.notify_station("STATION_RELEASED")
        self.goto_marker(
            self.role_config["home_marker"],
            altitude=self.return_altitude,
        )

        self.enter("LAND_HOME")
        self.land()
        self.bus.status("DONE")
        self.enter("DONE")

    def enter(self, state):
        self.log.enter(state)

    def wait_start(self):
        while not rospy.is_shutdown():
            if self.role == "uav2":
                self.bus.status(
                    "READY",
                    servo_ok=bool(self.servo_available),
                )
            else:
                self.bus.status("READY")
            try:
                self.bus.wait("START", 2.0)
                return
            except RuntimeError as error:
                if str(error) != "timeout waiting for START":
                    raise
        raise RuntimeError("ROS shutdown while waiting for START")

    def try_servo_action(self, action):
        try:
            success = bool(getattr(self.servo, action)())
            self.servo_available = success
            self.log.write(
                "servo_action_result",
                action=action,
                success=success,
                mission_continues=True,
            )
            return success
        except Exception as error:
            self.servo_available = False
            self.log.write(
                "servo_action_warning",
                action=action,
                error=str(error),
                mission_continues=True,
            )
            self.servo.release()
            return False

    def hold_before_station_route(self):
        delay = float(self.timing["uav1_route_delay"])
        self.enter("HOLD_BEFORE_STATION_YELLOW")
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown during UAV1 route delay")
        self.log.write("uav1_route_delay_done", seconds=delay)

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
        from led_msgs.msg import LEDState
        from led_msgs.srv import SetLEDs

        rospy.wait_for_service("led/set_leds", timeout=5)
        middle = self.led_count // 2
        colors = [
            LEDState(index, 255, 0, 0)
            if index < middle
            else LEDState(index, 0, 0, 255)
            for index in range(self.led_count)
        ]
        rospy.ServiceProxy("led/set_leds", SetLEDs)(colors)
        self.log.write(
            "led_half_red_blue",
            count=len(colors),
            red_count=middle,
            blue_count=len(colors) - middle,
        )

    def _extended_state_callback(self, message):
        self.extended_landed_state = int(message.landed_state)
        self.extended_state_updated = time.monotonic()

    def _ground_state_is_fresh(self, now):
        if self.extended_state_updated is None:
            return False
        max_age = float(self.navigation["landing_state_max_age"])
        return (
            now - self.extended_state_updated <= max_age
            and self.extended_landed_state
            == ExtendedState.LANDED_STATE_ON_GROUND
        )

    def takeoff(self, height):
        self.flight_active = True
        self.navigate_wait(
            x=0.0,
            y=0.0,
            z=float(height),
            frame_id="body",
            auto_arm=True,
        )

    def goto_marker(self, marker_id, altitude=None):
        marker_id = int(marker_id)
        target_altitude = (
            self.cruise_altitude
            if altitude is None
            else float(altitude)
        )
        current_x, current_y = marker_position(self.current_marker)
        target_x, target_y = marker_position(marker_id)
        route_mode = str(self.navigation.get("route_mode", "grid"))
        self.log.write(
            "goto_marker",
            marker=marker_id,
            x=target_x,
            y=target_y,
            z=target_altitude,
            route_mode=route_mode,
        )

        if route_mode == "direct":
            self.log.write(
                "route_direct",
                from_marker=self.current_marker,
                marker=marker_id,
                x=target_x,
                y=target_y,
                z=target_altitude,
            )
            self.navigate_wait(
                x=target_x,
                y=target_y,
                z=target_altitude,
                frame_id="aruco_map",
            )
            self.current_marker = marker_id
            return

        if route_mode != "grid":
            raise RuntimeError(
                "navigation.route_mode должен быть direct или grid"
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
                z=target_altitude,
            )
            self.navigate_wait(
                x=current_x,
                y=current_y,
                z=target_altitude,
                frame_id="aruco_map",
            )
        self.current_marker = marker_id

    def center_on_station(self):
        station_id = int(self.role_config["station_marker"])
        target_x, target_y = marker_position(station_id)
        self.enter("CENTER_ABOVE_STATION")
        strict_tolerance = float(self.role_config.get(
            "station_arrival_tolerance",
            self.navigation["station_arrival_tolerance"],
        ))
        relaxed_tolerance = float(self.role_config.get(
            "station_relaxed_tolerance",
            self.navigation["station_relaxed_tolerance"],
        ))
        self.navigate_wait(
            x=target_x,
            y=target_y,
            z=self.station_detection_altitude,
            frame_id="aruco_map",
            arrival_tolerance=strict_tolerance,
            speed=float(self.navigation["station_speed"]),
            timeout=float(self.navigation["station_center_timeout"]),
            timeout_tolerance=relaxed_tolerance,
            context="station_center",
        )

        hold_seconds = float(self.timing["station_hold_seconds"])
        self.hold_station_target(hold_seconds)
        self.log.write(
            "station_centered",
            station=station_id,
            altitude=self.station_detection_altitude,
            tolerance=strict_tolerance,
            relaxed_tolerance=relaxed_tolerance,
            hold_seconds=hold_seconds,
        )

    def stabilize_after_land_grant(self, permission_granted=True):
        station_id = int(self.role_config["station_marker"])
        target_x, target_y = marker_position(station_id)
        hold_seconds = float(
            self.timing["station_post_grant_hold_seconds"]
        )
        state = (
            "STABILIZE_AFTER_LAND_GRANTED"
            if permission_granted
            else "STABILIZE_FALLBACK_LANDING"
        )
        self.enter(state)
        strict_tolerance = float(self.role_config.get(
            "station_arrival_tolerance",
            self.navigation["station_arrival_tolerance"],
        ))
        relaxed_tolerance = float(self.role_config.get(
            "station_relaxed_tolerance",
            self.navigation["station_relaxed_tolerance"],
        ))
        self.navigate_wait(
            x=target_x,
            y=target_y,
            z=self.station_detection_altitude,
            frame_id="aruco_map",
            arrival_tolerance=strict_tolerance,
            speed=float(self.navigation["station_speed"]),
            timeout=float(self.navigation["station_center_timeout"]),
            timeout_tolerance=relaxed_tolerance,
            context="station_post_permission_center",
        )

        self.hold_station_target(hold_seconds)
        self.log.write(
            "post_grant_stabilized",
            station=station_id,
            altitude=self.station_detection_altitude,
            tolerance=strict_tolerance,
            relaxed_tolerance=relaxed_tolerance,
            hold_seconds=hold_seconds,
            permission_granted=bool(permission_granted),
        )

    def hold_station_target(self, hold_seconds):
        hold_seconds = float(hold_seconds)
        if hold_seconds <= 0.0:
            return

        deadline = time.monotonic() + hold_seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)

        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown while stabilizing on station")
        self.log.write("station_hold_complete", seconds=hold_seconds)

    def verify_role_ip(self, station_ip):
        expected_ip = str(self.network["{}_ip".format(self.role)])
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((station_ip, 45901))
            actual_ip = str(probe.getsockname()[0])
        finally:
            probe.close()
        if actual_ip != expected_ip:
            message = (
                "запущена роль {} на БВС с IP {}; ожидался {}. "
                "На 192.168.0.29 запускай uav1.py, на "
                "192.168.0.184 запускай uav2.py".format(
                    self.role,
                    actual_ip,
                    expected_ip,
                )
            )
            print("WARNING: {} Миссия продолжится.".format(message), flush=True)
            self.log.write(
                "role_ip_warning",
                expected_ip=expected_ip,
                actual_ip=actual_ip,
                message=message,
                mission_continues=True,
            )
            return False
        self.log.write(
            "role_ip_ok",
            expected_ip=expected_ip,
            actual_ip=actual_ip,
        )
        return True

    def await_station_permission(self):
        if self.station_mode == "real":
            self.enter("WAIT_STATION_RED_DETECTION")
            try:
                self.request_landing()
                return True
            except StationDetectionDenied as error:
                self.log.write(
                    "station_detection_fallback",
                    station=int(self.role_config["station_marker"]),
                    error=str(error),
                    fallback="aruco_center_land",
                    mission_continues=True,
                )
                return False
            except RuntimeError as error:
                if str(error) != "timeout waiting for LAND_GRANTED":
                    raise
                self.log.write(
                    "station_detection_fallback",
                    station=int(self.role_config["station_marker"]),
                    error=str(error),
                    fallback="aruco_center_land",
                    mission_continues=True,
                )
                return False

        self.enter("VIRTUAL_STATION")
        self.log.write(
            "virtual_station_no_invitation",
            marker=int(self.role_config["station_marker"]),
        )
        return False

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
        if response.get("status_led_ok") is not True:
            raise RuntimeError(
                "станция {} не подтвердила работу своей LED-ленты".format(
                    actual_station
                )
            )
        self.log.write(
            "station_preflight_ok",
            station=actual_station,
            station_ip=station_ip,
            target_color=response.get("target_color"),
            station_state=station_state,
            status_led_ok=True,
        )

    def navigate_wait(
        self,
        x,
        y,
        z,
        frame_id,
        auto_arm=False,
        arrival_tolerance=None,
        speed=None,
        timeout=None,
        timeout_tolerance=None,
        context="navigation",
    ):
        self.navigate(
            x=x,
            y=y,
            z=z,
            yaw=float("nan"),
            speed=float(
                self.navigation["speed"] if speed is None else speed
            ),
            frame_id=frame_id,
            auto_arm=auto_arm,
        )
        navigation_timeout = float(
            self.navigation["navigation_timeout"]
            if timeout is None
            else timeout
        )
        deadline = time.monotonic() + navigation_timeout
        tolerance = (
            float(self.navigation["arrival_tolerance"])
            if arrival_tolerance is None
            else float(arrival_tolerance)
        )
        last_distance = None
        best_distance = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            telemetry = self.get_telemetry(frame_id="navigate_target")
            distance = math.sqrt(
                telemetry.x ** 2
                + telemetry.y ** 2
                + telemetry.z ** 2
            )
            if math.isfinite(distance):
                last_distance = distance
                if best_distance is None or distance < best_distance:
                    best_distance = distance
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
        if (
            timeout_tolerance is not None
            and last_distance is not None
            and last_distance < float(timeout_tolerance)
        ):
            self.log.write(
                "navigate_arrived_relaxed",
                context=context,
                distance=round(last_distance, 3),
                tolerance=tolerance,
                relaxed_tolerance=float(timeout_tolerance),
                timeout=navigation_timeout,
            )
            return
        self.log.write(
            "navigation_timeout",
            context=context,
            distance=(
                None if last_distance is None else round(last_distance, 3)
            ),
            best_distance=(
                None if best_distance is None else round(best_distance, 3)
            ),
            tolerance=tolerance,
            timeout=navigation_timeout,
        )
        raise RuntimeError(
            "navigation timeout: {}; distance={}".format(
                context,
                "unknown"
                if last_distance is None
                else round(last_distance, 3),
            )
        )

    def land(self):
        started = time.monotonic()
        deadline = started + float(self.navigation["landing_timeout"])
        retry_seconds = float(self.navigation["landing_retry_seconds"])
        ground_confirm_seconds = float(
            self.navigation["landing_ground_confirm_seconds"]
        )
        disarm_retry_seconds = float(
            self.navigation["landing_disarm_retry_seconds"]
        )
        next_retry = started
        next_disarm = started
        ground_since = None
        attempt = 0
        disarm_attempt = 0
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
                self.log.write(
                    "landed_disarmed",
                    attempts=attempt,
                    disarm_attempts=disarm_attempt,
                )
                return

            if self._ground_state_is_fresh(now):
                if ground_since is None:
                    ground_since = now
                    self.log.write("landing_ground_detected")
                if (
                    now - ground_since >= ground_confirm_seconds
                    and now >= next_disarm
                ):
                    disarm_attempt += 1
                    next_disarm = now + disarm_retry_seconds
                    try:
                        response = self.arming_service(False)
                        success = bool(
                            getattr(response, "success", False)
                        )
                        self.log.write(
                            "landing_disarm_command",
                            attempt=disarm_attempt,
                            success=success,
                            ground_seconds=round(now - ground_since, 1),
                        )
                    except Exception as error:
                        self.log.write(
                            "landing_disarm_failed",
                            attempt=disarm_attempt,
                            error=str(error),
                            mission_continues=True,
                        )
            else:
                ground_since = None
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
    install_termination_handlers()
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
