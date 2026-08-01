import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def install_ros_stubs():
    rospy = types.ModuleType("rospy")
    rospy.is_shutdown = lambda: False
    rospy.sleep = lambda _seconds: None
    rospy.init_node = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.wait_for_service = lambda *args, **kwargs: None
    rospy.ServiceProxy = lambda *args, **kwargs: None

    aruco_pose = types.ModuleType("aruco_pose")
    aruco_pose_msg = types.ModuleType("aruco_pose.msg")
    aruco_pose_msg.MarkerArray = type("MarkerArray", (), {})
    aruco_pose.msg = aruco_pose_msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {})
    std_srvs.srv = std_srvs_srv

    technic = types.ModuleType("technic")
    technic_srv = types.ModuleType("technic.srv")
    technic_srv.GetTelemetry = type("GetTelemetry", (), {})
    technic_srv.Navigate = type("Navigate", (), {})
    technic_srv.SetLEDEffect = type("SetLEDEffect", (), {})
    technic.srv = technic_srv

    sys.modules.update({
        "rospy": rospy,
        "aruco_pose": aruco_pose,
        "aruco_pose.msg": aruco_pose_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
        "technic": technic,
        "technic.srv": technic_srv,
    })
    return rospy


ROS = install_ros_stubs()
MISSION = load_module("energy_race_main_mission", SCRIPTS / "mission.py")
CONTROL = load_module("energy_race_main_control", SCRIPTS / "control.py")
CONFIG = json.loads(
    (SCRIPTS / "mission_config.json").read_text(encoding="utf-8")
)


class FakeLog:
    def __init__(self):
        self.events = []

    def enter(self, state):
        self.events.append(("state_enter", {"state": state}))

    def write(self, event, **data):
        self.events.append((event, data))


class FakeBus:
    def __init__(self):
        self.states = []

    def status(self, state, **extra):
        self.states.append((state, extra))


class MissionTests(unittest.TestCase):
    def make_mission(self, role):
        mission = object.__new__(MISSION.Mission)
        mission.role = role
        mission.role_config = CONFIG["roles"][role]
        mission.navigation = CONFIG["navigation"]
        mission.timing = dict(CONFIG["timing"])
        mission.cruise_altitude = float(
            mission.role_config["cruise_altitude"]
        )
        mission.station_mode = mission.role_config["station_mode"]
        mission.current_marker = int(mission.role_config["home_marker"])
        mission.flight_active = True
        mission.log = FakeLog()
        mission.bus = FakeBus()
        return mission

    def test_expert_role_mapping_and_station_ips(self):
        uav1 = CONFIG["roles"]["uav1"]
        uav2 = CONFIG["roles"]["uav2"]
        self.assertEqual((uav1["home_marker"], uav1["station_marker"]), (48, 5))
        self.assertEqual(uav1["station_ip"], "192.168.0.224")
        self.assertEqual(uav1["cruise_altitude"], 2.0)
        self.assertEqual(
            (uav2["home_marker"], uav2["cargo_marker"], uav2["station_marker"]),
            (27, 0, 37),
        )
        self.assertEqual(uav2["station_ip"], "192.168.0.239")
        self.assertEqual(uav2["cruise_altitude"], 2.0)
        self.assertEqual(CONFIG["network"]["uav2_ip"], "192.168.0.184")
        self.assertEqual(CONFIG["timing"]["uav1_route_delay"], 5.0)
        self.assertEqual(CONFIG["navigation"]["route_mode"], "direct")
        self.assertEqual(CONFIG["navigation"]["speed"], 0.45)

    def test_preflight_rejects_non_free_station(self):
        mission = self.make_mission("uav1")
        mission.network = {"event_port": 45900}

        class StationBus(FakeBus):
            def send(self, *args, **kwargs):
                pass

            def wait(self, *args, **kwargs):
                return {
                    "station": 5,
                    "target_color": "red",
                    "station_state": "reserved",
                    "status_led_ok": True,
                }

        mission.bus = StationBus()
        with self.assertRaisesRegex(RuntimeError, "не свободна: reserved"):
            mission.preflight_station("192.168.0.224")

    def test_preflight_rejects_unavailable_station_led(self):
        mission = self.make_mission("uav1")
        mission.network = {"event_port": 45900}

        class StationBus(FakeBus):
            def send(self, *args, **kwargs):
                pass

            def wait(self, *args, **kwargs):
                return {
                    "station": 5,
                    "target_color": "red",
                    "station_state": "free",
                    "status_led_ok": False,
                }

        mission.bus = StationBus()
        with self.assertRaisesRegex(RuntimeError, "LED-ленты"):
            mission.preflight_station("192.168.0.224")

    def test_uav1_uses_one_direct_leg_to_station(self):
        mission = self.make_mission("uav1")
        calls = []
        mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

        mission.goto_marker(5)

        self.assertEqual(len(calls), 1)
        self.assertEqual(
            (calls[0]["x"], calls[0]["y"], calls[0]["z"]),
            (5.0, 6.0, 2.0),
        )
        self.assertEqual(calls[0]["frame_id"], "aruco_map")
        self.assertEqual(mission.bus.states, [])

    def test_all_main_targets_use_one_direct_setpoint(self):
        cases = (
            ("uav1", 5, (5.0, 6.0, 2.0)),
            ("uav1", 48, (6.0, 0.0, 2.0)),
            ("uav2", 0, (0.0, 6.0, 2.0)),
            ("uav2", 37, (2.0, 1.0, 2.0)),
            ("uav2", 27, (6.0, 3.0, 2.0)),
        )
        for role, marker, expected in cases:
            mission = self.make_mission(role)
            calls = []
            mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

            mission.goto_marker(marker)

            self.assertEqual(len(calls), 1)
            self.assertEqual(
                (calls[0]["x"], calls[0]["y"], calls[0]["z"]),
                expected,
            )
            self.assertEqual(calls[0]["frame_id"], "aruco_map")

    def test_grid_route_remains_available_as_fallback(self):
        mission = self.make_mission("uav1")
        mission.navigation = dict(mission.navigation)
        mission.navigation["route_mode"] = "grid"
        calls = []
        mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

        mission.goto_marker(5)

        markers = [
            int((6.0 - call["y"]) * 7 + call["x"])
            for call in calls
        ]
        self.assertEqual(markers, [47, 40, 33, 26, 19, 12, 5])

    def test_uav1_holds_five_seconds_before_station_route(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.hold_before_station_route()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertGreaterEqual(clock[0], 5.0)
        completed = [
            data for event, data in mission.log.events
            if event == "uav1_route_delay_done"
        ]
        self.assertEqual(completed, [{"seconds": 5.0}])

    def test_initial_route_delay_applies_only_to_uav1(self):
        class StopAtFirstRoute(Exception):
            pass

        def initial_events(role):
            mission = self.make_mission(role)
            events = []
            mission.enter = lambda state: events.append(state)
            mission.led = lambda *args: None
            mission.wait_start = lambda: None
            mission.takeoff = lambda altitude: events.append(
                ("takeoff", altitude)
            )
            mission.wait_any_marker = lambda: events.append("aruco_ready")
            mission.hold_before_station_route = lambda: events.append(
                "uav1_delay"
            )

            def stop_at_route(marker):
                events.append(("goto", marker))
                raise StopAtFirstRoute()

            mission.goto_marker = stop_at_route
            with self.assertRaises(StopAtFirstRoute):
                if role == "uav1":
                    mission.run_uav1()
                else:
                    mission.run_uav2()
            return events

        self.assertEqual(
            initial_events("uav1"),
            [
                "WAIT_START",
                "TAKEOFF_YELLOW",
                ("takeoff", 2.0),
                "aruco_ready",
                "uav1_delay",
                "SEARCH_STATION_RED",
                ("goto", 5),
            ],
        )
        self.assertEqual(
            initial_events("uav2"),
            [
                "WAIT_START",
                "TAKEOFF_YELLOW",
                ("takeoff", 2.0),
                "aruco_ready",
                "FLY_TO_CARGO_YELLOW",
                ("goto", 0),
            ],
        )

    def test_station_centering_uses_precise_tolerance(self):
        for role, expected in (
            ("uav1", (5.0, 6.0, 2.0)),
            ("uav2", (2.0, 1.0, 2.0)),
        ):
            mission = self.make_mission(role)
            mission.timing["station_hold_seconds"] = 0.0
            calls = []
            mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

            mission.center_on_station()

            self.assertEqual(
                (calls[0]["x"], calls[0]["y"], calls[0]["z"]),
                expected,
            )
            self.assertEqual(calls[0]["arrival_tolerance"], 0.15)

    def test_land_retries_after_ten_seconds(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        commands = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep

        class Telemetry:
            @property
            def armed(self):
                return clock[0] < 10.1

        mission.land_service = lambda: commands.append(round(clock[0], 1))
        mission.get_telemetry = lambda: Telemetry()
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.land()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(commands, [0.0, 10.2])
        self.assertFalse(mission.flight_active)

    def test_gpio_servo_matches_calibrated_values(self):
        servo = MISSION.Servo(CONFIG["servo"], FakeLog())
        self.assertEqual(servo.angle_to_pwm(45), 125)
        self.assertEqual(servo.angle_to_pwm(135), 275)

    def test_servo_preflight_never_sends_gpio_commands(self):
        log = FakeLog()
        servo = MISSION.Servo(CONFIG["servo"], log)
        commands = []
        original_which = MISSION.shutil.which
        MISSION.shutil.which = lambda _command: "/usr/bin/gpio"
        servo._run_gpio = lambda *args: commands.append(args)
        try:
            result = servo.preflight()
        finally:
            MISSION.shutil.which = original_which

        self.assertTrue(result)
        self.assertEqual(commands, [])
        self.assertFalse(servo.initialized)

    def test_servo_failure_is_non_fatal(self):
        mission = self.make_mission("uav2")

        class BrokenServo:
            released = False

            def close_grip(self):
                raise RuntimeError("gpio failed")

            def release(self):
                self.released = True

        mission.servo = BrokenServo()
        mission.servo_available = True

        self.assertFalse(mission.try_servo_action("close_grip"))
        self.assertFalse(mission.servo_available)
        self.assertTrue(mission.servo.released)
        warnings = [
            data for event, data in mission.log.events
            if event == "servo_action_warning"
        ]
        self.assertEqual(len(warnings), 1)
        self.assertTrue(warnings[0]["mission_continues"])

    def test_invalid_servo_config_is_non_fatal(self):
        log = FakeLog()
        servo = MISSION.create_servo({"enabled": True}, log)

        self.assertIsInstance(servo, MISSION.UnavailableServo)
        self.assertFalse(servo.preflight())
        self.assertFalse(servo.close_grip())
        self.assertFalse(servo.open_grip())


class ControllerTests(unittest.TestCase):
    def test_synchronous_start_order(self):
        waits = []
        sends = []
        sleeps = []
        answers = iter(("START", "FLY"))

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def setsockopt(self, *args):
                pass

            def bind(self, *args):
                pass

        originals = (
            CONTROL.socket.socket,
            CONTROL.wait_states,
            CONTROL.send_events,
            CONTROL.time.sleep,
        )
        original_input = CONTROL.__builtins__["input"]
        CONTROL.socket.socket = lambda *args, **kwargs: FakeSocket()
        CONTROL.wait_states = (
            lambda _sock, _states, required, _description: waits.append(required)
        )
        CONTROL.send_events = lambda _sock, commands: sends.append(commands)
        CONTROL.time.sleep = lambda seconds: sleeps.append(seconds)
        CONTROL.__builtins__["input"] = lambda _prompt="": next(answers)
        try:
            CONTROL.run_controller()
        finally:
            (
                CONTROL.socket.socket,
                CONTROL.wait_states,
                CONTROL.send_events,
                CONTROL.time.sleep,
            ) = originals
            CONTROL.__builtins__["input"] = original_input

        self.assertEqual(
            sends[0],
            (("START", "uav1"), ("START", "uav2")),
        )
        self.assertNotIn((("uav1", "TAKEOFF_DONE"),), waits)
        self.assertFalse(any("ROUTE_CLEAR" in str(wait) for wait in waits))
        self.assertEqual(sleeps, [])


class StationConfigTests(unittest.TestCase):
    def test_calibrations_and_sensitivity(self):
        cases = (
            (5, 0.0007552083333333333),
            (37, 0.005338541666666667),
        )
        for station_id, calibrated_threshold in cases:
            directory = ROOT / "station" / "field" / (
                "station-{}".format(station_id)
            ) / "red"
            config = json.loads(
                (directory / "config.json").read_text(encoding="utf-8")
            )
            calibration = json.loads(
                (directory / "calibration.json").read_text(encoding="utf-8")
            )
            self.assertEqual(config["station_id"], station_id)
            self.assertEqual(config["threshold_scale"], 0.5)
            self.assertEqual(config["status_led"]["mode"], "ros")
            self.assertEqual(calibration["threshold"], calibrated_threshold)
            self.assertAlmostEqual(
                calibration["threshold"] * config["threshold_scale"],
                calibrated_threshold * 0.5,
            )


if __name__ == "__main__":
    unittest.main()
