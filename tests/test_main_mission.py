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

    mavros_msgs = types.ModuleType("mavros_msgs")
    mavros_msgs_msg = types.ModuleType("mavros_msgs.msg")
    mavros_msgs_msg.ExtendedState = type(
        "ExtendedState",
        (),
        {"LANDED_STATE_ON_GROUND": 1},
    )
    mavros_msgs_srv = types.ModuleType("mavros_msgs.srv")
    mavros_msgs_srv.CommandBool = type("CommandBool", (), {})
    mavros_msgs.msg = mavros_msgs_msg
    mavros_msgs.srv = mavros_msgs_srv

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
        "mavros_msgs": mavros_msgs,
        "mavros_msgs.msg": mavros_msgs_msg,
        "mavros_msgs.srv": mavros_msgs_srv,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
        "technic": technic,
        "technic.srv": technic_srv,
    })
    return rospy


ROS = install_ros_stubs()
MISSION = load_module("energy_race_main_mission", SCRIPTS / "mission.py")
CONTROL = load_module("energy_race_main_control", SCRIPTS / "control.py")
sys.modules.setdefault("cv2", types.ModuleType("cv2"))
sys.modules.setdefault("numpy", types.ModuleType("numpy"))
STATION = load_module("energy_race_station", ROOT / "station" / "station.py")
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
        mission.return_altitude = float(
            mission.navigation["return_altitude"]
        )
        mission.station_detection_altitude = float(
            mission.navigation["station_detection_altitude"]
        )
        mission.station_mode = mission.role_config["station_mode"]
        mission.led_count = int(CONFIG["led"]["count"])
        mission.current_marker = int(mission.role_config["home_marker"])
        mission.flight_active = True
        mission.extended_landed_state = None
        mission.extended_state_updated = None
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
        self.assertEqual(
            CONFIG["navigation"]["station_detection_altitude"],
            1.8,
        )
        self.assertEqual(CONFIG["timing"]["uav1_route_delay"], 5.0)
        self.assertEqual(
            CONFIG["timing"]["station_post_grant_hold_seconds"],
            3.0,
        )
        self.assertEqual(CONFIG["timing"]["station_hold_seconds"], 3.0)
        self.assertEqual(
            CONFIG["navigation"]["station_departure_height"],
            2.0,
        )
        self.assertEqual(CONFIG["navigation"]["return_altitude"], 2.5)
        self.assertEqual(CONFIG["navigation"]["station_speed"], 0.25)
        self.assertEqual(
            CONFIG["navigation"]["station_arrival_tolerance"],
            0.1,
        )
        self.assertEqual(
            CONFIG["navigation"]["station_relaxed_tolerance"],
            0.2,
        )
        self.assertEqual(CONFIG["navigation"]["station_center_timeout"], 15.0)
        self.assertEqual(uav2["station_arrival_tolerance"], 0.18)
        self.assertEqual(uav2["station_relaxed_tolerance"], 0.35)
        self.assertEqual(CONFIG["navigation"]["route_mode"], "direct")
        self.assertEqual(CONFIG["navigation"]["speed"], 0.45)
        self.assertEqual(CONFIG["led"]["count"], 72)
        self.assertEqual(
            CONFIG["navigation"]["landing_ground_confirm_seconds"],
            1.0,
        )
        self.assertEqual(CONFIG["navigation"]["landing_timeout"], 30.0)
        self.assertEqual(
            CONFIG["navigation"]["landing_guarded_disarm_seconds"],
            8.0,
        )

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

    def test_station_detection_denial_is_distinguishable(self):
        bus = object.__new__(MISSION.UdpBus)
        bus.config = {"team": CONFIG["team"]}
        bus.role = "uav2"
        bus.log = FakeLog()
        bus.pending = []

        class FakeSocket:
            def settimeout(self, _timeout):
                pass

            def recvfrom(self, _size):
                payload = json.dumps({
                    "team": CONFIG["team"],
                    "event": "LAND_DENIED",
                    "request_id": "request-1",
                    "reason": "detection_timeout",
                }).encode("utf-8")
                return payload, ("192.168.0.239", 45901)

        bus.socket = FakeSocket()
        with self.assertRaisesRegex(
            MISSION.StationDetectionDenied,
            "станция не распознала LED",
        ):
            bus.wait("LAND_GRANTED", 1.0, request_id="request-1")

    def test_stale_station_denial_does_not_abort_return_home_wait(self):
        bus = object.__new__(MISSION.UdpBus)
        bus.config = {"team": CONFIG["team"]}
        bus.role = "uav1"
        bus.log = FakeLog()
        bus.pending = []
        payloads = iter((
            {
                "team": CONFIG["team"],
                "event": "LAND_DENIED",
                "request_id": "old-request",
            },
            {
                "team": CONFIG["team"],
                "event": "RETURN_HOME",
                "target": "uav1",
            },
        ))

        class FakeSocket:
            def settimeout(self, _timeout):
                pass

            def recvfrom(self, _size):
                return (
                    json.dumps(next(payloads)).encode("utf-8"),
                    ("192.168.0.224", 45901),
                )

        bus.socket = FakeSocket()
        result = bus.wait("RETURN_HOME", 1.0)

        self.assertEqual(result["event"], "RETURN_HOME")
        ignored = [
            data for event, data in bus.log.events
            if event == "udp_ignored"
        ]
        self.assertEqual(len(ignored), 1)
        self.assertEqual(ignored[0]["message"], "LAND_DENIED")
        self.assertEqual(ignored[0]["waiting_for"], "RETURN_HOME")

    def test_station_detection_denial_continues_with_aruco_landing(self):
        mission = self.make_mission("uav1")

        def deny():
            raise MISSION.StationDetectionDenied(
                "станция не распознала LED до таймаута"
            )

        mission.request_landing = deny

        self.assertFalse(mission.await_station_permission())
        fallbacks = [
            data for event, data in mission.log.events
            if event == "station_detection_fallback"
        ]
        self.assertEqual(len(fallbacks), 1)
        self.assertEqual(fallbacks[0]["fallback"], "aruco_center_land")
        self.assertTrue(fallbacks[0]["mission_continues"])

    def test_both_roles_finish_after_station_detection_fallback(self):
        for role in ("uav1", "uav2"):
            mission = self.make_mission(role)
            events = []
            mission.led = lambda *_args: None
            mission.try_led = lambda *_args: True
            mission.wait_start = lambda: None
            mission.takeoff = lambda height, **_kwargs: events.append(
                ("takeoff", height)
            )
            mission.wait_any_marker = lambda: None
            mission.hold_before_station_route = lambda: None
            mission.goto_marker = lambda marker, altitude=None: events.append(
                ("goto", marker, altitude)
            )
            mission.center_on_station = lambda: None
            mission.await_station_permission = lambda: False
            mission.stabilize_after_land_grant = (
                lambda permission: events.append(
                    ("stabilized", permission)
                )
            )
            mission.land = lambda: events.append(("land", None))
            mission.try_notify_station = lambda event: (
                events.append(("station", event)) or True
            )
            mission.charge = lambda: events.append(("charge", None))
            mission.bus.wait = lambda *_args, **_kwargs: None
            mission.try_servo_action = lambda action: (
                events.append(("servo", action)) or False
            )
            mission.try_half_red_blue = lambda: (
                events.append(("led", "half_red_blue")) or False
            )

            if role == "uav1":
                mission.run_uav1()
            else:
                mission.run_uav2()

            self.assertIn(("stabilized", False), events)
            self.assertIn(("station", "LANDED"), events)
            self.assertIn(("station", "STATION_RELEASED"), events)
            self.assertIn(("DONE", {}), mission.bus.states)
            self.assertIn(("charge", None), events)
            if role == "uav2":
                self.assertLess(
                    events.index(("servo", "open_grip")),
                    events.index(("charge", None)),
                )
            if role == "uav2":
                self.assertIn(("led", "half_red_blue"), events)

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

    def test_return_targets_use_2_5_meter_setpoint(self):
        for role, marker, expected_xy in (
            ("uav1", 48, (6.0, 0.0)),
            ("uav2", 27, (6.0, 3.0)),
        ):
            mission = self.make_mission(role)
            calls = []
            mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

            mission.goto_marker(marker, altitude=mission.return_altitude)

            self.assertEqual(len(calls), 1)
            self.assertEqual(
                (calls[0]["x"], calls[0]["y"], calls[0]["z"]),
                expected_xy + (2.5,),
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
            mission.takeoff = lambda altitude, **kwargs: events.append(
                (
                    "takeoff",
                    altitude,
                    kwargs.get("reapply_yellow_after_arm", False),
                )
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
                ("takeoff", 2.0, True),
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
                ("takeoff", 2.0, True),
                "aruco_ready",
                "FLY_TO_CARGO_YELLOW",
                ("goto", 0),
            ],
        )

    def test_regulation_outbound_led_sequences(self):
        class StopSequence(Exception):
            pass

        uav1 = self.make_mission("uav1")
        uav1_leds = []
        uav1.enter = lambda _state: None
        uav1.led = lambda effect, red, green, blue: uav1_leds.append(
            (effect, red, green, blue)
        )
        uav1.wait_start = lambda: None
        uav1.takeoff = lambda _height, **_kwargs: None
        uav1.wait_any_marker = lambda: None
        uav1.hold_before_station_route = lambda: None
        uav1.goto_marker = lambda _marker: (_ for _ in ()).throw(
            StopSequence()
        )
        with self.assertRaises(StopSequence):
            uav1.run_uav1()
        self.assertEqual(
            uav1_leds,
            [
                ("blink", 255, 255, 0),
                ("fill", 255, 0, 0),
            ],
        )

        uav2 = self.make_mission("uav2")
        uav2_leds = []
        uav2.enter = lambda _state: None
        uav2.led = lambda effect, red, green, blue: uav2_leds.append(
            (effect, red, green, blue)
        )
        uav2.wait_start = lambda: None
        uav2.takeoff = lambda _height, **_kwargs: None
        uav2.wait_any_marker = lambda: None
        uav2.goto_marker = lambda _marker: None
        uav2.land = lambda: None
        uav2.bus.status = lambda state, **_extra: (
            (_ for _ in ()).throw(StopSequence())
            if state == "CARGO_LANDED"
            else None
        )
        with self.assertRaises(StopSequence):
            uav2.run_uav2()
        self.assertEqual(
            uav2_leds,
            [
                ("blink", 255, 255, 0),
                ("blink", 255, 255, 0),
                ("fill", 255, 0, 0),
            ],
        )

    def test_initial_takeoff_reapplies_yellow_only_after_arm(self):
        mission = self.make_mission("uav1")
        samples_seen = []
        led_events = []

        class Telemetry:
            def __init__(self, armed, distance):
                self.armed = armed
                self.x = 0.0
                self.y = 0.0
                self.z = distance

        samples = iter((
            Telemetry(False, 2.0),
            Telemetry(True, 1.0),
            Telemetry(True, 0.0),
        ))
        mission.navigate = lambda **_kwargs: None

        def telemetry(**_kwargs):
            sample = next(samples)
            samples_seen.append(sample)
            return sample

        mission.get_telemetry = telemetry
        mission.try_led = lambda *args: (
            led_events.append((len(samples_seen), args)) or True
        )

        mission.takeoff(2.0, reapply_yellow_after_arm=True)

        self.assertEqual(
            led_events,
            [(2, ("blink", 255, 255, 0))],
        )
        reapplications = [
            data for event, data in mission.log.events
            if event == "takeoff_led_reapplied_after_arm"
        ]
        self.assertEqual(len(reapplications), 1)
        self.assertTrue(reapplications[0]["success"])

        calls = []
        mission.navigate_wait = lambda **kwargs: calls.append(kwargs)
        mission.takeoff(2.0)
        self.assertIsNone(calls[0]["post_arm_led"])

    def test_station_centering_uses_precise_tolerance(self):
        for role, expected, strict, relaxed in (
            ("uav1", (5.0, 6.0, 1.8), 0.1, 0.2),
            ("uav2", (2.0, 1.0, 1.8), 0.18, 0.35),
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
            self.assertEqual(calls[0]["arrival_tolerance"], strict)
            self.assertEqual(calls[0]["timeout_tolerance"], relaxed)
            self.assertEqual(calls[0]["timeout"], 15.0)
            self.assertEqual(calls[0]["context"], "station_center")
            self.assertEqual(calls[0]["speed"], 0.25)

    def test_both_uavs_recenter_and_hold_after_land_grant(self):
        for role, station, expected, strict, relaxed in (
            ("uav1", 5, (5.0, 6.0, 1.8), 0.1, 0.2),
            ("uav2", 37, (2.0, 1.0, 1.8), 0.18, 0.35),
        ):
            mission = self.make_mission(role)
            mission.timing["station_post_grant_hold_seconds"] = 0.0
            calls = []
            mission.navigate_wait = lambda **kwargs: calls.append(kwargs)

            mission.stabilize_after_land_grant()

            self.assertEqual(len(calls), 1)
            self.assertEqual(
                (calls[0]["x"], calls[0]["y"], calls[0]["z"]),
                expected,
            )
            self.assertEqual(calls[0]["frame_id"], "aruco_map")
            self.assertEqual(calls[0]["arrival_tolerance"], strict)
            self.assertEqual(calls[0]["timeout_tolerance"], relaxed)
            self.assertEqual(calls[0]["timeout"], 15.0)
            self.assertEqual(
                calls[0]["context"],
                "station_post_permission_center",
            )
            self.assertEqual(calls[0]["speed"], 0.25)
            self.assertIn(
                (
                    "post_grant_stabilized",
                    {
                        "station": station,
                        "altitude": 1.8,
                        "tolerance": strict,
                        "relaxed_tolerance": relaxed,
                        "hold_seconds": 0.0,
                        "permission_granted": True,
                    },
                ),
                mission.log.events,
            )

    def test_uav2_station_centering_reaches_detection_after_relaxed_arrival(self):
        mission = self.make_mission("uav2")
        clock = [0.0]
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep
        mission.navigate = lambda **_kwargs: None
        mission.get_telemetry = lambda **_kwargs: types.SimpleNamespace(
            x=0.24,
            y=0.0,
            z=0.0,
        )
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.navigate_wait(
                x=2.0,
                y=1.0,
                z=1.8,
                frame_id="aruco_map",
                arrival_tolerance=0.18,
                timeout=1.0,
                timeout_tolerance=0.35,
                context="station_center",
            )
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        relaxed = [
            data for event, data in mission.log.events
            if event == "navigate_arrived_relaxed"
        ]
        self.assertEqual(len(relaxed), 1)
        self.assertEqual(relaxed[0]["context"], "station_center")
        self.assertEqual(relaxed[0]["distance"], 0.24)

    def test_station_hold_keeps_setpoint_without_unreliable_telemetry(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep
        mission.get_telemetry = lambda **_kwargs: self.fail(
            "hold must not query navigate_target telemetry"
        )
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.hold_station_target(0.3)
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertGreaterEqual(clock[0], 0.3)
        self.assertTrue(any(
            event == "station_hold_complete"
            for event, _data in mission.log.events
        ))

    def test_uav1_stabilizes_between_permission_and_land(self):
        mission = self.make_mission("uav1")
        events = []

        class StopAtLand(Exception):
            pass

        mission.enter = lambda state: events.append(state)
        mission.led = lambda *args: None
        mission.wait_start = lambda: None
        mission.takeoff = lambda _altitude, **_kwargs: None
        mission.wait_any_marker = lambda: None
        mission.hold_before_station_route = lambda: None
        mission.goto_marker = lambda _marker: None
        mission.center_on_station = lambda: events.append("CENTER_INITIAL")
        def grant_permission():
            events.append("LAND_GRANTED")
            return True

        mission.await_station_permission = grant_permission
        mission.stabilize_after_land_grant = (
            lambda permission: events.append(
                ("STABILIZED", permission)
            )
        )

        def stop_at_land():
            events.append("LAND_CALLED")
            raise StopAtLand()

        mission.land = stop_at_land
        with self.assertRaises(StopAtLand):
            mission.run_uav1()

        self.assertLess(
            events.index("LAND_GRANTED"),
            events.index(("STABILIZED", True)),
        )
        self.assertLess(
            events.index(("STABILIZED", True)),
            events.index("LAND_CALLED"),
        )

    def test_uav2_stabilizes_between_permission_and_station_land(self):
        mission = self.make_mission("uav2")
        events = []
        land_calls = [0]

        class StopAtStationLand(Exception):
            pass

        mission.enter = lambda state: events.append(state)
        mission.led = lambda *args: None
        mission.wait_start = lambda: None
        mission.takeoff = lambda _altitude, **_kwargs: None
        mission.wait_any_marker = lambda: None
        mission.goto_marker = lambda _marker: None
        mission.center_on_station = lambda: events.append("CENTER_INITIAL")
        def grant_permission():
            events.append("LAND_GRANTED")
            return True

        mission.await_station_permission = grant_permission
        mission.stabilize_after_land_grant = (
            lambda permission: events.append(
                ("STABILIZED", permission)
            )
        )
        mission.try_servo_action = lambda _action: True
        mission.bus.wait = lambda *_args, **_kwargs: None

        def land():
            land_calls[0] += 1
            if land_calls[0] == 2:
                events.append("STATION_LAND_CALLED")
                raise StopAtStationLand()

        mission.land = land
        with self.assertRaises(StopAtStationLand):
            mission.run_uav2()

        self.assertLess(
            events.index("LAND_GRANTED"),
            events.index(("STABILIZED", True)),
        )
        self.assertLess(
            events.index(("STABILIZED", True)),
            events.index("STATION_LAND_CALLED"),
        )

    def test_full_role_flows_use_raised_return_and_release_uav2_cargo(self):
        for role, home_marker in (("uav1", 48), ("uav2", 27)):
            mission = self.make_mission(role)
            events = []
            routes = []
            mission.enter = lambda state: events.append(("state", state))
            mission.led = lambda *_args: None
            mission.wait_start = lambda: None
            mission.takeoff = lambda height, **_kwargs: events.append(
                ("takeoff", height)
            )
            mission.wait_any_marker = lambda: None
            mission.hold_before_station_route = lambda: None
            mission.goto_marker = lambda marker, altitude=None: routes.append(
                (marker, altitude)
            )
            mission.center_on_station = lambda: None
            mission.await_station_permission = lambda: True
            mission.stabilize_after_land_grant = lambda _permission: None
            mission.land = lambda: events.append(("land", None))
            mission.notify_station = lambda event: events.append(
                ("station", event)
            )
            def charge():
                events.append(("charge", None))

            mission.charge = charge
            mission.bus.wait = lambda *_args, **_kwargs: None
            mission.half_red_blue = lambda: events.append(("led", "half"))
            mission.try_servo_action = lambda action: (
                events.append(("servo", action)) or True
            )

            if role == "uav1":
                mission.run_uav1()
            else:
                mission.run_uav2()

            self.assertEqual(routes[-1], (home_marker, 2.5))
            if role == "uav2":
                self.assertEqual(
                    [event for event in events if event[0] == "servo"],
                    [
                        ("servo", "close_grip"),
                        ("servo", "open_grip"),
                    ],
                )
                self.assertIn(("charge", None), events)
                self.assertLess(
                    events.index(("servo", "open_grip")),
                    events.index(("charge", None)),
                )
                self.assertLess(
                    events.index(("charge", None)),
                    events.index(("led", "half")),
                )

    def test_ssh_disconnect_and_sigterm_install_safe_handlers(self):
        registrations = []
        original_signal = MISSION.signal.signal
        MISSION.signal.signal = (
            lambda signum, handler: registrations.append((signum, handler))
        )
        try:
            MISSION.install_termination_handlers()
        finally:
            MISSION.signal.signal = original_signal

        self.assertEqual(
            [signum for signum, _handler in registrations],
            [MISSION.signal.SIGHUP, MISSION.signal.SIGTERM],
        )
        with self.assertRaises(KeyboardInterrupt):
            MISSION.handle_termination_signal(MISSION.signal.SIGHUP, None)

    def test_interrupted_main_requests_safe_land_before_close(self):
        events = []

        class FakeMission:
            log = FakeLog()

            class Bus:
                @staticmethod
                def status(_state):
                    events.append("STATUS")

            bus = Bus()

            def __init__(self, _config, _role):
                pass

            def run(self):
                raise KeyboardInterrupt

            def safe_land(self):
                events.append("SAFE_LAND")

            def close(self):
                events.append("CLOSE")

        originals = (
            MISSION.Mission,
            MISSION.load_config,
            MISSION.install_termination_handlers,
        )
        MISSION.Mission = FakeMission
        MISSION.load_config = lambda: CONFIG
        MISSION.install_termination_handlers = lambda: None
        try:
            self.assertEqual(MISSION.main("uav2"), 130)
        finally:
            (
                MISSION.Mission,
                MISSION.load_config,
                MISSION.install_termination_handlers,
            ) = originals

        self.assertLess(events.index("SAFE_LAND"), events.index("CLOSE"))

    def test_role_ip_mismatch_warns_and_continues(self):
        mission = self.make_mission("uav2")
        mission.network = dict(CONFIG["network"])

        class FakeProbe:
            def connect(self, _destination):
                pass

            def getsockname(self):
                return ("192.168.0.29", 12345)

            def close(self):
                pass

        original_socket = MISSION.socket.socket
        MISSION.socket.socket = lambda *args, **kwargs: FakeProbe()
        try:
            self.assertFalse(mission.verify_role_ip("192.168.0.239"))
        finally:
            MISSION.socket.socket = original_socket

        warnings = [
            data for event, data in mission.log.events
            if event == "role_ip_warning"
        ]
        self.assertEqual(len(warnings), 1)
        self.assertTrue(warnings[0]["mission_continues"])
        self.assertEqual(warnings[0]["actual_ip"], "192.168.0.29")
        self.assertEqual(warnings[0]["expected_ip"], "192.168.0.184")

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

    def test_land_disarms_only_after_confirmed_on_ground(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        armed = [True]
        land_commands = []
        disarm_commands = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep

        class Telemetry:
            @property
            def armed(self):
                return armed[0]

        class Response:
            success = True

        mission.land_service = lambda: land_commands.append(clock[0])
        mission.get_telemetry = lambda: Telemetry()

        def arming(value):
            disarm_commands.append((clock[0], value))
            armed[0] = False
            return Response()

        mission.arming_service = arming
        mission.extended_landed_state = (
            MISSION.ExtendedState.LANDED_STATE_ON_GROUND
        )
        mission.extended_state_updated = 0.0
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.land()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(land_commands, [0.0])
        self.assertEqual(len(disarm_commands), 1)
        self.assertFalse(disarm_commands[0][1])
        self.assertGreaterEqual(disarm_commands[0][0], 1.0)
        self.assertFalse(mission.flight_active)

    def test_land_never_forces_disarm_without_ground_state(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        disarm_commands = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep

        class Telemetry:
            @property
            def armed(self):
                return clock[0] < 1.2

        mission.land_service = lambda: None
        mission.get_telemetry = lambda: Telemetry()
        mission.arming_service = lambda value: disarm_commands.append(value)
        mission.extended_landed_state = 4
        mission.extended_state_updated = 0.0
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.land()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(disarm_commands, [])

    def test_land_uses_guarded_disarm_when_extended_state_is_stale(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        armed = [True]
        disarm_commands = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep

        class Telemetry:
            @property
            def armed(self):
                return armed[0]

        class Response:
            success = True

        mission.land_service = lambda: None
        mission.get_telemetry = lambda: Telemetry()

        def arming(value):
            disarm_commands.append((clock[0], value))
            armed[0] = False
            return Response()

        mission.arming_service = arming
        mission.extended_landed_state = None
        mission.extended_state_updated = None
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.land()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(len(disarm_commands), 1)
        self.assertFalse(disarm_commands[0][1])
        self.assertGreaterEqual(disarm_commands[0][0], 8.0)
        self.assertLess(disarm_commands[0][0], 8.2)
        events = [
            data for event, data in mission.log.events
            if event == "landing_disarm_command"
        ]
        self.assertEqual(events[0]["source"], "px4_guarded_fallback")
        self.assertFalse(mission.flight_active)

    def test_half_red_blue_uses_exactly_36_plus_36_leds(self):
        mission = self.make_mission("uav2")
        sent = []
        original_service_proxy = ROS.ServiceProxy

        class LEDState:
            def __init__(self, index, red, green, blue):
                self.index = index
                self.r = red
                self.g = green
                self.b = blue

        led_msgs = types.ModuleType("led_msgs")
        led_msgs_msg = types.ModuleType("led_msgs.msg")
        led_msgs_srv = types.ModuleType("led_msgs.srv")
        led_msgs_msg.LEDState = LEDState
        led_msgs_srv.SetLEDs = type("SetLEDs", (), {})
        led_msgs.msg = led_msgs_msg
        led_msgs.srv = led_msgs_srv
        previous_modules = {
            name: sys.modules.get(name)
            for name in ("led_msgs", "led_msgs.msg", "led_msgs.srv")
        }
        sys.modules.update({
            "led_msgs": led_msgs,
            "led_msgs.msg": led_msgs_msg,
            "led_msgs.srv": led_msgs_srv,
        })
        ROS.ServiceProxy = lambda *_args, **_kwargs: (
            lambda colors: sent.extend(colors)
        )
        try:
            mission.half_red_blue()
        finally:
            ROS.ServiceProxy = original_service_proxy
            for name, module in previous_modules.items():
                if module is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = module

        self.assertEqual(len(sent), 72)
        self.assertEqual([item.index for item in sent], list(range(72)))
        self.assertTrue(
            all((item.r, item.g, item.b) == (255, 0, 0) for item in sent[:36])
        )
        self.assertTrue(
            all((item.r, item.g, item.b) == (0, 0, 255) for item in sent[36:])
        )

    def test_charge_blinks_red_then_holds_green(self):
        mission = self.make_mission("uav1")
        clock = [0.0]
        indications = []
        states = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep
        mission.enter = lambda state: states.append((state, clock[0]))
        mission.led = lambda effect, red, green, blue: indications.append(
            (effect, red, green, blue, clock[0])
        )
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.charge()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(
            [indication[:4] for indication in indications],
            [
                ("blink", 255, 0, 0),
                ("fill", 0, 255, 0),
            ],
        )
        self.assertAlmostEqual(indications[0][4], 0.0)
        self.assertGreaterEqual(indications[1][4], 15.0)
        self.assertLess(indications[1][4], 15.2)
        self.assertEqual(states[0], ("CHARGING_RED_BLINK", 0.0))
        self.assertEqual(states[1][0], "CHARGING_GREEN")
        self.assertGreaterEqual(states[1][1], 15.0)
        self.assertLess(states[1][1], 15.2)
        self.assertGreaterEqual(clock[0], 20.0)

    def test_uav2_charge_continues_after_led_failure(self):
        mission = self.make_mission("uav2")
        clock = [0.0]
        led_attempts = []
        original_monotonic = MISSION.time.monotonic
        original_sleep = ROS.sleep
        mission.try_led = lambda *args: (
            led_attempts.append(args) or False
        )
        MISSION.time.monotonic = lambda: clock[0]
        ROS.sleep = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
        try:
            mission.charge()
        finally:
            MISSION.time.monotonic = original_monotonic
            ROS.sleep = original_sleep

        self.assertEqual(
            led_attempts,
            [
                ("blink", 255, 0, 0),
                ("fill", 0, 255, 0),
            ],
        )
        self.assertGreaterEqual(clock[0], 20.0)
        self.assertTrue(any(
            event == "charging_done"
            for event, _data in mission.log.events
        ))

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

    def test_led_and_station_notification_failures_are_non_fatal(self):
        mission = self.make_mission("uav2")
        led_attempts = []

        def broken_led(*args):
            led_attempts.append(args)
            raise RuntimeError("LED unavailable")

        mission.led = broken_led
        self.assertFalse(mission.try_led("blink", 255, 0, 0))
        self.assertEqual(len(led_attempts), 3)

        mission.half_red_blue = lambda: (_ for _ in ()).throw(
            RuntimeError("set_leds unavailable")
        )
        self.assertFalse(mission.try_half_red_blue())

        mission.notify_station = lambda _event: (_ for _ in ()).throw(
            OSError("network unavailable")
        )
        self.assertFalse(mission.try_notify_station("LANDED"))

        warning_events = {
            event: data for event, data in mission.log.events
            if event.endswith("warning")
        }
        self.assertTrue(warning_events["led_action_warning"][
            "mission_continues"
        ])
        self.assertTrue(warning_events["led_half_red_blue_warning"][
            "mission_continues"
        ])
        self.assertTrue(warning_events["station_notification_warning"][
            "mission_continues"
        ])

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


class LauncherTests(unittest.TestCase):
    def test_ssh_key_installer_copies_only_public_key_and_verifies_login(self):
        installer = (ROOT / "install_ssh_keys.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn('ssh-copy-id -i "$KEY_FILE.pub"', installer)
        self.assertIn("-o BatchMode=yes", installer)
        self.assertNotIn('scp "$KEY_FILE"', installer)

    def test_update_and_launch_validate_remote_files(self):
        updater = (ROOT / "update_all.sh").read_text(encoding="utf-8")
        launcher = (ROOT / "mission.sh").read_text(encoding="utf-8")

        self.assertIn("sha256sum -c .energy-race-files.sha256", updater)
        self.assertIn("python3 -m py_compile", updater)
        self.assertIn("python3 -m json.tool", updater)
        self.assertIn(".energy-race-files.sha256", launcher)
        self.assertIn("sha256sum -c", launcher)
        self.assertIn("python3 -m py_compile", launcher)
        self.assertIn("python3 -m json.tool", launcher)

    def test_remote_uav_launch_loads_interactive_ros_environment(self):
        launcher = (ROOT / "mission.sh").read_text(encoding="utf-8")

        self.assertIn(
            'printf -v launch_command \'exec python3 -u %q\' "$script"',
            launcher,
        )
        self.assertIn('nohup bash -ic "$launch_command"', launcher)

    def test_launch_and_stop_quote_remote_arguments(self):
        launcher = (ROOT / "mission.sh").read_text(encoding="utf-8")
        stopper = (ROOT / "stop_mission.sh").read_text(encoding="utf-8")

        self.assertIn("printf -v remote_command", launcher)
        self.assertIn("printf -v remote_command", stopper)
        self.assertIn('kill -TERM "$pid"', stopper)
        self.assertNotIn('kill -KILL "$pid"', stopper)

    # -- PowerShell (Windows) launcher counterparts --------------------------

    def test_ps1_remote_uav_launch_loads_interactive_ros_environment(self):
        launcher = (ROOT / "mission.ps1").read_text(encoding="utf-8")

        self.assertIn(
            'printf -v launch_command \'exec python3 -u %q\' "$script"',
            launcher,
        )
        self.assertIn('nohup bash -ic "$launch_command"', launcher)

    def test_ps1_launch_and_stop_quote_remote_arguments(self):
        launcher = (ROOT / "mission.ps1").read_text(encoding="utf-8")
        stopper = (ROOT / "stop_mission.ps1").read_text(encoding="utf-8")

        # The remote bash snippets are embedded in here-strings; the same
        # safety invariants must hold.
        self.assertIn('kill -TERM "$pid"', stopper)
        self.assertNotIn('kill -KILL "$pid"', stopper)

    def test_ps1_scripts_normalise_crlf_before_ssh(self):
        """CRLF would break the remote bash; every .ps1 must strip CR."""
        for name in ("mission.ps1", "stop_mission.ps1", "update_all.ps1"):
            text = (ROOT / name).read_text(encoding="utf-8")
            self.assertIn('-replace "`r`n", "`n"', text)

    def test_ps1_mission_uses_python_for_local_control(self):
        launcher = (ROOT / "mission.ps1").read_text(encoding="utf-8")

        # Remote boards still use python3, but the local control.py call on
        # Windows must go through the python command found by Find-Python.
        self.assertIn("Find-Python", launcher)
        self.assertIn("control.py", launcher)

    def test_ps1_update_all_runs_local_tests_and_deploys(self):
        updater = (ROOT / "update_all.ps1").read_text(encoding="utf-8")

        self.assertIn("compileall", updater)
        self.assertIn("unittest discover", updater)
        self.assertIn("Deploy-Station", updater)
        self.assertIn("station_id", updater)

    def test_cmd_wrappers_call_powershell_scripts(self):
        for stem in ("mission", "stop_mission", "update_all"):
            wrapper = (ROOT / (stem + ".cmd")).read_text(encoding="utf-8")
            self.assertIn(stem + ".ps1", wrapper)
            self.assertIn("ExecutionPolicy Bypass", wrapper)



class StationConfigTests(unittest.TestCase):
    def test_station_led_shows_invitation_and_charge_cycle(self):
        effects = []
        status_led = object.__new__(STATION.StatusLed)
        status_led.mode = "ros"
        status_led.led = lambda **kwargs: effects.append(kwargs)

        status_led.set("reserved")
        status_led.set("charging")
        status_led.set("charged")

        self.assertEqual(
            effects,
            [
                {"effect": "fill", "r": 0, "g": 0, "b": 255},
                {"effect": "blink", "r": 255, "g": 0, "b": 0},
                {"effect": "fill", "r": 0, "g": 255, "b": 0},
            ],
        )

    def test_landed_starts_station_charge_and_is_idempotent(self):
        station = object.__new__(STATION.Station)
        station.pending = {
            "uav": "uav1",
            "request_id": "request-1",
            "ip": "192.168.0.29",
        }
        station.config = {
            "charge_seconds": 15.0,
            "charge_green_seconds": 5.0,
        }
        station.state = "reserved"
        station.reservation_deadline = 45.0
        station.charging_green_deadline = 0.0
        station.charging_green_active = False
        indications = []
        events = []

        class Led:
            def set(self, state):
                indications.append(state)

        station.led = Led()
        station.log = lambda event, **data: events.append((event, data))
        message = {"uav": "uav1", "request_id": "request-1"}
        original_monotonic = STATION.time.monotonic
        STATION.time.monotonic = lambda: 100.0
        try:
            station.handle_landed(message)
            station.handle_landed(message)
        finally:
            STATION.time.monotonic = original_monotonic

        self.assertEqual(station.state, "occupied")
        self.assertEqual(station.charging_green_deadline, 110.0)
        self.assertFalse(station.charging_green_active)
        self.assertEqual(indications, ["charging"])
        self.assertEqual(
            sum(event == "station_charging_started" for event, _ in events),
            1,
        )

        station.update_charging_indicator(109.9)
        self.assertEqual(indications, ["charging"])
        station.update_charging_indicator(110.0)
        self.assertEqual(indications, ["charging", "charged"])
        self.assertTrue(station.charging_green_active)

    def test_detection_timeout_keeps_station_reserved_for_fallback(self):
        station = object.__new__(STATION.Station)
        station.pending = {
            "uav": "uav1",
            "request_id": "request-1",
            "ip": "192.168.0.29",
        }
        station.config = {"reservation_timeout": 45.0}
        station.state = "pending"
        station.detection_hits = [True]
        events = []
        sent = []

        class Led:
            def set(self, state):
                events.append(("led", state))

        station.led = Led()
        station.log = lambda event, **data: events.append((event, data))
        station.send = lambda event, request, **extra: sent.append(
            (event, request, extra)
        )

        station.reserve_fallback_landing(10.0)

        self.assertEqual(station.state, "reserved")
        self.assertEqual(station.reservation_deadline, 55.0)
        self.assertEqual(station.detection_hits, [])
        self.assertEqual(sent[0][0], "LAND_DENIED")
        self.assertIn(("led", "reserved"), events)
        self.assertTrue(any(
            event == "fallback_landing_reserved"
            for event, _data in events
            if event != "led"
        ))

    def test_calibrations_and_sensitivity(self):
        cases = (
            (5, 0.0030598958333333333, 0.02),
            (37, 0.005338541666666667, 0.02),
        )
        for station_id, calibrated_threshold, threshold_scale in cases:
            directory = ROOT / "station" / "field" / (
                "station-{}".format(station_id)
            ) / "red"
            config = json.loads(
                (directory / "config.json").read_text(encoding="utf-8")
            )
            calibration = json.loads(
                (directory / "calibration.json").read_text(encoding="utf-8")
            )
            STATION.validate(config, calibration)
            self.assertEqual(config["station_id"], station_id)
            self.assertEqual(config["threshold_scale"], threshold_scale)
            self.assertEqual(config["detection_window_frames"], 5)
            self.assertEqual(config["detection_required_hits"], 2)
            self.assertEqual(config["morphology_open_kernel"], 1)
            self.assertEqual(config["request_timeout"], 10.0)
            self.assertEqual(config["charge_seconds"], 15.0)
            self.assertEqual(config["charge_green_seconds"], 5.0)
            self.assertEqual(config["status_led"]["mode"], "ros")
            self.assertEqual(calibration["threshold"], calibrated_threshold)
            self.assertAlmostEqual(
                calibration["threshold"] * config["threshold_scale"],
                calibrated_threshold * threshold_scale,
            )


if __name__ == "__main__":
    unittest.main()
