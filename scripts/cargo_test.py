#!/usr/bin/env python3

import math
import time

import rospy
from aruco_pose.msg import MarkerArray
from std_srvs.srv import Trigger
from technic.srv import GetTelemetry, Navigate, SetLEDEffect


ALTITUDE = 2.0
SPEED = 0.3
ARRIVAL_TIMEOUT = 45.0
MARKER_TIMEOUT = 10.0

HOME = (6.0, 3.0)   # ArUco 27
CARGO = (0.0, 6.0)  # ArUco 0

rospy.init_node("energy_race_cargo_test")

for service in ("get_telemetry", "navigate", "land", "led/set_effect"):
    rospy.loginfo("waiting for %s", service)
    rospy.wait_for_service(service, timeout=10)

get_telemetry = rospy.ServiceProxy("get_telemetry", GetTelemetry)
navigate = rospy.ServiceProxy("navigate", Navigate)
land = rospy.ServiceProxy("land", Trigger)
set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

visible_markers = set()


def markers_callback(message):
    global visible_markers
    visible_markers = {marker.id for marker in message.markers}


rospy.Subscriber("aruco_detect/markers", MarkerArray, markers_callback)


def led(effect, r, g, b):
    set_effect(effect=effect, r=r, g=g, b=b)


def navigate_wait(x, y, z, frame_id, auto_arm=False):
    navigate(
        x=x,
        y=y,
        z=z,
        yaw=float("nan"),
        speed=SPEED,
        frame_id=frame_id,
        auto_arm=auto_arm,
    )
    deadline = time.monotonic() + ARRIVAL_TIMEOUT

    while not rospy.is_shutdown():
        telemetry = get_telemetry(frame_id="navigate_target")
        distance = math.sqrt(
            telemetry.x ** 2 + telemetry.y ** 2 + telemetry.z ** 2
        )
        if math.isfinite(distance) and distance < 0.25:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("navigation timeout")
        rospy.sleep(0.2)


def wait_marker(marker_id):
    deadline = time.monotonic() + MARKER_TIMEOUT
    while not rospy.is_shutdown():
        if marker_id in visible_markers:
            rospy.loginfo("detected ArUco %d", marker_id)
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("ArUco {} is not detected".format(marker_id))
        rospy.sleep(0.1)


def wait_any_marker():
    deadline = time.monotonic() + MARKER_TIMEOUT
    while not rospy.is_shutdown():
        if visible_markers:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("ArUco map is not detected")
        rospy.sleep(0.1)


def land_wait():
    land()
    deadline = time.monotonic() + 30.0
    while get_telemetry().armed:
        if time.monotonic() >= deadline:
            raise RuntimeError("landing timeout")
        rospy.sleep(0.2)


def is_armed():
    try:
        return bool(get_telemetry().armed)
    except rospy.ServiceException:
        return False


def half_red_blue():
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


if not get_telemetry().connected:
    raise RuntimeError("PX4 is not connected")

print("Поставь БВС-2 на ArUco 27 и освободи полётную зону.")
if input("Для запуска введи FLY: ").strip() != "FLY":
    raise SystemExit("Полёт отменён")

try:
    led("blink", 255, 255, 0)
    navigate_wait(0.0, 0.0, ALTITUDE, "body", auto_arm=True)
    wait_any_marker()

    navigate_wait(CARGO[0], CARGO[1], ALTITUDE, "aruco_map")
    wait_marker(0)
    land_wait()
    rospy.loginfo("cargo point: landed and disarmed")

    led("fill", 255, 0, 0)
    print("Можно войти в полётную зону и установить деревянный груз.")
    print("После установки выйди из зоны и проверь, что внутри никого нет.")
    if input("Для взлёта с грузом введи FLY: ").strip() != "FLY":
        raise SystemExit("Продолжение отменено, дрон остаётся disarmed")

    led("fill", 255, 0, 0)
    navigate_wait(0.0, 0.0, ALTITUDE, "body", auto_arm=True)
    wait_any_marker()

    try:
        half_red_blue()
    except Exception as error:
        rospy.logwarn("half red/blue LED failed: %s", error)
    navigate_wait(HOME[0], HOME[1], ALTITUDE, "aruco_map")
    land_wait()
    rospy.loginfo("cargo test completed")
finally:
    if is_armed():
        rospy.logwarn("emergency landing")
        try:
            land_wait()
        except Exception as error:
            rospy.logerr("landing failed: %s", error)
