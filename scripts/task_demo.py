#!/usr/bin/env python3

import math
import time

import rospy
from aruco_pose.msg import MarkerArray
from std_srvs.srv import Trigger
from technic.srv import GetTelemetry, Navigate, SetLEDEffect


ALTITUDE = 1.3
SPEED = 0.35
ARRIVAL_TIMEOUT = 35.0
MARKER_TIMEOUT = 8.0


rospy.init_node("energy_race_task_demo")

for service in ("get_telemetry", "navigate", "land", "led/set_effect"):
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
            raise RuntimeError(f"ArUco {marker_id} is not detected")
        rospy.sleep(0.1)


def wait_any_marker():
    deadline = time.monotonic() + MARKER_TIMEOUT
    while not rospy.is_shutdown():
        if visible_markers:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("ArUco map is not detected after takeoff")
        rospy.sleep(0.1)


def land_wait():
    land()
    deadline = time.monotonic() + 30.0
    while get_telemetry().armed:
        if time.monotonic() >= deadline:
            raise RuntimeError("landing timeout")
        rospy.sleep(0.2)


telemetry = get_telemetry()
if not telemetry.connected:
    raise RuntimeError("PX4 is not connected")

print("Поставь дрон на H в позиции метки 48 и освободи поле.")
if input("Для запуска введи FLY: ").strip() != "FLY":
    raise SystemExit("Полёт отменён")

flight_active = False

try:
    led("fill", 255, 255, 255)
    flight_active = True
    navigate_wait(0.0, 0.0, ALTITUDE, "body", auto_arm=True)

    wait_any_marker()

    navigate_wait(6.0, 3.0, ALTITUDE, "aruco_map")
    wait_marker(27)
    led("blink", 0, 255, 0)
    rospy.sleep(1.0)

    led("fill", 255, 255, 255)
    navigate_wait(2.0, 3.0, ALTITUDE, "aruco_map")
    wait_marker(23)
    led("blink", 255, 0, 0)
    rospy.sleep(1.0)

    led("fill", 255, 255, 255)
    navigate_wait(6.0, 0.0, ALTITUDE, "aruco_map")
    led("fill", 255, 255, 0)
    land_wait()
    flight_active = False
finally:
    if flight_active:
        rospy.logwarn("emergency landing")
        try:
            led("fill", 255, 255, 0)
            land_wait()
        except Exception as error:
            rospy.logerr("landing failed: %s", error)
