#!/usr/bin/env python3

import math
import time

import rospy
from aruco_pose.msg import MarkerArray
from std_srvs.srv import Trigger
from technic.srv import GetTelemetry, Navigate, SetLEDEffect


ALTITUDE = 2
SPEED = 0.35
ARRIVAL_TOLERANCE = 0.25
ARRIVAL_TIMEOUT = 35.0

MARKER_CYCLE = [
    47, 46, 45, 44, 43, 42, 35, 28, 21, 14, 7, 0,
    1, 8, 15, 22, 29, 36, 37, 30, 23, 16, 9, 2,
    3, 10, 17, 24, 31, 38, 39, 32, 25, 18, 11, 4,
    5, 6, 13, 12, 19, 20, 27, 26, 33, 34, 41, 40,
]


def marker_position(marker_id):
    row, column = divmod(marker_id, 7)
    return float(column), float(6 - row)


rospy.init_node("energy_race_aruco_test")

for service in ("get_telemetry", "navigate", "land", "led/set_effect"):
    rospy.loginfo("waiting for %s", service)
    rospy.wait_for_service(service, timeout=10)

get_telemetry = rospy.ServiceProxy("get_telemetry", GetTelemetry)
navigate = rospy.ServiceProxy("navigate", Navigate)
land = rospy.ServiceProxy("land", Trigger)
set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

detected = set()


def markers_callback(message):
    new_ids = sorted(
        marker.id for marker in message.markers if marker.id not in detected
    )
    if not new_ids:
        return

    detected.update(new_ids)
    rospy.loginfo("detected ArUco: %s", new_ids)
    try:
        set_effect(effect="blink", r=0, g=255, b=0)
    except rospy.ServiceException as error:
        rospy.logwarn("LED failed: %s", error)


rospy.Subscriber("aruco_detect/markers", MarkerArray, markers_callback)


def navigate_wait(
    x=0.0,
    y=0.0,
    z=0.0,
    frame_id="",
    auto_arm=False,
):
    navigate(
        x=x,
        y=y,
        z=z,
        frame_id=frame_id,
        speed=SPEED,
        auto_arm=auto_arm,
    )
    deadline = time.monotonic() + ARRIVAL_TIMEOUT

    while not rospy.is_shutdown():
        telemetry = get_telemetry(frame_id="navigate_target")
        distance = math.sqrt(
            telemetry.x ** 2 + telemetry.y ** 2 + telemetry.z ** 2
        )
        if math.isfinite(distance) and distance < ARRIVAL_TOLERANCE:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("navigation timeout")
        rospy.sleep(0.2)


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


telemetry = get_telemetry()
if hasattr(telemetry, "connected") and not telemetry.connected:
    raise RuntimeError("PX4 is not connected")

home_x, home_y = marker_position(48)
if input("Send FLY: ").strip() != "FLY":
    raise SystemExit("denied")

flight_active = False

try:
    flight_active = True
    navigate_wait(z=ALTITUDE, frame_id="body", auto_arm=True)

    first_marker_deadline = time.monotonic() + 10.0
    while not detected and time.monotonic() < first_marker_deadline:
        rospy.sleep(0.1)
    if not detected:
        raise RuntimeError("aruco in not detect")

    for marker_id in MARKER_CYCLE:
        x, y = marker_position(marker_id)
        rospy.loginfo("marker %d: x=%.1f y=%.1f", marker_id, x, y)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id="aruco_map")
        rospy.sleep(0.3)

    navigate_wait(
        x=home_x,
        y=home_y,
        z=ALTITUDE,
        frame_id="aruco_map",
    )
    rospy.loginfo(
        "detected %d/48 visible markers (48 is covered by H)",
        len(detected.intersection(MARKER_CYCLE)),
    )
    land_wait()
    flight_active = False
finally:
    if flight_active or is_armed():
        rospy.logwarn("emergency landing")
        try:
            land_wait()
        except Exception as error:
            rospy.logerr("landing failed: %s", error)
