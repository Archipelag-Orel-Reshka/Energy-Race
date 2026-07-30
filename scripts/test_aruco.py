#!/usr/bin/env python3

import math
import time

import rospy
from aruco_pose.msg import MarkerArray
from clover.srv import GetTelemetry, Navigate, SetLEDEffect
from std_srvs.srv import Trigger


ALTITUDE = 1.3
SPEED = 0.35
ARRIVAL_TOLERANCE = 0.25
ARRIVAL_TIMEOUT = 35.0


def marker_position(marker_id):
    row, column = divmod(marker_id, 7)
    return float(column), float(6 - row)


def make_route():
    route = []
    for row in range(6, -1, -1):
        markers = list(range(row * 7, row * 7 + 7))
        route.extend(reversed(markers) if row % 2 == 0 else markers)
    return route


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

print("Поставь дрон над маркером 48, освободи поле и оставь пульт включённым.")
if input("Для взлёта введи FLY: ").strip() != "FLY":
    raise SystemExit("Полёт отменён")

flight_active = False

try:
    flight_active = True
    navigate_wait(z=ALTITUDE, frame_id="body", auto_arm=True)

    first_marker_deadline = time.monotonic() + 10.0
    while not detected and time.monotonic() < first_marker_deadline:
        rospy.sleep(0.1)
    if not detected:
        raise RuntimeError("ArUco is not detected after takeoff")

    for marker_id in make_route():
        x, y = marker_position(marker_id)
        rospy.loginfo("marker %d: x=%.1f y=%.1f", marker_id, x, y)
        navigate_wait(x=x, y=y, z=ALTITUDE, frame_id="aruco_map")
        rospy.sleep(0.3)

    navigate_wait(x=6.0, y=0.0, z=ALTITUDE, frame_id="aruco_map")
    rospy.loginfo("detected %d/49 markers", len(detected))
    land_wait()
    flight_active = False
finally:
    if flight_active or is_armed():
        rospy.logwarn("emergency landing")
        try:
            land_wait()
        except Exception as error:
            rospy.logerr("landing failed: %s", error)
