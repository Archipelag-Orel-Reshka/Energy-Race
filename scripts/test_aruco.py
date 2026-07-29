#!/usr/bin/env python3

import time

from clover2 import Clover2


ALTITUDE = 1.5
SPEED = 0.4


def marker_position(marker_id):
    row, column = divmod(marker_id, 7)
    return float(column - 6), float(6 - row)

route = []
for row in range(6, -1, -1):
    markers = list(range(row * 7, row * 7 + 7))
    route.extend(reversed(markers) if row % 2 == 0 else markers)

drone = Clover2("test_aruco")
time.sleep(2)

try:
    drone.navigate_wait("base_link", z=ALTITUDE, speed=SPEED)
    time.sleep(2)

    for marker_id in route:
        x, y = marker_position(marker_id)
        print(f"marker {marker_id}: x={x}, y={y}")
        drone.navigate_wait("map", x=x, y=y, z=ALTITUDE, speed=SPEED)
        time.sleep(0.3)

    drone.navigate_wait("map", x=0.0, y=0.0, z=ALTITUDE, speed=SPEED)
finally:
    if drone.is_armed():
        drone.land()
