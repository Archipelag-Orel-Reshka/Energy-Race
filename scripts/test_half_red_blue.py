#!/usr/bin/env python3

import sys
import time

import rospy
from led_msgs.msg import LEDState, LEDStateArray
from led_msgs.srv import SetLEDs
from technic.srv import GetTelemetry, SetLEDEffect


TEST_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0


def main():
    rospy.init_node("energy_race_test_half_red_blue")
    for service in (
        "get_telemetry",
        "led/set_effect",
        "led/set_leds",
    ):
        rospy.loginfo("waiting for %s", service)
        rospy.wait_for_service(service, timeout=10)

    get_telemetry = rospy.ServiceProxy("get_telemetry", GetTelemetry)
    set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)
    set_leds = rospy.ServiceProxy("led/set_leds", SetLEDs)

    telemetry = get_telemetry()
    if telemetry.armed:
        raise RuntimeError("БВС armed: тест LED разрешён только на земле")

    state = rospy.wait_for_message(
        "led/state", LEDStateArray, timeout=5
    )
    if not state.leds:
        raise RuntimeError("led/state не содержит светодиодов")

    middle = len(state.leds) // 2
    colors = []
    for position, current in enumerate(state.leds):
        if position < middle:
            colors.append(LEDState(current.index, 255, 0, 0))
        else:
            colors.append(LEDState(current.index, 0, 0, 255))

    try:
        set_leds(colors)
        print(
            "LED TEST: {} красных + {} синих, БВС disarmed".format(
                middle,
                len(colors) - middle,
            ),
            flush=True,
        )
        deadline = time.monotonic() + TEST_SECONDS
        while time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.1)
    finally:
        set_effect(effect="fill", r=0, g=0, b=0)
        print("LED TEST: лента выключена", flush=True)


if __name__ == "__main__":
    main()
