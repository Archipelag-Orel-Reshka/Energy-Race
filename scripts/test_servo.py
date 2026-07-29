#!/usr/bin/env python3

import subprocess
import time


TOPIC = "/model/px4/command/gripper"


def set_servo(position):
    subprocess.run([
        "gz", "topic",
        "-t", TOPIC,
        "-m", "gz.msgs.Double",
        "-p", f"data: {float(position)}",
    ], check=True)


print("close")
set_servo(-0.9)
time.sleep(2)

print("open")
set_servo(0.0)
