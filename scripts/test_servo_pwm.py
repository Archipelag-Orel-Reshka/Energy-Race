#!/usr/bin/env python3

import os
import time
from pathlib import Path


PWM = Path("/sys/class/pwm/pwmchip0/pwm0")
PERIOD = 20_000_000
CENTER = 1_500_000
DOWN = 1_300_000
UP = 1_700_000


print("MG995: коричневый=GND, красный=5-6 В, жёлтый/оранжевый=PWM_0.")
print("GND питания серво и GND Orange Pi должны быть общими.")
print("Сними груз и освободи ход палочки.")

if os.geteuid() != 0:
    raise SystemExit("Запусти: sudo python3 servo.py")

if input("Для теста введи SERVO: ").strip() != "SERVO":
    raise SystemExit("Тест отменён")

if not PWM.exists():
    raise SystemExit("{} не найден".format(PWM))


def read(name):
    return (PWM / name).read_text(encoding="utf-8").strip()


def write(name, value):
    (PWM / name).write_text(str(value), encoding="utf-8")


try:
    if read("enable") == "1":
        write("enable", 0)
    if read("polarity") != "normal":
        write("polarity", "normal")
    if int(read("period")) != PERIOD:
        write("period", PERIOD)
    write("duty_cycle", CENTER)
    write("enable", 1)

    print(
        "PWM включён: period={}, polarity={}, enable={}".format(
            read("period"), read("polarity"), read("enable")
        )
    )
    for name, pulse in (
        ("центр", CENTER),
        ("вниз", DOWN),
        ("вверх", UP),
        ("центр", CENTER),
    ):
        write("duty_cycle", pulse)
        print("{}: {} нс".format(name, pulse), flush=True)
        time.sleep(2.0)
finally:
    try:
        write("duty_cycle", CENTER)
        time.sleep(0.3)
    finally:
        if read("enable") == "1":
            write("enable", 0)

print("Готово. Управляющие импульсы выключены.")
