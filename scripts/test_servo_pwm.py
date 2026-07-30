#!/usr/bin/env python3

import os
import time
from pathlib import Path


PWM = Path("/sys/class/pwm/pwmchip0/pwm0")
PERIOD = 20_000_000
CENTER = 1_500_000
SIDE_A = 1_400_000
SIDE_B = 1_600_000


def read(name):
    return (PWM / name).read_text(encoding="utf-8").strip()


def write(name, value):
    (PWM / name).write_text(str(value), encoding="utf-8")


if os.geteuid() != 0:
    raise SystemExit("Запусти: sudo python3 test_servo_pwm.py")

if not PWM.exists():
    raise SystemExit("{} не найден".format(PWM))

period = int(read("period"))
if period != PERIOD:
    raise SystemExit(
        "Неожиданный period {} ns, ожидался {} ns".format(period, PERIOD)
    )

print("PWM:", PWM)
print("period:", period)
print("polarity:", read("polarity"))
print("enable:", read("enable"))
print("duty_cycle:", read("duty_cycle"))
print()
print("Сними нагрузку с серво и освободи ход качалки.")

if input("Для небольшого тестового движения введи SERVO: ").strip() != "SERVO":
    raise SystemExit("Тест отменён")

try:
    write("duty_cycle", CENTER)
    if read("enable") != "1":
        write("enable", 1)
    time.sleep(1.0)

    print("Сторона A:", SIDE_A)
    write("duty_cycle", SIDE_A)
    time.sleep(1.0)

    print("Сторона B:", SIDE_B)
    write("duty_cycle", SIDE_B)
    time.sleep(1.0)

    print("Центр:", CENTER)
    write("duty_cycle", CENTER)
    time.sleep(1.0)
finally:
    try:
        write("duty_cycle", CENTER)
        time.sleep(0.3)
    finally:
        if read("enable") == "1":
            write("enable", 0)

print("Готово. PWM выключен, серво освобождён.")
