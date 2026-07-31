#!/usr/bin/env python3

import time

import pigpio


PIN = 27
CENTER = 1500
DOWN = 1300
UP = 1700


print("MG995: коричневый=GND, красный=5-6 В, жёлтый/оранжевый=PWM_0.")
print("GND питания серво и GND Orange Pi должны быть общими.")
print("Сними груз и освободи ход палочки.")

if input("Для теста введи SERVO: ").strip() != "SERVO":
    raise SystemExit("Тест отменён")

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit(
        "pigpiod недоступен. Запусти: sudo systemctl enable --now pigpiod.service"
    )

try:
    pi.set_mode(PIN, pigpio.OUTPUT)
    for name, pulse in (
        ("центр", CENTER),
        ("вниз", DOWN),
        ("вверх", UP),
        ("центр", CENTER),
    ):
        result = pi.set_servo_pulsewidth(PIN, pulse)
        if result < 0:
            raise RuntimeError(
                "pigpio не установил импульс {} мкс: код {}".format(
                    pulse, result
                )
            )
        print("{}: {} мкс".format(name, pulse), flush=True)
        time.sleep(2.0)
finally:
    try:
        pi.set_servo_pulsewidth(PIN, 0)
    finally:
        pi.stop()

print("Готово. Управляющие импульсы выключены.")
