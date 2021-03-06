---
title: PWM for Servo and Motor Controll
date: 10 Dec 2019
---

```python
import time
import board
from adafruit_motor import servo

pwm = pulseio.PWMOut(board.A2, duty_cycle=2 ** 15, frequency=50)

# you can change the defaults:
#   range: 0-135
# most servos do 0-180
s = servo.Servo(
    pwm, 
    min_pulse=700, 
    max_pulse=2100,
    actuation_range=180
)

for angle in range(0, 180, 5):  # 0 - 180 degrees, 5 degrees at a time.
    s.angle = angle
    time.sleep(0.1)

```
