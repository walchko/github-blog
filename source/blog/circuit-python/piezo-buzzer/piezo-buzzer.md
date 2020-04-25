---
title: Piezo Buzzer
date: 10 Dec 2019
---

```python
import time
import board
import pulseio

class Piezo:
    def __init__(self, pin):
        self.piezo = pulseio.PWMOut(
            pin,
            duty_cycle=0,
            frequency=440,
            variable_frequency=True
        )

    def tone(self, f, duration=0.25):
        self.piezo.frequency = f
        self.piezo.duty_cycle = 65536 // 2  # On 50%
        time.sleep(duration)  # On for 1/4 second
        self.piezo.duty_cycle = 0  # Off
        # time.sleep(0.05)  # Pause between notes

pie = Piezo(board.A2)

for f in (262, 294, 330, 349, 392, 440, 494, 523):
    pie.tone(f)
    time.sleep(0.05)
        
```
