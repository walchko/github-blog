---
title: IR Camera with Trinket M0 and CircuitPython
date: 11 Aug 2019
image: "https://cdn-learn.adafruit.com/assets/assets/000/043/014/medium800/temperature_AMG8833.jpg?1498492692"
---

Install the needed libraries in your `library` folder:

- adafruit_amg88xx.mpy
- adafruit_bus_device

Hook up the camera. *Note:* the `INT` pi is an interrupt that will trigger when the scene has changed

![](https://cdn-learn.adafruit.com/assets/assets/000/059/229/medium640/temperature___humidity_FeatherM0_AMG8833_bb.jpg?1534369163)

```python
import time
import busio
import board
import adafruit_amg88xx

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

while True:
    for row in amg.pixels:
        # Pad to 1 decimal place
        print(['{0:.1f}'.format(temp) for temp in row])
        print("")
    print("\n")
    time.sleep(1)
```

## Linux

To see if the camera is hooked up, do:

```
sudo i2cdetect -y 1
```

# References

- Adafruit AMG8833 8x8 Thermal Camera [Tutorial](https://learn.adafruit.com/adafruit-amg8833-8x8-thermal-camera-sensor?view=all)
