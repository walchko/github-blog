---
title: Software Serial on RPi
date: 7 Sep 2016
image: "https://i.pinimg.com/564x/06/76/22/0676228c27bd95349738980b5d016179.jpg"
---

If you need an extra serial port, you do a SW serial port
using `pigpio` like this:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*
import pigpio

RX = 23

pi = pigpio.pi()
pi.set_mode(RX, pigpio.INPUT)
pi.bb_serial_read_open(RX, 115200)

try:
    while True:
        (count, recv) = pi.bb_serial_read(RX)
except:
    pi.bb_serial_read_close(RX)
    pi.stop()
```
