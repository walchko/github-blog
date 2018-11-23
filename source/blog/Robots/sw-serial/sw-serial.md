# Software Serial on RPi

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
