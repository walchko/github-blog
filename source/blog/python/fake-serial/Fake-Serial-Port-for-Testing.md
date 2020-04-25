---
title: Creating a Fake Serial Port for Testing
---

## `pyserial`

This is cross platform as far as I have found

```python
from serial import serial_for_url

loop_addr = 'loop://'
serial = serial_for_url(loop_addr, timeout=0.1)

serial.write(b"hi")   # send hi
ans = serial.read(10) # returns the same hi
```

## `pty`

You can create a fake serial port you can read/write on **Unix/Linux** by:

```python
import pty
import os
from serial import Serial

master, slave = pty.openpty()
# os.read(master,1000)
port = os.ttyname(slave)
serial = Serial(port)
```
