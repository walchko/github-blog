---
title: Using Python to Talk with a PS4 Controller over USB
date: 4 Jan 2020
---

![](https://github.com/walchko/github-blog/raw/master/source/blog/linux-raspbian/pics/ps4.jpg)

Basically take a look at my code [here](TBD). This will allow you to interact
with `/dev/js0` or any other linux joystick file descriptor like `/dev/input/by-id/Sony-xxx`
when the joystick is connected via USB. I have not tried it using bluetooth yet. This
uses standard linux/unix `fcntl`/`ioctl` to open the file descriptor and read the driver.

**Tested on OS:** Ubuntu 19.10

## Axes/Button Values

- Left/Right Stick Axis are float16 [-1.0, 1.0]:
    - y-axis: up -1, down 1
    - x-axis: left -1, right 1
- Left/Right Trigger Axes float16 [-1.0,1.0]
    - These also give boolean button responses of [0,1]
    - pushed 1, released -1 (0.0 is somewhere in between)
- Buttons uints16 [0,1]:
    - all buttons: pushed 1, released 0
