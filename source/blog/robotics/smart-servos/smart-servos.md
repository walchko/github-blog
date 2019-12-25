---
title: Robotic Smart Servos
date: 22 Dec 2019
---

There are various smart servos out there now, I am use to using
Dynamixel servos.

| Name | Price | Torque | Voltage | Range | Comm | Link |
|------|-------|--------|---------|-------|------|------|
| AX-12A | $44.90 | 1.5 Nm | 9-12V  | 300 | 1Mbps     | [Robotis](http://www.robotis.us/ax-series/) |
| LX-16A | $17    | 1.6 Nm | 6-8.4V | 240 | 115.2kbps | [Lewansoul](http://www.lewansoul.com/product/detail-17.html) |
| LX-15D | $22    | 1.6 Nm | 6-8.4V | 240 | 115.2kpbs | [Lewansoul](http://www.lewansoul.com/product/detail-7.html) |

- both all are capable of continous rotation
- both HW and SW communications protocols are very very similar to Dynamixel Ver 1.0 used by the AX-12A
    - Half-duplex UART asynchronous serial interface


## Thoughts

| Feature | LX-xx | Dynamixel |
|---|---|---|
| **servo horn** | self tapping screws         | M2 machine screws into nuts |
| **servo horn** | standard metal servo spline | plastic non-standard servo spline that is keyed | 
| **wiring**     | plastic coated wires        | silicon coated wires (higher current) |
| **connectors** | LX-16A same as AX-12A, LX-15D unknown | AX-12A same as LX-16A |
| **datarate**   | 115.2kbps, standard, slower | 1Mbps, non-standard, fast, can be set to a slower rate if needed |
| **mounting**   | self tapping screws, 10pts  | M2 machine screw and nut, 20 pts |
| **drivers**    | LX-16A many python and very similar to Dynamixel, LX-15D unknown | lots of C++ and python drivers |

## Python Software Drivers

- [PyLX-16A github](https://github.com/ethanlipson/PyLX-16A)
- [lewansoul-lx16a](https://github.com/maximkulkin/lewansoul-lx16a) which you can get from [pypi](https://pypi.org/project/lewansoul-lx16a/) using `pip`
- [SGVHAK rover](https://github.com/Roger-random/SGVHAK_Rover) has an lx-16a driver in its source code
