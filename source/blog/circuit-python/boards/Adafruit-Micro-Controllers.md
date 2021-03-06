---
title: Adafruit Microcontrollers and Circuitpython
date: 20 Nov 2019
---

Adafruit's more advanced micro controllers come with a version of
[MicroPython](http://micropython.org/) installed on them called
 [Circuitpython](https://learn.adafruit.com/welcome-to-circuitpython/what-is-circuitpython).
I have worked with some of these micro controllers and really like them.

## What is it?

CircuitPython is a lean and efficient implementation of the Python 3 programming language that includes a small subset of the Python standard library and is optimised to run on microcontrollers and in constrained environments.

## Documentation

[CircuitPython Latest Docs](https://circuitpython.readthedocs.io/en/latest/docs/index.html)
[Adafruit CircuitPython Library Bundle Docs](https://circuitpython.readthedocs.io/projects/bundle/en/latest/index.html)

## Libraries

There are a lot of libraries available:

- [Adafruit's github repos](https://github.com/adafruit/Adafruit_CircuitPython_Bundle)
- [HCSR-04 drivers](https://github.com/mmabey/CircuitPython_HCSR04)

## Boot

![](boot.png)

## USB Serial Connection to REPL

You can interact with the board's REPL using `screen /dev/tty* 115200`. Linux serial is typically `/dev/ttyACM*` and Apple is `/dev/usbmodem*`. You can stop `screen` with `Ctl-a` then `k`.

## Trinket M0

<img src="Trinket_M0.png" width="500px">
