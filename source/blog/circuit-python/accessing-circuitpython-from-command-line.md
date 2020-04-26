---
title: Accessing CircuitPython from the Command Line
date: 20 Nov 2019
---

## Ampy

[ampy](https://github.com/scientifichackers/ampy) Adafruit MicroPython Tool (ampy) - Utility 
to interact with a CircuitPython or MicroPython board over a serial connection.

## Screen

You can interact with the board’s REPL using `screen /dev/tty* 115200`. 

Serial ports: 

- Linux: `/dev/ttyACM*`
- Apple: `/dev/usbmodem*` 

You can stop screen with `Ctl-a` then `k`.
