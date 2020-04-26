---
title: Circuitpython Board LED Meanings
date: 26 Apr 2020
---

[CircuitPython RGB Status Light](https://learn.adafruit.com/welcome-to-circuitpython/troubleshooting#circuitpython-rgb-status-light-20-20)

Nearly all Adafruit CircuitPython-capable boards have a single NeoPixel 
or DotStar RGB LED on the board that indicates the status of CircuitPython. 
A few boards designed before CircuitPython existed, such as the Feather M0 
Basic, do not.

Circuit Playground Express and Circuit Playground Bluefruit have 
multiple RGB LEDs, but do NOT have a status LED. The LEDs are all green 
when in the bootloader. They do NOT indicate any status while running CircuitPython.

Here's what the colors and blinking mean:

- steady **GREEN**: code.py (or code.txt, main.py, or main.txt) is running
- pulsing **GREEN**: code.py (etc.) has finished or does not exist
- steady **YELLOW** at start up: (4.0.0-alpha.5 and newer) CircuitPython is waiting for a reset to indicate that it should start in safe mode
- pulsing **YELLOW**: Circuit Python is in safe mode: it crashed and restarted
- steady **WHITE**: REPL is running
- steady **BLUE**: boot.py is running

Colors with multiple flashes following indicate a Python exception and then 
indicate the line number of the error. The color of the first flash 
indicates the type of error:

- **GREEN**: IndentationError
- **CYAN**: SyntaxError
- **WHITE**: NameError
- **ORANGE**: OSError
- **PURPLE**: ValueError
- **YELLOW**: other error

These are followed by flashes indicating the line number, including place value. WHITE 
flashes are thousands' place, BLUE are hundreds' place, YELLOW are tens' place, and 
CYAN are one's place. So for example, an error on line 32 would flash YELLOW three 
times and then CYAN two times. Zeroes are indicated by an extra-long dark gap.
