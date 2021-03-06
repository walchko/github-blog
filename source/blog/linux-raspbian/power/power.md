---
title: Power
date: 1 June 2019
---

## Power LED

The newer Pi(3/2/B+) have a voltage monitor chip (APX803) which triggers at 4.63±0.07V. The Pi3B+ uses a MxL7704 chip to manage power, which has the same nominal trigger point. This controls the Red Power LED.

If the Red Power LED is not illuminated this means the supply voltage is inadequate. (The newer Pi have a well engineered power circuit, and may continue to function even if the input voltage is below spec; the same may not be true of peripherals). The GUI had an rainbow indicator (replaced by a lightning bolt) which comes up in the top right if the voltage is inadequate. This has a 3 second timer, and may display even if the LED appears to be lit.

NOTE the Red Power LED on the Pi3B+ only functions if the SD Card/USB key has up to date firmware because it is controlled by software - it is meaningless otherwise.

## Powering Through Header

[You can power through the header](https://codeload.github.com/raspberrypi/hats/zip/master)

> "It is possible to power the Pi by supplying 5V through the GPIO (sic)
> header pins 2,4 and GND. The acceptable input voltage range is 5V ±5%. ...
> Implement a duplicate power safety diode ... supply 5V at a minimum of 1.3A
> ... Under no circumstances should a power source be connected to the 3.3V pins."


## USB Power

The Pi(3/2/B+) USB Current are supplied through a Current-Limited Power Switch
(AP2553?) (U13), although this is not shown on the published schematics. The
default for 2/B+ is 600mA which can be doubled by setting `max_usb_current=1`
in `/boot/config.txt`.


- [Reference](https://raspberrypi.stackexchange.com/questions/51615/raspberry-pi-power-limitations)
