---
title: Adafruit AHRS
date: 16 Aug 2020
---

Install with: Tools -> Manage Libraries -> Adafruit AHRS

This Adafruit library written for Arduino (C++) supplies:

- **Mahony:** This basic but effective algorithm will run on smaller chips like the '328p which makes it a great one for any platform.
- **Madgwick:** This algorithm is very popular when you have faster Cortex M0, M3, M4 or faster chips. It isn't going to run on an atmega328p
- **NXP Sensor Fusion:** This really nice fusion algorithm was designed by NXP and requires a bit of RAM (so it isnt for a '328p Arduino) but it has great output results.

## Web Visualizer

![](https://cdn-learn.adafruit.com/assets/assets/000/088/487/original/sensors_99.gif?1581956335)

Any uC that outputs Yaw (Z) Pitch (Y) Roll (X) as `Orientation: 163.00 -4.90 33.56` can use it. Note, all axes only have the range of (-180,180] which is fine for roll and pitch, but surprises me for yaw (heading).

- Visit [`chrome://flags`](chrome://flags) from within Chrome, then find and enable the **experimental web platform features**

![](https://cdn-learn.adafruit.com/assets/assets/000/088/479/medium640/sensors_image.png?1581901164)

- Select serial port and set it to 115200
  - Trinket M0, linux: `/dev/serial/by-id/usb-Adafruit_Trinket_M0_F42D3DEC504C5430372E314AFF090732-if00`

# References

- Adafruit Learn: [How to Fuse Motion Sensor Data into AHRS Orientation (Euler/Quaternions)](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions?view=all)
