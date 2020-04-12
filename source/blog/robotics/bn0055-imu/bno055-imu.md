---
title: Adafruit BNO055 (IMU)
date: 2016-03-27
---

![](bno055.jpg)

## Warning

Adafruit's [Absolute Orientation
IMU](https://www.adafruit.com/products/2472) includes a Attitude Heading
Reference System (AHRS) which always determines the orientation of the
IMU. It has both serial and I2C output and a serial port output. This
sensor has two issues:

1.  For I2C, it often does clock stretching which the RPi doesn't like,
    so you have to use the serial port
2.  On the RPi v3, the hardware serial port is attached to the bluetooth
    module, so you either have to turn off bluetooth or use a
    usb-to-serial converter. Also, you have to tell the RPi to not
    connect a login terminal to the serial port.

Also note, the imu is built for cell phones and uses either the Andriod
or Windows phone coordinate system. This results in the AHRS reporting
values different from traditional aerospace coordinate systems.

|       | Phone  | INS    |
| ----- | ------ | ------ |
| Roll  | y-axis | x-axis |
| Pitch | x-axis | y-axis |

## Specs

The BNO055 can output the following sensor data:

  - Default Address: `0x28` or can be changed to `0x29` via the ADR pin
  - Absolute Orientation (Euler Vector, 100Hz) Three axis orientation
    data based on a 360° sphere
  - Absolute Orientation (Quaterion, 100Hz) Four point quaternion output
    for more accurate data manipulation
  - Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in
    (rad/s)
  - Acceleration Vector (100Hz) Three axis of acceleration (gravity +
    linear motion) in (m/s^2)
  - Magnetic Field Strength Vector (20Hz) Three axis of magnetic field
    sensing in micro Tesla (uT)
  - Linear Acceleration Vector (100Hz) Three axis of linear acceleration
    data (acceleration minus gravity) in (m/s^2)
  - Gravity Vector (100Hz) Three axis of gravitational acceleration
    (minus any movement) in (m/s^2)
  - Temperature (1Hz) Ambient temperature in degrees Celsius
