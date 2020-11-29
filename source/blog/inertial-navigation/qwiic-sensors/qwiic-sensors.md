---
title: Adafruit Qwiic Sensors for Robotics
date: 29 Nov 2020
---

# Accels

| Sensor               | Bias  | Bits | Noise Density | RMS[mg] @ 100Hz | mg/LSB @ 2G |
|----------------------|-------|------|---------------|-----------------|-------------|
| [NXP_FXOS8700CQ][i1] | 20    | 14   | 126           | 1.11636         | 0.244141    |
| [LSM6DS33][i2]       | 40    | 16   | 90            | 0.797402        | 0.0610352
| [LSM6DSOX][i3]       | 20    | 16   | 70            | 0.620202        | 0.0610352
| [ISM330DHCX][i4]     | None  | 16   | 60            | 0.531601        | 0.0610352
| [LSM9DS1][i5]        | 90    | 16   | 200           | 1.772           | 0.0610352
| [ICM-20649][i6]      | None  | 16   | 285           | 2.52511         | 0.0610352
| [BNO055][i7]         | 80    | 14   | 150           | 1.329           | 0.244141

# Gryos

| Sensor               | Bits  | Noise Density | Drift | RMS[mdps] @ 100Hz | mdps/LSB @ 1000dps | dps @ 26C |
|----------------------|-------|---------------|-------|-------------------|--------------------|-----------|
| [NXP_FXAS21002C][i1] | 16    | 25            | 0.02  | 223.607           | 30.5176            | 0.52 |
| [LSM6DS33][i2]       | 16    | 7             | 0.05  | 62.6099           | 30.5176            | 1.3 |
| [LSM6DSOX][i3]       | 16    | 3.8           | 0.01  | 33.9882           | 30.5176            | 0.26 |
| [ISM330DHCX][i4]     | 16    | 5             | 0.005 | 44.7214           | 30.5176            | 0.13 |

# Pressure

| Sensor       | Absolute Measurement (Pa)/(m) | Range (hPa) |
|--------------|-------------------------------|-------------|
| [LPS22][p1]  | 100 / 8.6                     | 260-1260 |
| [DPS310][p2] | .2 / 0.02                     | 300-1200 |

*Note:* 

- sea level is 1013.25 hPa
- 0.1 Pa is an accuracy of 10mm (1cm)

## Terms

Power Spectral Density:

- Power: mean-squared value
- Spectral: distribution of a signal over a spectrum of frequency
- Density: mangitude of the PSD is normalized to a single Hz bandwidth

$$
RMS_{noise} = NoiseDensity \sqrt{BW * filter}
$$

where $bandwidth$ is half the Output Data Rate (ODR), $filter$ is a low pass filter having one of the following values:

- 1.57(1st order)
- 1.11 (2nd order)
- 1.05(3rd order)

# References

- Mide: [Pressure/Altitude calculator](https://www.mide.com/air-pressure-at-altitude-calculator)
 NXP Accel/Magnetometer: FXOS8700CQ
 NXP Gyro: FXAS21002
ST Accel/Gyro LSM6DSOX
ST Accel/Gyro LSM6DS33
ST Accel/Gyro LSM9DS1 tech notes
ST Accel/Gyro LSM9DS1
ST Accel/Gyro ISM330DHCX
InvenSense Accel/Gyro ICM-20649
ST Magnetometer LIS3MDL


[i1]: https://www.adafruit.com/product/3463
[i2]: https://www.adafruit.com/product/4485
[i3]: https://www.adafruit.com/product/4517
[i4]: https://www.adafruit.com/product/4502
[i5]: https://www.adafruit.com/product/4634
[i6]: https://www.adafruit.com/product/4464
[i7]: https://www.adafruit.com/product/4646

[p1]: https://www.adafruit.com/product/4633
[p2]: https://www.adafruit.com/product/4494
