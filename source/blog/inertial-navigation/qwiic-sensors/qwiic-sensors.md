---
title: Adafruit Qwiic Sensors for Robotics
date: 29 Nov 2020
---

# Accels

| Sensor               | Bias [mg]  | Bits | Noise Density | RMS[mg] @ 100Hz | mg/LSB @ 2G |
|----------------------|------------|------|---------------|-----------------|-------------|
| [NXP_FXOS8700CQ][i1] | 20         | 14   | 126           | 1.11636         | 0.244141    |
| [LSM6DS33][i2]       | 40         | 16   | 90            | 0.797402        | 0.0610352
| [LSM6DSOX][i3]       | 20         | 16   | 70            | 0.620202        | 0.0610352
| [ISM330DHCX][i4]     | Unkn       | 16   | 60            | 0.531601        | 0.0610352
| [LSM9DS1][i5]        | 90         | 16   | 200           | 1.772           | 0.0610352
| [ICM-20649][i6]      | Unkn       | 16   | 285           | 2.52511         | 0.0610352
| [BNO055][i7]         | 80         | 14   | 150           | 1.329           | 0.244141

# Gryos

| Sensor               | Bits  | Noise Density | Drift | RMS[mdps] @ 100Hz | mdps/LSB @ 1000dps | dps @ 26C |
|----------------------|-------|---------------|-------|-------------------|--------------------|-----------|
| [NXP_FXAS21002C][i1] | 16    | 25            | 0.02  | 223.607           | 30.5176            | 0.52 |
| [LSM6DS33][i2]       | 16    | 7             | 0.05  | 62.6099           | 30.5176            | 1.3 |
| [LSM6DSOX][i3]       | 16    | 3.8           | 0.01  | 33.9882           | 30.5176            | 0.26 |
| [ISM330DHCX][i4]     | 16    | 5             | 0.005 | 44.7214           | 30.5176            | 0.13 |

# Pressure

| Sensor       | bits | Sampling (Hz) | Abs Accuracy (Pa) | Rel Accuracy (Pa) | Range (hPa) |
|--------------|------|---------------|-------------------|-------------------|-------------|
| [LPS22][p1]  |      |               | 100               |                   | 260-1260 |
| [DPS310][p2] | 24   | 128           | 100               | 6                 | 300-1200 |
| [BMP390][p3] | 24   | 200           | 50                | 3                 | 300-1250 | 

[Altitude][peqn] can be calculated with:

$$
altitude = 44330 \left(1-\frac{p}{p_0}^{\frac{1}{5.255}}\
$$

where $p_0$ (sea level) is 101325 Pa.

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

![](https://cdn-shop.adafruit.com/970x728/4209-02.jpg)

## Wiring

- Red: 3.3VDC Power
- Black: Ground
- Blue: I2C SDA Data
- Yellow: I2C SCL Clock

# References

- Mide: [Pressure/Altitude calculator](https://www.mide.com/air-pressure-at-altitude-calculator)
- NXP Accel/Magnetometer: [FXOS8700CQ](FXOS8700CQ.pdf)
- NXP Gyro: [FXAS21002](FXAS21002.pdf)
- ST Accel/Gyro [LSM6DSOX](lsm6dsox.pdf)
- ST Accel/Gyro [LSM6DS33](lsm6ds33.pdf)
- ST Accel/Gyro [LSM9DS1 tech notes](TA0343-LSM9DS1-IMU.pdf)
- ST Accel/Gyro [LSM9DS1](lsm9ds1.pdf)
- ST Accel/Gyro [ISM330DHCX](ism330dhcx.pdf)
- InvenSense Accel/Gyro [ICM-20649](icm-20649.pdf)
- ST Magnetometer [LIS3MDL](lis3mdl.pdf)


[i1]: https://www.adafruit.com/product/3463
[i2]: https://www.adafruit.com/product/4485
[i3]: https://www.adafruit.com/product/4517
[i4]: https://www.adafruit.com/product/4502
[i5]: https://www.adafruit.com/product/4634
[i6]: https://www.adafruit.com/product/4464
[i7]: https://www.adafruit.com/product/4646

[peqn]: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
[p1]: https://www.adafruit.com/product/4633
[p2]: https://www.adafruit.com/product/4494
[p3]: https://www.adafruit.com/product/4816
