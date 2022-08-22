---
title: Adafruit Qwiic Sensors for Robotics
date: 29 Nov 2020
---

# Accelerometer

| Sensor               | Bias (mg)  | Bits | Noise Density (2G) | RMS(mg) @ 100Hz | mg/LSB @ 2G | I2C (Hz) | Datasheet |
|----------------------|------------|------|---------------|-----------------|------------------|-----------|-----------|
| [NXP_FXOS8700CQ][i1] | 20         | 14   | 126           | 1.11636         | 0.244141    | 100k | [ds][id1]|
| [LSM6DS33][i2] (Discontinued) | 40         | 16   | 90            | 0.797402        | 0.0610352   | 100k | |
| [LSM6DSOX][i3]       | 20         | 16   | 70            | 0.620202        | 0.0610352   | 400k | [ds][id2] |
| [LSM6DS3TR][i8]      | 40         |      | 90            | 1.7             | 0.061       | 400k | |
| [ISM330DHCX][i4]     | 10         | 16   | 60            | 1.8             | 0.0610352   | 1M   | [ds][id4] |
| [LSM9DS1][i5]        | 90         | 16   | 200           | 1.772           | 0.0610352   |      | |
| [ICM-20649][i6]      | Unkn       | 16   | 285           | 2.52511         | 0.0610352   |      | |
| [ICM-20948][i10]     |            | 16   | 230 (10Hz)    |                 |             | 400k | |
| [BNO055][i7]         | 80         | 14   | 150           | 1.329           | 0.244141    |      | |
| [MPU6050][i9]        | 50/80      | 16   | 400           |                 | 16.384      | 400k | |

- Bias: Linear acceleration zero-g level offset accuracy
- Noise Density: $\mu g / \sqrt{Hz}$

[id1]: h
[id2]: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
[id4]: https://www.st.com/resource/en/datasheet/ism330dhcx.pdf

# Orientation

| Sensor               | I2C  | Notes |
|----------------------|------|---|
| [BNO055][o1]         | 0x28 |   |
| [BNO085][o2]         | 0x4A | Vendor sold to new company |

[o1]: https://www.adafruit.com/product/4646
[o2]: https://www.adafruit.com/product/4754

**WARNING:** these use Andriod/iOS definitions of coordinate systems 
and not standard aerospace definitions of the frame (x-forward, y-right wing, z-down)

# Gryoscope

| Sensor               | Bits  | Noise Density | Drift | RMS(mdps) @ 100Hz | mdps/LSB @ 1000dps | Zero |
|----------------------|-------|---------------|-------|-------------------|--------------------|------|
| [NXP_FXAS21002C][i1] | 16    | 25            | 0.02  | 223.607           | 30.5176            | 0.52 |
| [LSM6DS33][i2] (Discontinued) | 16    | 7             | 0.05  | 62.6099           | 30.5176            | 1.3  |
| [LSM6DSOX][i3]       | 16    | 3.8           | 0.01  | 33.9882           | 30.5176            | 0.26 |
| [LSM6DS3TR][i8]      |       | 5             | 0.05  | 75                | 30                 | 0.05 |
| [ISM330DHCX][i4]     | 16    | 5             | 0.005 | 44.7214           | 30.5176            | 0.13 |
| [MPU6050][i9]        | 16    | 0.005 (10Hz)  | 20    | 50                | 32.8               |      |
| [ICM-20948][i10]     | 16    | 15            | 0.05  |                   | 32.8               | 5    |

- Drift: Angular rate typical zero-rate level change vs. temperature (dps/C)
- Zero: Initial Zero Tolerance @ 25C
- Noise Density: $mdps/ \sqrt{Hz}$

[i1]: https://www.adafruit.com/product/3463
[i2]: https://www.adafruit.com/product/4485
[i3]: https://www.adafruit.com/product/4517
[i4]: https://www.adafruit.com/product/4502
[i5]: https://www.adafruit.com/product/4634
[i6]: https://www.adafruit.com/product/4464
[i7]: https://www.adafruit.com/product/4646
[i8]: https://www.adafruit.com/product/4503
[i9]: https://www.adafruit.com/product/3886
[i10]: https://www.adafruit.com/product/4554

# Magnetometer

Earth's magnetic field ranges between 0.25 and 0.65 gauss (25 - 65 $\mu$T)

| Sensor | Bits | Scale( $\pm$ gauss) | RMS(mgauss) | I2C(Hz) | Addr |
|--------|------|-------------------|-------------|---------|------|
|[LIS3MDL][mag1]   | 16 | 4,8,12,16 | 3.2 (@ 12 gauss) | 400k |`0x1C`,`0x1E`|
|[LIS2MDL][mag2]   | 16 | 50        | 3 (w/LPF)        | 3.4M |`0x1E`|
|[MMC5603NJ][mag3] | 20 | 30        | 2 (@ 150Hz)      | 400k |`0x30`|


[mag1]: https://www.adafruit.com/product/4479
[mag2]: https://www.adafruit.com/product/4488
[mag3]: https://www.adafruit.com/product/5579

# Pressure

| Sensor       | bits | Sampling (Hz) | Abs Accuracy (Pa) | Rel Accuracy (Pa) | Range (hPa) | I2C (Hz) |
|--------------|------|---------------|-------------------|-------------------|-------------|-----------|
| [LPS22][p1]  | 24   | 75            | 100               | Unknown           | 260-1260    | 400k |
| [DPS310][p2] | 24   | 128           | 100               | 6 (0.55m)         | 300-1200    | 3.4M |
| [BMP388][p4] | 24   | 200           | 50                | 8 (0.66m)         | 300-1100    | 3.4M |
| [BMP390][p3] | 24   | 200           | 50                | 3 (0.25m)         | 300-1250    | 3.4M |

[Altitude][peqn] can be calculated with:

$$
\begin{align*}
altitude &= \frac{T_0}{L} \left( \frac {p}{P_0} ^ {\frac{-R L}{g_0 M}} - 1.0 \right) \\
         &= 44330 [1-\frac{p}{P_0}^{\frac{1}{5.255}}]
\end{align*}
$$

where 

| Symbol | Description | Value |
|--------|-------------|-------|
| $P_0$ | Pressure at sea level     | 101325 Pa |
| $T_0$ | Temperature at sea level  | 288.15 K |
| $R$   | Universal gas constant    | 8.31446261815324 Nm/(mol K) |
| $M$   | Molar mass of Earth's air | 0.0289644 kg/mol |
| $g_0$ | Gravitational constant    | 9.80665 m/s^2 |
| $L$   | Lapse rate                | -0.0065 K/m |

[ref](https://www.mide.com/air-pressure-at-altitude-calculator)

[peqn]: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
[p1]: https://www.adafruit.com/product/4633
[p2]: https://www.adafruit.com/product/4494
[p3]: https://www.adafruit.com/product/4816
[p4]: https://www.adafruit.com/product/3966

## Miscellanious I2C (QWIIC) Parts

| Device | Part | I2C |
|--------|------|-----|
| LED backpack | [HT16K33][misc1]   | 400k |
| NFC/RFID     | [ST25DV16K][misc2] | 1M   |

[misc1]: https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf
[misc2]: https://cdn-learn.adafruit.com/assets/assets/000/093/906/original/st25dv04k.pdf?1596828496
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

## Wiring

- Red: 3.3VDC Power
- Black: Ground
- Blue: I2C SDA Data
- Yellow: I2C SCL Clock

# Datasheets

- Accels/Magnetometer
    - NXP: [FXOS8700CQ](FXOS8700CQ.pdf)
- Gyros:
    - NXP: [FXAS21002](FXAS21002.pdf)
- Accels/Gyros
    - ST [LSM6DSOX](lsm6dsox.pdf)
    - ST [LSM6DS33](lsm6ds33.pdf) (Discontinued)
    - ST [LSM9DS1](lsm9ds1.pdf)
        - [LSM9DS1 tech notes](TA0343-LSM9DS1-IMU.pdf)
    - ST [ISM330DHCX](ism330dhcx.pdf)
    - InvenSense [ICM-20649](icm-20649.pdf)
- Magnetometer
    - ST [LIS3MDL](lis3mdl.pdf)
    - ST [LIS2MDL](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
    - MEMSIC [MMC5603NJ](https://cdn-learn.adafruit.com/assets/assets/000/113/957/original/MMC5603NJ_RevB_7-12-18.pdf?1659554945)
- Pressure
    - ST [LPS22](https://www.st.com/resource/en/datasheet/dm00140895.pdf)
    - Bosch [BMP390](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)
    - Infineon [DPS310](https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_01-EN.pdf?fileId=5546d462576f34750157750826c42242)



