# INS Errors

## Accelerometers

- Sensitivity of a sensor to x-axis inputs ($S_x$), this is repeated for y and z-axis
- Sensitivity of a sensor to x-axis and y-axis inputs ($M_{xy}$), this is repeated for all axis combinations

![](pics/sensor-model-equations---figure-1.png)

![](pics/sensor-model-equations---figure-2.png)

Bias correction can be done by finding average offsets over a period of time.
Capturing scaling issues and axis misalignments requires more complex testing
like 4-point or 6-point tumble testing respectively.

![](pics/accelerometer-bias-calibration---figure-1.png)

![](pics/accelerometer-scale-factor---figure-1.png)

![](pics/accelerometer-axis-misalignment---figure-1.png)

## Gyroscopes

Gyro errors have the largest impact on INS performance. Gyros have a random walk
component of error that cannot be calibrated out using standard methods. 

![](pics/gyroscope-bias---figure-1.png)

Scale factor errors can be calibrated out by spinning a gyro around each axis at a known
rate and using a linear least squares to calcuate the best fit.

![](pics/gyroscope-scale-factor---figure-1.png)

Gyros do exhibit some sensitivity to accelerations

![](pics/gyroscope-sensitivity-to-linear-acceleration---figure-1.png)

![](pics/gyroscope-sensitivity-to-linear-acceleration---figure-2.png)

## Magnetometers

Magnetometers are strongly influenced by temperature which effects their scaling and bias
error terms.

![](pics/magnetometer-bias---figure-1.png)

![](pics/magnetometer-bias-sensitivity---figure-1.png)

![](pics/magnetometer-scale-factor-temperature-sensitivity---figure-1.png)

![](pics/magnetometer-misalignment---figure-1.png)

![](pics/mag-cross-axis.png)

## Temperature

![](pics/temperature-calibration---figure-1.png)

# References

- [Vectornav sensor calibration](https://www.vectornav.com/support/library/calibration)
