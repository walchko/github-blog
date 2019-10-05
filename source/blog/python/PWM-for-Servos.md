# Pulse Width Modulation (PWM) for Controlling RC Servos

## RPi.GPIO

This uses a software PWM library so it will consume some CPU time. In my very
informal estimate, about 5% per servo.

```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
pwm=GPIO.PWM(11,50)

# duty cycle 5-10 is full range
pwm.start(5)
pwm.ChangeDutyCycle(7.5)
```

# References

-[GPIO library](http://www.toptechboy.com/raspberry-pi/raspberry-pi-lesson-28-controlling-a-servo-on-raspberry-pi-with-python/)
