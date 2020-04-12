---
title: GPIO and Buttons
---

## Power Button

### Overlays

You can enable a power button to shut off the pi using and overlay.
Add the following to your `/boot/config.txt`:

```
# power button
dtoverlay=gpio-shutdown,gpio_pin=26
```
Here I have connected the switch between BCM 26 and ground.
As long as the button/switch is closed (logic 1), then pi will
stay on. Break the connection (off or logic 0) and the pi will
shutdown immediately. No warrning is given.

This also creates: `/dev/input/by-path/platform-soc:shutdown_button-event`.

### Python

Now the above doesn't give you much control over what happens.
When you flip the switch, the system just shuts off. If you use
python, you have more control:

```python
#!/usr/bin/env python

import RPi.GPIO as GPIO
import subprocess


GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.wait_for_edge(3, GPIO.FALLING)

subprocess.call(['shutdown', '-h', 'now'], shell=False)
```

## Temperature Controlled Fan

You can do a similar things with a fan. You can set the pin
and temperature which it will turn on:

```
dtoverlay=gpio-fan,gpiopin=12,temp=55000
```

Here I have set the defaults of pin BCM 12 and 55 C (the
units are mili-celcius for some reason).

# References

- [Power button and python](https://howchoo.com/g/mwnlytk3zmm/how-to-add-a-power-button-to-your-raspberry-pi)
