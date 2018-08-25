# Controlling the LEDs on an RPi

The green OK LED can be controlled from software. It's available as
`/sys/class/leds/led0/`

The kernel LED driver, which controls led0, has "triggers" which let some other
part of the kernel control the LED. The default trigger for the LED is `mmc0`,
which makes it come on when the SD card is accessed.

```
root@raspberrypi:~# cat /sys/class/leds/led0/trigger
none [mmc0]
```

Here, the `mmc0` trigger is selected. You can deactivate this as follows:

```
echo none >/sys/class/leds/led0/trigger
```

The LED can be turned on and off using the `brightness` file. The minimum
brightness is 0, and the maximum is 255 (specified in the `max_brightness`
file). However, as there is no hardware support for variable brightness,
any value greater than 0 will turn the LED on.

```
echo 1 >/sys/class/leds/led0/brightness
echo 0 >/sys/class/leds/led0/brightness
```

Setting the brightness to 0 automatically sets the trigger to "none"

If you want the LED to go back to its default function:

```
echo mmc0 >/sys/class/leds/led0/trigger
```

As an aside, there are a couple of kernel modules you can load up
(ledtrig_timer and ledtrig_heartbeat) which will flash the LED for you.

```
modprobe ledtrig_heartbeat
echo heartbeat >/sys/class/leds/led0/trigger
```

- [Reference](https://www.raspberrypi.org/forums/viewtopic.php?t=12530)
