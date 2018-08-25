# Determining if the Raspberry Pi is Under-voltage

```
$ vcgencmd get_throttled
throttled=0x50005
```

Okay, the bits in this number represent:

- 0: under-voltage
- 1: arm frequency capped
- 2: currently throttled
- 16: under-voltage has occurred
- 17: arm frequency capped has occurred
- 18: throttling has occurred

under-voltage occurs when voltage drops below 4.63V. The Pi is throttled
arm frequency capped occurs with temp > 80'C
over-temperature occurs with temp > 85'C. The Pi is throttled

Throttling removes turbo mode, which reduces core voltage, and sets arm and gpu
frequencies to non-turbo value. Capping just limits the arm frequency
(somewhere between 600MHz and 1200MHz) to try to avoid throttling. If you are
throttled and not under-voltage then you can assume over-temperature.
(confirm with `vcgencmd` measure_temp).

So `0x50005` means you are currently under-voltage and throttled.
If you want to be able to support this use case without throttling you will
need a better power supply.

This shows we are not undervoltaged, but frequency is capped:

```
$ vcgencmd get_throttled
throttled=0x20000
```



- [Reference](https://www.raspberrypi.org/forums/viewtopic.php?f=63&t=147781&start=50#p972790)
