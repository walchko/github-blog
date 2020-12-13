---
title: Change the Activity the LED Tracks
date: 7 Dec 2019
---

In `/boot/config.txt`:

```
dtparam=pwr_led_trigger=heartbeat
```

`/boot/overlays/README` has this text:

```
    act_led_trigger         Choose which activity the LED tracks.
                            Use "heartbeat" for a nice load indicator.
                            (default "mmc")
```

```
none                No trigger
kbd-scrolllock      Keyboard scroll lock
kbd-numlock         Keyboard num lock
kbd-capslock        Keyboard caps lock
kbd-kanalock        Keyboard kana lock
kbd-shiftlock       Keyboard shift
kbd-altgrlock       Keyboard altgr
kbd-ctrllock        Keyboard ctrl
kbd-altlock         Keyboard alt
kbd-shiftllock      Keyboard left shift
kbd-shiftrlock      Keyboard right shift
kbd-ctrlllock       Keyboard left ctrl
kbd-ctrlrlock       Keyboard right ctrl
timer               Flash at 1 second intervals
oneshot             Flash only once
heartbeat           Flash like a heartbeat (1-0-1-00000)
backlight           Always on
gpio                Flash when a certain GPIO is high???
cpu0                Flash on cpu0 usage
cpu1                Flash on cpu1 usage
cpu2                Flash on cpu2 usage
cpu3                Flash on cpu3 usage
default-on          Always on
[input]             Default state
panic               Flash on kernel panic
mmc0                Flash on mmc0 (primary SD Card interface) activity
mmc1                Flash on mmc1 (secondary SD Card interface) activity
rfkill0             Flash on wifi activity
rfkill1             Flash on bluetooth activity
```

## References

- [stack exchange question](https://raspberrypi.stackexchange.com/questions/69674/are-there-other-act-led-trigger-options-besides-mmc-and-heartbeat/69759#69759)
