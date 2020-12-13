---
title: Displaying Status of Pi Hole using RPi 7in LCD
date: 23 Sept 2019
image: "https://camo.githubusercontent.com/fe3dec9387952987a6a7fd5c7d9f7217a578737a/68747470733a2f2f6a706d636b2e636f6d2f696d672f626c6f672f706164642e706e67"
---

The software is available [here](https://github.com/jpmck/PADD). Most of the setup
is explained there, just a few clarifications are added here.

1. If the LCD is upside-down, you can fix it by adding `lcd_rotate=2` in `/boot/config.txt`
1. *[optional]* You can disable the touch screen capability by adding `disable_touchscreen=1` to `/boot/config.txt`
1. Get project: `git clone https://github.com/jpmck/PADD.git`
1. Append the following to the end of pi's `.bashrc` file. If `pi` logs in on the LCD
then `padd.sh` will run, else if `pi` is logged in on an `ssh` connection it will be
normal login.
    ```
    # Run PADD
    # If we’re on the PiTFT screen (ssh is xterm)
    if [ "$TERM" == "linux" ] ; then
      while :
      do
        ./padd.sh
        sleep 1
      done
    fi
    ```
1. Enable `pi` autologin
    1. Run: sudo raspi-config
    1. Choose option 3: Boot Options
    1. Choose option B1: Desktop / CLI
    1. Choose option B2: Console Autologin
    1. Select Finish, and reboot the pi

Now, you should see `padd.sh` running on the LCD.

## Brightness

Now `root` owns the memory location controlling brightness which ranges from 0-255.
There is no reason to keep this LCD screemingly bright all the time, so I run it at
a lower level than the default 255. You can play with the brightness via the following
and add it into your `.bashcr` when you are happy.

```
sudo bash -c "echo 50 > /sys/class/backlight/rpi_backlight/brightness"
```

# References

- [LCD parameter fine tunings](https://www.balena.io/blog/add-a-display-to-your-pi-hole-for-monitoring-and-stats/)
