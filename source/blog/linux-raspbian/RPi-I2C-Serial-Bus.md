# I2C Serial Bus

:date: 2017-02-05

First we need to enable i2c with adding `dtparam=i2c_arm=on` to `/boot/config.txt`.
This defaults the system to 100kHz, but you can change to 400kHz (if your
sensors support it) by adding `i2c_arm_baudrate=400000` to the file.

Now `/dev/i2c-0` and `/dev/i2c-1` should exist. **However**, `/dev/i2c-0` 
is for other things like the VideoCore (PiCamera), so you can't use it.
Also, it isn't enabled, but if you do, at boot time you can mess things
up because Raspbian is using it for other things.

If you want to see what is
on the i2c bus, install the `i2c-tools` using:

```
sudo apt install i2c-tools
```

Now to explore the i2c bus try:

```
    [kevin@raspberrypi ~]$ sudo i2cdetect -y 1
         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    70: -- -- -- -- -- -- -- --
```

To have these load at boot, add them to `/etc/modules`:

```
pi@bender ~/github/soccer2 $ sudo more /etc/modules
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.
# Parameters can be specified after the module name.

snd-bcm2835
i2c-dev
i2c-bcm2708
```

```
sudo apt install python-smbus
sudo apt install i2c-tools
```

```
    pi@bender ~/github/soccer2 $ sudo i2cdetect -y 1
         0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
    00:          -- -- -- -- -- -- -- -- -- -- -- -- --
    10: -- -- -- -- -- -- -- -- 18 -- -- -- -- -- 1e --
    20: 20 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- --
    70: 70 -- -- -- -- -- -- --
```

This shows what things are on the I2C bus: 0x18 (accelerometers), 0x1e
(forget?), and 0x69 (gyros).

Next, get Adafruit's python code which has I2C code for a variety of things:

```
git clone https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git
```
