# Python 3.7.x

`sources.list`:

```
deb http://raspbian.raspberrypi.org/raspbian/ stretch main contrib non-free rpi
# Uncomment line below then 'apt-get update' to enable 'apt-get source'
# deb-src http://raspbian.raspberrypi.org/raspbian/ stretch main contrib non-free rpi


# deb http://raspbian.raspberrypi.org/raspbian/ buster main contrib non-free rpi
# deb-src http://raspbian.raspberrypi.org/raspbian/ buster main contrib non-free rpi
```

`source.list.d/buster.list`:

```
deb http://raspbian.raspberrypi.org/raspbian/ buster main contrib non-free rpi
# deb-src http://raspbian.raspberrypi.org/raspbian/ buster main contrib non-free rpi
```

`preferences.d/buster.pref`:

```
Package: *
Pin: release a=buster
Pin-Priority: 100
```

`preferences.d/stretch.pref`:

```
Package: *
Pin: release a=stretch
Pin-Priority: 700
```

`preferences.d/stable.pref`:

```
Package: *
Pin: release a=stable
Pin-Priority: 900
```

## Installing

```
sudo apt update
sudo apt install python3 python3-venv -t testing
```
