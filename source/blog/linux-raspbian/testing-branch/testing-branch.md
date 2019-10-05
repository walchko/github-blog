# Adding Testing to Apt

Sometimes you want access to new software. Adding testing to your repos allows you
to get access to newer software.

```
sudo apt search gcc -t testing
sudo apt install gcc -t testing
```

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
