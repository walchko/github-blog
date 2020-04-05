---
title: Raspberry Pi Linux Kernel
date: 26 Oct 2014
---

## Determine Kernel version and upgrade

You can determine the current linux kernel version by:

```
[kevin@raspberrypi tmp]$ more /proc/version
Linux version 3.2.27+ (dc4@dc4-arm-01) (gcc version 4.7.2 20120731 (prerelease)
(crosstool-NG linaro-1.13.1+bzr2458 - Linaro GCC 2012.08) ) #250 PREEMPT Thu Oct
18 19:03:02 BST 2012
```

or

```
[kevin@raspberrypi tmp]$ uname -a
Linux raspberrypi 3.2.27+ #250 PREEMPT Thu Oct 18 19:03:02 BST 2012 armv6l GNU/Linux
```

In both cases, this is kernel version 3.2.27. Note that the *l* in
`armv6l` refers to little endian.

Get and install [rpi-update](http://github.com/Hexxeh/rpi-update): `sudo apt-get install rpi-update`

## Certificate Errors

If you get an error:

```
...
!!! Make sure you have ca-certificates installed and that the time is set correctly
```

Try:

    - `sudo dpkg-reconfigure tzdata`
    - `sudo rpi-update`
    - `sudo rpi-update -k`
    - `sudo CURL_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt rpi-update`
        - `export CURL_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt`
        - `sudo rpi-update`

Also:

```
sudo apt-get install ca-certificates
sudo apt-get install ntpdate
sudo ntpdate -u ntp.ubuntu.com
```
