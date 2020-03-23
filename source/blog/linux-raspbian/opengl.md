---
title: OpenGL on the Raspberry Pi
date: 23 Mar 2020
---

![](https://upload.wikimedia.org/wikipedia/en/thumb/3/3f/OpenGL_ES_logo_%28Nov14%29.svg/320px-OpenGL_ES_logo_%28Nov14%29.svg.png)

The GPU bundled with **Raspberry Pi 4** is a VideoCore VI capable of OpenGL ES 3.2, 
a significant step above the VideoCore IV present in **Raspberry Pi 3** which could only do 
OpenGL ES 2.0. 

The raspbian comes with OpenGL installed to `/opt/vc` from the `libraspberrypi-dev` package.

```
$ apt show libraspberrypi-dev
Package: libraspberrypi-dev
Version: 1.20200212-1
Priority: optional
Section: libdevel
Source: raspberrypi-firmware
Maintainer: Serge Schneider <serge@raspberrypi.org>
Installed-Size: 3,369 kB
Depends: libraspberrypi0 (= 1.20200212-1)
Homepage: https://github.com/raspberrypi/firmware
Download-Size: 408 kB
APT-Manual-Installed: yes
APT-Sources: http://archive.raspberrypi.org/debian buster/main armhf Packages
Description: EGL/GLES/OpenVG/etc. libraries for the Raspberry Pi's VideoCore IV (headers)
 This package contains headers and other development files for implementations
 of EGL, OpenGL ES, OpenVG, OpenWF Composition, and others for the Raspberry
 Pi's VideoCore IV multimedia processor.
```

```
$ ls /opt/vc
bin  include  lib  src

$ ls /opt/vc/include
bcm_host.h  EGL  GLES  GLES2  IL  interface  KHR  vcinclude  VG  WF
```

# References

- [Raspberrypi.com: OpenGl Drivers Update](https://www.raspberrypi.org/blog/vc4-and-v3d-opengl-drivers-for-raspberry-pi-an-update/)
