---
title: Cross Compiling ARM Binaries for Raspberry Pi
date: 21 Sept 2019
---

Steps:

1. Enable ARM emulation with: `docker run --rm --privileged hypriot/qemu-register`
1. Run an ARM image with: `docker run --rm -it balenalib/raspberrypi3-debian:latest bash`
1. At the bash prompt do: 
    ```
    $ uname -a
    Linux ffc27f604ee6 4.14.29-1-lts #1 SMP Wed Mar 21 17:41:42 CET 2018 armv7l GNU/Linux
    
    $ cat /etc/os-release
    PRETTY_NAME="Debian GNU/Linux 10 (buster)"
    NAME="Debian GNU/Linux"
    VERSION_ID="10"
    VERSION="10 (buster)"
    VERSION_CODENAME=buster
    ID=debian
    HOME_URL="https://www.debian.org/"
    SUPPORT_URL="https://www.debian.org/support"
    BUG_REPORT_URL="https://bugs.debian.org/"
    ```

The `balenalib/raspberrypi3-debian:latest` is a stripped down image, no python or gcc. The
`balenalib/raspberrypi3-debian-build` has a lot of development tools/libries installed.


# References

- [hypriot blog post](https://blog.hypriot.com/post/docker-intel-runs-arm-containers/)
- [hypriot qemu github repo](https://github.com/hypriot/qemu-register)
- [balenalib armv7hf debian image](https://hub.docker.com/r/balenalib/armv7hf-debian)
- [balenalib armv7hf ubuntu image](https://hub.docker.com/r/balenalib/armv7hf-ubuntu)
- [another way to do it using qemu](https://github.com/petrosagg/armv7hf-python-dockerhub/blob/master/Dockerfile)
- [cross compile opencv](https://github.com/sgtwilko/rpi-raspbian-opencv)
- [live docker prompt](https://www.katacoda.com/contino/courses/docker/basics)
- [ros docker and qemu blog post](https://discourse.ros.org/t/announcing-ros-docker-images-for-arm-and-debian/2467)
