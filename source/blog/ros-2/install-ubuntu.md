---
title: Installing ROS2 on Ubuntu
date: 5 Oct 2019
---

![](ros2.png)

As of this, ROS2 Dashing Diamenata is the latest [release](https://index.ros.org/doc/ros2/Releases/)

**WARNING:** the install instructions are crap. I originally use install linux, but only later noticed
there was an [installing ros2 via debian packages](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/#dashing-linux-install-debians-install-ros-2-packages)
option

## `apt`

Setup your system with needed programs and repos:

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```
Now update your system, *note* below, I am instally `foxy`, but you can change the name to any distro you want:

```
sudo apt update
sudo apt install ros-foxy-desktop
# sudo apt install ros-foxy-ros-base
```

Now do `source /opt/ros/dashing/setup.bash` to setup your `env` properly.

## `pip`

To install the python packages in a virtualenv:

```python
 pip3 install -U colcon-common-extensions
 ```

## Cross-Compiling for ARM

There are [directions](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/) for using
docker to cross-compile for ARM.
