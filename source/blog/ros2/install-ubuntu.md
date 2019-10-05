---
title: Installing ROS2 on Ubuntu
date: 5 Oct 2019
---

As of this, ROS2 Dashing Diamenata is the latest [release](https://index.ros.org/doc/ros2/Releases/)

**WARNING:** the install instructions are crap. I originally use install linux, but only later noticed
there was an [installing ros2 via debian packages](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/#dashing-linux-install-debians-install-ros-2-packages)
option

## Basics

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-dashing-desktop
# sudo apt install ros-dashing-ros-base
```

## Cross-Compiling for ARM

There are [directions](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/) for using
docker to cross-compile for ARM.
