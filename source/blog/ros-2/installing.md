---
title: Installing ROS2 on Ubuntu
date: 22 July 2022
---

![](ros2.png)

As of this, ROS2 [release](https://index.ros.org/doc/ros2/Releases/)

- [Desktop Ubuntu](https://docs.ros.org/en/humble/Installation.html)
- [Raspberry Pi Instructions (Humble)](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [ROS 2 Varients](https://www.ros.org/reps/rep-2001.html#id22)

## Ubuntu

- Ubuntu Linux - Jammy Jellyfish (22.04)
    - `sudo apt install ros-humble-desktop`
    - `sudo apt install ros-humble-ros-base`
    - [Ubuntu Instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Now do `source /opt/ros/humble/setup.zsh` to setup your `env` properly.

## `pip`

To install the python packages in a virtualenv:

```python
 pip3 install -U colcon-common-extensions
 ```

## Cross-Compiling for ARM

There are [directions](https://index.ros.org/doc/ros2/Tutorials/Cross-compilation/) for using
docker to cross-compile for ARM.
