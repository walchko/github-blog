---
title: ROS2 and the RPLidar
date: 9 Aug 2020
---

Ok, the design is a total piece of crap! The white serial/power cable coming out of the
bottom means you can't set it down flat ... what special idiot designed this?

Also, why does it never stop spinning? I have to disconnect the power to shut the damn
thing off ... crap!

## Install and Running

Ok, rants aside, it seems to work nicely

- Install drivers: `sudo apt install ros-foxy-rplidar-ros`
    - Then: `source /opt/ros/foxy/setup.bash`
- Run lidar node to get data: `ros2 run rplidar_ros rplidarNode`
- Run client node to see data: `ros2 run rplidar_ros rplidarNodeClient`
