---
title: Awesome ROS Nodes
date: 9 Aug 2020
---

## Camera

- [raspicam2_node](https://github.com/christianrauch/raspicam2_node)
- [opencv_cam](https://github.com/christianrauch/opencv_cam)

## Markers

- [apriltag_ros](https://github.com/christianrauch/apriltag_ros)

## LIDAR

- [ydlidar](https://github.com/YDLIDAR/ydlidar_ros2_driver)
    - Install first: [ydlidar sdk](https://github.com/YDLIDAR/YDLidar-SDK)
    - Avoid polluting system:
        - `cmake -DCMAKE_INSTALL_PREFIX=/home/ubuntu/SDKs -DBUILD_SHARED_LIBS=1 .. && make install` 
        - `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/SDKs/lib`
    - Build ROS2 driver with: `colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/home/ubuntu/SDKs`

## IMU

# References

- [Setting library paths](https://www.cyberciti.biz/faq/linux-setting-changing-library-path/)
