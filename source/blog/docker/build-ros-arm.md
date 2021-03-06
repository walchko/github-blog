---
title: ROS Cross Compiling for ARMv7
date: 25 Sept 2019
---

As root (`sudo`) run:

```
# make a catkin workspace
mkdir -p tmp/ws
cd tmp/ws

# set repo and get keys
sh -c 'echo "deb  http://packages.ros.org/ros/ubuntu  $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update

# get tools
apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential  cmake

# initialize and update rosdep
rosdep init
rosdep update

# setup and build ros ... or ros-comm
rosinstall_generator ros-base --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall 
wstool init -j8 src melodic-desktop-wet.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2
```

```
# fix path
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# References

- [Compile ROS for RPi 4 Debian Buster and the RPLIDAR package](https://www.instructables.com/id/ROS-Melodic-on-Raspberry-Pi-4-RPLIDAR/)
