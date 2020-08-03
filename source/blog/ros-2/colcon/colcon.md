---
title: Example Package with Colcon
date: 2 Aug 2020
---

![](../ros2.png)

*Note:* I am using the `zsh` shell instead of `bash`.

Unfortunately the examples are shitty, overly verbose for no reason, leave stuff out and don't make sense.

1. Source the ros setup file 
1. Create a workspace (`ros2`) and a source code folder (`src`)
1. Move into the workspace
1. Clone some example directories
    1. Make sure to check out the correct distro branch 
1. Build with examples `colcon`, the latest crappy ros build tool

```
source /opt/ros/foxy/setup.zsh
mkdir -p ros2/src
cd ros2
git clone https://github.com/ros2/examples src/examples
cd src/examples
git checkout $ROS_DISTRO
cd ../..
git clone https://github.com/ros2/example_interfaces.git src/example_interfaces
colcon build --symlink-install
```

So now you have a directory structure that looks like:

```
ros2
|-bulid
|-install
| +-setup.zsh
|-log
+-src
  |-examples
  +-example_interfaces
```

Now run `colcon test` to make sure all is well

Now run `source install/setup.zsh`

Now run a pub/sub, you need some windows (having run `source install/setup.zsh` in them): 

- `ros2 run examples_rclcpp_minimal_publisher publisher_member_function`
- `ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function`
- `ros2 run examples_rclpy_minimal_subscriber subscriber_member_function`

Notice, there is a mixture of cpp (`rclcpp`) and python (`rclpy`) in these ... neat!

## References

- Ros2 Tutorial: [Using Colcon to Build Packages](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
