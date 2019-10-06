---
title: ROS2 Cheatsheet
date: 5 Oct 2019
---

Most of the commands have changed for ros2:

- rosbag: `ros2 bag play|record`
- rostopic: `ros2 topic list|echo|bw`
- rviz2: `ros2 run rviz2 rviz2`

## Package Creation

- `colcon` replaces `catkin`
    - `colcon build` creates the workspace
    - devel directory is gone
- install packages with: `rosdep install --from-paths src -i -y`
- create a package with: `ros2 pkg create <pkg-name> --dependencies [deps]`

```
ros_ws
+- build
+- install
|  +- setup.bash
+- src
   +- package_cpp
   |  +- CMakeLists.txt
   |  +- scripts
   |  +- src
   |  +- etc ...
   +- package_python
      +- launch
      +- setup.py
      +- setup.cfg
      +- package.xml
```

### Launch File

`ros2 launch my_package script.launch.py`

```python
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])
```

# References

- [ROS2 colcon tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- [launch system for cpp and python](https://index.ros.org/doc/ros2/Tutorials/Launch-system/)
