---
title: ROS2 Cheatsheet
date: 5 Oct 2019
---

![](ros2.png)

## Commands

![](https://index.ros.org/doc/ros2/_images/Nodes-TopicandService.gif)

Most of the commands have changed for ros2:

- rosbag: `ros2 bag play|record`
- rostopic: `ros2 topic list|echo|bw`
- rviz2: `ros2 run rviz2 rviz2`
- rosmsg: `ros2 interface show std_msgs/Bool` -> `bool data`
- rossrv: `ros2 interface show std_srvs/srv/Trigger` -> 
```
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
```

- Find out what nodes are running: `ros2 node list`
```
/listener
/talker
```
- See node services, pub/sub, and actions: `ros2 node info <node>`
```
/talker
  Subscribers:

  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

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

- ros answers: [ROS2 equivalents to ROS1 roscd, rosmsg rossrv...](https://answers.ros.org/question/358573/ros2-equivalents-to-ros1-roscd-rosmsg-rossrv/)
- [ROS2 colcon tutorial](https://index.ros.org//doc/ros2/Tutorials/Colcon-Tutorial/)
- [launch system for cpp and python](https://index.ros.org/doc/ros2/Tutorials/Launch-system/)
