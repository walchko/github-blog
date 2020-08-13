---
title: Building a New Colcon Python Package
date: 3 Aug 2020
---

![](../ros2.png)

## Setup Files

```
cd ros2/src
ros2 pkg create ros2_demo_py
```

```
from setuptools import setup

package_name = 'ros2_demo_py'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@youremail.com',
    maintainer='YourFirstname Lastname',
    maintainer_email='your@youremail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A simple ROS2 Python package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = ros2_demo_py.demo:main'
        ],
    },
)
```

```
[develop]
script-dir=$base/lib/ros2_demo_py
[install]
install-scripts=$base/lib/ros2_demo_py
```

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>ros2_demo_py</name>
  <version>0.7.3</version>
  <description>A simple ROS2 Python package</description>

  <maintainer email="sloretz@openrobotics.org">Shane Loretz</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- These test dependencies are optional
  Their purpose is to make sure that the code passes the linters -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Build Package and Run

| Command                       | Description                             |
|-------------------------------|-----------------------------------------|
| `--symlink-install`           | Make links of python files to `install` |
| `--packages-select <package>` | Build a specific package                |

1. `colcon build --symlink-install --packages-select demo`
1. `souce install/setup.zsh`
1. `ros2 run ros2_demo_py demo`

```
% ros2 run ros2_demo_py demo                
[INFO] [1596421680.603279724] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1596421681.082182154] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1596421681.582148794] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [1596421682.082149247] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [1596421682.582145181] [minimal_publisher]: Publishing: "Hello World: 4"
[INFO] [1596421683.082130446] [minimal_publisher]: Publishing: "Hello World: 5"
[INFO] [1596421683.582146477] [minimal_publisher]: Publishing: "Hello World: 6"
...
```

## References

- [ROS2 Tutorials](https://www.theconstructsim.com/ros2-tutorials-5-how-to-create-a-ros2-package-for-python-update/)
