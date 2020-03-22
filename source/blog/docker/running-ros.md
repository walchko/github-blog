---
title: Running ROS in a Docker Container
date: 14 Sept 2019
---

The basics are [here](http://wiki.ros.org/docker/Tutorials/Docker) on the
ros wiki website.

1. If you haven't already, get the latest ros: `docker pull ros`
    1. If you need an older version you identify it and pull it ... I just use the latest
    1. What are the [versions of ros?](http://wiki.ros.org/Distributions)
1. Start it and get a command line inside the container: `docker run -it ros`
1. See the container started:
    ```
    $ docker ps -l
    CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES
    403b06247a82        ros                 "/ros_entrypoint.sh …"   8 minutes ago       Up 8 minutes                            thirsty_swanson
    ```
1. Get another command line: `docker exec -it thirsty_swanson bash`
    1. Note, from the `docker ps -l` command, my container is called `thirsty_swanson`
1. You will need to setup the environment: `source ros_entrypoint.sh`
1. See if you can see things: `rostopic list`
    1. Note, if `roscore` isn't running then it will complain about no master
1. Also update ros: `rosdep update`
