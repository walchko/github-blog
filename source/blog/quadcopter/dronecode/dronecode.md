---
title: Dronecode
date: 9 Jan 2020
---

## References

- [dronecode.org](https://www.dronecode.org/)
- [pixhawk.org](https://pixhawk.org/): Drone hardware standard
- [MAVLink](https://mavlink.io/en/): micro air vehicle communication protocol
- [QGroundControl](http://qgroundcontrol.com/): a ground control
- [PX4.io](https://px4.io/): An opensource autopilot
- [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python): Python interface to MAVLink
- [PX4 Headless Gazebo](https://github.com/JonasVautherin/px4-gazebo-headless)

## SITL Gazebo

Run broadcast mode

```
docker run --rm -it jonasvautherin/px4-gazebo-headless:1.11.0
```

## MAVSDK-Python

```
pip install -U mavsdk
```

### Simple Example

```python
#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
```

```python
dir(drone.telemetry)

[
    '__class__', 
    '__delattr__', 
    '__dict__', 
    '__dir__', 
    '__doc__', 
    '__eq__', 
    '__format__', 
    '__ge__', 
    '__getattribute__', 
    '__gt__', 
    '__hash__', 
    '__init__', 
    '__init_subclass__', 
    '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', 
    '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', 
    '__subclasshook__', '__weakref__', 
    '_extract_result', '_init_plugin', '_setup_stub', '_stub', 
    'actuator_control_target', 
    'actuator_output_status', 
    'armed', 
    'attitude_angular_velocity_body', 
    'attitude_euler', 
    'attitude_quaternion', 
    'battery', 
    'camera_attitude_euler', 
    'camera_attitude_quaternion', 
    'distance_sensor', 
    'fixedwing_metrics', 
    'flight_mode', 
    'get_gps_global_origin', 
    'gps_info', 
    'ground_truth', 
    'health', 
    'health_all_ok', 
    'home', 
    'imu', 
    'in_air', 
    'landed_state', 
    'name', 
    'odometry', 
    'position', 
    'position_velocity_ned', 
    'rc_status', 
    'set_rate_actuator_control_target', 'set_rate_actuator_output_status', 
    'set_rate_attitude', 'set_rate_battery', 'set_rate_camera_attitude', 
    'set_rate_distance_sensor', 'set_rate_fixedwing_metrics', 
    'set_rate_gps_info', 'set_rate_ground_truth', 'set_rate_home', 
    'set_rate_imu', 'set_rate_in_air', 'set_rate_landed_state', 
    'set_rate_odometry', 'set_rate_position', 'set_rate_position_velocity_ned', 
    'set_rate_rc_status', 'set_rate_unix_epoch_time', 'set_rate_velocity_ned', 
    'status_text', 
    'unix_epoch_time', 
    'velocity_ned']
```
