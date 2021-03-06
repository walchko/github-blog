---
title: Multirotor Drone Terms
date: 30 Dec 2020
image: "https://cdn.arstechnica.net/wp-content/uploads/2017/01/GettyImages-487540678-800x539.jpg"
image-height: "400px"
---

## References

- [Oscarliang.com very detailed overview of motors and propellers](https://oscarliang.com/quadcopter-motor-propeller/)
- [FAA UAV Ruling, 28 Dec 2020](https://www.faa.gov/news/media/attachments/OOP_Executive_Summary.pdf)
- ArsTechnica: [FAA finally sets rules for piloting small drones](https://arstechnica.com/tech-policy/2020/12/faa-finally-sets-rules-for-piloting-small-drones/)
- [ArduPilot docs](https://ardupilot.org/copter/index.html)

## Frame Sizes

| Class  | Dimension |
|--------|-----------|
| Nano   | 80-100 mm |
| Micro  | 100-150 mm |
| Small  | 150-250 mm |
| Medium | 250-400 |
| Large  | +400 mm |

## Propulsion

![](../esc-control/motor.jpg)
![](https://oscarliang.com/ctt/uploads/2017/12/quadcopter-brushless-motor-n-p-poles-magnets-number-12-14.jpg)
![]()

- Motors
    - Motors are rated by a 4 digit number AABB, like 2213: stator is 22mm in diameter, stator height is 13mm
        - Taller stator gives **more power** at **higher** RPM
        - Wider stator gives **more torque** at **lower** RPM
    - KV ratings relate RPMs to voltage.
        - 900KV motor at 2V produces 1800RPM
        - Typically lower KV motors produce more torque
    - Motors for 3-6 inch diameter props have M5 shafts
    - Typically have 16x16 mm mount points
    - 22XX and 23XX motors typically have 12 poles and 14 magnets
        - More poles = smoother flight
        - Fewer poles = more powerful motors
- Propellars
    - 4 digit number: 8045 means, 8 inch diameter with a pitch of 4.5 inches
        - Higher pitch produces more downward force
- Matching
    - Typically you would pair **high pitch** propellars with **low KV** motors or the reverse
    - For a 5 in prop, common motors are 2207, 2306, 2307, and 2407
    - High torque, high power motors can create vibration and feedback into the IMU on the flight controller causing instability. Soft mounting your flight controller can help
-Thrust to weight ratio
    - Need at least **twice** the thrust compared to the weight of the drone
- Motor orientation:

![](https://ardupilot.org/copter/_images/motororder-quad-x-2d.png)


## Raspberry Pi and ArduPilot Flight Computer

![](https://docs.emlid.com/navio/img/Navio-_34_600x6001-600x380.png)

- [website](https://navio2.emlid.com/)
- [docs](https://docs.emlid.com/navio/)

## Pixhawk

![](https://docs.px4.io/v1.9.0/assets/flight_controller/pixhawk4/pixhawk4_logo_view.jpg)

- [Pixhawk 4 spec](https://docs.px4.io/v1.9.0/en/flight_controller/pixhawk4.html)
