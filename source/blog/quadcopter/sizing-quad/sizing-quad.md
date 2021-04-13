---
title: Sizing Considerations
date: 12 Apr 2021
---

| Quad Diameter | Prop Size | Stator    | KV        |
|---------------|-----------|-----------|-----------|
| <150mm        | 3"        | 1105-1306 | 3000-4000 |
| 150-250mm     | 4"        | 1806,2204 | 2600-2800 |
| 190-220mm     | 5"        | 2205-2208,2305-2306 | 2300-2600 |
| 220-270mm     | 6"        | 2204-2208,2306 | 1960-2300 |
| 350mm         | 7"        | 2505-2510 | 1200-1600 |
| 450mm         | 8"        | 26xx      | <1200 |

- **Stator:** The Size of brushless motors in RC is normally indicated by a 4-digit number – AABB. “AA” 
is the stator width (or stator diameter) while “BB” is the stator height, both are measured in mm
    - Taller stator = more power at higher RPM
    - Wider stator = more torque at lower RPM
- **Efficiency:** Higher KV motors at high RPMs are more efficient, but have less torque
- **Power:** Ideally want a 2:1 (minimum), 4:1 (aerobatic), or 8:1 (racing) power to weight ratio
- **Torque:** Motors with larger stator size have more torque, while smaller stators have less torque 

The torque (T) is equal t othe torque constant ($K_t$) and the current (i):
$$
T = K_t i \\
K_t = -KV
$$

while the KV of a motor is equal to the inverse of the torque constant. So a motor with a KV of 3000
has less torque than a motor with a KV of 1000.

# References

- [Choose the Right Size Motors](https://quadquestions.com/blog/2017/02/22/choose-right-size-motors-drone/)
- [miniquadtestbench](https://www.miniquadtestbench.com/motor-explorer.html)
- [HOW TO CHOOSE MOTOR FOR RACING DRONE & QUADCOPTER](https://oscarliang.com/quadcopter-motor-propeller/)
