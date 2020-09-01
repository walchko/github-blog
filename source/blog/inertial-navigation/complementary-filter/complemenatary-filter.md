---
title: CF
date: 31 Aug 2020
---

Choosing $\alpha$?

$\alpha = Γ / (Γ + dt)$

alpha=(tau)/(tau+dt) where tau is the desired time constant (how fast you want the readings to respond) 
and dt = 1/fs where fs is your sampling frequency. This equation is derived from filter/control theory 
will put a link to this  from

# References

- github: [IMU tools for ROS](https://github.com/ccny-ros-pkg/imu_tools)
- [Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs](https://www.mdpi.com/1424-8220/15/8/19302)
- [Complementary filter design](https://gunjanpatel.wordpress.com/2016/07/07/complementary-filter-design/)
- MIT: [16.333: Lecture 15, Complementary filtering](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-333-aircraft-stability-and-control-fall-2004/lecture-notes/lecture_15.pdf)
