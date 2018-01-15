
Visual Odometry - Mono
=========================

:date: 2016-02-16
:summary: How to track distance travel via a camera - still work in progress

Visual odometry is using a camera to figure out how far a robot has gone. It is
most accurate when doing stereo vision because the system can determine how far
points have travelled. Monocular vision can be just as useful, except there is
a scaling issue that needs to be determined (from another sensor) in order to
determine the distance travelled.

Steps
------

1. Grab image
2. Rectify image
3. Determine movement of tracked points (optical flow and camera geometry)

Useful Eqns
------------

:math:`s \begin{pmatrix} u & v & 1 \end{pmatrix}^T = C E \begin{pmatrix} x & y & z & 1 \end{pmatrix}^T`

where :math:`(u,v)` are a point's location in the image plane and :math:`(x,y,z)`
are the corresponding point's location in the real-world.

Terms
-----

Essential Matrix (E)
  Contains the rotation and translation (R|t) of a second camera relative to
  the first camera in stereo vision.

Fundamental Matrix (F)
  Same as E, but with intrinsics. If the images are already rectified, then
  F = E.

Camera Matrix (C)
  The camera matrix contains both the focal lengths (x,y) and the center point
  (x,y) of the image for the camera. You can use the the camera matrix to
  move between 2D (in the image) and 3D (in the real world) points.

Mapping Camera Coordinates to Ground Plane
--------------------------------------------

.. figure:: pics/vo_ref.png
    :align: center
    :width: 300px

    Mapping coordinates in the camera frame to the ground plane

In order to recover the speed, we need to understand how pixels in the image are moving in
the real world.

.. math::
    \tan(\beta)= \frac{H}{D} \\
    \tan(\beta)=(2 \nu - V) \tan( \frac{VFOV}{2}) \\
    y = \frac{H}{\tan(\alpha + \beta)} \\
    z = \frac{H \cos(\beta)}{\sin(\alpha + \beta)}

where VFOV is the vertical field of view, :math:`\nu`.

Optical Flow
--------------

.. figure:: pics/vo_example.png
    :align: center
    :width: 300px

References
-----------

1. `OpenCV 3.1 Docs <http://docs.opencv.org/3.1.0/>`_
2. `Avi Singh <https://avisingh599.github.io/vision/monocular-vo/>`_
3. J. Campbell, R. Sukthankar, I. Nourbakhsh, A. Pahwa, A Robust Visual Odometry and Precipice Detection System Using Consumer-grade Monocular Vision, Robotics and Automation, 2005. ICRA 2005. Proceedings of the 2005 IEEE International Conference, p. 3421 - 3427, 18-22 April 2005.
