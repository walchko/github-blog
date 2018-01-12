Quadruped Robot
=================

:date: 2016-07-11
:modified: 2016-07-31
:summary: Simple quadruped made from a 3d printed body and 9 gram micro servos (TG9e).

.. figure:: {filename}/blog/robots/pics/rc-spider-1.jpg
	:align: center
	:width: 400

Kinematics
~~~~~~~~~~~~~

Useful Trigonometry
---------------------

.. figure:: {filename}/blog/robots/pics/law_cosines.png
	:align: center

The **law of cosines** for calculating one side of a triangle when the angle
opposite and the other two sides are known. Can be used in conjunction with
the law of sines to find all sides and angles.

Forward Kinematics
-------------------

Forward kinematics produces the position (x,y,z) of the foot given the angles
of the leg segment joints. We will use the following parameters (from the real
robot) and nomenclature

=========== ================================ ============
Leg Segment Symbol                           Length
=========== ================================ ============
Coxa        :math:`L_{coxa}` or :math:`L_c`  26 mm
Femur       :math:`L_{femur}` or :math:`L_f` 45 mm
Tibia       :math:`L_{tibia}` or :math:`L_t` 63 mm
=========== ================================ ============

.. figure:: {filename}/blog/robots/pics/Robot_Leg.png
	:align: center

	The figure on the left is for the forward kinematics, while the figure
	on the right helps show how the inverse kinematics was developed.

The forward kinematics will follow the Denavit–Hartenberg process [1]_. The
foot position is:

.. math::

	\begin{bmatrix}
		L_c \cos \alpha + L_f \cos \alpha \cos \beta + L_t (-\sin \beta \sin \gamma \cos \alpha + \cos \alpha \cos \alpha \cos \beta \cos \gamma) \\
		L_c \sin \alpha + L_f \sin \alpha \cos \beta + L_t (-\sin \alpha \sin \beta \sin \gamma + \sin \alpha \cos \cos \beta \cos \gamma) \\
		L_f \sin \beta + L_t (\sin \beta \cos \gamma + \sin \gamma \cos \beta)
	\end{bmatrix}

Inverse Kinematics
---------------------

Often, we need to find the angles of the leg joints given a location (x,y,z) of
the foot. To calculate this, remember the leg is composed of 3 revolute joints.
The shoulder is in one plane and both the femur joint and tibia joint are in
another plane. The tote project [2]_ helped with developing the inverse
kinematics, however, there were some errors in those equations.

So let's start with the shoulder, look down from the top, we see the x-y plane.
We can calculate the it's rotation :math:`\alpha` by using x and y of the foot
location.

.. math::
	\alpha = atan2(y,x)

Next, let's look at the plane which the next 2 joints lie in. Now we will calculate
the :math:`\beta` and :math:`\gamma` angles.

.. math::
	m = \sqrt{x^2 + y^2} \\
	f = m - L_{coxa} \\
	\beta_1 = atan2(f,z) \\
	d = \sqrt{f^2 + z^2} \\
	\beta_2 = \arccos( \frac{L_{femur}^2+d^2-L_{tibia}^2}{2 d L_{femur}} ) \\
	\beta = \beta_1 + \beta_2
	\gamma = \arccos( \frac{L_{femur}^2+L_{tibia}^2-d^2}{2 L_{femur} L_{tibia}} )

Now finally, you should notice the definition for :math:`\gamma` is different
between forward and reverse kinematics. So let's fix that ...

.. math::
	\gamma_{fk} = \gamma_{ik} - \pi \\
	\beta_{fk} = \beta_{ik} - \pi/2

Gait
~~~~~~


==================== ===================
k                    step
i                    leg
:math:`p_i`          x,y,z position of leg
:math:`\phi_{i,k}`   gait foot position for leg i at step k
:math:`z_i`          height modifier of leg i
:math:`R_z`          rotation matrix around z axis
==================== ===================

Gaits for robots follows that of animals. There is a rhythmic repetitive pattern
that they use and we will exploit here. The gait is based on the work of Metal [3]_
with some differences.

================== === === === === === === === === === === === ===
            k      0   1   2   3   4   5   6   7   8   9   10  11
================== === === === === === === === === === === === ===
:math:`\phi_{1,k}` 9/9 6/9 3/9 0/9 1/9 2/9 3/9 4/9 5/9 6/9 7/9 8/9
:math:`z_1`        0.1 0.2 0.2 0.1 0   0   0   0   0   0   0   0
================== === === === === === === === === === === === ===

Each leg executes this pattern in the order of: 0, 2, 1, 3. This leads to a stable
tripod gait. Each leg is at a different point of this gait, denoted by an offset.
The offset of each leg is:

======= = = = =
leg     0 1 2 3
======= = = = =
offset  0 6 3 9
======= = = = =

Thus at time 0, Leg 0 is at the beginning (k=0), Leg 1 is at k=6, etc.

The quadruped's linear and rotational movements are decoupled. The equation below
shows how the y axis movement is calculated, but the x axis equation is the same.
Basically, the :math:`\Delta` is the delta change in leg position from the normal
or resting leg position.

.. math::
	\delta(x,y) = linear(x,y) + rotational(x,y) \\
	\Delta_{i,k} = \delta(x,y)/2 - \phi_{i,k} \delta(x,y)

where the i identifies the leg (:math:`i \in [0,1,2,3]`), k is the
step (:math:`k \in [0,1, \ldots 11]`) and linear/rotation are the commanded
linear or rotation movements of the robot.

Now each leg exists in its own leg reference frame. Each leg frame is rotated 45
degrees around the robot. The


The rotation part is handled by taking the normal leg position (x,y,z), rotating it about
the z axis and calculating the delta difference by subtracting off the original
position.

.. math::
	R_z (\theta)=
	\begin{bmatrix}
		\cos(\theta) & -\sin(\theta) & 0 \\
		\sin(\theta) & \cos(\theta) & 0 \\
		0 & 0 & 1
	\end{bmatrix} \\
	rotational(x,y) = R_z(\theta) p_i - p_i

Finally, the new leg position is:

.. math::
	p_i' = p_i + \Delta_{i,k}

Lessons Learned
~~~~~~~~~~~~~~~~~

Servos
--------

Toy RC servos have issues:

* Every servo brand has different range of motion and pulse width timing since
  there is no real standard of what pulse width is what angle. There is a loose
  understanding that RC servo manufacturers shoot for. However, if they are off,
  then the end user has to adjust their system.

* Every servo even within the same brand has a different range of motion

	* A pulse width of 1.5 msec on one servo might be 90 degrees on one servo,
	  but 84 deg on another and 100 on another. It is painful to account for all
	  servo biases across a lot of servos (if you are trying to do high accuracy
	  positioning)

* Quality is an issue too, I had one servo die instantly on me and another is
  beginning to go. Also, I question if they are really meeting their torque
  performance specification.

	* The micros have very low torque, but should be enough for what I am doing,
	  however, they seem to struggle at times when they shouldn't.

		* Again, I bought the cheapest servos I could find.

Power:

* Try to keep your RPi on a different power bus than your motors. I had to put
  a lot of capacitance in to account for large motor draws, but 1 out of a 100 times
  there is a small hiccup that resets my system ... it isn't common, but annoying.

Suggestions
---------------

* Avoid RC servos if possible. My TG9e servos were bought for $2.15 each, but I
  think I wasted a lot of time because of them.

	* Use robot servos like DYNAMIXEL AX-12 ($44 each) or XL-320 ($22 each) which are much
	  more advanced and *should* overcome many of the issues above (e.g.,
	  performance, quality, standards) I noted above. Most university level robotic
	  systems use them and for good reason.

References
~~~~~~~~~~~~~~~

.. [1] https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
.. [2] https://tote.readthedocs.io/en/latest/ik.html
.. [3] Metal, Martin, "Quadrupedal walking robot, statically balanced walker,"
	found PDF on internet somewhere.
