Inertial Navigation
===================

2016-08-26

Inertial navigation systems (INS) uses a combination of accelerometers,
gyros, other sensors, and math to determine a robot\'s position in 3D
space. Today, everyone uses a strap down INS instead of the old
gimbaled/mechanical systems which were very expensive and complex.

This is primarily written as a cheatsheet for INS, not as a tutorial.
Thus lots of equations are presented, but not everything is explained
for a beginner.

Strap-down INS
--------------

For this we will use the Earth-Centered-Earth-Fixed (ECEF) reference
frame to calculate the robot\'s equations of motion. First some terms
and definitions.

  Symbol   Frame
  -------- -----------------------
  n        navigation (like LGV)
  b        body
  c        ECI
  e        ECEF

  Symbol       Definition
  ------------ ------------------------------------------------------------------------------
  $\phi$       roll (motion around x-axis)
  $\theta$     pitch (motion around y-axis)
  $\psi$       yaw (motion around z-axis)
  H            height above median sea level
  $\phi$       latitude (North/South, $\pm$ 90 $^\circ$)
  $\lambda$    longitude (East/West, $\pm$ 180 $^\circ$)
  $\omega_b$   body gyro rates $\begin{bmatrix} \omega_x \omega_y \omega_z \end{bmatrix}^T$
  $\dot V_b$   body acceleration rates $\begin{bmatrix} a_x a_y a_z \end{bmatrix}^T$

  Const                   Value                                  Definition                                            
  ----------------------- -------------------------------------- ----------------------------------------------------- --
  R                       6,378,137.0 m                          Radius Earth                                          
  $\omega_{ie}$           7.292115E-15 rads/sec                  Rotation rate of the Earth wrt ECI (inertial frame)   
  $g_{WGS0}$ $g_{WGS1}$   9.7803267715 m/sec\^2 0.019318538639   Gravity at sea level on the equator                   
  a                       6,378,137.0 m                          WGS84 semi-major axis                                 
  b                       6,356,752.314 245 m                    WGS84 semi-minor axis                                 

Decoder Key
-----------

Superscripts typically are the frame a vector is currently in and
subscripts describe the vector.

$A_{descriptor}^{reference frame}$

  Symbol            What it means
  ----------------- -----------------------------------------------------------------------------------------------------------------------------------------------------------
  $\omega_{ie}$     Earth rotation measure wrt the initial frame.
  $\omega_{eb}^b$   Body rates measured by the gyros wrt ECEF frame in the body frame. The rotation of the Earth is subtracted off the measured body rates to get this value.
  $\omega_{ib}^b$   Body rates measured by the gyros wrt ECI frame in the body frame. Note, this is what you get right off the gyros.

Coordinate Systems
------------------

Earth Center Inertial (ECI)

:   The ECI frame\'s origin is located at the center of the Earth, with
    the x-axis pointing to the vernal equinox. For all intensive
    purposes, this frame is a true inertial (no moving) reference frame.

Earth Centered Earth Fixed (ECEF)

:   The ECEF coordinate system assumes that the origin is at the center
    of the planet, the x-axis intersects the Greenwich meridian and the
    equator, the z-axis is the mean spin axis of the planet, positive to
    the north, and the y-axis completes the right-hand system.

Local Geodetic Vertical (LGV)

:   The LGV frame creates a local frame with the z-axis parallel to the
    local gravity vector. Now, the x-axis and y-axis can be oriented
    several different ways. A common aerospace frame is North (x),
    East(y), Down(z) or NED. This orientation has the characteristic of
    altitude above the ground being negative. Another option is North
    (x), West (y), Up (z) or NWU which now has the local z-axis opposite
    gravity and up being positive. The reference below also show a link
    to the European Space Agency (ESA) and gives a more complex way to
    have a local frame with East (x), North (y), Up (z) or ENU.

Latitude, Longitude, Altitude (LLA)

:   Standard GPS coordinates ($\phi$, $\lambda$, a or latitude,
    longitude altitude (m)) based on WGS84 geodetic reference frame.
    This isn\'t so much a reference frame we use, but a way to determine
    where our robot starts its naviation in the ECEF frame.

Transformations Between Frames
------------------------------

Conversion between EoM and LGV navigation frames[^1][^2]:

$$x_{NED} = C_e^{v-NED} x_ECEF \\
 C_e^{v-NED} =
 \begin{bmatrix}
     -s \phi c \lambda & -s \phi s \lambda & c \phi \\
     -s \lambda        & c \lambda         & 0 \\
     -c \phi c \lambda & -c \phi s \lambda & -s \phi
 \end{bmatrix} \\$$$$C_e^{v-NWU} =
\begin{bmatrix}
    -s \phi c \lambda & -s \phi s \lambda & c \phi \\
    s \lambda         & -c \lambda        & 0 \\
    c \phi c \lambda & -c \phi s \lambda & s \phi
\end{bmatrix}$$

where NED is North, East, Down and NWU is North, West, Up.

Conversion between LLA and ECEF[^3] 1 and[^4] 1.18:

$$x = (\frac{a}{d}+h) \cos \phi \cos \lambda \\
 y = (\frac{a}{d}+h) \cos \phi \sin \lambda \\
 z = (\frac{a(1-e^2)}{d}+h) \sin \phi \\
 d = \sqrt{1-e^2 \sin^2 \phi} \\
 e^2 = 1 - (\frac{b}{a})^2$$

Again, this is mainly to deterine the starting location for an in door
robot. If navigating for a long duration out side, it can be an external
input into the Kalman Filter for the robot\'s current position.

Attitude
--------

Euler angles:

-   3 angles that relate one coordinate frame to another
-   Have a non-linear relationship to body axis angle rates
-   They are non-orthogonal due how the rotations are handled
-   Depending on order, have singularities at different orientations
    that have to be avoided
-   Euler angles are human interpretable, but not typically used in
    equations of motion

From[^5] eqns 3.44-3.48 (note, the subscripts in the book are wrong and
have been corrected here: x-1, y-2, and z-3):

$$C_3 =
 \begin{bmatrix}
     c \psi  & s \psi & 0 \\
     -s \psi & c \psi & 0 \\
     0       & 0      & 1
 \end{bmatrix}$$$$C_2 =
\begin{bmatrix}
    c \theta & 0 & -s \theta \\
    0        & 1 & 0 \\
    s \theta & 0 & c \theta
\end{bmatrix}$$$$C_1 =
\begin{bmatrix}
    1 & 0       & 0 \\
    0 & s \phi  & s \phi \\
    0 & -s \phi & c \phi
\end{bmatrix}$$$$C_n^b = C_1 C_2 C_3 \\
C_b^n = (C_n^b)^{-1} = C_n^{bT} = C_3^T C_2^T C_1^T \\$$

Thus, the transform from body to nav is in the order of roll (x), pitch
(y), and yaw (z). While the reverse, nav to body, is yaw, pitch, and
then roll. The body to nav sequence is also referred to a 1-2-3 (x-y-z)
by some authors. Note the inverse is equal to the transpose of a
rotation matrix and a re-ordering of the individual matrices. The
transformation is[^6] 3.49:

$$C^b_n =
 \begin{bmatrix}
        c \theta c \psi & s \phi s \theta c \psi - c \phi s \psi & s \phi s \psi + c \phi s \theta c \psi \\
        c \theta s \psi & c \phi c \psi + s \phi s \theta s \psi & c \phi s \theta s \psi - s \phi s \psi \\
        -s \theta       & s \phi c \theta                        & c \phi c \theta
 \end{bmatrix} \\$$

Now, depending on what LGV frame you are using, you can calculate the
transformation from body to ECEF:

$$C^e_b = C^e_n C^n_b$$

Ultimately we will use quaternions to avoid singularities.

Quaternions
-----------

Quaternions where described by Olinde Rodriques in 1840 and
independently by William Rowan Hamilton in 1843[^7]. Prior to his
discovery, it was believed impossible that any algebra could violate the
laws of commutativity for multiplication. His work introduced the idea
of hyper-complex numbers. Here real numbers can be thought of as
hyper-complex numbers with a rank of 1, ordinary complex numbers with a
rank of 2, and quaternions with a rank of 4. Hamilton's crucial rule
that made this possible:

$$i^2=j^2=k^2=ijk=-1$$

Hamilton supposedly developed this rule while on his way to a party.
When he realized what the solution was, he took out his pocket knife and
carved the answer into a wooden bridge. This rule would forever change
mathematics as was known at the time. Now mathematicians could look at
algebra where commutativity did not work. This is where Gibbs and others
developed algebra of vector spaces, and quickly eclipsed Hamilton's work
until recently.

Quaternions, also known as Euler symmetric parameters, are more
mathematically efficient ways to compute rotations of rigid and
non-rigid body systems than traditional methods involving standard
rotational matrices or Euler angles. Quaternions have the advantage of
few trigonometric functions needed to compute attitude. Also, there
exists a product rule for successive rotations that greatly simplifies
the math, thus reducing processor computation time. Quaternions also
hold the advantage of being able to interpolate between two quaternions
(through a technique called spherical linear interpolation or SLERP)
without the danger of singularities, maintaining a constant velocity,
and minimum distance travelled between points

The quaternion is composed of a scalar and a vector part. The scalar is
a redundant element that prevents singularities from occurring since the
four elements are all dependent upon each other. There are many
different ways to represent a quaternion[^8] 3.53-3.54:

$$q = \begin{bmatrix} a & b & c & d \end{bmatrix}^T \\
 q = \begin{bmatrix}\cos(\mu/2) & \hat e_x \sin(\mu/2) & \hat e_y \sin(\mu/2) & \hat e_z \sin(\mu/2) \end{bmatrix}^T \\
 q = \begin{bmatrix} q_r & q_x & q_y & q_z \end{bmatrix}^T \\$$

where $\hat e$ is the axis of rotation and $\mu$ is the angle of
rotation about the axis. Also, a quaternion is a complex number with a
real component ($q_r$) and an imaginary component ($q_{xyz}$). The order
of the quaternion elements is not standardized. I have chosen to follow
other complex numbers and do real then imaginary.

### Rigid Bodies Rotations

A rigid body can be rotated about an arbitrary moving/fixed axis
($\hat e$) in space by:

$$q_{x,y,z} = \hat e \sin( \frac{\mu}{2} ) \\
q_r = \cos(\frac{\mu}{2} )$$

Quaternion multiplication ($\otimes$) is[^9] 3.56:

$$q \otimes p =
\begin{bmatrix}
    a & -b & -c & -d \\
    b &  a & -d &  c \\
    c &  d &  a & -b \\
    d & -c &  b &  a \\
\end{bmatrix} \cdot p = Q \cdot p \\$$

Quaternion differential equation[^10] 3.56, 11.34-11.35:

$$\dot q = \frac{1}{2} q \otimes w \\
 w = \begin{bmatrix} 0 & \omega_b \end{bmatrix}^T \\
 \dot q = \frac{1}{2} W q \\
 W =
 \begin{bmatrix}
     0   & -w_x & -w_y & -w_z \\
     w_x & 0    & w_z  & -w_y \\
     w_y & -w_z & 0    & w_x \\
     w_z & w_y  & -w_x & 0
 \end{bmatrix}$$

Now the transformation can also be done using a quaternion rather than
Euler angles[^11] 3.63:

$$C_n^b =
 \begin{bmatrix}
     (a^2+bb^2-c^2-d^2) & 2(bc-ad)          & 2(bd+ac) \\
     2(bc+ad)           & (a^2-b^2+c^2-d^2) & 2(cd-ab) \\
     2(bd-ac)           & 2(cd+ab)          & (a^2-b^2-c^2+d^2)
 \end{bmatrix}$$

Converting between Euler and Quaternions is not always easy, but a
solution that may not always work is[^12] 3.66:

$$\phi = atan2(C_{32}, C_{33}) = atan2(2(cd+ab), (a^2-b^2-c^2+d^2)) \\
 \theta = asin(-C_{31}) = asin(-2(bd-ac)) \\
 \phi = atan2(C_{21}, C_{11}) = atan2(2(bc+ad), (a^2+bb^2-c^2-d^2))$$

See Titterton for solutions when Euler angles are near singularities.

Angular Rates
-------------

Gyros are used to measure body rotation rates with respect to (wrt) the
Inertial (ECI) frame. It is important to understand that Euler rotations
are not orthoginal and you cannot use the transformation given
previously to transform the rates.[^13] p 40.

$$\omega_b = \begin{bmatrix} p & q & r \end{bmatrix}^T \\
 \dot \Theta = \begin{bmatrix} \dot \phi & \dot \theta & \dot \phi \end{bmatrix}^T \\
 \dot \Theta = L_b^I \omega_b \\
 L_b^I =
 \begin{bmatrix}
     1 & \sin \phi \tan \theta & \cos \phi \tan \theta \\
     0 & \cos \phi & -\sin \phi \\
     0 & \sin \phi \sec \theta & \cos \phi \sec \theta
 \end{bmatrix}$$

This is only useful if you are trying to integrate euler angles in an
interal frame and don\'t want to use quaternions.

Titterton ECEF EoM
------------------

These equations follow the derivations in Titterton[^14]. Later
equations from Chatfield are shown to be the same, but Titterton\'s
derivation is a little easier to follow.

$$\newcommand{\dv}[2]{\left. \frac{ dv_{#1} }{dt} \right|_{#2}}$$

The equations of motion in an ECEF frame are[^15] 3.15, 3.19-3.23:

$$\dv{e}{e} = \dv{e}{i} - \omega_{ie} \times v_e \\
 \dv{e}{i} = f - \omega_{ie} \times v_e + g_l \\
 \dv{e}{e} = f - 2 \omega_{ie} \times v_e + g_l \\
 \dot v_e^e = C_b^e f^b - 2 \omega_{ie}^e \times v_e^e + g_l$$

where from before:

$$C^e_b = C^e_n C^n_b$$

The cross product can be replaced with a skew-symmetric matrix[^16] if
desired

$$a \times b = Ab \\
 A = [a]_{\times} =
 \begin{bmatrix}
     0    & -a_3 & a_2 \\
     a_3  & 0    & -a_1 \\
     -a_2 & a_1  & 0
 \end{bmatrix}$$

The local gravity model is given by[^17] 3.14:

$$g_l = g - \omega_{ie} \times [ \omega_{ie} \times r ]$$

Updating the transforms using gyro data[^18] 3.23:

$$\omega_{eb}^b = \omega_{ib}^b - C_e^b \omega_{ie}^e$$

Remember, the gyros measure body rates wrt the inertial frame (i.e.,
$\omega_{ib}^b$) and we need to remove the Earth\'s rotational movement
from the gyro measurements. Since we are using the ECEF frame, we need
to move those measurements into that frame and also subtract off the
rotation of the Earth.

Chatfield ECEF EoM
------------------

The results above are the basically the same as Chatfield[^19] EoM for
ECEF although he puts them into a state space equation:

$$\frac{f}{m} = a = S \\
v_i = v_s + \Omega_{ie} \times r_i \\
\dot v_i = \dot v_s + \dot \Omega_{ie} \times r_i + \Omega_{ie} \times v_i \\
\dot v_i = \dot v_s + \Omega_{ie} \times v_s + \Omega_{ie} \times [\Omega_{ie} \times r_i ] \\
\Omega_{ie} = const \Rightarrow \dot \Omega_{ie} = 0 \\
S^i + g^i = \dot v_s + \dot \omega_{ie} \times v_s + \Omega_{ie} \times [\Omega_{ie} \times r_i ] \\
\dot v_s = S^i + g^i - \omega_{ie} \times v_s - \Omega_{ie} \times [\Omega_{ie} \times r_i ]$$

Now all of these equations were derived in the inertial frame and they
must be transformed into the ECEF frame.

$$\dot v_e = \dot v_s - \Omega_{ie} \times v_s \\
\dot v_s = S^i + g^i - \omega_{ie} \times v_s - \Omega_{ie} \times [\Omega_{ie} \times r_i ]$$

Putting this into state space:

$$\begin{bmatrix}
    \dot V^e \\
    \dot P^e
\end{bmatrix}
=
\begin{bmatrix}
    -2 \Omega^e_{ie} & -\Omega^e_{ie}\Omega^e_{ie} \\
    I & 0
\end{bmatrix}
\begin{bmatrix}
    V \\
    P
\end{bmatrix}
+
\begin{bmatrix}
    R^e_c & R^e_b \\
    0 & 0
\end{bmatrix}
\begin{bmatrix}
    g^c_{SHC} \\
    S^b
\end{bmatrix} \\$$$$\Omega^e_{ie} = \begin{bmatrix}
    0 & -\omega_{ie} & 0 \\
    \omega_{ie} & 0 & 0 \\
    0 & 0 & 0
\end{bmatrix}$$

Now including attitude using quaternions, the equations become:

$$\begin{bmatrix}
    \dot V^e \\
    \dot P^e \\
    \dot \Phi
\end{bmatrix}
=
\begin{bmatrix}
    -2 \Omega^e_{ie} & -\Omega^e_{ie}\Omega^e_{ie} & 0 \\
    I & 0 & 0 \\
    0 & 0 & Q
\end{bmatrix}
\begin{bmatrix}
    V \\
    P \\
    \Phi
\end{bmatrix}
+
\begin{bmatrix}
    R^e_c & R^e_b \\
    0 & 0
\end{bmatrix}
\begin{bmatrix}
    g^c_{SHC} \\
    S^b
\end{bmatrix} \\$$$$\Omega^e_{ie} = \begin{bmatrix}
    0 & -\omega_{ie} & 0 \\
    \omega_{ie} & 0 & 0 \\
    0 & 0 & 0
\end{bmatrix} \\$$$$Q = \frac{1}{2} \begin{bmatrix}
    0 & \omega_z & -\omega_y & \omega_x \\
    -\omega_z & 0 & \omega_z & -\omega_y \\
    \omega_y & -\omega_x & 0 & \omega_z \\
    -\omega_x & -\omega_y & -\omega_z & 0
 \end{bmatrix} \\$$$$\Phi = \begin{bmatrix} q_x & q_y & q_z & q_w \end{bmatrix}^T$$

$$g=g_{WGS0} \frac{1+g{WGS1} \sin(\phi)^2}{ \sqrt{1-\epsilon^2 \sin(\phi)^2}} \\
g^c_{SHC} = \begin{bmatrix}
    \xi  g \\
    -\eta g \\
    g
\end{bmatrix}$$

Sources of Error
----------------

  Source         Description
  -------------- ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Bias           Small offsets in accelerometers of (especially) the gyros lead to incorrect forces which produce more velocity and position changes than is really occurring.
  Scale Factor   This is a calibration issue where the IMU is reporting a proportional amount of the actual accelerations/rotation rates it is really subjected too
  Temperature    An IMU\'s accelerometers and gyroscopes are sensitive to temperature
  Hysteresis     Gyro drift rates and accelerometer biases tend to change each time a unit is switched on. One culprit of this is running white noise through a low pass filter produces a random walk, which contributes to the randomness of drift and bias values.
  Vibration      IMU\'s need to isolated from vibration sources and in some systems, the IMU mount needs to avoid certain resonance frequencies.

Now, unfortunately, using the navigation EoM with inputs from gyros and
accelerometers will most likely not give you good results for a variety
of reasons (some listed above). Thus, some sort of correction needs to
be incorporated and a Kalman filter is typically used to make the
corrections.

Kalman Filter
-------------

  Variable                                        Definition
  ----------------------------------------------- ------------------------------------------
  $x_k$                                           State at time k
  $z_k$                                           Measurement at time k
  $\Phi = \frac{\partial}{\partial x} F(x,u,t)$   Jacobian of the state transition matrix
  $H = \frac{\partial}{\partial x} C(x,u,t)$      Jacobian of the observation matrix
  $Q$                                             Covariance of white process noise
  $R$                                             Covariance of the measurement noise
  $P_k$                                           Error covariance at time k
  $D$                                             Direct transmission of inputs to outputs
  $u$                                             Control inputs
  $v_k$                                           Measurement noise
  $w_k$                                           Process noise

Assume our system is of the following form:

$$\dot x = Fx+Bu+Gw \\
z=Cx+Du+v$$

This process can be modeled (assuming no control inputs for now) as:

$$x_{k+1} = \Phi x_k + w_k \\
z_k = H x_k + v_k  \\
Q = E[w_k w^T_k]  \\
R = E[v_k v^T_k]  \\
P_k = E[e_k e_k^T = E[(x_k - \hat x_k)(x_k - \hat x_k)^T]  \\$$

  Description         Equation
  ------------------- ------------------------------------------------------------------
  Kalman Gain         $K_k = P_k' H^T (H P_k' H^T + R)^(-1)$
  Update Estimate     $\hat x_k = \hat x_k' + K_k (z_k - H \hat x_k')$
  Update Covariance   $P_k = (I - K_k H) P_k'$
  Project into k+1    $\hat x_{k+1}' = \Phi \hat x_k \\ P_{k+1} = \Phi P_k \Phi^T + Q$

### Augmentation

The Kalman filter can be use to estimate unknown parameters. This can be
done by augmenting, or modifying, both the state vector and the state
transition matrix.

$$\begin{bmatrix}
    \Phi_{system} & \Phi_{coupling} \\
    0 & \Phi_{augment}
\end{bmatrix}$$

Aided INS
---------

Kalman filters are typically employed in INS with external measurement
sensors (i.e., GPS, rangers, encoders, etc). In this form, the filter
tracks navigation errors and attempts to correct them.

### Position Error Model

$$\Delta \dot V^e = -2 \Omega_{ie}^e \Delta V^e - \Omega \Omega \Delta P^e + S^e \Delta \phi^e \\
 \Delta \dot P^e = \Delta V^e \\
 \Delta \dot \Phi = \omega_b^e \times \Delta \Phi - C_b^e \Delta w^b \\
 \Delta \dot S^b = \Delta S_N^b \\
 \Delta \dot w^b = \Delta w_N^b$$

Where the terms with subscripts N are white noise to mimic a random
walk. Also, the $\Delta S_N^b$ and $\Delta w_N^b$ represent the
accelerometer and gyro biases which, in this augmented Kalman filter,
are being estimated.

$$P^e = ? \\
 V^e = ? \\
 \omega^e = ?$$

### Attitude Error Model

References
----------

Resources
---------

-   <https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python>
-   [MIT Kalman filter
    derivation](http://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf)
-   [Interactive Kalman filter demo and
    explaination](http://home.wlu.edu/~levys/kalman_tutorial/)
-   [Kalman filter on
    Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)
-   [Extended Kalman filter on
    Wikipedia](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
-   [Extended Kalman filter tutorial from University of
    Buffalo](https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf)
-   [My Masters
    Thesis](http://walchko.github.io/pages/Publications/walchko-MS-EE.pdf)
-   [Mathworks ECEF to
    NED](http://www.mathworks.com/help/aeroblks/directioncosinematrixeceftoned.html)
-   [ESA ECEF to
    ENU](http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates)

[^1]: <http://www.mathworks.com/help/aeroblks/directioncosinematrixeceftoned.html>

[^2]: Chatfield, \'Fundamentals of High Accuracy Inertial Navigation,\'
    AIAA, Vol 174, 1997.

[^3]: Drake, \'Converting GPS Coordinates (φλh) to Navigation
    Coordinates (ENU),\'
    <http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf>,
    April 2002.

[^4]: Chatfield, \'Fundamentals of High Accuracy Inertial Navigation,\'
    AIAA, Vol 174, 1997.

[^5]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^6]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^7]: <https://en.wikipedia.org/wiki/History_of_quaternions>

[^8]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^9]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^10]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^11]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^12]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^13]: Stengel, \'Aircraft Equations of Motion 2,\'
    <http://www.princeton.edu/~stengel/MAE331Lecture9.pdf>

[^14]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^15]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^16]: <https://en.wikipedia.org/wiki/Skew-symmetric_matrix#Cross_product>

[^17]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^18]: Titerton, \'Strapdown Inertial Navigation Technology, 2nd Ed,\'
    Progress in Astronautics and Aeronautics, Vol 207, 2004.

[^19]: Chatfield, \'Fundamentals of High Accuracy Inertial Navigation,\'
    AIAA, Vol 174, 1997.
