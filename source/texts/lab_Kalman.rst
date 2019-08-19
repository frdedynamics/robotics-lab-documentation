****************************
Kalman Filter
****************************

Introduction
==============================================

You have learned that robot navigation is the problem of guiding a robot towards a goal in Chapter 5. In order to achieve navigation tasks, you need to localize your robot using some sensor systems (mentioned in Chapter 6). If you are doing outdoor applications with your robot, then you may use GPS signals from the satalites. However, it is not reliably applicable for intoor applications.

In order to get the information about the position/orientation of your robot, you may use some camera systems. After some image processing, you can determine your robot's position/orientation. What about your robot is a mobile robot, let say? What if your taks requires going from one room to another? Would you equip all the rooms with expensive cameras on each corners of each room? That would not be the most practical and wisely designed solution.

For instance, how do we get the orientation information from our mobile phones? They are *mobile* at the end, right? Mainly for this purpose, there is a sensor type called IMU (i.e. Inertial Measurement Unit) which can measures the angular velocity and the linear acceleration of the body - in this case your mobile phone.

.. figure:: ../_static/images/IMU.png
          :align: center


There are different techniques to obtain position/orientation of the phone using angular velocity and the linear acceleration. In this tutorial, you will learn how to implement **four** different orientation calculation techniques using MATLAB and `IMU <https://www.spartonnavex.com/imu/>`_. Those techniques are:

#. Integration_of_angular_velocity_ *(only gyroscope)*
#. Inclination_sensing_ *(only accelerometer)*
#. Complementary_filter_ *(accelerometer + gyroscope)*
#. Kalman_filter_ *(accelerometer + gyroscope)*

.. note::
  You will find the explanations of all those techniques in the following sections. The values which has *hat* on top means that they are estimated values.

.. _Integration_of_angular_velocity:

Integration of Angular Velocity
=======================================
Integration of angular velocity principle leans on the the basic idea: *the integral of the velocity is the position*. Since an IMU reads the angular velocity of the body, then *the integration of the angular velocity is the orientation*. In literature the orientation angles with respect to their rotation axes are defined as follows:

.. figure:: ../_static/images/coordinates.png
          :align: center

By this definition, the orientation angles are calculated as:

.. math::

    roll \rightarrow \phi &= \int_{t_0}^{t} G_x dt\\
    pitch \rightarrow \theta &= \int_{t_0}^{t} G_y dt\\
    yaw \rightarrow \psi &= \int_{t_0}^{t} G_z dt\\

where :math:`G_x`, :math:`G_y` and :math:`G_z` are the angular velocities read by the IMU about x,y and z axes respectively in body fixed frame. To find the orientations with respect to world frame, you need this rotation matrix:

.. math::

    \begin{bmatrix}
      \dot\phi \\
      \dot\theta \\
      \dot\psi
    \end{bmatrix}
    =
    \begin{bmatrix}
      1       &sin(\phi) tan(\theta)    &cos(\phi) tan(\theta) \\
      0       &cos(\phi)                &-sin(\phi)\\
      0       &sin(\phi) sec(\theta)    &cos(\phi) sec(\theta)
    \end{bmatrix}
    \cdot
    \begin{bmatrix}
      G_x \\
      G_y \\
      G_z
    \end{bmatrix}

and,

.. math::
  \hat\phi = \phi_{prev} + \dot\phi\\
  \hat\theta = \theta_{prev} + \dot\theta

.. warning::
  In theory, this method of orientation calculation works perfect. However the computers on which we are working are doing discrete calculations. On the other hand, we measure the angular velocity in real world, moving the phone in our hands. When you integrate a continuous signal in a discrete environment, you have an accumulation problem (i.e. drift error).

  .. figure:: ../_static/images/accumulationError.png
            :align: center

  Although Integration_of_angular_velocity_ looks a clean-cut for short period of time, it is not the best solution for longer periods.


.. _Inclination_sensing:

Inclination Sensing
=======================

Accelerometers are sensitive to both linear acceleration and the local gravitational field. If the linear acceleration on the body is negligble, then assume that the only acceleration exerting on the body is the gravity. In our case, as the body stays still, the gravitational acceleration is measured only by the z-axis of the IMU.

.. figure:: ../_static/images/tilt0.png
          :align: center

If we rotate the IMU, let say :math:`\theta` angles around y-axis, then the gravity vector is expressed by x and z components on the IMU readings.

.. figure:: ../_static/images/tilt1.png
          :align: center

.. note::
  There is still no component of gravity vector along the y-axis of the IMU since the rotation was made around y-axis.

In more general form,  the rotation is not only around one axis. If the IMU is rotated around x and y axes, by calling the rotation :math:`\phi` around x-axis, the vectoral representation would be like that:

.. figure:: ../_static/images/tilt2.png
          :align: center

Here the :math:`\theta` and :math:`\phi` angles is found by the following formula:

.. math::

    tan(\theta) = a_x / \tilde a_z = a_x / \sqrt (a_y^2 + a_z^2)\\
    tan(\phi) = a_y / \tilde a_z = a_y / \sqrt (a_x^2 + a_z^2)

Therefore,

.. math::

    \hat\theta = arctan(\frac{A_x}{\sqrt{A_y^2 + A_z^2}})\\
    \hat\phi = arctan(\frac{A_y}{\sqrt{A_x^2 + A_z^2}})

.. seealso::
  In order to find the rotation angles, we may either use rotation matrices or we can approach geometrically. You have already seen how to calculate rotation matrices in your earlier lessons. You can try calculating :math:`\theta` and :math:`\phi` using rotation matrices by yourself and finding out the same results as here. You can check the reference :cite:`tuck2007tilt`.

.. warning::
  As you see, we calculated only pitch and roll angles but not yaw. The reason for that, any motion about z-axis doesn't give any variation in accelerometer readings. It is not possible detect the rotations around z-axis using *only accelerometer*.

.. _Complementary_filter:

Complementary Filter
=======================
Idea behind complementary filter is to take slow moving signals from accelerometer and fast moving signals from a gyroscope and combine them. Accelerometer gives a good indicator of orientation in static conditions. Gyroscope gives a good indicator of tilt in dynamic conditions. So the idea is to pass the accelerometer signals through a low-pass filter and the gyroscope signals through a high-pass filter and combine them to give the final rate.

.. figure:: ../_static/images/complementary.jpg
          :align: center

To implement Complementary_filter_, first a constant :math:`\alpha` angle is chosen as a cut-off value for the filters. The larger :math:`\alpha`, the more the accelerometer measurements are ‘trusted’. As :math:`\alpha` goes to zero, we base our estimate mainly on the gyroscope measurements. A good starting point is :math:`\alpha` = 0.1.

.. math::

  \hat{\phi} = \alpha \cdot \hat\phi_{Acc} + (1-\alpha) \cdot (\hat\phi_{prev}+ \dot\phi_{Gyro} \cdot \Delta t)\\
  \hat{\theta} = \alpha \cdot \hat\theta_{Acc} + (1-\alpha) \cdot (\hat\theta_{prev}+ \dot\theta_{Gyro} \cdot \Delta t)\\

.. _Kalman_filter:

Kalman Filter
=======================
Kalman filter is one of the most common estimation algorithms. It produces estimates of cannot-measured states of a system based on the past estimations and current measurements. In another words, it is an estimator (and observer). Using the system model, it reduces the estimation error in every iteration. As well, the Kalman filter provides a prediction of the future system state.

Kalman filter consists of two parts;
**Prediction** and **Correction**.

In prediction step, the system model is used in calculation. A linear and time-invariant system can be expressed as:

.. math::

  \begin{split}
    \vec{x}_{t+1} &= \textbf{A} \cdot \vec{x}_t + \textbf{B} \cdot \vec{u}_t + \vec{w}_t\\
    \vec{y}_{t+1} &= \textbf{C} \cdot \vec{x}_{t+1} + \vec{v}_{t+1}
  \end{split}

Where :math:`\vec{x}_{t}` is the system’s state vector at time t and \vec{u}_t is the input vector at time t.

A : system matrix (relates the current states to the next states)

B : input bmatrix (relates inputs to the next states)

C : output matrix (system states to the measured states)

:math:`(\vec{w}_t)` : process noise

:math:`(\vec{v}_t)` : measurement noise – both assumed to be zero-mean Gaussian noise.

We then define our state vector, input vector, and measurement vector:

.. math::

  \vec{x}_t = \begin{bmatrix} \hat{\phi}_t \\ b_{\hat{\phi}_t} \\ \hat{\theta}_t \\ b_{\hat{\theta}_t} \end{bmatrix}

  \vec{u}_t = \begin{bmatrix} \dot{\phi}_{G_t} \\ \dot{\theta}_{G_t} \end{bmatrix}\\
  \vec{z}_t = \begin{bmatrix} \hat{\phi}_{Acc_t} \\ \hat{\theta}_{Acc_t} \end{bmatrix}

Where :math:`b_{\phi_t}` is the gyro bias at time t associated with our estimate :math:`\hat{\phi}`.

After defining all the parameters, now we can start building up the Kalman filter.

**Prediction**

.. math::

  \vec{x}_{t+1} = \textbf{A} \cdot \vec{x}_t + \textbf{B} \cdot \vec{u}_t\\
  \textbf{P} = \textbf{A} \cdot \textbf{P} \cdot \textbf{A}^T + \textbf{Q}

**Correction**

.. math::

  \widetilde{y}_{t+1} = \vec{z}_{t+1} - \textbf{C} \cdot \vec{x}_{t+1}\\
  \textbf{S} = \textbf{C} \cdot \textbf{P} \cdot \textbf{C}^T + \textbf{R}\\
  \textbf{K} = \textbf{P} \cdot \textbf{C}^T \cdot \textbf{S}^{-1}\\
  \vec{x}_{t+1} = \vec{x}_{t+1} + \textbf{K} \cdot \widetilde{y}_{t+1}\\
  \textbf{P} = (\textbf{I} - \textbf{K} \cdot \textbf{C}) \cdot \textbf{P}


Where,

**K** is the Kalman gain,

**P** is the error covariance,

**Q** is covariance matrix of the process noise,

**R** is covariance matrix of the measurement noise,

.. note::

  Lower variance in measurement noise (R -> 0) makes the Kalman gain **K** closer to 1 and our estimates will be more based on the measurements.

.. note::

  If in the first case the prior estimate covariance is zero (P -> 0), then only prior estimates contribute to our current estimation.











.. warning::
 The Kalman filter is only applicable in casual, linear and time-invariant systems. If the system model is not satisfy these three conditions, then another type of filter/estimator/observer or a different variation of Kalman filter should be implemented.


Experimental Process
==============================================


.. literalinclude:: ../_static/scripts/KalmanManual.m
   :language: matlab
   :linenos:
   :caption: Example 1. our code.



Conclusion and Further Readings
==============================================

We have seen some filtering algorithms applied on IMU in order to get some orientation data. The most important lesson in this tutorial is to realize that the sensor systems are not completely reliable if you are reading the raw data. As it is mentioned at the beginning, other localization solutions such as using GPS data or camera systems are also requires after-processing as we did on IMU in the tutorial. Today, some of the expensive sensor systems have their own filtering circuits inside the sensor.

See :cite:`Strunk1979` for an introduction :cite:`Corke2011`> to stylish blah, blah...

.. [Peter Corke Ch.6] Detailed mathematical explanation of Kalman filter.

.. bibliography:: ../references.bib
