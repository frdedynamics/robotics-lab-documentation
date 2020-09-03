********************************
Lab 4 ROS Turtlebot Tutorial 1
********************************

.. warning::
    Batteries are dangerous.

.. note::
    You can do this lab virtually #COVID-19.

Theme
==============================================

#. Turtlebot tutorials
#. ROS
#. Turtlebot
#. Bringup
#. Get familiar

Equipment
==============================================
#. PC with ROS installed
#. OPTIONAL: A real turtlebot
#. Turtelbot tutorials from `robotis.com <https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>`_

Before the lab
==============================================
#. Install ROS.

#. Try to get together in a group of 2-4 people.

#. To use a real turtlebot, find your way to the HVL Robotics lab.


Report
==============================================
There is no need to hand in a report after this lab.

Signed attendance will suffice as approved lab exercise.

Tasks
==============================================
#. `[ROS 1] Setup, PC Setup, Remote PC`_
#. `[ROS 1] Simulation`_
#. `[ROS 1] Bringup`_
#. `[ROS 1] Basic Operation`_


_`[ROS 1] Setup, PC Setup, Remote PC`
==============================================
Follow the `pc setup tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/>`_ to set 
up your PC ("Remote PC"). The real turtlebots themselves should be ready to go. 


_`[ROS 1] Simulation`
==============================================
This lab is done with simulation. Using a real robot is optional.

Complete tasks up to and including 11.2.1.3. This should let you drive the turtlebot around in a few
simulated worlds using your keyboard.

See `simulation tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation/>`_.

Take special note of 

roslaunch turtlebot3_gazebo turtlebot3_world.launch

which can be used to replace the real robot and world for testing in later cases.


_`[ROS 1] Bringup`
==============================================
Follow the `bringup tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/#ros-1-bringup/>`_ to 
bringup the turtlebot and show it in RViz.


_`[ROS 1] Basic Operation`
==============================================
Follow the `basic operation tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/#ros-1-basic-operation/>`_ to 
bringup the turtlebot. If you don't have a turtlebot, then skip all the [TurtleBot] parts and rather
launch a simulated robot and world.

Focus on keyboard only for teleoperation. Though, we do have ALL the other controllers at the lab, just saying.

Skip the Additional Sensors part, we don't have those.


Questions
==============================================

#. What is the Turtlebot3 Waffle Pi?
#. Did you use a real robot?
#. Which flavour of OS and ROS did you use?
#. What is a launch file?
#. What is a key difference between RViz and Gazebo?
#. Did you remember to have fun?

The next ROS lab will continue the tutorials.
