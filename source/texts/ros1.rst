***************************************
L1- Getting Started with Turtlebots
***************************************

Before diving into the core of ROS, let's see and practice on capabilities of ROS. We are going to simulate a *Turtlebot Waffle Pi* with and without the *Open Manipulator* on. Turtlebot Waffle Pi is a generic differential robot with 2 actuated wheels and one passive ball joint. It is the same model which you are supposed to use in your project.

In this lecture, we are going see how to visualize, control, simulate such a robot in the ROS environment as well as exploring the available data using the ready-to-use turtlebot packages.

.. figure:: ../_static/images/ros/turtlebot3_models.png
          :align: center

          Source: `emanual.robotis.com <https://emanual.robotis.com/docs/en/platform/turtlebot3/features/>`_


.. figure:: ../_static/images/ros/RobotLAB_Waffle_Pi_OpenManipulator.png
          :align: center

          Source: `www.robotlab.com <https://www.robotlab.com/store/robotis-turtlebot-openmanipulator>`_



Theme
==============================================

#. Turtlebot tutorials
#. ROS
#. Turtlebot
#. Bringups
#. Get familiar

Equipment
==============================================
#. PC with ROS installed
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
#. `[ROS 1] Turtlebot Control`_
#. `[ROS 1] Basic Operation: With Simulated Robot`_




_`[ROS 1] Setup, PC Setup, Remote PC`
==============================================

.. note::
   The turtlebot packages are installed in the VM copy provided to you. However, for those who used a different installation method should have had those packages available in your \src folder:

   * OM+TB3: https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3
   * OM: https://github.com/ROBOTIS-GIT/open_manipulator and https://github.com/ROBOTIS-GIT/open_manipulator_simulations
   * TB: https://github.com/ROBOTIS-GIT/turtlebot3.git
   * TB manipulation: https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations and https://github.com/ROBOTIS-GIT/turtlebot3_manipulation
   * ros_controll ``sudo apt install ros-melodic-ros-controll ros-melodic-ros-controllers``

   **How to**:
   ::

      cd ~/catkin_ws/src
      git clone -b master https://github.com/XXX # For all links
      rosdep install --from-paths src --ignore-src -r -y
      catkin_make
      source devel/setup.bash
      rospack profile


_`[ROS 1] Simulation`
==============================================
This lab is done with simulation. Using a real robot is later.

See `simulation tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation/>`_.

Simulations in ROS are done by both `RViz <http://wiki.ros.org/rviz>`_ and `Gazebo <http://gazebosim.org/>`_. These are not interchangable. Rviz is a ROS package for visualization purposes. On the other hand, Gazebo is a simulation environment with physical properties with gravity, lights/shadows, collisions, mass etc. Gazebo simulations can be considered as *real* robot since it requires `Gazebo ROS control <http://gazebosim.org/tutorials/?tut=ros_control>`_.

Turtlebot with RViz
---------------------

.. figure:: ../_static/images/ros/tb_rviz.png
          :align: center

          Turtlebot in Rviz

#. Run the command: ``roslaunch turtlebot3_bringup turtlebot3_model.launch``
#. Fiddle with the **joint state publisher** gui.

.. note::
   You can run Rviz standalone with ``rosrun rviz rviz`` and load the desired tools on the left bar.


Turtlebot with Gazebo
----------------------

.. figure:: ../_static/images/ros/tb_gazebo.png
          :align: center

          Turtlebot in Gazebo

#. Run the command: ``roslaunch turtlebot3_bringup turtlebot3_empty_world.launch``
#. Navigate in Gazebo tools.

.. note::
   You can run gazebo as a ROS node with ``rosrun gazebo_ros gazebo`` or as standalone software by simply typing ``gazebo`` in the terminal. Please see the difference between these two commands. One starts a node which can communicate by other ROS nodes. The other one starts a standalone Gazebo software which can be used seperately than ROS.

   You can check the difference by checking which ROS nodes are running by ``rosrun rqt_graph rqt_graph`` or just listing available ROS nodes by ``rosnode list``. There will be no Gazebo related ROS node with ``gazebo`` terminal command but a node named **/gazebo** with ``rosrun gazebo_ros gazebo`` ROS command.


_`[ROS 1] Turtlebot Control`
==============================================
When you launch the Gazebo simulated robot, there are several nodes started and those nodes publish/subscribe topics. 

.. figure:: ../_static/images/ros/rostopic-list.png
          :align: center

          Available topics


#. Run the command: ``roslaunch turtlebot3_bringup turtlebot3_empty_world.launch``
#. See which nodes and topics are available:

   #. ``rosnode list``
   #. ``rostopic list``
   #. ``rostopic info /joint_states``
   #. ``rostopic echo /joint_states``

Normally, you can control wheel joints by publishing ``/joint_states`` topic in Rviz. However, to control a *real* robot you need a controller. There are various ways in achieving it in Gazebo but for this tutorial we have an already set *teleoperation* package which enables sending velocity commands to the robot by PC keyboard.

#. Run the command: ``roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch``
#. Use W-A-S-D to control the robot speed
#. Check the available nodes: ``rosrun rqt_graph rqt_graph``
#. Chech the message in ``/cmd_vel`` with the command ``rostopic echo /cmd_vel``
#. Get info about the topic ``rostopic info /cmd_vel``
#. See what features have the **rosmsg info geometry_msgs/Twist``
#. Kill the teleoperation launcher with Ctrl+C.
#. Control the robot velocity by publishing ``/cmd_vel`` via terminal (Use Tab auto completion): 

   ::

      rostopic pub cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
      angular:
      x: 0.0
      y: 0.0
      z: 2.0" 


Questions
==============================================

#. What is the Turtlebot3 Waffle Pi?
#. What is Rviz?
#. What is Gazebo?
#. What is a key difference between RViz and Gazebo?
#. How can you see running nodes?
#. How can you see available topics?
#. How can you see the message type of a topic?
#. **Bonus**: How can you see that topics are published/subscribed by a ROS node?


