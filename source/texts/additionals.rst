*****************
Additionals
*****************

We can add whatever extra is needed here.

* :ref:`ROS with Matlab`
* `AR tags <http://wiki.ros.org/ar_track_alvar>`_
* `OpenCV and ROS <https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html>`_, `OpenCV and ROS 2 <http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython>`_
* `2 Turtlebots SLAM <https://www.youtube.com/watch?v=ndvwDFi-I3I>`_
* `Multiple Robot Control in Gazebo <https://www.youtube.com/watch?v=es_rQmlgndQ>`_
* Open Manipulator with Moveit
* UR5 with Moveit


ROS with Matlab
=================

.. note::
   This section is covered only for ELE306 students.

.. note::
   Matlab has already been installed in the VM copy you have but it is not activated. Please follow the instructions on the ~/Desktop to activate Matlab using your own account.
 
We have seen that ROS is a very powefull tool which allows us to use many predefined packages including various robots' packages, various algorithms, sensor drivers etc. On the other hand, we have another powerful in our hands for which mostly we use signal processing, modelling, in control process. It would be cool if we used them together. 

Recently Matlab included **ROS** under its Robotics Toolbox which enables the ROS environment accessible via Matlab. Now, we are going to set our Matlab to control turtlesim which we started from our ROS.

Beneficial tutorials:

#. `ROS Toolbox <https://se.mathworks.com/help/ros/index.html?s_tid=CRUX_lftnav>`_

As we have :code:`roscore` on ROS, there is an equivalent on Matlab as :code:`rosinit` to start the ROS master in the network. There is a slight difference of starting the ROS master between using :code:`roscore` and :code:`rosinit`. If you type :code:`rosnode list`, you will see a global node which was started by Matlab but you won't see :code:`\rosout` (as we are used to see). In usage, there is not a significant difference right now since we are not going to connect a real robot. Just keep in mind that if you need to include more modules/robots/PC in the same ROS network, they should be connected to the same ROS master. If you want to initialize your ROS master on a specific address you should specify it :code:`rosinit('192.168.17.157')`.

Since we successfully ran the ros master, we can create the first publisher. There are two ways of doing it:

#. Via Simulink
#. Via Matlab Script

Via Simulink
~~~~~~~~~~~~~~
This is the most intuitive and the easiest way of creating a ROS publisher using Matlab. Using Simulink, please create the following model.

.. figure:: ../_static/images/ros_simulink.png
        :align: center

.. note::
   I will show you the communication between two environments (Ubuntu and Matlab). For those who use ROSDS, this activity won't work because ROSDS works on a cloud PC which is not connected the same network as our personal computer. 

   Since ROSDS users do not have any preinstalled ROS distribution, they cannot run :code:`turtlesim` nodes. On Matlab, there is a similar tutorial simulation. I will show this part after I am done with the *turtlesim* tutorial.

**On turtlesim:**

* So, let's start the tf example of two turtles: :code:`roslaunch turtle_tf turtle_tf_demo.launch`.
* Check the available ROS topics: :code:`rostopic list`.
* The topic in which we would like to publish is: :code:`/turtle1/cmd_vel`.

**On ExampleHelperSimulinkRobotROS:**

* After initializing the ROS master, you need to type :code:`sim = ExampleHelperSimulinkRobotROS;` on the Matlab command window.
* Check the available ROS topics: :code:`rostopic list`.
* The topic in which we would like to publish is: :code:`/mobile_base/commands/velocity`.
* Set the simulink publisher link to :code:`/mobile_base/commands/velocity`.

Via Matlab Script
~~~~~~~~~~~~~~~~~~
.. note::
   You can write this code directly on the Command Window also.

**On turtlesim:**
Copy this piece of code into a new script which you created under the work directory of Matlab.

 .. literalinclude:: ../_static/scripts/turtlesim_control_script.m
       :language: Python

**On ExampleHelperSimulinkRobotROS:**

* Chance the :code:`/turtle1/cmd_vel` into :code:`/mobile_base/commands/velocity`.


AR tags
=========
.. note::
   This section is covered only for DAT160 students.

AR tags are QR-code like black and white images which can be created with a unique ID number and used as an identification of a place/object/state in the robot world.

.. figure:: ../_static/images/ros/artags.png
          :align: center

          Example AR tags

There is a specigic package called ``ar-track-alvar`` available in ROS enables to create and read AR tags. This package is not installed in default. Please run the following commang to install it:

``sudo apt install ros-melodic-ar-track-alvar``

To generate new AR tags run the ROS node: ``rosrun ar_track_alvar createMarker``, give a prefered ID number and leave the dimension/position as default by pressing only **Enter** without any value entered.


Navigation From Scratch
=========================

.. note::
   This section is covered only for DAT160 students.


Multiple Robot Control in Gazebo
==================================

.. note::
   This section is covered only for DAT160 students.





MoveIt
========

.. note::
   This section is covered only for ELE306 students.

MoveIt Only Visualization
-----------------------------
asdasd

Control a Real Robotic Arm with MoveIt
----------------------------------------
asdasd
