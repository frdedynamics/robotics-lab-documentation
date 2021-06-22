.. raw:: latex

   \pagebreak

L0-Introduction
===============

All they need to know in one document like a summary. Serves as cheat
sheet through the whoe ROS teaching.

Installation
------------

-  VM install
-  VM .ova import

Important Shortcuts
-------------------

-  Open terminal
-  Copy Paste
-  tab auto-complete

Important Commands
------------------

-  roscore
-  rosrun
-  roslaunch
-  catkin_make
-  source
-  rospack profile
-  rostopic echo/list/info
-  rosnode echo/list/info
-  rqt
-  rosmsg info
-  roscd
-  rosparam list/load/get/set
-  rosdep install –fropaths src…

Dictionary
----------

-  ROS master
-  node
-  launch
-  publisher
-  subscriber
-  package
-  topic
-  message
-  gazebo
-  rviz
-  rqt
-  urdf/sdf
-  parameter server
-  tf

Installing TB3 packages
-----------------------

OM+TB3: https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3

::

   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   source devel/setup.bash
   rospack profile

OM: https://github.com/ROBOTIS-GIT/open_manipulator and
https://github.com/ROBOTIS-GIT/open_manipulator_simulations

(If you don’t have it, you can’t load ``open_manipulator_description`` )

TB: https://github.com/ROBOTIS-GIT/turtlebot3.git

(The simulation in Gazebo is without OM)

OM+TB:
https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations and
https://github.com/ROBOTIS-GIT/turtlebot3_manipulation

​ –> removed ``open_manipulator_with_tb3`` after these installation

for moveit+gazebo or real robot:

-  ``sudo apt install ros-melodic-moveit-simple-controller-manager``,
-  ``ros-melodic-effort_controllers``
-  ``sudo apt install ros-melodic-ros-controll ros-melodic-ros-controllers``
-  Very hardcore for **Unknown controller type**:
   ``sudo apt install ros*controllers*``

Appendix
--------

ROS Cheat Sheet

.. raw:: latex

   \pagebreak
