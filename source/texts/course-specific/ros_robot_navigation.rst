.. _ros_robot_navigation:

**********************************
Robot navigation with ROS
**********************************

Launching one robot in gazebo (preparation)
========================================================
The video called **Launich One Robot in Gazebo** should be available on Canvas in the Media Gallery. It explains the different components of the launch file which includes gazebo world, turtlebot3, SLAM package and RVIZ. Watch it as a preparation for the Lecture. To replicate what has been done in the video and prepare a ROS package for the lecture the needed files can be downloaded `here <https://hvl365.sharepoint.com/:f:/s/RobotikkUndervisningHVL/EiMkZWhQFVBGuSMwCKt169MBSl2zqY5AUcCk0dvRSBtxQQ?e=6a2zg8>`_.

Go to Point
============================
The robot is supposed to turn until it faces in the direction of the target. Once it does it should stop turning and move forward as long as the robot still faces towards the target point. Reapeat until the goal is reached. The following code is a starting point and has to be adapted to achieve the wanted behaviour:

.. literalinclude:: ../../_static/scripts/ros_navigation/go_to_point.py
       :language: python
       :caption: go_to_point.py

Follow wall
============================

.. literalinclude:: ../../_static/scripts/ros_navigation/follow_wall.py
       :language: python
       :caption: follow_wall.py

Bug2 Algorithm
============================

.. literalinclude:: ../../_static/scripts/ros_navigation/bug2.py
       :language: python
       :caption: bug2.py

Reading the map (preparation or lecture?)
==========================================
Video explaining how to work with Occupacy Grids (data format of the maps created by slam) and how to extract positions from it.

Move Base (Video to watch after lecture?)
========================================================
Video on how to use move_base navigation stack.

Quaternions
-------------
Video with a brief explanation how quaternions work and then showing how to use the by me provided function to calculate the orientation needed to send goal commands to the move_base navigation stack.
