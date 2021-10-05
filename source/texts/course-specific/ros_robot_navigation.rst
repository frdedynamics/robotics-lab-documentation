.. _ros_robot_navigation:

**********************************
Robot navigation with ROS
**********************************

Launching one robot in gazebo (preparation)
========================================================
The video called **Launch One Robot in Gazebo** should be available on Canvas in the Media Gallery. It explains the different components of the launch file which includes the gazebo world, Turtlebot3, SLAM package and RVIZ. Watch it as a preparation for the Lecture. To replicate what has been done in the video and prepare a ROS package for the lecture the needed files can be downloaded `here <https://hvl365.sharepoint.com/:f:/s/RobotikkUndervisningHVL/EiMkZWhQFVBGuSMwCKt169MBSl2zqY5AUcCk0dvRSBtxQQ?e=6a2zg8>`_.

Go to Point
============================
The robot is supposed to turn until it faces in the direction of the target. Once it does it should stop turning and move forward as long as the robot still faces towards the target point. Repeat until the goal is reached. The following code is a starting point and has to be adapted to achieve the wanted behavior:

.. literalinclude:: ../../_static/scripts/ros_navigation/go_to_point.py
       :language: python
       :caption: go_to_point.py

Follow wall
============================
The robot should have 3 behaviors: find wall, turn left and follow wall. When no wall is close enough, the robot is supposed to move forward and to the right until it finds one. Then the robot should turn left until it faces parallel to the wall. During wall following it should just move forward as long as the previous two statements are still true. The following code is a starting point and has to be adapted to achieve the wanted behavior:

.. literalinclude:: ../../_static/scripts/ros_navigation/follow_wall.py
       :language: python
       :caption: follow_wall.py

Bug2 Algorithm
============================
In the beginning the robot should calculate a direct line between its current position and the target position. It should the start following that line as long as it doesn't run into an obstacle. If it does, it should start following the obstacle until it reaches a point on the line which is closer to the target then the point where it encountered the obstacle. The robot should repeat these behaviors until the goal is reached. You are supposed to use the previous two scripts (got to point and follow wall) to achieve this behavior. Each script should run separately while communicating through ROS the necessary information. The following code is a starting point for the bug2 algorithm script and has to be adapted to achieve the wanted behavior:

.. figure:: ../../_static/images/ros/bug2_algorithm.jpg
          :align: center

.. literalinclude:: ../../_static/scripts/ros_navigation/bug2.py
       :language: python
       :caption: bug2.py

Reading the map
==========================================
The map data published by the SLAM algorithm has the type OccupancyGrid. It is a list of integer values where each value gives shows the probability of an obstacle being there with a value between 0 and 100, where 100 (displayed in rviz as black) means high probability and 0 (white) means no obstacle. The map values can also be -1 which stands for unknown areas.

.. figure:: ../../_static/images/ros/slam_map.PNG
          :align: center

As already mentioned previously, the map is stored in a list of int values. The following image gives a visualization of what list item corresponds to what cell on a two dimensional grid as well as how to access certain properties in python which can be used to transform between grid cells and position values:

.. figure:: ../../_static/images/ros/occupancy_grid.PNG
          :align: center

Move Base
========================================================
Video on how to use move_base navigation stack.

Orientation (Quaternions)
--------------------------
Showing how to use the by me provided function to calculate the orientation needed to send goal commands to the move_base navigation stack.
