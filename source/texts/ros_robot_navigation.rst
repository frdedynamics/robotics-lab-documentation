**********************************
Robot navigation with ROS
**********************************

Launching one robot in gazebo
==============
Writing a launch file which includes gazebo world, SLAM package, RVIZ.

Go to Point
==============
Most of the code is given here with an explanation what it does during the lecture. The students have to complete the code by adding the ros communication: creating a ros package, figuring out what sensor data is needed and subscribing to it.

Follow wall
==============
Similar to the previous point.

Bug2 Algorithm
==============
Similar to previous point. Students will have to use services here to use go to point and follow wall to combine it into a bug2 algorithm.

Reading the map
==============
Video explaining how to work with Occupacy Grids (data format of the maps created by slam) and how to extract positions from it.

Move Base
==============
Video on how to use move_base navigation stack.

Quaternions
-------------
Video with a brief explanation how quaternions work and then showing how to use the by me provided function to calculate the orientation needed to send goal commands to the move_base navigation stack.
