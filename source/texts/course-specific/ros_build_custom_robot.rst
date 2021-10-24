.. _ros_custom_robot:

******************************************
ROS Build a Custom Robot
******************************************

In this Chapter we will look at how to build a gazebo model of a robot given the mechanical design. The following topics will be covered:
 
 * Building up the model in Gazebo
 * 
 
Building a Gazebo Model
=========================

::

 cd ~/catking_ws/src
 catkin_create_pkg custom_robot_tutorial rospy geometry_msgs urdf
 cd ..
 catkin_make
 source devel/setup.bash
 rospack profile
 roscd custom_robot_tutorial
 mkdir launch urdf config meshes worlds scripts rviz
 touch urdf/mobile_manipulator_robot.xarco
