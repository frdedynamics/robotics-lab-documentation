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



Link definition
---------------------


Inertial Parameters
---------------------
gazebo tutorial on calculation inertia parameters: http://gazebosim.org/tutorials?tut=inertia
automated inertia parameters calculator from mesh file (mesh cleaner): https://www.hamzamerzic.info/mesh_cleaner/
formulas for calculating simple shape inertia parameters: https://en.wikipedia.org/wiki/List_of_moments_of_inertia


Joint definition
---------------------
urdf joint definition: http://wiki.ros.org/urdf/XML/joint
