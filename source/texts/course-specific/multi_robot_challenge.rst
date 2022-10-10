.. _ros_multi_robot_challenge:

***************************************************
[Semester Project] Multi Robot Challenge
***************************************************

For the group project you will work as part of a team to solve a Search and Rescue (S&R) scenario with a small team of mobile robots. Typical skills to be acquired/demonstrated in the project include mobile robot control, navigation and localisation, robot teams, robot software architectures, and the Robot Operating System (ROS). A more detailed description can be found in canvas in **Filer/Semester Projects/DAT160_semester_prosjekt_H22.pdf**


Setup Process
==============================================

* Download the ROS package `here <https://hvl365.sharepoint.com/:f:/s/RobotikkUndervisningHVL/EqTtMd4lm9NPhQGufrERl7ABKuiDbOkWEyqk_Pd5WJN01w?e=tC4rtg>`_
* Copy it into catkin_ws/src folder
* Delete the build folder in your catkin_ws
* In a terminal run:

::

  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  rospack profile

* Copy the folders inside the models folder of the ROS package and copy them into ~/.gazebo/models/
* To spawn the robots in the different worlds use the launch files rescue_robots_w1.launch - w5.launch.


Namespaces
==============================================
In ROS, Namespaces are used to allow multiple topics with the same name to exists. It works similar to a folder structure in any operating system, where inside one folder no duplicate names are allowed but in a different folder a file can have the exact same name. We therefore use Namespaces to have multiple topics with the same name e. g. when launching multiple Turtlebots we need two different "cmd_vel" topics to control each of the robots.

Inside a launch file you can use the following code to launch a node inside a namepsace. **Don't forget to replace the in uppercase written names**.

::

  <group ns="NAMESPACE_NAME">
      <node pkg="ROS_PACKAGE_NAME" name="NODE_NAME" type="NAME_OF_PYTHON_SCRIPT" output="screen" />
  </group>

**When referencing topic names inside a Namepspace be aware of the syntax.** Defining the name with a leading "/" means that I will defined the full name with all Namespaces e.g. "/tb3_0/odom". If you don't put the leading "/" you are defining the name from the Namespace you are in. If I launch my ROS Node inside the Namespace "tb3_0" I can then reference the same topic as before with just "odom".


Exercise 1 - Namespaces
==============================
Inside the given ROS package **multi_robot_challenge_22** do the following steps:

* Create 2 launch files which start a teleoperation keyboard for each robot using the following example code:

::

  <launch>
    <group ns = "NAMESPACE_NAME">
      <node pkg="turtlebot3_teleop" type="turtlebot3_teleop_key" name="turtlebot3_teleop_keyboard"  output="screen" />
    </group>
  </launch>

* Create a python class used for the robot controller
* Create a python class used for the leader
* Create a launch file which launches the robot controller class 2 times in the same Namespaces as the Turtlebots and the leader class outside of any Namespaces
* Each robot controller class should subscribe to the LiDAR topic of the Turtlebot in the same Namespace and publish a single value in a topic called "namespace_com"
* The leader class should subscribe to both "namespace_com" Topics and print out whatever is received. Add an identifier to the print out so that you know which value comes from which robot class.

Exercise 2 - AR-Tags
==============================
Continue inside the ROS package **multi_robot_challenge_22** with the following steps:

* In the robot class, create a subscriber to the topic "ar_pose_maker".
* If any AR Tags are registered publish the id in a topic "marker_id"
* In the leader class, subscribe to both "marker_id" topics and print out what has been received.
* To check your work use the teleop keyboard to move the Turtlebots in front of an AR Tag
