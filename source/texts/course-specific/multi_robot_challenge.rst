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
