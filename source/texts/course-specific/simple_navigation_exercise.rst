.. _ros_simple_navigation_exercise:

***************************************************
[Exercise] Simple Navigation on a Turtlebot Robot
***************************************************

In this exercise you will create a controller which navigates a turtlebot robot through an obstacle course. The steps to complete this task will be outlined but the exact code/commands needed you will have to figure out yourself using what you have learned in the lecture, the information available on this website or what you can find on the internet.

ADD A GENERAL OUTLINE WHAT TO DO IN THE EXERCISE. ALSO EXPLAINING THE LOGIC NEEDED

Setup Process
==============================================

#. Create a ros package with the name **simple_navigation_exercise** and the following dependencies: **std_msgs**, **rospy**, **gazebo_ros**.
#. Inside the package create new folders with the names **launch** and **worlds**.
#. Download already prepared files here (ADD LINK TO THE DOWNLOAD)
#. Copy the .launch file into the launch folder, the .py file in the src folder and the .world file in the worlds folder.
#. Open a terminal and navigate to the catkin_ws folder
.. literalinclude:: ../../_static/scripts/ros_communication/custom_msg.msg
       :language: C
       :caption: Custom Message Type: rand_num.msg
#. Run catkin_make
    Testing stuff  
#. Run source devel/setup.bash
#. Make the python script executable
#. Test if the setup process was done correctly by running the launch file. Gazebo should open up with the obstacle course and a turtlebot.

ADD SCREENSHOT OF HOW IT LOOKS

Exercise
==============================================
#. Open up the .py file with the editor of you choice (e.g. Atom). Inside the python script you will comments showing you where to write the different code snippets.
#. Initialize a ros node
#. Create a subscriber to the **/scan** topic using as a callback function the already existing function inside the class called **clbk_laser**
#. Create a publisher to the **/cmd_vel** topic 
#. Inside the **runNavigation** function...

