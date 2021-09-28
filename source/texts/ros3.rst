.. _First-Package-Nodes-Launchers-Parameters:

**********************************************
Creating Your First ROS Package
**********************************************

Let's start hands on activities!

Create Package
================
A ROS package is simply a folder contains executables. It must have a **CMakeLists.txt** and **package.xml** files to define dependencies include it as a *ROS package* to the ROS environment. **CMakeLists.txt** is a build script, it does not explicitly express the run dependencies of the targets that get built. **package.xml** contains meta-data (in addition to the dependency declarations you identified): things not necessarily needed for building (or even running) a package, but still nice to have around in a central place. Examples would be author, maintainer, url, description and license. More about these files: `CMakeLists.txt <http://wiki.ros.org/catkin/CMakeLists.txt>`_ and `package.xml <http://wiki.ros.org/catkin/package.xml>`_.

Apart from these two files, the folders are up to you (a.k.a. the developer). However, to have a consistency with all the ROS community, you should not place your files unintendedly. A mid-size ROS package has generally these folders:

   .. figure:: ../_static/images/folders.png
          :align: center

Basically, a **/src** folder for source codes (nodes), a **/launch** folder for launch files (written in xml with .launch extension), an **/include** folder for header files, a **/config** or **/cfg** folder for configuration parameters (mostly .yaml files), a **/worlds** for Gazebo world files (for .world extension) if the package is aimed to use a simulated robot, a **/urdf** folder where robot models (written in xml, .urdf, .xacro, .sdf extensions), and finally a **/models** folder to keep any custom models drawn in some CAD programs (mostly .stl files in this folder). Except for these, depending on the package purpose, there can be some other folders as well.

.. seealso::
   Complete the `ROS tutorials on creating a new package <http://wiki.ros.org/catkin/Tutorials/CreatingPackage>`_.

Create Publisher and Subscriber
================================
Publishers and Subscribers are what we have been telling so far as **nodes**. Publishers provide data to the ROS server and subscribers retrieve data from the server. They are run using the following command:

``rosrun package_name node_name``

``rosrun package_name node_name.py``

``rosrun package_name node_name.cpp``

.. note::
   You need to have the ROS master running to start a node. (Remember ``roscore``).


Simple Publisher
--------------------
::

   #!/usr/bin/env python
   # license removed for brevity
   import rospy
   from std_msgs.msg import String

   def talker():
      pub = rospy.Publisher('chatter', String, queue_size=10)
      rospy.init_node('talker', anonymous=True)
      rate = rospy.Rate(10) # 10hz
      while not rospy.is_shutdown():
         hello_str = "hello world %s" % rospy.get_time()
         rospy.loginfo(hello_str)
         pub.publish(hello_str)
         rate.sleep()

   if __name__ == '__main__':
      try:
         talker()
      except rospy.ROSInterruptException:
         pass


Simple Subscriber
------------------
::

   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String

   def callback(data):
      rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
      
   def listener():

      # In ROS, nodes are uniquely named. If two nodes with the same
      # name are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
      rospy.init_node('listener', anonymous=True)

      rospy.Subscriber("chatter", String, callback)

      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()

   if __name__ == '__main__':
      listener() 

.. seealso::
   Complete the ROS tutorials on simple publisher and subscriber in the `following link <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_.


Understanding ROS Topics
-------------------------
Please see the comprehensive `ROS topics tutorials on the official guide <http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_.

Practical
----------
We are ready to play around now. Let's control the turtlesim with our own publisher. 

Let's first start a GUI form of a turtlebot (well, it is turtlesim) using the following command: ``rosrun turtlesim turtlesim_node``

To control the turtle, here is the code for you. Don't celebrate it immediately for that you have the source code, there are some empty lines for you!

 .. literalinclude:: ../_static/scripts/turtlebotPublisher.py
       :language: Python

**Hint:** To find out which topic name you should publish your message, use :code:`rostopic list`.


.. seealso::
   Can you make your turtlesim to draw a square by using your own publisher?


Create Launcher
=================
A launcher file is an XML file which can start ROS master, multiple nodes with required arguments, set parameters and start the system in a desired state using only one command:

``roslaunch package_name launch_file_name.launch``


.. note::
   You don't need to start ROS master to execute a launch file. (No need ``roscore``). If there is a ROS master running, then the launcher starts the inside nodes on this ROS master. If there is not a ROS master running, then the launch file starts the ROS master before starting any nodes.

A simple launch file looks like this:

.. code-block:: xml

   <?xml version="1.0" encoding="UTF-8"?>

   <launch>

      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_pkg)/urdf/my_robot.xacro'"/>

      <!-- Combine joint values -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

      <!-- Show in Rviz   -->
      <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_pkg)/launch/config.rviz"/> 

      <!-- publish joint state-->
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
         <param name="use_gui" value="True"/>
      </node>

      <node name="a_random_node_name" pkg="package_name" type="node_name" output="screen" >
         <param name="speed" type="int" value="100" />
         <rosparam command="load" file="$(find my_package)/config/parameters.yaml" />
      </node>

   </launch>


.. seealso::
   Check out the `roslaunch tutorial <http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch>`_ starting from 2.2 in the given link.

Services and Parameters
-------------------------
Services are another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response.

   .. figure:: ../_static/images/ros/params-and-services.png
          :align: center

.. note::
   ROS services are not going to be a main target for now, so we will not hold any hands on activities about them. For those who would like to get more information about ROS services, please follow the `Services <http://wiki.ros.org/Services>`_ and `rosservice <http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams>`_ links.

Parameters, on the other hand, are very useful to store and manipulate data in the ROS server. They can be set in a launch file manually or they can be loaded (again in a launch file) by simply calling a script file mostly with .yaml extension.

Let's first start the turtlesim again: ``rosrun turtlesim turtlesim_node``

A single turtle on a console window appeared. Try to solve the steps below by yourself:

#. See which nodes are running 
#. See which topics are available
#. See wich parameters are loaded
#. Get the value of ``/turtlesim/background_r``
#. Set the value 255 of ``/turtlesim/background_r``



.. admonition:: Solution
   :class: dropdown

    ::

      rosrun rqt_graph rqt_graph # or rosnode list
      rostopic list
      rosparam list
      rosparam get /turtlesim/background_r
      rosparam set /turtlesim/background_r 255

.. admonition:: Troubleshoot
   :class: dropdown

   Nothing change when you set the parameter? Well, the value 255 is *loaded* into parameter server but it *has not been changed*. For that you need to clear the set values: ``rosservice call /clear``

Extra
=======
ROS world is big. There are lots of things to touch uppon but we aimed to give a target-based and condense information so that you can use in your course project. Custom message types, action-clients, various ROS commands are waiting to be explored by enthusiasts. Please check the `official ROS tutorials <http://wiki.ros.org/ROS/Tutorials>`_.


Questions
============

#. What is a launch file? How to run?
#. What is a node? How to run?
#. What is a parameter in parameter server?
