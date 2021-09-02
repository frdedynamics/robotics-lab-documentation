*****************
Additionals
*****************

We can add whatever extra is needed here.

* :ref:`with-Matlab`
* `AR tags <http://wiki.ros.org/ar_track_alvar>`_
* `OpenCV and ROS <https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html>`_, `OpenCV and ROS 2 <http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython>`_
* `2 Turtlebots SLAM <https://www.youtube.com/watch?v=ndvwDFi-I3I>`_
* `Multiple Robot Control in Gazebo <https://www.youtube.com/watch?v=es_rQmlgndQ>`_
* Open Manipulator with Moveit
* UR5 with Moveit


.. _with-Matlab:

ROS with Matlab
=======================

.. note::
   This section is covered only for ELE306 students.

.. note::
   Matlab has already been installed in the VM copy you have but it is not activated. Please follow the instructions on the ~/Desktop to activate Matlab using your own account.
 
We have seen that ROS is a very powefull tool which allows us to use many predefined packages including various robots' packages, various algorithms, sensor drivers etc. On the other hand, we have another powerful in our hands for which mostly we use signal processing, modelling, in control process. It would be cool if we used them together. 

Recently Matlab included **ROS** under its Robotics Toolbox which enables the ROS environment accessible via Matlab. Now, we are going to set our Matlab to control turtlesim which we started from our ROS.

Beneficial tutorials:

#. `ROS Toolbox <https://se.mathworks.com/help/ros/index.html?s_tid=CRUX_lftnav>`_

As we have :code:`roscore` on ROS, there is an equivalent on Matlab as :code:`rosinit` to start the ROS master in the network. There is a slight difference of starting the ROS master between using :code:`roscore` and :code:`rosinit`. If you type :code:`rosnode list`, you will see a global node which was started by Matlab but you won't see :code:`\rosout` (as we are used to see). In usage, there is not a significant difference right now since we are not going to connect a real robot. Just keep in mind that if you need to include more modules/robots/PC in the same ROS network, they should be connected to the same ROS master. If you want to initialize your ROS master on a specific address you should specify it :code:`rosinit('192.168.17.157')`.

Since we successfully ran the ros master, we can create the first publisher. There are two ways of doing it:

#. Via Simulink
#. Via Matlab Script

Via Simulink
~~~~~~~~~~~~~~
This is the most intuitive and the easiest way of creating a ROS publisher using Matlab. Using Simulink, please create the following model.

.. figure:: ../_static/images/ros_simulink.png
        :align: center

.. note::
   I will show you the communication between two environments (Ubuntu and Matlab). For those who use ROSDS, this activity won't work because ROSDS works on a cloud PC which is not connected the same network as our personal computer. 

   Since ROSDS users do not have any preinstalled ROS distribution, they cannot run :code:`turtlesim` nodes. On Matlab, there is a similar tutorial simulation. I will show this part after I am done with the *turtlesim* tutorial.

**On turtlesim:**

* So, let's start the tf example of two turtles: :code:`roslaunch turtle_tf turtle_tf_demo.launch`.
* Check the available ROS topics: :code:`rostopic list`.
* The topic in which we would like to publish is: :code:`/turtle1/cmd_vel`.

**On ExampleHelperSimulinkRobotROS:**

* After initializing the ROS master, you need to type :code:`sim = ExampleHelperSimulinkRobotROS;` on the Matlab command window.
* Check the available ROS topics: :code:`rostopic list`.
* The topic in which we would like to publish is: :code:`/mobile_base/commands/velocity`.
* Set the simulink publisher link to :code:`/mobile_base/commands/velocity`.

Via Matlab Script
~~~~~~~~~~~~~~~~~~
.. note::
   You can write this code directly on the Command Window also.

**On turtlesim:**
Copy this piece of code into a new script which you created under the work directory of Matlab.

 .. literalinclude:: ../_static/scripts/turtlesim_control_script.m
       :language: Python

**On ExampleHelperSimulinkRobotROS:**

* Chance the :code:`/turtle1/cmd_vel` into :code:`/mobile_base/commands/velocity`.


.. _AR-tags:

AR tags
============
.. note::
   This section is covered only for DAT160 students.

AR tags (aka Artifitial Reality tags) are QR-code like black and white images which can be created with a unique ID number and used as an identification of a place/object/state in the robot world. In this section, we are not interested in the image process of an AR tag to visualize its representative image but we are interested in creating them, detecting in an environment, identifying and taking an action based on their ID.

.. figure:: ../_static/images/ros/artags.png
          :align: center

          Example AR tags

There is a specigic package called ``ar-track-alvar`` available in ROS enables to create and read AR tags. This package is a part of **ros-perception** and it is not installed in default with ``ros_DISTRO-desktop_full`` installation. However, the VM copy which we provided has this package additionally installed. If you don't have the package, please run the following commang to install it:

``sudo apt install ros-melodic-ar-track-alvar``

The official documentation of the `ar_track_alvar package is here <http://wiki.ros.org/ar_track_alvar>`_.

To generate new AR tags run the ROS node: ``rosrun ar_track_alvar createMarker``, give a prefered ID number and leave the dimension/position as default by pressing only **Enter** without any value entered.

.. code::

   rosrun ar_track_alvar createMarker 4 -s 5   ## Creates Marker4.png 5cm*5cm

To use the created AR tag in real world, you should simply print them out in original size. We are going to use them in the Gazebo environment. Therefore, we have used them as surface meshes on a custom Gazebo model.

Let's check if you have the necessary models in place:

.. code::

   cd ~/.gazebo/models
   ls marker*

   ## OUTPUT should be:
   marker0:
   materials  meshes  model-1_4.sdf  model-1_5.sdf  model.config  model.sdf

   marker1:
   materials  meshes  model-1_4.sdf  model-1_5.sdf  model.config  model.sdf

   marker2:
   materials  meshes  model-1_4.sdf  model-1_5.sdf  model.config  model.sdf

   marker3:
   materials  meshes  model-1_4.sdf  model-1_5.sdf  model.config  model.sdf

   marker4:
   materials  meshes  model-1_4.sdf  model-1_5.sdf  model.config  model.sdf


If you cannot list any markers plese download the **models** folder from follow the link to `download the AR tag models <https://hvl365.sharepoint.com/:f:/r/sites/RobotikkUndervisningHVL/Delte%20dokumenter/ROSTeaching/download_materials/markers?csf=1&web=1&e=JnHhtz>`_.


We are ready to spawn a Turtlebot in a custom world filled with 5 different AR tags.

.. code::

   roslaunch ar_tutorials tb_ar_world.launch
   roslaunch ar_tutorials ar_track.launch
   rosrun rqt_image_view rqt_image_view
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   rostopic echo /ar_pose_marker

Please analyse the topic and try to capture id of a visible marker through terminal.


Subscriber node to ``/ar_pose_marker``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Now we can have a template subscriber to get AR tag pose data and take an action according to that.

.. code::
   Python

   #!/usr/bin/env python

   import rospy
   from ar_track_alvar_msgs.msg import AlvarMarkers

   detected_markers=set()
   markers_in_sight=['a']


   def callback(msg):
      global detected_markers, markers_in_sight
      # markers_in_sight.clear() doesn't work. This is a Python 3.3+ command
      markers_in_sight = []
      n = len(msg.markers)
      for i in range(n):
         detected_markers.add(msg.markers[i].id)
         markers_in_sight.append(msg.markers[i].id)
      # print("detected_markers"+str(detected_markers))
      # print(markers_in_sight)


   def ar_action(markers):
      human_id = 0
      if human_id in markers:
         print("Human found!")


   def main():
      global markers_in_sight
      rospy.init_node('ar_pose_subscriber', anonymous=False)
      rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback)

      rate = rospy.Rate(10)  # 10hz
      while not rospy.is_shutdown():
         ar_action(markers_in_sight)
         rate.sleep()


   if __name__ == '__main__':
      main()  



.. _Navigation-From-Scratch:

Navigation From Scratch
=========================

.. note::
   This section is covered only for DAT160 students.


.. _Multiple-Robot-Control-in-Gazebo:

Multiple Robot Control in Gazebo
==================================

.. note::
   This section is covered only for DAT160 students.



.. _MoveIt:


MoveIt
========

.. note::
   This section is covered only for ELE306 students.


MoveIt Only Visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
asdasd


Control a Real Robotic Arm with MoveIt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
asdasd
