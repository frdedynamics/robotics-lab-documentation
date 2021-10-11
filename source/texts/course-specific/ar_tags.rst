.. _AR-tags:

AR tags
============
.. note::
   This section is covered only for DAT160 students.

AR tags (aka Artifitial Reality tags) are QR-code like black and white images which can be created with a unique ID number and used as an identification of a place/object/state in the robot world. In this section, we are not interested in the image process of an AR tag to visualize its representative image but we are interested in creating them, detecting in an environment, identifying and taking an action based on their ID.

.. figure:: ../../_static/images/ros/artags.png
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

.. literalinclude:: ../../_static/scripts/ar_tutorials/launch/tb_ar_world.launch
      :language: xml
      :caption: tb_ar_world.launch


.. literalinclude:: ../../_static/scripts/ar_tutorials/worlds/custom.world
      :language: xml
      :caption: custom.world


.. literalinclude:: ../../_static/scripts/ar_tutorials/launch/ar_track.launch
      :language: xml
      :caption: tb_ar_world.launch


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
