*****************
Additionals
*****************

We can add whatever extra is needed here.

* `AR tags <http://wiki.ros.org/ar_track_alvar>`_
* `OpenCV and ROS <https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html>`_, `OpenCV and ROS 2 <http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython>`_
* `2 Turtlebots SLAM <https://www.youtube.com/watch?v=ndvwDFi-I3I>`_
* `Multiple Robot Manipulation in Gazebo <https://www.youtube.com/watch?v=es_rQmlgndQ>`_
* Open Manipulator with Moveit
* UR5 with Moveit

AR tags
=========
AR tags are QR-code like black and white images which can be created with a unique ID number and used as an identification of a place/object/state in the robot world.

.. figure:: ../_static/images/ros/artags.png
          :align: center

          Example AR tags

There is a specigic package called ``ar-track-alvar`` available in ROS enables to create and read AR tags. This package is not installed in default. Please run the following commang to install it:

``sudo apt install ros-melodic-ar-track-alvar``

To generate new AR tags run the ROS node: ``rosrun ar_track_alvar createMarker``, give a prefered ID number and leave the dimension/position as default by pressing only **Enter** without any value entered.