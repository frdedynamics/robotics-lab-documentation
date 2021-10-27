.. _ros_custom_robot:

******************************************
ROS Build a Custom Robot
******************************************

In this Chapter we will look at how to build a gazebo model of a robot given the mechanical design. The following topics will be covered:
 
 * Building up the model in Gazebo
 * Adding a ROS control interface to the model
 
Building a Gazebo Model
=========================
A model in Gazebo is made of links which are connected through joints. Since this is a ROS lecture, we use URDF files to declare the model. There are other ways of doing it, which will not be covered. **In general it is important to notice that all distances are generally in** `meters <https://en.wikipedia.org/wiki/Metre>`_ **and all angles in** `radian <https://en.wikipedia.org/wiki/Radian>`_.


Link definition
---------------------
The official documentation of how to define a link in urdf can be found `here <http://wiki.ros.org/urdf/XML/link>`_.

Box
~~~~~~~~
.. literalinclude:: ../../_static/scripts/build_custom_robot/link_box.urdf.xacro
       :language: XML
       :caption: xacro link for box object

Cylinder
~~~~~~~~~~
.. literalinclude:: ../../_static/scripts/build_custom_robot/link_cylinder.urdf.xacro
       :language: XML
       :caption: xacro link for cylinder object
       
Sphere
~~~~~~~~~~
.. literalinclude:: ../../_static/scripts/build_custom_robot/link_sphere.urdf.xacro
       :language: XML
       :caption: xacro link for sphere object

Mesh
~~~~~~~~~~
.. literalinclude:: ../../_static/scripts/build_custom_robot/mesh_geometry.urdf.xacro
       :language: XML
       :caption: using mesh files for link geometry

Inertial Parameters
---------------------
An accurate simulation requires accurate dynamic properties of the links. These are defined by the inertial parameters. An official guide on the Gazebo website can be found `here <http://gazebosim.org/tutorials?tut=inertia>`_. An online tool (mesh cleaner) to calculate the inertial parameters for mesh files automatically can be found `here <https://www.hamzamerzic.info/mesh_cleaner/>`_. When using simple shapes the previous example code for the different links automatically calculates the inertial parameters assuming you use the defined variables. A list of formulas to calculate the inertial parameters for simple shapes can be found `here <https://en.wikipedia.org/wiki/List_of_moments_of_inertia>`_.


Joint definition
---------------------
The official documentation of how to define a joint in urdf can be found `here <http://wiki.ros.org/urdf/XML/joint>`_. Some important parts of the joint definition are the following:

 * **<axis>:** defines which axis is used by the joint for movement.
 * **<limit>**
   
      * **effort** maximum torque/force measured in [Nm]
      * **velocity** maximum speed measured in [m/s] for primatic joints and [rad/s] for revolute joints
      * **lower** minimum allowed joint angle/position measured in [m] for prismatic joints and [rad] for revolute joints.
      * **upper** maximum allowed joint angle/position
   

Continuous Joint
~~~~~~~~~~~~~~~~~~
Continuous joints rotate around the defined axis (1 degree of freedom).

.. literalinclude:: ../../_static/scripts/build_custom_robot/joint_continuous.urdf.xacro
       :language: XML
       :caption: defining a continuous joint in xacro
       
Revolute Joint
~~~~~~~~~~~~~~~~~~
Revolute joints also rotate around the defined axis (1 degree of freedom) similar to continuous joints but have a defined minimum and maximum joint angle in [rad].

.. literalinclude:: ../../_static/scripts/build_custom_robot/joint_revolute.urdf.xacro
       :language: XML
       :caption: defining a revolute joint in xacro 
       
Prismatic Joint
~~~~~~~~~~~~~~~~~~
Prismatic joints move along the defined axis (1 degree of freedom) and have a minimum and maximum joint position definedin [m].

.. literalinclude:: ../../_static/scripts/build_custom_robot/joint_prismatic.urdf.xacro
       :language: XML
       :caption: defining a prismatic joint in xacro    
       
Fixed Joint
~~~~~~~~~~~~~~~~~~
Fixed joints are not really joints because all degrees of freedom are blocked.

.. literalinclude:: ../../_static/scripts/build_custom_robot/joint_fixed.urdf.xacro
       :language: XML
       :caption: defining a fixed joint in xacro       

Exercise
---------------------
Download the ROS package used for the exercise from this `link <https://hvl365.sharepoint.com/:u:/s/RobotikkUndervisningHVL/Ea8kb_mYnChEgkMMtsu2ADsBUF5L2bPqn_ZkFg1B4gEjYQ?e=P1R1tu>`_. After unzipping it, copy it to the VM in catkin_ws/src/. Run the following commands in a terminal window:

::

 cd ~/catking_ws/
 catkin_make

Copy the following code into the file "mobile_manipulator_robot.urdf.xacro" located in the "urdf" folder of the ROS package:

.. literalinclude:: ../../_static/scripts/build_custom_robot/mobile_manipulator_robot_init.urdf.xacro
       :language: XML
       :caption: mobile_manipulator_robot.urdf.xarco located in the urdf file 


To start the Gazebo world use the following command in the terminal:

::

 roslaunch custom_robot_tutorial mobile_manipulator.launch
 
To spawn the robot in the existing Gazebo world use the following terminal command:

::

 roslaunch custom_robot_tutorial robot_spawn.launch
 
Start adding the missing 3 wheels to the robot model by modifying the "mobile_manipulator_robot.urdf.xacro" file. Where to place the wheels can be deducted from the following mechanical drawings:

.. figure:: ../../_static/scripts/build_custom_robot/mobile_robot_plan_2.png
          :align: center   

.. figure:: ../../_static/scripts/build_custom_robot/robot_arm_plan_2.png
          :align: center
          
Once you finished with adding all the wheels to the mobile platform, copy the following code into the "mobile_manipulator_robot.urdf.xacro" file just **before** the </robot> tag at the end of the file:

.. literalinclude:: ../../_static/scripts/build_custom_robot/mobile_manipulator_robot_arm_init.urdf.xacro
       :language: XML
       :caption: mobile_manipulator_robot.urdf.xarco located in the urdf file 
  
  
