****************************
Robot Operating System
****************************

So, if you like we can focus on Turtlebot simulation. Something like:

#. What is ROS? Rosmaster, node, topic, message, subscriber, publisher, etc.
#. How to use these terms? The very first turtlebot tutorial, teleoperation with keyboard on the simulation, echo some topics, change the background...
#. But the simulation doesn't have any physical properties, we use Gazebo for that - Show them how to implement turtlebot. Or to provide a simple 2 wheel robot URDF as in the tutorial which I shared formerly so that they know what is URDF and how robots are modeled in a very simple way.
#. Even though you can code everything in Python/C++ as ROS node, sometimes you don't want to deal with every single details of everything and would like to use some other powerful tools like MATLAB. - Here is how you control your ROS side turtlebot with a publisher from MATLAB.


Installation
=============
.. note::
   Only the first option is recommended but for those who don't want to install Ubuntu can use other alternatives.

* For Ubuntu users please follow `the official guide <http://wiki.ros.org/ROS/Installation>`_
* For Windows users please follow the link to install `WSL <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_, `Visual Studio Code <https://code.visualstudio.com/download>`_ and for simplicity you can install `Visual Studio Code Remote - WSL Extension <https://code.visualstudio.com/docs/remote/wsl>`_.
* For browser-based ROS environment `ROSDS <https://auth.theconstructsim.com/auth/login/?next=/oidc/authorize/%3Fscope%3Dopenid%2Bprofile%2Bemail%26state%3D0U3HkB5WbPlLVhwsR10eFHXeWlbgStpy%26redirect_uri%3Dhttps%253A%252F%252Frds.theconstructsim.com%252Foidc%252Fcallback%252F%26response_type%3Dcode%26client_id%3D983516>`_.





Hands On Activities
====================
We will first start running the built-in packages, execute nodes, publish and receive some topics. For these, we will follow the official ROS tutorials from the Wiki page.

#. `Understanding ROS Nodes <http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes>`_
#. `Understanding ROS Topics <http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics>`_
#. `Understanding ROS Services and Parameters <http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams>`_

Create First Publisher/Subscriber
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
We have learned the concepts of a publisher and a subscriber. In fact, one single node can be both publisher and subscriber at the same time, and this is more common than you may imagine. 

For now, we will seperate our publisher and subscriber in different scripts (nodes). Let's first create a *my_first_package* and place our nodes in it.

* To create a ROS package, we will follow the official `Creating a ROS Package <Creating a ROS Package>`_ tutorial (until section 5).

Now we are ready to create our first publisher of our first package. We will use Python language in today's lecture but it is possible to write your scripts in C++ also without any further hassle.

* `Writing a Simple Publisher and Subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_

We are ready to play around now. Let's control the turtlesim with our own publisher. Here is the code for you. Don't celebrate it immediately for that you have the source code, there are some empty lines for you!

 .. literalinclude:: ../_static/scripts/turtlebotPublisher.py
       :language: Python

**Hint:** To find out which topic name you should publish your message, use :code:`rostopic list`.


.. seealso::
   Can you make your turtlesim to draw a square by using your own publisher?


Simulating Robots in ROS
==========================
In the ROS environment, there are 2 main simulation tools. `Rviz <http://wiki.ros.org/rviz>`_ and `Gazebo <http://gazebosim.org/tutorials?tut=ros_overview>`_. Even though they both are simulation tools, they diverge when it comes to usage. Rviz is more *visualization* purposes such that map creation, transfer function visualization, point cloud, image visualization etc. Gazebo, on the other hand, is used for physical testing. Despite the fact that it is possible to test sensor readings on Rviz as well, it is not possible to test collisions in Rviz; you need Gazebo (or another physics engine embedded simulation tools.

.. figure:: ../_static/images/mybot.png
          :align: center

          Source: moorerobots.com

.. note::
   Gazebo is a standalone simulation tool with its own physics engine. It does not require ROS to run.


Rviz Introduction
~~~~~~~~~~~~~~~~~~
The `original tutorial <http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf>`_ is linked. Let's modify this to understand Rviz elements a bit more. First start the node with two turtles with the following command: :code:`roslaunch turtle_tf turtle_tf_demo.launch`.

Start Rviz: :code:`rosrun rviz rviz`.

As you see, there is no element on the left toolbar. First add the :code:`/world` frame. Also add the TF element.

Click on the terminal again, use your arrow keys to move your turtle and observe the TF poses on Rviz.


Gazebo Introduction
~~~~~~~~~~~~~~~~~~~~
Gazebo is the most used simulator in ROS. It has integrated physics rules like gravity, colision, light distribution with reflection and refraction properties, mass and inertia etc. Now, we will see how to set the Gazebo for a very basic usage.

.. warning::
   Before moving on, I would like you all to have the same folder organization as shown in the figure. This is important for you to follow the rest of the tutorial easily. Although such an organization is not necessarry for ROS system particularly, this is the accepted/official way of organizing ROS packages.

   .. figure:: ../_static/images/folders.png
          :align: center


First, let's create an empty world under the /world folder.

.. literalinclude:: ../_static/scripts/empty_world.world
      :language: xml
      :caption: empty_world.world

Secondly, create a launch file under /launch folder to run Gazebo and import our empty world.

.. literalinclude:: ../_static/scripts/my_world_gazebo.launch
      :language: xml
      :caption: my_world_gazebo.launch

We can add some items by just adding items as we included the *sun* and the *ground plane* into our world file. There are many ready-to-use models in the official `gazebo-models repository <https://github.com/osrf/gazebo_models>`_. Now, choose a model and as in the following code.

.. code-block:: xml

   <!-- Add an object -->
	<include>
	  <uri>model://postbox</uri>
	</include>

Also you can change the position and orientation of any object which you have added with the :code:`<pose>x y z roll pitch yaw</pose>` tag.

URDF
-----
Unified Robot Description Format. It is used to describe the robot model. It contains links, joints and basic material information of each part of the robot. You can follow the `list of parts <http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch>`_ is the R2D2 model written in URDF!

All the elements that you can use in an URDF file are `these. <http://wiki.ros.org/urdf/XML/robot>`_ A simple URDF file looks like this. Let's first understand it line by line.

 .. literalinclude:: ../_static/scripts/simplexacro.xacro
       :language: xml

URDF in RViz
-------------

 .. literalinclude:: ../_static/scripts/my_robot.xacro
       :language: xml
       :caption: beginners_tutorials/urdf/my_robot.xacro


 .. literalinclude:: ../_static/scripts/my_robot_rviz.launch
       :language: xml
       :caption: beginners_tutorials/launch/my_robot_rviz.launch


URDF in Gazebo
----------------

URDF files are able to be used in Gazebo by including mass, inertia, joint efforts, joint limits etc properties of the links and joint. In fact, Gazebo simulator does not use the URDF files directly. The file format in which Gazebo uses is SDF (Simulation Description Format). The main reason is that the URDF files are lacking many features and have not been updated to deal with the evolving needs of robotics. URDF can only specify the kinematic and dynamic properties of a single robot in isolation. URDF can not specify the pose of the robot itself within a world. It is also not a universal description format since it cannot specify joint loops (parallel linkages), and it lacks friction and other properties. Additionally, it cannot specify things that are not robots, such as lights, heightmaps, etc.

However, at this moment, there is no need to frustrate with a new file format than URDF yet it is nice to know this fact. For more detailed information, you can `read this <http://gazebosim.org/tutorials?tut=ros_urdf>`_.

Xacro: beginners_tutorials/urdf/mybot/mybot.xacro :code:`git clone -b base https://github.com/richardw05/mybot_ws.git`

 .. literalinclude:: ../_static/scripts/my_robot_gazebo.launch
       :language: xml
       :caption: beginners_tutorials/launch/my_robot_gazebo.launch

This tutorial is based on the `Moore Robots <http://www.moorerobots.com/blog>`_ tutorials. They provide a series of robot modelling/simulation/navigation tutorial. It can be a bit confusing for the beginners but I believe that they are very useful for those who would like to use ROS in their term projects. Here is the `youtube tutorial of Moore robots <https://www.youtube.com/watch?v=8ckSl4MbZLg&list=PLTEmcYHVE7dPWixFnzkd68jPjwGzxHjUG>`_.



Adding Sensors to the Gazebo Model (part 2)
---------------------------------------------
Adding scanner and some obstacles on our 2 wheel robot. It is going to look like this:

.. figure:: ../_static/images/mybotSensor.png
        :align: center

This part is going to be a homework. The tutorial is `in this link <https://www.youtube.com/watch?v=EZ3MYf24c6Y&list=PLTEmcYHVE7dPWixFnzkd68jPjwGzxHjUG&index=2&ab_channel=RichardWang>`_.

ROS with Matlab
=================
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









