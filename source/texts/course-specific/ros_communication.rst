.. _ros_communication:

******************************************
ROS Communication Patterns
******************************************

ROS provides different patterns that can be used to communcate between ROS Nodes:

  * `Topics <https://wiki.ros.org/Topics>`_: are used to send continuous data streams like e.g. sensor data. Data can be published on the topic independent of if there are any subscribers listening. Similarly, Nodes can also subscribe to topics independent of if a publisher exists. Topics allow for many to many connections, meaning that multiple nodes can publish and/or subscribe to the same topic.
  * `Services <https://wiki.ros.org/Services>`_: are used when it is important that a message is recieved by the recipient and a response on the outcome is wanted. Services should only be used for short procedure calls e. g. changing the state of a system, inverse kinematics calculations or triggering a process.
  * `Actions <https://wiki.ros.org/actionlib>`_: are similar as services but are used if the triggered event needs more time and the possebility for recieving updates on the process or being able to cancel the process is wanted. An example could be when sending a navigation command.
  * `Parameters <https://wiki.ros.org/Parameter%20Server>`_: are not actually a communication pattern but rather is a storage space for variables. It is not designed for high-performance and therefore mostly used for static variables like configuration parameters.

In the Media Gallery of your course page in Canvas are videos, explaining how to use these communication patterns in python. The following sections contain the example code which is explained in the videos. 

Topics
==============
The video called **Simple Topic Implementation** is about how to write a simple publisher and subsriber in python. Additional information can also be found on the ROS webpage: `creating a ROS package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>`_, `publisher and subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_. The following code is an example implementation created and explained in the video:

.. literalinclude:: ../../_static/scripts/ros_communication/simple_publisher.py
       :language: python
       :caption: Simple Publisher in Python
       
.. literalinclude:: ../../_static/scripts/ros_communication/simple_subscriber.py
       :language: python
       :caption: Simple Subscriber in Python       

Custom Message
----------------
The video called **Create Custom Messages** is about how to create a custom message type which can be used by ROS Topics. Additional information can also be found on the ROS webpage: `creating a message and service <http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv>`_. The following is an example of a message type definition which was created in the tutorial video:

.. literalinclude:: ../../_static/scripts/ros_communication/custom_msg.msg
       :language: C
       :caption: Custom Message Type: rand_num.msg

Services
==============
The video called **Simple Service Implementation** is about how to write a simple service server and client in python. Additional information can also be found on the ROS webpage: `service server and client <http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29>`_. The following code is an example implementation created and explained in the video:

.. literalinclude:: ../../_static/scripts/ros_communication/simple_server.py
       :language: python
       :caption: Simple Service Server in Python
       
.. literalinclude:: ../../_static/scripts/ros_communication/simple_client.py
       :language: python
       :caption: Simple Service Client in Python

Custom Service
---------------
The video calle **Creating Custom Services** explains how to create a custom service message type usable by ROS Services. Additional information can also be found on the ROS webpage: `creating a message and service <http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv>`_. The following is an example of a service message type definition which was created in the tutorial video:

.. literalinclude:: ../../_static/scripts/ros_communication/random_sum.srv
       :language: C
       :caption: Custom Service Message Type: random_sum.srv

Actions
==============
The video called **Simple Action Implementation** explains how to create a custom action message type usable by ROS Actions. It also shows how to write a simple action server and client in python. Additional information can also be found on the ROS webpage: `action server <http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29>`_, `action client <http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29>`_, creating an action message <http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29>. The following code is an example implementation created and explained in the video:

.. literalinclude:: ../../_static/scripts/ros_communication/simple_action_server.py
       :language: python
       :caption: Simple Action Server in Python
       
.. literalinclude:: ../../_static/scripts/ros_communication/simple_action_client.py
       :language: python
       :caption: Simple Action Client in Python

Parameters
==============
The video called **Working with Parameters** shows how to write parameters to the parameterserver and launch a python script from a launch file as well as read and write parameters in a python script. Additional information can also be found on the ROS webpage: `writing parameters from launch file <http://wiki.ros.org/roslaunch/XML/param>`_, `using parameters in python <http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters>`_. The following code is an example implementation of a launch file and python script working with parameters:

.. literalinclude:: ../../_static/scripts/ros_communication/param_test.launch
       :language: C
       :caption: Writing Parameters from Launch Files
       
.. literalinclude:: ../../_static/scripts/ros_communication/simple_param.py
       :language: python
       :caption: Working with Parameters in Python
