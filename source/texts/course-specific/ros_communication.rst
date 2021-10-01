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
The video called Simple Topic Implementation is about how to write a simple publisher and subsriber in python. Extra information can also be found on the ROS webpage (`creating a ROS package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>`_, `publisher and subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29>`_). The following code is explained in the video:

::
    #! /usr/bin/env python

    import rospy
    import random
    from std_msgs.msg import Int64
    from communication_tutorial.msg import rand_num

    class simple_publisher():
        def __init__(self):
            rospy.init_node("simple_publisher", anonymous=False)

            self.topic_pub = rospy.Publisher("random_number", rand_num, queue_size=10)

        def main(self):
            numbers = [1,2,3,4,5,6]
            objects = ["Apples","Bananas","Oranges"]

            r = rospy.Rate(1)
            while not rospy.is_shutdown():
                msg = rand_num()
                msg.random_number = random.choice(numbers)
                msg.object = random.choice(objects)
                self.topic_pub.publish(msg)
                rospy.loginfo("Published data: "+str(msg))

                r.sleep()


    if __name__=="__main__":
        spub = simple_publisher()
        spub.main()

Custom Message
----------------
The video called 

Services
==============
Video on how to make a simple Service server + client combo

Custom Service
---------------
Video on creating a custom service request/response message

Actions
==============
Video on how to make a simple action server + client combo


Parameters
==============
Video on how to work with parameters in python and launch files
