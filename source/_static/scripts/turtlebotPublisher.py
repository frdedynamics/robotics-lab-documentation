#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def GoForward():
    # initiliaze
    rospy.init_node('GoForwardNode', anonymous=False)

    # tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    # Create a publisher which can "talk" to TurtleBot and tell it to move
    pub = rospy.Publisher('TOPICNAME', Twist, queue_size=10)

    #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
    r = rospy.Rate(10);

    # Twist is a datatype for velocity
    move_cmd = Twist()
    # let's go forward at 0.2 m/s
    move_cmd.linear.x = 0.2
    # let's turn at 0 radians/s
    move_cmd.angular.z = 0

    # as long as you haven't ctrl + c keeping doing...
    while not rospy.is_shutdown():
        # publish the velocity
        pub.publish(move_cmd)
        # wait for 0.1 seconds (10 HZ) and publish again
        r.sleep()

def GoUpward():
    # Fill yourself

def GoCircularClockwise():
    # Fill yourself


if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")
