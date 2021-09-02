#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

joints = []
corrected_joints = JointState()


def callback(msg):
    global joints
    joints = msg.position


def listener():
    global joints

    rospy.Subscriber("joint_states", JointState, callback)
    pub = rospy.Publisher("/joint_states_corrected", JointState, queue_size=1)
    rospy.init_node('joints_corrector', anonymous=False)
    # spin() simply keeps python from exiting until this node is stopped

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        print(joints)
        corrected_joints.position = joints
        pub.publish(corrected_joints)
        rate.sleep()


if __name__ == '__main__':
    listener()
