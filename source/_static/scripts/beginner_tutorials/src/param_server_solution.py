#!/usr/bin/env python

import rospy


rospy.init_node('myfirstnode')

rate = rospy.Rate(10)

if rospy.has_param("/background_b"):
    print("Parameter found!")
    color_b = rospy.get_param("/background_b")
    print(color_b)

rospy.set_param("/background_b", 111)

color_b = rospy.get_param("/background_b")
print(color_b)

while not rospy.is_shutdown():
    rospy.set_param("/background_b", 55)
    rate.sleep()
    rospy.set_param("/background_b", 155)
    rate.sleep()
    rospy.set_param("/background_b", 255)
    rate.sleep()
