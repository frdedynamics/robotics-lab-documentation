#! /usr/bin/env python

import rospy

class simple_param():
    def __init__(self):
        rospy.init_node("simple_param", anonymous=False)

        self.init_x = rospy.get_param("starting_pos_x")
        self.init_y = rospy.get_param("starting_pos_y")

        rospy.loginfo("initial x position: "+str(self.init_x))
        rospy.loginfo("initial y position: "+str(self.init_y))

        rospy.set_param("starting_pos_x", 18.2)
        self.init_x = rospy.get_param("starting_pos_x")
        rospy.loginfo("initial x position: "+str(self.init_x))




if __name__=="__main__":
    sp = simple_param()
