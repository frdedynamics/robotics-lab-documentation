#! /usr/bin/env python

import rospy
import random
from std_msgs.msg import Int64
from communication_tutorial.msg import rand_num

class simple_subscriber():
    def __init__(self):
        rospy.init_node("simple_subscriber", anonymous=False)

        rospy.Subscriber("random_number", rand_num, self.random_number_callback)

    def random_number_callback(self, data):
        self.number = data
        rospy.loginfo("Received random_number: "+str(self.number.random_number))
        rospy.loginfo("Received object: "+str(self.number.object))

    def main(self):
        rospy.spin()


if __name__=="__main__":
    spub = simple_subscriber()
    spub.main()
