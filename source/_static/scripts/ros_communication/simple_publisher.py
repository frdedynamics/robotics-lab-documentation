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
