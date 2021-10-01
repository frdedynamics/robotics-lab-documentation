#! /usr/bin/env python

import rospy
import random
from std_srvs.srv import SetBool, SetBoolResponse
from communication_tutorial.srv import random_sum

class simple_client():
    def __init__(self):
        rospy.init_node("simple_client", anonymous=False)

        self.service_client = rospy.ServiceProxy("random_number_service", random_sum)
        rospy.wait_for_service("random_number_service")

    def main(self):
        numbers = [1,2,3,4,5,6]

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg = random.choice(numbers)
            rospy.loginfo("Message sent: "+str(msg))
            res = self.service_client(msg)
            rospy.loginfo("Response received: "+str(res))

            r.sleep()



if __name__=="__main__":
    service_server = simple_client()
    service_server.main()
