#! /usr/bin/env python

import rospy
import random
from std_srvs.srv import SetBool, SetBoolResponse
from communication_tutorial.srv import random_sum, random_sumResponse

class simple_server():
    def __init__(self):
        rospy.init_node("simple_server", anonymous=False)

        self.srv = rospy.Service("random_number_service", random_sum, self.random_number_service_callback)

    def random_number_service_callback(self, req):
        self.req_number = req.random_init
        rospy.loginfo("Client request: "+str(self.req_number))
        res = random_sumResponse()
        numbers = [1,2,3,4,5,6]
        rnd = random.choice(numbers)
        msg = self.req_number + rnd
        rospy.loginfo("Number chosen: "+str(rnd))
        rospy.loginfo("Random sum: "+str(msg))
        res.success = True
        res.random_sum = msg


        return res

    def main(self):
        rospy.spin()



if __name__=="__main__":
    service_server = simple_server()
    service_server.main()
