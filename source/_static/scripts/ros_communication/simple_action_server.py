#! /usr/bin/env python

import rospy
import random
import actionlib
from communication_tutorial.msg import SumAction, SumGoal, SumResult, SumFeedback

class simple_action_server():
    def __init__(self):
        rospy.init_node("simple_action_server", anonymous=False)

        self.aserver = actionlib.SimpleActionServer("random_sum_action", SumAction, execute_cb=self.execute_cb, auto_start = False)
        self.aserver.start()

        self.result = SumResult()
        self.feedback = SumFeedback()

    def execute_cb(self, goal):
        self.random_init = goal.random_start
        self.threshold = goal.goal_number

        rospy.loginfo("Goal received: "+str(self.threshold))
        rospy.loginfo("Starting number: "+str(self.random_init))

        numbers = [1,2,3,4,5,6,7,8,9]
        current_value = self.random_init
        counter = 0

        r = rospy.Rate(1)

        while current_value < self.threshold:
            if self.aserver.is_preempt_requested():
                self.aserver.set_preempted()
                rospy.loginfo("Client requested cancelling of the goal")
                break
            new_number = random.choice(numbers)
            current_value += new_number
            counter += 1

            self.feedback.current_sum = current_value
            self.feedback.current_iterations = counter
            self.aserver.publish_feedback(self.feedback)

            r.sleep()

        if not self.aserver.is_preempt_requested():
            self.result.final_number = current_value
            self.result.final_iterations = counter
            self.aserver.set_succeeded(self.result)

    def main(self):
        rospy.spin()



if __name__=="__main__":
    sas = simple_action_server()
    sas.main()
