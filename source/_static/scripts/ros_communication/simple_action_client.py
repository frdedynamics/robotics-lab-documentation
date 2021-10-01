#! /usr/bin/env python

import rospy
import random
import actionlib
from communication_tutorial.msg import SumAction, SumGoal, SumResult, SumFeedback

class simple_action_client():
    def __init__(self):
        rospy.init_node("simple_action_client", anonymous=False)
        self.client = actionlib.SimpleActionClient("random_sum_action", SumAction)
        self.client.wait_for_server()

    def done_cb(self, state, result):
        rospy.loginfo("Final number "+str(result.final_number)+" reached after "+str(result.final_iterations)+" iterations")

    def active_cb(self):
        rospy.loginfo("Action server transitioned into ACTIVE")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current sum "+str(feedback.current_sum)+" reached after "+str(feedback.current_iterations)+" iterations")
        if feedback.current_iterations > 10:
            self.client.cancel_goal()
            rospy.loginfo("Server is taking too long. Canceling the goal")

    def main(self):
        numbers = [1,2,3,4,5,6,7,8,9]
        rnd = random.choice(numbers)

        goal = SumGoal()
        goal.random_start = rnd
        goal.goal_number = 100

        rospy.loginfo("Random start number: "+str(goal.random_start))
        rospy.loginfo("Goal number: "+str(goal.goal_number))

        self.client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)

        self.client.wait_for_result()

        #result = self.client.get_result()







if __name__=="__main__":
    sac = simple_action_client()
    sac.main()
