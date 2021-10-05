#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class move_base_example():
    def __init__(self):
        rospy.init_node('move_base_example', anonymous=False)
        #Create a client for the move_base action server
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

    def main(self):
        #Define the target position and orientation
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = 0.0
        self.goal.target_pose.pose.position.y = 8.0
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = 0.0
        self.goal.target_pose.pose.orientation.y = 0.0
        self.goal.target_pose.pose.orientation.z = 0.707
        self.goal.target_pose.pose.orientation.w = 0.707

        #Send the defined target to the server
        self.client.send_goal(self.goal)
        #Wait for the server to finish the navigation
        self.client.wait_for_result()
