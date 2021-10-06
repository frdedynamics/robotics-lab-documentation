#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from tf import transformations
import math

class bug2():
    def __init__(self):
        
        self.yaw_ = 0
        self.yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
        
        self.position_ = Point()
        
        self.desired_position_ = Point()
        self.desired_position_.x = 0
        self.desired_position_.y = 0
        self.desired_position_.z = 0

        self.regions_ = None
        self.state_desc_ = ['Go to point', 'wall following', 'Goal reached']
        self.state_ = 0
        self.count_state_time_ = 0 # seconds the robot is in a state
        self.count_loop_ = 0
        # 0 - go to point
        # 1 - wall following
        
        #INITIALIZE PUBLISHERS, SUBSCRIBERS, SERVICES ETC. HERE

        
        
    #callback function for odometry subscriber
    def clbk_odom(self, msg):
        # global position_, yaw_

        # position
        self.position_ = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]
        
    #callback function for laser subscriber
    def clbk_laser(self, msg):
        # global regions_
        self.regions_ = {
            'right':  min(min(msg.ranges[180:299]), 1.0),
            'fright': min(min(msg.ranges[300:339]), 1.0),
            'front':  min(min(min(msg.ranges[0:19]), min(msg.ranges[340:359])),1.0),
            'fleft':  min(min(msg.ranges[20:59]), 1.0),
            'left':   min(min(msg.ranges[60:179]), 1.0),
        }

    def change_state(self, state):
        self.count_state_time_ = 0
        self.state_ = state
        log = "state changed: %s" % self.state_desc_[state]
        rospy.loginfo(log)
        
        # ADD CODE HERE

    #calculating the distance to the line dependend on the current position of the robot
    def distance_to_line(self, p0):
        # p0 is the current position
        # p1 and p2 points define the line
        # global initial_position_, desired_position_
        p1 = self.initial_position_
        p2 = self.desired_position_
        # here goes the equation
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq

        return distance

    #ensures that the angle is between pi and -pi [rad] which is between 180 and -180 [degrees]
    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def main(self):
        rospy.loginfo("starting bug2 algorythm")
        rospy.sleep(3)
        self.initial_position_ = self.position_
        self.target_reached = False
        

        # initialize going to the point
        self.change_state(0)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.regions_ == None:
                continue

            distance_position_to_line = self.distance_to_line(self.position_)

            if self.target_reached:
               self.change_state(2)
               break 

            if self.state_ == 0:
                if self.regions_['front'] > 0.15 and self.regions_['front'] < 1:
                    self.change_state(1)

            elif self.state_ == 1:
                if self.count_state_time_ > 5 and distance_position_to_line < 0.1:
                    self.change_state(0)

            self.count_loop_ = self.count_loop_ + 1
            if self.count_loop_ == 20:
                self.count_state_time_ = self.count_state_time_ + 1
                self.count_loop_ = 0
            
            # rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", self.distance_to_line(self.position_), self.position_.x, self.position_.y)
            rate.sleep()

if __name__ == '__main__':
    b2 = bug2()
    b2.main()
