#! /usr/bin/env python

# import ros stuff
import rospy
from tf import transformations
import math


# global variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 9
desired_position_.y = 5
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.2

# publishers
pub = None

#ensures that the angle is between pi and -pi [rad] which is between 180 and -180 [degrees]
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

#if the robot is not facing towards the target position it starts turning to correct that
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
    pub.publish(twist_msg)
    rospy.loginfo('Yaw error: ['+str(err_yaw)+']')
    rospy.loginfo('Yaw precision: ['+str(yaw_precision_)+']')
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        rospy.loginfo('Yaw error: ['+str(err_yaw)+']')
        change_state(1)

#as long as the robot faces towards the target, move forward
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        pub.publish(twist_msg)
    else:
        rospy.loginfo('Position error: '+str(err_pos))
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        rospy.loginfo('Yaw error: '+str(err_yaw))
        change_state(0)
#function should be called when the target is reached; stops all movement of the robot
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    rospy.loginfo('go-to-point -> finished')

def change_state(state):
    global state_
    state_ = state
    rospy.loginfo('State changed to '+str(state_))

#callback function for odometry subscriber 
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def main():
    global pub, active_
    
    #ADD CODE HERE


if __name__ == '__main__':
    main()
