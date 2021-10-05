#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

class map_sub():
    def __init__(self):
        rospy.init_node('map_sub', anonymous=False)
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)

def map_callback(self, data):
        self.map = data
