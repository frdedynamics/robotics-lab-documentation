#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MapInterpreterClass:
    def __init__(self):
        rospy.init_node("MapInterpreterClass", anonymous =False)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.clbk_map)

        self.map = OccupancyGrid()
        while len(self.map.data) < 1:
            rospy.loginfo("waiting for map...")
            rospy.Rate(1).sleep()

        self.borders_pub = rospy.Publisher('/borders', Marker, queue_size=10)
        self.border_points = Marker()
        self.border_points.header.frame_id = self.map.header.frame_id
        self.border_points.ns = "markers"
        self.border_points.id = 0
        self.border_points.type = self.border_points.POINTS
        self.border_points.action = self.border_points.ADD
        self.border_points.pose.orientation.w =1.0
        self.border_points.scale.x=0.1
        self.border_points.scale.y=0.1
        self.border_points.color.r = 255.0/255.0
        self.border_points.color.g = 0.0/255.0
        self.border_points.color.b = 0.0/255.0
        self.border_points.color.a = 1
        self.border_points.lifetime = rospy.Duration()

    def clbk_map(self, msg):
        self.map = msg

    def get_map_pos(self, map_iter):
        map_pos = Point()
        map_pos.x = int(map_iter/self.map.info.width)
        map_pos.y = map_iter - map_pos.x*self.map.info.width

        return map_pos

    def getPosition(self, x, y):
        map_position = Point()
        map_position.x = self.map.info.origin.position.x + y*self.map.info.resolution
        map_position.y = self.map.info.origin.position.y + x*self.map.info.resolution

        return map_position

    def run(self):
        r = rospy.Rate(1)



        for i in range(len(self.map.data)):
            if self.map.data[i] > 80:
                map_iter = i
                break

        map_pos = self.get_map_pos(map_iter)
        rospy.loginfo(map_pos)
        map_position = self.getPosition(map_pos.x, map_pos.y)
        rospy.loginfo(map_position)



        while True:

            self.border_points.points = [map_position]
            self.borders_pub.publish(self.border_points)
            r.sleep()



if __name__ == '__main__':
    mi = MapInterpreterClass()
    mi.run()
