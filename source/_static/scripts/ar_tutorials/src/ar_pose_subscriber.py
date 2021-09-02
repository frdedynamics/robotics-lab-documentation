#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

detected_markers=set()
markers_in_sight=['a']


def callback(msg):
    global detected_markers, markers_in_sight
    # markers_in_sight.clear() doesn't work. This is a Python 3.3+ command
    markers_in_sight = []
    n = len(msg.markers)
    for i in range(n):
        detected_markers.add(msg.markers[i].id)
        markers_in_sight.append(msg.markers[i].id)
    # print("detected_markers"+str(detected_markers))
    # print(markers_in_sight)


def ar_action(markers):
    human_id = 2
    if human_id in markers:
        print("Human found!")


def main():
    global markers_in_sight
    rospy.init_node('ar_pose_subscriber', anonymous=False)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        ar_action(markers_in_sight)
        rate.sleep()


if __name__ == '__main__':
    main()