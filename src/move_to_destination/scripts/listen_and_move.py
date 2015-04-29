#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def get_pose_and_move():
    rospy.wait_for_service('get_xylophone_pose')
    try:
        pose = rospy.ServiceProxy('get_xylophone_pose', MeshCloud)
        print "get pose"
    except rospy.ServiceException, e:
        print "Service call failed"

if __name__ == "__main__":
    get_pose_and_move()
