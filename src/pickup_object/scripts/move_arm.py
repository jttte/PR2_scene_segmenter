#!/usr/bin/env python
'''
import rospy


if __name__ == '__main__':

    #initialize the node
    rospy.init_node('pickup_object', anonymous=True)

    #sleep for a moment while everything starts up
    rospy.sleep(1.0)

    # your code here:
    rospy.loginfo("Try to pick up the cup through python script!")

'''

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
    
     
       
def move(w, x, y, z, arm, moveit_commander):

    rospy.init_node('test_arm_node');
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(arm)
    pose_target = geometry_msgs.msg.Pose()
    #pose_target.orientation.x = 0
    #pose_target.orientation.y = 1
    #pose_target.orientation.z = 0
    pose_target.orientation.w = w
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    group.set_pose_target(pose_target)
    myplan = group.plan()
    group.execute(myplan)

