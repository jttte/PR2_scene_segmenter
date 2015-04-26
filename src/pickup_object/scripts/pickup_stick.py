#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import move_arm
import control_gripper
import control_base
    
     
       
if __name__ == '__main__':
    control_gripper.open_gripper()
    moveit_commander.roscpp_initialize(sys.argv)
    move_arm.move(0.0, 0.7, 0.0, 0.92, "left_arm", moveit_commander)
    control_base.move_forward()
    control_gripper.close_gripper()
    #control_base.move_backward()

