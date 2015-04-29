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

import roslib
#roslib.load_manifest('pr2_gripper_reactive_approach')
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
    
class Arm:
	def __init__(self, arm_name):
	#arm_name should be l_arm or r_arm
		self.name = arm_name #/l_gripper_controller/command
		self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                                                    JointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')
    
	def move(self, angles):
		goal = JointTrajectoryGoal()
		char = self.name[0] #either 'r' or 'l'
		goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
						char+'_shoulder_lift_joint',
						char+'_upper_arm_roll_joint',
						char+'_elbow_flex_joint',
						char+'_forearm_roll_joint',
						char+'_wrist_flex_joint',
						char+'_wrist_roll_joint']
		point = JointTrajectoryPoint()
		point.positions = angles
		point.time_from_start = rospy.Duration(3)
		goal.trajectory.points.append(point)
		self.jta.send_goal_and_wait(goal)
    
def main():
	arm = Arm('l_arm')
	arm.move([0.1]*7)
	#arm.move([-0.1]*7)
     
       
if __name__ == '__main__':
	rospy.init_node('lift_right_arm_tester')
	main()
