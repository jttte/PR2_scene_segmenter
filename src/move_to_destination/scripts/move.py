#!/usr/bin/env python
import rospy
import subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Pose

current_pose = None
extract = False
block = False

def callback(data):
    rospy.loginfo("got pose")
    print data.position.x
    global current_pose
    current_pose = data
    global extract
    extract = True

    done = False
    while (not done):
        print "examine"
        if extract == True & block == False:
            global block
            block = True
            x = current_pose.position.x
            y = current_pose.position.y
            z = current_pose.position.z
            #move x
            if x>0.1:
                move_left(x)
            if x<-0.1:
                move_right(x)

            done = True

    
    
def listener():
    rospy.loginfo("Initialize listener")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("segmented_objects", Pose, callback)

def move_left(x):
    print 'move left'
    iteration = x/0.1
    args = ("rosrun", "move_base", "turn_left")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output

    for i in range(iteration):
        args = ("rosrun", "move_base", "move_forward")
        popen = subprocess.Popen(args, stdout=subprocess.PIPE)
        popen.wait()
        output = popen.stdout.read()
    print output

    args = ("rosrun", "move_base", "turn_right")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
    
def move_right(x):
    print 'move right'
    
    iteration = x/0.1
    args = ("rosrun", "move_base", "turn_right")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output

    for i in range(iteration):
        args = ("rosrun", "move_base", "move_forward")
        popen = subprocess.Popen(args, stdout=subprocess.PIPE)
        popen.wait()
        output = popen.stdout.read()
    print output

    args = ("rosrun", "move_base", "turn_left")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output

def move_forward(z):
    print 'move forward'
    

if __name__ == '__main__':
    listener()
    done = False
    while (not done):
        print "examine"
        if extract == True:
            x = current_pose.position.x
            y = current_pose.position.y
            z = current_pose.position.z
            #move x
            if x>0.1:
               move_left(x)
            if x<-0.1:
                move_right(x)

            done = true
    
        #move x
        #while z > 1:
        #    move_forward(z)

        #if(x < 0.1 & x > -0.1 & z <1):
        #    done= True
