#!/usr/bin/env python
import subprocess

def open_gripper():

    args = ("rosrun", "move_gripper", "open_gripper")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
def close_gripper():
    args = ("rosrun", "move_gripper", "close_gripper")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
