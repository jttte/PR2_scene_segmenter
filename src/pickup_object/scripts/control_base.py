#!/usr/bin/env python
import subprocess

def move_forward():
    args = ("rosrun", "move_base", "move_forward")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
def move_backward():
    args = ("rosrun", "move_base", "move_backward")
    popen = subprocess.Popen(args, stdout=subprocess.PIPE)
    popen.wait()
    output = popen.stdout.read()
    print output
