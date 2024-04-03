#!/usr/bin/env python
import rospy, sys
import numpy as np
import sys, tty, termios
from std_msgs.msg import String
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == '__main__':
    rospy.init_node('teleop_all')
    pub = rospy.Publisher('/teleop', String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = getch()
        #if key ==esc: exit
        if key == 'q':
            break
        msg = String()
        msg.data = key
        pub.publish(msg)