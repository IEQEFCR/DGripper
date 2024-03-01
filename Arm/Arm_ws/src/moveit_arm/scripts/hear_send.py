#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory

def callback(data):
    # print(data.trajectory[0].joint_trajectory.points)
    global launch_time ,pub,msg;
    if rospy.Time.now() - launch_time > rospy.Duration(0.1):
            angle_list = []
            for i in data.trajectory[0].joint_trajectory.points:
                temp = list(i.positions)
                temp [5] = 0
                angle_list = np.append(angle_list,temp)
                # angle_list[-1][-1] = 0
            angle_list*=180/np.pi
            msg.data = angle_list
            pub.publish(msg)
            # rospy.sleep(0.05)
            # pub.publish(msg)

def get_plan():
    sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, callback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('hear_send')
    launch_time = rospy.Time.now()
    pub = rospy.Publisher('/arm', Float64MultiArray, queue_size=10)
    msg = Float64MultiArray()
    rospy.Rate(5)
    get_plan()