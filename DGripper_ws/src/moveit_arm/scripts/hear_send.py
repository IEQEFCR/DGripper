#!/usr/bin/env python

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState

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

def arm_state_callback(data):
    global arm_state_pub
    angle_list = data.data.split(',')
    arm_state_msg.position = [float(i) for i in angle_list]
    arm_state_msg.velocity = [0,0,0,0,0,0]
    arm_state_msg.effort = [0,0,0,0,0,0]
    arm_state_msg.header.stamp = rospy.Time.now()
    arm_state_pub.publish(arm_state_msg)

if __name__ == "__main__":
    rospy.init_node('hear_send')
    launch_time = rospy.Time.now()
    pub = rospy.Publisher('/arm', Float64MultiArray, queue_size=10)
    arm_state_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
    msg = Float64MultiArray()
    arm_state_msg = JointState()
    rospy.Rate(20)
    sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, callback)
    arm_state = rospy.Subscriber('/arm_state', String, arm_state_callback)
    rospy.spin()