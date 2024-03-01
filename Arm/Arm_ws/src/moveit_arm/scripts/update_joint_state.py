#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('update_joint_state')
joint_state=[0.0,0.0,0.0,0.0,0.0,0.0]

pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    for i in range(6):
        joint_state[i]=1
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = joint_state
    pub.publish(msg)
    rate.sleep()
