#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory
#import nav_msgs/Odometry
from nav_msgs.msg import Odometry

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def rot2quet(rot):
    # rot: 3*3
    # quet: 1*4
    quet = np.zeros(4)
    quet[3] = np.sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])/2
    quet[0] = (rot[2,1]-rot[1,2])/(4*quet[3])
    quet[1] = (rot[0,2]-rot[2,0])/(4*quet[3])
    quet[2] = (rot[1,0]-rot[0,1])/(4*quet[3])
    return quet

def t265_pose_callback(msg):
    global t265_pose
    t265_pose = msg
    # print('t265_pose:',t265_pose)

# def send_position(plan):
#     #transform the postion list in the plan to multi64array

t265_pose = Odometry()

class MoveItIkDemo:
    def __init__ (self):

        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('send_goal')
        global t265_pose
        t265_pose_sub = rospy.Subscriber('/camera/odom/sample', Odometry, t265_pose_callback)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)

        joint_goal = arm.get_current_joint_values()
        print('joint_goal:',joint_goal)
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.25)
        arm.set_max_velocity_scaling_factor(0.25)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('origin')
        # arm.go()
        last_time = rospy.get_time()
        rate = rospy.Rate(30)
        way_points=[]
        now_pose = PoseStamped()

        now_pose.pose.position.x = 0.313
        now_pose.pose.position.y = 0
        now_pose.pose.position.z = 0.625
        now_pose.pose.orientation.x = 0.707
        now_pose.pose.orientation.y = 0
        now_pose.pose.orientation.z = 0.707
        now_pose.pose.orientation.w = 0
        last_time = rospy.get_time()
        single_step = 0.01
        while rospy.is_shutdown()==False:
            key = getch()
            if key == 'w':
                now_pose.pose.position.x += single_step
                way_points.append(now_pose.pose)
            elif key == 's':
                now_pose.pose.position.x -= single_step
                way_points.append(now_pose.pose)
            elif key == 'a':
                now_pose.pose.position.y += single_step
                way_points.append(now_pose.pose)
            elif key == 'd':
                now_pose.pose.position.y -= single_step
                way_points.append(now_pose.pose)
            elif key == ' ':
                now_pose.pose.position.z += single_step
                way_points.append(now_pose.pose)
            #elif key is shift:
            elif key == 'c':
                now_pose.pose.position.z -= single_step
                way_points.append(now_pose.pose)
            elif key =='u':
                single_step*=2
            elif key =='j':
                single_step/=2
                
            print(single_step)
            
            rate.sleep()

            if rospy.get_time()-last_time>0.5:
                (plan, fraction) = arm.compute_cartesian_path(
                    way_points,   # waypoints to follow
                    0.002,        # eef_step
                    0.0)         # jump_threshold
                arm.execute(plan)
                way_points = []
                last_time = rospy.get_time()
            # arm.set_start_state_to_current_state()
            # # 设置机械臂终端运动的目标位姿
            # arm.set_pose_target(target_pose, end_effector_link)
            
            # # 规划运动路径
            # plan_success, plan, planning_time, error_code = arm.plan()

            # if plan_success:
            #     print('plan_success')
            #     arm.execute(plan)
            # else :
            #     print('plan_fail')

if __name__ == "__main__":
    MoveItIkDemo()
    # get_plan()