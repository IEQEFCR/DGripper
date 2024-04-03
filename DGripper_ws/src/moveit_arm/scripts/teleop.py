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
import tf

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

def teleop_callback(msg):
    global teleop , received_time
    received_time = rospy.get_time()
    teleop = msg.data

t265_pose = Odometry()
teleop_sub = rospy.Subscriber('/teleop', String, teleop_callback)
teleop = 'n'
received_time = None


class MoveItIkDemo:
    def __init__ (self):

        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('send_goal')
        global t265_pose, teleop, received_time
        received_time = rospy.get_time()
        rospy.sleep(0.3)
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

        # tf look up between world and link6
        listener = tf.TransformListener()
        listener.waitForTransform('world', 'link6', rospy.Time(), rospy.Duration(4.0))
        link62world = listener.lookupTransform('world', 'link6', rospy.Time(0))
        gripper2world = listener.lookupTransform('world', 'gripper', rospy.Time(0))

        now_pose.pose.position.x = link62world[0][0]
        now_pose.pose.position.y = link62world[0][1]
        now_pose.pose.position.z = link62world[0][2]
        now_pose.pose.orientation.x = 0.707
        now_pose.pose.orientation.y = 0
        now_pose.pose.orientation.z = 0.707
        now_pose.pose.orientation.w = 0

        last_time = rospy.get_time()
        single_step = 0.01
        rate = rospy.Rate(30)

        while rospy.is_shutdown()==False:
            now_time = rospy.get_time()
            if now_time - received_time < 0.2:
                key = teleop[0]
                print('teleop:',key)
                teleop = 'n'  
            else:
                key = 'n'

            link62world = listener.lookupTransform('world', 'link6', rospy.Time(0))
            now_pose.pose.position.x = link62world[0][0]
            now_pose.pose.position.y = link62world[0][1]
            now_pose.pose.position.z = link62world[0][2]
            if key =='q':
                break
            if key == 's':
                now_pose.pose.position.x += single_step
                way_points.append(now_pose.pose)
            elif key == 'w':
                now_pose.pose.position.x -= single_step
                way_points.append(now_pose.pose)
            elif key == 'd':
                now_pose.pose.position.y += single_step
                way_points.append(now_pose.pose)
            elif key == 'a':
                now_pose.pose.position.y -= single_step
                way_points.append(now_pose.pose)
            elif key == ' ':
                now_pose.pose.position.z += single_step
                way_points.append(now_pose.pose)
            #elif key is shift:
            elif key == 'c':
                now_pose.pose.position.z -= single_step
                if now_pose.pose.position.z < 0.185:
                    now_pose.pose.position.z = 0.185
                way_points.append(now_pose.pose)
            elif key =='u':
                single_step*=2
            elif key =='j':
                single_step/=2
            #clear the screen

            rate.sleep()
            if rospy.get_time()-last_time>0.5 and len(way_points)>0:
                (plan, fraction) = arm.compute_cartesian_path(
                    way_points,   # waypoints to follow
                    0.002,        # eef_step
                    0.0)         # jump_threshold
                arm.execute(plan)
                way_points = []
                last_time = rospy.get_time()

            print("\033c")
            print("single_step:<<<<",single_step,">>>>")
            link62world = listener.lookupTransform('world', 'link6', rospy.Time(0))
            gripper2world = listener.lookupTransform('world', 'gripper', rospy.Time(0))
            #round the number
            for i in range(3):
                link62world[0][i] = round(link62world[0][i],3)
                gripper2world[0][i] = round(gripper2world[0][i],3)
            print('link62world:',link62world[0])
            print('gripper2world:',gripper2world[0])
            rate.sleep()


if __name__ == "__main__":
    MoveItIkDemo()
    # get_plan()