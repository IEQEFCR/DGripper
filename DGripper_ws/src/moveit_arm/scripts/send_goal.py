#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import DisplayTrajectory

def rot2quet(rot):
    # rot: 3*3
    # quet: 1*4
    quet = np.zeros(4)
    quet[3] = np.sqrt(1+rot[0,0]+rot[1,1]+rot[2,2])/2
    quet[0] = (rot[2,1]-rot[1,2])/(4*quet[3])
    quet[1] = (rot[0,2]-rot[2,0])/(4*quet[3])
    quet[2] = (rot[1,0]-rot[0,1])/(4*quet[3])
    return quet


# def send_position(plan):
#     #transform the postion list in the plan to multi64array

class MoveItIkDemo:
    def __init__ (self):

        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('send_goal')
                
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
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)

        joint_goal = arm.get_current_joint_values()
        print('joint_goal:',joint_goal)
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        # arm.set_named_target('origin')
        # arm.go()
        # rospy.sleep(1)
        while rospy.is_shutdown()==False:
            #询问目标位置
            print('input the target position and the x axis direction:')
            pose_str = input()
            #split pose_str by space
            pose_list = pose_str.split(' ')
            if pose_list[0] == 'exit':
                # 关闭并退出moveit
                moveit_commander.roscpp_shutdown()
                moveit_commander.os._exit(0)
            if pose_list[0] == 'origin':
                arm.set_named_target('origin')
                arm.go()
                rospy.sleep(1)
                continue
            x = pose_list[0]
            y = pose_list[1]
            z = pose_list[2]
            x1 = pose_list[3]
            x2 = pose_list[4]
            x3 = pose_list[5]
            x = float(x)
            y = float(y)
            z = float(z)
            x1 = float(x1)
            x2 = float(x2)
            x3 = float(x3)
            # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
            # 姿态使用四元数描述，基于base_link坐标系
            target_pose = PoseStamped()
            target_pose.header.frame_id = reference_frame
            target_pose.header.stamp = rospy.Time.now()     
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z

            axis_x = np.array([-x1,-x2,-x3])
            axis_x = axis_x/np.linalg.norm(axis_x)
            pose_vector = np.array([x,y,z])
            pose_vector = pose_vector/np.linalg.norm(pose_vector)
            axis_y = np.cross(axis_x,pose_vector)
            axis_y = axis_y/np.linalg.norm(axis_y)
            axis_z = np.cross(axis_x,axis_y)
            axis_z = axis_z/np.linalg.norm(axis_z)
            
            rot=np.array([[axis_x[0],axis_y[0],axis_z[0]],[axis_x[1],axis_y[1],axis_z[1]],[axis_x[2],axis_y[2],axis_z[2]]])

            quet=rot2quet(rot)
            target_pose.pose.orientation.x = quet[0]
            target_pose.pose.orientation.y = quet[1]
            target_pose.pose.orientation.z = quet[2]
            target_pose.pose.orientation.w = quet[3]

            
            # way_points = []
            # way_points.append(target_pose.pose)
            # #use cartesian path
            # (plan, fraction) = arm.compute_cartesian_path(way_points, 0.01, 0.0, True)
            # arm.execute(plan)

            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()
            # 设置机械臂终端运动的目标位姿
            arm.set_pose_target(target_pose, end_effector_link)
            # 规划运动路径
            plan_success, plan, planning_time, error_code = arm.plan()
            # print(traj)
            # 按照规划的运动路径控制机械臂运动
            #plan.joint_trajectory 2 ndarray
            #define a numpy array named angle_list to store the joint angle

            # angle_list = []

            # for i in plan.joint_trajectory.points:
            #     angle_list = np.append(angle_list,i.positions)
            # angle_list*=180/np.pi

            # msg = Float64MultiArray()
            # pub = rospy.Publisher('/arm', Float64MultiArray, queue_size=10)
            # msg.data = angle_list
            # pub.publish(msg)
            # rospy.sleep(0.05)
            # pub.publish(msg)
            if plan_success:
                print('plan_success')
                arm.execute(plan)
            else :
                print('plan_fail')

if __name__ == "__main__":
    MoveItIkDemo()
    # get_plan()