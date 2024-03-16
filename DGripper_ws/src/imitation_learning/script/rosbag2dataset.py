import rospy
import os
from sensor_msgs.msg import Image
import cv2 as cv
import rosbag
# sqrt is in the math module
from math import sqrt
from cv_bridge import CvBridge

dataset_path = './dataset'
bag_path = './rosbag'

diff_threshold = 4  # 单位是mm


def diff_state(last_state, now_state):
    # 计算两个状态之间的差值
    diff = 0
    for i in range(3):  # N20[0\1]位移，N20[2]夹爪间距
        diff += (now_state[i] - last_state[i]) ^ 2
    return sqrt(diff)


if __name__ == '__main__':
    # chdir to the python file directory
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    bridge = CvBridge()
    dataset_index = 0

    for bag in os.listdir(bag_path):
        bag_file = os.path.join(bag_path, bag)
        print('Processing', bag_file)
        save_path = os.path.join(dataset_path, str(dataset_index))
        dataset_index += 1
        if not os.path.exists(save_path):  # 创建文件夹
            os.makedirs(save_path)
        print('Saving to', save_path)
        print('Press y to start processing or n to skip')
        if input() == 'n':
            continue
        state_list = []
        tactile_list = []
        img_list = []
        bag = rosbag.Bag(bag_file)
        last_state = [None, None, None, None, None, None]

        # 读取gripper_state
        for topic, msg, t in bag.read_messages(topics=['/gripper_state']):
            now_state = msg.data+[t]
            if last_state[0] == None:  # 第一个状态
                # last_state 前五个元素为msg.data，后一个元素为时间戳
                last_state = now_state
                state_list.append(now_state)
                continue
             # 五个电机位置信息 N20[0\1]位移，N20[2]夹爪间距，servo[0\1]角度
            if diff_state(last_state, now_state) > diff_threshold:
                state_list.append(now_state)
                last_state = now_state  # 更新上一个状态
                continue

        index = 0
        for topic, msg, t in bag.read_messages(topics=['/tactile_merged']):
            if t >= state_list[index][5]:  # gripper_state的时间对应的tactile_img
                # 保存tactile_merged和gripper_state
                tactile_list.append(msg)
                index += 1
                if index >= len(state_list):
                    break

        index = 0
        for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw']):
            if t >= state_list[index][5]:
                # 保存color_img
                img_list.append(msg)
                index += 1
                if index >= len(state_list):
                    break

        for i in range(len(state_list)-1):  # 数据集格式,i_roll1_roll2_gripper.jpg
            img = bridge.imgmsg_to_cv2(img_list[i], 'passthrough')
            # img 转灰度图
            img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # 将img resize到和tactile_img一样的大小
            img = cv.resize(
                img, (tactile_list[i].width, tactile_list[i].height))
            tactile_img = bridge.imgmsg_to_cv2(tactile_list[i], 'passthrough')
            # 将tactile_img的第三个通道设置为img
            tactile_img[:, :, 2] = img
            label = str(i)+'_'+str(state_list[i+1][0]-state_list[i][0])+'_'+str(
                state_list[i+1][1]-state_list[i][1])+'_'+str(state_list[i+1][2]-state_list[i][2])+'.jpg'
            cv.imwrite(os.path.join(save_path, label), tactile_img)
        bag.close()
        print("Done!")
    print("All done!")
