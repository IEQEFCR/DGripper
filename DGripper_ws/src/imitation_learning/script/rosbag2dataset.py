import rospy
import os
from sensor_msgs.msg import Image

dataset_path = './dataset'

gripper_state =None
last_gripper_state =None

def gripper_state_callback(msg):
    global gripper_state
    gripper_state = msg.data

def tactile_image_callback(msg):
    global tactile_image
    tactile_image = msg

def play_bag():
    rospy.init_node('play_bag', anonymous=True)
    bag = rosbag.Bag('data.bag')
    for topic, msg, t in bag.read_messages(topics=['/camera/depth_registered/image_raw']):
        print(msg)
    bag.close()

def step_dif():
    

def make_dataset():
    if not os.path.exists(dataset_path):
        os.makedirs(dataset_path)
    max_folder = 0
    for folder in os.listdir(dataset_path):
        if int(folder) > max_folder:
            max_folder = int(folder)
    max_folder += 1
    dataset_path = os.path.join(dataset_path, str(max_folder))
    os.makedirs(dataset_path)
    return dataset_path


if __name__ == '__main__':
    pid = os.fork()
    if pid == 0:
        make_dataset()
    else:
        print("play the bag")
        #一一播放rosbag路径下所有包，播完要重置全局变量