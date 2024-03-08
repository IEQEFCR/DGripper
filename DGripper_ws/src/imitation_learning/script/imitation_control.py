#!/usr/bin/env python3
import cnn
import torch
import numpy as np
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import cv2 as cv
import pyaml
import os
from tqdm import tqdm
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

model_path = './model'

def chose_model():
    os.chdir(os.path.dirname(__file__))
    index = 0
    model_list = []
    for model in os.listdir(model_path):
        print('['+str(index)+'] ', model)
        model_list.append(model)
        index += 1
    model_index = int(input('Choose a model: '))
    model=torch.load(os.path.join(model_path, model_list[model_index]))
    return model

class ImitationControl:
    def __init__(self, model):
        rospy.init_node('imitation_control')
        self.tactile_merged = None
        self.state=[None, None, None]
        self.model = model
        self.bridge = CvBridge()

        self.camera_sub = rospy.Subscriber('tactile_merged', Image, self.tactile_merged_callback)
        self.state_sub = rospy.Subscriber('/gripper_state', Float32MultiArray, self.state_callback)
        self.pub = rospy.Publisher('/gripper/control', Float32MultiArray, queue_size=10)

        self.transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
        ])

    def tactile_merged_callback(self, msg):
        self.tactile_merged = Image.fromarray(self.bridge.imgmsg_to_cv2(msg.data, 'passthrough'))

    def state_callback(self, msg):
        self.state = msg.data

    def poscontrol(self, output):
        control = Float32MultiArray()
        control.data = [0, 0, 0,0,0] #可能还需要加角度
        for i in range(3):
            control.data[i] = self.state[i] + output[i]
        self.pub.publish(control)
        while 1:    
            for i in range(3):
                if abs(self.state[i] - control.data[i]) > 1:
                    rospy.sleep(0.1)#等待0.1s
                    continue
            break #说明三个电机都到位了
        print('Get to the target position')


    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.tactile_merged is not None:
                img = self.transform(self.tactile_merged)
                img = img.unsqueeze(0).to(device)
                output = self.model(img)
                output = output.squeeze(0).detach().cpu().numpy() 
                self.poscontrol(output)
            else :
                print('Waiting for tactile_merged')
            rate.sleep()


if __name__ == '__main__':
    model = chose_model()
    ic = ImitationControl(model)



