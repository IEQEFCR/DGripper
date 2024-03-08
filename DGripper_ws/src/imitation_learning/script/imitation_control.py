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
        self.model = model
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/gripper/control', Float32MultiArray, queue_size=10)
        self.transform = transforms.Compose([
                    transforms.Resize((256, 256)),
                    transforms.ToTensor(),
                    ])
        self.roll1 = 0
        self.roll2 = 0
        self.grip = 0




if __name__ == '__main__':
    model = chose_model()
    ic = ImitationControl(model)



