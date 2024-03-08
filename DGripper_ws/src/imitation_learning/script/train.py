#!/usr/bin/env python3
import cnn
import torch
import numpy as np
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import cv2
import pyaml
import os
from tqdm import tqdm

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
model_path = './model'

def generate_random_dataset(dataset_path):
    max_folder = -1
    for folder in os.listdir(dataset_path):
        if int(folder) > max_folder:
            max_folder = int(folder)
    max_folder += 1
    os.mkdir(os.path.join(dataset_path, str(max_folder)))
    print("save random dataset to " + dataset_path)

    for i in tqdm(range(20), desc="Generating random dataset"):
        c1 = np.random.randint(0, 255, size=(480, 640, 1))
        c2 = np.random.randint(0, 255, size=(480, 640, 1))
        c3 = np.random.randint(0, 255, size=(480, 640, 1))
        img = np.concatenate((c1, c2, c3), axis=2)  # 480x640x3
        label = np.random.randn(3)
        label = np.round(label, 3)
        label_name = str(i)+'_'+str(label[0]) + '_' + \
            str(label[1]) + '_' + str(label[2]) + '.jpg'
        cv2.imwrite(os.path.join(
            dataset_path, str(max_folder), label_name), img)


class CustomDataset(Dataset):  # 输入为图片路径和标签
    def __init__(self, dataset_path, transform=None):
        self.dataset_path = dataset_path
        self.transform = transform
        self.imgs = []
        self.labels = []
        for folder in os.listdir(dataset_path):
            # open the folder, get the image
            for file in os.listdir(os.path.join(dataset_path, folder)):
                img = Image.open(os.path.join(dataset_path, folder, file))
                # name structure: index_roll1_roll2_grip.jpg
                label = file.split('_')
                label = label[1:4]  # roll1, roll2, grip
                label[2] = label[2][:-4]  # remove.jpg
                label = [float(i) for i in label]
                # print(label)
                self.imgs.append(img)
                self.labels.append(label)
        self.data_len = len(self.imgs)

    def __getitem__(self, index):
        img = self.imgs[index]
        label = self.labels[index]
        if self.transform is not None:
            img = self.transform(img)
        label = torch.tensor(label)
        return img, label

    def __len__(self):
        return self.data_len


def load_dataset():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    config = pyaml.yaml.load(open('./config.yaml'),
                             Loader=pyaml.yaml.SafeLoader)
    dataset_path = config['dataset_path']

    transform = transforms.Compose([
        transforms.Resize((256, 256)),
        transforms.ToTensor(),
    ])
    custom_dataset = CustomDataset(dataset_path, transform=transform)
    return custom_dataset


def train(dataset):
    batch_size = 1
    data_loader = DataLoader(
        dataset, batch_size=batch_size, shuffle=True)
    model = cnn.Densenet(layer=169, pretrained=True).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    for epoch in range(50):
        print("Epoch:", epoch)
        # for image, label in data_loader:
        #     model.train()
        #     optimizer.zero_grad()
        #     image, label = image.to(device), label.to(device)
        #     outputs = model(image)
        #     criterion = torch.nn.MSELoss()
        #     # 计算损失
        #     loss = criterion(outputs.double(), label.to(device).double())
        #     # 反向传播,adam优化
        #     loss.backward()
        #     optimizer.step()
        #     print(loss.item())

        for image, label in tqdm(data_loader):
            model.train()
            optimizer.zero_grad()
            image, label = image.to(device), label.to(device)
            outputs = model(image)
            criterion = torch.nn.MSELoss()
            # 计算损失
            loss = criterion(outputs.double(), label.to(device).double())
            # 反向传播,adam优化
            loss.backward()
            optimizer.step()
            print(loss.item())

    # save model
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    if not os.path.exists(model_path):
        os.makedirs(model_path)
    torch.save(model, os.path.join(model_path, 'model_'+str(len(os.listdir(model_path)))+'.pth'))


def evaluate(model, dataset):
    batch_size = 1
    data_loader = DataLoader(
        dataset, batch_size=batch_size, shuffle=True)
    for image, label in data_loader:
        image, label = image.to(device), label.to(device)
        outputs = model(image)
        print(outputs)
        print(label)
        print(torch.mean(torch.abs(outputs - label)))
        print("")

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

if __name__ == '__main__':
    dataset = load_dataset()
    model = chose_model()
    evaluate(model, dataset)
    # train(dataset)
