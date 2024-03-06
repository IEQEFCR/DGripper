import cnn
import torch
import numpy as np
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import cv2
import pyaml
import os

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def generate_random_dataset(dataset_path):
    train_img = []
    train_label = []
    for i in range(10):
        img = np.random.randint(0, 255, size=(480, 640, 3))
        # resize img to 256*256
        img = img.astype(np.uint8)
        img = cv2.resize(img, (256, 256))
        # cv2.imshow("img", img)
        cv2.imwrite( 'dataset/'+str(i) + '.jpg', img)
        print(dataset_path + '/' + str(i) + '.jpg')
        # cv2.waitKey(0)
        # img 2 tensor
        train_img.append(img)
        train_label.append(np.random.randn(3))
        print(train_label[-1])
    train_img = np.asarray(train_img)
    train_label = np.asarray(train_label)
    return train_img, train_label


class CustomDataset(Dataset):  # 输入为图片路径和标签
    def __init__(self, train_img, train_label, transform=None):
        # self.data_path = data_path
        self.transform = transform
        # self.imgs = []
        # self.labels = []
        # with open(data_path + '/train.txt', 'r') as f:
        #     for line in f.readlines():
        #         img_path, label = line.strip().split()
        #         self.imgs.append(img_path)
        #         self.labels.append(int(label))
        self.imgs = train_img
        self.labels = train_label
        self.data_len = len(self.imgs)

    def __getitem__(self, index):
        img = self.imgs[index]
        label = self.labels[index]
        # scalar type
        # exchange the channel
        img = np.transpose(img, (2, 0, 1))
        img = torch.from_numpy(img)
        # expected scalar type Byte but found Float
        # if self.transform is not None:
        # img = self.transform(img)
        return img, label

    def __len__(self):
        return self.data_len


def load_dataset():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    #print now path

    config = pyaml.yaml.load(open('./config.yaml'),
                             Loader=pyaml.yaml.SafeLoader)
    dataset_path = config['dataset_path']
    # 找到数据集路径中最大的文件夹
    max_folder = -1
    for folder in os.listdir(dataset_path):
        if int(folder) > max_folder:
            max_folder = int(folder)
    max_folder += 1
    print(max_folder)
    #mkdir dataset_path + '/' + str(max_folder)
    
    generate_random_dataset(dataset_path + '/' + str(max_folder))

if __name__ == '__main__':
    load_dataset()
    exit(0)
    # img, label =
    # 数据集路径
    data_path = "/path/to/your/dataset"
    # 假设有一个包含图像文件和标签的数据集，可以使用transforms进行图像预处理
    transform = transforms.Compose([
        transforms.Resize((64, 64)),
        transforms.ToTensor(),
    ])
    # 创建自定义数据集实例
    custom_dataset = CustomDataset(img, label, transform=transform)
    #
    # 使用 DataLoader 加载数据
    batch_size = 10
    data_loader = DataLoader(
        custom_dataset, batch_size=batch_size, shuffle=True)

    model = cnn.Densenet(layer=121, pretrained=True).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    # 训练
    for epoch in range(100):
        for images, labels in data_loader:
            optimizer.zero_grad()
            # images = images.to(device)
            # labels = labels.to(device)
            # 前向传播
            images = images.type(torch.FloatTensor).to(device)
            outputs = model(images)
            criterion = torch.nn.MSELoss()
            # 计算损失
            loss = criterion(outputs.double(), labels.to(device).double())
            # 反向传播,adam优化
            loss.backward()
            optimizer.step()
            print(loss.item())

    for images, labels in data_loader:
        outputs = model(images.type(torch.FloatTensor))
        print("outputs:", outputs)
        print("labels:", labels)
        break
