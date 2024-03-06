import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
import torch.optim as optim

# 定义模型


class Densenet121Model(nn.Module):
    def __init__(self, num_features=3):
        super(Densenet121Model, self).__init__()
        self.densenet = models.densenet121(pretrained=True)
        self.densenet.classifier = nn.Linear(
            self.densenet.classifier.in_features, num_features)

    def forward(self, x):
        return self.densenet(x)


# 创建模型实例
model = Densenet121Model(num_features=3)

# 加载图片
img = Image.open("your_image.jpg").convert('RGB')
transform = transforms.Compose([
    transforms.Resize((640, 480)),
    transforms.ToTensor(),
])
img = transform(img)
