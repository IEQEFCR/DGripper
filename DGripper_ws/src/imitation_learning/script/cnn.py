from torchvision import models
import torch
from torch import nn


def Resnet(layer=50, pretrained=True):
    if layer == 18:
        model = models.resnet18(pretrained=pretrained)
        print('use Resnet-18')
    elif layer == 34:
        model = models.resnet34(pretrained=pretrained)
        print('use Resnet-34')
    elif layer == 50:
        model = models.resnet50(pretrained=pretrained)
        print('use Resnet-50')
    else:
        model = models.resnet101(pretrained=pretrained)
        print('use Resnet-101')
    num_in_features = model.fc.in_features
    model.fc = nn.Linear(num_in_features, 6)
    return model


def Densenet(layer=169, pretrained=True):
    if layer == 121:
        model = models.densenet121(pretrained=pretrained)
        print('use Densenet-121')
    elif layer == 161:
        model = models.densenet161(pretrained=pretrained)
        print('use Densenet-161')
    elif layer == 169:
        model = models.densenet169(pretrained=pretrained)
        print('use Densenet-169')
    else:
        model = models.densenet201(pretrained=pretrained)
        print('use Densenet-201')
    num_in_features = model.classifier.in_features
    model.classifier = nn.Linear(num_in_features, 3)
    return model
