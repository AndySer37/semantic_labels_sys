import torch.nn as nn
from torch.hub import load_state_dict_from_url
from torchvision.models import ResNet
from .se_module import SELayer


def conv3x3(in_planes, out_planes, stride=1):
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride, padding=1, bias=False)


class SEBasicBlock(nn.Module):
    expansion = 1

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None,
                 *, reduction=16):
        super(SEBasicBlock, self).__init__()
        self.conv1 = conv3x3(inplanes, planes, stride)
        self.bn1 = nn.BatchNorm2d(planes)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = conv3x3(planes, planes, 1)
        self.bn2 = nn.BatchNorm2d(planes)
        self.se = SELayer(planes, reduction)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.se(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)

        return out


class SEBottleneck(nn.Module):
    expansion = 4

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None,
                 *, reduction=16):
        super(SEBottleneck, self).__init__()
        self.conv1 = nn.Conv2d(inplanes, planes, kernel_size=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=stride,
                               padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)
        self.conv3 = nn.Conv2d(planes, planes * 4, kernel_size=1, bias=False)
        self.bn3 = nn.BatchNorm2d(planes * 4)
        self.relu = nn.ReLU(inplace=True)
        self.se = SELayer(planes * 4, reduction)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        residual = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.bn3(out)
        out = self.se(out)

        if self.downsample is not None:
            residual = self.downsample(x)

        out += residual
        out = self.relu(out)

        return out


# def se_resnet18(num_classes=1_000):
#     """Constructs a ResNet-18 model.

#     Args:
#         pretrained (bool): If True, returns a model pre-trained on ImageNet
#     """
#     model = ResNet(SEBasicBlock, [2, 2, 2, 2], num_classes=num_classes)
#     model.avgpool = nn.AdaptiveAvgPool2d(1)
#     return model


# def se_resnet34(num_classes=1_000):
#     """Constructs a ResNet-34 model.

#     Args:
#         pretrained (bool): If True, returns a model pre-trained on ImageNet
#     """
#     model = ResNet(SEBasicBlock, [3, 4, 6, 3], num_classes=num_classes)
#     model.avgpool = nn.AdaptiveAvgPool2d(1)
#     return model


# def se_resnet50(num_classes=1_000, pretrained=False):
#     """Constructs a ResNet-50 model.

#     Args:
#         pretrained (bool): If True, returns a model pre-trained on ImageNet
#     """
#     model = ResNet(SEBottleneck, [3, 4, 6, 3], num_classes=num_classes)
#     model.avgpool = nn.AdaptiveAvgPool2d(1)
#     if pretrained:
#         model.load_state_dict(load_state_dict_from_url(
#             "https://github.com/moskomule/senet.pytorch/releases/download/archive/seresnet50-60a8950a85b2b.pkl"))
#     return model


# def se_resnet101(num_classes=1_000):
#     """Constructs a ResNet-101 model.

#     Args:
#         pretrained (bool): If True, returns a model pre-trained on ImageNet
#     """
#     model = ResNet(SEBottleneck, [3, 4, 23, 3], num_classes=num_classes)
#     model.avgpool = nn.AdaptiveAvgPool2d(1)
#     return model


# def se_resnet152(num_classes=1_000):
#     """Constructs a ResNet-152 model.

#     Args:
#         pretrained (bool): If True, returns a model pre-trained on ImageNet
#     """
#     model = ResNet(SEBottleneck, [3, 8, 36, 3], num_classes=num_classes)
#     model.avgpool = nn.AdaptiveAvgPool2d(1)
#     return model


class CifarSEBasicBlock(nn.Module):
    def __init__(self, inplanes, planes, stride=1, reduction=16, down = False):
        super(CifarSEBasicBlock, self).__init__()
        self.inplanes = inplanes
        self.planes = planes
        self.conv1 = conv3x3(inplanes, planes, stride)
        self.bn1 = nn.BatchNorm2d(planes)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = conv3x3(planes, planes)
        self.bn2 = nn.BatchNorm2d(planes)
        self.se = SELayer(planes, reduction)
        if inplanes != planes or down:
            self.downsample = nn.Sequential(nn.Conv2d(inplanes, planes, kernel_size=1, stride=stride, bias=False),
                                            nn.BatchNorm2d(planes))
        else:
            self.downsample = lambda x: x
        self.stride = stride
        self.down = nn.Sequential(nn.Conv2d(self.inplanes, self.planes, kernel_size=1, stride=self.stride, bias=False),
                nn.BatchNorm2d(self.planes))

    def forward(self, x):
        residual = self.downsample(x)
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.se(out)

        if out.shape != residual.shape:

            residual = self.down(residual)

        out += residual
        out = self.relu(out)

        return out


class CifarSEResNet(nn.Module):
    def __init__(self, block, n_size, num_classes=10, reduction=16):
        super(CifarSEResNet, self).__init__()
        self.inplane = 32
        self.conv1 = nn.Conv2d(
            3, self.inplane, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(self.inplane)
        self.relu = nn.ReLU(inplace=True)
        self.layer1 = self._make_layer(
            block, 32, blocks=n_size, stride=1, reduction=reduction)
        self.layer2 = self._make_layer(
            block, 64, blocks=n_size, stride=2, reduction=reduction)
        self.layer3 = self._make_layer(
            block, 128, blocks=n_size, stride=2, reduction=reduction)
        self.layer4 = self._make_layer(
            block, 256, blocks=n_size, stride=2, reduction=reduction)
        self.layer5 = self._make_layer(
            block, 512, blocks=n_size, stride=2, reduction=reduction)
        self.layer6 = self._make_layer(
            block, 512, blocks=n_size, stride=2, reduction=reduction)
        # self.avgpool = nn.AdaptiveAvgPool2d(1)
        # self.fc = nn.Linear(64, num_classes)
        self.initialize()

    def initialize(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def _make_layer(self, block, planes, blocks, stride, reduction):
        strides = [stride] + [1] * (blocks - 1)
        layers = []
        for stride in strides:
            layers.append(block(self.inplane, planes, stride, reduction))
            self.inplane = planes

        return nn.Sequential(*layers)

    def forward(self, x):        # torch.Size([8, 3, 512, 512])
        C1 = self.conv1(x)       # torch.Size([8, 32, 512, 512])
        C1 = self.bn1(C1)         
        C1 = self.relu(C1)

        ##### lite 
        # C2 = self.layer1(C1)      # torch.Size([8, 32, 512, 512])  

        C3 = self.layer2(C1)      # torch.Size([8, 64, 256, 256])

        C4 = self.layer3(C3)      # torch.Size([8, 128, 128, 128])

        C5 = self.layer4(C4)      # torch.Size([8, 256, 64, 64])

        C6 = self.layer5(C5)      # torch.Size([8, 512, 32, 32])

        C7 = self.layer6(C6)      # torch.Size([8, 512, 16, 16])

        # x = self.avgpool(x)
        # x = x.view(x.size(0), -1)
        # x = self.fc(x)

        return C3, C4, C5, C6 ,C7


class CifarSEPreActResNet(CifarSEResNet):
    def __init__(self, block, n_size, num_classes=10, reduction=16):
        super(CifarSEPreActResNet, self).__init__(
            block, n_size, num_classes, reduction)
        self.bn1 = nn.BatchNorm2d(self.inplane)
        self.initialize()

    def forward(self, x):
        x = self.conv1(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)

        x = self.bn1(x)
        x = self.relu(x)

        x = self.avgpool(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)


def se_resnet20(**kwargs):
    """Constructs a ResNet-18 model.

    """
    model = CifarSEResNet(CifarSEBasicBlock, 3, **kwargs)
    return model


def se_resnet32(**kwargs):
    """Constructs a ResNet-34 model.

    """
    model = CifarSEResNet(CifarSEBasicBlock, 5, **kwargs)
    return model


def se_resnet56(**kwargs):
    """Constructs a ResNet-34 model.

    """
    model = CifarSEResNet(CifarSEBasicBlock, 9, **kwargs)
    return model


def se_preactresnet20(**kwargs):
    """Constructs a ResNet-18 model.

    """
    model = CifarSEPreActResNet(CifarSEBasicBlock, 3, **kwargs)
    return model


def se_preactresnet32(**kwargs):
    """Constructs a ResNet-34 model.

    """
    model = CifarSEPreActResNet(CifarSEBasicBlock, 5, **kwargs)
    return model


def se_preactresnet56(**kwargs):
    """Constructs a ResNet-34 model.

    """
    model = CifarSEPreActResNet(CifarSEBasicBlock, 9, **kwargs)
    return model
