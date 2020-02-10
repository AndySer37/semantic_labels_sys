# import torch.nn as nn
# # import torch.utils.model_zoo as model_zoo
# import torch



# class VGG(nn.Module):

#     def __init__(self, features, num_classes=1000, init_weights=True):
#         super(VGG, self).__init__()
#         self.features = features
#         self.classifier = nn.Sequential(
#             nn.Linear(512 * 7 * 7, 4096),
#             nn.ReLU(True),
#             nn.Dropout(),
#             nn.Linear(4096, 4096),
#             nn.ReLU(True),
#             nn.Dropout(),
#             nn.Linear(4096, num_classes),
#         )
#         if init_weights:
#             self._initialize_weights()

#     def forward(self, x):
#         x = self.features(x)
#         x = x.view(x.size(0), -1)
#         x = self.classifier(x)
#         return x

#     def _initialize_weights(self):
#         for m in self.modules():
#             if isinstance(m, nn.Conv2d):
#                 nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
#                 if m.bias is not None:
#                     nn.init.constant_(m.bias, 0)
#             elif isinstance(m, nn.BatchNorm2d):
#                 nn.init.constant_(m.weight, 1)
#                 nn.init.constant_(m.bias, 0)
#             elif isinstance(m, nn.Linear):
#                 nn.init.normal_(m.weight, 0, 0.01)
#                 nn.init.constant_(m.bias, 0)


# def make_layers(cfg, batch_norm=False):
#     layers = []
#     in_channels = 3
#     for v in cfg:
#         if v == 'M':
#             layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
#         else:
#             conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
#             if batch_norm:
#                 layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
#             else:
#                 layers += [conv2d, nn.ReLU(inplace=True)]
#             in_channels = v
#     return nn.Sequential(*layers)


# cfg = {
#     'A': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
#     'B': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
#     'D': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
#     'E': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
# }

# class VGG16(nn.Module):

#     def __init__(self, pretrain=True):
#         super().__init__()
#         net = VGG(make_layers(cfg['D']), init_weights=False)
#         if pretrain:
#             net.load_state_dict(model_zoo.load_url(model_urls['vgg16']))

#         self.stage1 = nn.Sequential(*[net.features[layer] for layer in range(0, 5)])
#         self.stage2 = nn.Sequential(*[net.features[layer] for layer in range(5, 10)])
#         self.stage3 = nn.Sequential(*[net.features[layer] for layer in range(10, 17)])
#         self.stage4 = nn.Sequential(*[net.features[layer] for layer in range(17, 24)])
#         self.stage5 = nn.Sequential(*[net.features[layer] for layer in range(24, 31)])

#         # self.attn1 = Self_Attn( 256, 'relu')
#         # self.attn2 = Self_Attn( 512,  'relu')
#     def forward(self, x):
#         C1 = self.stage1(x)
#         C2 = self.stage2(C1)
#         C3 = self.stage3(C2)
#         # C3, p1 = self.attn1(C3)
#         C4 = self.stage4(C3)
#         # C4, p1 = self.attn2(C4)
#         C5 = self.stage5(C4)
#         return C1, C2, C3, C4, C5


# if __name__ == '__main__':
#     import torch
#     input = torch.randn((4, 3, 512, 512))
#     net = VGG16()
#     C1, C2, C3, C4, C5 = net(input)
#     print(C1.size())
#     print(C2.size())
#     print(C3.size())
#     print(C4.size())
#     print(C5.size())
