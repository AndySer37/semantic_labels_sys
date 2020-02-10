import torch.nn as nn
import torch.utils.model_zoo as model_zoo
import torch

class Self_Attn(nn.Module):
    """ Self attention Layer"""
    def __init__(self,in_dim,activation):
        super(Self_Attn,self).__init__()
        self.chanel_in = in_dim
        self.activation = activation
        
        self.query_conv = nn.Conv2d(in_channels = in_dim , out_channels = in_dim//8 , kernel_size= 1)
        self.key_conv = nn.Conv2d(in_channels = in_dim , out_channels = in_dim//8 , kernel_size= 1)
        self.value_conv = nn.Conv2d(in_channels = in_dim , out_channels = in_dim , kernel_size= 1)
        self.gamma = nn.Parameter(torch.zeros(1))

        self.softmax  = nn.Softmax(dim=-1) #
    def forward(self,x):
        """
            inputs :
                x : input feature maps( B X C X W X H)
            returns :
                out : self attention value + input feature 
                attention: B X N X N (N is Width*Height)
        """
        m_batchsize,C,width ,height = x.size()
        proj_query  = self.query_conv(x).view(m_batchsize,-1,width*height).permute(0,2,1) # B X CX(N)
        proj_key =  self.key_conv(x).view(m_batchsize,-1,width*height) # B X C x (*W*H)
        energy =  torch.bmm(proj_query,proj_key) # transpose check
        attention = self.softmax(energy) # BX (N) X (N) 
        proj_value = self.value_conv(x).view(m_batchsize,-1,width*height) # B X C X N

        out = torch.bmm(proj_value,attention.permute(0,2,1) )
        out = out.view(m_batchsize,C,width,height)
        
        out = self.gamma*out + x
        # print (self.gamma)
        return out,attention

model_urls = {
    'vgg11': 'https://download.pytorch.org/models/vgg11-bbd30ac9.pth',
    'vgg13': 'https://download.pytorch.org/models/vgg13-c768596a.pth',
    'vgg16': 'https://download.pytorch.org/models/vgg16-397923af.pth',
    'vgg19': 'https://download.pytorch.org/models/vgg19-dcbb9e9d.pth',
    'vgg11_bn': 'https://download.pytorch.org/models/vgg11_bn-6002323d.pth',
    'vgg13_bn': 'https://download.pytorch.org/models/vgg13_bn-abd245e5.pth',
    'vgg16_bn': 'https://download.pytorch.org/models/vgg16_bn-6c64b313.pth',
    'vgg19_bn': 'https://download.pytorch.org/models/vgg19_bn-c79401a0.pth',
}


class VGG(nn.Module):

    def __init__(self, features, num_classes=1000, init_weights=True):
        super(VGG, self).__init__()
        self.features = features
        self.classifier = nn.Sequential(
            nn.Linear(512 * 7 * 7, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(True),
            nn.Dropout(),
            nn.Linear(4096, num_classes),
        )
        if init_weights:
            self._initialize_weights()

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)


def make_layers(cfg, batch_norm=False):
    layers = []
    in_channels = 3
    for v in cfg:
        if v == 'M':
            layers += [nn.MaxPool2d(kernel_size=2, stride=2)]
        else:
            conv2d = nn.Conv2d(in_channels, v, kernel_size=3, padding=1)
            if batch_norm:
                layers += [conv2d, nn.BatchNorm2d(v), nn.ReLU(inplace=True)]
            else:
                layers += [conv2d, nn.ReLU(inplace=True)]
            in_channels = v
    return nn.Sequential(*layers)


cfg = {
    'A': [64, 'M', 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'B': [64, 64, 'M', 128, 128, 'M', 256, 256, 'M', 512, 512, 'M', 512, 512, 'M'],
    'D': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 'M', 512, 512, 512, 'M', 512, 512, 512, 'M'],
    'E': [64, 64, 'M', 128, 128, 'M', 256, 256, 256, 256, 'M', 512, 512, 512, 512, 'M', 512, 512, 512, 512, 'M'],
}

class VGG16(nn.Module):

    def __init__(self, pretrain=True):
        super().__init__()
        net = VGG(make_layers(cfg['D']), init_weights=False)
        if pretrain:
            net.load_state_dict(model_zoo.load_url(model_urls['vgg16']))

        self.stage1 = nn.Sequential(*[net.features[layer] for layer in range(0, 5)])
        self.stage2 = nn.Sequential(*[net.features[layer] for layer in range(5, 10)])
        self.stage3 = nn.Sequential(*[net.features[layer] for layer in range(10, 17)])
        self.stage4 = nn.Sequential(*[net.features[layer] for layer in range(17, 24)])
        self.stage5 = nn.Sequential(*[net.features[layer] for layer in range(24, 31)])

        # self.attn1 = Self_Attn( 256, 'relu')
        # self.attn2 = Self_Attn( 512,  'relu')
    def forward(self, x):           # torch.Size([8, 3, 512, 512]) 
        C1 = self.stage1(x)         # torch.Size([8, 64, 256, 256]) 
        C2 = self.stage2(C1)        # torch.Size([8, 128, 128, 128])   
        C3 = self.stage3(C2)        # torch.Size([8, 256, 64, 64])
        # C3, p1 = self.attn1(C3)     
        C4 = self.stage4(C3)        # torch.Size([8, 512, 32, 32])
        # C4, p1 = self.attn2(C4)     
        C5 = self.stage5(C4)        # torch.Size([8, 512, 16, 16])

        # print (x.shape)
        # print (C1.shape, C2.shape, C3.shape, C4.shape, C5.shape)
        return C1, C2, C3, C4, C5

class SELayer(nn.Module):
    def __init__(self, channel, reduction=16):
        super(SELayer, self).__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channel, channel // reduction, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(channel // reduction, channel, bias=False),
            nn.Sigmoid()
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avg_pool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y.expand_as(x)

class VGG16_senet(nn.Module):

    def __init__(self, pretrain=True):
        super().__init__()
        net = VGG(make_layers(cfg['D']), init_weights=False)
        if pretrain:
            net.load_state_dict(model_zoo.load_url(model_urls['vgg16']))

        self.stage1 = nn.Sequential(*[net.features[layer] for layer in range(0, 5)])
        self.stage2 = nn.Sequential(*[net.features[layer] for layer in range(5, 10)])
        self.stage3 = nn.Sequential(*[net.features[layer] for layer in range(10, 17)])
        self.stage4 = nn.Sequential(*[net.features[layer] for layer in range(17, 24)])
        self.stage5 = nn.Sequential(*[net.features[layer] for layer in range(24, 31)])
        self.relu = nn.ReLU(inplace=True)
        self.se_1 = SELayer(64, 16)
        self.se_2 = SELayer(128, 16)
        self.se_3 = SELayer(256, 16)
        self.se_4 = SELayer(512, 16)
        self.se_5 = SELayer(512, 16)

    def forward(self, x):           # torch.Size([8, 3, 512, 512]) 

        C1 = self.stage1(x)         # torch.Size([8, 64, 256, 256]) 

        #### senet
        C1_se = self.se_1(C1)
        C1_se += C1
        C1 = C1_se
        C1 = self.relu(C1)
        ####   

        C2 = self.stage2(C1)        # torch.Size([8, 128, 128, 128]) 

        #### senet
        C2_se = self.se_2(C2)
        C2_se += C2
        C2 = C2_se
        C2 = self.relu(C2)
        ####         

        C3 = self.stage3(C2)        # torch.Size([8, 256, 64, 64])

        #### senet
        C3_se = self.se_3(C3)
        C3_se += C3
        C3 = C3_se
        C3 = self.relu(C3)
        ####
        
        C4 = self.stage4(C3)        # torch.Size([8, 512, 32, 32])

        #### senet
        C4_se = self.se_4(C4)
        C4_se += C4
        C4 = C4_se
        C4 = self.relu(C4)
        ####

        C5 = self.stage5(C4)        # torch.Size([8, 512, 16, 16])

        #### senet
        C5_se = self.se_5(C5)
        C5_se += C5
        C5 = C5_se
        C5 = self.relu(C5)
        ####

        # print (x.shape)
        # print (C1.shape, C2.shape, C3.shape, C4.shape, C5.shape)
        return C1, C2, C3, C4, C5

if __name__ == '__main__':
    import torch
    input = torch.randn((4, 3, 512, 512))
    net = VGG16()
    C1, C2, C3, C4, C5 = net(input)
    print(C1.size())
    print(C2.size())
    print(C3.size())
    print(C4.size())
    print(C5.size())
