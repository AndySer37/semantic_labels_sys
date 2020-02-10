import torch.nn as nn
import torch
import torch.nn.functional as F
from network.vgg import *
from .senet.se_resnet import *

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


class Upsample(nn.Module):

    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.conv1x1 = nn.Conv2d(in_channels, in_channels, kernel_size=1, stride=1, padding=0)
        self.conv3x3 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=1, padding=1)
        self.deconv = nn.ConvTranspose2d(out_channels, out_channels, kernel_size=4, stride=2, padding=1)

    def forward(self, upsampled, shortcut):
        x = torch.cat([upsampled, shortcut], dim=1)
        x = self.conv1x1(x)
        x = F.relu(x)
        x = self.conv3x3(x)
        x = F.relu(x)
        x = self.deconv(x)
        return x

class TextNet(nn.Module):

    def __init__(self, backbone='vgg', output_channel=7, is_training=True):
        super().__init__()

        self.is_training = is_training
        self.backbone_name = backbone
        self.output_channel = output_channel
        # self.attn1 = Self_Attn( 256, 'relu')
        # self.attn2 = Self_Attn( 128,  'relu')
        # self.attn3 = Self_Attn( 64,  'relu')
        # self.attn4 = Self_Attn( 32,  'relu')
        if backbone == 'vgg':
            self.backbone = VGG16(pretrain=self.is_training)
            self.deconv5 = nn.ConvTranspose2d(512, 256, kernel_size=4, stride=2, padding=1)
            self.merge4 = Upsample(512 + 256, 128)
            self.merge3 = Upsample(256 + 128, 64)
            self.merge2 = Upsample(128 + 64, 32)
            self.merge1 = Upsample(64 + 32, 16)
            self.predict = nn.Sequential(
                nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1),
                nn.Conv2d(16, self.output_channel, kernel_size=1, stride=1, padding=0)
            )
        elif backbone == 'resnet':
            pass
        elif backbone == 'senet':
            # self.backbone = CifarSEResNet(CifarSEBasicBlock, 3) 
            self.backbone = VGG16_senet(pretrain=self.is_training)
            self.deconv5 = nn.ConvTranspose2d(512, 256, kernel_size=4, stride=2, padding=1)
            self.merge4 = Upsample(512 + 256, 128)
            self.merge3 = Upsample(256 + 128, 64)
            self.merge2 = Upsample(128 + 64, 32)
            self.merge1 = Upsample(64 + 32, 16)
            self.predict = nn.Sequential(
                nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1),
                nn.Conv2d(16, self.output_channel, kernel_size=1, stride=1, padding=0)
            )            

    def forward(self, x):
        C1, C2, C3, C4, C5 = self.backbone(x)
        up5 = self.deconv5(C5)
        up5 = F.relu(up5)

        # up5 ,p1 = self.attn1(up5)

        up4 = self.merge4(C4, up5)
        up4 = F.relu(up4)

        # up4 ,p2 = self.attn2(up4)

        up3 = self.merge3(C3, up4)
        up3 = F.relu(up3)

        # up3 ,p3 = self.attn3(up3)

        up2 = self.merge2(C2, up3)
        up2 = F.relu(up2)

        # up2 ,p4 = self.attn4(up2)

        up1 = self.merge1(C1, up2)
        output = self.predict(up1)

        return output

    def load_model(self, model_path):
        print('Loading from {}'.format(model_path))
        state_dict = torch.load(model_path)
        self.load_state_dict(state_dict['model'])

if __name__ == '__main__':
    import torch

    input = torch.randn((4, 3, 512, 512))
    net = TextNet().cuda()
    output = net(input.cuda())
    print(output.size())

