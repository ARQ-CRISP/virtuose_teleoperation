import torch
from torch import nn
import numpy as np
import torch.nn.functional as F



class ResidualBlock(nn.Module):
    """
    A residual block with dropout option
    """

    def __init__(self, in_channels, out_channels, kernel_size=3):
        super(ResidualBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size, padding=1)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size, padding=1)
        self.bn2 = nn.BatchNorm2d(out_channels)

    def forward(self, x_in):
        x = self.bn1(self.conv1(x_in))
        x = F.relu(x)
        x = self.bn2(self.conv2(x))
        return x + x_in



class AE(nn.Module):
    def __init__(self, input_channels=1) -> None:
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, 16, kernel_size=3, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(16)

        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(32)

        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.bn3 = nn.BatchNorm2d(64)

        self.conv4 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        self.bn4 = nn.BatchNorm2d(128)

        self.res1 = ResidualBlock(128, 128)
        self.res2 = ResidualBlock(128, 128)

        self.conv5 = nn.ConvTranspose2d(128, 64, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn5 = nn.BatchNorm2d(64)

        self.conv6 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn6 = nn.BatchNorm2d(32)

        self.conv7 = nn.ConvTranspose2d(32, 16, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn7 = nn.BatchNorm2d(16)

        self.conv8 = nn.ConvTranspose2d(16, 1, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
    
class AE_modify(nn.Module):
    def __init__(self, input_channels=1) -> None:
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, 16, kernel_size=3, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(16)

        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(32)

        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.bn3 = nn.BatchNorm2d(64)

        self.conv4 = nn.Conv2d(64, 16, kernel_size=3, stride=2, padding=1)
        self.bn4 = nn.BatchNorm2d(16)

        self.res1 = ResidualBlock(16, 16)
        self.res2 = ResidualBlock(16, 16)

        self.conv5 = nn.ConvTranspose2d(16, 64, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn5 = nn.BatchNorm2d(64)

        self.conv6 = nn.ConvTranspose2d(64, 32, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn6 = nn.BatchNorm2d(32)

        self.conv7 = nn.ConvTranspose2d(32, 16, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)
        self.bn7 = nn.BatchNorm2d(16)

        self.conv8 = nn.ConvTranspose2d(16, 1, kernel_size=3, stride=2, padding=1,
                                        output_padding=1)

    def forward(self, x):
        x = self.bn1(self.conv1(x))
        x = self.bn2(self.conv2(x))
        x = self.bn3(self.conv3(x))
        x = self.bn4(self.conv4(x))

        x = self.res1(x)
        x = self.res2(x)

        x = self.bn5(self.conv5(x))
        x = self.bn6(self.conv6(x))
        x = self.bn7(self.conv7(x))
        x = self.conv8(x)
        x = torch.sigmoid(x)

        return x

    def get_latent_vector(self, x):
        x = self.bn1(self.conv1(x))
        x = self.bn2(self.conv2(x))
        x = self.bn3(self.conv3(x))
        x = self.bn4(self.conv4(x))

        x = self.res1(x)
        x = self.res2(x)        
        
        return x

class behaviourCloning(nn.Module):

    "BC with cov structure"
    def __init__(self, input_channels=26) -> None: 
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, 16, kernel_size=3, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 =nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 =nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(64)
        self.conv4 =nn.Conv2d(64, 16, kernel_size=3, stride=1, padding=1)
        self.bn4 = nn.BatchNorm2d(16)
        self.conv5 =nn.Conv2d(16, 1, kernel_size=3, stride=1, padding=1)

        self.flatten = nn.Flatten()
        self.relu = nn.ReLU()
        self.lin1 = nn.Linear(8*8, 32)
        self.lin2 = nn.Linear(32, 12)
        
        
    def forward(self, x):
        x = self.bn1(self.conv1(x))
        x = self.bn2(self.conv2(x))
        x = self.bn3(self.conv3(x))
        x = self.bn4(self.conv4(x))

        x = self.conv5(x)
        x = self.relu(self.flatten(x))

        x = self.lin1(x)
        x= self.relu(x)
        x = self.lin2(x)
        
        return x

class BC_MLP(nn.Module):

    "BC with mlp structure"
    def __init__(self, input_channels=103, output_channels=19) -> None: 
        super().__init__()
        self.relu = nn.ReLU()
        self.lin1 = nn.Linear(input_channels, 256)
        self.lin2 = nn.Linear(256, 256)
        self.lin3 = nn.Linear(256, 128)
        self.lin4 = nn.Linear(128, 64)
        self.lin5 = nn.Linear(64, output_channels)
        
        
    def forward(self, x):
        x = self.lin1(x)
        x = self.relu(x)
        x = self.lin2(x)
        x = self.relu(x)
        x = self.lin3(x)
        x = self.relu(x)
        x = self.lin4(x)
        x = self.lin5(x)

        return x

class BC_COV_MLP(nn.Module):

    "BC with mlp structure"
    def __init__(self, input_channels=12, output_channels=19) -> None: 
        super().__init__()
        self.conv1 = nn.Conv2d(input_channels, 16, kernel_size=3, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(16)
        self.relu = nn.ReLU()
        self.lin1 = nn.Linear(input_channels, 256)
        self.lin2 = nn.Linear(256, 256)
        self.lin3 = nn.Linear(256, 128)
        self.lin4 = nn.Linear(128, 64)
        self.lin5 = nn.Linear(64, output_channels)
        
        
    def forward(self, x):
        x = self.lin1(x)
        x = self.relu(x)
        x = self.lin2(x)
        x = self.relu(x)
        x = self.lin3(x)
        x = self.relu(x)
        x = self.lin4(x)
        x = self.lin5(x)

        return x