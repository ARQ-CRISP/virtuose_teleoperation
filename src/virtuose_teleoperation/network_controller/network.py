import torch
from torch import nn
import numpy as np
import torch.nn.functional as F



# class ResidualBlock(nn.Module):
#     """
#     A residual block with dropout option
#     """

#     def __init__(self, in_channels, out_channels, kernel_size=3):
#         super(ResidualBlock, self).__init__()
#         self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size, padding=1)
#         self.bn1 = nn.BatchNorm2d(out_channels)
#         self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size, padding=1)
#         self.bn2 = nn.BatchNorm2d(out_channels)

#     def forward(self, x_in):
#         x = self.bn1(self.conv1(x_in))
#         x = F.relu(x)
#         x = self.bn2(self.conv2(x))
#         return x + x_in

class ResidualBlock(nn.Module):
    def __init__(self, input_features):
        super(ResidualBlock, self).__init__()
        
        self.fc = nn.Linear(input_features, input_features)
        self.bn = nn.BatchNorm1d(input_features)
        self.lrelu = nn.LeakyReLU(negative_slope=0.01)
    
    def forward(self, x):

        out = self.fc(x)
        out = self.bn(out)
        out = self.lrelu(out)

        return out + x


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

class BC_MLP_AHEE_yes_norm_ouput_hglove_ee(nn.Module):

    "BC with mlp structure"
    def __init__(self, input_channels=27, output_channels=12) -> None:
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
        x = self.relu(x)
        x = self.lin5(x)

        return x  
    

class Autoencoder(nn.Module):
    def __init__(self, input_size=144*3+20*3, bottleneck_size=256):
        super(Autoencoder, self).__init__()
        self.input_size = input_size
        # Encoder
        self.encoder = nn.Sequential(
            nn.Linear(input_size, 1024),
            # nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(1024, 512),
            # nn.BatchNorm1d(512),
            nn.LeakyReLU(negative_slope=0.01),
            # ResidualBlock(512),
            nn.Linear(512, 512),
            # nn.BatchNorm1d(512),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(512, bottleneck_size),
            # nn.BatchNorm1d(bottleneck_size),
            nn.LeakyReLU(negative_slope=0.01),

            # nn.Linear(256, 256),
            # nn.BatchNorm1d(256),
            # nn.LeakyReLU(negative_slope=0.01),
            # nn.Linear(256, 256),
            # nn.BatchNorm1d(256),
            # nn.LeakyReLU(negative_slope=0.01),
            # nn.Linear(256, 128),
            # nn.BatchNorm1d(128),
            # nn.LeakyReLU(negative_slope=0.01),
            # ResidualBlock(128),
            # nn.Linear(128, bottleneck_size),  # Additional layers to reach 16 linear layers
            # nn.BatchNorm1d(bottleneck_size),
            # nn.LeakyReLU(negative_slope=0.01),
            # ... (Continue adding layers until you have 16 linear layers in total)
        )
        self.res1 = ResidualBlock(bottleneck_size)
        self.res2 = ResidualBlock(bottleneck_size)
        # Decoder
        self.decoder = nn.Sequential(
            # nn.Linear(bottleneck_size, 128),
            # nn.BatchNorm1d(128),
            # nn.LeakyReLU(negative_slope=0.01),
            # ResidualBlock(128),
            # nn.Linear(128, 256),
            # nn.BatchNorm1d(256),
            # nn.LeakyReLU(negative_slope=0.01),
            # nn.Linear(256, 256),
            # nn.BatchNorm1d(256),
            # nn.LeakyReLU(negative_slope=0.01),
            # nn.Linear(256, 256),
            # nn.BatchNorm1d(256),
            # nn.LeakyReLU(negative_slope=0.01),
            # ResidualBlock(256),
            nn.Linear(bottleneck_size, 512),
            # nn.BatchNorm1d(512),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(512, 512),
            # nn.BatchNorm1d(512),
            nn.LeakyReLU(negative_slope=0.01),
            # ResidualBlock(512),
            nn.Linear(512, 1024),
            # nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.01),
            nn.Linear(1024, input_size)
            # ... (Continue adding layers until you have 16 linear layers in total)
        )

    def forward(self, x):
        # x = x.view(x.size(0), -1)  # Flatten the input
        x = self.encoder(x)
        x = self.res1(x)
        x = self.res2(x)
        x = self.decoder(x)
        # x = x.view(-1, 1, self.input_size)  # Reshape back to original input shape
        return x
    def get_latent_vector(self, x):
        # x = x.view(x.size(0), -1)
        x = self.encoder(x)
        return x


class CustomNetwork(nn.Module):
    def __init__(self, num_classes =7, key_frame_output_size = 7, input_size=256):
        super(CustomNetwork, self).__init__()


        # LSTM Layers
        # self.lstm1 = nn.LSTM(input_size=23, hidden_size=128, batch_first=True)
        # self.lstm2 = nn.LSTM(input_size=128, hidden_size=256, batch_first=True)
        
        # Fully Connected Layers
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 256)
        self.fc3 = nn.Linear(256, 64)
        
        # Output Layers
        self.classifier_output = nn.Linear(64, num_classes)
        self.contact_indicator_output = nn.Linear(64, 1)
        self.key_frame_output = nn.Linear(64, key_frame_output_size)

    def forward(self, x):
        # x shape: (batch_size, 100, 20)

        # LSTM Layers
        # x, _ = self.lstm1(x)
        # x, _ = self.lstm2(x)

        # # Selecting the last time step output for classification
        # x = x[:, -1, :]

        # Fully Connected Layers
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))

        # Output Layers
        # classifier_out = F.softmax(self.classifier_output(x), dim=1)
        classifier_out = self.classifier_output(x)
        contact_indicator_out = torch.sigmoid(self.contact_indicator_output(x))
        key_frame_out = self.key_frame_output(x)  # Apply softmax/relu/linear based on your requirement

        return classifier_out, contact_indicator_out, key_frame_out