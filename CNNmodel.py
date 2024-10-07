import torch.nn as nn
import torch.nn.functional as F


#当前谱图fre=20 t = 2s window = 6 普图尺寸24*15

class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Sequential(  # input shape (6, 21, 11)
            nn.Conv2d(
                in_channels=6,      # input height
                out_channels=12,    # n_filters
                kernel_size=2,      # filter size
                stride=1,           # filter movement/step
                #padding=0,      # 如果想要 con2d 出来的图片长宽没有变化, padding=(kernel_size-1)/2 当 stride=1
            ),      # output shape (12, 19, 19)
            nn.ReLU(),    # activation
            nn.MaxPool2d(kernel_size=2),    # 在 2x2 空间里向下采样, output shape (12, 18, 8)
        )
        self.conv2 = nn.Sequential(  # input shape (12, 18, 8)
            nn.Conv2d(
                in_channels=12,      # input height
                out_channels=24,    # n_filters
                kernel_size=2,      # filter size
                stride=1,           # filter movement/step
                #padding=1,      # 如果想要 con2d 出来的图片长宽没有变化, padding=(kernel_size-1)/2 当 stride=1
            ),  # output shape (24, 16, 6)
            nn.ReLU(),  # activation
            nn.MaxPool2d(2),  # output shape (24, 15, 5)
        )
        self.out1 = nn.Linear(24, 32)   # fully connected layer
        self.out2 = nn.Linear(32, 4)   # fully connected layer

    def forward(self, x):
        x = self.conv1(x)
        x = self.conv2(x)
        x = x.view(x.size(0), -1)   # 展平多维的卷积图成 (batch_size, 24 * 15 * 5)
        x = F.relu(self.out1(x))
        output = self.out2(x)
        return output