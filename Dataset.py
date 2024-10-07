'''
数据采集后将STFT获得视频谱图存储在txt文件
本代码实现将txt中文件整理为可供CNN训练用的数据集格式

'''

import os
import cv2
import numpy as np
from torch.utils.data import Dataset,DataLoader
import torchvision.transforms as transforms
import FTtransfer

# 训练集,先只实现一个训练集
class TrainDataset(Dataset):  # 继承Dataset
    def __init__(self, path):  # 制作一个list，将图片路径以及标签信息存储在一个txt中
        # 读取数据，数据中第一行就是label，没有单独的label文件
        self.data_path = os.listdir(path)
        #暂时不考虑数据的变换，有需要在这里加上
        self.resize = (30,30)
        self.transform = transforms.Compose([
                transforms.ToTensor(),

            ])
        self.data_list = []
        for i in range(len(self.data_path)):
            self.data_list.append(os.path.join(path, self.data_path[i]))

    def __getitem__(self, index):  # 读取数据和标签，并返回数据和标签
        # 读取数据
        self.samp_path = self.data_list[index]
        # 数据读入
        raw_data = np.loadtxt(self.samp_path)
        label = (raw_data[0,0:4]*100).astype(np.float32)
        data = raw_data[1:,:]
        h = np.shape(data)[0]
        w = np.shape(data)[1]
        spec = data.reshape((6,int(h/6),w)).astype(np.float32)
        #print(np.shape(spec))
        if self.transform is not None:
            spec = self.transform(spec).permute(1,2,0)
            #print(np.shape(spec))
        return spec, label  # 返回索引

    def __len__(self):  # 必须写，返回数据集的长度
        return len(self.data_list)

'''
该函数将采集到的原始压力数据进行STFT变换，获得能被数据集整理函数处理的形式
fre:传感器采集频率
colect_step：采集过程中每隔多少个数据存一次位置数据
samp_step:将多少次位置数据之间的压力数据进行变换处理作为一个数据集
win:STFT变换时的窗口宽度
默认，每5次采集一次位置数据，采集频率为20Hz，将两秒作为数据集单位就需要sampstep为8
'''
def dataset_STFT(pos_dir,press_dir,save_dir,fre=20,samp_step = 8,colect_step = 5,win = 5):
    STFTtransfer = FTtransfer.FTtransfer()
    posdataP_list = os.listdir(pos_dir)
    presdataP_list = os.listdir(press_dir)
    if len(posdataP_list)!=len(presdataP_list):
        print("direrror")
        return 0

    posdata_list = []
    presdata_list = []
    for i in range(len(posdataP_list)):
        posdata_list.append(os.path.join(pos_dir, posdataP_list[i]))
        presdata_list.append(os.path.join(press_dir, presdataP_list[i]))


    test_buff = np.ones([1,fre*2])
    testfre, ts, amp = STFTtransfer.STFT(x=test_buff,fs=fre,n=win)
    t_span = np.size(ts)
    save_index = 0
    for i in range(len(posdata_list)):
    #for i in range(1):
        pos_path = posdata_list[i]
        press_path = presdata_list[i]
        raw_pos = np.loadtxt(pos_path, delimiter=',', dtype=float)
        raw_press = np.loadtxt(press_path, delimiter=',', dtype=float)
        index = 0
        while(index+8<len(raw_pos)):
            pos1 = raw_pos[index]
            pos2 = raw_pos[index+samp_step]
            press = raw_press[index*colect_step:(index+8)*colect_step,:]
            STFT_buff = np.zeros((1,t_span))
            STFT_buff[0,0:4] = np.hstack((pos1[0:2],pos2[0:2]))
            for i in range(6):
                f, t, amp = STFTtransfer.STFT(x=press[:,i],fs=fre,n = 5)
                STFT_buff = np.vstack((STFT_buff,amp))
            save_path = save_dir+'/%04d' % save_index+'.txt'
            np.savetxt(save_path,STFT_buff,delimiter=",")
            save_index+=1
            print(save_index)

if __name__ == '__main__':
    #main函数仅作测试用
    # STFT_path = './train_data'
    # train_data = TrainDataset(path=STFT_path)
    # data_loader= DataLoader(dataset=train_data, batch_size=1, shuffle=True)
    # spec_batch, label_batch = train_data[100]
    # print(np.shape(spec_batch))
    dataset_STFT(pos_dir="./pos_dataset",press_dir="./press_dataset",save_dir="./STFT_dataset")

