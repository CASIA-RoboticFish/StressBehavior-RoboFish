'''
2023-5-10
该版本数据采集程序适用于上电自启动的侧线采集下位机
上传数据的内容为
FF 11 44 LEN IMU_acc IMU-YAW IMU-ROLL ALL_pressure FC

开始运行后按下‘s’ 开始采集
按下‘q’停止采集
rue
如果需要同步记录相对位置数据将pos_coll_f标志位修改为True
'''

import serctl
import rflink
import numpy as np
import keyboard
import time
import matplotlib.pyplot as plt
from datetime import datetime
# 串口类

cmd_sertool = serctl.RobotSerial()
recv_sertool = serctl.RobotSerial()

# rf通讯协议类
rftool = rflink.RFLink()

data_dr = './dataset0601/'
head_dr = './dataset/'
pos_dr = './pos_dataset/'
press_dr = './press_dataset/'
imu_dr = './imu_dataset/'
pos_detec = '../pos_data.txt'#存储内容为angle+posx+posy
pos_pre_detec = '../pospre_data.txt'
save_step = 10
pos_coll_f = True

if __name__ == '__main__':
    #cmd_sertool.init_serial("COM27",9600)
    dataset_num = 0
    while 1:
        key = input("enter s for start")
        recv_sertool.close_serial()
        recv_sertool.init_serial("COM15", 19200)
        count = 0
        press_set = np.zeros((1,6))
        pos_set = np.zeros((1,3))#angle+posx+posy
        pos = np.zeros((1,3))
        pos_pre = pos
        imu_set = []
        if key =="l":
            start = time.time()
            while 1:
                # 接收数据
                rx_data = recv_sertool.read_data()

                # 数据送入状态机，需要重写状态机器
                if rftool.RFLink_receivedata(rx_data): #
                    #  如果返回True,那么通知数据分析线程
                    imu_data = rftool.data_int_pack[0:2]
                    imu_set.append(imu_data)
                    pressure_data = rftool.data_int_pack[2:]
                    press_set = np.vstack((press_set,pressure_data))
                    count+=1
                if count%20==0:
                    print("\r"+"采集中... 采集数量: ",count,end="")
                if keyboard.is_pressed('p'):
                    break
            end = time.time()
            f = count/(end-start)
            press_set.astype('int16')
            now = datetime.now()  # 获得当前时间
            timestr = now.strftime("%m%d%H%M%S")
            #pres_name = press_dr+'press'+'%04d' % dataset_num+'.txt'
            pres_name = press_dr+'press'+timestr+'.txt'
            np.savetxt(pres_name, press_set[1:,:], delimiter=",")
            imu_name = imu_dr+'imu'+timestr+'.txt'
            np.savetxt(imu_name, imu_set, delimiter=",")

            # dataset_num += 1
            # print("\n采集结束...采集时间：",end-start,"共采集",count,"存储序列：",timestr,"平均采集频率",f,"Hz")
            # plt.figure(1)
            # plt.plot(press_set[1:])
            # plt.figure(2)
            # plt.plot(pos_set)
            # plt.show()














