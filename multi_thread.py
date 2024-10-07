
'''
该代码跑在机器鱼的小电脑，读取双目视觉数据、侧线感知数据并对数据进行处理，通过串口上传辨识后 的威胁相对位置
'''

import threading
import time
import cv2
import serctl
import rflink
import numpy as np
import torch
from Queue_def import FixedLengthQueue
import pandas as pd
from scipy.interpolate import BSpline
from datetime import datetime
import scipy.signal as ss

data_dr = './dataset0601/'
head_dr = './dataset/'
pos_dr = './pos_dataset/'
press_dr = './press_dataset/'
imu_dr = './imu_dataset/'
video_dr = './video_data/'
pos_detec = '../pos_data.txt'#存储内容为angle+posx+posy
pos_pre_detec = '../pospre_data.txt'
save_step = 10
pos_coll_f = True




class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        self.running = False
        self.camera_l = cv2.VideoCapture(0)  # 初始化相机
        self.camera_2 = cv2.VideoCapture(1)  # 初始化相机
        self.frame_width = int(self.camera_l.get(3))
        self.frame_height = int(self.camera_l.get(4))
        self.frame_rate = int(self.camera_l.get(5))
        self.timestr = None
        self.red_coords = np.array([0,0])
        self.max_area = 0



    def run(self):
        self.running = True
        self.came_left_name = video_dr+'came_left'+self.timestr+'.avi'
        self.came_right_name = video_dr+'came_right'+self.timestr+'.avi'
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out_left = cv2.VideoWriter(self.came_left_name, self.fourcc, 20, (self.frame_width, self.frame_height))
        self.out_right = cv2.VideoWriter(self.came_right_name, self.fourcc, 20, (self.frame_width, self.frame_height))
        while self.running:
            _, left_image = self.camera_l.read()  # 从相机捕获图像
            _, right_image = self.camera_2.read()  # 从相机捕获图像
            combined_image = cv2.hconcat([left_image, right_image])
            self.red_coords,self.max_area = self.find_largest_red_area(combined_image,area_threshold=500)
            #print(self.red_coords,self.max_area)
            cv2.circle(combined_image, (self.red_coords[0], self.red_coords[1]), 20, (255,0,0), -1)
            cv2.imshow('Video', combined_image)
            self.out_left.write(left_image)
            self.out_right.write(right_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def get_red_coords_from_camera(self,combined_image):

        # 在这里添加处理图像并获取红色标记坐标的代码
        area_threshold = 1000  # 设置面积阈值
        # 将两个相机的图像按照水平方向拼接

        red_coords = self.find_largest_red_area(combined_image, area_threshold)

        return red_coords

    def find_largest_red_area(self,image, area_threshold):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_area_center = np.array([0,0])

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold and area > max_area:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    max_area_center = np.array([cX,cY])
                    max_area = area

        return max_area_center,max_area

    def stop(self):
        self.running = False
        self.camera_l.release()  # 释放相机资源
        self.camera_2.release()  # 释放相机资源
        self.out_left.release()
        self.out_right.release()

    def read_data(self):
        #将最大区域的面积除以100一并上传给上位机
        return np.array([self.red_coords[0],self.red_coords[1],self.max_area])

class SerialThread(threading.Thread):
    def __init__(self):
        super(SerialThread, self).__init__()
        self.running = False

        self.recv_sertool = None

        self.rftool = rflink.RFLink()

        self.buff_length = 50
        self.press_length = 6
        self.left_press_set = FixedLengthQueue(self.buff_length, self.press_length)
        self.right_press_set = FixedLengthQueue(self.buff_length, self.press_length)
        self.imu_set =  FixedLengthQueue(self.buff_length, 1)

        self.save_num = 300
        self.press_set = np.zeros((1,6))
        self.preess_res_set = np.zeros((1,2))
        self.timestr = None

        #高通滤波器相关的参数
        # 设计高通Butterworth滤波器
        self.cutoff_frequency = 2  # 截止频率
        self.order = 4  # 滤波器阶数
        self.b, self.a = ss.butter(self.order, self.cutoff_frequency, 'high', analog=False, fs=50)



#后面所有设计数组的需要重新判断维度问题
    def run(self):
        self.running = True
        self.left_response = np.zeros([2])
        self.right_response = np.zeros([2])
        self.left_done = False
        #测试版本中没有使用右侧侧线，所以直接置为True，并且返回无响应和响应值为0的状态
        self.right_done = True
        left_cnt = 0
        right_cnt = 0
        self.pres_name = press_dr+'press'+self.timestr+'.txt'
        self.pres_res_name = press_dr+'press_res'+self.timestr+'.txt'
        self.curent_cnt = 0
        self.all_data_num = 0
        while self.running:
            rx_data = self.recv_sertool.read_data()
            # 数据送入状态机，需要重写状态机器
            if self.rftool.RFLink_receivedata(rx_data): #
                self.all_data_num+=1
            # if(1):
                #  如果返回True,那么通知数据分析线程
                #new_imu_data = self.rftool.data_int_pack[0:2]
                #new_imu_data = np.array([1])
                #self.imu_set.push(new_imu_data)

                if(1):#如果返回的是左侧侧线数据，根据返回数据的标志位判断
                    new_pressure_data = self.rftool.data_int_pack[2:]
                    self.press_set = np.vstack((self.press_set,new_pressure_data))
                    #new_pressure_data = np.array([1,2,3,4,5,6])
                    self.left_press_set.push(new_pressure_data)
                    left_cnt+=1
                    if self.all_data_num>50 and left_cnt>10:
                        left_cnt = 0
                        eval_set = np.array(self.left_press_set.get_array())
                        for i in range(6):
                            eval_set[:,i] = self.signal_filter(eval_set[:,i],window_size=5,threshold=500)#滤除收发信号导致的奇异值
                        self.left_response = self.find_max_response(eval_set)
                        self.preess_res_set = np.vstack((self.preess_res_set,self.left_response))
                        #表示完成了数据的采集和一次判断
                        self.left_done = True
                #过程中保存侧线的数据
                self.curent_cnt+=1
                if self.curent_cnt>self.save_num:
                    np.savetxt(self.pres_name, self.press_set[1:,:],fmt="%.6f", delimiter=",")
                    np.savetxt(self.pres_res_name, self.press_set[1:,:],fmt="%.6f", delimiter=",")
                    self.curent_cnt=0
#直接去除静态误差的操作有点过于粗暴，可能需要高通滤波器进行滤波
    def find_max_response(self,eval_set):
        #去除静态偏移值
        subset_amp = np.abs(eval_set-np.mean(eval_set, axis=0, keepdims=True))
        #计算每个传感器对应的幅值均值
        current_means = np.mean(subset_amp[-10:], axis=0)
        #将均值展平，返回幅值最大的传感器索引
        #current_means = temp.reshape(1,-1)
        max_press_index = np.argmax(current_means)
        #返回最大响应索引和最大响应值
        return np.array([max_press_index,current_means[max_press_index]])
    def signal_filter(self,data, window_size, threshold):
        filtered_data = data.copy()
        n = len(data)

        #再进行种植滤波去除错误接收导致的异常值
        #进一步处理前面没能被处理的数据
        for i in range(window_size+1):
            window = filtered_data[i :i+ window_size]
            median = sorted(window)[window_size // 2]
            if abs(filtered_data[i] - median) > threshold:
                filtered_data[i] = median
        for i in range(window_size, n):
            window = filtered_data[i - window_size:i]
            median = sorted(window)[window_size // 2]
            if abs(filtered_data[i] - median) > threshold:
                filtered_data[i] = median
        #先进行高通滤波去除低频噪声
        #filtered_data =ss.filtfilt(self.b, self.a, filtered_data)

        return filtered_data

    def read_serial_data(self):
        #整理好后的返回值包含三个数，最大响应的边（1左，2右边），对应边的传感器位置和幅值量
        if self.left_done and self.right_done:
            self.left_done = False
            #self.right_done = False
            #这里需要根据实际情况判断多大的阈值可以被视为威胁的扰动
            if max(self.left_response[1],self.right_response[1])>50:
                if self.left_response[1]>self.right_response[1]:
                    #将响应值除以10一并上传给上位机
                    return np.array([self.left_response[0]+1,self.left_response[1]])
                else:
                    return np.array([self.right_response[0]+1+6,self.right_response[1]])
            else:
                return np.array([0,0])
        else:
            return np.array([0,0])

def recover_resample(key_times,key_points,interval):
    # 将数据转换为DataFrame
    data = pd.DataFrame({'timestamp': key_times, 'data': key_points})

    # 将时间戳设置为索引
    data.set_index('timestamp', inplace=True)

    # 设置重采样的目标时间间隔
    target_interval_minutes = interval  # 重采样时间间隔，可以根据需要调整

    # 使用重采样方法进行重采样
    #resampled_data = data.resample(f'{target_interval_minutes}T').asfreq()

def recover_Bspline(key_times,key_points,interval):
    # 将数据转换为DataFrame
    data = pd.DataFrame({'timestamp': key_times, 'data': key_points})

    # 将时间戳设置为索引
    data.set_index('timestamp', inplace=True)

    # 设置重采样的目标时间间隔
    target_interval_minutes = interval  # 重采样时间间隔，可以根据需要调整

    # 创建B样条拟合对象
    k = 2  # 拟合阶数
    t = data.index.values
    spl = BSpline(t, data['data'], k)

    # 创建重采样的目标时间范围
    start_time = min(t)
    end_time = max(t)
    resampled_relative_timestamps = np.arange(start_time, end_time, target_interval_minutes)

    # 在拟合曲线上进行重采样
    resampled_data_points = spl(resampled_relative_timestamps)

    # 创建重采样数据的DataFrame
    resampled_data = pd.DataFrame({'timestamp': resampled_relative_timestamps, 'data': resampled_data_points})
    resampled_data.set_index('timestamp', inplace=True)
    # 绘制原始数据点和拟合曲线
    return resampled_data_points

def main():
    recv_ser = "COM55"
    recv_baud = 19200
    snd_ser = "COM39"
    snd_baud = 19200


    send_sertool = serctl.RobotSerial()
    send_sertool.close_serial()
    send_sertool.init_serial(snd_ser, snd_baud)

    rec_sertool = serctl.RobotSerial()
    rec_sertool.close_serial()
    rec_sertool.init_serial(recv_ser, recv_baud)

    now = datetime.now()  # 获得当前时间
    timestr = now.strftime("%m%d%H%M%S")

    camera_thread = CameraThread()
    serial_thread = SerialThread()
    serial_thread.recv_sertool = rec_sertool

    serial_thread.timestr = timestr
    camera_thread.timestr = timestr

    camera_thread.start()
    serial_thread.start()

    rftool = rflink.RFLink()


    try:
        while True:
            time.sleep(0.2)
            #读取串口子线程和相机子线程中的数据
            press_res = serial_thread.read_serial_data()#触发的传感器编号、对应的响应值
            vision_res = camera_thread.read_data()#红色区域的中心位置（x,y）、对应的红色区域面积
            print(press_res,vision_res)

            data = int(press_res[0]).to_bytes(1, byteorder='big')#触发的传感器ID
            data += int(press_res[1]/10).to_bytes(1, byteorder='big')#触发传感器对应的响应值
            data += int(vision_res[0]/10).to_bytes(1, byteorder='big')#红色区域x坐标除以10
            data += int(vision_res[1]/10).to_bytes(1, byteorder='big')#红色区域y坐标除以10
            data += int(vision_res[2]/100).to_bytes(1, byteorder='big')#红色区域面积除以100

            snd_pack = rftool.RFLink_packdata(rflink.Command.SET_LPECT_OFFSET.value,data)

            try:
                send_sertool.write_cmd(snd_pack)
            except:
                print("send error")

    except KeyboardInterrupt:
        camera_thread.stop()
        serial_thread.running = False
        camera_thread.join()
        serial_thread.join()

if __name__ == "__main__":
    main()
