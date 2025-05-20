
'''
This code runs on the robotic fish's embedded computer, 
reads and processes stereo vision data and lateral line sensor data,
then transmits identified relative threat positions via serial port.
'''

import threading
import time
import cv2
import serctl
import rflink
import numpy as np
import torch
from Queue_def import FixedLengthQueue
from scipy.interpolate import BSpline
from datetime import datetime
import scipy.signal as ss

data_dr = './dataset0601/'
head_dr = './dataset/'
pos_dr = './pos_dataset/'
press_dr = './press_dataset/'
imu_dr = './imu_dataset/'
video_dr = './video_data/'
pos_detec = '../pos_data.txt'
pos_pre_detec = '../pospre_data.txt'
save_step = 10
pos_coll_f = True




class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        self.running = False
        self.camera_1 = cv2.VideoCapture(0)  # Initialize camera device
        self.camera_2 = cv2.VideoCapture(1)  # Initialize camera device
        self.frame_width = int(self.camera_1.get(3))
        self.frame_height = int(self.camera_1.get(4))
        self.frame_rate = int(self.camera_1.get(5))
        self.timestr = None
        self.red_coords = np.array([0,0])
        self.max_area = 0
        self.save_pos = np.zeros((1,3))
        self.save_num = 30
        self.cur_cont = 0



    def run(self):
        self.running = True
        self.came_left_name = video_dr+'came_left'+self.timestr+'.avi'
        self.came_right_name = video_dr+'came_right'+self.timestr+'.avi'
        self.save_pos_name = pos_dr+'vision_pos'+self.timestr+'.txt'
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out_left = cv2.VideoWriter(self.came_left_name, self.fourcc, 20, (self.frame_width, self.frame_height))
        self.out_right = cv2.VideoWriter(self.came_right_name, self.fourcc, 20, (self.frame_width, self.frame_height))
        while self.running:
            _, left_image = self.camera_2.read()  # Capture image frame from camera 
            _, right_image = self.camera_1.read()  # Capture image frame from camera 
            # print(left_image)
            black_image = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
            # Align and stitch dual-camera images horizontally
            combined_image = cv2.hconcat([left_image, right_image])
            self.red_coords,self.max_area = self.find_largest_red_area(combined_image,area_threshold=500)
            # hsv_image = cv2.cvtColor(combined_image, cv2.COLOR_BGR2HSV)
            # cv2.circle(hsv_image, (self.red_coords[0], self.red_coords[1]), 20, (255,0,0), -1)
            # cv2.imshow('Video', hsv_image)
            # print(self.red_coords,hsv_image[int(self.red_coords[1]),int(self.red_coords[0]),:])
            self.out_left.write(left_image)
            self.out_right.write(right_image)
            
            res = np.array([self.red_coords[0],self.red_coords[1],self.max_area])
            self.save_pos = np.vstack((self.save_pos,res))
            self.cur_cont+=1
            if self.cur_cont>self.save_num:
                np.savetxt(self.save_pos_name, self.save_pos[1:,:],fmt="%.6f", delimiter=",")
                self.cur_cont=0
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def get_red_coords_from_camera(self,combined_image):

        #  process image and extract red marker coordinates
        area_threshold = 1000  # Set area threshold

        red_coords = self.find_largest_red_area(combined_image, area_threshold)

        return red_coords

    def find_largest_red_area(self,image, area_threshold):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([4, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_red = np.array([177, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30,255,255])
        mask3 = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
        self.camera_1.release()  # Release camera resources
        self.camera_2.release()  # Release camera resources
        self.out_left.release()
        self.out_right.release()

    def read_data(self):
        # Upload the maximum region area (divided by 100) to the host computer
        return np.array([self.red_coords[0],self.red_coords[1],self.max_area])

class SerialThread(threading.Thread):
    def __init__(self):
        super(SerialThread, self).__init__()
        self.running = False

        self.recv_sertool = None

        self.rftool = rflink.RFLink()

        self.buff_length = 200
        self.press_length = 6
        self.left_press_set = FixedLengthQueue(self.buff_length, self.press_length)
        self.right_press_set = FixedLengthQueue(self.buff_length, self.press_length)
        self.imu_set =  FixedLengthQueue(self.buff_length, 1)

        self.save_num = 50
        self.press_set = np.zeros((1,6))
        self.preess_res_set = np.zeros((1,2))
        self.timestr = None

        # Configuration parameters for high-pass filtering
        # Butterworth high-pass filter design (order=4, cutoff=10Hz)
        self.cutoff_frequency = 2  # Cutoff frequency
        self.order = 4  # Filter order
        self.b, self.a = ss.butter(self.order, self.cutoff_frequency, 'high', analog=False, fs=50)



    def run(self):
        # Initialize running state and variables
        self.running = True
        self.left_response = np.zeros([2])  # Stores left side response data
        self.right_response = np.zeros([2])  # Stores right side response data
        self.left_done = False  # Flag indicating left side processing complete
        self.right_done = False  # Flag indicating right side processing complete
        left_cnt = 0  # Counter for left side data processing
        right_cnt = 0  # Counter for right side data processing
        
        # Set up file names for data recording
        self.pres_name = press_dr+'press_new_record'+self.timestr+'.txt'
        self.pres_res_name = press_dr+'press_res'+self.timestr+'.txt'
        
        self.curent_cnt = 0  # Counter for periodic data saving
        self.all_data_num = 0  # Total data counter
        
        while self.running:
            # Read incoming serial data
            rx_data = self.recv_sertool.read_data()
            # Process data through state machine (needs state machine rewrite)
            if self.rftool.RFLink_receivedata(rx_data):
                self.all_data_num += 1

                if(1):  # If valid data is received
                    new_pressure_data = self.rftool.data_int_pack[1:]
                    # Append new data to pressure dataset
                    self.press_set = np.vstack((self.press_set, new_pressure_data))
                    
                    # Split data into left and right channels (assuming data contains at least 12 values)
                    left_data = new_pressure_data[:6]    # First 6 values for left side
                    right_data = new_pressure_data[6:12]  # Next 6 values for right side
                    
                    # Push data to respective buffers
                    self.left_press_set.push(left_data)
                    self.right_press_set.push(right_data)
                    left_cnt += 1
                    right_cnt += 1
                    
                    # Process data after initial buffer fill
                    if self.all_data_num > 100:
                        # Process left side data
                        if left_cnt > 100:
                            left_cnt = 0
                            # Get left side data array
                            eval_set = np.array(self.left_press_set.get_array())
                            # Apply filtering to each channel
                            for i in range(6):
                                eval_set[:, i] = self.signal_filter(eval_set[:, i], 
                                                                smooth_window=20, 
                                                                median_window_size=5, 
                                                                threshold=500)
                            # Calculate response and store results
                            self.left_response = self.find_max_response(eval_set)
                            #self.preess_res_set = np.vstack((self.preess_res_set, self.left_response))
                            self.left_done = True  # Mark left processing complete
                        
                        # Process right side data (same logic as left side)
                        if right_cnt > 100:
                            right_cnt = 0
                            eval_set = np.array(self.right_press_set.get_array())
                            for i in range(6):
                                eval_set[:, i] = self.signal_filter(eval_set[:, i], 
                                                                smooth_window=20, 
                                                                median_window_size=5, 
                                                                threshold=500)
                            self.right_response = self.find_max_response(eval_set)
                            #self.preess_res_set = np.vstack((self.preess_res_set, self.right_response))
                            self.right_done = True  # Mark right processing complete

                # Periodically save the collected data
                self.curent_cnt += 1
                if self.curent_cnt > self.save_num:
                    np.savetxt(self.pres_name, self.press_set[1:,:], fmt="%.6f", delimiter=",")
                    self.curent_cnt = 0  # Reset save counter

    def find_max_response(self,eval_set):
        # Remove DC offset
        subset_amp = np.abs(eval_set-np.mean(eval_set, axis=0, keepdims=True))
        col_range = np.ptp(subset_amp[5:-5],axis=0)
        max_press_index = np.argmax(col_range)
        # Return the index and value of maximum response
        return np.array([max_press_index,col_range[max_press_index]])
    def signal_filter(self,data, smooth_window = 20,median_window_size=5, threshold=500):
        filtered_data = data.copy()
        n = len(data)

        # Apply median filtering to remove outliers caused by erroneous reception
        # Post-process the remaining unprocessed data
        for i in range(median_window_size+1):
            window = filtered_data[i :i+ median_window_size]
            median = sorted(window)[median_window_size // 2]
            if abs(filtered_data[i] - median) > threshold:
                filtered_data[i] = median
        for i in range(median_window_size, n):
            window = filtered_data[i - median_window_size:i]
            median = sorted(window)[median_window_size // 2]
            if abs(filtered_data[i] - median) > threshold:
                filtered_data[i] = median
        # First apply high-pass filtering to remove low-frequency noise
        #filtered_data =ss.filtfilt(self.b, self.a, filtered_data)
        extend_data = np.pad(filtered_data,(smooth_window//2,smooth_window//2),mode='edge')
        smooth_data = np.convolve(extend_data,np.ones(smooth_window)/smooth_window,mode='valid')
        # smooth_data = smooth_data-np.mean(smooth_data)
        return smooth_data[0:-1]

    def read_serial_data(self):
        """
        The processed return value contains three parameters:
        1. Peak response side (1:left, 2:right)
        2. Sensor position of the corresponding side 
        3. Amplitude value
        """
        if self.left_done and self.right_done:
            self.left_done = False
            self.right_done = False
            # The threshold for identifying threat disturbances should be determined based on empirical data
            if max(self.left_response[1],self.right_response[1])>5:
                if self.left_response[1]>self.right_response[1]:

                    return np.array([self.left_response[0]+1,self.left_response[1]])
                else:
                    return np.array([self.right_response[0]+1+6,self.right_response[1]])
            else:
                return np.array([0,0])
        else:
            return np.array([0,0])
def main():
    recv_ser = '/dev/ttyUSB1'
    recv_baud = 9600
    snd_ser = '/dev/ttyUSB0'
    snd_baud = 19200

    send_sertool = serctl.RobotSerial()
    send_sertool.close_serial()
    send_sertool.init_serial(snd_ser, snd_baud)

    rec_sertool = serctl.RobotSerial()
    rec_sertool.close_serial()
    rec_sertool.init_serial(recv_ser, recv_baud)

    now = datetime.now()  # Get current timestamp
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
            # Read data from serial sub-thread and camera sub-thread
            press_res = serial_thread.read_serial_data()  # [sensor_number, response_value]
            vision_res = camera_thread.read_data()  # [red_region_center_x, center_y, area]
            print(press_res, vision_res)

            data = int(press_res[0]).to_bytes(1, byteorder='big')
            data += int(min(press_res[1], 255)).to_bytes(1, byteorder='big')  # Response value from triggered sensor
            data += int(vision_res[0]/10).to_bytes(1, byteorder='big')  # Red region x-coordinate divided by 10
            data += int(vision_res[1]/10).to_bytes(1, byteorder='big')  # Red region y-coordinate divided by 10
            data += int(min(vision_res[2]/100, 255)).to_bytes(1, byteorder='big')  # Red region area divided by 100

            snd_pack = rftool.RFLink_packdata(rflink.Command.SET_LPECT_OFFSET.value, data)

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
