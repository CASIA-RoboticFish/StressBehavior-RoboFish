import sys
import time
import numpy as np
import random
import math
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QLineEdit, QVBoxLayout, QWidget, QComboBox
from PyQt5 import QtCore,QtGui,QtWidgets
from PyQt5.QtCore import Qt, QThread, pyqtSignal
import rflink
import serctl
import struct
from datetime import datetime
import matplotlib.pyplot as plt
from enum import Enum
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
# Define the network for reconstructing stress actions
class Decode_LSTMModel(nn.Module):
    def __init__(self,hideen_num,layer_num):
        super(Decode_LSTMModel, self).__init__()
        self.lstm = nn.LSTM(input_size=1, hidden_size=hideen_num, num_layers=layer_num, batch_first=True)
        self.fc = nn.Linear(hideen_num, 30)

    def forward(self, x):
        out, _ = self.lstm(x)
        out = self.fc(out[:, -1, :])  # Get the output of the last time step of the LSTM
        return out

# Define the action transfer network
class Trans_LSTMModel(nn.Module):
    def __init__(self,input_dim,output_dim,hideen_num,layer_num):
        super(Trans_LSTMModel, self).__init__()
        self.lstm = nn.LSTM(input_size=input_dim, hidden_size=hideen_num, num_layers=layer_num, batch_first=True)
        self.fc = nn.Linear(hideen_num, output_dim)

    def forward(self, x):
        out, _ = self.lstm(x)
        out = self.fc(out[:, -1, :])  # Get the output of the last time step of the LSTM
        return out

# Define the basic representation of a piecewise function
def piecewise_linear(x, b0, b1, b2, b3,b4, b5, b6, b7,b8,b9):
    return np.piecewise(x, [x < 15, (x >= 15)&(x<30),(x >= 30)&(x<125),(x >= 125)&(x<165),x>=165],
                        [lambda x: b0 + b1 * x, lambda x: b2 + b3 * x,lambda x: b4 + b5 * x, lambda x: b6 + b7* x,lambda x: b8 + b9* x])



class SignalGenerator(QThread):
    signal_generated = pyqtSignal(float)

    def __init__(self, amplitude, frequency, bias, snd_port, snd_baud_rate,rev_port, rev_baud_rate,factor):
        super().__init__()
        self.amplitude = amplitude
        self.frequency = frequency
        self.bias = bias
        self.snd_port = snd_port
        self.snd_baud_rate = snd_baud_rate
        self.rev_port = rev_port
        self.rev_baud_rate = rev_baud_rate
        self.running = False
        self.send_sertool = None
        self.rev_sertool = None
        self.rftool = rflink.RFLink()
        self.save_data_dr = './dataset/'
        self.rev_data_dr = './rev_dataset/'
        self.pos_detec = './pos_data.txt'# Store content as red_cor, yellow_cor
        self.inverse_dyna_dataset = np.zeros((1,5)) #
        self.rev_dataset = np.zeros((1,5)) #press_id,press_amp,x,y,area
        self.action_set = [0]
        self.pos = np.zeros((4))#color_point:[self.red_point_img_pos_x,self.red_point_img_pos_y,self.yellow_point_img_pos_x,self.yellow_point_img_pos_y]
        self.pos_pre = self.pos

        self.single_amp = 0
        self.single_factor = 0.5
        self.single_step = 20
        self.vision_flag = 0

        self.factor = factor

        # State variables for the robotic fish's stress operation
        # Recstate enumeration type
        self.ACTION_STATE = Enum('ACTION_STATE', ( \
            'WATTING_INFO', \
            'ESP_START', \
            'ESPING',\
            'SPEED_UP',\
            'SLIDING',\
            'TURNNING',\
            'STOP'))

        self.cur_state = self.ACTION_STATE.WATTING_INFO
        self.drec = 0# Used to represent the orientation information of the threat

        self.decode_model = torch.load('./esp_model/decode_model.pth')
        self.decode_model.eval()

        self.trans_model = torch.load('./esp_model/transfor_model.pth')
        self.trans_model.eval()

        self.fit_para = pd.read_csv('fit_para.txt', sep='\t', header=None)

        self.popt_mean = self.fit_para.iloc[:, 0].values
        self.popt_std = self.fit_para.iloc[:, 1].values

        self.LSTM_action = True

        self.action_len = int(bias)



    def init_serial(self):
        #self.ser = serial.Serial(self.port, self.baud_rate)
        try:
            self.send_sertool = serctl.RobotSerial()
            self.send_sertool.close_serial()
            self.send_sertool.init_serial(self.snd_port, self.snd_baud_rate)

            self.rev_sertool = serctl.RobotSerial()
            self.rev_sertool.close_serial()
            self.rev_sertool.init_serial(self.rev_port, self.rev_baud_rate)
            print("serial is connected successfully!!")
        except:
            self.send_sertool = None
            print("serial error")

    def close_serial(self):
        self.send_sertool.close_serial()
        self.send_sertool = None

        self.rev_sertool.close_serial()
        self.rev_sertool = None

# Initialize the robotic fish's motor and other states
    def init_fish(self):
        datapack = self.rftool.RFLink_packdata(rflink.Command.SHAKING_HANDS.value, 0)
        try:
            self.send_sertool.write_cmd(datapack)
            # Save the control input and the robotic fish's state response
        except:
            print("send error")

        time.sleep(1)
        datapack = self.rftool.RFLink_packdata(rflink.Command.SET_SWIM_RUN.value, 0)
        try:
            self.send_sertool.write_cmd(datapack)
            # Save the control signals and the robotic fish's state response
        except:
            print("send error")

        time.sleep(1)
        datapack = self.rftool.RFLink_packdata(rflink.Command.SET_SWIM_START.value, 0)
        try:
            self.send_sertool.write_cmd(datapack)
            # Save the control values and the robotic fish's state responses
        except:
            print("send error")

    # Generate the action sequence to be executed by the robotic fish
    def gene_action_set(self,cur_state,danger_drec = 0,amp=80,fre = 1):
        # Fix the length of the generated action sequence to 10 steps, with a time interval of 0.05 when issuing commands. The actual interval will depend on the robotic fish's execution.
        if cur_state==self.ACTION_STATE.ESP_START:
            # Generate the steering start action sequence using the inverse dynamics model. Currently, a fixed action sequence is used.
            if self.LSTM_action:
                esp_drec_mean = piecewise_linear(danger_drec, *self.popt_mean)
                esp_drec_std = piecewise_linear(danger_drec, *self.popt_std)
                
                esp_drec =np.random.normal(esp_drec_mean,esp_drec_std)
                esp_drec = [[esp_drec]]

                decode_input = torch.tensor(esp_drec).unsqueeze(1).float()
                state_seq = self.decode_model(decode_input).detach().numpy()
                transfor_input = torch.tensor(state_seq).unsqueeze(1).float()
                action_set = 1*self.trans_model(transfor_input).detach().numpy()[0]
                print(self.factor)
            else:
                amp_ = 90
                factor_ = 0.5
                steps = self.action_len
                len1 = round(steps*factor_)
                len2 = round(steps*(1-factor_))
                seq1_t = np.linspace(0, np.pi, len1)-0.5*np.pi
                action_1 = amp_*(np.sin(seq1_t)+1)
                seq2_t = np.linspace(0, np.pi, len2)
                action_2 = amp_*(np.cos(seq2_t)+1)
                action_set = np.hstack((action_1,action_2))

        elif cur_state==self.ACTION_STATE.SPEED_UP:
            seq1_t = np.linspace(0, 2*np.pi, int(self.action_len/self.frequency))
            action_set = 110*np.sin(seq1_t)

        elif cur_state==self.ACTION_STATE.SLIDING:
            action_set = np.zeros(self.action_len)
        elif cur_state==self.ACTION_STATE.TURNNING:
            # Generate the steering start action sequence using the inverse dynamics model
            # Adjust the turning amplitude and bias size for normal steering here
            amp_= -80
            turn_amp = -80
            factor_ = 0.5
            steps = self.action_len
            len1 = round(steps*factor_)
            len2 = round(steps*(1-factor_))
            seq1_t = np.linspace(0, np.pi, len1)-0.5*np.pi
            action_1 = amp_*(np.sin(seq1_t))+turn_amp
            seq2_t = np.linspace(0, np.pi, len2)
            action_2 = amp_*(np.cos(seq2_t))+turn_amp
            action_set = np.hstack((action_1,action_2))
        else:
            action_set = np.zeros(self.action_len)
        return action_set

    # Send motion commands to robotic fish and log its status
    def single_run(self,cur_state,danger_drec = 0,amp=80,fre = 1):
        if self.vision_flag == 1:
            self.action_set = self.gene_action_set(cur_state=cur_state,danger_drec = danger_drec,amp=amp,fre = fre)
        else:
            self.action_set = self.gene_action_set(cur_state=cur_state,danger_drec = danger_drec,amp=amp,fre = fre)

        # self.action_set = -1*self.gene_action_set(cur_state=cur_state,danger_drec = danger_drec,amp=amp,fre = fre)
        for action in self.action_set:
            action_snd = struct.pack('<f', float(action))
            datapack = self.rftool.RFLink_packdata(rflink.Command.SET_WAISTPITCH_OFFSET.value, action_snd)
            try:
                self.send_sertool.write_cmd(datapack)
                # Send the action execution command to the robotic fish and store the robotic fish's state
            except:
                print("send error")

            # Save the action and the robotic fish's state
            try:
                self.pos = np.loadtxt(self.pos_detec, delimiter=",")
                if len(self.pos)==4:
                    self.pos_pre = self.pos
                else:
                    self.pos = self.pos_pre
            except:
                self.pos = self.pos_pre

            inverse_dyna_data = np.hstack((self.pos,[action]))
            self.inverse_dyna_dataset = np.vstack((self.inverse_dyna_dataset,inverse_dyna_data))
            self.signal_generated.emit(action)
            time.sleep(0.08)

        # self.pos = np.zeros((1,3))
        # self.pos_pre = self.pos

        # now = datetime.now()  
        # timestr = now.strftime("%m%d%H%M%S")
        # dis_name = self.save_data_dr+'inverse_dyna_state_'+timestr+'.txt'
        # np.savetxt(dis_name, self.inverse_dyna_dataset[1:,:],fmt="%.6f", delimiter=",")
        #
        # dis_name = self.save_data_dr+'inverse_dyna_action_'+timestr+'.txt'
        # np.savetxt(dis_name, self.action_set,fmt="%.6f", delimiter=",")
        # self.inverse_dyna_dataset = np.zeros((1,5))
        # print("Generator has been colosed")

        # plt.plot(self.action_set)
        # plt.xlabel('Relative Timestamp')
        # plt.ylabel('Data')
        # plt.legend()
        # plt.show()
    def is_danger(self,data):
        vision_thre = 10# Set a visual detection threshold to prevent false detection of small objects
        fal_detec_x,fal_detec_y = 124,16
        press_id,press_amp,vision_x,vision_y,vision_area = data
        danger_flag = False
        danger_drec = 0
        vision_norm_l = np.sqrt((vision_x-30)**2+(vision_y-20)**2)
        vision_norm_r = np.sqrt((vision_x-98)**2+(vision_y-20)**2)
        if press_id>0 or vision_norm_l<15 or vision_norm_r<15:
            danger_flag = True
            if vision_norm_l<15 or vision_norm_r<15:
                self.vision_flag = 1
        if press_id>0:
            # Here, each pressure sensor's response is mapped to an angle value
            self.vision_flag = 0
            danger_drec = press_id*20+20
        elif vision_area>vision_thre:
            
            if vision_x<640:
                danger_drec = (vision_y-10)*30 # Map the 10-40 pixel range in the field of view to 0-180
            else:
                danger_drec = (vision_y-40)*30 # Map the 40-10 pixel range in the field of view to the negative range of 0-180

        return danger_flag,danger_drec
    # Continuously receive the threat orientation and other information detected by the robotic fish,
    # build an escape finite state, and conduct demonstration experiments.
    def run(self):
        try:
            while self.running:
                if self.cur_state == self.ACTION_STATE.WATTING_INFO:
                    rx_data = self.rev_sertool.read_data()
                    # The data is fed into the state machine, and the state machine needs to be rewritten
                    if self.rftool.RFLink_receivedata(rx_data): #
                        new_data = self.rftool.data_int_pack
                        # print(new_data)
                        self.rev_dataset = np.vstack((self.rev_dataset,new_data))
                        danger_flag,danger_drec = self.is_danger(new_data)

                        print(new_data,danger_flag,danger_drec,self.vision_flag)
                        # danger_flag = True
                        if danger_flag:
                            # Execute the action for the stress start phase
                            self.cur_state = self.ACTION_STATE.ESP_START
                            self.drec = danger_drec
                    
                    #self.cur_state = self.ACTION_STATE.ESP_START
                    #self.drec = 20
                elif self.cur_state==self.ACTION_STATE.ESP_START:
                    self.single_run(self.ACTION_STATE.ESP_START,danger_drec = self.drec)
                    self.cur_state = self.ACTION_STATE.ESPING

                elif self.cur_state==self.ACTION_STATE.SPEED_UP:
                    self.single_run(self.ACTION_STATE.SPEED_UP)
                    self.cur_state = self.ACTION_STATE.ESPING
                elif self.cur_state == self.ACTION_STATE.ESPING:
                    # Randomly generate the states of sliding, acceleration, and deflection direction, and perform three samples based on statistical probability
                    for t in range(2):
                        prob = [0.2,0.3,0.5]
                
                        results = [self.ACTION_STATE.TURNNING,self.ACTION_STATE.SPEED_UP]
                        #results = [self.ACTION_STATE.TURNNING,self.ACTION_STATE.TURNNING]
                        cumulative_probabilities = [sum(prob[:i+1]) for i in range(len(prob))] # Cumulative probability list
                        random_number = random.random() #
                        for i in range(len(results)):
                            # if random_number < cumulative_probabilities[i]:
                            #     random_state = results[i]
                            #     break
                            random_state = results[i]
                            self.single_run(random_state)
                    now = datetime.now() 
                    timestr = now.strftime("%m%d%H%M%S")
                    dis_name = self.save_data_dr+'inverse_dyna_state_'+timestr+'.txt'
                    np.savetxt(dis_name, self.inverse_dyna_dataset[1:,:],fmt="%.6f", delimiter=",")
                    #self.cur_state = self.ACTION_STATE.STOP
                    self.sleep(2)
                    self.cur_state = self.ACTION_STATE.WATTING_INFO
                    self.rev_sertool.clear_rev_buff()
                #self.sleep(0.1)



        except serial.SerialException as e:
            print(f'Serial Exception: {e}')

    def stop(self):
        # Reset the traces of the previous run
        self.running = False
        self.pos = np.zeros((1,3))
        self.pos_pre = self.pos

        #self.inverse_dyna_dataset.astype('int16')
        now = datetime.now()  # Get the current time
        timestr = now.strftime("%m%d%H%M%S")
        #pres_name = press_dr+'press'+'%04d' % dataset_num+'.txt'
        dis_name = self.save_data_dr+'inverse_dyna_state_'+timestr+'.txt'
        np.savetxt(dis_name, self.inverse_dyna_dataset[1:,:],fmt="%.6f", delimiter=",")

        dis_name = self.save_data_dr+'inverse_dyna_action_'+timestr+'.txt'
        np.savetxt(dis_name, self.action_set,fmt="%.6f", delimiter=",")

        dis_name = self.rev_data_dr+'rev_dataset_'+timestr+'.txt'
        np.savetxt(dis_name, self.rev_dataset,fmt="%.6f", delimiter=",")

        # plot_data = self.inverse_dyna_dataset
        # self.inverse_dyna_dataset = np.zeros((1,5))
        self.cur_state = self.ACTION_STATE.WATTING_INFO
        #
        # plt.plot(plot_data[1:,-1])
        # plt.xlabel('Relative Timestamp')
        # plt.ylabel('Data')
        # plt.legend()
        # plt.show()

        print("Generator has been colosed")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.isrunning = False
        self.amplitude_label = QLabel('Amplitude:')
        self.frequency_label = QLabel('Frequency/Factor:')
        self.bias_label = QLabel('Bias/steps:')
        self.snd_port_label = QLabel('Snd Port: COM')
        self.snd_baud_rate_label = QLabel('Snd Baud Rate:')

        self.rev_port_label = QLabel('Rev Port: COM')
        self.rev_baud_rate_label = QLabel('Rev Baud Rate:')

        self.amplitude_input = QLineEdit()
        self.amplitude_input.setText('80')
        self.frequency_input = QLineEdit()
        self.frequency_input.setText('1.2')
        self.bias_input = QLineEdit()
        self.bias_input.setText('20')
        self.snd_port_combo = QComboBox()
        self.snd_baud_rate_combo = QComboBox()

        self.rev_port_combo = QComboBox()
        self.rev_baud_rate_combo = QComboBox()

        self.snd_port_combo.addItems(['6','59','55'])  # Add 20 COM port options
        self.snd_baud_rate_combo.addItems(['9600', '19200', '115200'])

        self.rev_port_combo.addItems(['5','54','55'])  # Add 20 COM port options
        self.rev_baud_rate_combo.addItems(['19200', '9600', '115200'])

        self.set_button = QPushButton('Set Parameters')
        self.set_button.clicked.connect(self.set_parameters)

        self.connect_button = QPushButton('Connect')
        self.connect_button.clicked.connect(self.connect_to_port)
        self.disconnect_button = QPushButton('Disconnect')
        self.disconnect_button.clicked.connect(self.disconnect_from_port)

        self.init_button = QPushButton('init fish')
        self.init_button.clicked.connect(self.init_collection)

        self.start_button = QPushButton('Start Action')
        self.start_button.clicked.connect(self.start_collection)
        self.start_button.setShortcut('r')

        self.factor_label = QLabel('factor:')

        self.factor_input = QLineEdit()
        self.factor_input.setText('-0.1')

        main_widget = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout()
        main_widget.setLayout(layout)
        row_cnt = 0
        col_cnt = 0
        layout.addWidget(self.amplitude_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.amplitude_input,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.frequency_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.frequency_input,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.bias_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.bias_input,row_cnt,col_cnt,1,1)

        row_cnt +=1
        col_cnt = 0
        layout.addWidget(self.snd_port_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.snd_port_combo,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.snd_baud_rate_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.snd_baud_rate_combo,row_cnt,col_cnt,1,1)
        col_cnt+=1.5
        layout.addWidget(self.set_button,row_cnt,col_cnt,1,1)

        row_cnt +=1
        col_cnt = 0
        layout.addWidget(self.rev_port_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.rev_port_combo,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.rev_baud_rate_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.rev_baud_rate_combo,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.connect_button,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.disconnect_button,row_cnt,col_cnt,1,1)

        row_cnt +=1
        col_cnt = 0
        layout.addWidget(self.set_button,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.init_button,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.start_button,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.factor_label,row_cnt,col_cnt,1,1)
        col_cnt+=1
        layout.addWidget(self.factor_input,row_cnt,col_cnt,1,1)


        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.generator = None

    def set_parameters(self):

        amplitude = float(self.amplitude_input.text())
        frequency = float(self.frequency_input.text())
        bias = float(self.bias_input.text())
        factor = float(self.factor_input.text())
        snd_port = 'COM' + self.snd_port_combo.currentText()
        snd_baud_rate = int(self.snd_baud_rate_combo.currentText())
        rev_port = 'COM' + self.rev_port_combo.currentText()
        rev_baud_rate = int(self.rev_baud_rate_combo.currentText())
        if self.generator:
            # self.generator.stop()
            # self.generator.wait()
            self.generator.amplitude = amplitude
            self.generator.frequency = frequency
            self.generator.bias = bias
            self.generator.factor = factor
            print("parameter has been changed")
        else:
            self.generator = SignalGenerator(amplitude, frequency, bias, snd_port, snd_baud_rate,rev_port, rev_baud_rate,factor)
            print("generator has been creaeted!!!")

    def connect_to_port(self):
        if self.generator and not self.isrunning:
            self.generator.init_serial()
            self.start_button.setText('Start Collection')
            self.start_button.setShortcut('r')

    def disconnect_from_port(self):
        if self.generator:
            self.generator.close_serial()
        if self.isrunning:
            self.generator.stop()
            self.start_button.setText('Start Collection')
            self.start_button.setShortcut('r')

    def init_collection(self):
        self.generator.init_fish()

    def start_collection(self):
        if self.generator:
            if not self.generator.running:
                self.generator.running = True
                self.generator.start()
                self.start_button.setText('Stop Collection')
                self.start_button.setShortcut('r')
            else:
                self.generator.stop()
                self.start_button.setText('Start Collection')
                self.start_button.setShortcut('r')
                self.generator.running = False
        else:
            print('none generator')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
