# python3
# @Time    : 2021.05.18
# @Author  : 张鹏飞
# @FileName: rflink.py
# @Software: 机器鲨鱼上位机
# @修改历史:
# version: 机器海豚上位机    author: Sijie Li    data: 2022.04.06

from enum import Enum
import numpy as np
import struct

FishID = Enum('Fish_id', ({ \
    'FISH_ALL': b'\x00', \
    'Fish_1': b'\x33'}))

# Recstate枚举类型
Recstate = Enum('Recstate', ( \
    'WAITING_FF', \
    'SENDER_ID', \
    'RECEIVER_ID', \
    'RECEIVE_LEN', \
    'RECEIVE_PACKAGE', \
    'RECEIVE_CMD',\
    'RECEIVE_CHECK'))

# Command枚举类型(set:3-28,read:29-40,goto:41-43)
Command = Enum('Command', ( \
    'ALL_LEFT_PRE', \
    'ALL_RIGHT_PRE', \
    'START_STFT'
))

# Command枚举类型(set:3-28,read:29-40,goto:41-43)
Command = Enum('Command',(\
    'SHAKING_HANDS',\
    'SYNCHRONIZE_CLOCK',\
    'SET_SWIM_RUN',\
    'SET_SWIM_START',\
    'SET_SWIM_STOP',\
    'SET_SWIM_FORCESTOP',\
    'SET_RE_POWER_MOTOR',\
    'SET_SWIM_SPEEDUP',\
    'SET_SWIM_SPEEDDOWN',\
    'SET_SWIM_LEFT',\
    'SET_SWIM_RIGHT',\
    'SET_SWIM_STRAIGHT',\
    'SET_PECT_LEFT_TURNING',\
    'SET_PECT_RIGHT_TURNING',\
    'SET_SWIM_UP',\
    'SET_SWIM_DOWN',\
    'SET_WAISTPITCH_AMP',\
    'SET_WAISTPITCH_FREQ',\
    'SET_WAISTPITCH_OFFSET',\
    'ENABLE_SDSAVE',\
    'ENABLE_DATASEND',\
    'ENABLE_RESET',\
	'SET_LPECT_AMP',\
	'SET_LPECT_FREQ',\
	'SET_LPECT_OFFSET',\
	'SET_RPECT_AMP',\
	'SET_RPECT_FREQ',\
	'SET_RPECT_OFFSET',\
    'SET_PECFIN_ZERO',\
    'SET_LEFTPECFIN_UP',\
	'SET_LEFTPECFIN_ZERO',\
    'SET_LEFTPECFIN_DOWN',\
    'SET_RIGHTPECFIN_UP',\
    'SET_RIGHTPECFIN_ZERO',\
    'SET_RIGHTPECFIN_DOWN',\
    'SET_24VOLTAGE_CONTROL',\
    'SET_DEPTH_CONTROL_START',\
    'SET_DEPTH_CONTROL_OVER',\
    'SET_DEPTH_CONTROL_TARGET',\
    'SET_DEPTH_CONTROL_PARAS',\
    'SET_PUMP_OFF',\
    'SET_PUMP_IN',\
    'SET_PUMP_OUT',\
    'SET_PASSIVE_UP',\
    'SET_PASSIVE_DOWN',\
    'SET_PASSIVE_MID',\
	'SET_LATERAL_MASS_FMOVE',\
    'SET_LATERAL_MASS_BMOVE',\
    'SET_LATERAL_MASS_STOP',\
    'SET_DATASHOW_OVER',\
    'SET_AUTOCTL_RUN',\
    'SET_AUTOCTL_STOP',\
    'READ_ROBOT_STATUS',\
    'READ_SWIM_PARAM',\
    'READ_AHRS_ATTITUDE',\
    'READ_AHRS_ACCEL',\
    'READ_AHRS_GYRO',\
    'READ_DEPTH',\
    'READ_INFRARED_DISTANCE',\
    'READ_DVL_VELOCITY',\
    'READ_DVL_ALTITUDE',\
    'READ_BEIDOU_INFORMATION',\
    'READ_ALL_LIMIT_SWITCH_STATE',\
	'READ_FILE_LIST',\
    'GOTO_STORAGE_DATA',\
    'GOTO_STOP_STORAGE',\
    'GOTO_SEND_DATA',\
    'PRINT_SYS_MSG',\
    'LAST_COMMAND_FLAG',\
    'STARTCOLL',\
    'STOPCOLL'))


class RFLink():
    """
    Robotic Fish 通讯协议类
    通讯协议规范:(一帧完整数据如下:)
    2022.04.06自定义修改后:
    0xFF, SENDER_ID, RECEIVER_ID, RECEIVE_LEN, RECEIVE_PACKAGE, RECEIVE_CHECK

    2021.10.11自定义修改后:
    :arg length: 消息长度
    :arg message: 消息(byte类型)

    :attributes RFLink_receivedata:接收状态机,解码RFLink通讯协议
    :attributes RFLink_packdata:将待发送数据按RFLink通讯协议打包

    2023年1月用于侧线数据接收的协议修改
    0xFF, SENDER_ID, RECEIVER_ID, RECEIVE_LEN, ALL_ID1, RECEIVE_PACKAGE1...ALL_ID6, RECEIVE_PACKAGE6
    """

    def __init__(self):
        self.sender_id = b''
        self.receiver_id = b''
        self.length = 0
        self.message = b''
        self._receive_state = Recstate.WAITING_FF
        self._checksum = 0
        self._byte_count = 0
        # self.MY_ID = b'\x11'
        # self.FRIEND_ID = b'\x44'
        self.MY_ID = b'\x44'#用在小电脑上，改过下位机ID
        self.FRIEND_ID = b'\x11'
        self.sensor_num = 6#使用全部的传感器
        self.cmd = 0

    def RFLink_receivedata(self, rx_data):
        """
        RFLink接收状态机
        :param rx_data: 串口接收到的数据
        :return: 当接收到一帧完整数据时,返回1;否则,返回0.
        :没有校验，结尾0xfc
        ：没有cmd，数据长度之后直接是12个传感器共24位数据
        """
        if self._receive_state == Recstate.WAITING_FF:
            if rx_data == b'\xff':
                self._receive_state = Recstate.SENDER_ID
                #self._checksum = ord(rx_data)  # 转换为ASCII码
                self.message = b''
                self.length = 0
                self._byte_count = 0

        elif self._receive_state == Recstate.SENDER_ID:
            # print('进入SENDER_ID')
            if rx_data == b'\x44':
                self._receive_state = Recstate.RECEIVER_ID
                #self._checksum += ord(rx_data)
            else:
                self._receive_state = Recstate.WAITING_FF

        elif self._receive_state == Recstate.RECEIVER_ID:
            # print('进入RECEIVER_ID')
            if rx_data == b'\x11':
                self._receive_state = Recstate.RECEIVE_LEN
                #self._checksum += ord(rx_data)
            else:
                self._receive_state = Recstate.WAITING_FF

        elif self._receive_state == Recstate.RECEIVE_LEN:
            # print('进入RECEIVE_LEN')
            self._receive_state = Recstate.RECEIVE_PACKAGE
            #self._checksum += ord(rx_data)
            self.length = ord(rx_data)

        elif self._receive_state == Recstate.RECEIVE_PACKAGE:
            # print('进入RECEIVE_PACKAGE')
            #self._checksum += ord(rx_data)
            self.message = self.message + rx_data
            self._byte_count += 1
            if self._byte_count >= self.length:
                self._receive_state = Recstate.RECEIVE_CHECK
                #self._checksum = self._checksum % 255

        elif self._receive_state == Recstate.RECEIVE_CHECK:
            if rx_data == b'\xfc':
            #if rx_data == self._checksum.to_bytes(1, 'big'):
                # print('成功收到1帧数据')
                # print(rx_data)
                # print(self._checksum.to_bytes(1,'big'))
                # print(self.message)
                # 将数据帧进行分析
                self.analysis_data()
                #self._checksum = 0
                self._receive_state = Recstate.WAITING_FF
                return 1
            else:
                self._receive_state = Recstate.WAITING_FF

        else:
            self._receive_state = Recstate.WAITING_FF

        return 0

        #########################################################################################################
    def RFLink_receivecmd(self, rx_data):
        """
        RFLink接收状态机
        :param rx_data: 串口接收到的数据
        :return: 当接收到一帧完整数据时,返回对应的指令;否则,返回0.
        :没有校验，结尾0xfc
        """

        if self._receive_state == Recstate.WAITING_FF:
            if rx_data == b'\xff':
                self._receive_state = Recstate.SENDER_ID
                #self._checksum = ord(rx_data)  # 转换为ASCII码
                self.message = b''
                self.length = 0
                self._byte_count = 0

        elif self._receive_state == Recstate.SENDER_ID:
            # print('进入SENDER_ID')
            if rx_data == self.FRIEND_ID:
                self._receive_state = Recstate.RECEIVER_ID
                #self._checksum += ord(rx_data)
            else:
                self._receive_state = Recstate.WAITING_FF

        elif self._receive_state == Recstate.RECEIVER_ID:
            # print('进入RECEIVER_ID')
            if rx_data == self.MY_ID:
                self._receive_state = Recstate.RECEIVE_CMD
                #self._checksum += ord(rx_data)
            else:
                self._receive_state = Recstate.WAITING_FF

        elif self._receive_state == Recstate.RECEIVE_CMD:
            self.cmd= rx_data
            # print('进入RECEIVE_LEN')
            self._receive_state = Recstate.RECEIVE_CHECK
            #self._checksum += ord(rx_data)
            self.length = ord(rx_data)

        elif self._receive_state == Recstate.RECEIVE_CHECK:
            if rx_data == b'\xfc':
                self._receive_state = Recstate.WAITING_FF
                cmd = self.cmd
                self.cmd = 0
                return cmd
            else:
                self._receive_state = Recstate.WAITING_FF

        else:
            self._receive_state = Recstate.WAITING_FF

        return 0

        #########################################################################################################

    def analysis_data(self,with_cmd = 0):  # 分析串口接收到的rflink数据,更新robosharkstate的状态
        """
        para:with_cmd=1,message中有一个指令，用于两侧分别采集
             with_cmg=0,·没有指令，直接进行数据存储
        本函数将串口接收到的完整rflink数据帧进行分析,
        根据数据帧中的指令描述解码数据
        如果with_cmd=1
        :return: 收到Command的ID
        否则
        return 0,直接进行数据转换
        """
        data_pack = self.message
        Len = self.length
        if with_cmd ==1:

            command_id = data_pack[0]
            self.data_command = Command(command_id)
            self.data_int_pack = np.zeros(int(Len / 2))
            # 下位机上传的数据是按顺序存储的高低八位
            if self.data_command is Command.ALL_LEFT_PRE or self.data_command is Command.ALL_RIGHT_PRE:
                if Len == self.sensor_num * 2:
                    for i in range(1, Len, 2):
                        a = int(data_pack[i] << 8 | data_pack[i + 1])
                        self.data_int_pack[int((i - 1) / 2)] = a
        else:
            data_pack = self.message
            self.data_int_pack = np.zeros(int(Len / 2))
            # 下位机上传的数据是按顺序存储的高低八位
            #if Len == self.sensor_num * 2:
            if 1:
                for i in range(0, Len, 2):
                    a = int(data_pack[i] << 8 | data_pack[i + 1])
                    self.data_int_pack[int(i / 2)] = a


    def RFLink_packdata(self, cmd, databyte):
        """
        RFLink数据与指令打包函数
        :param cmd:Command
        :param data:待发送数据
        :return:符合RFLink通讯协议的消息包
        """
        first_byte = b'\xff'
        second_byte = self.MY_ID
        third_byte = self.FRIEND_ID

        cmdbyte = cmd.to_bytes(1, 'big')
        if databyte != 0 and databyte is not None:
            datalenbyte = len(databyte).to_bytes(1, 'big')
        else:
            databyte = b''
            datalenbyte = b'\x00'

        check_num = ord(first_byte) + ord(second_byte) + ord(third_byte)
        check_num = check_num + datalenbyte[0] + ord(cmdbyte)
        for data in databyte:
            check_num = check_num + data
        check_num = (check_num % 255).to_bytes(1, 'big')
        datapack = first_byte + second_byte + third_byte + datalenbyte + cmdbyte + databyte + check_num
        return datapack


if __name__ == "__main__":
    # # print(RFLink_packdata(Command.SET_CPG_AMP.value,0.0))
    # p = [0, 1, 2, 3]
    # data = 1
    rf = RFLink()
    # rf.RFLink_receivedata(b'\xff')
    # rf.RFLink_receivedata(b'\x33')
    # rf.RFLink_receivedata(b'\x11')
    # rf.RFLink_receivedata(b'\x0c')
    # # 数据包开始，包含六个数据，每个数据分高低八位
    # # command不计算数据长度
    # rf.RFLink_receivedata(b'\x01')
    #
    # rf.RFLink_receivedata(b'\x14')
    # rf.RFLink_receivedata(b'\x1D')
    #
    # rf.RFLink_receivedata(b'\x22')
    # rf.RFLink_receivedata(b'\x21')
    #
    # rf.RFLink_receivedata(b'\x34')
    # rf.RFLink_receivedata(b'\x3D')
    #
    # rf.RFLink_receivedata(b'\x42')
    # rf.RFLink_receivedata(b'\x41')
    #
    # rf.RFLink_receivedata(b'\x54')
    # rf.RFLink_receivedata(b'\x5D')
    #
    # rf.RFLink_receivedata(b'\x64')
    # rf.RFLink_receivedata(b'\x00')
    #
    # # 假定校验码是正确的
    # rf.RFLink_receivedata(b'\xD0')
    # # rf.RFLink_receivedata(b'\x71')
    # # message = rf.message
    # # print(Command['READ_AHRS_ATTITUDE'].value)
    # # print(message[0].to_bytes)
    # # print(p[0:])
    res_drec = 2
    res_amp = 5
    data = res_drec.to_bytes(1, byteorder='big')+res_amp.to_bytes(1, byteorder='big')
    snd_pack = rf.RFLink_packdata(10,data)
    hex_string = ' '.join([f'{byte:02X}' for byte in snd_pack])
    print(hex_string)
