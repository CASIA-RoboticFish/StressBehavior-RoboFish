'''
This file defines the send and receive operations for communication devices, 
directly controlling the serial port device for opening, closing, and read/write operations.
'''

import serial


class RobotSerial():
    """
    Serial Communication Class
    """

    def __init__(self):
        """
        Create a serial object
        """
        self.ser = serial.Serial()

    def init_serial(self, port, baudrate):
        """
        Open the serial port
        :param port: Device name
        :param baudrate: Baud rate
        :return:
        """
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.open()

    def close_serial(self):
        """
        Close the serial port
        :return:
        """
        self.ser.close()

    def write_cmd(self, cmd):
        """
        Send command via serial port
        :param cmd: RFLink message
        :return:
        """
        self.ser.write(cmd)
        #print('已发送:cmd为', cmd)

    def read_data(self):
        """
        Receive data from the serial port
        :return: Data received from the serial port (byte type)
        """
        rx_data = self.ser.read()
        return rx_data


if __name__ == "__main__":
    ser = RobotSerial()
    ser.init_serial("/dev/ttyUSB0", 9600)

    cmd = b"\xff\xff\x11\x01\x00\x00\x01\x13"
    #ser.write_cmd(cmd)
    while 1 :
        rev = ser.read_data()
        print(rev)