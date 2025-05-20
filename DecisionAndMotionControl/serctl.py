# This file is used to define operations on the serial port device, directly handling read and write operations for the RF communication module


import serial


class RobotSerial():
    """
    Serial Communication Class
    """
    def __init__(self):
        """
        Create Serial Port Object
        """

        self.ser = serial.Serial()

    def init_serial(self, port, baudrate):
        """
        Open Serial Port
        :param port: Device name
        :param baudrate: Baud rate
        :return:
        """

        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.open()

    def close_serial(self):
        """
        Close Serial Port
        :return:
        """

        self.ser.close()

    def write_cmd(self, cmd):
        """
        Send Command via Serial Port
        :param cmd: RFLink message
        :return:
        """

        self.ser.write(cmd)
        print('已发送:cmd为', cmd)

    def read_data(self):
        """
        Receive Data from Serial Port
        :return: Data received from the serial port (byte type)
        """

        rx_data = self.ser.read()
        return rx_data

    def clear_rev_buff(self):
        self.ser.flushInput()

    def clear_snd_buff(self):
        self.ser.flushOutput()


if __name__ == "__main__":
    ser = RobotSerial()
    ser.init_serial("/dev/ttyUSB0", 9600)

    cmd = b"\xff\xff\x11\x01\x00\x00\x01\x13"
    #ser.write_cmd(cmd)
    while 1 :
        rev = ser.read_data()
        print(rev)