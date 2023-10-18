import serial

class TnsyInterface:
    """
    The interface between the teensy and raspberry pi
    """

    def __init__(self, baud_rate, serial_name):
        self.baud_rate = baud_rate
        self.serial_name = serial_name
        self.tserial = None

    def init_serial(self):
        self.tserial = serial.Serial(self.serial_name, self.baud_rate, timeout=1)
        self.tserial.flush()

    def read_tstate(self):
        while True:
            if self.tserial.in_waiting > 0:

        

