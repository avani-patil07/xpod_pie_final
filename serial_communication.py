import serial
import time

PORT = 'COM3'
BAUD = 115200

class channel:
    def __init__(self, name:str, port:str, baud_rate:int, debug:bool=True):
        self.debug = debug
        self._name = name
        self._port = port
        self._baud_rate = baud_rate
        self._ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # Wait for microcontroller to reset and initialize

    @property
    def name(self):
        return self._name

    @property
    def port(self):
        return self._port

    @property
    def baud_rate(self):
        return self._baud_rate
    
    def send(self, message:str):
        self._ser.write(f"{message}\n".encode())
        if self.debug:
            print(f"Sent: {message}")