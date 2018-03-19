import struct
import serial
from Constants import *
from time import sleep
import bluetooth


class Arduino_BT_DC:
    
    # Initialize communication
    def __init__(self, bd_addr=DC_BT_ADDR, port=DC_BT_PORT):                
        self.sock = bluetooth.BluetoothSocket (bluetooth.RFCOMM)
        self.sock.connect((bd_addr,port))

    # Rotate DC
    def move(self, speed):
        if DEBUG_MODE or DC_DEBUG_MODE:
            print("DC move @speed ", speed)
        if speed >= 0:
            self.sock.send('+')
        else:
            self.sock.send('-')
        #self.sock.send(speed)#struct.pack('>i', abs(speed)))
        self.sock.send(struct.pack('>B', abs(speed)))
        #self.sock.send(str(speed))
         
    #Close serial
    def cleanup(self):
        self.move(0);
        self.sock.close()

