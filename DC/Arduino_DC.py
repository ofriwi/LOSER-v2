import struct
import serial
from Constants import *
from time import sleep
import bluetooth


class Arduino_BT_Stepper:
    
    # Initialize communication
    def __init__(self, bd_addr=Stepper_BT_ADDR, port=Stepper_BT_PORT):                
        self.sock = bluetooth.BluetoothSocket (bluetooth.RFCOMM)
        self.sock.connect((bd_addr,port))

    # Rotate Stepper
    def move(self, angle):
        if DEBUG_MODE or Stepper_DEBUG_MODE:
            print("Stepper move degrees: ", angle)
        if angle >= 0:
            self.sock.send('+')
        else:
            self.sock.send('-')
        self.sock.send(struct.pack('>B', abs(angle)))
        #self.sock.send(angle)#struct.pack('>i', abs(angle)))
        #self.sock.send(str(angle))
    
    #Close serial
    def cleanup(self):
        self.move(0);
        self.sock.close()

