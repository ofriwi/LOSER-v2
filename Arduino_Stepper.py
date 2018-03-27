import struct
import serial
from Constants import *
from time import sleep
import bluetooth
import datetime

class Arduino_BT_Stepper:
    
    # Initialize communication
    def __init__(self, bd_addr=STEPPER_BT_ADDR, port=STEPPER_BT_PORT):                
        self.sock = bluetooth.BluetoothSocket (bluetooth.RFCOMM)
        self.sock.connect((bd_addr,port))
        print(GREEN + 'BT connection acquired' + NORMAL)

    # Rotate Stepper
    def rotate(self, rel_angle):
        if DEBUG_MODE or Stepper_DEBUG_MODE:
            print("Stepper rotate degrees: ", rel_angle)
        if rel_angle >= 0:
            self.sock.send('+')
        else:
            self.sock.send('-')
        self.sock.send(struct.pack('>B', abs(rel_angle)))
        #self.sock.send(rel_angle)#struct.pack('>i', abs(rel_angle)))
        #self.sock.send(str(rel_angle))
    
    # Send distance to motor
    def send_distance(self, distance):
        if DEBUG_MODE or Stepper_DEBUG_MODE:
            print("Distance: ", distance)
        self.sock.send('d')
        self.sock.send(struct.pack('>B', round(abs(distance * 10))))
        
    def ping(self):
        sent = datetime.datetime.now()
        self.sock.send('p')
        while self.sock.recv() != 'p':
            pass
        now = datetime.datetime.now()
        print((sent - now).totalSeconds())

    
    #Close serial
    def cleanup(self):
        self.rotate(0);
        self.sock.close()

