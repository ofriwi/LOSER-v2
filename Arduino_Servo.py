import struct
import serial
from Constants import *
from time import sleep
import bluetooth

offset = 100
reset_to_mid = 255

class Arduino_Servo:
    # Initialize communication
    def __init__(self, baudrate=9600, port=SERVO_PORT):
        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = port
        self.ser.open()
        sleep(2)
        self.goto_mid()
        print(GREEN + 'Servo connection acquired' + NORMAL)

    # Rotate Servo
    def rotate(self, rel_angle):
        if (DEBUG_MODE or SERVO_DEBUG_MODE):
            print('Servo turn ' + str(rel_angle))
        rel_angle += offset
        self.ser.write(struct.pack('>B', round(rel_angle)))
        #self.sock.send(rel_angle)#struct.pack('>i', abs(rel_angle)))
        #self.sock.send(str(rel_angle))
    
    # Move to mid position
    def goto_mid(self):
        print('Servo: ' + str(reset_to_mid))
        self.ser.write(struct.pack('>B', reset_to_mid))

    #Close serial
    def cleanup(self):
        self.goto_mid()
        self.ser.close()

