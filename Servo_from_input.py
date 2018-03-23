from time import sleep
from Arduino_Servo import Arduino_Servo
import select
import threading
import math
looping = True

class ServoThread(threading.Thread):
    def run(self):
        while looping:
            x = int(input("Rotate:"))
            servo.rotate(x)
            print("Rotating: " + str(x))

servo = Arduino_Servo()
try:
    servo_th = ServoThread().start()
    while 1:
        sleep(1)
except KeyboardInterrupt:
    looping = False
    print("Ending")
    servo.cleanup()