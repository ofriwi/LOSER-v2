from time import sleep
from Arduino_Stepper import Arduino_BT_Stepper
import select
import threading
import math

looping = True

class StepperThread(threading.Thread):
    def run(self):
        while looping:
            x = int(input("Rotate:"))
            stepper.rotate(x)
            print("Rotating: " + str(x))
    
class BTReaderThread(threading.Thread):
    def run(self):
        while looping:
            ready = select.select([stepper.sock], [], [], 0.01)
            if ready[0]:
                if (str)(stepper.sock.recv(1024).decode()) == 'g':
                    data = (stepper.sock.recv(1024).decode())
                    print("gyro: " + str(data))
                    if abs(float(str(data))) > 1.0:
                        stepper.rotate(math.floor(float(str(data))))
                    sleep(1)


stepper = Arduino_BT_Stepper()
try:
    stepper_th = StepperThread().start()
    reader_th = BTReaderThread().start()
    while 1:
        sleep(1)
except KeyboardInterrupt:
    looping = False
    print("Ending")
    stepper.cleanup()