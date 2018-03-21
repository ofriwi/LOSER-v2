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

stepper = Arduino_BT_Stepper()
try:
    stepper_th = StepperThread().start()
    while 1:
        sleep(1)
except KeyboardInterrupt:
    looping = False
    print("Ending")
    stepper.cleanup()