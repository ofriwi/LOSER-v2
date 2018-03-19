from time import sleep
from Arduino_DC import Arduino_BT_DC
import select
import threading
import math

looping = True

class DCThread(threading.Thread):
    def run(self):
        while looping:
            x = int(input("Move:"))
            dc.move(x)
            print("Moving: " + str(x))

dc = Arduino_BT_DC()
try:
    dc_th = DCThread().start()
    while 1:
        sleep(1)
except KeyboardInterrupt:
    looping = False
    print("Ending")
    dc.cleanup()