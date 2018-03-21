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
    
class BTReaderThread(threading.Thread):
    def run(self):
        while looping:
            ready = select.select([dc.sock], [], [], 0.01)
            if ready[0]:
                if (str)(dc.sock.recv(1024).decode()) == 'g':
                    data = (dc.sock.recv(1024).decode())
                    print("gyro: " + str(data))
                    if abs(float(str(data))) > 1.0:
                        dc.move(math.floor(float(str(data))))
                    sleep(1)


dc = Arduino_BT_DC()
try:
    dc_th = DCThread().start()
    reader_th = BTReaderThread().start()
    while 1:
        sleep(1)
except KeyboardInterrupt:
    looping = False
    print("Ending")
    dc.cleanup()