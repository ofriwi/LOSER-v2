from DC.Arduino_DC import Arduino_BT_DC
from PID import PID
from Constants import *

# Init
Kp = 0; Ki = 0; Kd = 0;
pid = PID(Kp, Ki, Kd);
motor = Arduino_BT_DC()
pid.setWindup(!!!!!!!!!!);
pid.SetPoint = 0




pid.update(!!!current!!!)
output = pid.output

