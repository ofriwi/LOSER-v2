''''from Arduino_Stepper import Arduino_BT_Stepper'''
from math import tan, degrees, radians, atan

X = 0
Y = 1

# Setup
'''stepper_motor = Arduino_BT_Stepper()'''



def pixels_to_degrees(pixels, axis):
    if axis == X:
        px_width = 200 ########## CHANGE
        deg_width = 62.2
    if axis == Y:
        px_width = 200 ########### CHANGE
        deg_width = 48.8
    alpha = degrees(atan(pixels * tan(radians(deg_width / 2))*2/px_width))
    return alpha

        
'''
# Main Loop
# 1. Remove the fail handling

# 2. When target detected
pos_from_mid = [1] #################### CHANGE
stepper_motor.rotate(-pixels_to_degrees(pos_from_mid[0], X))
input() # for debugging
'''