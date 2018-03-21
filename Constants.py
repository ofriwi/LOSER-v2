'''GENERAL'''
DEBUG_MODE = True

'''CAMERA'''
PICTURE_WIDTH_IN_DEG = 42

'''SERVO'''
SERVO_DEBUG_MODE = True
SERVO_PORT = "/dev/ttyUSB0"
SERVO_MIN_DEG = 0
SERVO_MAX_DEG = 180
SERVO_CENTER_DEG = (SERVO_MAX_DEG + SERVO_MIN_DEG)/2


'''STEPPER'''
STEPPER_DEBUG_MODE = True
STEPPER_PORT = "/dev/ttyUSB0"
STEPPER_BT_ADDR = "00:18:E4:0A:00:01"
STEPPER_BT_PORT = 1

'''DC'''
DC_DEBUG_MODE = True
DC_PORT = "/dev/ttyUSB2"
DC_BT_ADDR = "00:18:E4:0A:00:01"
DC_BT_PORT = 1
