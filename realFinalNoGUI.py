from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Ofri

import time, datetime
from math import atan, degrees, radians, tan
from Arduino_Stepper import Arduino_BT_Stepper
from Arduino_Servo import Arduino_Servo
from Constants import *

use_screen = True
magic_number = 0.4 # Magic number is P of servo

# *** Partial mode ***
partial_mode = True
disable_mode = True
RPM = 20
deg_time = 1.0 / (RPM * 6.0) * 1000
send_time = 50
movement_end_time = datetime.datetime.now() 
movement_start_time = datetime.datetime.now()
movement_end_time = datetime.datetime.now()
direction = 1

sign = lambda a: (a>0) - (a<0)
frame_num = 0

# *** Stepper ***

X = 0
Y = 1

stepper_motor = Arduino_BT_Stepper()
servo_motor = Arduino_Servo()

def pixels_to_degrees(pixels, axis):
    if axis == X:
        px_width = 512.0 ########## CHANGE
        deg_width = 50.0#####
    if axis == Y:
        px_width = 256.0 ########### CHANGE
        deg_width = 41.41######
    alpha = degrees(atan(float(pixels) * tan(radians(deg_width / 2))*2/px_width)) # Trigo
    # alpha = float(pixels) * deg_width / px_width
    return round(alpha)


# End
model = cv2.ml.SVM_load("SVM.dat")

timeArr = []
def get_hog():
    winSize = (20, 20)
    blockSize = (8, 8)
    blockStride = (4, 4)
    cellSize = (8, 8)
    nbins = 9
    derivAperture = 1
    winSigma = -1.
    histogramNormType = 0
    L2HysThreshold = 0.2
    gammaCorrection = 1
    nlevels = 64
    signedGradient = True

    hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins, derivAperture, winSigma,
                            histogramNormType, L2HysThreshold, gammaCorrection, nlevels, signedGradient)
    return hog

def crop_rect(img,rect):
    angle = rect[2]
    rows,cols = img.shape[0],img.shape[1]
    
    
    M = cv2.getRotationMatrix2D(rect[0],angle,1)
    img_rot = cv2.warpAffine(img,M,(cols,rows))
    rect0 = (rect[0],rect[1],0.0)
    box = cv2.boxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]),M))[0]
    pts[pts<0]=0
    img_crop = img_rot[pts[1][1]:pts[0][1], pts[1][0]:pts[2][0]]
    if(img_crop.shape[0]==0 or img_crop.shape[1]==0):
        return img_crop
    if img_crop.shape[1]>img_crop.shape[0]:      
       img_crop = np.rot90(img_crop)
    return img_crop


def cleanup():
    print(RED + "Exit" + NORMAL)
    stepper_motor.cleanup()
    servo_motor.cleanup()

cam = PiCamera()
cam.vflip=True
cam.resolution=(512,256)#300150500250
center = (512/2,256/2)

cam.awb_gains=[1.3,1.3]
#cam.rotation = 270
#cam.hflip = True
cam.shutter_speed = 5000
cam.exposure_mode='antishake'
raw= PiRGBArray(cam)
time.sleep(0.1)


time.sleep(0.1)
cam.capture(raw,format="bgr")
frame = raw.array

raw.truncate(0)

x_height=1
predict=0
predict2=0

hog = get_hog()

kernal = np.ones((3,3),np.uint8)
kernal1 = np.ones((1,1),np.uint8)


print(GREEN + 'Run started!' + NORMAL)
for fram in cam.capture_continuous(raw,format='bgr',use_video_port=True):
    # Ofri
    frame_num += 1
    if frame_num == 100:
        print('100 frames reached')
        frame_num = 0
    shoot_time = datetime.datetime.now()    
    # End

    raw.truncate(0)
    timer = cv2.getTickCount()
    frame =  fram.array       
    
    frame = (255-frame)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)     
     
    l = np.array([40,40,170]) #good
    u = np.array([100,200,220]) # good
    mask = cv2.inRange(hsv,l,u)
        
        
    mask = cv2.erode(mask,kernal1, iterations = 1)
    mask = cv2.dilate(mask,kernal, iterations = 4)
        
        
    frame = (255-frame)
    masked = cv2.bitwise_and(frame,frame,mask=mask)

    im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
    c = 0
    rect = 0
    if len(contours) > 0:
        contours = sorted(contours, key = cv2.contourArea, reverse = True)
        contours = contours[0:10]
            
        for con in contours:               

            x,y,w,h = cv2.boundingRect(con)
            rect = cv2.minAreaRect(con)
                
            if w!=0 and h!=0 and w*h > 400 and float(w)/ h < 3 and float(w)/ h > 0.1:
                    
                roi= crop_rect(frame,rect)                  
                 #   roi = crop_rect(frame,(rect[0],(rect[1][0]*1.2,rect[1][1]*1.2),rect[2]))
                x_height = len(roi)                 
                predict = 0
                    
              
                if use_screen:
                    cv2.imshow("roig",roi)
                
                if roi is not None and len(roi)>0 and len(roi[0])>0:
                    imArr = []
                        
                    roi=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)                        
                    roi = cv2.resize(roi,(20,20))
                        
                       
                        
                    imArr.append(hog.compute(roi))                  
                    imArr = np.array(imArr)
                      
                    predict = model.predict(imArr)[1][0]
                    if predict == 1:                  
                        c = con
                        break             
            
    if type(c) != int:
   
        rect = cv2.minAreaRect(c)
        a = rect[0]
        pos_from_mid = (a[0]-center[0], a[1]-center[1])
        distance = 86.67*pow(x_height,-0.8693)
        if use_screen:
            cv2.drawContours(frame,[np.int0(cv2.boxPoints(rect))],0,[0,255,0],2)
        # Ofri
        #ofriflag += 1
        #if (ofriflag == 1):
        now_time = datetime.datetime.now() # Partial
        deg_moved = 0 # Disable mode
        if not disable_mode and partial_mode and shoot_time < movement_end_time: # Partial
            print('Still moving')
            deg_moved = direction * round(min(max(0, (now_time - movement_start_time).total_seconds()), (movement_end_time - movement_start_time).total_seconds(), (now_time - shoot_time).total_seconds(), (movement_end_time - shoot_time).total_seconds()) * 1000.0 / deg_time) # Partial
            print(deg_moved)
            print('shoot to end=' + str(round((movement_end_time - shoot_time).total_seconds() * 1000.0 / deg_time))) # Partial
            print('shoot to now=' + str(round((now_time - shoot_time).total_seconds() * 1000.0 / deg_time))) # Partial
            print('start to end=' + str(round((movement_end_time - movement_start_time).total_seconds() * 1000.0 / deg_time))) # Partial
            print('start to now=' + str(round((now_time - movement_start_time).total_seconds() * 1000.0 / deg_time))) # Partial
        #ofriflag = 0
        stepper_to_move = -(pixels_to_degrees(pos_from_mid[X], X) - deg_moved) # Partial : - deg_moved
        servo_to_move = -magic_number*pixels_to_degrees(pos_from_mid[Y], Y)
        if disable_mode and shoot_time > movement_end_time or not disable_mode:
            #print('to move: ' + str(stepper_to_move+deg_moved) + 'move more: ' + str(stepper_to_move))
            direction = sign(stepper_to_move) # Partial
            stepper_motor.rotate(stepper_to_move)
            servo_motor.rotate(servo_to_move)

            stepper_motor.send_distance(distance)
            movement_start_time = datetime.datetime.now() + datetime.timedelta(microseconds=send_time * 1000) # Partial
            movement_end_time = movement_start_time + abs(datetime.timedelta(microseconds=round(stepper_to_move * deg_time * 1000))) # Partial
            

        
    if use_screen:
          # i frame = cv2.resize(frame,(640,480))
        cv2.circle(frame,center,5,[0,0,255],1)
        cv2.imshow("Tracking", frame)
            #cv2.imshow("masked2", maskbgr)
            #cv2.imshow("masked3", hsvbgr)
        cv2.imshow("masked", masked)
          #  cv2.imshow("masked2", maskedbgr)
        cv2.waitKey(1)
            
    #else NO DETECTION
  
         

      
   
            
