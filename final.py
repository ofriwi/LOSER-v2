import time, datetime
from math import atan, degrees, radians, tan

import cv2
import matplotlib.pyplot as plt
import numpy as np
import scipy
# Ofri
from Arduino_Stepper import Arduino_BT_Stepper
from Constants import *
from keras.models import load_model
from picamera import PiCamera
from picamera.array import PiRGBArray

# *** Partial mode ***
partial_mode = True
disable_mode = True
RPM = 20
deg_time = 1.0 / (RPM * 6.0) * 1000
send_time = 100
movement_end_time = datetime.datetime.now() 
movement_start_time = datetime.datetime.now()
movement_end_time = datetime.datetime.now()
direction = 1

sign = lambda a: (a>0) - (a<0)

X = 0
Y = 1

stepper_motor = Arduino_BT_Stepper()

ofriflag = 0

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
pNum = 1


model1 = load_model("net1.h5")
model2= load_model("netNoHidden10000.h5")
#model3 = load_model("zulzul.h5")

timeArr = []
def crop_rect(img,rect):
    angle = rect[2]
    rows,cols = img.shape[0],img.shape[1]
    
    
    M = cv2.getRotationMatrix2D(rect[0],angle,1)
    #print("ANGLE "+str(angle))
    img_rot = cv2.warpAffine(img,M,(cols,rows))
    #cv2.imshow("rot",img_rot)
    rect0 = (rect[0],rect[1],0.0)
    box = cv2.boxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]),M))[0]
    pts[pts<0]=0
    img_crop = img_rot[pts[1][1]:pts[0][1], pts[1][0]:pts[2][0]]
    if(img_crop.shape[0]==0 or img_crop.shape[1]==0):
        return img_crop
    if img_crop.shape[1]>img_crop.shape[0]:
       # M = cv2.getRotationMatrix2D((img_crop.shape[0]/2,img_crop.shape[1]/2),90,1)
        #img_crop = cv2.warpAffine(img_crop,M,(img_crop.shape[1],img_crop.shape[0]))
       #img_crop=(np.transpose(img_crop[0]),np.transpose(img_crop[1]),np.transpose(img_crop[2]))
       img_crop = np.rot90(img_crop)
    return img_crop
                                
DEBUG = True
template = cv2.imread("smaller2Temp.png",0)

h,w=template.shape[:2]
tempSize=(w,h)
cam = PiCamera()

cam.resolution=(500,250)#300150500250
center = (500/2,250/2)
cam.resolution=(512,256)#300150500250
center = (512/2,256/2)

cam.awb_gains=[1.3,1.3]
cam.rotation = 270
#cam.vflip = False
cam.hflip = False
#cam.exposure_mode = 'sports'
#cam.framerate = 20
cam.shutter_speed = 5000 #33186
#cam.exposure_speed = 5000
cam.exposure_mode='antishake'
#cam.iso = 8
#cam.awb_mode='auto'
#cam.vlip = True

#cam.shutter_speed = 0
#cam.exposure_mode = 'off'
#g= cam.awb_gains
#cam.awb_mode  = 'fluorescent'
#cam.contrast =-50
#cam.awb_gains = g
raw= PiRGBArray(cam)
time.sleep(0.1)


time.sleep(0.1)
cam.capture(raw,format="bgr")
frame = raw.array
#cam.saturation=100

raw.truncate(0)


kernal = np.ones((3,3),np.uint8)
kernal1 = np.ones((1,1),np.uint8)

try:
    for fram in cam.capture_continuous(raw,format='bgr',use_video_port=True):
        # Ofri
        
        shoot_time = datetime.datetime.now()
        
        # End
        frame =  fram.array
        
        timer = cv2.getTickCount()

        



        frame = (255-frame)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)     
     
        # l = np.array([40,10,190]) #good
        # u = np.array([100,200,220]) # good
        l = np.array([40,40,130]) #good
        u = np.array([100,200,220]) # good
        l = np.array([40,40,170]) #good
        u = np.array([100,200,220]) # good
        mask = cv2.inRange(hsv,l,u)
        
        
        mask = cv2.erode(mask,kernal1, iterations = 1)
        mask = cv2.dilate(mask,kernal, iterations = 4)
        
        
        frame = (255-frame)
        masked = cv2.bitwise_and(frame,frame,mask=mask)
   #     cv2.circle(frame, center, 3, l)
        cv2.imshow("masked", masked)
        im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        c = 0
        rect = 0
        if len(contours) > 0:
            contours = sorted(contours, key = cv2.contourArea, reverse = True)#################
            contours = contours[0:10] #TODO numberof contours
            maxPred = 0
            maxCon = 0
            for con in contours:
                x,y,w,h = cv2.boundingRect(con)
                rect = cv2.minAreaRect(con)
      
                LC = (rect[0][0]-rect[1][0]/2,rect[0][1]-rect[1][1]/2)
                WIDTH=rect[1][1]
                HEIGHT=rect[1][0]
                ANGLE = rect[2]
                
                if w!=0 and h!=0 and w*h > 400 and float(w)/ h < 1.5 and float(w)/ h > 0.5:
                    roi= crop_rect(frame,rect)
                 #   roi = crop_rect(frame,(rect[0],(rect[1][0]*1.5,rect[1][1]*1.5),rect[2]))
                  
                    
                        
                        
                  #  else:
                  #      continue
#                  
                    predict = 0
                    
                    cv2.imshow("roig",roi)
                    cv2.waitKey(1)
                    roiToSave = roi
                    if roi is not None :
                        lstTemp = []
                        
                     
                        roi=cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
                        roi2 = cv2.resize(roi,(30,45))
                        roi1 = cv2.resize(roi,(20,20))
                        
                        
                       # roi = roi-roi.mean()
                       # roi = roi / np.std(roi)
                        lstTemp.append(roi1)
                        imArr = np.array(lstTemp)
                        imArr = imArr.reshape((1, -1))
                        
                        
                        predict = (model1.predict(imArr))
                        roi2 = roi2-roi2.mean()
                        roi2 = roi2 / np.std(roi2)
                        lstTemp=[]
                        lstTemp.append(roi2)
                        imArr = np.array(lstTemp)
                        imArr = imArr.reshape((1, -1))
                        predict2 = model2.predict(imArr)
                    cv2.waitKey(1)
                    print(str(predict)+ ", " + str(predict2))
                    if predict > 0.95 and predict2 > 0.1:                  
                        #print("Net1: "+str(predict[0])+"    Net2: "+str(predict[1])+"    zulzul: "+str(predict[2]))                      
                        if predict > maxPred:
                            c = con
                            maxPred = predict            
        if type(c) == int:
            cv2.putText(frame, "LOST X", (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                
        else:
              
            rect = cv2.minAreaRect(c) #### NEW 
            a = rect[0]
            pos_from_mid = (a[0]-center[0], a[1]-center[1])
            if DEBUG_MODE or SERVO_DEBUG_MODE or STEPPER_DEBUG_MODE:
                print("pos=(" + str(pos_from_mid[0]) + ", " + str(pos_from_mid[1]) + ")")#str(a[0]) + ", " + str(a[1]))
           
            cv2.line(frame,((int)(a[0]),(int)(a[1])),((int)(len(frame[0])/2),(int)(len(frame)/2)),(0,255,0),1)
            cv2.drawContours(frame,[np.int0(cv2.boxPoints(rect))],0,[0,255,0],2)
            
            
            
            # Ofri
            #print('Ready?')
            #raw_input()
            ofriflag += 1
            if (ofriflag == 1):
                now_time = datetime.datetime.now() # Partial
                deg_moved = 0 # Partial
                if not disable_mode and partial_mode and shoot_time < movement_end_time: # Partial
                    print('Still moving')
                    deg_moved = direction * round(min(max(0, (now_time - movement_start_time).total_seconds()), (movement_end_time - movement_start_time).total_seconds(), (now_time - shoot_time).total_seconds(), (movement_end_time - shoot_time).total_seconds()) * 1000.0 / deg_time) # Partial
                    print(deg_moved)
                    print('shoot to end=' + str(round((movement_end_time - shoot_time).total_seconds() * 1000.0 / deg_time))) # Partial
                    print('shoot to now=' + str(round((now_time - shoot_time).total_seconds() * 1000.0 / deg_time))) # Partial
                    print('start to end=' + str(round((movement_end_time - movement_start_time).total_seconds() * 1000.0 / deg_time))) # Partial
                    print('start to now=' + str(round((now_time - movement_start_time).total_seconds() * 1000.0 / deg_time))) # Partial
                ofriflag = 0
                deg_to_move = pixels_to_degrees(pos_from_mid[0], X) - deg_moved # Partial : - deg_moved
                if disable_mode and shoot_time > movement_end_time:
                    print('to move: ' + str(deg_to_move+deg_moved) + 'move more: ' + str(deg_to_move))
                    direction = sign(deg_to_move) # Partial
                    stepper_motor.rotate(deg_to_move)
                    movement_start_time = datetime.datetime.now() + datetime.timedelta(microseconds=send_time * 1000) # Partial
                    movement_end_time = movement_start_time + abs(datetime.timedelta(microseconds=round(deg_to_move * deg_time * 1000))) # Partial
                    print('Good?')
            #raw_input() # for debugging


            # End
            
        timeArr.append((cv2.getTickCount() - timer)/cv2.getTickFrequency())
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        

        if DEBUG:
            cv2.putText(frame, "fps : " + str(int(fps)), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (50,170,50), 1)
     
     
      # i frame = cv2.resize(frame,(640,480))
        cv2.imshow("Tracking", frame)
        #cv2.imshow("masked2", maskbgr)
        #cv2.imshow("masked3", hsvbgr)
        cv2.imshow("masked", masked)
      #  cv2.imshow("masked2", maskedbgr)

        raw.truncate(0)
            # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 :
            
            print("Exit")
            stepper_motor.cleanup()
           # stepper.stepper.cleanup()#motor
          #  servo.servo.cleanup()#motor
            break;
            
except KeyboardInterrupt:
        raw.truncate(0)
        print("Exit")
        stepper_motor.cleanup()
        #plt.plot(timeArr)
        #plt.show()
       # stepper.stepper.cleanup()#motor
       # servo.servo.cleanup()#motor
