from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from servo_pid import *

DEBUG = True


cam = PiCamera()
cam.resolution=(300,150)
center = (300/2,150/2)
#640 480

cam.vflip = True
#cam.exposure_mode = 'sports'
#cam.framerate = 20
#cam.shutter_speed = 20000
cam.exposure_mode='antishake'
#cam.iso = 8
cam.awb_mode='auto'
#cam.vlip = True

#cam.shutter_speed = 0
#cam.exposure_mode = 'off'
#g= cam.awb_gains
#cam.awb_mode  = 'off'
#cam.awb_gains = g
#print(str(cam.shutter_speed))
#print(str(cam.awb_gains))
raw= PiRGBArray(cam)
time.sleep(0.1)


time.sleep(0.1)
cam.capture(raw,format="bgr")
frame = raw.array
#cam.saturation=100

raw.truncate(0)

kernal = np.ones((5,5),np.uint8)
kernal1 = np.ones((1,1),np.uint8)
kernals = np.ones((2,2),np.uint8)


#   PID
P = 0.31
I = 0.02
D = 0.0
servo = servo_pid(P, I, D, 0)


P = 0.31
I = 0.02
D = 0.0
stepper = stepper_pid(P, I, D, 0)

# deg to pixel ratio
ratio=1#/3.0



for fram in cam.capture_continuous(raw,format='bgr',use_video_port=True):
   
    frame =  fram.array
    
    frame = cv2.medianBlur(frame,3)
    
    timer = cv2.getTickCount()
    #frame = cv2.medianBlur(frame,5)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    #LASER
   # l = np.array([(int)(0.989*180),   0,   (int)(0.894*255)])
   #u=np.array([180,   (int)(0.119*255),   255])
   # l = np.array([(int)(0.139*180),   0,   (int)(0.597*255)])
  #  u=np.array([(int)(0.416*180),   (int)(0.175*255),   255])
    #l1 = np.array([0,   0,   (int)(0.894*255)])
    #u1=np.array([(int)(0.465*180),   (int)(0.119*255),   255])
  #  print(str(cam.shutter_speed))
  #  print(str(cam.awb_gains))
    
    #l = np.array([70,10,180])
    #u=np.array([85,255,255])
 #   mask = cv2.inRange(hsv,l,u)
    #mask1 = cv2.inRange(hsv,l1,u1)
    #mask = cv2.bitwise_or(mask,mask1)
    
    #mask=cv2.dilate(mask,kernal)
    #mask = cv2.erode(mask,kernal1)
  #  mmmask=mask
  #  masked = cv2.bitwise_and(frame,frame,mask=mask)
  #  im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  #  if len(contours) > 0:
 #       roundest = max(contours,key = lambda p: cv2.contourArea(p)/cv2.arcLength(p,True) if cv2.arcLength(p,True) != 0 else 0)
 #       cv2.drawContours(frame, roundest, 0, (255,0,0), 2)
    #LASER
    
    #X
    l = np.array([(int)(0.881*180),(int)(0.369*255),(int)(0.3*255)])
    u = np.array([180,255,255])
    
   # l1 = np.array([0,(int)(0.427*255),(int)(0.3*255)])
   # u1= np.array([(int)(0.094*180),255,255])
    mask = cv2.inRange(hsv,l,u)
   # mask1 = cv2.inRange(hsv,l1,u1)
   # mask = cv2.bitwise_or(mask,mask1)
    #mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernal)
    mask = cv2.dilate(mask,kernal)
    mask = cv2.erode(mask,kernals)
    masked = cv2.bitwise_and(frame,frame,mask=mask)
    
    im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(frame, contours, -1, (0,255,0), 1)
    #/X
    rect = 0
    if len(contours) > 0:
        contours = sorted(contours, key = cv2.contourArea, reverse = True)#################
        contours = contours[0:5]
        c = 0
        for con in contours:
            #x,y,w,h = cv2.boundingRect(con)
            rect = cv2.minAreaRect(con)
            w = rect[1][0]
            h = rect[1][1]
            #42
            #54
            if abs (cv2.contourArea(con)/(w*h) - 0.55) < 0.1 and w/h<1.5 and h/w<1.5:
                print("Calibration: "+str(cv2.contourArea(con)/(w*h)))
                c = con
                #print((rect[0][0]-center[0],rect[0][1]-center[1]))
                x = rect[0][0] - center[0]
                y = rect[0][1] - center[1]
                print("pos=" + str(y))
                servo.do_step(-y*ratio)
                stepper.do_step(x*ratio)
            #CONVEX HULL
             #   hull = cv2.convexHull(con,returnPoints = False)
             #   defects = cv2.convexityDefects(con,hull)
             #   if defects is not None:
             #       for i in range(defects.shape[0]):
             #           s,e,f,d = defects[i,0]
             #           start = tuple(con[s][0])
             #           end = tuple(con[e][0])
             #           far = tuple(con[f][0])
             #           cv2.line(frame,start,end,[255,0,0],2)
             #           cv2.circle(frame,far,1,[0,0,255],-1)
             #/CONVEX HULL
                #eps = 0.1*cv2.arcLength(con,True)
                #approx = cv2.approxPolyDP(con,eps,True)
                 
                break
        if type(c) == int:
            cv2.putText(frame, "FAIL", (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        else:
            #cv2.drawContours(frame,c,-1,(0,255,0),1)
            a = rect[0]
            if DEBUG:
                cv2.line(frame,((int)(a[0]),(int)(a[1])),((int)(len(frame[0])/2),(int)(len(frame)/2)),(0,255,0),1)
                     
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if DEBUG:
                cv2.drawContours(frame,[box],0,[0,255,0],2)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
    

            # Tracking success
        #p1 = (int(bbox[0]), int(bbox[1]))
        #p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        #cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    if DEBUG:
        cv2.putText(frame, "FPS : " + str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (50,170,50), 1)
    
    #frame = cv2.resize(frame,(640,480),interpolation=cv2.INTER_AREA)
        # Display result
    if DEBUG:
        frame = cv2.resize(frame,(640,480))
        cv2.imshow("Tracking", frame)
        cv2.imshow("masked", masked)
    
    raw.truncate(0)
        # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27 : break
