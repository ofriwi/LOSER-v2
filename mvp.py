from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import scipy
from keras.models import load_model

from Constants import *
# from servo_pid import *
from Arduino_DC import Arduino_BT_DC
from PID import PID

model = load_model("net1.h5")

def crop_rect(img,rect):
    angle = rect[2]
    rows,cols = img.shape[0],img.shape[1]
  #  rowsX,colsX = ((rect[1][0]),(rect[1][1]))
    #if rect[1][0] > rect[1][1]:
   # M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    M = cv2.getRotationMatrix2D(rect[0],angle,1)
  #  else:
     #   M = cv2.getRotationMatrix2D(rect[0],-3.14/2+angle,1)
    img_rot = cv2.warpAffine(img,M,(cols,rows))
    #cv2.imshow("rot",img_rot)
    rect0 = (rect[0],rect[1],0.0)
    box = cv2.boxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]),M))[0]
    pts[pts<0]=0
    img_crop = img_rot[pts[1][1]:pts[0][1], pts[1][0]:pts[2][0]]
    return img_crop


# normalize values from pid to be between 50 to 255
def normalize(input, min_value, max_value, cut_off):
    if DEBUG_MODE:
        print('unnormalized: ' + str(input))
    norm = abs(input)
    if norm < cut_off:
        return 0
    norm = norm + min_value - cut_off
    norm = min(norm, max_value)
    if input < 0:
        norm = -norm
    return norm


DEBUG = True
template = cv2.imread("smaller2Temp.png",0)

h,w=template.shape[:2]
tempSize=(w,h)
cam = PiCamera()

cam.resolution=(500,250)#300150500250
center = (500/2,250/2)
#640 480

cam.rotation = 270
#cam.vflip = False
cam.hflip = True
#cam.exposure_mode = 'sports'
#cam.framerate = 20
cam.shutter_speed = 5000 #33186
#cam.exposure_speed = 5000
cam.exposure_mode='antishake'
#cam.iso = 8
cam.awb_mode='auto'
#cam.vlip = True

#cam.shutter_speed = 0
#cam.exposure_mode = 'off'
#g= cam.awb_gains
#cam.awb_mode  = 'fluorescent'
#cam.contrast =-50
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



#   PID
stepper_mode = True
dc_normalize = True
if stepper_mode:
    min_value = 0
    max_value = 40
    cut_off = 0
    Kp = 0.08
else:
    min_value = 50
    max_value = 255
    cut_off = 5
    Kp = 0.18


Ki = 0
Kd = 0
dc_pid = PID(Kp, Ki, Kd)
dc_pid.SetPoint = 0
dc_motor = Arduino_BT_DC()
#pid.setWindup(!!!!!!!!!!);






kernal = np.ones((3,3),np.uint8)
kernal1 = np.ones((1,1),np.uint8)
kernals = np.ones((1,1),np.uint8)
try:
    for fram in cam.capture_continuous(raw,format='bgr',use_video_port=True):
       
        frame =  fram.array
        
        frame = cv2.medianBlur(frame,3)
        
        timer = cv2.getTickCount()
        #frame = cv2.medianBlur(frame,5)
        frame = (255-frame)
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
        #l = np.array([158,94,76]) #3.1.2018 made filter weak for conv
        #u = np.array([180,255,255])
        #l = np.array([150,87,60]) doesnt filter out library desk
     
        #l = np.array([78,60,100]) library
        l = np.array([78,20,100]) #classroom  S20->30 azulay
       # l = np.array([65,30,100]) azulzul
##        u = np.array([90,149,200])#H95 is tight 185 149 library
        u = np.array([90,149,220])#90 100  V255->220 azulay
       
     #   u = np.array([100,149,220]) azulzul
        
       # l1 = np.array([0,(int)(0.427*255),(int)(0.3*255)])
       # u1= np.array([(int)(0.094*180),255,255])
        mask = cv2.inRange(hsv,l,u)
       # mask1 = cv2.inRange(hsv,l1,u1)
       # mask = cv2.bitwise_or(mask,mask1)
        #mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernal)
        mask = cv2.erode(mask,kernal1, iterations = 3)
        mask = cv2.dilate(mask,kernal, iterations = 3)
        
        
        frame = (255-frame)
        masked = cv2.bitwise_and(frame,frame,mask=mask)
        
        im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame, contours, -1, (0,255,0), 1)
        #/X
        c = 0
        rect = 0
        if len(contours) > 0:
            contours = sorted(contours, key = cv2.contourArea, reverse = True)#################
            contours = contours[0:10] #TODO numberof contours
            
            maxPred = 0;
            for con in contours:
                
                #area ratio filter
                x,y,w,h = cv2.boundingRect(con)
              #  rect = cv2.minAreaRect(con)
                #w = rect[1][0]
                #h = rect[1][1]
                #if h!=0 and w!=0:
                #    area2RectRatio = cv2.contourArea(con)/(w*h)
               # else:
               #     area2RectRatio = 10
               #/area ratio filter
        
                #box = cv2.boxPoints(rect)
               # box = np.int0(box)
                
                rect = cv2.minAreaRect(con)
                
              #  print("RECTANGLE                                "+str(rect[1])) #TODO collapse fun minAreaRect
                if w!=0 and h!=0 :#TODO filter long rects using rect[1]
                 
                     #CLASSIC ROI
                    #x=max(x,3)
                    #y=max(y,3)
                    #roi = frame[y-3:y+he+6,x-3:x+wi+6]                                     
                   #/CLASSIC ROI
                    
                    cv2.drawContours(frame,[np.int0(cv2.boxPoints(rect))],0,[0,255,0],2)
                    roi = crop_rect(frame,rect)
                    
                    if roi.shape[0]<10 or roi.shape[1]<10:
                        continue
                    roi = cv2.resize(roi,((int)(template.shape[1]*1.2),(int)(template.shape[0]*1.2)))

                    #cv2.imshow("seemek",roi)
                    roi=cv2.resize(roi,tempSize) #TODO RESIZE AFTER THRESH
                    roi = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
                    #cv2.imshow("seemek",roi)
                    #ret,roi = cv2.threshold(roi,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) ###BINARY IMG
                    
                    
             #   template = cv2.blur(template,(3,3))
                    #template = cv2.resize(template,((int)(roi.shape[1]/1.1),(int)(roi.shape[0]/1.1)))
                  #  roi = cv2.resize(roi,((int)(template.shape[1]*1.2),(int)(template.shape[0]*1.2)))
                    #roi = cv2.blur(roi,(4,4))
                    
                   # roi = cv2.medianBlur(roi,13)
                   # roi = cv2.erode(roi, kernals)
##                    tempMatch = cv2.matchTemplate(roi,template,cv2.TM_SQDIFF)
##                    min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(tempMatch)
                    predicted = 0
                    
                    if roi is not None :
                        lstTemp = []
                        roi = cv2.resize(roi,(20,20))
                        roi_flat = np.asarray([pix for pix in roi])
                        lstTemp.append(roi)
                        imArr = np.array(lstTemp)
                        imArr = imArr.reshape((1, -1))
                        predicted = model.predict(imArr)
                      
                   # roi = cv2.resize(roi,((int)(roi.shape[1]*5),(int)(roi.shape[0]*5)))
                #    cv2.imshow("roi",roi)
                   # template = cv2.resize(template,((int)(template.shape[1]*5),(int)(template.shape[0]*5)))
                 #   cv2.imshow("template",template)
                    #if min_val < 60000000:#4560
                    if predicted >0.9:
                     #cv2.drawContours(frame,[np.int0(cv2.boxPoints(rect))],0,[255,0,0],2)
                        #cv2.putText(frame, str(np.amax(tempMatch)), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,0,255), 1)
                        print(predicted)

                        # print("tempMatch    "+str(min_val/10000000))
                        #c = con  
                        #break
                        if predicted > maxPred:
                            c = con
                            maxPred = predicted
                            break;
                   
               # if abs (area2RectRatio - 0.42) < 0.2 and w/h<1.5 and h/w<1.5:
                #    c = con
                    
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
                     
            
        if type(c) == int:
            cv2.putText(frame, "FAIL", (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            # PID
            if not stepper_mode:
                dc_motor.move(0)
                
        else:
                #cv2.drawContours(frame,c,-1,(0,255,0),1)
                #a = rect[0]
          #  a=(x+wi/2,y+he/2)
            rect = cv2.minAreaRect(c) #### NEW 
            a = rect[0]
            pos_from_mid = (a[0]-center[0], a[1]-center[1])
            if DEBUG_MODE or SERVO_DEBUG_MODE or STEPPER_DEBUG_MODE:
                print("pos=(" + str(pos_from_mid[0]) + ", " + str(pos_from_mid[1]) + ")")#str(a[0]) + ", " + str(a[1]))
            # PID
          #  servo.do_step(-pos_from_mid[1]*ratio)#motor
            dc_pid.update(pos_from_mid[0])
            dc_output = dc_pid.output
            if dc_normalize:
                dc_output = normalize(dc_output, min_value, max_value, cut_off)
            dc_motor.move(dc_output)
            
            cv2.line(frame,((int)(a[0]),(int)(a[1])),((int)(len(frame[0])/2),(int)(len(frame)/2)),(0,255,0),1)
                         
               # box = cv2.boxPoints(rect)
              #  box = np.int0(box)
            
                   # cv2.drawContours(frame,[box],0,[0,255,0],2)
            cv2.drawContours(frame,[np.int0(cv2.boxPoints(rect))],0,[0,255,0],2)
            
            #  cv2.rectangle(frame,(x,y),(x+wi,y+he),[0,255,0],2)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        

        if DEBUG:
            cv2.putText(frame, "FPS : " + str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (50,170,50), 1)
     
        frame = cv2.resize(frame,(640,480))
        cv2.imshow("Tracking", frame)
            #cv2.imshow("masked", masked)
        
        raw.truncate(0)
            # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 :
            print("Exit")
            dc_motor.cleanup()#motor
            #servo.servo.cleanup()#motor
            break;
            
except KeyboardInterrupt:
        print("Exit")
        dc_motor.cleanup()
       # servo.servo.cleanup()#motorcton