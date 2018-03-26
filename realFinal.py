from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt


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


cam = PiCamera()

cam.resolution=(500,250)#300150500250
center = (500/2,250/2)

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



#   PID
P = 0.13
I = 0.0
D = 0.0
#servo = servo_pid(P, I, D, 0) #motor


#P = 0.1
#I = 0.06#-0.07
#D = 0.03

P = 0.13
I = 0.003
D = 0.02
#stepper = stepper_pid(P, I, D, 0) #motor
# deg to pixel ratio
ratio=1#/3.0
x_height=1
predict=0
predict2=0

hog = get_hog()

kernal = np.ones((3,3),np.uint8)
kernal1 = np.ones((1,1),np.uint8)

for fram in cam.capture_continuous(raw,format='bgr',use_video_port=True):
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
        distance = 8667*pow(x_height,-0.8693)
    #else NO DETECTION
  
         

      
   
            
