#Brent Misare, Perry Rodenbeck, Brett Schearer, Nick Zimkas
#EENG350

#Description: This python code implelemts functions and sequential loops to detect the angles
#and distances of seven aruco markers such that the robot can navigate around them for the final demo.

from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math as m
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0,50,0]

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
addr = 0x04

corners = 0

#The following block of code establishes the OpenCV image object.
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 60)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
ret, frame = cap.read()

#The following function is used to detect aruco marker id 0.
def Angle0():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 0 or (len(ids)==2 and ids[1] == 0)):
        cornerList = list(corners)
        if(ids[0] == 0):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 0):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D


#The following function is used to detect aruco marker id 1.
def Angle1():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 1 or (len(ids)==2 and ids[1] == 1)):
        cornerList = list(corners)
        
        if(ids[0] == 1):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 1):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D


#The following function is used to detect aruco marker id 2.
def Angle2():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 2 or (len(ids)==2 and ids[1] == 2)):
        cornerList = list(corners)
        
        if(ids[0] == 2):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 2):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D


#The following function is used to detect aruco marker id 3.
def Angle3():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 3 or (len(ids)==2 and ids[1] == 3)):
        cornerList = list(corners)
        
        if(ids[0] == 3):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 3):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D


#The following function is used to detect aruco marker id 4.
def Angle4():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 4 or (len(ids)==2 and ids[1] == 4)):
        cornerList = list(corners)
        
        if(ids[0] == 4):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 4):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D



#The following function is used to detect aruco marker id 5.
def Angle5():
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None and (ids[0] == 5 or (len(ids)==2 and ids[1] == 5)):
        cornerList = list(corners)
        
        if(ids[0] == 5):
            bottomRightY = int(cornerList[0][0][2][1])
            bottomLeftY = int(cornerList[0][0][3][1])
            topRightY = int(cornerList[0][0][1][1])
            topLeftY = int(cornerList[0][0][0][1])
            bottomRightX = int(cornerList[0][0][2][0])
            bottomLeftX = int(cornerList[0][0][3][0])
            topRightX = int(cornerList[0][0][1][0])
            topLeftX = int(cornerList[0][0][0][0])
            
        if(len(ids) == 2 and ids[1] == 5):
            bottomRightY = int(cornerList[1][0][2][1])
            bottomLeftY = int(cornerList[1][0][3][1])
            topRightY = int(cornerList[1][0][1][1])
            topLeftY = int(cornerList[1][0][0][1])
            bottomRightX = int(cornerList[1][0][2][0])
            bottomLeftX = int(cornerList[1][0][3][0])
            topRightX = int(cornerList[1][0][1][0])
            topLeftX = int(cornerList[1][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        #f = 1224.223207208012 #4X4
        f = 1059.2105263157894
        #H = 3.8189  6X6
        H = 5.472  #4X4
        #H = 1.732 #Height/width of aruco marker being used in inches (44mm).
        D = (H * f)/(PY)
        #FOV = 63.9
        distFromCenter = ((H / PY) * (640 - centerX))
        #distFromCenter = distFromCenter / 1.06042
        phi = m.atan((distFromCenter / D)) * (180 / m.pi)
        phi = phi / 1.015461178
        
        
    else:
        phi = 0
        D = 0
       
        
    return phi, ids, D

    
i = 0


#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 0.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle0()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 6):
        break


i = 0

sleep(1)

#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 1.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle1()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break


i = 0

sleep(2)
 
#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 2.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle2()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break
        
        
i = 0

sleep(2)

#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 3.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle3()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break
        
        
        
i = 0
    
sleep(2)

#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 4.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle4()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break
        
        
sleep(2)
        
i = 0

#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 5.
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle5()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break
    
    
sleep(2)
i = 0

#The following while lopp is used to send the appropriate angle and distance value associated with aruco marker id 0 (last marker, marker #7).
while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle0()
    print(phi)
    while True:
        try:
            if (D != 0):
                lcd.message = "Angle: %.2f     "  %phi
                bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*D))])
            break
        except:
            print("I2C Error")
            lcd.message = "I2C Error"
    if ids is not None:
        i += 1 
    if (ids is None) and (i > 8):
        break
    

 
cap.release()


