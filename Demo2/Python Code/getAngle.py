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

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
#cap.set(cv.CAP_PROP_EXPOSURE, -4)
ret, frame = cap.read()

def Angle():
#    print("check1")
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #cv.imshow("window", grayImg)
    #if (cv.waitKey(1) & 0xFF == ord('q')):
        #cv.destroyAllWindows()
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    if ids is not None:
        cornerList = list(corners)
        bottomRightY = int(cornerList[0][0][2][1])
        bottomLeftY = int(cornerList[0][0][3][1])
        topRightY = int(cornerList[0][0][1][1])
        topLeftY = int(cornerList[0][0][0][1])
        bottomRightX = int(cornerList[0][0][2][0])
        bottomLeftX = int(cornerList[0][0][3][0])
        topRightX = int(cornerList[0][0][1][0])
        topLeftX = int(cornerList[0][0][0][0])
        PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
        #f = 1772.4680966603312 #This value for 1920x1080
        #f = 1161.9310344827586 #This value corresponds to a resolution of 1280x720
        #f = 1205.54272517321 #This corresponds to small marker of 44mm
        #f = 1225.4837780512712 6X6
        f = 1224.223207208012 #4X4
        #H = 3.8189  6X6
        H = 3.58268   #4X4
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

while(1):
    ret, frame = cap.read()
    phi, ids, D = Angle()
    print(D)
    #cv.imshow("window", frame)
#    D = Distance(frame)
#    ids = ID(frame)
#    Dint = int(D)
#    D3 = D*10-Dint*10
#    print("check2")
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
    if (ids is None) and (i == 6):
        break

    sleep(0)
cap.release()

