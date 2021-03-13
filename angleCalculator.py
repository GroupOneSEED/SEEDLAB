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

def Angle():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while(True):
        ret, frame = cap.read()
        grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
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
            f = 1205.54272517321
            H = 1.732
            D = (H * f)/(PY)
            print(D)
            #FOV = 63.9
            distFromCenter = (H / PY) * (abs(640 - centerX))
            #distFromCenter = distFromCenter / 1.06042
            print(distFromCenter)
            phi = m.atan((distFromCenter / D)) * (180 / m.pi)
            phi = phi / 1.015461178
            lcd.message = "Angle: %.2f     "  %phi
    cap.release()
    
Angle()  
