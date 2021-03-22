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
    #The following three lines start the video capture at 1280x720
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    while(True):
        #The following five lines of code take the images "frames" from the video capture, convert it to a grayscale image, define aruco parameters, and then search for the aruco marker
        ret, frame = cap.read()
        grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
        arucoParameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
        #The following if conditional runs if a marker is detected.
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
            #PY is the average height of the marker in the image in pixels
            PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) 
            #CenterX is the center x coordinate of  the aruco marker in the image
            centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)
            #f = 1772.4680966603312 #This value for 1920x1080
            #This value for 1280x720
            f = 1205.54272517321 
            #H is the height of the marker in inches
            H = 1.732
            D = (H * f)/(PY)
            print(D)
            #FOV = 63.9
            distFromCenter = (H / PY) * (640 - centerX)
            print(distFromCenter)
            phi = m.atan((distFromCenter / D)) * (180 / m.pi)
            phi = phi / 1.015461178
            lcd.message = "Angle: %.2f     "  %phi
        else:
            lcd.message = "No maker found!"
    cap.release()
    
Angle()  
