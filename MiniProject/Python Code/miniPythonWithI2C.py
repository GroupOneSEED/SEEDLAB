from time import sleep
import smbus
from picamera import PiCamera as Camera
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

camera = Camera(resolution=(1920, 1080), framerate=60)

lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0,50,0]

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
addr = 0x04

def writeNumber(value):
    number = bus.write_byte(addr, value)
    return -1

def readNumber():
    number = bus.read_i2c_block_data(addr, 6)
    return number

def cameraCalibration():
    camera.iso = 100
    sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    gain = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = gain
    camera.capture('calibration.jpg')

    return(gain)

def detectAruco(awb_gain):
    camera.awb_gains = awb_gain
    camera.capture('object.jpg')
    img = cv.imread('object.jpg')
    grayImg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary, parameters=arucoParameters)
    if ids is None:
        return(4)
    cornerList = list(corners)
    bottomRightX = int(cornerList[0][0][2][0]) 
    bottomLeftX = int(cornerList[0][0][3][0])  
    bottomRightY = int(cornerList[0][0][2][1]) 
    bottomLeftY = int(cornerList[0][0][3][1])  
    centerX = (bottomRightX + bottomLeftX) / (2)
    centerY = (bottomRightY + bottomLeftY) / (2)
    if((centerX > 960) and (centerY < 540)):
        return(0)
    elif((centerX < 960) and (centerY < 540)):
        return(1)
    elif((centerX < 960) and (centerY > 540)):
        return(2)
    elif((centerX > 960) and (centerY > 540)):
        return(3)
    else:
        return(4)



gain = cameraCalibration()

camera.start_preview()

quadrant = detectAruco(gain)

lcd.message = "Move to %dpi/2     "  %quadrant

while True:
    
    while True:
        try:
            writeNumber(quadrant)
            break
        except:
            print("I2C Error1")
            lcd.message = "I2C Error"
            camera.stop_preview()
            sleep(2)
            camera.start_preview()
    sleep(.1)

    while True:
        try:
            current = readNumber()
            
            break
        except:
            print("I2C Error2")
            lcd.message = "I2C Error"
            camera.stop_preview()
            sleep(2)
            camera.start_preview()

    currentNum = float(str(current[3]) + str(current[2]) + str(current[1]) + str(current[0]))

    currentNum = float((currentNum+40)/800)

    lcd.message = "Move to %dpi/2\nNow at %.2fpi/2        " % (quadrant,currentNum)
    quadrant = detectAruco(gain)

    lcd.message = "Move to %dpi/2     "  %quadrant



