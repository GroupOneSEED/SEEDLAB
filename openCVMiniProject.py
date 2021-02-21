from time import sleep
from picamera import PiCamera as Camera
import cv2 as cv
import cv2.aruco as aruco
import numpy as np

def cameraCalibration():
    camera = Camera(resolution=(1920, 1080), framerate=60)
    camera.iso = 100
    sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    awbGains = []
    for i in range(3):
        camera.capture('calibration.jpg')
        awbGains.append(camera.awb_gains)

    AverageGain = (sum(awbGains))/(len(awbGains))
    print("awb_gains:")
    print(awbGains)
    print("Average awb gain value: %f" % AverageGain)
    return(AverageGain)

def detectAruco(awb_gain):
    camera.awb_gains = awb_gains
    camera.capture('object.jpg')
    img = cv.imread('object.jpg')
    grayImg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary, parameters=arucoParameters)
    cornerList = list(corners)
    bottomRightX = int(cornerList[0][0][2][0]) #need to check this
    bottomLeftX = int(cornerList[0][0][3][0])  #need to check this
    bottomRightY = int(cornerList[0][0][2][0]) #need to check this
    bottomLeftY = int(cornerList[0][0][3][0])  #need to check this
    centerX = (bottomRightX + bottomLeftX) / (2)
    centerY = (bottomRightY + bottomLeftY) / (2)
    if((centerX > 960) and (centerY > 540)):
        return(0)
    elif((centerX < 960) and (centerY > 540)):
        return(1)
    elif((centerX < 960) and (centerY < 540)):
        return(2)
    elif((centerX > 960) and (centerY < 540)):
        return(3)
    else:
        return(4)

gain = cameraCalibration()
cameraCalibration()
detectAruco(gain)
    
