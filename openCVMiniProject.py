from time import sleep
from picamera import PiCamera as Camera
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
camera = Camera(resolution=(1920, 1080), framerate=60)
def cameraCalibration():
    camera.iso = 100
    sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
	gain = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = gain
    for i in range(3):
        camera.capture('calibration.jpg')

    return(gain)

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
    bottomRightY = int(cornerList[0][0][2][1]) #need to check this
    bottomLeftY = int(cornerList[0][0][3][1])  #need to check this
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
quadrant = detectAruco(gain)
    
