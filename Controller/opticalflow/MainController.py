# -*- coding: utf-8 -*-
import time
import cv2
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from PIL import Image as I
import matplotlib.pyplot as plt

from behaviourDriveForward import DriveForward
from behaviourAvoidRobot import AvoidRobot
from behaviourUnwedgeFromWall import UnwedgeFromWall

from PIL import Image as I 

def main():
    stepCounter = 0
    synchron = False

    robot = EPuckVRep('ePuck', port=19999, synchronous=synchron)

    if synchron:
        robot.startsim()

    robot.enableAllSensors()
    robot.enableCamera()
    robot.setImageCycle(1)
    robot.setSensesAllTogether(True)   # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    #get initial image from robot
    resolX, resolY = 64, 64
    #get initial image
    open_cv_image = getOpenCVCameraImage(robot.getCameraImage(),resolX,resolY)
    #Convert the new image to gray
    prvsGray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)

    #subsumption behaviour list, order defines priorities!
    behaviours = [UnwedgeFromWall(), AvoidRobot(),DriveForward()]
    
    magNew = angNew = mag = ang = 0
    noChangeCount = 0
    first = True

    # main sense-act cycle
    while robot.isConnected():
        
        #Get sensor values and camera image
        robot.fastSensingOverSignal()
        proxSensors = robot.getProximitySensorValues()
        open_cv_image = getOpenCVCameraImage(robot.getCameraImage(),resolX,resolY)
        noDetectionDistance = 0.05 * robot.getS()
        #Convert the new image to gray
        nextGray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('Gray', nextGray)

        if prvsGray is not None and next is not None:
            flow = cv2.calcOpticalFlowFarneback(prvsGray, nextGray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            
            #get angle and magnitude
            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            ret, mask = cv2.threshold(mag, 2.25, 1, cv2.THRESH_BINARY)
            cv2.multiply(mag, mask, mag)
            cv2.multiply(ang, mask, ang)
            mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            ang = ang*180/np.pi

            #Check if flow is detected
            if np.any(mag[:,:] > 0) or np.any(ang[:,:] > 0) or noChangeCount > 4 or first == True or behaviour.getState() == 'unwedgeFromWall':
                first = False
                noChangeCount = 0
                magNew = mag
                angNew = ang
                prvsGray = nextGray

                #cv2.imshow('Flow arrow', draw_flow(nextGray, flow,6))
            else:
                noChangeCount+=1

        behaviour = next(behavior for behavior in behaviours if behavior.applicable(magNew, angNew, proxSensors, noDetectionDistance))

        print("behaviour: ", behaviour.getState())
        leftMotor, rightMotor = behaviour.calculateMotorValues(magNew, angNew, proxSensors)

        #to abort controller
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
        
        robot.setMotorSpeeds(leftMotor, rightMotor)

        if synchron:
            robot.stepsim(1)

    robot.disconnect()

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def getOpenCVCameraImage(image,resolX,resolY):
    if image:
        qim = image
    else:
        qim = I.new("RGB", (resolX,resolY), "white")

    # convert to opencv image
    open_cv_image = np.array(qim)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    #cv2.imshow('Org', open_cv_image)

    return open_cv_image

if __name__ == '__main__':
    main()
