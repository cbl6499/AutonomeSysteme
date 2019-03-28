# -*- coding: utf-8 -*-
import time
import cv2
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from pprint import pprint
from PIL import Image as I
import matplotlib.pyplot as plt

from PIL import Image as I

maxVel = 120 * np.pi / 180  # 4/3 of a full wheel turn
s = True

def graduateMagnitudeValuesInSections(mag):
    """
    Graduates the 64x64 matrix into a 5x5 matrix by creating the
    mean of 12x12 pixels.
    :return: 5x5 matrix of means
    """
    c00 = np.mean(mag[0:11, 0:11])
    c01 = np.mean(mag[0:11, 12:23])
    c02 = np.mean(mag[0:11, 24:40])
    c03 = np.mean(mag[0:11, 41:52])
    c04 = np.mean(mag[0:11, 53:64])

    c10 = np.mean(mag[12:23, 0:11])
    c11 = np.mean(mag[12:23, 12:23])
    c12 = np.mean(mag[12:23, 24:40])
    c13 = np.mean(mag[12:23, 41:52])
    c14 = np.mean(mag[12:23, 53:64])

    c20 = np.mean(mag[24:40, 0:11])
    c21 = np.mean(mag[24:40, 12:23])
    c22 = np.mean(mag[24:40, 24:40])
    c23 = np.mean(mag[24:40, 41:52])
    c24 = np.mean(mag[24:40, 53:64])

    c30 = np.mean(mag[41:52, 0:11])
    c31 = np.mean(mag[41:52, 12:23])
    c32 = np.mean(mag[41:52, 24:40])
    c33 = np.mean(mag[41:52, 41:52])
    c34 = np.mean(mag[41:52, 53:64])

    c40 = np.mean(mag[53:64, 0:11])
    c41 = np.mean(mag[53:64, 12:23])
    c42 = np.mean(mag[53:64, 24:40])
    c43 = np.mean(mag[53:64, 41:52])
    c44 = np.mean(mag[53:64, 53:64])

    magNew = np.array([[c00, c01, c02, c03, c04],
                       [c10, c11, c12, c13, c14],
                       [c20, c21, c22, c23, c24],
                       [c30, c31, c32, c33, c34],
                       [c40, c41, c42, c43, c44]
                       ])
    return magNew

def calculateMotorValues(magnitude, angle, proxSensors):

        mag = graduateMagnitudeValuesInSections(magnitude)

        leftMag = np.mean(mag[:, 0])
        rightMag = np.mean(mag[:, 4])

        print leftMag
        print rightMag

        leftMotor = 2.0 - ((2.0 / 150) * rightMag)
        rightMotor = 2.0 - ((2.0 / 150) * leftMag)

        return leftMotor, rightMotor


def main():
    stepCounter = 0
    synchron = False

    robot = EPuckVRep('ePuck', port=19999, synchronous=synchron)

    if synchron:
        robot.startsim()

    robot.enableAllSensors()
    robot.enableCamera()
    robot.setImageCycle(1)
    robot.setSensesAllTogether(True)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    # get initial image from robot
    resolX, resolY = 64, 64
    # get initial image
    open_cv_image = getOpenCVCameraImage(robot.getCameraImage(), resolX, resolY)
    # Convert the new image to gray
    prvsGray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)


    magNew = angNew = mag = ang = 0
    noChangeCount = 0
    first = True

    # main sense-act cycle
    while robot.isConnected():

        # Get sensor values and camera image
        robot.fastSensingOverSignal()
        proxSensors = robot.getProximitySensorValues()
        open_cv_image = getOpenCVCameraImage(robot.getCameraImage(), resolX, resolY)
        noDetectionDistance = 0.05 * robot.getS()
        # Convert the new image to gray
        nextGray = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('Gray', nextGray)

        if prvsGray is not None and next is not None:
            flow = cv2.calcOpticalFlowFarneback(prvsGray, nextGray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            # get angle and magnitude
            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            ret, mask = cv2.threshold(mag, 2, 1, cv2.THRESH_BINARY)
            cv2.multiply(mag, mask, mag)
            cv2.multiply(ang, mask, ang)
            mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            ang = ang * 180 / np.pi



            # Check if flow is detected
            if np.any(mag[:, :] > 0) or np.any(ang[:, :] > 0) or noChangeCount > 4 or first == True:
                first = False
                noChangeCount = 0
                magNew = mag
                angNew = ang
                prvsGray = nextGray
            else:
                noChangeCount += 1

            # show image
            hsv = np.zeros_like(open_cv_image)
            hsv[..., 1] = 255
            hsv[..., 0] = angNew * 180 / np.pi / 2
            hsv[..., 2] = magNew
            rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            res = cv2.resize(rgb, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)
            cv2.imshow('flow', res)

        leftMotor, rightMotor = calculateMotorValues(magNew, angNew, proxSensors)

        # to abort controller
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

        robot.setMotorSpeeds(leftMotor, rightMotor)

        if synchron:
            robot.stepsim(1)

    robot.disconnect()


def getOpenCVCameraImage(image, resolX, resolY):
    if image:
        qim = image
    else:
        qim = I.new("RGB", (resolX, resolY), "white")

    # convert to opencv image
    open_cv_image = np.array(qim)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    #cv2.imshow('Org', open_cv_image)

    return open_cv_image


if __name__ == '__main__':
    main()