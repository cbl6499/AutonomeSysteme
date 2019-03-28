# -*- coding: utf-8 -*-
"""
Created on Wed 7 19:41:16 2018
simple optical flow test for BasicEPuck.ePuckVRep
make sure to start first VRep scene with boxPushingExerciseCSynch.ttt, or
another scene with a ePuckS5SynchV12, then this program


@author: hans vollbrecht
"""
import time
import cv2
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from PIL import Image as I
import matplotlib.pyplot as plt

noChangeCount = 0

def main():
    synchron = True

    robot = EPuckVRep('ePuck', port=19999, synchronous=synchron)

    robot.enableAllSensors()
    robot.enableCamera()
    robot.setSensesAllTogether(True)   # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    if synchron:
        robot.startsim()

    image = robot.getCameraImage()
    if image:
        qim = image
    else:
        qim = I.new("RGB", (64, 64), "white")

    # convert to opencv image
    open_cv_image = np.array(qim)
    # Convert RGB to BGR
    open_cv_image = open_cv_image[:, :, ::-1].copy()
    #res = cv2.resize(open_cv_image, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)

    prvs = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(open_cv_image)
    hsv[..., 1] = 255
    # main sense-act cycle
    leftMotor = rightMotor = 0

    while robot.isConnected():

        robot.fastSensingOverSignal()

        image = robot.getCameraImage()

        while image is None:
            image = robot.getCameraImage()

        if image:
            qim = image
        else:
            qim = I.new("RGB", (64, 64), "white")

        # convert to opencv image
        open_cv_image = np.array(qim)
        # Convert RGB to BGR
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        #res = cv2.resize(open_cv_image, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
        next = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)

        if prvs is not None and next is not None:
            flow = cv2.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            #print("mag: ", mag.size )
            #print("ang: ", ang)
            ret, mask = cv2.threshold(mag, 2, 1, cv2.THRESH_BINARY)

            cv2.multiply(mag, mask, mag)
            cv2.multiply(ang, mask, ang)
            mag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            newMag = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            newAng = ang * 180 / np.pi / 2

            global countStep

            if newMag.any() > 0 or newAng.any() > 0 or countStep > 4:
                countStep = 0
                prvs = next

            else:
                countStep+=1


            #show image
            hsv[..., 0] = ang * 180 / np.pi / 2
            hsv[..., 2] = mag
            rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            res = cv2.resize(rgb, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)
            cv2.imshow('frame2', res)


        k = cv2.waitKey(5) & 0xff
        if k == 27:
            break
        #elif k == ord('s'):
        #    print('saved')
        #    cv2.imwrite('opticalfb.png', res)
        #    cv2.imwrite('opticalhsv.png', rgb)
        #    plt.hist(np.reshape(mag, 256*256), 50, normed=1, facecolor='g', alpha=0.75)

        #    plt.xlabel('flowMagnitude')
        #    plt.ylabel('Probability')
        #    plt.title('Histogram of optical flow magnitude')

        #    plt.axis([0, 15, 0, 0.1])
        #    plt.grid(True)
        #    plt.show()
        #    plt.pause(1.0)
        #prvs = next
        robot.setMotorSpeeds(leftMotor, rightMotor)

        if synchron:
            robot.stepsim(5)


    robot.disconnect()

if __name__ == '__main__':
    main()
