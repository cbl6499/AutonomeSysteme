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

def main():
    np.set_printoptions(threshold=np.inf)

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

    # main sense-act cycle
    leftMotor = rightMotor = 2
    robot.setImageCycle(1)

    noChangeCount = 0

    while robot.isConnected():

        robot.fastSensingOverSignal()

        #while image is None:
        image = robot.getCameraImage()

        while image is None:
            if synchron:
                robot.stepsim(1)
            image = robot.getCameraImage()

        if image:
            qim = image
        else:
            qim = I.new("RGB", (64, 64), "white")

        # convert to opencv image
        open_cv_image = np.array(qim)
        # Convert RGB to BGR
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        cv2.imshow('Org', open_cv_image)
        #res = cv2.resize(open_cv_image, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
        next = cv2.cvtColor(open_cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Gray', next)

        if prvs is not None and next is not None:
            flow = cv2.calcOpticalFlowFarneback(prvs, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            
            mag = ang = 0
            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            ret, mask = cv2.threshold(mag, 2, 1, cv2.THRESH_BINARY)
            cv2.multiply(mag, mask, mag)
            cv2.multiply(ang, mask, ang)
            magNew = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
            angNew = ang*180/np.pi

            #Check if flow is detected
            if magNew.any() > 0 or angNew.any() > 0 or noChangeCount > 4:
                noChangeCount = 0
                prvs = next
                cv2.imshow('Flow arrow', draw_flow(next, flow,8))

                #|---+---+---+---+---|
                #|c00|c01|c02|c03|c04|
                #|---+---+---+---+---+
                #|c10|c11|c12|c13|c14|
                #|---+---+---+---+---+
                #|c20|c21|c22|c23|c24|
                #|---+---+---+---+---+
                #|c30|c31|c32|c33|c34|
                #|---+---+---+---+---+
                #|c40|c41|c42|c43|c44|
                #|---+---+---+---+---+
                
                c00 = np.mean(mag[0:11,0:11])
                c01 = np.mean(mag[0:11,12:23])
                c02 = np.mean(mag[0:11,24:40])
                c03 = np.mean(mag[0:11,41:52])
                c04 = np.mean(mag[0:11,53:64])

                c10 = np.mean(mag[12:23,0:11])
                c11 = np.mean(mag[12:23,12:23])
                c12 = np.mean(mag[12:23,24:40])
                c13 = np.mean(mag[12:23,41:52])
                c14 = np.mean(mag[12:23,53:64])

                c20 = np.mean(mag[24:40,0:11])
                c21 = np.mean(mag[24:40,12:23])
                c22 = np.mean(mag[24:40,24:40])
                c23 = np.mean(mag[24:40,41:52])
                c24 = np.mean(mag[24:40,53:64])

                c30 = np.mean(mag[41:52,0:11])
                c31 = np.mean(mag[41:52,12:23])
                c32 = np.mean(mag[41:52,24:40])
                c33 = np.mean(mag[41:52,41:52])
                c34 = np.mean(mag[41:52,53:64])

                c40 = np.mean(mag[53:64,0:11])
                c41 = np.mean(mag[53:64,12:23])
                c42 = np.mean(mag[53:64,24:40])
                c43 = np.mean(mag[53:64,41:52])
                c44 = np.mean(mag[53:64,53:64])

                #Idee: z.B. in c24 alle mag felder ein hoher wert -> roboter kommt von 
                # rechts -> geschwindigkeit leicht reduzieren und leicht in richtung c24 bewegen
                # Wenn z.b c14, c24, c34 -> fast zu stillstand abbremsen und in richtung c24 bewegen

                print('')

            else:
                noChangeCount+=1

        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
        
        robot.setMotorSpeeds(leftMotor, rightMotor)

        if synchron:
            robot.stepsim(1)


    robot.disconnect()

if __name__ == '__main__':
    main()
