# -*- coding: utf-8 -*-
"""
Created on Wed Aug 5 16:40:22 2017
simple test for BasicEPuck.ePuckVRep
make sure to start first VRep scene ePuckBasicS5.ttt, then this program


@author: hans vollbrecht
"""
import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from PIL import Image as I


# behavior matrices for numPy version
avoidFrontCollisionMatrix = np.asarray([[1, 2, 0, 0],
                                        [-1, -2, 0, 0]])

boxPushingMatrix = np.asarray([[0, 0, 0.0, 1.0],
                               [1.0, 0.0, 0, 0]])

maxVel = 120 * np.pi / 180  # 4/3 of a full wheel turn

baseVelocity = np.asarray([maxVel/2.0, maxVel/2.0])

# state: 'search', 'approach', 'push', 'unwedge'  a box
state = 'search'

resolX, resolY = 64, 64


def detectBox(image, resolX, resolY, xCenter):
    """
    looks in current image for a black blob on a red background, from left to right
    :param
            resolX, resolY: int
                image resolution in pixel
            image: PIL.Image
                a rgb image with black blobs on red background
            xCenter: [int]
                the center of the image: result of the function

    :return: true,  if black blob found
    """
    minBlobWidth = 5
    xStart = -1
    for y in range(resolY):
        blobwidth = 0
        for x in range(resolX):
            pixel = image.getpixel((x, y))
            if pixel == (0, 0, 0):  # black pixel: a box!
                blobwidth += 1
                if  blobwidth == 1:
                    xStart = x
            else:
                if blobwidth >= minBlobWidth:
                    xCenter[0] = xStart + blobwidth/2
                    # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                    return True
                elif blobwidth > 0:
                    blobwidth = 0
        if blobwidth >= minBlobWidth:
            xCenter[0] = xStart + blobwidth/2
            # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
            return True

    return False

def normalizeDistances(distances, noDetectionDistance):
    """
    ePuck has higher proximity values for lower distances; also normalize to [0,1]
    :param distances:  numpy float array
        distances measured by proximity sensors; in meters
    :param noDetectionDistance:  float
        maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
    :return: distances: numpy float array
        normalized distances measured by proximity sensors
    """
    distances = 1 - (distances / noDetectionDistance)
    return distances


def calculateMotorValues(distances, noDetectionDistance, accel, image, stepCounter, newImage):
    """
    :param distances: numpy float array
        distances measured by proximity sensors; in meters
        starts with index 0: far left, in clockwise sequence
    :param noDetectionDistance: float
        maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
    :return:  (float,float)
        left and right motor velocity
    """

    leftMotor, rightMotor = 0, 0

    global state
    global resolX, resolY

    xCenter = [-1]

    if state == 'search':
        leftMotor, rightMotor = 0.1, -0.1  # turn clockwise, since box detection in image searches from left to right,
                                            # and we want to detect the same box again, not a second one appearing now
        if newImage:
            if detectBox(image, resolX, resolY, xCenter):
                state = 'approach'
                print('now in state approach')
            else:
                return leftMotor, rightMotor
        else:
            return leftMotor, rightMotor

    if state == 'approach':
        if all(np.greater(distances[1:5], 0.25 * noDetectionDistance * np.ones(4))):
            # nothing in front, go ahead approaching a box
            if xCenter[0] == -1:
                if not detectBox(image, resolX, resolY, xCenter):
                    # lost the box: go on searching
                    state = 'search'
                    print('now in state search')
                    leftMotor, rightMotor = 0.1, -0.1
                    return leftMotor, rightMotor
            err = (float(resolX)/2 - xCenter[0])/float(resolX)
            leftMotor = (-0.5*float(err) + 0.75)*maxVel/2.0
            rightMotor = (0.5*float(err) + 0.75)*maxVel/2.0
            return leftMotor, rightMotor
        else:
            #box approached: now push it
            state = 'push'
            print('now in state push')


    if (stepCounter > 5) and state == 'push' and accel[0] + accel[1] > 0.05:  #bumping with box against wall: turn
        state = 'unwedge'
        print('now in state unwedge')

    if state == 'push':
        [leftMotor, rightMotor] = baseVelocity + maxVel*boxPushingMatrix.dot(normalizeDistances(distances[1:5], noDetectionDistance))
        return leftMotor, rightMotor

    if state == 'unwedge':
        if all(np.greater(distances[1:5], 0.25*noDetectionDistance * np.ones(4))):
            state = 'search'
            print('now in state search')

            leftMotor, rightMotor = 0.1, -0.1
            return leftMotor, rightMotor

        else:
            leftMotor, rightMotor = maxVel, -maxVel
            return leftMotor, rightMotor

    return leftMotor, rightMotor


def main():
    global state
    image = I.new("RGB", (resolX, resolY), "white")

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableCamera()
    robot.enableAllSensors()
    robot.setSensesAllTogether(True)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    stepCounter = 0
    xcenter = [-1]
    # main sense-act cycle
    while robot.isConnected():
        stepCounter += 1
        newImage = False
        robot.fastSensingOverSignal()
        distVector = robot.getProximitySensorValues()
        acceleration = robot.getAccelerometerValues()

        if stepCounter%10 == 0 and (state == 'search' or state == 'approach'):
            image = robot.getCameraImage()
            newImage = True

        leftMotor, rightMotor = calculateMotorValues(distVector, noDetectionDistance, acceleration[0:2], image,
                                                     stepCounter, newImage)
        robot.setMotorSpeeds(leftMotor, rightMotor)
        time.sleep(0.05)

    robot.disconnect()




if __name__ == '__main__':
    main()
