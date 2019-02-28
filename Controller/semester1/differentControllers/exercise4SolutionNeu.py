# -*- coding: utf-8 -*-
"""
Created on Wed Aug 5 16:40:22 2017
simple test for BasicEPuck.ePuckVRep
make sure to start first VRep scene pathWithPerfectPoseInfo.ttt, then this program


@author: hans vollbrecht
"""
import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
import matplotlib.pyplot as plt


# lists with behavior constants
# obstacles in front of the robot
braitFrontSens_leftMotor = [1, 2, 0, 0]
braitFrontSens_rightMotor = [-1, -2, 0, 0]

# obstacles at the side of the robot
braitSideSens_leftMotor = [-1, 0]
braitSideSens_rightMotor = [0, -1]





def getPoseByOdometry(robotPose, oldLeftWheelOrientation, oldRightWheelOrientation, newLeftWheelOrientation, newRightWheelOrientation, robotWheelDistance, wheelRadius):

    if oldLeftWheelOrientation is not None and oldRightWheelOrientation is not None:
        # follows 5.2.4, S. 270-271, Siegwart
        # 1. calculate delta_s


        if (newLeftWheelOrientation >= oldLeftWheelOrientation) or (newLeftWheelOrientation < oldLeftWheelOrientation - np.pi):
            # left wheel moves forward
            delta_s_l = wheelRadius * (np.abs(newLeftWheelOrientation + 2.0 * np.pi - oldLeftWheelOrientation) % (2.0 * np.pi))
        elif (oldLeftWheelOrientation > newLeftWheelOrientation) or (oldLeftWheelOrientation < newLeftWheelOrientation - np.pi):
            # left wheel moves backward
            delta_s_l = -wheelRadius * (np.abs(oldLeftWheelOrientation + 2.0 * np.pi - newLeftWheelOrientation) % (2.0 * np.pi))
        if (newRightWheelOrientation >= oldRightWheelOrientation) or (newRightWheelOrientation < oldRightWheelOrientation - np.pi):
            # right wheel moves forward
            delta_s_r = wheelRadius * (np.abs((newRightWheelOrientation + 2.0 * np.pi - oldRightWheelOrientation) % (2.0 * np.pi)))
        elif (oldRightWheelOrientation > newRightWheelOrientation) or (oldRightWheelOrientation < newRightWheelOrientation - np.pi):
            # left wheel moves backward
            delta_s_r = -wheelRadius * (np.abs(oldRightWheelOrientation + 2.0 * np.pi - newRightWheelOrientation) % (2.0 * np.pi))

        #delta_s_l = wheelRadius * (np.abs(newLeftWheelOrientation + 2.0 * np.pi - oldLeftWheelOrientation) % (2.0 * np.pi))
        #delta_s_r = wheelRadius * (np.abs((newRightWheelOrientation + 2.0 * np.pi - oldRightWheelOrientation) % (2.0 * np.pi)))
        print('deltaSl+Sr: ', delta_s_l, delta_s_r)
        print('new left+right', newLeftWheelOrientation, newRightWheelOrientation)
        print('old left+right', oldLeftWheelOrientation, oldRightWheelOrientation)
        delta_s = 0.5*(delta_s_l + delta_s_r)

        # 2. calculate delta_theta
        delta_theta = (delta_s_r - delta_s_l)/robotWheelDistance

        # 3. calculate pose difference
        delta_pose = ( delta_s * np.cos(robotPose[2] + 0.5*delta_theta), \
                       delta_s * np.sin(robotPose[2] + 0.5*delta_theta), \
                       delta_theta )

        # return [robotPose[i] + delta_pose[i] for i in range(len(robotPose))]
        newRobotPose = [robotPose[0] + delta_pose[0],  robotPose[1] + delta_pose[1], (robotPose[2] + delta_pose[2]) % (2.0 * np.pi)]

        if newRobotPose[2] > np.pi:
            newRobotPose[2] = newRobotPose[2] - 2.0*np.pi
        elif newRobotPose[2] < -np.pi:
            newRobotPose[2] = newRobotPose[2] + 2.0*np.pi

        return newRobotPose

    else:
        return robotPose





def main():

    robot = EPuckVRep('ePuck', port=19999, synchronous=True)

    robot.enableAllSensors()
    robot.enablePose()
    robot.enableWheelEncoding()
    robot.setSensesAllTogether(True)   # we want fast sensing, so set robot to sensing mode where all sensors are sensed
    maxVel = 120 * np.pi / 180  # 4/3 of a full wheel turn

    targetPose = np.array((1.225, 0.0, 0.0))   # theta in degrees, counterclockwise
    #targetPose = np.array((1.0, 0.75, np.pi / 2.0))  # theta in degrees, counterclockwise

    r = robot._wheelDiameter/2.0
    l = robot._wheelDistance
    k_rho = 1.5     #req.: >0
    k_beta = -1.5   #req.: <0
    k_alpha = 2.0   #req.: k_alpha > k_rho
    #controller matrix:
    velScaling = 0.03
    controllerMatrix = velScaling * np.asarray([[k_rho/r, k_alpha*l/r, k_beta*l/r],
                                                [k_rho/r, -k_alpha*l/r, -k_beta*l/r]])

    robot.startsim()

    robotWheelDistance = robot._wheelDistance
    print "wheel distance"
    print robot._wheelDistance
    print robot._wheelDiameter/2.0
    wheelRadius = robot._wheelDiameter/2.0

    oldLeftWheelOrientation = None
    oldRightWheelOrientation = None

    pose = np.asarray(robot.getPose())

    if (pose[2] < 0):
        pose[2] += (2*np.pi)

    odometry = True

    stepCounter = 0

    #oldPose = pose

    #oldPose = np.array((0.5, 0, np.pi/2.0))

    # main sense-act cycle
    while robot.isConnected():

        try:

            robot.fastSensingOverSignal()


            # robot.getPose()
            # estimate current robot pose

            try:
                newLeftWheelOrientation, newRightWheelOrientation = robot.getWheelEncoderValues()
                #print('we: ', newLeftWheelOrientation)
                if odometry:
                     pose = np.array(getPoseByOdometry(pose, oldLeftWheelOrientation, oldRightWheelOrientation,
                                                   newLeftWheelOrientation, newRightWheelOrientation,
                                                   robotWheelDistance, wheelRadius))
                else:
                    pose = np.array(robot.getPose())

                [oldLeftWheelOrientation, oldRightWheelOrientation] = [newLeftWheelOrientation,
                                                                       newRightWheelOrientation]
            except ValueError:
                print "No wheel orientation values available"

            delta = targetPose[:2]-pose[:2]

            rho = np.sqrt((delta.dot(delta)))
            alpha = -pose[2] + np.arctan2(delta[1], delta[0])
            beta = -pose[2] - alpha
            #beta = -pose[2] - alpha + np.pi/2.0

            print('pose: ', pose)
            print('rho, alpha, beta: ', rho, alpha, beta)

            if all(np.less(abs(np.array([rho,alpha,beta])), np.array([0.02, 0.05, 0.05]))):
            #if all(np.less(abs(np.array([rho, alpha, beta])), np.array([0.05, 0.05, 0.05]))):
                robot.setMotorSpeeds(0, 0)
                print "arrived at goal pose"
                time.sleep(1000.0)
                break

            rightMotor, leftMotor = controllerMatrix.dot(np.array((rho, alpha, beta)))
            print('vel: ',leftMotor, rightMotor )


            robot.setMotorSpeeds(leftMotor, rightMotor)
            #robot.setMotorSpeeds(0.35, 0.0)

            # maxVel = 120 * 3.1415 / 180
            #robot.setMotorSpeeds(maxVel, maxVel)

            robot.stepsim(1)

        except KeyboardInterrupt:
            # you may interrupt the program execution with ctrl-c, eventually repeating this keystroke
            # make shure to check "emulate terminal in output console" in the pycharm configuration of this module
            print('controller pauses')
            programPause = raw_input("Press the <ENTER> key to continue...")
            print('controller resumes')

    robot.disconnect()


if __name__ == '__main__':
    main()
