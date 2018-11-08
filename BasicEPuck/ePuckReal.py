# -*- coding: utf-8 -*-
"""
Created on Wed Jul 19 16:40:22 2017
the class "EPuckReal" encapsulates the real ePuck accessed by a bluetooth connection for a controller: it represents
a proxy of the ePuck for the controller. Note that the real proxy object for accessing the real robot by bluetooth is
self.__robot of class EPuckRobot



@author: hans vollbrecht
"""

import logging
import math
import numpy as np
from ePuck import EPuck
from ePuckRobot import EPuckRobot


class EPuckReal(EPuck):

    def __init__(self, name, mac_address):
        """
        constructor for a robot that connects to the VRep Simulator on port
        :param
            name: string
                the name of the robot, best if you use the Bluetooth code written on the ePuck chassis
            mac_address: string
                the mac_address of the ePuck robot

        """
        self.__mac = mac_address
        self.__robot = EPuckRobot(mac_address)    # __robot is the interface object for communication via bluetooth

        super(EPuckReal, self).__init__(name)


    def connect(self):

        try:
            self.__robot.connect()
        except Exception, e:
            print str(e)


    def _initRobotModel(self):
        # get wheel diameters from simulator, and set maximum velocity to the simulator

        self._numProximitySensors = 8
        self._numLightSensors = 8
        #maximum speeds: be careful, calibrate
        self._maxIntVel = 1000
        self._maxVel = 1  #attention: should be such that with _maxIntVel of the real ePuck, we get _maxVel rad/sec!
        self._wheelDiameter = 0.0425  # in meters
        self._wheelDistance = 0.0623  # in meters


    def enableProximitySensors(self, sensorIDs=[]):
        super(EPuckReal, self).enableProximitySensors(sensorIDs)
        self.__robot.enable('proximity')

    def enableLightSensors(self, sensorIDs=[]):
        super(EPuckReal, self).enableLightSensors(sensorIDs)
        self.__robot.enable('light')

    def enableAccelerometer(self):
        super(EPuckReal, self).enableAccelerometer()
        self.__robot.enable('accelerometer')

    def enableWheelEncoding(self):
        super(EPuckReal, self).enableWheelEncoding()
        self.__robot.enable('motor_position')

    def enableCamera(self):
        super(EPuckReal, self).enableCamera()
        self.__robot.enable('camera')

    def enableAllSensors(self):
        self.enableProximitySensors()
        # self.enableGroundSensor()
        self.enableAccelerometer()
        self.enableWheelEncoding()
        self.enableLightSensors()


    def _getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            implies a request for sensing
        """

        if not self._proximitySensorsEnabled:
            logging.error('cannot read proximity sensors if not enabled')
            return self._proximitySensorValues

        # read distance sensor values from __robot
        ret_floats = self.__robot.get_proximity()
        # convert to numpy array
        self._proximitySensorValues = np.array(ret_floats)[self._enabledProximitySensors]
        logging.debug('dist values: ', self._proximitySensorValues)

        return self._proximitySensorValues


    def _getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            implies a request for sensing
        """
        if not self._lightSensorsEnabled:
            logging.error('cannot read light sensors if not enabled')
            return self._lightSensorValues

        # read light sensor values from __robot
        ret_floats = self.__robot.get_light_sensor()
        # convert to numpy array
        self._lightSensorValues = np.array(ret_floats)[self._enabledLightSensors]
        logging.debug('light values: ',self._lightSensorValues)

        return self._lightSensorValues



    def _getGroundSensorValues(self):
        """
        :return: numpy 3x1 float array
            current (latest) values of enabled ground sensor.
            implies a request for sensing
        """
        if not self._groundSensorEnabled:
            logging.error('cannot read ground sensor if not enabled')
            return self._groundSensorValues

        # read ground sensor values from __robot
        ret_floats = self.__robot.get_floor_sensors()
        # convert to numpy array
        self._groundSensorValues = np.array(ret_floats)
        logging.debug('ground values: ',self._groundSensorValues)

        return self._groundSensorValues


    def _getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            implies a request for sensing
        """
        if not self._accelerometerEnabled:
            logging.error('cannot read accelerometer if not enabled')
            return self._accelerometerValues

        # read accelerometer values from __robot
        ret_floats = self.__robot.get_accelerometer()
        # convert to numpy array
        self._accelerometerValues = np.array(ret_floats)
        logging.debug('accelerometer values: ', self._accelerometerValues)

        return self._accelerometerValues




    def _getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            implies a request for sensing
        """
        if not self._cameraEnabled:
            logging.error('cannot read camera image if not enabled')
            return self._image

        # read accelerometer values from __robot
        self._image = self.__robot.get_image()

        return self._image


    def setIntMotorSpeeds(self, leftSpeed, rightSpeed):
        """
        set the desired speeds of the left and right wheel of the robot
        :param
        leftSpeed: int
            speed in [-1000,1000]
        :param
        rightSpeed: int
            speed in [-1000,1000]

        """
        #set robot velocity attributes eventually clamping them to upper or lower bound
        self._velLeft = max(min(leftSpeed, self._maxIntVel), -self._maxIntVel)
        self._velRight = max(min(rightSpeed, self._maxIntVel), -self._maxIntVel)

        self.__robot.set_motors_speed(self._velLeft, self._velRight)


    def _setMotorSpeeds(self, leftSpeed, rightSpeed):
        """
        set the desired speeds of the left and right wheel of the robot
        :param
        leftSpeed: float
            speed in rad/sec of left motor
        :param
        rightSpeed: float
            speed in rad/sec of right motor

        """


        leftSpeedInt = math.copysign(int(self._maxIntVel*(math.fabs(leftSpeed)/self._maxVel)), leftSpeed)
        rightSpeedInt = math.copysign(int(self._maxIntVel*(math.fabs(rightSpeed)/self._maxVel)), rightSpeed)

        self.__robot.set_motors_speed(leftSpeedInt, rightSpeedInt)  #must be an integer, see ePuckRobot



    def step(self, nr=1):
        """
          runs nr steps of the real ePuck: write actuators, reads sensors, evtl. read camera

        :param
            nr: int
                number of basic steps to take
        :return:
        """
        for i in range(nr):
            self.__robot.step()


    def close(self):
        self.__robot.close()


    def getVersion(self):
        return self.__robot.version

