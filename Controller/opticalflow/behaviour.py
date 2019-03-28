from abc import ABCMeta, abstractmethod
import numpy as np
import math

class Behaviour:
    # the abstract superclass of all subsumption behaviours
    __metaclass__ = ABCMeta

    def __init__(self, state):
        """
        :param state:  string
            the name of this behaviour
        """
        self._maxVel = 120 * math.pi / 180  # 4/3 of a full wheel turn
        self.__state = state


    def getState(self):
        return self.__state

    @abstractmethod
    def applicable(self, magnitude, angle, proxSensors, noDetectionDistance):
        """
        :param distances: numpy float array
            distances measured by proximity sensors; in meters
            starts with index 0: far left, in clockwise sequence
        :param noDetectionDistance: float
            maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
        :return:  boolean
            behaviour is applicable in current situation
        """
        pass

    @abstractmethod
    def calculateMotorValues(self, magnitude, angle, proxSensors):
        """
        :param distances: numpy float array
            distances measured by proximity sensors; in meters
            starts with index 0: far left, in clockwise sequence
        :param noDetectionDistance: float
            maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
        :return:  (float,float)
            left and right motor velocity
        """
        pass