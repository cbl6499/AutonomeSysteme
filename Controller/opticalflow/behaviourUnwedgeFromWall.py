from behaviour import Behaviour
import numpy as np

class UnwedgeFromWall(Behaviour):

    def __init__(self):
        super(UnwedgeFromWall, self).__init__("unwedgeFromWall")

    def applicable(self,  magnitude, angle, proxSensors, noDetectionDistance):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """

        #Sensor 0 to 5
        if all(np.greater(proxSensors[0:6], 0.5 * noDetectionDistance * np.ones(6))):
            return False
        else:
            return True

    def calculateMotorValues(self,  magnitude, angle, proxSensors):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """

        leftMotor = 2.09
        rightMotor = -2.09

        return leftMotor, rightMotor