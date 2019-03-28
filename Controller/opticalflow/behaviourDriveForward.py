from behaviour import Behaviour

class DriveForward(Behaviour):

    def __init__(self):
        super(DriveForward, self).__init__("driveForward")

    def applicable(self,  magnitude, angle, proxSensors, noDetectionDistance):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """

        return True

    def calculateMotorValues(self,  magnitude, angle, proxSensors):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        leftMotor = rightMotor = 2.09

        return leftMotor, rightMotor