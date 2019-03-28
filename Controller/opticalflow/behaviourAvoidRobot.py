from behaviour import Behaviour
import numpy as np

class AvoidRobot(Behaviour):

    avoidRobotMatrix = np.array([   [1/100, 0.25/100, 0.5/100, 0, 0],
                                    [0, 0, 0.5/100, 0.25/100, 1/200]
                                    ])

    def __init__(self):
        super(AvoidRobot, self).__init__("avoidRobot")
    
    def applicable(self,  magnitude, angle, proxSensors, noDetectionDistance):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        
        #brack down the image to 5x5
        mag = self.graduateMagnitudeValuesInSections(self.filterEdge(magnitude))

        #There is something in the frame when 
        if (np.mean(mag[1:4,0]) > 50 or np.mean(mag[1:4,4]) > 50 or np.mean(mag[1:4,2]) > 50):
            return True
        else:
            return False

    def filterEdge(self,magnitude):
        """
        Filter the edges of ground to wall and wall to sky
        :return:  filtered magnitueds
        """
        #Try to remove edges to wall
        magnitude[magnitude > 135] -= 135
        return magnitude

    def calculateMotorValues(self,  magnitude, angle, proxSensors):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        #graduate magnitued matrix into a 5x5 matrix 
        mag = self.graduateMagnitudeValuesInSections(self.filterEdge(magnitude))

        [left,right] = (self.avoidRobotMatrix.dot(np.transpose(mag[1:4,:]))).sum(axis=1)    

        #object moving left to right
        if np.mean(angle[24:40,0:40]) < 150 and np.mean(angle[24:40,41:64]) < 150:
            return 2.09 - left, 2.09 - right
        #object moving right to left
        else:
            return 2.09 - right, 2.09 - left

    def graduateMagnitudeValuesInSections(self, mag):
        """
        Graduates the 64x64 matrix into a 5x5 matrix by creating the
        mean of 12x12 pixels.
        :return: 5x5 matrix of means
        """
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

        magNew = np.array([ [c00, c01, c02, c03, c04], 
                            [c10, c11, c12, c13, c14],
                            [c20, c21, c22, c23, c24],
                            [c30, c31, c32, c33, c34],
                            [c40, c41, c42, c43, c44]
                            ])
        return magNew