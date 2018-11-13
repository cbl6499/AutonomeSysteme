import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np

# lists with behavior constants
# obstacles in front of the robot
#braitFrontSens_leftMotor = [1, 2, 0, 0]
#braitFrontSens_rightMotor = [-1, -2, 0, 0]

# obstacles at the side of the robot
#braitSideSens_leftMotor = [-1, 0]
#braitSideSens_rightMotor = [0, -1]


#def calc_distance(distances, noDetectionDistance):
#    distances = 1 - (distances / noDetectionDistance)
#    return distances


def calculateMotorValues(current_pose, noDetectionDistance):
    delta_x = 1

    roh = np.sqrt(delta_x)
    maxVel = 120 * 3.1415 / 180  # 4/3 of a full wheel turn

    #velRight = maxVel - (dist[3] + dist[4] + dist[5])
    #velLeft = maxVel - (dist[0] + dist[1] + dist[2])


    return 1


def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableAllSensors()
    robot.setSensesAllTogether(True)   # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    #noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    # main sense-act cycle
    while robot.isConnected():

        robot.fastSensingOverSignal()

        distVector = robot.getProximitySensorValues()
        current_pose = robot._getPose()
        print current_pose
        #leftMotor, rightMotor = calculateMotorValues(current_pose)

        robot.setMotorSpeeds(leftMotor, rightMotor)

        # maxVel = 120 * 3.1415 / 180
        #robot.setMotorSpeeds(maxVel, maxVel)


        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()