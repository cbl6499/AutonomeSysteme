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




def calculateMotorValues(current_pose, final_pose, matrix_k, wheel_radius, wheel_distance):
    delta_x = final_pose[0] - current_pose[0]
    delta_y = final_pose[1] - current_pose[1]


    roh = np.sqrt((delta_x**2) + (delta_y**2))
    alpha = -(current_pose[2]) + np.arctan2(delta_y, delta_x)
    beta = - (current_pose[2]) - alpha

    phi_r = (matrix_k[0,0] / wheel_radius * roh) + ((matrix_k[1,1]*wheel_distance) / wheel_radius * alpha) + ((matrix_k[1,2]*wheel_distance) / wheel_radius * beta)
    phi_l = (matrix_k[0,0] / wheel_radius * roh) - ((matrix_k[1,1]*wheel_distance) / wheel_radius * alpha) - ((matrix_k[1,2]*wheel_distance) / wheel_radius * beta)



    #maxVel = 120 * 3.1415 / 180  # 4/3 of a full wheel turn

    #velRight = maxVel - (dist[3] + dist[4] + dist[5])
    #velLeft = maxVel - (dist[0] + dist[1] + dist[2])


    return phi_l, phi_r


def main():
    final_pose = np.array([
                        [1.225],
                        [0.0],
                        [0.0]])

    matrix_k = np.array([
            [0.05, 0, 0],
            [0, 1.5, 0.2]
        ])

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    wheel_radius = robot._wheelDiameter / 2;

    wheel_distance = robot._wheelDistance

    robot.enablePose()

    #noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    # main sense-act cycle
    while robot.isConnected():

        #robot.fastSensingOverSignal()
        #distVector = robot.getProximitySensorValues()

        current_pose = robot._getPose()
        print wheel_radius
        leftMotor, rightMotor = calculateMotorValues(current_pose, final_pose, matrix_k, wheel_radius, wheel_distance)

        robot.setMotorSpeeds(leftMotor, rightMotor)

        #maxVel = 120 * 3.1415 / 180
        #robot.setMotorSpeeds(maxVel, maxVel)


        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()