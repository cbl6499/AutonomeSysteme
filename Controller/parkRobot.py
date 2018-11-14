import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np



def calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance):
    delta_x = final_pose[0] - current_pose[0]
    delta_y = final_pose[1] - current_pose[1]

    matrix_k = np.array([
        [0.04, 0, 0],
        [0, 0.2, -0.1]
    ])

    roh = np.sqrt((delta_x**2) + (delta_y**2))
    alpha = -(current_pose[2]) + np.arctan2(delta_y, delta_x)
    beta = - (current_pose[2]) - alpha

    phi_r = (matrix_k[0,0] / wheel_radius * roh) + ((matrix_k[1,1]*wheel_distance) / wheel_radius * alpha) + ((matrix_k[1,2]*wheel_distance) / wheel_radius * beta)
    phi_l = (matrix_k[0,0] / wheel_radius * roh) - ((matrix_k[1,1]*wheel_distance) / wheel_radius * alpha) - ((matrix_k[1,2]*wheel_distance) / wheel_radius * beta)

    return phi_l, phi_r


def main():

    final_pose = np.array([
                        [1.225],
                        [0.0],
                        [0.0]])

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    wheel_radius = robot._wheelDiameter / 2

    wheel_distance = robot._wheelDistance

    robot.enablePose()

    # main sense-act cycle
    while robot.isConnected():

        current_pose = robot._getPose()

        leftMotor, rightMotor = calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance)

        #print leftMotor, rightMotor

        if leftMotor < 1.5 and rightMotor < 1.5:
            leftMotor = leftMotor + 0.7
            rightMotor = rightMotor + 0.7

        if round(current_pose[0],1) == round(final_pose[0],1) and round(current_pose[1],1) == final_pose[1] and round(current_pose[2],1) == final_pose[2]:

            robot.setMotorSpeeds(0, 0)
            print "STOP"

        else:
            print "current_pose", current_pose[0], current_pose[1], current_pose[2]
            robot.setMotorSpeeds(leftMotor, rightMotor)

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()