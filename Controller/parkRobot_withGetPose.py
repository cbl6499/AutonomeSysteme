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

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)
    robot.enablePose()

    final_pose = np.array([
                        [1.225],
                        [0.0],
                        [0.0]])

    wheel_radius = robot._wheelDiameter / 2

    wheel_distance = robot._wheelDistance

    robot.enableWheelEncoding()

    # main sense-act cycle
    while robot.isConnected():

        current_pose = robot._getPose()

        leftMotor, rightMotor = calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance)

        #print leftMotor, rightMotor

        robot.getWheelEncoderValues()

        #values = robot.getWheelEncoderValues()

        #print values


        if  0.05 < leftMotor < 1.5 and 0.05 < rightMotor < 1.5:
            leftMotor = leftMotor + 0.7
            rightMotor = rightMotor + 0.7

        tol = 0.005

        if round(final_pose[0], 3) - tol < round(current_pose[0],3) < round(final_pose[0],3) + tol and \
           round(final_pose[1], 3) - tol < round(current_pose[1],3) < round(final_pose[1],3) + tol and \
           round(final_pose[2], 0) - tol < round(current_pose[2],0) < round(final_pose[2],0) + tol:
            robot.setMotorSpeeds(0, 0)
            print "STOP"

        else:
            print "current_pose", current_pose[0], current_pose[1], current_pose[2]
            robot.setMotorSpeeds(leftMotor, rightMotor)

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()