import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np


def calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance):
    delta_x = final_pose[0] - current_pose[0]
    delta_y = final_pose[1] - current_pose[1]

    matrix_k = np.array([
        [0.03, 0, 0],
        [0, 0.2, -0.1]
    ])

    roh = np.sqrt((delta_x**2) + (delta_y**2))
    alpha = -(current_pose[2]) + np.arctan2(delta_y, delta_x)
    beta = - (current_pose[2]) - alpha

    phi_lr = np.array([[matrix_k[0, 0] / wheel_radius, (matrix_k[1, 1]*wheel_distance) / wheel_radius, (matrix_k[1, 2]*wheel_distance) / wheel_radius],
                      [matrix_k[0, 0] / wheel_radius, -(matrix_k[1, 1]*wheel_distance) / wheel_radius, -(matrix_k[1, 2]*wheel_distance) / wheel_radius]])

    angles = np.array([roh,alpha,beta])

    phi_l_r = phi_lr.dot(angles)

    return phi_l_r[1], phi_l_r[0]



def calculateOdometrie(driven_route_left_wheel, driven_route_right_wheel, currentPosition, wheel_distance):
    # Gefahrene Wegstrecke delta s
    driven_route = (driven_route_right_wheel + driven_route_left_wheel) / 2.0

    # Aktuelle Pose
    pose = currentPosition

    matrix = np.array([
        [driven_route * (cos(currentPosition[2] + ((driven_route_right_wheel - driven_route_left_wheel) / (2.0 * wheel_distance))))],
        [driven_route * (sin(currentPosition[2] + ((driven_route_right_wheel - driven_route_left_wheel) / (2.0 * wheel_distance))))],
        [((driven_route_right_wheel - driven_route_left_wheel) / (2.0 * wheel_distance))]])

    # Berechnung der naechsten Pose (P n+1)
    matrix_P_n1 = pose + matrix



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

    found = False

    while robot.isConnected():

        current_pose = robot._getPose()

        leftMotor, rightMotor = calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance)

        values = robot._getWheelEncodingValues()
        current_wheel_encoding_L = values[0]
        current_wheel_encoding_R = values[1]



        maxVel = 120 * np.pi / 180

        leftMotor = leftMotor + maxVel/2
        rightMotor = rightMotor + maxVel/2
        tol = 0.02

        if found != True:
            if final_pose[0] - tol < current_pose[0] < final_pose[0] + tol and \
                    final_pose[1] - tol < current_pose[1] < final_pose[1] + tol and \
                    final_pose[2] - tol < current_pose[2] < final_pose[2] + tol:
                robot.setMotorSpeeds(0, 0)
                found = True

            else:
                #print "current_pose", current_pose[0], current_pose[1], current_pose[2]
                robot.setMotorSpeeds(leftMotor, rightMotor)

        print found

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()