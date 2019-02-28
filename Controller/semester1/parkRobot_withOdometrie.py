import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from math import sin, cos


def calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance):
    delta_x = final_pose[0] - current_pose[0]
    delta_y = final_pose[1] - current_pose[1]

    matrix_k = np.array([
        [0.03, 0, 0],
        [0, 0.2, -0.1]
    ])

    #matrix_k = np.array([
    #    [2, 0, 0],
    #    [0, 2.5, -1]
    #])

    roh = np.sqrt((delta_x**2) + (delta_y**2))
    alpha = -(current_pose[2]) + np.arctan2(delta_y, delta_x)
    beta = -(current_pose[2]) - alpha

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

    #pose = np.array([
    #                    [currentPosition[0]],
    #                    [currentPosition[1]],
    #                    [currentPosition[2]]])

    #print "test", pose

    matrix = np.array([
        [driven_route * (cos(pose[2] + ((driven_route_right_wheel - driven_route_left_wheel) / (2.0 * wheel_distance))))],
        [driven_route * (sin(pose[2] + ((driven_route_right_wheel - driven_route_left_wheel) / (2.0 * wheel_distance))))],
        [((driven_route_right_wheel - driven_route_left_wheel) / (wheel_distance))]])

    # Berechnung der naechsten Pose (P n+1)
    matrix_P_n1 = pose + matrix

    print "matrix_P_n1", matrix_P_n1

    return matrix_P_n1


def main():

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)
    robot.enablePose()

    final_pose = np.array([
                        [1.225],
                        [0.0],
                        [0.0]])

    wheel_radius = robot._wheelDiameter / 2
    print "wheel_radius", wheel_radius

    wheel_distance = robot._wheelDistance
    print "wheel_distance", wheel_distance
    robot.enableWheelEncoding()

    found = False

    current_pose_od_a = robot._getPose()
    current_pose_od = np.array([
        [current_pose_od_a[0]],
        [current_pose_od_a[1]],
        [current_pose_od_a[2]]])
    print "current_pose_od", current_pose_od

    values = robot._getWheelEncodingValues()
    last_left_wheel_a = 0
    last_right_wheel_a = 0
    delta_sl = 0
    delta_sr = 0

    while robot.isConnected():
        #current_pose = robot._getPose()

        #print "current_pose", current_pose[0], current_pose[1], current_pose[2]


        values = robot.getWheelEncoderValues()


        current_left_wheel_a = values[0]
        current_right_wheel_a = values[1]
        delta_sl = (current_left_wheel_a + 2*np.pi - last_left_wheel_a) % (2*np.pi)
        delta_sr = (current_right_wheel_a + 2*np.pi - last_right_wheel_a) % (2*np.pi)

        delta_sl = delta_sl * wheel_radius
        delta_sr = delta_sr * wheel_radius


        last_left_wheel_a = current_left_wheel_a
        last_right_wheel_a = current_right_wheel_a


        #print "L: ", delta_sl
        #print "R: ", delta_sr


        #print "current_pose_odo", current_pose_od



        leftMotor, rightMotor = calculateMotorValues(current_pose_od, final_pose, wheel_radius, wheel_distance)

        current_pose_od = calculateOdometrie(delta_sl, delta_sr, current_pose_od, wheel_distance)

        maxVel = 120 * np.pi / 180
        leftMotor = leftMotor
        rightMotor = rightMotor
        tol = 0.05

        if found != True:
            if final_pose[0] - tol < current_pose_od[0] < final_pose[0] + tol and \
                    final_pose[1] - tol < current_pose_od[1] < final_pose[1] + tol and \
                    final_pose[2] - tol < current_pose_od[2] < final_pose[2] + tol:
                robot.setMotorSpeeds(0, 0)
                found = True

            else:
                #print "current_pose", current_pose_od[0], current_pose_od[1], current_pose_od[2]
                robot.setMotorSpeeds(leftMotor, rightMotor)

        print found

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()