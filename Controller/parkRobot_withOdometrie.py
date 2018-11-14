import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
#import datetime as datetime
from math import sin, cos



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
    return matrix_P_n1


def main():

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)
    robot.enableWheelEncoding()
    pose_from_get_pose = robot._getPose()

    current_pose= np.array([
                        [pose_from_get_pose[0]],
                        [pose_from_get_pose[1]],
                        [pose_from_get_pose[2]]])


    final_pose = np.array([
                        [1.225],
                        [0.0],
                        [0.0]])

    wheel_radius = robot._wheelDiameter / 2

    wheel_distance = robot._wheelDistance

    #current_time = datetime.datetime.time(datetime.datetime.now())
    #last_time = datetime.datetime.time(datetime.datetime.now())

    values = robot.getWheelEncoderValues()
    current_wheel_encoding_L = values[0]
    current_wheel_encoding_R = values[1]
    last_wheel_encoding_L = values[0]
    last_wheel_encoding_R = values[1]

    diven_route_L = 0
    diven_route_R = 0

    # main sense-act cycle
    while robot.isConnected():

        values = robot._getWheelEncodingValues()
        current_wheel_encoding_L = values[0]
        current_wheel_encoding_R = values[1]

        current_wheel_encoding_difference_L = current_wheel_encoding_L - last_wheel_encoding_L
        current_wheel_encoding_difference_R = current_wheel_encoding_R - last_wheel_encoding_R
        #print "values", values

        diven_route_L = diven_route_L + (wheel_radius * current_wheel_encoding_difference_L)
        diven_route_R = diven_route_R + (wheel_radius * current_wheel_encoding_difference_R)

        print diven_route_L, diven_route_R

        current_pose = calculateOdometrie(diven_route_L,diven_route_R,current_pose,wheel_distance)

        #DistancePerCount = (2 * 3.14159265 * wheel_radius); #(2 * PI * r) / ppr

        #deltaLeft = tick_x - _PreviousLeftEncoderCounts;
        #deltaRight = tick_y - _PreviousRightEncoderCounts;

        #v_left = (deltaLeft * DistancePerCount) / (current_time - last_time).toSec();
        #v_right = (deltaRight * DistancePerCount) / (current_time - last_time).toSec();


        leftMotor, rightMotor = calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance)

        #print leftMotor, rightMotor

        if leftMotor < 1.5 and rightMotor < 1.5:
            leftMotor = leftMotor + 0.7
            rightMotor = rightMotor + 0.7

        if round(current_pose[0],1) == round(final_pose[0],1) and round(current_pose[1],1) == final_pose[1] and round(current_pose[2],1) == final_pose[2]:
            robot.setMotorSpeeds(0, 0)
            #print "STOP"

        else:
            #print "current_pose", current_pose[0], current_pose[1], current_pose[2]
            robot.setMotorSpeeds(leftMotor, rightMotor)

        last_wheel_encoding_L = current_wheel_encoding_L
        last_wheel_encoding_R = current_wheel_encoding_R

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()