import time
from BasicEPuck.ePuckVRep import EPuckVRep
import numpy as np
from math import sin, cos
from PIL import Image as I

resolX, resolY = 64, 64
boxFound = False
last_left_wheel_a = 0
last_right_wheel_a = 0
round_one = False
current_pose_od = np.array([
                        [0.0],
                        [0.0],
                        [0.0]])


def searchBox(image, robo):
    global boxFound

    xCenter = [-1]
    leftMotor, rightMotor = 0.1, -0.1

    if detectBox(image, resolX, resolY, xCenter):
        boxFound = True
        print "Box found"
        leftMotor = 0.5
        rightMotor = 0.5
        setCurrentPose(robo)
        return leftMotor, rightMotor
    else:
        return leftMotor, rightMotor

# Box suchen
def detectBox(image, resolX, resolY, xCenter):
    minBlobWidth = 5
    xStart = -1
    for y in range(resolY):
        blobwidth = 0
        for x in range(resolX):
            pixel = image.getpixel((x, y))
            if pixel == (0, 0, 0):  # black pixel: a box!
                blobwidth += 1
                if  blobwidth == 1:
                    xStart = x
            else:
                if blobwidth >= minBlobWidth:
                    xCenter[0] = xStart + blobwidth/2
                    # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                    return True
                elif blobwidth > 0:
                    blobwidth = 0
        if blobwidth >= minBlobWidth:
            xCenter[0] = xStart + blobwidth/2
            # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
            return True

    return False

# auf die Box zufahren
def approach (xCenter, image, maxVel):
    if xCenter[0] == -1:
        if not detectBox(image, resolX, resolY, xCenter):
            # lost the box: go on searching
            leftMotor, rightMotor = 0.1, -0.1
            return leftMotor, rightMotor
    err = (float(resolX) / 2 - xCenter[0]) / float(resolX)
    leftMotor = (-0.5 * float(err) + 0.75) * maxVel / 2.0
    rightMotor = (0.5 * float(err) + 0.75) * maxVel / 2.0
    return leftMotor, rightMotor

def calculateMotorValues(current_pose, final_pose, wheel_radius, wheel_distance):
    delta_x = final_pose[0] - current_pose[0]
    delta_y = final_pose[1] - current_pose[1]

    #matrix_k = np.array([
    #    [0.03, 0, 0],
    #    [0, 0.2, -0.1]])
    matrix_k = np.array([
        [0.04, 0, 0],
        [0, 0.02, -0.015]])

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

    delta_theta = (driven_route_right_wheel - driven_route_left_wheel) / (wheel_distance)

    matrix = np.array([
        [driven_route * (cos(pose[2] + (delta_theta / 2.0)))],
        [driven_route * (sin(pose[2] + (delta_theta / 2.0)))],
        [delta_theta]])

    matrix_P_n1 = np.add(pose, matrix)

    return matrix_P_n1

def setCurrentPose(robot):
    global current_pose_od

    current_pose_od_a = robot.getPose()
    current_pose_od = np.array([
        [current_pose_od_a[0]],
        [current_pose_od_a[1]],
        [current_pose_od_a[2]]])


def main():
    image = I.new("RGB", (resolX, resolY), "white")
    robot = EPuckVRep('ePuck', port=19999, synchronous=True)
    robot.enablePose()
    robot.enableCamera()
    global boxFound

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

    global current_pose_od
    current_pose_od = np.array([
        [0.5],
        [0.5],
        [0.0]])

    print "current_pose_od", current_pose_od

    while robot.isConnected():
        values = robot.getWheelEncoderValues()

        current_left_wheel_a = values[0]
        current_right_wheel_a = values[1]

        global round_one

        delta_sl = 0
        delta_sr = 0

        if(round_one == True):
            delta_sl = (current_left_wheel_a + 2*np.pi - last_left_wheel_a) % (2*np.pi)
            delta_sr = (current_right_wheel_a + 2*np.pi - last_right_wheel_a) % (2*np.pi)

            delta_sl = delta_sl * wheel_radius
            delta_sr = delta_sr * wheel_radius

            global current_pose_od
            current_pose_od = calculateOdometrie(delta_sl, delta_sr, current_pose_od, wheel_distance)

        global last_left_wheel_a
        global last_right_wheel_a
        last_left_wheel_a = current_left_wheel_a
        last_right_wheel_a = current_right_wheel_a

        maxVel = 120 * np.pi / 180
        # get new image
        xCenter = [-1]
        image = robot.getCameraImage()

        delta_x = final_pose[0] - current_pose_od[0]
        delta_y = final_pose[1] - current_pose_od[1]
        roh = np.sqrt((delta_x ** 2) + (delta_y ** 2))
        print "roh", roh

        #print boxFound
        if boxFound != True:
            leftMotor, rightMotor = searchBox(image, robot)
        elif roh > 0.45:
            leftMotor, rightMotor = approach(xCenter, image, maxVel)
        else:
            leftMotor, rightMotor = calculateMotorValues(current_pose_od, final_pose, wheel_radius, wheel_distance)

            tol = 0.05

            if found != True:
                if final_pose[0] - tol < current_pose_od[0] < final_pose[0] + tol and \
                        final_pose[1] - tol < current_pose_od[1] < final_pose[1] + tol and \
                        final_pose[2] - tol < current_pose_od[2] < final_pose[2] + tol:
                    robot.setMotorSpeeds(0, 0)
                    found = True
                    time.sleep(1000.0)
                    break
                else:
                    # print "current_pose", current_pose_od[0], current_pose_od[1], current_pose_od[2]
                    robot.setMotorSpeeds(leftMotor, rightMotor)


            print found

        robot.setMotorSpeeds(leftMotor, rightMotor)
        round_one = True

        #print "current_pose_od", current_pose_od[0], current_pose_od[1], current_pose_od[2]
        #test = robot.getPose()
        #print "curren_pos", test[0], test[1], np.rad2deg(test[2])

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()