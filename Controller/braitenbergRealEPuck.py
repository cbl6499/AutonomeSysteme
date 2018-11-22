# -*- coding: utf-8 -*-
"""
Created on Wed Sep 10 20:40:22 2017
simple test for BasicEPuck.ePuckReal
make sure to connect your ePuck to your PC/Notebook via bluetooth , then run this program
configure this program with an argument (as parameter to main) indicating the id of your robot which is written
in front of the robot chassis


@author: Manuel Martín Ortiz / hans vollbrecht
"""
from BasicEPuck.ePuckReal import EPuckReal
import sys
import re

# You can use this dictionary to associate an ePuck ID with its MAC Address
epucks = {
	'2836' : '10:00:E8:AD:78:32',
	'3012' : '10:00:E8:AD:78:75',
	'3092' : '10:00:E8:C5:61:C9',
	'3243' : '10:00:E8:D7:03:A2',
	'3312' : '10:00:E8:AD:78:6A',
	'3253' : '10:00:E8:AD:78:2B'
}

def log(text):
	"""	Show @text in standart output with colors """

	blue = '\033[1;34m'
	off = '\033[1;m'

	print(''.join((blue, '[Log] ', off, str(text))))

def error(text):
	red = '\033[1;31m'
	off = '\033[1;m'

	print(''.join((red, '[Error] ', off, str(text))))

def main(mac):

	log('Connecting with the ePuck')
	try:
		# First, create an ePuck object.
		# If you want debug information:
		#~ robot = ePuck(mac, debug = True)
		# else:
		robot = EPuckReal('3012', mac)
		# robot = EPuckReal('2836', mac)
		# robot = EPuckReal('3243', '10:00:E8:D7:03:A2')
		# robot = EPuckReal('3312', '10:00:E8:AD:78:6A')
		# Second, connect to it
		robot.connect()

		# You can enable various sensors at the same time. Take a look to
		# to DIC_SENSORS for know the name of the sensors
		robot.enableProximitySensors()
		robot.enableCamera()
		# robot.enable('proximity', 'light', 'camera')

		log('Conection complete. CTRL+C to stop')
		log('Library version: ' + robot.getVersion())

	except Exception, e:
		error(e)
		sys.exit(1)

	try:
		#avoidance
		#matrix = ( (150, -35), (100, -15), (80, -10), (-10, -10),
		#(-10, -10), (-10, 80), (-30, 100), (-20, 150) )
		#boxpushing
		matrix = ( (150, -35), (100, -15), (80, -10), (-10, -10),
		(-10, -10), (-10, 80), (-30, 100), (-20, 150) )
		counter=1
		while True:
			# Important: when you execute 'step()', al sensors
			# and actuators are updated. All changes you do on the ePuck
			# will be effectives after this method, not before
			robot.step()

			# Now, we can get updated information from the sensorss
			prox_sensors = robot.getProximitySensorValues()
			print prox_sensors

			# The Braitenberg algorithm is really simple, it simply computes the
			# speed of each wheel by summing the value of each sensor multiplied by
			# its corresponding weight. That is why each sensor must have a weight
			# for each wheel.
			wheels = [0, 0]
			for w, s in ((a, b) for a in range(len(wheels)) for b in range(len(prox_sensors))):
				# We need to recenter the value of the sensor to be able to get
				# negative values too. This will allow the wheels to go
				# backward too.
				wheels[w] += matrix[s][w] * (1.0 - (prox_sensors[s] / 512))

			# Now, we set the motor speed. Remember that we need to execute 'step()'
			# for make this command effective
			print( wheels[0], wheels[1])
			robot.setIntMotorSpeeds(wheels[0], wheels[1])

 		log('Stoping the robot. Bye!')
		robot.close()
		sys.exit()
	except KeyboardInterrupt:
		log('Stoping the robot. Bye!')
		robot.close()
		sys.exit()
	except Exception, e:
		print e

	return 0

if __name__ == '__main__':
	X = '([a-fA-F0-9]{2}[:|\-]?){6}'
	if len(sys.argv) < 2:
		error("Usage: " + sys.argv[0] + " ePuck_ID | MAC Address")
		sys.exit()
	robot_id = sys.argv[1]

	if epucks.has_key(robot_id):
		main(epucks[robot_id])

	elif re.match(X, robot_id) != 0:
		main(robot_id)

	else:
		error('You have to indicate the MAC direction of the robot')

