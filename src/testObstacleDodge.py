#!/usr/bin/env python

from Ros2Python import *
import thread
import time

import numpy as np

from MotorControl import *

# r2p = None

def rosLoop(r2p):
	r2p.run()

def doControl(r2p, motorControl):
	''' Do the actual control stuff'''
	#print r2p.mask
	#r2p.objectDetector.showDetected(r2p.mask)
	r2p.showDetected = True
	objectCloud = r2p.obstacleCloud.copy()
	if len(objectCloud) == 0: return



	keepFar = objectCloud > np.array([-np.inf, 0])
	keepNear = objectCloud < np.array([np.inf, 5])
	keep = np.logical_and(keepNear.all(axis=1), keepFar.all(axis=1))
	objectCloud = objectCloud[keep]


	print 'Setting right motor'
	motorControl.setLeftMotor(100)
	# time.sleep(0.25)
	motorControl.setRightMotor(100)
	print 'set right motor'


	if len(objectCloud) < 800: return # Not enough good points

	#print len(objectCloud)
	#print objectCloud
	meanObjects = np.mean(objectCloud, axis=0)
	print 'Mean of Object Location', meanObjects
	# return
	if meanObjects[1] < 5:
		if meanObjects[0] > 0:
			motorControl.setRightMotor(150)
			motorControl.setLeftMotor(0)
			print 'Setting right motor'
		else:
			motorControl.setRightMotor(0)
			motorControl.setLeftMotor(100)
			print 'Setting left motor'


def controlLoop(r2p, loop_time):
	motorControl = MotorControl('/dev/ttyACM0', 9600)
	while True:
		now = time.time()
		doControl(r2p, motorControl)
		elapsed = (time.time() - now)
		# Should be good enough
		time.sleep( loop_time - elapsed )
		print motorControl.ser.isOpen()


def main():
	r2p = Ros2Python()

	# Start the ROS loop
	#thread.start_new_thread(rosLoop, (r2p,))
	# Start control loop @ 10Hz
	thread.start_new_thread(controlLoop, (r2p, 0.1))

	r2p.run()

if __name__ == '__main__':
	main()
