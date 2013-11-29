#!/usr/bin/env python

# from Ros2Python import *
import thread
import time

import numpy as np

from MotorControl import *
import GPS

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

	print "There are obstacles"

	keepFar = objectCloud > np.array([-np.inf, 0])
	keepNear = objectCloud < np.array([np.inf, 5])
	keep = np.logical_and(keepNear.all(axis=1), keepFar.all(axis=1))
	objectCloud = objectCloud[keep]


	# print 'Setting right motor'
	# motorControl.setLeftMotor(100)
	# # time.sleep(0.25)
	# motorControl.setRightMotor(100)
	# print 'set right motor'

	print 'Checking amount of obstacles'


	if len(objectCloud) < 300: return # Not enough good points

	# The program has now found an obstacle
	print "Object Cloud"
	for i in range(len(objectCloud)):
		obstaclePlace = (objectCloud[0],objectCloud[1],1)

	#print len(objectCloud)
	#print objectCloud
	meanObjects = np.mean(objectCloud, axis=0)
	print 'Mean of Object Location', meanObjects
	# return
	if meanObjects[1] < 5:
		if meanObjects[0] > 0:
			print 'Setting right motor 150'
			motorControl.setRightMotor(150)
			motorControl.setLeftMotor(0)
			print 'Set right motor'
			print "||||||||||||||||||||||||||||||||||||||||"
		else:
			print 'Setting left motor 100'
			motorControl.setRightMotor(150)
			motorControl.setLeftMotor(0)
			print 'Set left motor'
			print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"



def controlLoop(r2p, loop_time):
	i = 0
	motorControl = MotorControl('/dev/ttyACM', range(4), 9600)
	while True:

		#if not motorControl.ser.isOpen():
		#	motorControl.ser.open()
		now = time.time()
		doControl(r2p, motorControl)
		elapsed = (time.time() - now)
		# Should be good enough
		time.sleep( abs(int(loop_time - elapsed)) )
		print motorControl.ser.isOpen()
		#if i%10 == 0:
		#	motorControl.ser.close()
		print "------------------------------------IN CONTROL------------------------------------"
		i+=1

def GPS_thread(gps, loop_time):
	#print gps.portlist
	# gps.open_port('/dev/ttyUSB0')
	curLatitude = 0;
	curLongitude = 0;
	while True:
		now = time.time()

		# FAKE GPS
		curLatitude += 1
		curLongitude += 1

		# REAL GPS
		# curLatitude = gps.latitude
		# curLongitude = gps.longitude
		#sudo rm -r -f /path/rint gps.time, gps.latitude, gps.longitude
		elapsed = (time.time() - now)
		if (loop_time > (elapsed)):
			time.sleep(abs(int(loop_time - elapsed)))

def Serial_thread(serial, loop_time):
	# The thread that does the serial send
	pass


def main():
	# r2p = Ros2Python()
	gps = GPS.GPSData()

	# Print GPS
	thread.start_new_thread(GPS_thread, (gps, 0.1))

	# Start the ROS loop
	#thread.start_new_thread(rosLoop, (r2p,))
	# Start control loop @ 10Hz
	# thread.start_new_thread(controlLoop, (r2p, 2))

	# r2p.run()

if __name__ == '__main__':
	main()
