#!/usr/bin/env python

from Ros2Python import *
import thread
import time
import math
import queu
e
from SerialUSB import *

import numpy as np

from scipy.misc import imresize

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

	print 'Checking amount of obstacles'
	lines = r2p.objectDetector.get2DCloudFromMask(r2p.maskall)
	#objectCloud = np.kron(objectCloud,np.ones((3,1)))
	objectCloud = imresize(objectCloud,((lines).shape))
	objectCloud = np.logical_or(objectCloud,lines)


	scaler = 200
	if (heading < -4):
		motorSpeedL = scaler * 1.5
		motorSpeedR = scaler
	elif (heading > 4):
		motorSpeedL = scaler
		motorSpeedR = scaler * 1.5
	else:
		motorSpeedR = scaler
		motorSpeedL = scaler

	if len(objectCloud) < 300: 
		print "Going forward"
		motorControl.setMotorSpeeds(motorSpeedL,motorSpeedR)
		#motorControl.setMotor(motorSpeedL, motorSpeedR)
		return # Not enough good points

	# The program has now found an obstacle
	print "Object Cloud"
	for i in range(len(objectCloud)):
		obstaclePlace = (objectCloud[0],objectCloud[1],1)

	meanObjects = np.mean(objectCloud, axis=0)
	print 'Mean of Object Location', meanObjects
	print (meanObjects[0],meanObjects[1])
	# return
	if meanObjects[0]>12*math.pi/180:
		print 'Setting right motor'
		motorSpeedR = min(100,scaler/meanObjects[1])
		motorSpeedL = 0
	elif meanObjects[0]<-2*math.pi/180:
		print 'Setting left motor'
		motorSpeedL = min(100,scaler/meanObjects[1])
		motorSpeedR = 0
	else:
		print "Going forward"
		motorSpeedL = 0
		motorSpeedR = 20
	print "Setting motor"
	motorControl.setMotorSpeeds(motorSpeedL,motorSpeedR)
	#motorControl.setMotor(motorSpeedL, motorSpeedR)

def doGPS(r2p,gps):
	# GPS STUFF YO!
	desLatitude = -38.1978504384
	desLongitude = 144.2991186111
	heading = 0
	curLongitude = gps.longitude
	curLatitude = gps.latitude
	curHeading = gps.headingM
	heading = curHeading - (math.degrees(math.atan((desLatitude - curLatitude)/(desLongitude - curLongitude))))
	return

def doSerial(r2p,serialControl):
	motorSpeedL = 0
	motorSpeedR = 0

	serialControl.sendCommandMotors(motorSpeedL,motorSpeedR)
	return

def controlLoop(r2p, loop_time):
	motorControl = MotorControl('/dev/ttyACM', range(4), 9600)
	while True:
		now = time.time()
		try:			
			doControl(r2p, motorControl)
			print "------------------------------------IN CONTROL------------------------------------"
		except Exception, e:
			print "Control Failed",e
		finally:
			elapsed = (time.time() - now)
			# Should be good enough
			time.sleep( abs(int(loop_time - elapsed)) )

def gpsLoop(r2p, loop_time):
	gps = GPS.GPSData()
	gps.open_port('/dev/ttyUSB', range(4), 19200)
	desLatitude = -38.1978504384
	desLongitude = 144.2991186111
	while True:
		now = time.time()
		try:
			doGPS(r2p,gps)	
			print "------------------------------------GPS LOCK----------------------------------------"
		except Exception, e:
			print "GPS not locked",e
		finally:
			elapsed = (time.time() - now)
			time.sleep( abs(int(loop_time - elapsed)) )
		#sudo rm -r -f /path/rint gps.time, gps.latitude, gps.longitude

def serialLoop(r2p,loop_time):	
	serialControl = SerialUSB('/dev/ttyACM', range(4), 115200)
	while True:
		# curLongitude = gps.longitude 
		# curLatitude = gps.latitude
		now = time.time()
		try:
			doSerial(r2p,serialControl)
			print "---------------------------------SERIAL SENDING-------------------------------------"
		except Exception, e:
			print "Serial Unavailable", e
		finally:
			elapsed = (time.time() - now)
			time.sleep( abs(int(loop_time - elapsed)) )

def main():
	r2p = Ros2Python()

	#Control Loop
	#GPS Loop
	#Serial Loop
	#ROS Loop
	#IMU Loop

	# Start the ROS loop
	#thread.start_new_thread(rosLoop, (r2p,))
	# Start control loop @ 10Hz
	thread.start_new_thread(controlLoop, (r2p, 2))
	# GPS control loop @  2Hz
	thread.start_new_thread(gpsLoop, (r2p,4))
	# Serial comms loop @  10Hz
	thread.start_new_thread(serialLoop, (r2p,2))
	# ROS 2 Python Thread
	r2p.run()

if __name__ == '__main__':
	main()
