# PID CONTROL OF WHEELCHAIR
# Will return the speed for each of the motors

import numpy as np

Kp = 1
Kd = 1
# Ki = 10
dt = 0.5

error = 0
lasterror = 0

# The speed of the wheelchair
motorAvg = np.array([0,0])			# 0 SPEED
motorMax = np.array([100,100])		# FULL FORWARD
motorMin = np.array([-100, -100])	# FULL REVERSE

def getNewSpeed(curPos, desPos, lasterror):
	error = desPos - curPos
	speed = Kp * error + Kd * (error - lasterror)
	lasterror = error
	newSpeed = motorAvg + speed
	for i in range(2):
		if(newSpeed[i] > motorMax[i]):
			newSpeed[i] = motorMax[i]
		elif (newSpeed[i] < motorMin[i]):
			newSpeed[i] = motorMin[i]
	return newSpeed, lasterror

def getNewPos(curPos, speed, dt):
	return curPos + speed * dt

curPos = np.array([0,0])
desPos = np.array([50,100])
speed = np.array([50,50])


for i in range(20):
	speed, lasterror = getNewSpeed(curPos, desPos, lasterror)
	curPos = getNewPos(curPos, speed, dt)
	print lasterror, speed, curPos
