# PID CONTROL OF WHEELCHAIR
# Will return the speed for each of the motors

import numpy as np
import matplotlib.pyplot as plot
import math

class PID():

	def __init__(self):
		self.Kp = 1
		self.Kd = 0.1
		# self.Ki = 10
		# The speed of the wheelchair
		self.motorAvg = np.array([0,0])         # 0 SPEED
		self.motorMax = np.array([100,100])     # FULL FORWARD
		self.motorMin = np.array([-100, -100])  # FULL REVERSE

	def getNewSpeed(self, curPos, desPos, lasterror):
		error = desPos - curPos
		speed = self.Kp * error + self.Kd * (error - lasterror)
		lasterror = error
		newSpeed = self.motorAvg + speed
		for i in range(2):
			if(newSpeed[i] > self.motorMax[i]):
				newSpeed[i] = self.motorMax[i]
			elif (newSpeed[i] < self.motorMin[i]):
				newSpeed[i] = self.motorMin[i]
		return newSpeed, lasterror

	def getNewPos(self, curPos, speed, dt):
		return curPos + speed * dt


def main():

	PIDx = PID()
	dt = 0.5
	threshold = 0.01    # The threshold for the error allowance
	curPos = np.array([0,0])

	desPos = [[50,20], [100,100], [30,30]]
	speed = np.array([0,0])

	positionx = [curPos[0]]
	positiony = [curPos[1]]
	for i in range(len(desPos)):
		lasterror = np.array([2,2])
		while((math.fabs(lasterror[0]) > threshold) and (math.fabs(lasterror[1]) > threshold)):
			speed, lasterror = PIDx.getNewSpeed(curPos, desPos[i], lasterror)
			curPos = PIDx.getNewPos(curPos, speed, dt)
			print lasterror, speed, curPos
			positionx.append(curPos[0])
			positiony.append(curPos[1])

	plot.plot(positionx, positiony)
	for i in range(len(desPos)):
		plot.plot(desPos[i][0], desPos[i][1], 'ro', markersize=10)
	plot.show()



if __name__ == '__main__':
	main()