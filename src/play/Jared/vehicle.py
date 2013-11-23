import math 
from numpy import *

class Vehicle:
	''' The Elderly Gentleman'''

	def __init__(self, x=0, y=0, radius=1):
		self.x = x;
		self.y = y;
		self.positions = []
		self.positions.append((self.x,self.y))
		self.radius = radius;

	def getSpeed(self):
		# Return speed from GPS
		return 1

	def getHeading(self):
		#Return heading from GPS or Compass
		return 1
	def setPosition(self,x,y):
		self.x = x
		self.y = y

	def getPosition(self):
		return array([self.x,self.y])

	def addPosition(self,x,y):
		self.positions.append((x,y))

	def getNextPosition(self,goal):
		return array([self.x,self.y])