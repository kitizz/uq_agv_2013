import cv2
import numpy as np
import time
from obstacle import *

class ObstacleMap:
	'''
	An ObstacleMap provides methods to represent a list of obstacle objects
	as an image. The map can be processed to show locations where a robot of a
	given radius cannot go.
	The ObstacleMap needs to know how many units per pixel in order to draw to
	the correct scale.
	'''

	def __init__(self, unitsPerPixel, widthUnits, heightUnits, robotRadius):
		self.width = widthUnits/unitsPerPixel
		self.height = heightUnits/unitsPerPixel
		self.unitsPerPixel = unitsPerPixel
		self.robotRadius = robotRadius

		self.imageMap = np.zeros((self.height, self.width))

	def addObstacle(self, obstacle):
		(x, y, radius) = (obstacle.x, obstacle.y, obstacle.radius)
		cv2.circle(self.imageMap, x/unitsPerPixel, y/unitsPerPixel, radius/unitsPerPixel, (255, 0, 0))

	def addObstacles(self, obstacles):
		for obstacle in obstacles:
			addObstacle(obstacle)


	def closeMap(self):
		


def main():
	obMap = ObstacleMap(0.02, 20, 15)
	cv2.imshow('Test', obMap.imageMap)
	time.sleep(2)

if __name__ == '__main__':
	main()