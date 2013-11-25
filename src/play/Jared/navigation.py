import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import genObjects
import loader
import obstacle
import map
from robotSim import Robot
from imageReader import ImageReader
from point import point
import mapper
import ConfigParser
import path
#import dstar
#import dstar2
#import dstar3
import dlite
from vehicle import *
from PIL import Image
import random
from scipy.misc import comb

class Navigation:
	"Navigation of a vehicle through a map"

	def __init__(self, map, obs, vehicle):
		self.map = map
		self.obsList = obs
		self.vehicle = vehicle
		self.goal = 0

	def calculatePath(self):
		height = self.map.height
		width = self.map.width
		newHeight = int(height*0.1)
		newWidth = int(width*0.1)
		dliteimage = cv2.resize(self.map.getImage(),(newWidth,newHeight))
		cv2.imwrite('AGVCmap2.bmp', dliteimage)
		robot = Robot(self.vehicle.x,self.vehicle.y,self.vehicle.radius*2)
		imageMap = ImageReader()
		imageMap.loadFile("AGVCmap2.bmp")
		mapper.initalize(imageMap,robot)
		moveGrid = imageMap.convertToGrid().copy()

		##goal = point(3,17)
		testdivider = 1
		self.goal = point(int(newHeight/testdivider*0.8),int(newWidth/testdivider*0.8))
		#cv2.waitKey(0)

		##mapper.printMoveGrid()

		print "STARTIN LOOP"
		moveId=0
		Xlength = mapper.grid.shape[0]/testdivider
		Ylength = mapper.grid.shape[1]/testdivider
		#dstar = dstar3.DStar(Xlength,Ylength,goal)
		dstar = dlite.Dlite(Xlength,Ylength,self.goal,robot)
		print "Entering Loop"
		testvar = 0

		#for i in range(10):
		while (robot.y != self.goal.y or robot.x != self.goal.x) :
			if testvar%2 == 0:
				newObs = obstacle.Obstacle(random.randint(0,height),random.randint(0,width), 40)
				self.map.placeObstacle(newObs,3)
				self.obsList.append(newObs)
				#Place obstacles on map
				self.map.updateObstacles(self.obsList)

				#Morph the obstacles
				self.map.updateMorph();
				dliteimage = cv2.resize(self.map.getImage(),(newWidth,newHeight))
				cv2.imwrite('AGVCmap2.bmp', dliteimage)
				imageMap.loadFile("AGVCmap2.bmp")
				mapper.initalize(imageMap,robot)
				moveGrid = imageMap.convertToGrid().copy()
				
			testvar = testvar + 1

			moveId = moveId+1
			print moveId
			if path.pathIsBroken(mapper.grid) :
				path.restart()
				print "The path is broken"
				#  dstar2.dstar(mapper, robot, goal, path)

				dstar.dstar(mapper, robot, self.goal, path)
		 
			#dlite.dstar(robot,goal,path)
			 #  #  DstarLite.dstar(mapper, robot, goal, path)
			  #  astar.astar(mapper, robot, goal, path)
			pathNode=path.getNextMove()
			robot.x = pathNode.x
			robot.y = pathNode.y
			mapper.moveGrid[pathNode.x][pathNode.y]="1"
			#mapper.printMoveGrid()
			self.vehicle.x = pathNode.x
			self.vehicle.y = pathNode.y
			self.vehicle.addPosition(pathNode.x,pathNode.y)
			mapper.updateMap(robot)
			#raw_input("TEST")
			cv2.imshow('AGVC Map', self.map.getMap())
			cv2.imshow('AGVC Map Morphed', self.map.getImage())
			for i in range(len(self.vehicle.positions)):
				self.map.placeRobot(self.vehicle.positions[i][1]*height/newHeight,self.vehicle.positions[i][0]*width/newWidth,self.vehicle.radius)
			nPoints = len(self.vehicle.positions)
			points = np.array(self.vehicle.positions)*height/newHeight
			#points = np.random.rand(nPoints,2)*200
			xpoints = [p[0] for p in points]
			ypoints = [p[1] for p in points]

			xvals, yvals = self.bezier_curve(points, nTimes=1000)
			for i in range(len(xvals)):
				self.map.placeRobot(int(yvals[i]),int(xvals[i]),self.vehicle.radius*2)

			self.map.updateActObstacles(self.obsList)
        
			cv2.waitKey(0)

			#self.goal = point(int(newHeight/testdivider*i/10),int(newWidth/testdivider*i/10))

	def bernstein_poly(seflf,i, n, t):
		"""
		 The Bernstein polynomial of n, i as a function of t
		"""

		return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


	def bezier_curve(self, points, nTimes=1000):
		"""
		   Given a set of control points, return the
		   bezier curve defined by the control points.

		   points should be a list of lists, or list of tuples
		   such as [ [1,1], 
					 [2,3], 
					 [4,5], ..[Xn, Yn] ]
			nTimes is the number of time steps, defaults to 1000

			See http://processingjs.nihongoresources.com/bezierinfo/
		"""

		nPoints = len(points)
		xPoints = np.array([p[0] for p in points])
		yPoints = np.array([p[1] for p in points])

		t = np.linspace(0.0, 1.0, nTimes)

		polynomial_array = np.array([ self.bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

		xvals = np.dot(xPoints, polynomial_array)
		yvals = np.dot(yPoints, polynomial_array)

		return xvals, yvals


	def column(matrix, i):
		return [row[i] for row in matrix]


def main():
	 #Initialise Vehicle Class
	TEG =  Vehicle(0,0,2)
	#Initialise blank image
	height = 500
	width = 500  
	#Set the current map class to blank
	agvcmap = map.map(height, width, 255)
	#Generate a list of objects
	genObjects.main(agvcmap)
	#Retrieve objects from file
	obsList = loader.getObs()

	#M Mapping of obstacles and lines onto a pixel image and skeleton planning returning GPS (or distance) waypoints 
		
	#Place obstacles on map
	agvcmap.updateObstacles(obsList)

	#Morph the obstacles
	agvcmap.updateMorph();

	nav = Navigation(agvcmap,obsList,TEG)

	nav.calculatePath()


if __name__ == "__main__":
	main()