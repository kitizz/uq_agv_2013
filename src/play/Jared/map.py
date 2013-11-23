import cv2
import math
from numpy import *

class map:
	''' All dimensions in metres'''

	def __init__(self, height=0, width=0, color=0,test=0):
		self.height = height;
		self.width = width;
		self.color = color;
		self.map = color*ones((height,width,3), uint8)
		if test == 1:
		    self.map == cv2.imread("mapori.bmp")

	def getHeight(self):
		return self.height

	def getWidth(self):
		return self.width

	def getMap(self):
		return self.map
	
	def getImage(self):
	    return self.open

	def placeObstacle(self,obs,ver):        
	    if ver == 0:
	        cv2.circle(self.map,(obs.x,obs.y),int(obs.radius*1.1),(0,0,255),thickness=-1)
	        cv2.circle(self.map,(obs.x,obs.y),obs.radius,(0,0,0),thickness=-1)
	    elif ver == 3:
	        cv2.circle(self.map,(obs.x,obs.y),int(obs.radius*1.1),(0,255,0),thickness=-1)
	        cv2.circle(self.map,(obs.x,obs.y),obs.radius,(0,0,0),thickness=-1)
	    else:            
	        cv2.circle(self.map,(obs.x,obs.y),obs.radius/2,(255,0,0),thickness=-1)
	        cv2.circle(self.open,(obs.x,obs.y),obs.radius/2,(255,0,0),thickness=-1)

	def placeRobot(self,x,y,radius):        
		cv2.circle(self.open,(x,y),radius,(0,0,255),thickness=-1)

	def updateObstacles(self,obsList):
	    for i in range(len(obsList)):
	       self.placeObstacle(obsList[i],0)
	        #obsList[i].place(self.map)    

	def updateActObstacles(self,obsList):
	    for i in range(len(obsList)):
	       self.placeObstacle(obsList[i],1)
	        #obsList[i].place(self.map)    

	def updateMorph(self):
	    dilSize = 20;
	    kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( dilSize , dilSize  ) );
	    #erosion = cv2.erode(img,kernel,iterations = 1)
	    #dilate = cv2.erode(img,kernel,iterations = 1)
	    self.open = cv2.morphologyEx(self.map, cv2.MORPH_OPEN, kernel,iterations = 1)

	def refreshMap(self):
	    updateObstacles()
	    updateMorph()