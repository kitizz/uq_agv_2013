#!/usr/bin/env python

import cv2
import math
import numpy as np
#print cv2.__version__

class LaneDetector:
	'''Grass Lane Detection'''

	def __init__(self, size):
		self.size = size
		self.image = np.zeros(size)
		self.houghPImg = np.zeros(size)
		self.houghImg = np.zeros(size)

	def updateImage(self, image):
		self.image = image

	def findLines(self):
		h,w = self.image.shape[0:2]
		if (h,w) != self.size:
			print "Received image of bad size. Required (h,w): (%d,%d). Got: (%d,%d)" % (h,w,self.size[0],self.size[1])
			return

		#Kernel
		kernel10 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
		
		#Apply Morph Transforms
		image = cv2.GaussianBlur(self.image, (5, 5), 0)
		image = cv2.morphologyEx(image, cv2.MORPH_DILATE, kernel10)
		#Canny image
		self.contours = cv2.Canny(image,50,100)

		#PROBABALISTIC HOUGH TRANSFORM-----------
		self.houghPImg[...,...] = 0
		houghP = cv2.HoughLinesP(self.contours,2, 5.*math.pi/180, 2 )

		for i in range(0,len(houghP[0])):
			cv2.line(self.houghPImg,(houghP[0][i][0], houghP[0][i][1]),(houghP[0][i][2],houghP[0][i][3]),255,2)
		
		#HOUGH TRANSFORM-----------
		self.houghImg[...,...] = 0
		hough = cv2.HoughLines(self.contours,2, 5.*math.pi/180, 60)

		nLines = 0
		for i in range(0,len(hough[0])):
			rho= hough[0][i][0]
			theta= hough[0][i][1]
			if theta < math.pi/3.5 or theta > 3.*math.pi/3.5:
				if nLines > 10: break
				nLines += 1
				#point of intersection of the line with first row
				pt1 = (int(rho/math.cos(theta)),0)
				#point of intersection of the line with last row
				pt2 = (int((rho-h*math.sin(theta))/math.cos(theta)),h)
				#Draw a white line
				cv2.line(self.houghImg, pt1, pt2, 255, 5)

		#Apply a logical 'and' to both images 
		self.detectedLines = np.logical_and(self.houghPImg, self.houghImg)
		# if self.detectedLines:
		# 	self.detectedLines = cv2.morphologyEx(self.detectedLines, cv2.MORPH_OPEN, np.array(0),1)
		return self.detectedLines


	def showImage(self):
		#cv2.imshow("Contours", self.contours)
		#cv2.imshow("Hough", self.houghImg)
		#cv2.imshow("HoughP", self.houghPImg)
		cv2.imshow("Masked", self.detectedLines.astype(float)*255)
		#cv2.waitKey(0)

def main():
	image = cv2.imread("track.jpg")
	TEG = LaneDetector(image.shape[0:2])
	TEG.updateImage(image)
	TEG.findLines()

if __name__ == '__main__':
	main()

