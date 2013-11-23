import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
#print cv2.__version__

class Lanes:
	'''Grass Lane Detection'''

	def __init__(self, image):
		self.image = image;

	def findLines(self):
		#Kernel
		kernel10 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
		image = cv2.resize(self.image, (500,500))
		h,w = image.shape[0:2]
		#Apply Morph Transforms
		image = cv2.GaussianBlur(image, (5, 5), 0)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
		image = cv2.morphologyEx(image, cv2.MORPH_DILATE, kernel10)
		#Canny image
		contours = cv2.Canny(image,50,100)
		#PROBABALISTIC HOUGH TRANSFORM-----------
		houghPimg = np.zeros((h,w))
		houghP = cv2.HoughLinesP(contours,2, 5.*math.pi/180, 1, 0,0 )

		for i in range(0,len(houghP[0])):
			cv2.line(houghPimg,(houghP[0][i][0], houghP[0][i][1]),(houghP[0][i][2],houghP[0][i][3]),255,2)
		
		#HOUGH TRANSFORM-----------
		houghimg = np.zeros((h,w))
		hough = cv2.HoughLines(contours,2, 5.*math.pi/180, 100,0,0)

		nLines = 0
		for i in range(0,len(hough[0])):
			rho= hough[0][i][0]
			theta= hough[0][i][1]
			if theta<math.pi/4 or theta > 3. * math.pi/4:
				if nLines > 10: break
				nLines += 1
				#point of intersection of the line with first row
				pt1 = (int(rho/math.cos(theta)),0)
				#point of intersection of the line with last row
				pt2 = (int((rho-h*math.sin(theta))/math.cos(theta)),h)
				#Draw a white line
				cv2.line(houghimg, pt1, pt2, 255, 10)

		#Apply a logical 'and' to both images 
		houghMasked = np.logical_and(houghPimg, houghimg)

		#Set class
		self.houghimg = houghimg
		self.houghPimg = houghPimg
		self.houghMasked = houghMasked.astype(float)*255
		self.contours = contours

		cv2.waitKey(0)


	def showImage(self):
		cv2.imshow("Contours", self.contours)
		cv2.imshow("Hough", self.houghimg)
		cv2.imshow("HoughP", self.houghPimg)
		cv2.imshow("Masked", self.houghMasked.astype(float)*255)
		cv2.waitKey(0)


if __name__ == "__main__":
	image = cv2.imread("track.jpg")
	TEG = Lanes(image)
	TEG.findLines()
	TEG.showImage()