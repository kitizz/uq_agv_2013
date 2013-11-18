
import cv2
import numpy as np

import SACModelPlane
import RRansac

class ObstacleDetector:

	def __init__(self, size):
		self.image = np.zeros(size)
		self.depth = np.zeros(size)
		self.points = np.zeros(size)
		self.coeff = np.array([0.,0.,1.,-15.])
		self.coeffRating = 1.

	def updateImage(self, image_msg):
		self.image = image_msg.image

	def updateDepth(self, depth):
		self.depth = depth

	def updatePoints(self, point_msg):
		self.points = point_msg.points
		self.update()

	@staticmethod
	def goodCoeff(coeff):
		maxTheta = np.deg2rad(45)
		norm = coeff[0:3]
		des = np.array([0.,1.,0.])
		theta = np.arccos( np.dot(norm, des) )
		return theta <= maxTheta

	def update(self):
		# This is where the magic happens
		depth = np.nan_to_num(self.points[...,...,2])
		depth /= 50.
		cv2.imshow('Depth', depth)

		model = SACModelPlane.SACModelPlane(self.points)

		startCoeff = None
		if self.coeffRating > 0.3: # 30 percent coverage
			success, initialFit = model.selectWithinDistance(self.coeff, 1.)
			if len(initialFit) > 10: startCoeff = self.coeff
			print 'Initial Fit:', len(initialFit)

		ran = RRansac.RRansac(model, 1.5)
		ran.maxIterations = 200
		ran.fractionPretest = 0.01

		# cv2.waitKey()

		success, self.coeff, inlierInd, select = ran.computeModel(startCoeff, constraint=self.goodCoeff)
		if success:
			N = float(len(inlierInd))
			self.coeffRating = N/len(model.indices)

			# Ensure that the plane is always facing "up" (Y-axis negative)
			if np.sign(self.coeff[1]) < 0:
				self.coeff *= -1

			print "Inliers:", N
			print "Coeffs:", self.coeff
			# Convert the image to colour so we can draw on it
			imageVis = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)
			imCoord = model.ind2coord(inlierInd)
			# imageVis[imCoord][0:2] = 0. # Highlight plane in red
			imageVis[imCoord] = [0, 0, 255]

			# From points not defined as the plane, we define object points as
			# points that are above the ground plane (negative side)
			outlierInd = np.setdiff1d(model.indices, inlierInd)
			success, dist = model.getDistancesToModel(self.coeff, \
				subset=outlierInd, signed=True)
			if success:
				objectInd = outlierInd[dist<0]
				objectCoord = model.ind2coord(objectInd)
				# imageVis[objectCoord][1:] = 0.
				imageVis[objectCoord] = [255, 0, 0]

			cv2.imshow('Plane', imageVis)
			cv2.waitKey(10)

class PointsMsg:
	def __init__(self,stamp,cloud):
		self.stamp = stamp
		(h, w) = cloud.shape
		# self.image = np.zeros((h,w,3))
		self.points = np.zeros((h,w,3))

		# for x in range(w):
		# 	for y in range(h):
		# 		self.image[y,x] = float2rgb(cloud['rgb'][y,x])
		self.points[...,...,0] = cloud['x']
		self.points[...,...,1] = cloud['y']
		self.points[...,...,2] = cloud['z']

class ImageMsg:
	def __init__(self, stamp, image):
		self.stamp = stamp
		self.image = image


import struct
def float2rgb(fl):
	I = struct.unpack('I', fl)[0]
	rgb = [(I>>16)&255, (I>>8)&255, I&255]
	return rgb


def main():
	from os import listdir
	from os.path import isfile, join
	mypath = 'data/'
	onlyfiles = [ f for f in listdir(mypath) if isfile(join(mypath,f)) ]
	nImages = 0
	nDepths = 0
	nPoints = 0

	images = []
	depths = []
	points = []

	for f in onlyfiles:
		name = f.split('.')[0]
		(label,cnt) = (name[0:-4], int(name[-4:]))
		if label == 'image' and cnt > nImages:
			A = np.load(mypath + f)
			images.append(ImageMsg(A[0], A[1]))
			nImages = cnt
		# elif label == 'depth' and cnt > nDepths:
		# 	nDepths = cnt
		# el
		elif label == 'points' and cnt > nPoints:
			print cnt
			A = np.load(mypath + f)
			points.append(PointsMsg(A[0], A[1]))
			nPoints = cnt


	print "Images:", nImages, "Depths:", nDepths, "Points:", nPoints

	# Now let's load them in, in the correct order to the detector
	indImg = 0
	indPnt = 0
	obDetect = ObstacleDetector((240,320))
	lastImage = None
	lastPoints = None
	while indImg < nImages and indPnt < nPoints:
		if indPnt >= nPoints or \
		(indImg < nImages and images[indImg].stamp < points[indPnt].stamp):
			obDetect.updateImage(images[indImg])
			lastImage = images[indImg]
			indImg += 1
			# cv2.imshow('Loading', lastImage.image)
			# cv2.waitKey(30)

		else:
			obDetect.updatePoints(points[indPnt])
			lastPoints = points[indPnt]
			indPnt += 1

	# Let's give it a spin...
	# obDetect.updateImage(lastImage)
	# obDetect.updatePoints(lastPoints)


if __name__ == '__main__':
	main()

