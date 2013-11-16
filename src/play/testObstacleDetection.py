

import numpy as np

class ObstacleDetector:

	def __init__(self, size):
		self.image = np.zeros(size)
		self.depth = np.zeros(size)
		self.points = np.zeros(size)

	def updateImage(self, image_msg):
		self.image = image_msg.image

	def updateDepth(self, depth):
		self.depth = depth

	def updatePoints(self, point_msg):
		self.points = point_msg.points
		self.update()

	def update(self):
		
		pass

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
	while indImg < nImages and indPnt < nPoints:
		if indPnt >= nPoints or (indImg < nImages and images[indImg].stamp < points[indPnt].stamp):
			obDetect.updateImage(images[indImg])
			indImg += 1

		else:
			obDetect.updateImage(points[indPnt])
			indPnt += 1


if __name__ == '__main__':
	main()

