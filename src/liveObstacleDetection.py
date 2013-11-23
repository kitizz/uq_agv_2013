#!/usr/bin/env python

'''
This scripts runs the obstacle detector on a live feed from ROS.
'''

import numpy as np
from ObstacleDetector import *
from LaneDetector import *

import sys, time

import cv2
import roslib
import rospy
from sensor_msgs.msg import Image, PointCloud2
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

# Set this to true to see printed output...
VERBOSE = False

def points2cloud_to_array(cloud_msg):
    '''
    Converts a rospy PointCloud2 message to a numpy recordarray
    Assumes all fields 32 bit floats, and there is no padding.
    '''
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

class Ros2Python:

	def __init__(self, objectDetector=None, laneDetector=None):
		# Subscribe to PointCloud2
		self.subImage = rospy.Subscriber("/left/image_rect_color",\
			Image, self.cbImage, queue_size=1)
		self.subPoints = rospy.Subscriber("/points2",\
			PointCloud2, self.cbPoints, queue_size=1)

		self.bridge = CvBridge()

		self.objectDetector = objectDetector
		self.laneDetector = laneDetector

		if VERBOSE:
			print "Subscribed to /left/image_rect_color"
			print "Subscribed to /points2"

	def cbPoints(self, point_msg):
		if VERBOSE:
			print 'Received points. Size: (', point_msg.width, ',', point_msg.height, ')'
		# Convert to numpy array (http://goo.gl/s6OGja)
		cloud = np.array(points2cloud_to_array(point_msg))

		# ObDetector only recalculates everything when the cloud is updated
		cloud = np.dstack((cloud['x'], cloud['y'], cloud['z']))
		print cloud.dtype
		self.objectDetector.updatePoints(cloud)

		##### Jared, these are the 2 most useful variables for you:

		''' mask: is a numpy array the same size as the camera image. The
			elements in the mask are set to 1 if that pixel is part of the
			plane/ground, 2 if that pixel is an object, and 0 otherwise.
		'''
		mask = self.objectDetector.getPlaneObjectMask()

		''' obsacleCloud: a list of 2D points relative to the camera.
			The positive X-axis goes to the right of the camera.
			The positive Y-axis goes away from the camera.
			TODO: Double check this =S
		'''
		obstacleCloud = self.objectDetector.getObstacleCloud()

		# Do the lane detect also
		detectedLanes = self.laneDetector.findLines()

	def cbImage(self, img_msg):
		image = np.asarray(self.bridge.imgmsg_to_cv(img_msg))
		#self.images.append(image)
		self.objectDetector.updateImage(image)
		self.laneDetector.updateImage(image)

def main():
	laneDetect = LaneDetector((240,320))
	obDetect = ObstacleDetector((240,320))
	# We assume that the normal of the plane is pointing straight up in the
	# vision (-Y direction)
	obDetect.setNormalConstraint( np.array([0.,-1.,0.]) )
	# And allow the search to deviate only 30 degrees from that
	obDetect.angleConstraint = 30.

	# Set up the ROS node for pulling in the images
	data = Ros2Python(obDetect,laneDetect)
	rospy.init_node('Ros2Python', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down liveObstacleDetection module"

if __name__ == '__main__':
	main()




