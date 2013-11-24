#!/usr/bin/env python

'''
This scripts runs the obstacle detector on a live feed from ROS.
'''

import numpy as np
from ObstacleDetector import *
from LaneDetector import *
from GrassDetector import *

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

    def __init__(self, objectDetector=None, laneDetector=None, grassDetector=None):
        # Subscribe to PointCloud2
        self.subImage = rospy.Subscriber("/left/image_rect_color",\
            Image, self.cbImage, queue_size=1)
        self.subPoints = rospy.Subscriber("/points2",\
            PointCloud2, self.cbPoints, queue_size=1)

        self.images = []
        self.imageStamps = []
        self.imageInd = 0

        self.bridge = CvBridge()

        if objectDetector:
            self.objectDetector = objectDetector
        else:
            obDetect = ObstacleDetector((240,320))
            # We assume that the normal of the plane is pointing straight up
            # in the vision (-Y direction)
            obDetect.setNormalConstraint( np.array([0.,-1.,-0.2.]) )
            # And allow the search to deviate only 30 degrees from that
            obDetect.angleConstraint = 30.

        if laneDetector:
            self.laneDetector = laneDetector
        else:
            laneDetect = LaneDetector((240,320))

        if grassDetector:
            self.grassDetector = grassDetector
        else:
            # Init with H,S,V thresholds
            grassDetect = GrassDetector(45, 25, 35)

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

        pointT = point_msg.header.stamp.to_sec()

        # print 'Point Time:', pointT
        # print 'Image Times:', self.imageStamps
        image = None
        for i in range(len(self.imageStamps)):
            imageT = self.imageStamps[i]
            if imageT > pointT:
                image = self.images[i]
                break

        if image==None:
            print 'Image not assigned'
            return
        else:
            self.images = self.images[i:]
            self.imageStamps = self.imageStamps[i:]

        self.objectDetector.updateImage(image)
        self.laneDetector.updateImage(image)
        self.grassDetector.updateImage(image)

        self.objectDetector.updatePoints(cloud)

        ##### Jared, these are the 2 most useful variables for you:

        ''' mask: is a numpy array the same size as the camera image. The
            elements in the mask are set to 1 if that pixel is part of the
            plane/ground, 2 if that pixel is an object, and 0 otherwise.
        '''
        self.mask = self.objectDetector.getPlaneObjectMask()
        self.grassDetector.updateMask(mask==1)
        grassMask = self.grassDetector.findGrass()
        self.mask[grassMask] = 1

        ''' obsacleCloud: a list of 2D points relative to the camera.
            The positive X-axis goes to the right of the camera.
            The positive Y-axis goes away from the camera.
            TODO: Double check this =S
        '''
        self.obstacleCloud = self.objectDetector.getObstacleCloud()

        # Do the lane detect also
        detectedLanes = self.laneDetector.findLines()
        self.detectedLanes = np.logical_and(detectedLanes, self.mask==1)


        #self.objectDetector.showDepth()
        # Show the plane and abjects
        # im = self.objectDetector.image.copy()
        # im[grassMask] = [255,0,0]
        # im[mask==2] = [0,0,255]
        # cv2.imshow('DetectedObjects', im)

        # Show the lines
        #self.laneDetector.showImage()
        # cv2.imshow('DetectedLines', detectedLanes.astype(float)*255)


        # cv2.waitKey(15)

    def cbImage(self, img_msg):
        image = np.asarray(self.bridge.imgmsg_to_cv(img_msg))
        self.images.append(image)
        self.imageStamps.append(img_msg.header.stamp.to_sec())

        #self.objectDetector.updateImage(image)
        #self.laneDetector.updateImage(image)
        #self.grassDetector.updateImage(image)

def main():
    # Set up the ROS node for pulling in the images
    data = Ros2Python()
    rospy.init_node('Ros2Python', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down liveObstacleDetection module"

if __name__ == '__main__':
    main()



