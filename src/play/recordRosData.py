#!/usr/bin/env python

import sys, time

import cv2
import roslib
import rospy
from sensor_msgs.msg import Image, PointCloud2
from stereo_msgs.msg import DisparityImage
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

VERBOSE=False

def points2cloud_to_array(cloud_msg):
    '''
    Converts a rospy PointCloud2 message to a numpy recordarray
    Assumes all fields 32 bit floats, and there is no padding.
    '''
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

class cloud2depth:

    def __init__(self):
        # Subscribe to PointCloud2
        self.subDisp = rospy.Subscriber("/disparity", DisparityImage,
            self.cbDisparity, queue_size=1)
        self.subImage = rospy.Subscriber("/left/image_rect", Image, self.cbImage, queue_size=1)
        self.subPoints = rospy.Subscriber("/points2", PointCloud2, self.cbPoints, queue_size=1)

        self.firstMsg = True
        self.bridge = CvBridge()

        self.imgPath = 'data/image'
        self.depthPath = 'data/depth'
        self.pointPath = 'data/points'
        self.imgCnt = 0
        self.depthCnt = 0
        self.pointCnt = 0

        if VERBOSE:
            print "Subscribed to /disparity"
            print "Subscribed to /left/image_rect"

    def cbPoints(self, point_msg):
        if VERBOSE:
            print 'Received points. Size: (', point_msg.width, ',', point_msg.height, ')'
        # Convert to numpy array (http://goo.gl/s6OGja)
        cloud = np.array(points2cloud_to_array(point_msg))

        f = open(("%s%04d.npy" % (self.pointPath, self.pointCnt)), 'w')
        self.pointCnt += 1
        A = np.array([point_msg.header.stamp.to_sec(), cloud.copy()],
            dtype=object)
        np.save(f, A)
        f.close()
        

    def cbDisparity(self, disp_msg):
        if VERBOSE:
            print 'Received points. Size: (', disp_msg.image.width, ',', disp_msg.image.height, ')', disp_msg.image.step
        # Convert to numpy array ()
        disparity = np.asarray(self.bridge.imgmsg_to_cv(disp_msg.image))
        f = disp_msg.f
        T = disp_msg.T
        depth = np.nan_to_num(f*T*np.reciprocal(disparity))

        if self.firstMsg and VERBOSE:
            print "BaseLine:", T
            print depth.shape
            self.firstMsg = False
        if VERBOSE:
            cv2.imwrite('depth.jpg', depth)


        depth[depth<0] = 0
        #self.depths.append(depth)
        f = open(("%s%04d.npy" % (self.depthPath, self.depthCnt)), 'w')
        self.depthCnt += 1
        A = np.array([disp_msg.header.stamp.to_sec(), depth.copy()],
            dtype=object)
        np.save(f,A)
        f.close()

    def cbImage(self, img_msg):
        image = np.asarray(self.bridge.imgmsg_to_cv(img_msg))
        f = open(("%s%04d.npy" % (self.imgPath, self.imgCnt)), 'w')
        self.imgCnt += 1
        A = np.array([img_msg.header.stamp.to_sec(), image.copy()],
            dtype=object)
        np.save(f, A)
        f.close()
        #self.images.append(image)

def main(args):
    global c2d
    c2d = cloud2depth()
    rospy.init_node('cloud2depth', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cloud2depth module"
    except rospy.ROSInterruptException:
        print "Interrupted. Shutting Down..."

import signal
def signal_handler(signal, frame):
    global c2d
    filename = 'tmpRecorded'
    nImg = c2d.imgCnt
    nDepths = c2d.depthCnt
    nPoints = c2d.pointCnt
    print ('Interrupted. Saving %d images, %d depthmaps, and %d point clouds to' % (nImg, nDepths, nPoints)), ('%s<Images/Depths>.npy' % filename)
    print 'Use np.load(file) to read it in.'
    #fImg = open(('%sImages.npy' % filename),'w')
    #fDepth = open(('%sDepths.npy' % filename),'w')
    #np.save(fImg, c2d.images)
    #np.save(fDepth, c2d.depths)
    #fImg.close()
    #fDepth.close()

    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main(sys.argv)


