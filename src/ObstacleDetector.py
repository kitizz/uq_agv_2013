#!/usr/bin/env python

import cv2
import numpy as np

import SACModelPlane
import RRansac

import LaneDetector

colors = np.array([[255, 0, 0],
    [0,255,0],
    [0,0,255],
    [255,255,0],
    [0,255,255],
    [255,0,255]])

class ObstacleDetector:

    def __init__(self, size):
        self.size = size
        
        self.image = np.zeros(size)
        self.depth = np.zeros(size)
        self.points = np.zeros(size)
        self.coeff = np.array([0.,0.,1.,-15.])
        self.coeffRating = 1.

        self.model = None

        self.planeObjectMask = np.zeros(size)
        self.obstacleCloud = np.array([])

        self.normalConstraint = np.array([0.,0.,0.])
        self.angleConstraint = np.inf # Degrees

    def setNormalConstraint(self, newNormal):
        L = np.linalg.norm(newNormal)
        if L == 0: L =1 # Just let the newNormal be a zero vector
        self.normalConstraint = newNormal / L

    def setAngleConstraint(self, newAngle):
        self.angleConstraint = newAngle

    def getPlaneObjectMask(self):
        '''
        The plane is a numpy array the same size as the camera image. The
        elements in the mask are set to 1 if that pixel is part of the
        plane/ground, 2 if that pixel is an object, and 0 otherwise.
        '''
        return self.planeObjectMask

    def getObstacleCloud(self):
        '''
        ObstacleCloud is a list of 2D points relative to the camera.
        The positive X-axis goes to the right of the camera.
        The positive Y-axis goes away from the camera.
        TODO: Double check this =S
        '''
        return self.obstacleCloud

    def get2DCloudFromMask(self, mask):
        '''
        Given a mask, return a 2D point cloud (see all 'getObstacleCloud')
        containing points *that exist* for selected pixels in the mask.
        '''
        coord = np.array( np.where(mask>0) ).T
        ind = self.model.coord2ind(coord)
        return self.model.points[ind,:][:,[0,2]]

    def get3DCloudFromMask(self, mask):
        '''
        Given a mask, return a 3D point cloud (see all 'getObstacleCloud')
        containing points *that exist* for selected pixels in the mask.
        '''
        coord = np.array( np.where(mask>0) ).T
        ind = self.model.coord2ind(coord)
        return self.model.points[ind,:]

    def updateImage(self, image_msg):
        if type(image_msg) == np.ndarray:
            self.image = image_msg
        else:
            self.image = image_msg.image

    def updatePoints(self, point_msg, updateAll=True):
        if type(point_msg) == np.ndarray:
            self.points = point_msg
        else:
            self.points = point_msg.points

        if updateAll: self.update()

    @staticmethod
    def goodCoeff(coeff, normal, angle):
        maxTheta = np.deg2rad(angle)
        planeNorm = coeff[0:3]
        theta = np.abs( np.arccos( np.dot(planeNorm, normal) ) )
        return theta <= maxTheta

    def update(self):
        # This is where the magic happens
        self.depth = np.nan_to_num(self.points[...,...,2])
        #` 'Points', self.points.shape
        # self.depth /= 10.
        # cv2.imshow('Depth', self.depth/10.)

        self.model = SACModelPlane.SACModelPlane(self.points)

        startCoeff = None
        if self.coeffRating > 0.3: # 30 percent coverage
            success, initialFit = self.model.selectWithinDistance(self.coeff, 0.1)
            if len(initialFit) > 10: startCoeff = self.coeff
            #print 'Initial Fit:', len(initialFit)

        ran = RRansac.RRansac(self.model, 0.15)
        ran.maxIterations = 200
        ran.fractionPretest = 0.01

        # cv2.waitKey()
        constraintFunc = lambda x: \
            self.goodCoeff(x, self.normalConstraint, self.angleConstraint)

        # Run the R-Ransac algorithm to look for planes. We give it a warm
        # start with the coefficients of the previous frame
        success, self.coeff, inlierInd, select = ran.computeModel(startCoeff, constraint=constraintFunc)
        if not success: return # Let's not waste our time

        # Fine tune
        # success, self.coeff = model.optimiseModelCoefficients(inlierInd, self.coeff)
        # success, inliearInd = model.selectWithinDistance(self.coeff, 0.15)

        N = float(len(inlierInd))
        self.coeffRating = N/len(self.model.indices)

        # Ensure that the plane is always facing "up" (Y-axis negative)
        if np.sign(self.coeff[1]) > 0:
            self.coeff *= -1

        # From points not defined as the plane, we define object points as
        # points that are above the ground plane (negative side)
        outlierInd = np.setdiff1d(self.model.indices, inlierInd)
        success, dist = self.model.getDistancesToModel(self.coeff, \
            subset=outlierInd, signed=True)
        if not success: return

        objectInd = outlierInd[dist>0]

        # Clear and remark the mask
        self.planeObjectMask[...,...] = 0
        self.planeObjectMask[self.model.ind2coord(inlierInd)] = 1
        self.planeObjectMask[self.model.ind2coord(objectInd)] = 2
        
        # self.showObstacles(objectInd, inlierInd)

        # Get the locations of the obstacles in the plane
        #rot = self.getRotationMatrixOfPlane(self.coeff).T
        #projected = np.dot( rot, model.points[objectInd].T ).T
        #self.obstacleCloud = projected[...,[0,2]]
        #print 'Getting Obstacle Cloud', model.points.shape, type(model.points)
        #print objectInd.shape

        # We retrieve the 3D points for the given object indices. And we're
        # interested in the in-plane locations of them. So X+Z in camera coords
        self.obstacleCloud = self.model.points[objectInd,:][:,[0,2]]
        # self.showMap(obstacleCloud)
        
        # cv2.waitKey(10)

    def showDepth(self):
        cv2.imshow('Depth', self.depth/10.)

    def showDetected(self, mask):
        if mask == None: return
        #print 'Showing Detected'
        im = self.image.copy()
        im[mask==1] = [255,0,0]
        im[mask==2] = [0,0,255]
        #print im.shape
        cv2.imshow('DetectedObjects', im)

    def showObstacles(self, obstacleIndices, planeIndices=None):
        '''
        Show the plane and obstacles as horrible colours on overlaid on the
        original image.
        '''
        drawPlane = planeIndices != None and len(planeIndices) > 0
        drawObstacles = obstacleIndices == None or len(obstacleIndices) == 0
        if not drawPlane and not drawObstacles: return
        

        # Convert the image to colour so we can draw on it
        imageVis = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)

        if drawPlane:
            imCoord = model.ind2coord(planeIndices)
            # imageVis[imCoord][0:2] = 0. # Highlight plane in red
            imageVis[imCoord] = [0, 0, 255]

        if drawObstacles:
            objectCoord = model.ind2coord(obstacleIndices)
            # imageVis[objectCoord][1:] = 0.
            imageVis[objectCoord] = [255, 0, 0]

        cv2.imshow('Plane', imageVis)


    def showMap(self, obstacleCloud):
        '''
        Show the 2D obstacle cloud. Maybe cluste them... but this is
        quite computationally expensive.
        '''
        # Do some clustering on the obstacle cloud
        clusters = self.findClusters(obstacleCloud, 0.3)

        # Convert to pixel locations 5cm = 1pxl
        scale = 50.
        viewDimensions = np.array([10., 10.]) # 10x10m
        imageDimensions = np.round(viewDimensions*scale)
        obstacleImage = np.zeros(np.append( viewDimensions*scale, 3 ))
        colorIndex = 0
        for cluster in clusters:
            cluster = (cluster+viewDimensions/2.)*scale # Centers cloud
            cluster = np.round(cluster).astype(int)

            # Remove out of bounds
            inBounds = np.logical_and(cluster > [0, 0],
                cluster < imageDimensions)
            inBounds = inBounds.all(axis=1)
            cluster = cluster[inBounds]

            obstacleImage[tuple(cluster.T)] = colors[colorIndex]
            colorIndex = (colorIndex+1) % len(colors)
        cv2.imshow('Map', obstacleImage)

    def getRotationMatrixOfPlane(self, coeff):
        '''
        Finds the rotation matrix that will obtain the positions of the points
        in the plane reference frame.
        The plane normal is the new Y-axis.
        The plane's Z-axis is constrained to the world YZ plane to keep it
        pointing forwards.
        The X-axis is simply the cross product of the Y and Z.
        '''
        Y = coeff[0:3]
        Y /= np.linalg.norm(Y)

        # Z is 90 deg rotation
        Z = np.array([0., -Y[2], Y[1]])
        Z /= np.linalg.norm(Z)

        # X has no option but to be the cross product of YZ
        X = np.cross(Y, Z)

        return np.column_stack((X,Y,Z))

    def findClusters(self, points2d, radius):
        '''
        Note: This isn't an ordinary clustering.
        2 points are part of a cluster if they are within a specified radius
        of each other.
        Return clusters (a list of lists, each list is a cluster which refers
            to the indices of points2d in the cluster)
        '''
        return [points2d]
        r2 = radius**2
        len2 = lambda p: (p[0]**2 + p[1]**2)

        m = 0 # Keep track of the latest cluster label
        N = len(points2d)
        # Label the points with the cluster number. -1 is no cluster.
        label = -np.ones(N)
        for i in range(N):
            if label[i] >= 0: continue # Already labelled.
            p1 = points2d[i]

            # Get a list of the points nearby and remember if any are labelled
            closePoints = []
            currentLabel = -1
            
            for j in range(i+1, N):
                # No need to check labelled points if we already have a label
                if currentLabel >= 0 and label[j] >= 0: continue

                p2 = points2d[j]
                dist2 = len2(p1-p2) # Get the squared distance
                if dist2 <= r2:
                    closePoints.append(j)
                    if label[j] >= 0: currentLabel = label[j]

            if len(closePoints) == 0: continue # Nothing close
            # Check if we need to grab a new label
            if currentLabel < 0:
                currentLabel = m
                m += 1

            # Label them up
            for j in closePoints:
                label[j] = currentLabel

        clusters = [[] for x in range(m)]
        # Now we forumulate the clusters in list form
        for i in range(N):
            l = int(label[i])
            if l >= 0: clusters[l].append(points2d[i])

        return np.array(clusters)


class PointsMsg:
    def __init__(self,stamp,cloud):
        self.stamp = stamp
        (h, w) = cloud.shape
        # self.image = np.zeros((h,w,3))
        self.points = np.zeros((h,w,3))

        # for x in range(w):
        #   for y in range(h):
        #       self.image[y,x] = float2rgb(cloud['rgb'][y,x])
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
        #   nDepths = cnt
        # el
        elif label == 'points' and cnt > nPoints:
            #print cnt
            A = np.load(mypath + f)
            points.append(PointsMsg(A[0], A[1]))
            nPoints = cnt


    #print "Images:", nImages, "Depths:", nDepths, "Points:", nPoints

    # Now let's load them in, in the correct order to the detector
    indImg = 0
    indPnt = 0
    obDetect = ObstacleDetector((240,320))
    obDetect.normalConstraint = np.array([0.,-1.,0.])
    obDetect.angleConstraint = 30.
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

def main2():
    obDetect = ObstacleDetector((2,2))
    points = np.array([ [[0,0,3],[1,3,4]],
                        [[1,2,2],[np.nan,np.nan,np.nan]] ])
    
    obDetect.updatePoints(points)
    mask = np.array([[1,1],[0,1]], dtype=bool)
    print obDetect.get2DCloudFromMask( mask )

if __name__ == '__main__':
    main2()

