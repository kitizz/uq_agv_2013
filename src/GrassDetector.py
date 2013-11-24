#!/usr/bin/env python

import numpy as np
import cv2

class GrassDetector:

    def __init__(self, hThresh, sThresh, vThresh):
        self.image = np.array([])
        self.mask = np.array([])
        self.thresh = np.array([hThresh, sThresh, vThresh])
        self.grassMask = np.array([], dtype=bool)

    def updateImage(self, image):
        self.image = image

    def updateMask(self, mask):
        self.mask = mask

    def findGrass(self):
        imBlur = cv2.GaussianBlur(self.image.copy(), (21, 21), 0)

        #cv2.imshow('Image', imBlur)
        # cv2.imshow('Mask', self.mask.astype(float)*255)

        imgHsv = cv2.cvtColor(imBlur, cv2.COLOR_BGR2HSV)
        hsv = imgHsv[self.mask]
        meanHsv = np.mean(hsv, axis=0)

        # meanRgb = cv2.cvtColor(np.reshape(meanHsv, (1,1,3)), cv2.COLOR_HSV2BGR)
        # cv2.imshow('Color', np.tile(meanRgb, (100,100,1)))
        # cv2.waitKey(0)

        # Select the pixels in the image that are close to this colour
        deltas = np.abs(imgHsv.astype(float) - meanHsv)
        self.grassMask = np.logical_or(self.mask, (deltas < self.thresh).all(axis=2))

        return self.grassMask

        # im = self.image.copy()
        # im[similar] = [255,255,255]

        # cv2.imshow('Segmented', im)
        # cv2.waitKey(0)


def main():
    im = cv2.imread('GrassTest.jpg')
    mask = cv2.imread('GrassTest_Plane.jpg')

    mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)>128

    gDetect = GrassDetector(45,30,35)
    gDetect.updateImage(im)
    gDetect.updateMask(mask)

    gDetect.findGrass()


if __name__ == '__main__':
    main()
