import cv2
import math
from numpy import *
import matplotlib.pyplot as plt
import genObjects
import loader
import obstacle
import map
from robotSim import Robot
from imageReader import ImageReader
from point import point
import mapper
import ConfigParser
import path
import astar
#import dstar
#import dstar2
#import dstar3
import dlite
from vehicle import *
from PIL import Image
import random

goal = array([10,0.1])
sig = 2
eta = 30
import numpy as np
from scipy.misc import comb

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals

def column(matrix, i):
    return [row[i] for row in matrix]

if __name__ == "__main__":

    #Initialise Vehicle Class
    TEG =  Vehicle(0,0,2)
    #Initialise blank image
    height = 500
    width = 500  
    #Set the current map class to blank
    agvcmap = map.map(height, width, 255)
    #Generate a list of objects
    genObjects.main(agvcmap)
    #Retrieve objects from file
    obsList = loader.getObs()

    #M Mapping of obstacles and lines onto a pixel image and skeleton planning returning GPS (or distance) waypoints 
        
    #Place obstacles on map
    agvcmap.updateObstacles(obsList)

    #Morph the obstacles
    agvcmap.updateMorph();

    #Plan routes through obstacles
    #Skeleton Method---------------------------------------
    #cv2.imwrite('AGVCmap2.jpg', agvcmap.getImage())
    #img = cv2.imread('AGVCmap2.jpg',0)
    #size = size(img)
    #skel = zeros(img.shape,uint8)
 
    #ret,img = cv2.threshold(img,127,255,0)
    #element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    #done = False
 
    #while( not done):
    #    eroded = cv2.erode(img,element)
    #    temp = cv2.dilate(eroded,element)
    #    temp = cv2.subtract(img,temp)
    #    skel = cv2.bitwise_or(skel,temp)
    #    img = eroded.copy()
 
         
    #    zeros = size - cv2.countNonZero(img)
    #    if zeros==size:
    #        done = True
 
    #cv2.imshow("skel",skel)
    #D* Lite Method ---------------------------------------------
    newHeight = int(height*0.1)
    newWidth = int(width*0.1)
    dliteimage = cv2.resize(agvcmap.getImage(),(newWidth,newHeight))
    cv2.imwrite('AGVCmap2.bmp', dliteimage)
    robot = Robot(TEG.x,TEG.y,TEG.radius*2)
    imageMap = ImageReader()
    imageMap.loadFile("AGVCmap2.bmp")
    mapper.initalize(imageMap,robot)
    moveGrid = imageMap.convertToGrid().copy()

    ##goal = point(3,17)
    testdivider = 1
    goal = point(int(newHeight/testdivider*0.8),int(newWidth/testdivider*0.8))
    #cv2.waitKey(0)

    ##mapper.printMoveGrid()

    print "STARTIN LOOP"
    moveId=0
    Xlength = mapper.grid.shape[0]/testdivider
    Ylength = mapper.grid.shape[1]/testdivider
    #dstar = dstar3.DStar(Xlength,Ylength,goal)
    dstar = dlite.Dlite(Xlength,Ylength,goal,robot)
    print "Entering Loop"
    testvar = 0

    while (robot.y != goal.y or robot.x != goal.x) :
        if testvar%10 == 0:
            newObs = obstacle.Obstacle(random.randint(0,height),random.randint(0,width), 10)
            agvcmap.placeObstacle(newObs,3)
            obsList.append(newObs)
            #Place obstacles on map
            agvcmap.updateObstacles(obsList)

            #Morph the obstacles
            agvcmap.updateMorph();
            dliteimage = cv2.resize(agvcmap.getImage(),(newWidth,newHeight))
            cv2.imwrite('AGVCmap2.bmp', dliteimage)
            imageMap.loadFile("AGVCmap2.bmp")
            mapper.initalize(imageMap,robot)
            moveGrid = imageMap.convertToGrid().copy()
            
        testvar = testvar + 1

        moveId = moveId+1
        print moveId
        if path.pathIsBroken(mapper.grid) :
            path.restart()
            print "The path is broken"
            #  dstar2.dstar(mapper, robot, goal, path)

            dstar.dstar(mapper, robot, goal, path)
     
        #dlite.dstar(robot,goal,path)
         #  #  DstarLite.dstar(mapper, robot, goal, path)
          #  astar.astar(mapper, robot, goal, path)
        pathNode=path.getNextMove()
        robot.x = pathNode.x
        robot.y = pathNode.y
        mapper.moveGrid[pathNode.x][pathNode.y]="1"
        #mapper.printMoveGrid()
        TEG.x = pathNode.x
        TEG.y = pathNode.y
        TEG.addPosition(pathNode.x,pathNode.y)
        mapper.updateMap(robot)
        #raw_input("TEST")
        cv2.imshow('AGVC Map', agvcmap.getMap())
        cv2.imshow('AGVC Map Morphed', agvcmap.getImage())
        for i in range(len(TEG.positions)):
            agvcmap.placeRobot(TEG.positions[i][1]*height/newHeight,TEG.positions[i][0]*width/newWidth,TEG.radius)
        nPoints = len(TEG.positions)
        points = np.array(TEG.positions)*height/newHeight
        #points = np.random.rand(nPoints,2)*200
        xpoints = [p[0] for p in points]
        ypoints = [p[1] for p in points]

        xvals, yvals = bezier_curve(points, nTimes=1000)
        for i in range(len(xvals)):
            agvcmap.placeRobot(int(yvals[i]),int(xvals[i]),TEG.radius*2)

        agvcmap.updateActObstacles(obsList)


        cv2.waitKey(0)


    #mapper.printMoveGrid()


    for i in range(len(TEG.positions)):
        agvcmap.placeRobot(TEG.positions[i][1]*height/newHeight,TEG.positions[i][0]*width/newWidth,TEG.radius)

    voronoiimage = agvcmap.getMap()
    
    for y in range(height):
        for x in range(width):
            dmin = math.hypot(width-1, height-1)
            j = -1
            for i in range(len(obsList)):
                d = math.hypot(obsList[i].x-x, obsList[i].y-y)
                if d < dmin:
                    dmin = d
                    j = i
            voronoiimage[y,x] = (255/abs(j+1),255/abs(j+1),255/abs(j+1))

    agvcmap.updateActObstacles(obsList)
    #Display the image
    cv2.imshow('AGVC Map', agvcmap.getMap())
    cv2.imshow('AGVC Map Morphed', agvcmap.getImage())
    cv2.imshow('AGVC Map Resize', dliteimage)
    cv2.imwrite('AGVCpath.bmp', agvcmap.getImage())

    #Save figure for use    
    #fig.savefig('AGVCmap.jpg')

    print "Done"
    #astar.main(agvcmap)
    #print 'Finished astar'
    cv2.waitKey(0)


    ##Potential Fields Method
    #for i in range(100):
    #    #Plot the obstacle list
    #    plt.plot(obsList[1].x,obsList[0].y,'o', markersize = 20)
    #    plt.plot(obsList[2].x,obsList[2].y,'o', markersize = 20)
    #    plt.plot(obsList[3].x,obsList[3].y,'o', markersize = 20)
    #    #Plot the current position
    #    plt.plot(TEG.getPosition()[0],TEG.getPosition()[1],'+', markersize = 20)
    #    #plot goal position
    #    plt.plot(goal[0],goal[1],'s', markersize = 20)
    #    #Work out the repulsive forces
    #    Fatt = -sig*(TEG.getPosition() - goal)
    #    temp = (TEG.getPosition()-array([obsList[0].x,obsList[0].y]))
    #    Frep1 = eta*temp/(sqrt(temp[0]**2 + temp[1]**2))**3
    #    temp = (TEG.getPosition()-array([obsList[3].x,obsList[3].y]))
    #    Frep2 = eta*temp/(sqrt(temp[0]**2 + temp[1]**2))**3
    #    temp = (TEG.getPosition()-array([obsList[2].x,obsList[2].y]))
    #    Frep3 = eta*temp/(sqrt(temp[0]**2 + temp[1]**2))**3
    #    #print Fatt,Frep
    #    Fres = Fatt+Frep1+Frep2+Frep3
    #    #print Fres
    #    #What to set the position to
    #    print 'Currpos' + str(TEG.getPosition()[0]) + "," + str(TEG.getPosition()[1])
    #    TEG.setPosition(TEG.getPosition()[0] + 0.05*Fres[0], TEG.getPosition()[1] + 0.05*Fres[1])
