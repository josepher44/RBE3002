#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy as np
import math
import rospy, tf, math
import AStarTemplate
import copy



def __init():
     global mapDataOut
     global wallDataOut
     global checkedDataOut
     global openDataOut
     global hasStart
     global hasGoal
     global allFrontiers
     global allCentroids
     allFrontiers = list()
     allCentroids = list()
     hasStart = False
     hasGoal = False
     mapDataOut = GridCells()
     wallDataOut = GridCells()
     checkedDataOut = GridCells()
     global odom_tf
     global odom_list
     odom_list = tf.TransformListener()



def getpose():
    global odom_list
    pose = Pose()
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    return position.pose.pose

# reads in global map
def mapCallBack(data):
    print("Map callback executed")
    global mapData
    global reshapedMap
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    global recalcFlag
    
    
    recalcFlag = True
    mapgrid = data

    resolution = data.info.resolution
    mapData = data.data
    #print(mapData)
    print(len(mapData))
    width = data.info.width
    height = data.info.height
    print(width)
    print(height)
    a = np.array(data.data)
    reshapedMap = np.reshape(a, (height, width))
    print(mapData)
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    #print data.info

class MapDataToAStar:
    def __init__(self):
        self._offsetX = offsetX
        self._offsetY = offsetY
        self._resolution = resolution
        self._reshapedMap = reshapedMap

class xy(object):
    def __init__(self, X, Y):
        self.x = X
        self.y = Y

def gridFromPose(pose):
    gridx = int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    gridy = int((pose.position.y - offsetY - (.5 * resolution)) / resolution)
    return xy(gridx,gridy)

def gridFromXY(x, y):
    gridx = int((x - offsetX - (.5 * resolution)) / resolution)
    gridy = int((y - offsetY - (.5 * resolution)) / resolution)
    return xy(gridx,gridy)

def readGoal(goal):
    global hasGoal
    print("read goal")
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    # Start Astar
    global poseGoal
    poseGoal = Pose()
    poseGoal.position.x = goalX
    poseGoal.position.y = goalY
    poseGoal.position.z = 0
    poseGoal.orientation = goal.pose.orientation
    hasGoal=True
    return poseGoal

def readStart(startPos):
    global hasStart
    print("read start")
    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose
    global poseStart
    poseStart = Pose()
    poseStart.position.x = startPosX
    poseStart.position.y = startPosY
    poseStart.position.z = 0
    poseStart.orientation = startPos.pose.pose.orientation
    hasStart=True
    return poseStart

def aStar(start,goal):
    pass
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points

#publishes map to rviz using gridcells type

def getWall(x,y):
    if reshapedMap[x,y]<50:
        return False
    else:
        return True

def getUnknown(x,y):
    if reshapedMap[x,y]>-1:
        return False
    else:
        return True

def getFree(x,y):
    if reshapedMap[x,y]>-0.5 and reshapedMap[x,y]<1:
        #print("Freedom!")
        return True
    else:
        #print("Communists!")
        return False

def getFrontier(x,y):
    if getUnknown(x,y):
        for i in range(x-1, x+2):
            for k in range(y-1, y+2):
                if getFree(i,k):
                    #reshapedMap[x, y]=20
                    return True

    return False

def getFrontierMap(x,y):
    if reshapedMap[x,y]>10 and reshapedMap[x,y]<30:
        return True
    else:
        return False



def getFrontierTermination(x, y):
    for i in range(x - 1, x + 2):
        for k in range(y - 1, y + 2):
            if (getWall(i,k)):
                return True

    return False


def getTrueWall(x,y):
    if reshapedMap[x,y]<99:
        return False
    else:
        return True

def publishNavFrontier():
    global reshapedMap
    data = GridCells()
    executioncount = 0
    frontiercount = 0
    for i in range(0,height-1):
        for k in range(0,width-1):
            #print("Tried a frontier at")
            #print(i)
            #print(k)
            executioncount = executioncount+1
            #print(executioncount)
            if getFrontier(i,k):
                data.cells.append(addScaledPoint(k,i))
                reshapedMap[i, k] = 20
                print(reshapedMap[i,k])
                allFrontiers.append(xy(i,k))
                print("Found a frontier")
                frontiercount = frontiercount+1
                print(frontiercount)
                print(allFrontiers.__len__())
    publishCells(navfrontierpub,data)
    print("Published frontiers")
    print("Total frontier count is")
    print(frontiercount)
    print allFrontiers.__len__()

def elementInSet(set, x,y):
    for ele in set:
        if ele.x is x and ele.y is y:
            return True
    return False

def generateSubFrontiers():
    countbymap = 0
    for e in range (0,height-1):
        for f in range(0,width-1):
            if (getFrontierMap(e,f)):
                print("found a frontier via mapping")
                countbymap = countbymap+1
                print(countbymap)



    workingSet = copy.deepcopy(allFrontiers)
    deepflag = False
    #for element in workingSet:
        #print("have a frontier at")
        #print (element.x)
        #print (element.y)
        #print ("working set size is")
        #print (workingSet.__len__())
    allFrontierSegments = list()

    while workingSet.__len__()>0:
        startLocation = workingSet.pop()

        frontier_in_yo_frontier = list()
        frontier_in_yo_frontier.append(startLocation)
        subFrontier = list()
        while frontier_in_yo_frontier.__len__()>0:
            print("frontiers in yo frontier: ")
            print(frontier_in_yo_frontier.__len__())
            print("running the frontier in yo frontier while loop")
            for element in frontier_in_yo_frontier:
                subFrontier.append(element)
                for element2 in workingSet:
                    if element2.x is element.x and element2.y is element.y:
                        workingSet.remove(element2)
                        print("removed an element via the element2 loop")
                frontier_in_yo_frontier.remove(element)
                print("removed element from frontier in yo frontier")
                #print(element.x)
                #print(element.y)

                for i in range(-1, 2):
                    for j in range(-1, 2):
                        print(element.x+i)
                        print(element.y+j)
                        if getFrontierMap(element.x+i,element.y+j) is True:
                            frontier_in_yo_frontier.append(xy(element.x + i, element.y + j))
                            reshapedMap[element.x+i, element.y+j] = 5

                            #if element3.x == element.x+i and element3.y == element.y+j:


                                #print("added element")
                                #print(element.x+i)
                                #print(element.y+j)
                #print("frontiers in yo frontier: ")
                #print(frontier_in_yo_frontier.__len__()))

        allFrontierSegments.append(copy.deepcopy(subFrontier))
        print("Added a sub frontier of size")
        print(subFrontier.__len__())
    print("finished adding frontiers, frontier count is")
    print(allFrontierSegments.__len__())
    return allFrontierSegments






def expandWalls():
    appended = copy.deepcopy(reshapedMap)
    for i in range(0,height):
        for j in range(0,width):
            if getTrueWall(i,j):
                for k in range(-3,4):
                    for m in range(-3,4):

                        if(j+k>=0 and j+k<=width-1 and i+m>=0 and i+m<=height-1):
                            if reshapedMap[i+m,j+k]<99:
                                reshapedMap[i+m, j+k] = 80
                                appended[i+m, j+k] = 80

def publishCentroids():
    data = GridCells()
    listOfFrontiers = generateSubFrontiers()
    print("publishing centroids now")
    print(listOfFrontiers.__len__())
    for element in listOfFrontiers:
        xavg=0
        yavg=0
        for node in element:
            xavg = xavg+node.x
            yavg = yavg+node.y
        xavg=xavg/element.__len__()
        yavg=yavg/element.__len__()
        #print(xavg)
        #print(yavg)
        data.cells.append(addScaledPoint(yavg, xavg))
        allCentroids.append(xy(xavg,yavg))
    publishCells(frontiercentroidpub, data)

def publishWalls():
    data = GridCells()
    appended = copy.deepcopy(reshapedMap)


    for i in range(0,height):
        for j in range(0,width):
            if getTrueWall(i,j):
                data.cells.append(addScaledPoint(j, i))
    print("Wrote wall cells")
    publishCells(wallpub,data)

def publishTarget():
    data = GridCells()
    x= gridFromXY(getx()).x
    y= gridFromXY(gety()).y
    point = minimumFrontier(150,280)
    for i in range(point.x-1,point.x+2):
        for j in range(point.y-1,point.y+2):
            data.cells.append(addScaledPoint(j,i))
    publishCells(targetcentroidpub, data)

def publishExpandedWalls():
    data = GridCells()
    for i in range(0,height):
        for j in range(0,width):
            if getWall(i,j) and not getTrueWall(i,j):
                data.cells.append(addScaledPoint(j, i))
    publishCells(exwallpub, data)


def publishChecked(checkedCells):
    data = GridCells()

    for node in checkedCells:
        data.cells.append(addScaledPoint(node.x, node.y))
    publishCells(checkedpub, data)

def publishOpen(openCells):
    data = GridCells()

    for node in openCells:
        data.cells.append(addScaledPoint(node.x, node.y))
    publishCells(openpub, data)

def publishPath(pathCells):
    data = GridCells()

    for node in pathCells:
        data.cells.append(addScaledPoint(node.x, node.y))
    publishCells(pathpub, data)

def addScaledPoint(x,y):
    pointToAdd = Point()
    pointToAdd.x = (x * resolution) + offsetX + (.5 * resolution)
    pointToAdd.y = (y * resolution) + offsetY + (.5 * resolution)
    pointToAdd.z = 0
    return pointToAdd


def publishCells(publisher, data):
    data.header.frame_id = 'map'
    data.cell_width = resolution
    data.cell_height = resolution
    publisher.publish(data)

def distanceBetweenPoints(x1, y1, x2, y2):
    return (math.sqrt((x1-x2)**2 + (y1-y2)**2))

def minimumFrontier(xcomp, ycomp):
    minimum = 999999999

    for point in allCentroids:
        if distanceBetweenPoints(xcomp, ycomp, point.x, point.y) <minimum:
            minimum = distanceBetweenPoints(xcomp, ycomp, point.x, point.y)
            pointout = copy.deepcopy(point)
    return pointout




#Main handler of the project
def run():


    global pub
    global wallpub
    global exwallpub
    global checkedpub
    global start_sub
    global goal_sub
    global openpub
    global pathpub
    global navfrontierpub
    global frontiercentroidpub
    global targetcentroidpub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    wallpub = rospy.Publisher("/walls", GridCells, queue_size=1)
    exwallpub = rospy.Publisher("/exwalls", GridCells, queue_size=1)
    navfrontierpub = rospy.Publisher("/navfrontier", GridCells, queue_size=1)
    frontiercentroidpub = rospy.Publisher("/frontiercentroid", GridCells, queue_size=1)
    targetcentroidpub = rospy.Publisher("/targetcentroid", GridCells, queue_size=1)
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    checkedpub = rospy.Publisher("/checked", GridCells, queue_size=1)
    openpub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    pathpub = rospy.Publisher("/pathtotravel", GridCells, queue_size=1)
    pubfrontier = rospy.Publisher("/frontier", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results




    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    time=0
    initialize = True
    loopdone = False
    expandWalls()
    #publishExpandedWalls()
    publishWalls()
    publishNavFrontier()
    rospy.sleep(1)
    publishCentroids()
    publishTarget()
    global lists

    while (1 and not rospy.is_shutdown()):
        if (recalcFlag == True):
            initialize = True
            loopdone = False
            while (not loopdone and not rospy.is_shutdown()):
                 #publishing map data every 2 seconds
        
                if hasGoal and hasStart:
                    if (initialize):
                        lists = AStarTemplate.aStar(poseStart, poseGoal, MapDataToAStar())
                        initialize=False
                    else:
                        lists = AStarTemplate.aStarLoop(lists['openset'],lists['closedset'],lists['closedpoints'],lists['closedarray'],lists['openarray'],lists['finalpath'])
                        if len(lists['finalpath'])>0:
                            loopdone=True
                            print ("Escaped the for loop")
                    publishChecked(lists['closedset'])
                    publishOpen(lists['openset'])
                #rospy.sleep(0.002)
        print ("Really escaped the for loop")
        emptySet = set()
    
        publishChecked(emptySet)
        publishOpen(emptySet)
        publishPath(lists['finalpath'])
        rospy.sleep(1)



if __name__ == '__main__':
    try:
        __init()
        run()
    except rospy.ROSInterruptException:
        pass
