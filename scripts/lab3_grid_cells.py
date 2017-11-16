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



def __init():
     global mapDataOut
     global wallDataOut
     global checkedDataOut
     global hasStart
     global hasGoal
     hasStart = False
     hasGoal = False
     mapDataOut = GridCells()
     wallDataOut = GridCells()
     checkedDataOut = GridCells()

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
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    #print data.info

class MapDataToAStar:
    def __init__(self):
        self._offsetX = offsetX
        self._offsetY = offsetY
        self._resolution = resolution
        self._reshapedMap = reshapedMap



def gridFromPose(pose):
    gridx = int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    gridy = int((pose.position.y - offsetY - (.5 * resolution)) / resolution)

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

def publishWalls():
    data = GridCells()

    for i in range(0,height):
        for j in range(0,width):
            if getWall(i,j):

                data.cells.append(addScaledPoint(j,i))
                #print("Wrote cell at"+repr(i)+", "+repr(j))

    print("Wrote wall cells")
    publishCells(wallpub,data)


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
    publishCells(openpub, data)

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



#Main handler of the project
def run():


    global pub
    global wallpub
    global checkedpub
    global start_sub
    global goal_sub
    global openpub
    global pathpub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    wallpub = rospy.Publisher("/walls", GridCells, queue_size=1)
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    checkedpub = rospy.Publisher("/checked", GridCells, queue_size=1)
    openpub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    pathpub = rospy.Publisher("/pathtotravel", GridCells, queue_size=1)
    pubfrontier = rospy.Publisher("/frontier", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('/move_base_simple/goal2', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('/initialpose2', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results




    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    time=0
    initialize = True
    loopdone = False
    publishWalls()
    global lists

    while (not loopdone and not rospy.is_shutdown()):
         #publishing map data every 2 seconds

        if hasGoal and hasStart:
            if (initialize):
                lists = AStarTemplate.aStar(poseStart, poseGoal, MapDataToAStar())
                initialize=False
            else:
                lists = AStarTemplate.aStarLoop(lists['openset'],lists['closedset'],lists['closedpoints'],lists['finalpath'])
                if len(lists['finalpath'])>0:
                    loopdone=True
                    print ("Escaped the for loop")
            publishChecked(lists['closedset'])
            publishOpen(lists['openset'])
        #rospy.sleep(0.002)
    print ("Really escaped the for loop")
    emptySet = set()
    while (1 and not rospy.is_shutdown()):
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
