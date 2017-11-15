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
     mapDataOut = GridCells()
     wallDataOut = GridCells()


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
    print(mapData)
    print(len(mapData))
    width = data.info.width
    height = data.info.height
    print(width)
    print(height)
    a = np.array(data.data)
    reshapedMap = np.reshape(a, (width,height))
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def gridFromPose(pose):
    gridx = int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    gridy = int((pose.position.y - offsetY - (.5 * resolution)) / resolution)



def readGoal(goal):
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
    return poseGoal
def readStart(startPos):
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


    wallDataOut.header.frame_id = 'map'
    wallDataOut.cell_width = resolution
    wallDataOut.cell_height = resolution

    for i in range(0,width):
        for j in range(0,height):
            if getWall(i,j):

                wallDataOut.cells.append(addScaledPoint(i,j))
                print("Wrote cell at"+repr(i)+", "+repr(j))

    wallpub.publish(wallDataOut)

def addScaledPoint(x,y):
    pointToAdd = Point()
    pointToAdd.x = (x * resolution) + offsetX + (.5 * resolution)
    pointToAdd.y = (y * resolution) + offsetY + (.5 * resolution)
    pointToAdd.z = 0
    return pointToAdd


def publishCells(grid, publisher):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    mapDataOut.header.frame_id = 'map'
    mapDataOut.cell_width = resolution
    mapDataOut.cell_height = resolution
    pointToAdd = Point()

    pointToAdd.x = (1 * resolution) + offsetX + (.5 * resolution)
    pointToAdd.y = (1 * resolution) + offsetY + (.5 * resolution)

    pointToAdd.z = 0
    mapDataOut.cells.append(pointToAdd)
    print("Wrote cell at"+repr(1)+", "+repr(1))
    publisher.publish(mapDataOut)



#Main handler of the project
def run():
    global pub
    global wallpub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    wallpub = rospy.Publisher("/walls", GridCells, queue_size=1)
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    pubchecked = rospy.Publisher("/checked", GridCells, queue_size=1)
    pubfrontier = rospy.Publisher("/frontier", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    
   


    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    time=0


    while (1 and not rospy.is_shutdown()):
        publishWalls() #publishing map data every 2 seconds

        AStarTemplate.aStar(poseStart, poseGoal)  #start then goal
        rospy.sleep(1)
        print("Complete")
        time=time+1
    


if __name__ == '__main__':
    try:
        __init()
        run()
    except rospy.ROSInterruptException:
        pass
