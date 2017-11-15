#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
#import AStarTemplate

Class AStar:


    def __init__(self):
        """
            This constructor sets up class variables and pubs/subs
        """
        self._testStart = Pose()
        self._testStart.position.x = 0
        self._testStart.position.y = 0
        self._testStart.orientation.x = 0
        self._testStart.orientation.y = 0
        self._testStart.orientation.z = 1

        rospy.Timer(rospy.Duration(.1), self.timerCallback)


    # reads in global map
    def mapCallBack(data):
        global mapData
        global width
        global height
        global mapgrid
        global resolution
        global offsetX
        global offsetY
        mapgrid = data
        resolution = data.info.resolution
        mapData = data.data
        width = data.info.width
        height = data.info.height
        offsetX = data.info.origin.position.x
        offsetY = data.info.origin.position.y
        print data.info

    def dummyPoseStamped(self, _x, _y, _theta):
        self._pose = PoseStamped()
        self._pose.position.x = _x
        self._pose.position.y = _y
        self._pose.orientation.x = 0
        self._pose.orientation.y = 0
        self._pose.orientation.z = 1
        self._pose.orientation.w = _theta
        return self._pose

    def dummyPose(self, _x, _y, _theta):
        self._pose = PoseStamped()
        self._pose.position.x = _x
        self._pose.position.y = _y
        self._pose.orientation.x = 0
        self._pose.orientation.y = 0
        self._pose.orientation.z = 1
        self._pose.orientation.w = _theta
        return self._pose

    def readGoal(goal):
        global goalX
        global goalY
        goalX= goal.pose.position.x
        goalY= goal.pose.position.y
        print goal.pose
        # Start Astar


    def readStart(startPos):

        global startPosX
        global startPosY
        startPosX = startPos.pose.pose.position.x
        startPosY = startPos.pose.pose.position.y
        print startPos.pose.pose

    def aStar(start,goal):
        pass
        # create a new instance of the map

        # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

        # for each node in the path, process the nodes to generate GridCells and Path messages

        # Publish points

    #publishes map to rviz using gridcells type

    def publishCells(grid):
        global pub
        print "publishing"

        # resolution and offset of the map
        k=0
        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = resolution
        cells.cell_height = resolution

        for i in range(1,512): #height should be set to hieght of grid
            k=k+1
            for j in range(1,480): #width should be set to width of grid
                k=k+1
                #print k # used for debugging
                if (grid[k] == 100):
                    point=Point()
                    point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset
                    point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                    point.z=0
                    cells.cells.append(point)
        pub.publish(cells)

    #Main handler of the project
    def run():
        global pub
        rospy.init_node('lab3')
        sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
        pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
        pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
        pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
        goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
        goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

        # wait a second for publisher, subscribers, and TF
        rospy.sleep(1)

        start = dummyPoseStamped(self, 0,0,0)

        while (1 and not rospy.is_shutdown()):
            publishCells(mapData) #publishing map data every 2 seconds
            rospy.sleep(2)
            print("Complete")



    if __name__ == '__main__':
        try:
            run()
        except rospy.ROSInterruptException:
            pass
