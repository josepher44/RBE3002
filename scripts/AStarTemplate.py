#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import tf
import heapq
import numpy
import math
import rospy, tf, numpy, math
from curses.ascii import NUL

""""
The A* search algorithm is an informed search algorithm of a known or explorable area, meaning that the program knows where the robot is, and where the goal is.  It also works with unexplored maps 
if the robot is equipped to explore and update the map in real time.  This algorithm works on any sort of graph that can be navigated, including directional graphs, but this lab uses a grid, so 
there are many ways to implement it.  Below is some modified pseudocode based on Wikipedia's article on A* (https://en.wikipedia.org/wiki/A*_search_algorithm) which I recommend reading to 
supplement course material.  The algorithm functions by expanding a frontier from the start node until it discovers the goal, in a method that could be described as a prioritized and incomplete 
depth-first search.  All nodes are assigned three values, the F, G, and H scores.  the G score of a node is the best navigable path that the search has found so far.  the H score is a heuristic 
that estimates the best possible distance between the node and the goal.  The F score is the sum of the G and H scores, and represents the best theoretical distance between the start node and the 
goal which travels through the current node.  By iterating the algorithm through the node on the frontier with the best F score, it will find the best possible path.

Authors: Wikipedia, Connor Flanigan
"""



def finalRollout (current_pose, final_path):
    for v, w in zip(final_path[:-1], final_path[1:]):
        if (checkNodeEquality(current_pose, v)):
            return angle_pose_to_path(current_pose, w)
        return NUL #TODO THIS SHOULD RERUN A* IF IT IS NOT ON THE ROUTE INSTEAD OF NUL
        



def newPose (parent, x, y):
    poseOut = Pose()
    poseOut.position.x = x
    poseOut.position.y = y
    poseOut.position.z = 0
    poseOut.orientation = tf.transformations.quaternion_from_euler(0, 0, math.atan2((y-parent.y),(x-parent.x)))
    return poseOut

class xy(object):
    def __init__(self, x, y):
        self.x=x
        self.y=y

class node(object):
    def __init__(self, x, y, isOccupied, g, Parent):
        self.x = x
        self.y = y
        self.parent = Parent

        if self.parent is not None:
            self.pose = newPose(self.parent, x, y)
            self.isStart = False
            self.g = g + Parent.g
            print ("parent is not null")
        else:
            self.pose = startPose
            self.isStart = True
            self.g = g
        self.h = heuristic_cost_estimate(self.pose, goalPose)

        self.f = self.g + self.h

        self.isOccupied = isOccupied
        self.index = nodeIndex
        self.position = xy(x, y)


    def __cmp__(self, other):
        return cmp(self.f, other.f)




#things to subscribe to: /move_base_simple/goal, /geometry_msgs/Pose



def convertMapPoseToGridPose(pose):
    outPose = Pose()
    outPose.position.x=int((pose.position.x - offsetX - (.5 * resolution)) / resolution)
    outPose.position.y=int((pose.position.y - offsetY - (.5 * resolution)) / resolution)

    q_map = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]

    (roll_map, pitch_map, yaw_map) = euler_from_quaternion(q_map)

    #round arbitrary input angle to nearest 45 degrees
    yaw_new = int(math.pi/4 * round(float(yaw_map)/(math.pi/4)))
    outPose.orientation = tf.transformations.quaternion_from_euler(roll_map, pitch_map, yaw_new)

    return outPose


def getWall(x,y):
    if reshapedMap[x,y]<50:
        return False
    else:
        return True

def startNodeFromPose(pose):
    global startPose
    startPose = convertMapPoseToGridPose(pose)
    print(startPose.position.x)
    print(startPose.position.y)
    startNode = node(startPose.position.x, startPose.position.y, False, 0, None)
    return startNode


#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def aStar(start,goal,mapData):
    global nodeIndex
    nodeIndex = 0
    global offsetX
    global offsetY
    global resolution
    global goalPose
    offsetX = mapData._offsetX
    offsetY = mapData._offsetY
    resolution = mapData._resolution
    global reshapedMap
    reshapedMap = mapData._reshapedMap
    goalPose = convertMapPoseToGridPose(goal)
    global kDistance
    global kTurn

    kDistance = .075
    kTurn = 0

    closedset = set()    # The set of nodes already evaluated.
    closedpoints = set()
    finalpath = set()
    global opensetPoints
    global openset
    openset = []  #this is a heapq
    heapq.heapify(openset)
    #heapq.heappush = (openset, (distance_calculation(start, goal), start))  # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed
       # set and all other nodes.

    openset.append(startNodeFromPose(start))

    came_from = []    # The map of navigated nodes. TODO

    while (len(openset) != 0):    # while there are still nodes that have not been checked, continually run the algorithm

        current = heapq.heappop(openset) # this is the most promising node of all nodes in the open set
        print ("current node index is ")
        print(current.index)
        print ("current heap size is ")
        print(len(openset))
        if (current.x==goalPose.position.x and current.y==goalPose.position.y):                                               # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            return reconstruct_path(current)

        #remove current from openset                  # mark this node as having been evaluated
        #add current to closedset
        closedset.add(current)
        closedpoints.add(current.position)
        neighborSet = neighbor_nodes(current)
        for neighbor in neighborSet: # re-evaluate each neighboring node

            if (checkIfListContainsPoint(closedpoints, neighbor) and neighbor not in openset and checkExistingNodes(neighbor)):
                heapq.heappush(openset,neighbor) #add neighbor to open set

        print ("current heap size after adds is ")
        print(len(openset))
        for node in openset:
            opensetPoints = GridCells()
            opensetPoints.cells.append(addScaledPoint(node.x, node.y))
        for node in closedset:
            closedsetPoints = GridCells()
            closedsetPoints.cells.append(addScaledPoint(node.x, node.y))
        return {'closedset':closedset, 'closedpoints':closedpoints, 'openset':openset, 'finalpath':finalpath}
    print("failed")
    #return failure #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

def aStarLoop(open_set, closed_set, closed_points, final_path):
    while (len(open_set) != 0):    # while there are still nodes that have not been checked, continually run the algorithm

        current = heapq.heappop(open_set) # this is the most promising node of all nodes in the open set
        #print ("current node index is ")
        #print(current.index)
        #print ("current heap size is ")
        #print(len(open_set))
        print ("current f is ")
        print(current.f)
        print ("current g is ")
        print(current.g)
        if (current.x==goalPose.position.x and current.y==goalPose.position.y):                                               # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            final_path = reconstruct_path(current)
            return {'closedset': closed_set, 'closedpoints': closed_points, 'openset': openset, 'finalpath': final_path}
            print("Found path! Go get a signoff now!")

        #remove current from openset                  # mark this node as having been evaluated
        #add current to closedset
        closed_set.add(current)
        closed_points.add(current.position)
        neighborSet = neighbor_nodes(current)
        for neighbor in neighborSet: # re-evaluate each neighboring node

            if (checkIfListContainsPoint(closed_points, neighbor) and neighbor not in openset and checkExistingNodes(neighbor)):
                heapq.heappush(openset,neighbor) #add neighbor to open set

        #print ("current heap size after adds is ")
        #print(len(open_set))
        for node in open_set:
            opensetPoints = GridCells()
            opensetPoints.cells.append(addScaledPoint(node.x, node.y))
        for node in closed_set:
            closedsetPoints = GridCells()
            closedsetPoints.cells.append(addScaledPoint(node.x, node.y))
        return {'closedset':closed_set, 'closedpoints':closed_points, 'openset':openset, 'finalpath':final_path}

def addScaledPoint(x,y):
    pointToAdd = Point()
    pointToAdd.x = (x * resolution) + offsetX + (.5 * resolution)
    pointToAdd.y = (y * resolution) + offsetY + (.5 * resolution)
    pointToAdd.z = 0
    return pointToAdd

def checkIfListContainsPoint(list, point):
    for entry in list:
        if entry.x==point.x and entry.y==point.y:
            return False
    return True




# Starting from the goal, work backwards to find the start.  We recommend returning a path nav_msgs, which is an array of PoseStamped with a header
def reconstruct_path(node):

    # start by adding goal to the path
    total_path = set()
    current = node

    print ("started reconstruction")
    # run while reconstruct_path hasn't reached the start
    while current.parent is not None:
        print ("ran loop for reconstruction")
        # The current node is now the node that leads to the previous node
        current = current.parent

        # add the current node to the front of the list
        total_path.add(current)

    # The list is now the shortest path from the start to the end
    return total_path

def heuristic_cost_estimate(startpose, goalpose):

    return kDistance*distance_calculation(startpose, goalpose) + kTurn*(angle_pose_to_path(startpose, goalpose) + kTurn*(angle_path_to_pose(startpose, goalpose)))
    #if there were no obstacles in the way of the robot, what is the shortest path to the goal?  Return that value



def distance_calculation(startpose, goalpose):
    startx = startpose.position.x
    starty = startpose.position.y
    endx = goalpose.position.x
    endy = goalpose.position.y
    deltax = endx-startx
    deltay = endy-starty
    return math.sqrt(deltax**2 + deltay**2)

def angle_pose_to_path(startpose, goalpose):
    #Calculates the angle from a start pose, to the path between the start pose and the goal pose
    startQuat = startpose.orientation
    (rollStart, pitchStart, yawStart) = euler_from_quaternion(startQuat)
    startx = startpose.position.x
    starty = startpose.position.y
    endx = goalpose.position.x
    endy = goalpose.position.y
    deltax = endx-startx
    deltay = endy-starty

    travelAngle = math.atan2(deltay, deltax)

    return travelAngle-yawStart

def angle_path_to_pose(startpose, goalpose):
    #Calculates the angle from a path of travel between two poses, and the desired angle at the end pose
    endQuat = goalpose.orientation
    (rollEnd, pitchEnd, yawEnd) = euler_from_quaternion(endQuat)
    startx = startpose.position.x
    starty = startpose.position.y
    endx = goalpose.position.x
    endy = goalpose.position.y
    deltax = endx-startx
    deltay = endy-starty

    travelAngle = math.atan2(deltay, deltax)

    return travelAngle-yawEnd


def neighbor_nodes(currentNode):
    global nodeIndex
    print(currentNode.x)
    print(currentNode.y)
    adjacent = []
    for i in range(-1, 2):
        for j in range (-1, 2):
            if not (getWall(currentNode.y+j, currentNode.x+i)or(i==0 and j==0)):
                adjacent.append(node(currentNode.x+i, currentNode.y+j, False, gValueFunction(currentNode, i, j), currentNode))
                nodeIndex = nodeIndex+1


    return adjacent

def checkExistingNodes(node1):
    i=0
    for node2 in openset:
        if checkNodeEquality(node1, node2):
            if (node2.g<=node1.g):
                return False
            #else:
                #print("it actually hit the part it shouldn't hit")
                #openset[i] = openset[-1]
                #nodeOut = openset.pop()
                #print("Node has values of x=")
                #print (nodeOut.x)
                #print (", y=")
                #print (nodeOut.y)
                #heapq.heapify(openset)

    i=i+1
    return True
#Returns true only if the entered node does not share a space with an existing frontier node, and removes a node from the heap if it is reached more efficiently than by previous method


def checkNodeEquality(node1, node2):
    if node1.x == node2.x and node1.y == node2.y:
        return True
    else:
        return False

    #Returns true if two nodes share the same x, y position, false if otherwise



def gValueFunction(currentNode, i, j):
    if (currentNode.isStart):
        return kDistance*math.sqrt(i**2+j**2)
    else:
        return kTurn*(math.atan2(currentNode.y-currentNode.parent.y, currentNode.x-currentNode.parent.x)-math.atan2(j, i)) + kDistance*math.sqrt(i**2+j**2)
    #finds difference between next heading and current heading, multiplies by turn constant, adds to distance difficulty and returns


def dist_between(current,neighbor):

    return math.sqrt((neighbor.pose.position.x - current.pose.position.x)**2 + (neighbor.pose.position.y - current.pose.position.y)**2)
    #TODO the distance necessary to travel to the neighbor from the current node

def addAngleToAngle(start_angle, added_angle):
    out = start_angle+added_angle
    if out>2*math.pi:
        out=out-2*math.pi
    elif out<-2*math.pi:
        out=out+2*math.pi
    return out


def distanceBetweenAngles(angle1, angle2):
    if angle1 == angle2:
        return 0
    elif angle1>angle2:
        split = angle2-angle1
        if split<-math.pi:
            split =2*math.pi+split
    else:
        split = angle2-angle1
        if split>math.pi:
            split=-2*math.pi+split
    return split
    #Compares two angles and returns the shortest (always a<pi) angle between them, assuming that angles go from -pi to pi. The output is directional such that

"""

some advice:

    A) don't be like me and make this monstrosity

"""
