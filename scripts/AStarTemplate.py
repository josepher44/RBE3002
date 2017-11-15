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

def newPose (parent, x, y)
	poseOut = Pose()
	poseOut.position.x = x
	poseOut.position.y = y
	poseout.position.z = 0
	poseOut.orientation.x = 0
	poseOut.orientation.y = 0
	poseOut.orientation.z = 1
	poseOut.orientation.w = math.atan2((y-parent.y)/(x-parent.x))

class node(object):
    def __init__(self, x, y, isOccupied, g, h, parent):
        self.priority = priority
        self.description = description
        self.x = x
        self.y = y
        self.parent = None
	self.h = heuristicFunction(self)
	self.g = previousG + 1
        self.f = self.G + self.H)
        self.isOccupied = isOccupied
	self.parent = parent
	self.pose = newPose(parent,x,y)
        
        return
    def __cmp__(self, other):
        return cmp(self.priority, other.priority)



def __init__(self):
    kDistance = .75
    kTurn = .25

def getWall(x,y)
	location = GridCells.width*y+x
	if OccupencyGrid.data[location]<50
	return False
	else
	return True
	


#it's recommended that start is a poseStamped msg and goal is a pose msg, RViz likes using that for visualization.
def aStar(start,goal):
    


    closedset = set()    # The set of nodes already evaluated.
    openset = []
	heapq.heapify(openset)
    heapq.heappush = (heap, (distance_calculation(start, goal), start)  # The set of tentative nodes to be evaluated, initially containing the start node.  The nodes in this set are the nodes that make the frontier between the closed
       # set and all other nodes.
    came_from = the empty map    # The map of navigated nodes. TODO

	# The g_score of a node is the distance of the shortest path from the start to the node.
	# Start by assuming that all nodes that have yet to be processed cannot be reached
    g_score = math.inf

	# The starting node has zero distance from start
    g_score[start] = 0

    # The f_score of a node is the estimated total cost from start to goal to the goal.  This is the sum of the g_score (shortest known path) and the h_score (best possible path).
    # assume same as g_score
	f_score = math.inf

	# heuristic_cost_estimate(a, b) is the shortest possible path between a and b, this can be euclidean, octodirectional, Manhattan, or something fancy based on how the machine moves
	# the best possible distance between the start and the goal will be the heuristic
    f_score[start] = g_score[start] + heuristic_cost_estimate(start, goal)


    while openset is not empty                                          # while there are still nodes that have not been checked, continually run the algorithm

        

        f, current = heapq.heappop(self.opened) # this is the most promising node of all nodes in the open set

        if current = goal                                               # if the best possible path found leads to the goal, it is the best possible path that the robot could discover
            return reconstruct_path(came_from, goal)

        #remove current from openset                  # mark this node as having been evaluated
        #add current to closedset
        
        closedset.add(current) 

        for each neighbor in neighbor_nodes(current) # re-evaluate each neighboring node
            if neighbor in closedset
                continue
            tentative_g_score = g_score[current] + dist_between(current,neighbor) # create a new g_score for the current neighbor by adding the g_score from the current node and
			                                                                      # the distance to the neighbor

            if neighbor not in openset or tentative_g_score < g_score[neighbor]                 # if the neighbor has not been evaluated yet, or if a better path to the neighbor has been found,
				                                                                                # update the neighbor
                came_from[neighbor] = current                                                   # The node to reach this node from in the best time is the current node
                g_score[neighbor] = tentative_g_score                                           # The G score of the node is what we tentatively calculated earlier
                f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal) # The F score is the G score and the heuristic
                if neighbor not in openset                                                      # add this neighbor to the frontier if it was not in it already
                    add neighbor to openset

    return failure #if the program runs out of nodes to check before it finds the goal, then a solution does not exist

# Starting from the goal, work backwards to find the start.  We recommend returning a path nav_msgs, which is an array of PoseStamped with a header
def reconstruct_path(came_from,current):

	# start by adding goal to the path
    total_path = [current]

	# run while reconstruct_path hasn't reached the start
    while current in came_from:

		# The current node is now the node that leads to the previous node
        current = came_from[current]

		# add the current node to the front of the list
        total_path.append(current)

	# The list is now the shortest path from the start to the end
    return total_path

def heuristic_cost_estimate(start, goal):

	return kDistance*distance_calculation(start, goal) + kTurn*(angle_pose_to_path(startpose, goalpose) + angle_path_to_pose(startpose, goalpose))
    #if there were no obstacles in the way of the robot, what is the shortest path to the goal?  Return that value



def distance_calculation(startpose, goalpose):
    startx = startpose.pose.pose.position.x
    starty = startpose.pose.pose.position.y
    return math.sqrt(startx**2 + starty**2)

def angle_pose_to_path(startpose, goalpose):
    #Calculates the angle from a start pose, to the path between the start pose and the goal pose
    startQuat = startpose.pose.pose.orientation
    (rollStart, pitchStart, yawStart) = euler_from_quaternion(startQuat)
    startx = startpose.pose.pose.position.x
    starty = startpose.pose.pose.position.y
    endx = goalpose.pose.position.x
    endy = goalpose.pose.position.y
    deltax = endx-startx
    deltay = endy-starty

    travelAngle = math.atan2(deltay, deltax)

    return travelAngle-yawStart

def angle_path_to_pose(startpose, goalpose):
    #Calculates the angle from a path of travel between two poses, and the desired angle at the end pose
    endQuat = goalpose.pose.orientation
    (rollEnd, pitchEnd, yawEnd) = euler_from_quaternion(endQuat)
    startx = startpose.pose.pose.position.x
    starty = startpose.pose.pose.position.y
    endx = goalpose.pose.position.x
    endy = goalpose.pose.position.y
    deltax = endx-startx
    deltay = endy-starty

    travelAngle = math.atan2(deltay, deltax)

    return travelAngle-yawEnd


def neighbor_nodes(currentNode):
	adjacent = []
	for i in range(-1, 2):
		for j in range (-1, 2):
			if not getWall(currentNode.x+i, currentNode.y+j):
				adjacent.append(node(currentNode.x+i, currentNode.y+j, False, gValueFunction(currentNode, i, j), heuristic_cost_estimate(currentNode.pose, newPose (currentNode, currentNode.x+i, currentNode.y+j)), currentNode)
	return adjacent 
	
def dist_between(current,neighbor):
	return math.sqrt((neighbor.pose.position.x - current.pose.position.x)**2 + (neighbor.pose.position.y - current.pose.position.y)**2)
    #TODO the distance necessary to travel to the neighbor from the current node
	
"""

some advice:

	A) don't be like me and make this monstrosity

"""
