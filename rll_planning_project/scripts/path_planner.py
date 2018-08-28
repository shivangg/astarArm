#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
from rll_planning_project.srv import *
from rll_planning_project.msg import *
from geometry_msgs.msg import Pose2D
from heapq import heappush, heappop # for priority queue
import math
import time

# import astar


# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop

Infinite = float('inf')


class AStar:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a subclass. The second parameter is always the goal."""
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1, n2):
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        raise NotImplementedError

    @abstractmethod
    def neighbors(self, node):
        """For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedError

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from
        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reversePath=False):
        if self.is_goal_reached(start, goal):
            return [start]
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))
        openSet = []
        heappush(openSet, startNode)
        while openSet:
            current = heappop(openSet)
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reversePath)
            current.out_openset = True
            current.closed = True
            for neighbor in [searchNodes[n] for n in self.neighbors(current.data)]:
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                    self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                    self.heuristic_cost_estimate(neighbor.data, goal)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor)
        return None


def find_path(start, goal, neighbors_fnct, reversePath=False, heuristic_cost_estimate_fnct=lambda a, b: Infinite, distance_between_fnct=lambda a, b: 1.0, is_goal_reached_fnct=lambda a, b: a == b):
    """A non-class version of the path finding algorithm"""
    class FindPath(AStar):

        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            print("Core algorithm :::: current" + str(current))
            print("Core algorithm :::: goal" + str(goal))

            currentArr = NameToArr(current)
            goalArr = NameToArr(goal)
            # goalTheta = NameToArr(goalArr)[2]   

            nearGoal = ((arrToPose(currentArr, currentArr[2]).x - pose_goal.x) ** 2 + (arrToPose(currentArr, currentArr[2]).y - pose_goal.y) ** 2) < 0.0016

            if nearGoal and abs(currentArr[2] - goalArr[2]) < 0.17 :
                # if isLegal(currentArr[0:2], currentArr[2], currentArr[0:2], goalArr[2] ):
                #     print("Move to goal theta. Legal! from current: " + str(currentArr))
                #     return True
                # elif isLegal(currentArr[0:2], currentArr[2], goalArr[0:2], goalArr[2] ):
                #     print("Move to goal theta. Legal! from current: " + str(currentArr))
                #     return True
                print("current - goal pose")
                print((arrToPose(currentArr, currentArr[2]).x - pose_goal.x) ** 2 + (arrToPose(currentArr, currentArr[2]).y - pose_goal.y) ** 2)
                return True

            return False
            # return is_goal_reached_fnct(current, goal)
    return FindPath().astar(start, goal, reversePath)

__all__ = ['AStar', 'find_path']



# decides 1 step length
# 0.025 is 2.5 in cms
resolution = 0.025
map_width = 0.0
map_length = 0.0

maze = []

# stores arr index in name form x1y1tn
# n belongs to [0, 0.785 ,1.57]
startArr = ''
goalArr = ''
currentArrAstar = ''

all_nodes = []

rows = 0
cols = 0

pose_goal = ''


move_srv = rospy.ServiceProxy('move', Move)
check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)

def poseToArr(pose):
    # print("using length x width of: " + str(map_length) + " " + str(map_width) )
    arrX = int(( pose.x + map_width/2) / resolution)
    arrY = int(( pose.y + map_length/2) / resolution)

    return [arrX, arrY]

def arrToPose(arr, pose_theta=0.0):
    poseX = arr[0] * resolution - map_width / 2
    poseY = arr[1] * resolution - map_length / 2

    return Pose2D(poseX, poseY, pose_theta)


def isLegal(currentArr, currentTheta, nextArr, nextTheta):
    if not ( currentArr == nextArr and abs(currentTheta - nextTheta) < 0.35):
        resp = check_srv(arrToPose(currentArr, currentTheta), arrToPose(nextArr, nextTheta))
    # print("from " + str(currentArr) + " " + str(currentTheta) + " to " + str(nextArr) + " " +  str(nextTheta) + " : " + str(resp))
        return resp.valid
    return False

def orthoStart(startArr):
    # why check for 4 when we need just the closest
    print("Recieved startArr: " + str(startArr))
    startArr = NameToArr(startArr)
    base = 1.57
    # angles = [round((1.57 * x) % 6.28, 3 )  for x in range(4)]
    # # print("At the Start Arr" + str(currentArr))
    
    # for x in angles:
    #     if isLegal(startArr[0:2], startArr[2], startArr, x):
    #         print("ortho Theta: " + str(ArrToName(startArr[0], startArr[1], x)))
    #         return( ArrToName(startArr[0], startArr[1], x) )
    print("ortho Theta: " + str(ArrToName(startArr[0], startArr[1], (base * round(float(startArr[2])/base)))))
    return(ArrToName(startArr[0], startArr[1], (base * round(float(startArr[2])/base))))


def legalTheta(currentArr, currentTheta, nextArr, add_custom = False):
    # returns current + pi/4 * (2n+1)
    angles = [round((currentTheta + 0.785 * (2*x+1)) % 6.28, 3 )  for x in range(4)]
    
    goalTheta = NameToArr(goalArr)[2]

    # angles.extend([round(( goalTheta + 0.785 * (2*x+1)) % 6.28, 3 )  for x in range(4)])
    # angles = [round((0.785 * (2*x+1)) % 6.28, 3 )  for x in range(4)]
    
    # print(angles)
    valid_angles = []
    if add_custom:
        print("####################NEAR GOAL: Theta diff:")
        print(str(currentTheta) + " " + str(goalTheta))
        if isLegal(currentArr[0:2], currentTheta, nextArr, goalTheta ):
            print("Move to goal theta. Legal! from current: " + str(currentArr))
            print("Goal is : " + str(NameToArr(goalArr)))
            valid_angles.append( goalTheta)
        else:
            print("ILLLegal! from current: " + str(currentArr) + " to " + str(goalTheta))
            print("Goal is : " + str(NameToArr(goalArr)))
        # elif isLegal(currentArr[0:2], currentTheta, nextArr, -6.28 - goalTheta ):
        #     print("Near goal")
        #     valid_angles.append(-6.28 - NameToArr(goalArr)[2])
    else:
        for x in angles:
            if isLegal(currentArr[0:2], currentTheta, nextArr, x):
                valid_angles.append(x)

    return valid_angles


def moveTo(arr, goalTheta):
    goalPose = arrToPose(arr, goalTheta)
    print("goal Pose")
    print(goalPose)
    resp = move_srv(goalPose)

    return resp


def NameToArr(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1].split('t')[0]), float(name.split('x')[1].split('y')[1].split('t')[1])]

def ArrToName(arrX, arrY, arrT):
    # print("Arr to State Name ")
    # print("x" + str(arrX) + "y" + str(arrY) + "t" + str(arrT))
    return "x" + str(arrX) + "y" + str(arrY) + "t" + str(arrT)

def optimizePath(traj_path):
    # assumes no diagonal moves in the path
    optmized_path = []
    # will be 'x' or 'y'
    optimize_in = ''

    # add first element without doubt
    print("\t\t\t\t\t" + traj_path[0])
    optmized_path.append(traj_path[0])

    # starting optimization direction
    if NameToArr(traj_path[0])[0] is NameToArr(traj_path[1])[0]:
        print("optimize in x")
        optimize_in = 'x'

    if NameToArr(traj_path[0])[1] is NameToArr(traj_path[1])[1]:
        print("optimize in y")
        optimize_in = 'y'

    # 
    for x in range(1, len(traj_path) - 1):
        currentOptimized = NameToArr(traj_path[x])
        nextOptimized = NameToArr(traj_path[x+1])
        print(str(currentOptimized) + str(nextOptimized))
        # print("nextOptimized: " + str(nextOptimized))

        # if thetas not equal, append to path
        if(currentOptimized[2] != nextOptimized[2]):
            print("thetas not equal")
            print("\t\t\t\t\t" + traj_path[x])
            
            optmized_path.append(traj_path[x])
            optmized_path.append(traj_path[x+1])
            if x < len(traj_path) - 2:
                x = x + 1
            
            if NameToArr(traj_path[x])[0] is NameToArr(traj_path[x+1])[0]:
                    print("optimize in x")
                    optimize_in = 'x'

            if NameToArr(traj_path[x])[1] is NameToArr(traj_path[x+1])[1]:
                print("optimize in y")
                optimize_in = 'y'  

        else:
            if currentOptimized[0] is nextOptimized[0] and optimize_in is 'y':
                print("optimize in x")
                optimize_in = 'x'
                print("\t\t\t\t\t" + traj_path[x])
                optmized_path.append(traj_path[x])
            if currentOptimized[1] is nextOptimized[1] and optimize_in is 'x':
                print("optimize in y")
                optimize_in = 'y'
                print("\t\t\t\t\t" + traj_path[x])
                optmized_path.append(traj_path[x])

    # add last element without doubt
    print("\t\t\t\t\t" + traj_path[-1])
    optmized_path.append(traj_path[-1])

    print("Optimized path length: " + str(len(optmized_path)))
    print("Optimized path : " + str(optmized_path))

    return optmized_path


def traj_move(traj_path):
    # a function to accept arr indices, as state names

    if len(traj_path) < 1:
        print("traj_path length less than 2!")
        return

    traj_path = optimizePath(traj_path)

    for x in range(len(traj_path) - 1):
        currentArr = NameToArr(traj_path[x])
        nextArr = NameToArr(traj_path[x+1])
        
        # resp = isLegal(currentArr[0:2], currentArr[2], nextArr[0:2], nextArr[2]) 
        # if resp:
        resp = moveTo(nextArr[0:2], nextArr[2])
        if resp:
            print("moved to " + str(nextArr) )
            time.sleep(1)
        else:
            print("Could not move to " + str(nextArr))


def plan_to_goal(req):
    """ Plan a path from Start to Goal """
    pose_start = Pose2D()
    global pose_goal
    pose_goal = Pose2D()
    # pose_check_start = Pose2D()
    # pose_check_goal = Pose2D()
    pose_move = Pose2D()

    rospy.loginfo("Got a planning request")

    pose_start = req.start
    pose_goal = req.goal

    # to make global changes to map diementions vars for use in 
    # other functions
    global map_width
    global map_length

    map_width = rospy.get_param('~map_width')
    map_length = rospy.get_param('~map_length')

    # defining a maze with a resolution. 
    # More resolution, more precision, more slow.
    # resolution = 0.1
    global rows
    global cols
    cols = int(map_length/resolution)
    rows = int(map_width/resolution)
    global maze
    maze = [[0 for x in range(cols + 1)] for y in range(rows + 1)]

    global all_nodes
    for x in range(rows):
        for y in range(cols):
            all_nodes.append([x, y])


    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    # Input: map dimensions, start pose, and goal pose
    # retrieving input values  

    xStart, yStart, tStart = pose_start.x, pose_start.y, float('%.3g' % pose_start.theta)
    xGoal, yGoal, tGoal = pose_goal.x, pose_goal.y, float('%.3g' %  pose_goal.theta)
    
    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f", xStart, yStart, tStart)
    rospy.loginfo("goal pose: x %f, y %f, theta %f", xGoal, yGoal, tGoal)

    rospy.loginfo("Starting testing")

    startXArr, startYArr = poseToArr(pose_start)
    goalXArr, goalYArr = poseToArr(pose_goal)
    
    global startArr
    global goalArr
    
    startArr = ArrToName(startXArr, startYArr, tStart)

    path_with_ortho_start = []
    path_with_ortho_start.append(startArr)
    
    startArr = orthoStart(startArr)
    
    goalArr =  ArrToName(goalXArr, goalYArr, 6.28 + (tGoal % 6.28))

    print("Goal Arr indices: " + goalArr) 
    print("Start Arr indices: " + startArr)
    
    maze[startXArr][startYArr] = "S"
    maze[goalXArr][goalYArr] = "E"

    print("Initilized Maze")
    for x in maze:
        for y in x:
            print(y, end=' ')
        print()

    xCurrentArr, yCurrentArr, tCurrent = startXArr, startYArr, tStart
    global currentArrAstar
    currentArrAstar = ArrToName(xCurrentArr, yCurrentArr, tStart)
    
    # tester()
    # time.sleep(50)

    print("startArr" + startArr)
    print("goalArr" + goalArr)

    start_time = time.time()
    
    solved_path = list(find_path(startArr, goalArr , neighbors_fnct=neighbors,
                heuristic_cost_estimate_fnct=cost, distance_between_fnct=distance))

    end_time = time.time()
    print("total time taken", str(end_time - start_time))
    path_with_ortho_start.extend(solved_path)
    print(path_with_ortho_start)

    print("path length :" + str(len(path_with_ortho_start)))

    # path_with_ortho_start = ['x9y12t0.2', 'x9y12t0.0', 'x9y13t0.0', 'x9y14t0.0', 'x9y15t0.0', 'x10y15t0.0', 'x10y15t0.785', 'x10y15t1.57', 'x11y15t1.57', 'x12y15t1.57', 'x13y15t1.57', 'x14y15t1.57', 'x15y15t1.57', 'x16y15t1.57', 'x17y15t1.57', 'x18y15t1.57', 'x19y15t1.57', 'x20y15t1.57', 'x21y15t1.57', 'x22y15t1.57', 'x23y15t1.57', 'x24y15t1.57', 'x25y15t1.57', 'x26y15t1.57', 'x27y15t1.57', 'x28y15t1.57', 'x29y15t1.57', 'x30y15t1.57', 'x31y15t1.57', 'x32y15t1.57', 'x33y15t1.57', 'x34y15t1.57', 'x34y16t0.785', 'x33y16t0.0', 'x33y17t0.0', 'x33y18t0.0', 'x33y19t0.0', 'x33y20t0.0', 'x33y21t0.0', 'x33y22t0.0', 'x33y22t3.925', 'x33y22t6.28', 'x33y23t6.28', 'x33y24t6.28', 'x33y25t6.28', 'x33y26t6.28', 'x33y27t6.28', 'x33y28t6.28', 'x33y29t6.28', 'x33y30t6.28', 'x33y31t6.28', 'x33y32t6.28', 'x33y33t6.28', 'x33y34t6.28', 'x33y35t6.28', 'x33y36t6.28', 'x33y37t6.28', 'x33y38t6.28', 'x33y39t6.28', 'x33y40t6.28', 'x33y41t6.28', 'x33y42t6.28', 'x33y43t6.28', 'x33y44t6.28', 'x33y45t6.28', 'x33y46t6.28', 'x33y47t6.28', 'x33y48t6.28', 'x33y49t0.785', 'x32y49t0.785', 'x31y49t1.57', 'x30y49t1.57', 'x29y49t1.57', 'x28y49t1.57', 'x27y49t1.57', 'x26y49t1.57', 'x25y49t1.57', 'x24y49t1.57', 'x23y49t1.57', 'x22y49t1.57', 'x21y49t1.57', 'x20y49t1.57', 'x19y49t1.57', 'x18y49t1.57', 'x18y50t0.785', 'x18y51t0.0', 'x18y52t0.0', 'x18y53t0.0', 'x18y54t0.0', 'x17y54t0.0', 'x17y54t0.785', 'x17y55t1.57', 'x18y55t1.57', 'x19y55t1.57', 'x20y55t1.57', 'x21y55t1.57', 'x22y55t1.57', 'x23y55t1.57', 'x24y55t1.57', 'x25y55t1.57', 'x26y55t1.57', 'x27y55t1.57', 'x28y55t1.57', 'x29y55t1.57', 'x30y55t1.57', 'x31y55t1.57', 'x31y56t0.785', 'x31y57t0.0', 'x31y57t6.4']

    traj_move(path_with_ortho_start)
    

# def tester():
#     cols = int(map_length/resolution)
#     rows = int(map_width/resolution)
    
#     start_time = time.time()

#     for i in xrange(1,rows -1 ):
#         for j in xrange(1,cols -1 ):
#             currentArr = [i, j]
#             # nextArr = [i, j+1]
#             # isLegal(currentArr, 0, nextArr, 0)
#             checkAndMarkObstacles(currentArr, 0)
#     end_time = time.time()
#     print("total time taken")
#     print(end_time - start_time)

class PathPlanner:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("plan_to_goal", PlanToGoalAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        plan_to_goal(req)
        self.server.set_succeeded()

def printMaze(currentArrAstar):
    if maze[currentArrAstar[0]][currentArrAstar[1]] != 'S' and maze[currentArrAstar[0]][currentArrAstar[1]] != 'E' :
        maze[currentArrAstar[0]][currentArrAstar[1]] = "M"
    
    # numbering the array for display
    k = 0
    for y in maze[0]:
        print(k, end=' ')
        k = k + 1
    print()
    # printing for debuging expansion of nodes
    k = 0
    for x in maze:
        print(k, end = " ")
        k = k + 1
        for y in x:
            print(y, end=' ')
        print()

    if maze[currentArrAstar[0]][currentArrAstar[1]] != 'S' and maze[currentArrAstar[0]][currentArrAstar[1]] != 'E' :
        maze[currentArrAstar[0]][currentArrAstar[1]] = "*"
    # printing end

def nearGoal(currentArrAstar):
    goalInArr = NameToArr(goalArr)
    near_goal = (abs(currentArrAstar[0] - goalInArr[0]) < 2 and currentArrAstar[1] == goalInArr[1]  ) or (abs(currentArrAstar[1] - goalInArr[1]) < 2 and currentArrAstar[0] == goalInArr[0])
    return near_goal

def nearCurrent(currentArrAstar):
    currentInArr = NameToArr(startArr)
    near_current = (abs(currentArrAstar[0] - currentInArr[0]) < 1) and abs((currentArrAstar[1] - currentInArr[1]) < 1)
    return near_current

def neighbors(currentArrAstar):
# return the 4 neighbors from 2D array
    # print("received in name state. Converting to arr")
    currentArrAstar = NameToArr(currentArrAstar)
    # print("Converted to arr.")

    print("neighbors of : " + str(currentArrAstar))
    # goalInArr = NameToArr(goalArr)        
    # if currentArrAstar[0:2] == goalInArr[0:2]:
    #     print("MATCHING ARR, trying to equalize theta")
    #     if isLegal(currentArrAstar[0:2], currentArrAstar[2], goalInArr[0:2], goalInArr[2]):
    #         print("true to goal")
    #         yield( ArrToName(currentArrAstar[0], currentArrAstar[1], goalInArr[2]) )
    #     else:
    #         print("Can not reach goaltheta from current theta")
    #     time.sleep(10)

    printMaze(currentArrAstar)
    
    # valid_angles = [0]
    print("neighbors of : " + str(currentArrAstar))
    k = 1
    dirs = [[0, 0], [k, 0], [0, k], [-k, 0], [0, -k]]
    for dir in dirs:
        neighbor = [currentArrAstar[0] + dir[0], currentArrAstar[1] + dir[1]]
        if (neighbor[0] <= rows and neighbor[1] <= cols) or (neighbor[0] >= 0 and neighbor[1] >= 0):
        # if neighbor in all_nodes:
            if isLegal(currentArrAstar[0:2], currentArrAstar[2], neighbor, currentArrAstar[2]) and dir != [0, 0]:
                print( ArrToName(neighbor[0], neighbor[1], currentArrAstar[2]) )
                yield( ArrToName(neighbor[0], neighbor[1], currentArrAstar[2]) )
            else:
                print("cannot move with currentTheta: " + str(currentArrAstar[2]))
                valid_angles = legalTheta(currentArrAstar, currentArrAstar[2], neighbor, nearGoal(currentArrAstar))
                print("finding legal thetas to: " + str(neighbor))
                if len(valid_angles) > 0:
                    for x in valid_angles:
                        # print( ArrToName(neighbor[0], neighbor[1], x) )
                        # moveTo(neighbor[0:2], x)
                        print( ArrToName(neighbor[0], neighbor[1], x) )
                        yield( ArrToName(neighbor[0], neighbor[1], x) )
                else:
                    print("NO legal thetas available to " + str(neighbor))





def distance(startArr, currentArrAstar):
    # path length travelled
    startArrX, startArrY, startArrT = NameToArr(startArr)
    currentArrAstarX, currentArrAstarY, currentArrAstarT = NameToArr(currentArrAstar)

    coord_diff = abs(currentArrAstarX - startArrX) + abs(currentArrAstarY - startArrY)
    
    # how to calcultate the cost for the theta.
    # depends on current and previous 
    return abs(startArrX - currentArrAstarX) + abs(startArrY - currentArrAstarY) + \
    abs((currentArrAstarT % (2 * 3.14) - startArrT % (2 * 3.14)) * coord_diff ** 4 )

def cost(currentArrAstar, goalArr):
    # heuristic cost
    goalArrX, goalArrY, goalArrT = NameToArr(goalArr)
    currentArrAstarX, currentArrAstarY, currentArrAstarT = NameToArr(currentArrAstar)

    coord_diff = abs(currentArrAstarX - goalArrX) + abs(currentArrAstarY - goalArrY)

    return abs(currentArrAstarX - goalArrX) + abs(currentArrAstarY - goalArrY) + \
    abs((currentArrAstarT % (2 * 3.14) - goalArrT % (2 * 3.14)) * coord_diff ** 3 )


if __name__ == '__main__':
    rospy.init_node('path_planner')

    server = PathPlanner()

    rospy.spin()


# for self neighbor with diff thetas
# valid_angles = legalTheta(currentArrAstar, currentArrAstar[2], currentArrAstar)
#     if len(valid_angles) > 0:
#         for x in valid_angles:
#             print( ArrToName(currentArrAstar[0], currentArrAstar[1], x) )
#             # moveTo(neighbor[0:2], x)
#             yield( ArrToName(currentArrAstar[0], currentArrAstar[1], x) )





