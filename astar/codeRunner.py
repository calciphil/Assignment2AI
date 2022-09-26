# -*- coding: utf-8 -*-
"""
Created on Sat Sep 17 14:14:33 2022

@author: Philip
"""

from Map import Map_Obj
from queue import PriorityQueue
from typing import TypeVar, Tuple
import math
import numpy as np
import heapq as hq
import sys




#print("current = "+ str(themap.current_pos))
#themapsub = np.array(themap)
#print(themapsub[12,12])
# themap.show_map()
#print("start = " + str(themap.start_pos))
def manhattan(a, b):
    return sum(abs(val1-val2) for val1, val2 in zip(a,b))

def manhattan_distance(point1, point2):
    distance = 0
    for x1, x2 in zip(point1, point2):
        difference = x2 - x1
        absolute_difference = abs(difference)
        distance += absolute_difference
    return distance

def neighbors(pos : Map_Obj, current : list()) :
    values = pos.get_maps()[0] #aims for values of the map a.k.a. 1, -1 to find walls and paths
    validneighbors = []
    # current = pos.current_pos
    print("Neighbors of : "+str(current))
    left=current[0]
    right=current[1]
    # print("left: "+ str(left))
    # print("right: "+str(right))
    south = current[0]+1
    north = current[0]-1
    est = current[1]+1  
    ouest = current[1]-1
    # print(type(south))
    # print("South = "+str(south)+" North = "+str(north)+ "Est = "+str(est)+" Ouest = "+str(ouest))
    if values[south, right]!=-1: #not aiming on value
        validneighbors.append([south,right])
    if values[north, current[1]]!=-1:
        validneighbors.append([north,right])
    if values[current[0], est]!=-1:
        validneighbors.append([left, est])
    if values[current[0], ouest]!=-1:
        validneighbors.append([left, ouest])
    return validneighbors

def a_star_algo(themap : Map_Obj):
    goal = themap.get_end_goal_pos()
    print("Goal = "+str(goal))
    # print(goal)
    start = themap.get_start_pos()
    openList = []
    # openList.append((2, [12,12]))
    # print(openList)
    # hq.heapify(openList)
    # print(openList)
    # hq.heapify(openList)
    # print(openList)
    # print("1")
    closedList = []
    openList.append((0, start))
    g = 0
    h = manhattan(start, goal)
    print("OpenList @ start = " + str(openList))
    # openList.append((3, [30,12]))
    #print(openList)
    # print("h="+str(h))
    while len(openList)>0 :
        current = openList[0][1:][0] #Priority is unnecessary info for current
        print("Current : " + str(current))
        if current == goal:
            print("You reached your goal")
            break
        closedList.append(current)
        openList.pop(0)
        print("Openlist after pop element 0 = " + str(openList))
        childs = neighbors(themap, current) #Prints neighbor of + current cell ---> Always on [45,18] how to modify 
        for child in childs:
            print("All childs of current = "+ str(childs))
            print("child = "+ str(child))
            if child in closedList:
                continue;
            #g = manhattan(child, current)
            h = manhattan(child, goal)
            f = g+h
            for link in openList:
                if child == link and g > manhattan(link, goal):
                    continue;
            openList.append((h+1, child))
            print("OpenList with child : " + str(openList))
            hq.heapify(openList)
            print("Closed list running = " +str(closedList) +"\n")
    print("Closed List @ end = " + str(closedList))
    

themap = Map_Obj(task=1)
the = themap.get_maps()
        


print("A* tests")
a_star_algo(themap)
    
        
'''
def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far
'''