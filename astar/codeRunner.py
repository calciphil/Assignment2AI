# -*- coding: utf-8 -*-
"""
Created on Sat Sep 17 14:14:33 2022

@author: Philip
"""

from Map import Map_Obj
import heapq
from typing import TypeVar, Tuple
import math


themap = Map_Obj(task=1)
# themap.show_map()
print(themap.goal_pos)
def manhattan(a, b):
    return sum(abs(val1-val2) for val1, val2 in zip(a,b))

print(manhattan(themap.get_start_pos(), themap.get_end_goal_pos()))

Location = TypeVar('Location')


heuristic = math.dist(themap.get_start_pos(),themap.get_goal_pos())


print(heuristic)

def neighbors(pos : Map_Obj()):
    validneighbors = []
    current = pos.current_pos
    south = current[0]+1
    north = current[0]-1
    est = current[1]+1  
    ouest = current[1]-1
    if current[south, current[1]]!=-1:
        validneighbors.append(current[south,current[1]])
    if current[north, current[1]]!=-1:
        validneighbors.append(current[north,current[1]])
    if current[current[0], est]!=-1:
        validneighbors.append(current[current[0], est])
    if current[current[0], ouest]!=-1:
        validneighbors.append(current[current[0], ouest])
    return validneighbors
    

def validate_cell(pos):
    if pos[0] == -1 or pos[1] == -1:
        return False
        return True
    
'''
def neighbors(current):
    neighbors = [neighbor for neighbor in neighbors if validate_cell(neighbor)]
    return neighbors
'''

def a_star_algo(Map_Obj: themap):
    goal = themap.get_end_goal_pos()
    openList = [themap.get_start_pos()]
    closedList = []
    g = 0
    h = manhattan(themap.get_start_pos(), goal)
    while not openList.empty() :
        current = openList[0]
        if current == goal:
            break
        closedList.append(openList[0])
        openList.pop(0)
        neighborhood = neighbors(themap.current_pos)
        print(neighborhood)
        
        
        
        
    
    
        
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